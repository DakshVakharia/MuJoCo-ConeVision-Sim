import numpy as np
import pytest

from fs_mono_cam_sim.render import bboxes as bb
from fs_mono_cam_sim.render import rosmsgs
from fs_mono_cam_sim.render.bboxes import Detection, compute_bboxes, pixel_stats


# ---------------------------------------------------------------- synthetic model/data
class FakeModel:
    """Duck-typed enough for compute_bboxes with mujoco calls monkeypatched below."""
    nbody = 6
    geom_bodyid = np.array([0, 1, 2, 3, 4, 5])


class FakeData:
    def __init__(self):
        self.cam_xpos = np.array([[0.0, 0.0, 1.0]])
        self.xpos = np.zeros((6, 3))
        self.xpos[1] = [10, 0, 0]   # cone_blue_0
        self.xpos[2] = [30, 0, 0]   # cone_yellow_0
        self.xpos[3] = [80, 0, 0]   # cone_orange_0 (far)
        self.xpos[4] = [5, 0, 0]    # a non-cone body


@pytest.fixture
def synth(monkeypatch):
    names = {0: "world", 1: "cone_blue_0", 2: "cone_yellow_0", 3: "cone_orange_0", 4: "tree_1", 5: "cone_orange_big_2"}
    monkeypatch.setattr(bb.mujoco, "mj_id2name", lambda m, t, i: names[i])
    monkeypatch.setattr(bb.mujoco, "mj_name2id", lambda m, t, n: 0)
    bb._CACHE.clear()
    return FakeModel(), FakeData()


CAM = {"name": "cam"}
BASE = {"min_visible_pixels": 1, "min_box_height_px": 0, "max_range_m": 1e9,
        "center_jitter_px": 0, "size_jitter_frac": 0, "dropout_prob": 0}


def blank(h=100, w=200):
    return np.full((h, w), -1, np.int32)


def test_exact_box_and_classes(synth):
    m, d = synth
    seg = blank()
    seg[20:41, 10:31] = 1          # blue 0: rows 20..40, cols 10..30
    seg[50:60, 100:105] = 2        # yellow
    seg[5:9, 150:152] = 4          # tree: ignored
    seg[70:80, 50:60] = 5          # orange_big index 2
    dets = compute_bboxes(m, d, seg, CAM, BASE, np.random.default_rng(0))
    by = {x.body: x for x in dets}
    assert set(by) == {"cone_blue_0", "cone_yellow_0", "cone_orange_big_2"}
    b = by["cone_blue_0"]
    assert (b.x_min, b.y_min, b.x_max, b.y_max) == (10, 20, 31, 41)
    assert b.cls == "blue" and b.index == 0 and b.visible_pixels == 21 * 21
    assert b.range_m == pytest.approx(np.hypot(10, 1))
    assert by["cone_orange_big_2"].cls == "orange_big" and by["cone_orange_big_2"].index == 2
    assert [x.range_m for x in dets] == sorted(x.range_m for x in dets)


def test_occlusion_shrinks_box_and_border_truncation(synth):
    m, d = synth
    seg = blank()
    seg[20:41, 10:31] = 1
    full = compute_bboxes(m, d, seg, CAM, BASE, np.random.default_rng(0))[0]
    seg[20:41, 20:31] = 4          # something in front covers the right half
    occ = compute_bboxes(m, d, seg, CAM, BASE, np.random.default_rng(0))[0]
    assert occ.x_max - occ.x_min < full.x_max - full.x_min
    assert occ.visible_pixels < full.visible_pixels
    seg = blank()
    seg[90:100, 190:200] = 2       # cone touching image corner
    c = compute_bboxes(m, d, seg, CAM, BASE, np.random.default_rng(0))[0]
    assert c.x_max == 200 and c.y_max == 100


def test_fully_occluded_gives_nothing(synth):
    m, d = synth
    assert compute_bboxes(m, d, blank(), CAM, BASE, np.random.default_rng(0)) == []


def test_filters(synth):
    m, d = synth
    seg = blank()
    seg[20:41, 10:31] = 1          # 441 px, 21 high, range ~10
    seg[50:54, 100:102] = 2        # 8 px, 4 high, range ~30
    seg[70:90, 50:60] = 3          # range 80
    rng = np.random.default_rng(0)
    assert len(compute_bboxes(m, d, seg, CAM, BASE, rng)) == 3
    assert {x.cls for x in compute_bboxes(m, d, seg, CAM, {**BASE, "min_visible_pixels": 10}, rng)} == {"blue", "orange"}
    assert {x.cls for x in compute_bboxes(m, d, seg, CAM, {**BASE, "min_box_height_px": 5}, rng)} == {"blue", "orange"}
    assert {x.cls for x in compute_bboxes(m, d, seg, CAM, {**BASE, "max_range_m": 50}, rng)} == {"blue", "yellow"}
    assert compute_bboxes(m, d, seg, CAM, {**BASE, "dropout_prob": 1.0}, rng) == []


def test_noise_seeded_and_bounded(synth):
    m, d = synth
    seg = blank()
    seg[40:61, 80:101] = 1
    cfg = {**BASE, "center_jitter_px": 3.0, "size_jitter_frac": 0.1}
    a = compute_bboxes(m, d, seg, CAM, cfg, np.random.default_rng(5))[0]
    b = compute_bboxes(m, d, seg, CAM, cfg, np.random.default_rng(5))[0]
    c = compute_bboxes(m, d, seg, CAM, cfg, np.random.default_rng(6))[0]
    assert a == b and a != c
    assert 0 <= a.x_min < a.x_max <= 200 and 0 <= a.y_min < a.y_max <= 100


def test_pixel_stats_speed():
    import time
    img = np.full((720, 1280), -1, np.int32)
    for i in range(30):
        img[300 + i:330 + i, 40 * i:40 * i + 25] = i % 5
    t = time.perf_counter()
    for _ in range(10):
        pixel_stats(img, 6)
    assert (time.perf_counter() - t) / 10 < 0.02


# ---------------------------------------------------------------- ROS message helpers (fake msgs)
class _Obj:
    def __init__(self, **kw):
        self.__dict__.update(kw)


class FakeHeader:
    stamp = None
    frame_id = ""


class FakeMarker:
    POLYGON = 4
    ADD = 0

    def __init__(self):
        self.header = FakeHeader()
        self.points = []


class FakeArray:
    def __init__(self):
        self.markers = []


class FakePoint:
    x = y = z = 0.0


class FakeColor:
    def __init__(self, r=0, g=0, b=0, a=0):
        self.r, self.g, self.b, self.a = r, g, b, a


def test_marker_array_matches_cone_base_detector_expectations():
    msgs = _Obj(ImageMarker=FakeMarker, ImageMarkerArray=FakeArray, Point=FakePoint, ColorRGBA=FakeColor)
    colors = {"blue": [0, 0, 1], "yellow": [1, 1, 0], "orange": [1, .2, 0], "orange_big": [1, .2, 0]}
    dets = [Detection("blue", 7, "cone_blue_7", 10.0, 20.0, 30.0, 60.0, 100, 12.0)]
    arr = rosmsgs.build_marker_array(msgs, dets, colors, "T", "front_cam_optical")
    (m,) = arr.markers
    assert m.header.stamp == "T" and m.header.frame_id == "front_cam_optical"
    assert m.type == 4 and m.ns == "blue" and m.id == 7 and m.scale == 2.0
    assert [(p.x, p.y, p.z) for p in m.points] == [(10, 20, 0), (30, 20, 0), (30, 60, 0), (10, 60, 0)]
    # detector logic: max y over points, mean x of the points at that y == bottom-centre
    ymax = max(p.y for p in m.points)
    bx = np.mean([p.x for p in m.points if p.y == ymax])
    assert (bx, ymax) == (20.0, 60.0)
    c = m.outline_color
    assert c.b > 0.5 and c.r < 0.3 and c.g < 0.3 and c.a == 1.0
    assert rosmsgs.build_marker_array(msgs, [], colors, "T", "f").markers == []


# ---------------------------------------------------------------- integration with the real scene
def _rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _project(sim, cam, world_pt):
    """Pinhole projection using camera.yaml intrinsics/extrinsics and the car body pose from MuJoCo."""
    import mujoco
    car = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, "car")
    Rc = sim.data.xmat[car].reshape(3, 3)
    pc = sim.data.xpos[car]
    p_base = Rc.T @ (np.asarray(world_pt) - pc)
    R_cam = _rot_z(np.radians(cam.get("yaw_deg", 0))) @ _rot_y(np.radians(cam.get("pitch_deg", 0)))
    p_cam = R_cam.T @ (p_base - np.array([cam["x"], cam["y"], cam["z"]]))   # x fwd, y left, z up
    X, Y, Z = -p_cam[1], -p_cam[2], p_cam[0]                                 # optical: right, down, fwd
    fx = cam.get("fx") or cam["width"] / 2 / np.tan(np.radians(cam["hfov_deg"]) / 2)
    fy = cam.get("fy") or fx
    return fx * X / Z + cam["cx"], fy * Y / Z + cam["cy"], Z


def test_integration_boxes_match_projected_cone_base():
    from fs_mono_cam_sim.config import load_all
    from fs_mono_cam_sim.render.sim import Simulation
    from fs_mono_cam_sim.track.track_types import Track

    cfg = load_all()
    cfg["camera"] = {**cfg["camera"], "width": 640, "height": 360, "fx": 550.0, "fy": 550.0,
                     "cx": 320.0, "cy": 180.0, "hfov_deg": None}
    cfg["perception"] = {**cfg["perception"], "bbox": {**cfg["perception"]["bbox"],
                         "min_visible_pixels": 15, "max_range_m": 40.0}}
    track = Track.make_test_oval()
    sim = Simulation(cfg, track=track)
    try:
        f = sim.frame_at_index(30)
        cones = {(c, i): (x, y) for c, i, x, y in track.all_cones()}
        assert f.detections, "expected visible cones on the oval"
        checked = 0
        for det in f.detections[:6]:
            if det.x_min <= 0 or det.x_max >= 640 or det.y_max >= 360:   # border-truncated
                continue
            cx, cy = cones[(det.cls, det.index)]
            u, v, z = _project(sim, cfg["camera"], [cx, cy, 0.0])
            assert z > 0
            assert abs((det.x_min + det.x_max) / 2 - u) < max(4.0, 0.2 * (det.x_max - det.x_min)), det
            # The box bottom is the near edge of the square base plate (axis-aligned), not the
            # cone centre, so compare against the lowest projected plate corner.
            size = cfg["scene"]["cones"]["big" if det.cls == "orange_big" else "small"]["base_width"]
            h = size / 2
            v_plate = max(_project(sim, cfg["camera"], [cx + dx, cy + dy, 0.0])[1]
                          for dx in (-h, h) for dy in (-h, h))
            assert abs(det.y_max - v_plate) < max(3.0, 0.06 * (det.y_max - det.y_min)), (det, v_plate)
            checked += 1
        assert checked >= 2
        assert f.t == pytest.approx(30 / sim.fps)
    finally:
        sim.close()
