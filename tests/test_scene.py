import copy
import math
import sys
from pathlib import Path

import mujoco
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from conevision_sim import config                                   # noqa: E402
from conevision_sim.scene import scenery as scn                      # noqa: E402
from conevision_sim.scene.builder import build_scene_xml, write_scene  # noqa: E402
from conevision_sim.track.track_types import Track                   # noqa: E402


@pytest.fixture(scope="module")
def cfg():
    return config.load_all()


@pytest.fixture(scope="module")
def track():
    return Track.make_test_oval()


def _scenery_xy(model):
    out = []
    for b in range(model.nbody):
        if model.body(b).name.startswith("scenery_"):
            out.append(model.body_pos[b][:2].copy())
    return np.array(out).reshape(-1, 2)


def test_xml_loads_and_has_cone_bodies(cfg, track):
    model = mujoco.MjModel.from_xml_string(build_scene_xml(track, cfg))
    names = [model.body(i).name for i in range(model.nbody)]
    for cls, i, x, y in track.all_cones():
        n = f"cone_{cls}_{i}"
        assert names.count(n) == 1, n
        b = model.body(n)
        assert np.allclose(b.pos[:2], [x, y], atol=1e-3)
        geoms = [g for g in range(model.ngeom) if model.geom_bodyid[g] == b.id]
        assert geoms, f"{n} has no geoms"
        assert all(model.geom_contype[g] == 0 and model.geom_conaffinity[g] == 0 for g in geoms)
    assert sum(n.startswith("cone_") for n in names) == len(track.all_cones())
    assert "car" in names
    cam = cfg["camera"]
    assert model.cam(cam["name"]).id >= 0
    assert model.vis.global_.offwidth >= cam["width"] and model.vis.global_.offheight >= cam["height"]
    assert model.vis.map.znear == pytest.approx(cam["near_clip"])
    assert model.vis.map.zfar == pytest.approx(cam["far_clip"])
    assert model.stat.extent == pytest.approx(1.0)


def test_write_scene_loads_from_own_folder(cfg, track, tmp_path):
    xml = write_scene(track, cfg, tmp_path / "out" / "scene.xml")
    model = mujoco.MjModel.from_xml_path(str(xml))
    assert model.nbody > len(track.all_cones())
    assert (tmp_path / "out" / "textures").is_dir()


def test_scenery_exclusion_zone(cfg, track):
    model = mujoco.MjModel.from_xml_string(build_scene_xml(track, cfg))
    xy = _scenery_xy(model)
    assert len(xy) > 10
    scfg = cfg["scene"]["scenery"]
    cones = np.array([[x, y] for _, _, x, y in track.all_cones()])
    d_cone = np.linalg.norm(xy[:, None, :] - cones[None], axis=2).min(1)
    assert d_cone.min() >= 2.0 - 1e-6
    dense = scn._dense_centerline(track.centerline, 0.25)
    d_center = np.linalg.norm(xy[:, None, :] - dense[None], axis=2).min(1)
    assert d_center.min() >= scfg["min_dist_from_track_m"] - 0.3        # centerline sampled at 0.5 m
    sx, sy, _ = track.start_pose
    assert np.hypot(xy[:, 0] - sx, xy[:, 1] - sy).min() >= 2.0


def test_scenery_non_overlapping_and_deterministic(cfg, track):
    catalog, _, _ = scn.load_catalog(cfg["scene"]["scenery"])
    a = scn.place_scenery(track, cfg["scene"]["scenery"], catalog)
    b = scn.place_scenery(track, cfg["scene"]["scenery"], catalog)
    assert [(p["name"], p["x"], p["y"]) for p in a] == [(p["name"], p["x"], p["y"]) for p in b]
    for i, p in enumerate(a):
        for q in a[i + 1:]:
            assert math.hypot(p["x"] - q["x"], p["y"] - q["y"]) >= p["fp"] + q["fp"] - 1e-6


def test_procedural_fallback_without_assets(cfg, track):
    c = copy.deepcopy(cfg)
    c["scene"]["scenery"]["use_assets"] = False
    model = mujoco.MjModel.from_xml_string(build_scene_xml(track, c))
    assert len(_scenery_xy(model)) > 10


def test_scenery_disabled(cfg, track):
    c = copy.deepcopy(cfg)
    c["scene"]["scenery"]["enabled"] = False
    model = mujoco.MjModel.from_xml_string(build_scene_xml(track, c))
    assert len(_scenery_xy(model)) == 0


def test_segmentation_maps_cone_pixels_to_cone_body(cfg, track):
    model = mujoco.MjModel.from_xml_string(build_scene_xml(track, cfg))
    data = mujoco.MjData(model)
    x, y, yaw = track.start_pose
    data.mocap_pos[0] = [x, y, 0]
    data.mocap_quat[0] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
    mujoco.mj_forward(model, data)
    try:
        r = mujoco.Renderer(model, 360, 640)
    except Exception as e:                      # no GL available
        pytest.skip(f"no OpenGL context: {e}")
    r.enable_segmentation_rendering()
    r.update_scene(data, camera=cfg["camera"]["name"])
    seg = r.render()
    r.close()
    gids = np.unique(seg[..., 0][seg[..., 1] == mujoco.mjtObj.mjOBJ_GEOM])
    cone_bodies = {model.body(model.geom_bodyid[g]).name for g in gids
                   if model.body(model.geom_bodyid[g]).name.startswith("cone_")}
    assert len(cone_bodies) >= 5
