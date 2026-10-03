import copy
import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from conevision_sim.car.camera_geometry import car_mocap_pose
from conevision_sim.car.car_model import CAR_BODY, add_car
from conevision_sim.car.driver import CenterlineDriver
from conevision_sim.config import load_all
from conevision_sim.track.track_types import Track

CFG = load_all()


def _car_cfg(**driver):
    c = copy.deepcopy(CFG["car"])
    c["driver"].update(driver)
    return c


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


@pytest.fixture(scope="module")
def track():
    return Track.make_test_oval()


@pytest.mark.parametrize("drv", [{}, {"max_lateral_accel_mps2": 6.0, "speed_mps": 14.0},
                                 {"lateral_offset_m": 0.5, "start_s_m": 20.0}])
def test_continuity_and_wrap(track, drv):
    d = CenterlineDriver(track, _car_cfg(**drv))
    dt = 1 / 60
    ts = np.arange(0, 2.2 * d.lap_time, dt)          # includes the lap wrap twice
    st = [d.state_at(t) for t in ts]
    xy = np.array([[s.x, s.y] for s in st])
    yaw = np.array([s.yaw for s in st])
    v = np.array([s.v for s in st])
    step = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    assert step.max() <= v.max() * dt * 1.05 + 1e-6, "position jump"
    dyaw = np.abs(_wrap(np.diff(yaw)))
    assert dyaw.max() < 0.05, f"yaw jump {dyaw.max()}"
    # yaw rate consistent with finite-difference yaw
    yr = np.array([s.yaw_rate for s in st])
    assert np.abs(yr[:-1] - _wrap(np.diff(yaw)) / dt).max() < 0.15
    # pure function of t
    assert d.state_at(3.3) == d.state_at(3.3)
    assert d.state_at(1.0).x == pytest.approx(d.state_at(1.0 + d.lap_time).x, abs=1e-6)


def test_stays_in_track(track):
    d = CenterlineDriver(track, _car_cfg(lateral_offset_m=0.4))
    c = track.centerline
    half = 3.5 / 2
    for t in np.arange(0, d.lap_time, 0.1):
        s = d.state_at(t)
        dist = np.min(np.linalg.norm(c - [s.x, s.y], axis=1))
        assert dist <= half, (t, dist)


def test_starts_at_start_pose(track):
    d = CenterlineDriver(track, _car_cfg())
    s = d.state_at(0.0)
    assert math.hypot(s.x - track.start_pose[0], s.y - track.start_pose[1]) < 0.2


def test_speed_profile_slows_in_corners(track):
    d = CenterlineDriver(track, _car_cfg(speed_mps=15.0, max_lateral_accel_mps2=5.0))
    vs = [d.state_at(t).v for t in np.arange(0, d.lap_time, 0.1)]
    assert min(vs) < 9.0 and max(vs) <= 15.0 + 1e-6      # sqrt(5*12)=7.7 m/s in the arcs


def test_body_motion():
    c = _car_cfg()
    c["body_motion"].update(pitch_amplitude_deg=2.0, roll_amplitude_deg=1.0)
    d = CenterlineDriver(Track.make_test_oval(), c)
    ps = [d.state_at(t).pitch for t in np.linspace(0, 2, 200)]
    assert max(ps) == pytest.approx(math.radians(2.0), rel=0.02)


def test_scene_with_car_loads():
    mujoco = pytest.importorskip("mujoco")
    root = ET.Element("mujoco")
    asset = ET.SubElement(root, "asset")
    wb = ET.SubElement(root, "worldbody")
    ET.SubElement(wb, "geom", type="plane", size="50 50 .1")
    add_car(wb, asset, CFG["car"], CFG["camera"])
    m = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, CAR_BODY) >= 0
    assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, CFG["camera"]["name"]) >= 0
    assert m.nmocap == 1
    # all car geoms are visual-only
    car_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, CAR_BODY)
    for g in range(m.ngeom):
        if m.geom_bodyid[g] == car_id:
            assert m.geom_contype[g] == 0 and m.geom_conaffinity[g] == 0
    d = mujoco.MjData(m)
    p, q = car_mocap_pose(CenterlineDriver(Track.make_test_oval(), CFG["car"]).state_at(1.0))
    d.mocap_pos[0], d.mocap_quat[0] = p, q
    mujoco.mj_forward(m, d)


def test_camera_does_not_see_car():
    mujoco = pytest.importorskip("mujoco")
    cam = copy.deepcopy(CFG["camera"])
    cam.update(width=640, height=360, fx=520.0, fy=520.0, cx=320.0, cy=180.0)
    root = ET.Element("mujoco")
    vis = ET.SubElement(root, "visual")
    ET.SubElement(vis, "global", offwidth="640", offheight="360")
    ET.SubElement(root, "statistic", extent="1")
    asset = ET.SubElement(root, "asset")
    wb = ET.SubElement(root, "worldbody")
    ET.SubElement(wb, "geom", type="plane", size="50 50 .1")
    add_car(wb, asset, CFG["car"], cam)
    m = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    r = mujoco.Renderer(m, 360, 640)
    r.update_scene(d, camera=cam["name"])
    r.enable_segmentation_rendering()
    seg = r.render()
    r.close()
    car_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, CAR_BODY)
    geom_ids = seg[..., 1] == mujoco.mjtObj.mjOBJ_GEOM
    hit = np.zeros(seg.shape[:2], bool)
    hit[geom_ids] = m.geom_bodyid[seg[..., 0][geom_ids]] == car_id
    assert hit.sum() == 0, "the car body is visible in the default camera view"
