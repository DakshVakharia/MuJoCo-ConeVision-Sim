"""Camera geometry: pure-numpy projection vs the real MuJoCo render, and the ground homography."""
import copy
import xml.etree.ElementTree as ET

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

from fs_mono_cam_sim.car.camera_geometry import (car_mocap_pose, ground_homography,
                                                 intrinsic_matrix, pixel_to_ground,
                                                 project_world_points, T_base_cam)
from fs_mono_cam_sim.car.car_model import add_car
from fs_mono_cam_sim.car.driver import CarState
from fs_mono_cam_sim.config import load_all

CFG = load_all()


def _cam(**over):
    c = copy.deepcopy(CFG["camera"])
    c.update(width=640, height=360, fx=520.0, fy=520.0, cx=320.0, cy=180.0)
    c.update(over)
    return c


def _scene(points, cam, car_cfg=None):
    root = ET.Element("mujoco")
    vis = ET.SubElement(root, "visual")
    ET.SubElement(vis, "global", offwidth=str(cam["width"]), offheight=str(cam["height"]))
    ET.SubElement(vis, "map", znear=str(cam["near_clip"]), zfar=str(cam["far_clip"]))
    ET.SubElement(root, "statistic", extent="1")
    asset = ET.SubElement(root, "asset")
    wb = ET.SubElement(root, "worldbody")
    ET.SubElement(wb, "geom", name="ground", type="plane", size="100 100 .1")
    for i, p in enumerate(points):
        ET.SubElement(wb, "geom", name=f"ball{i}", type="sphere", size="0.06",
                      pos=" ".join(map(str, p)), contype="0", conaffinity="0")
    add_car(wb, asset, car_cfg or CFG["car"], cam)
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))


def _render_centroids(model, state, cam, n):
    data = mujoco.MjData(model)
    pos, quat = car_mocap_pose(state)
    data.mocap_pos[0], data.mocap_quat[0] = pos, quat
    mujoco.mj_forward(model, data)
    r = mujoco.Renderer(model, cam["height"], cam["width"])
    r.update_scene(data, camera=cam["name"])
    r.enable_segmentation_rendering()
    seg = r.render()
    r.close()
    ids = seg[..., 0]
    out = []
    for i in range(n):
        gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"ball{i}")
        ys, xs = np.nonzero((ids == gid) & (seg[..., 1] == mujoco.mjtObj.mjOBJ_GEOM))
        out.append((xs.mean() + 0.5, ys.mean() + 0.5) if len(xs) >= 6 else (np.nan, np.nan))
    return np.array(out)


CASES = [
    ("level_centred", {}, CarState(0, 0, 0)),
    ("offcentre_fx_ne_fy", dict(fx=600.0, fy=480.0, cx=250.0, cy=140.0), CarState(0, 0, 0)),
    ("pose_rpy", dict(fx=560.0, fy=540.0, cx=350.0, cy=200.0, roll_deg=3.0, pitch_deg=7.0, yaw_deg=-5.0,
                      x=0.5, y=0.15, z=1.2),
     CarState(3.0, -2.0, 0.7, roll=0.03, pitch=0.02)),
]


@pytest.mark.parametrize("name,over,state", CASES, ids=[c[0] for c in CASES])
def test_projection_matches_render(name, over, state):
    cam = _cam(**over)
    # ground-truth targets: pick pixels, back-project onto assorted depths -> world points
    rng = np.random.default_rng(1)
    T = np.array([[np.cos(state.yaw), -np.sin(state.yaw)], [np.sin(state.yaw), np.cos(state.yaw)]])
    pts = []
    for fwd in (4.0, 7.0, 12.0, 20.0, 35.0):
        for lat in (-0.35, 0.0, 0.3):
            local = np.array([fwd, lat * fwd])
            xy = np.array([state.x, state.y]) + T @ local
            pts.append([xy[0], xy[1], rng.uniform(0.1, 0.6)])
    pts = np.array(pts)
    model = _scene(pts, cam)
    uv_ref = _render_centroids(model, state, cam, len(pts))
    uv, depth = project_world_points(pts, state, cam)
    vis = (depth > 0) & ~np.isnan(uv_ref[:, 0])
    assert vis.sum() >= 6, "too few visible test points"
    err = np.linalg.norm(uv[vis] - uv_ref[vis], axis=1)
    print(f"{name}: n={vis.sum()} max err {err.max():.3f}px mean {err.mean():.3f}px")
    assert err.max() < 1.0


def test_ground_homography_roundtrip():
    cam = _cam(fx=600.0, fy=480.0, cx=250.0, cy=140.0, pitch_deg=6.0, yaw_deg=3.0, roll_deg=1.0)
    for state in (None, CarState(5, 3, 1.0, roll=0.02, pitch=-0.03)):
        H = ground_homography(cam, state)
        assert abs(H[2, 2] - 1) < 1e-9
        # a flat-ground point in the gravity-aligned base frame -> world -> pixel -> back
        st = state or CarState(0, 0, 0)
        g = np.array([[8.0, 1.0], [15.0, -2.0], [30.0, 0.5]])
        c, s_ = np.cos(st.yaw), np.sin(st.yaw)
        world = np.column_stack([st.x + c * g[:, 0] - s_ * g[:, 1], st.y + s_ * g[:, 0] + c * g[:, 1],
                                 np.zeros(len(g))])
        uv, depth = project_world_points(world, st, cam)
        assert (depth > 0).all()
        assert np.allclose(pixel_to_ground(H, uv), g, atol=1e-6)


def test_homography_vs_render_ground_points():
    cam = _cam(fx=700.0, fy=700.0, cx=300.0, cy=170.0)
    ground = np.array([[6.0, 0.0], [10.0, 2.0], [14.0, -3.0], [20.0, 1.0], [8.0, -1.5], [30.0, 0.0]])
    pts = np.column_stack([ground, np.full(len(ground), 0.06)])  # sphere centre one radius up
    model = _scene(pts, cam)
    uv = _render_centroids(model, CarState(0, 0, 0), cam, len(pts))
    ok = ~np.isnan(uv[:, 0])
    assert ok.sum() >= 5
    g = pixel_to_ground(ground_homography(cam), uv[ok])
    # the pixel sees the sphere centre (6 cm up); the ray hits the ground further out
    c = np.array([cam["x"], cam["y"]])
    expect = c + (ground[ok] - c) * cam["z"] / (cam["z"] - 0.06)
    assert np.abs(g - expect).max() < 0.05 * 1.0 + 0.002 * np.hypot(*ground[ok].T).max()


def test_hfov_derivation():
    cam = _cam(fx=None, fy=None, hfov_deg=90.0)
    K = intrinsic_matrix(cam)
    assert abs(K[0, 0] - 320.0) < 1e-9 and abs(K[1, 1] - 320.0) < 1e-9


def test_T_base_cam_orthonormal_and_pitch_down():
    cam = _cam(pitch_deg=10.0, yaw_deg=0.0, roll_deg=0.0)
    T = T_base_cam(cam)
    assert np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3))
    z_axis = T[:3, 2]                    # optical axis in base_link
    assert z_axis[0] > 0.9 and z_axis[2] < -0.1          # forward and tilted down
    assert T[:3, 0][1] < -0.9                            # optical x points to the right (-y)
    cam2 = _cam(yaw_deg=10.0, pitch_deg=0.0)
    assert T_base_cam(cam2)[:3, 2][1] > 0.1              # yaw +ve = turned left
