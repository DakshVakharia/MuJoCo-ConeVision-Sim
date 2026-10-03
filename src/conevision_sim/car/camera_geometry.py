"""Pure-numpy pinhole camera geometry for the car's single camera (no MuJoCo import).

Frames
------
world      : x/y on the ground, z up.
base_link  : ground point under the car centre, x forward, y left, z up. Attached to the car, so it
             is rotated by the car's yaw / pitch / roll (R_world_base = Rz(yaw) Ry(pitch) Rx(roll),
             pitch positive = nose DOWN, roll positive = right side down... i.e. a right-handed
             rotation about +x).
camera     : OPTICAL frame, x right, y down, z forward (OpenCV convention).
pixel      : u right, v down, origin top-left of the full-resolution image.

Camera extrinsics from camera.yaml: position (x, y, z) in base_link; orientation
R_base_cam = Rz(yaw) Ry(pitch) Rx(roll) @ R_OPT, where pitch +ve tilts the camera DOWN and yaw +ve
turns it LEFT.
"""
import math

import numpy as np

# optical axes (x right, y down, z forward) expressed in base_link at zero roll/pitch/yaw
R_OPT = np.array([[0.0, 0.0, 1.0],
                  [-1.0, 0.0, 0.0],
                  [0.0, -1.0, 0.0]])


def _Rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def _Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def _Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def rpy_matrix(roll, pitch, yaw):
    """Rz(yaw) Ry(pitch) Rx(roll), angles in radians. Pitch +ve = x axis rotates DOWN."""
    return _Rz(yaw) @ _Ry(pitch) @ _Rx(roll)


# ------------------------------------------------------------------ intrinsics
def intrinsics(cam_cfg):
    """(fx, fy, cx, cy, width, height) in pixels. fx/fy derived from hfov_deg when null."""
    w, h = int(cam_cfg["width"]), int(cam_cfg["height"])
    fx, fy = cam_cfg.get("fx"), cam_cfg.get("fy")
    if fx is None or fy is None:
        hfov = cam_cfg.get("hfov_deg")
        if hfov is None:
            raise ValueError("camera config needs fx/fy or hfov_deg")
        fx = fy = (w / 2.0) / math.tan(math.radians(hfov) / 2.0)
    cx = cam_cfg.get("cx")
    cy = cam_cfg.get("cy")
    cx = w / 2.0 if cx is None else cx
    cy = h / 2.0 if cy is None else cy
    return float(fx), float(fy), float(cx), float(cy), w, h


def intrinsic_matrix(cam_cfg):
    fx, fy, cx, cy, _, _ = intrinsics(cam_cfg)
    return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]])


# ------------------------------------------------------------------ extrinsics
def R_base_cam(cam_cfg):
    r, p, y = (math.radians(cam_cfg.get(k, 0.0)) for k in ("roll_deg", "pitch_deg", "yaw_deg"))
    return rpy_matrix(r, p, y) @ R_OPT


def T_base_cam(cam_cfg):
    """4x4 transform: points in the optical camera frame -> base_link."""
    T = np.eye(4)
    T[:3, :3] = R_base_cam(cam_cfg)
    T[:3, 3] = [cam_cfg["x"], cam_cfg["y"], cam_cfg["z"]]
    return T


def T_world_base(car_state):
    """4x4 transform base_link -> world for a CarState (z of base_link origin = 0)."""
    T = np.eye(4)
    T[:3, :3] = rpy_matrix(car_state.roll, car_state.pitch, car_state.yaw)
    T[:3, 3] = [car_state.x, car_state.y, 0.0]
    return T


def mat_to_quat(R):
    """Rotation matrix -> unit quaternion (w, x, y, z), the MuJoCo convention."""
    R = np.asarray(R, dtype=float)
    t = np.trace(R)
    if t > 0:
        s = 2.0 * math.sqrt(t + 1.0)
        q = [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2.0 * math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k])
        q = [0.0] * 4
        q[0] = (R[k, j] - R[j, k]) / s
        q[1 + i] = 0.25 * s
        q[1 + j] = (R[j, i] + R[i, j]) / s
        q[1 + k] = (R[k, i] + R[i, k]) / s
    q = np.array(q)
    return q / np.linalg.norm(q)


def car_mocap_pose(car_state):
    """(pos[3], quat_wxyz[4]) to write into data.mocap_pos[i] / data.mocap_quat[i] for the car body."""
    T = T_world_base(car_state)
    return T[:3, 3].copy(), mat_to_quat(T[:3, :3])


# ------------------------------------------------------------------ projection
def project_world_points(points_world, car_state, cam_cfg):
    """Project world points (N,3) into the image. Returns (uv (N,2) pixels, depth (N,) metres along
    the optical axis). Points behind the camera (depth <= 0) get uv = NaN."""
    P = np.atleast_2d(np.asarray(points_world, dtype=float))
    T_wc = T_world_base(car_state) @ T_base_cam(cam_cfg)
    R, t = T_wc[:3, :3], T_wc[:3, 3]
    pc = (P - t) @ R                     # == R.T @ (p - t) for each row
    depth = pc[:, 2].copy()
    K = intrinsic_matrix(cam_cfg)
    with np.errstate(divide="ignore", invalid="ignore"):
        uv = np.stack([K[0, 0] * pc[:, 0] / depth + K[0, 2],
                       K[1, 1] * pc[:, 1] / depth + K[1, 2]], axis=1)
    uv[depth <= 0] = np.nan
    return uv, depth


def ground_homography(cam_cfg, car_state=None):
    """3x3 H mapping pixel (u, v, 1) -> flat-ground point (x forward, y left, 1) in metres, exact.

    Ground coordinates are in the gravity-aligned base_link frame (origin under the car centre,
    x along the car heading, z = world up). With car_state=None the car is level (roll = pitch = 0);
    with a car_state its roll/pitch (not yaw) tilt the camera relative to the ground.
    Normalised so H[2, 2] = 1 (when that entry is non-zero).
    """
    Rb = R_base_cam(cam_cfg)
    t = np.array([cam_cfg["x"], cam_cfg["y"], cam_cfg["z"]], dtype=float)
    if car_state is not None:
        Rl = rpy_matrix(car_state.roll, car_state.pitch, 0.0)   # level frame <- base_link
        Rb, t = Rl @ Rb, Rl @ t
    Rt = Rb.T
    # pixel ~ K R^T (X - t) with X = (gx, gy, 0)
    M = intrinsic_matrix(cam_cfg) @ np.column_stack([Rt[:, 0], Rt[:, 1], -Rt @ t])
    H = np.linalg.inv(M)
    if abs(H[2, 2]) > 1e-12:
        H = H / H[2, 2]
    return H


def pixel_to_ground(H, uv):
    uv = np.atleast_2d(np.asarray(uv, dtype=float))
    p = np.column_stack([uv, np.ones(len(uv))]) @ H.T
    return p[:, :2] / p[:, 2:3]
