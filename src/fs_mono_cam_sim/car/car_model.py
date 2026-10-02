"""STUB - to be replaced by the car agent.

Contract:
    add_car(worldbody: ET.Element, asset: ET.Element, car_cfg: dict, cam_cfg: dict) -> ET.Element
Adds a mocap body named CAR_BODY (origin = base_link: ground under the car centre,
x fwd, y left, z up) containing the car geoms and a camera named cam_cfg["name"].
Returns the car body element.
"""
import math
import xml.etree.ElementTree as ET

CAR_BODY = "car"


def camera_xyaxes(roll_deg, pitch_deg, yaw_deg):
    """MuJoCo camera axes (x right, y up, looks along -z) expressed in base_link."""
    import numpy as np
    r, p, y = (math.radians(v) for v in (roll_deg, pitch_deg, yaw_deg))
    Rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    Ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    Rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    # columns: camera x = -y_base, camera y = +z_base, camera z = -x_base
    R0 = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]], dtype=float)
    R = Rz @ Ry @ Rx @ R0
    return R[:, 0], R[:, 1]


def add_car(worldbody, asset, car_cfg, cam_cfg):
    body = ET.SubElement(worldbody, "body", name=CAR_BODY, mocap="true", pos="0 0 0")
    L, W = car_cfg.get("length", 2.9), car_cfg.get("width", 1.4)
    rgba = " ".join(str(v) for v in car_cfg.get("body_color", [0.8, 0.1, 0.1, 1]))
    ET.SubElement(body, "geom", name="car_chassis", type="box",
                  size=f"{L/2} {W/4} 0.2", pos="0 0 0.35", rgba=rgba,
                  contype="0", conaffinity="0")
    xa, ya = camera_xyaxes(cam_cfg["roll_deg"], cam_cfg["pitch_deg"], cam_cfg["yaw_deg"])
    fovy = math.degrees(2 * math.atan(cam_cfg["height"] / (2 * cam_cfg["fy"])))
    ET.SubElement(body, "camera", name=cam_cfg["name"],
                  pos=f"{cam_cfg['x']} {cam_cfg['y']} {cam_cfg['z']}",
                  xyaxes=" ".join(f"{v:.6f}" for v in (*xa, *ya)),
                  fovy=f"{fovy:.4f}")
    return body
