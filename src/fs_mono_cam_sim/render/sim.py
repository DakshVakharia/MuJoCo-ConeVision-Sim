"""Simulation: track + scene + car + renderer, driven by simulated time."""
import time
from dataclasses import dataclass, field

import mujoco
import numpy as np

from ..car.driver import CenterlineDriver
from ..scene.builder import build_scene_xml
from ..track.generator import generate_track
from .bboxes import compute_bboxes
from .renderer import CameraRenderer


@dataclass
class Frame:
    t: float
    rgb: np.ndarray
    detections: list
    car_state: object
    timings: dict = field(default_factory=dict)


def euler_to_quat(yaw, pitch, roll):
    """R = Rz(yaw) Ry(pitch) Rx(roll) -> MuJoCo quat (w, x, y, z).

    pitch is the rotation angle about +y (positive = nose down in an x-forward, z-up frame).
    """
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    return np.array([cr * cp * cy + sr * sp * sy,
                     sr * cp * cy - cr * sp * sy,
                     cr * sp * cy + sr * cp * sy,
                     cr * cp * sy - sr * sp * cy])


class Simulation:
    def __init__(self, cfg, track=None, seed=None):
        self.cfg = cfg
        self.cam_cfg = cfg["camera"]
        self.bbox_cfg = cfg["perception"]["bbox"]
        self.fps = float(self.cam_cfg["fps"])
        self.track = track if track is not None else generate_track(cfg["track"])
        self.model = mujoco.MjModel.from_xml_string(build_scene_xml(self.track, cfg))
        self.data = mujoco.MjData(self.model)
        self.driver = CenterlineDriver(self.track, cfg["car"])
        self.renderer = CameraRenderer(
            self.model, self.cam_cfg,
            msaa_samples=cfg["perception"].get("render", {}).get("msaa_samples"))
        if seed is None:
            seed = cfg["perception"].get("seed", 0)
        self.rng = np.random.default_rng(seed)
        self.t = 0.0
        self.step_to(0.0)

    def step_to(self, t):
        st = self.driver.state_at(t)
        self.data.mocap_pos[0] = [st.x, st.y, 0.0]
        self.data.mocap_quat[0] = euler_to_quat(st.yaw, st.pitch, st.roll)
        mujoco.mj_forward(self.model, self.data)
        self.t = t
        return st

    def frame(self, t):
        """Render RGB + detections at simulated time t (seconds)."""
        t0 = time.perf_counter()
        st = self.step_to(t)
        t1 = time.perf_counter()
        rgb = self.renderer.render_rgb(self.data)
        t2 = time.perf_counter()
        seg = self.renderer.render_segmentation(self.data)
        t3 = time.perf_counter()
        dets = compute_bboxes(self.model, self.data, seg, self.cam_cfg, self.bbox_cfg, self.rng)
        t4 = time.perf_counter()
        tm = {"step": t1 - t0, "rgb": t2 - t1, "seg": t3 - t2, "bbox": t4 - t3, "total": t4 - t0}
        return Frame(t=t, rgb=rgb, detections=dets, car_state=st, timings=tm)

    def frame_at_index(self, k):
        """Frame k at exactly t = k / fps."""
        return self.frame(k / self.fps)

    def close(self):
        self.renderer.close()
