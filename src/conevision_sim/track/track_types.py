"""The Track data contract shared by every part of the simulator.

World frame: x/y on the ground plane (metres), z up. FS convention:
blue cones on the LEFT of the driving direction, yellow on the RIGHT,
big orange cones at the start/finish line.
"""
from dataclasses import dataclass, field

import numpy as np
import yaml

CONE_CLASSES = ("blue", "yellow", "orange", "orange_big")


@dataclass
class Track:
    centerline: np.ndarray            # (N, 2) closed loop, ordered in driving direction; last point != first
    cones: dict                       # class name -> (M, 2) array of cone positions
    start_pose: tuple = (0.0, 0.0, 0.0)   # (x, y, yaw) of the car at the start line
    seed: int = None
    meta: dict = field(default_factory=dict)  # backend name, length, etc.

    def __post_init__(self):
        self.centerline = np.asarray(self.centerline, dtype=float).reshape(-1, 2)
        self.cones = {c: np.asarray(self.cones.get(c, []), dtype=float).reshape(-1, 2)
                      for c in CONE_CLASSES}

    def length(self):
        d = np.diff(np.vstack([self.centerline, self.centerline[:1]]), axis=0)
        return float(np.linalg.norm(d, axis=1).sum())

    def all_cones(self):
        """List of (class, index, x, y) for every cone."""
        return [(c, i, float(p[0]), float(p[1]))
                for c in CONE_CLASSES for i, p in enumerate(self.cones[c])]

    # ---- serialisation -------------------------------------------------
    def to_dict(self):
        return {
            "seed": self.seed,
            "start_pose": [float(v) for v in self.start_pose],
            "meta": self.meta,
            "centerline": self.centerline.round(4).tolist(),
            "cones": {c: self.cones[c].round(4).tolist() for c in CONE_CLASSES},
        }

    def save(self, path):
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(self.to_dict(), f, default_flow_style=None, sort_keys=False)

    @classmethod
    def load(cls, path):
        with open(path, "r", encoding="utf-8") as f:
            d = yaml.safe_load(f)
        return cls(centerline=d["centerline"], cones=d["cones"],
                   start_pose=tuple(d.get("start_pose", (0, 0, 0))),
                   seed=d.get("seed"), meta=d.get("meta", {}))

    # ---- test fixture --------------------------------------------------
    @classmethod
    def make_test_oval(cls, straight=30.0, radius=12.0, width=3.5, spacing=4.0):
        """Simple stadium-shaped track, driven counter-clockwise. For tests and stubs."""
        pts = []
        n_arc = 40
        for x in np.arange(-straight / 2, straight / 2, 1.0):          # bottom straight, +x
            pts.append((x, -radius))
        for a in np.linspace(-np.pi / 2, np.pi / 2, n_arc, endpoint=False):
            pts.append((straight / 2 + radius * np.cos(a), radius * np.sin(a)))
        for x in np.arange(straight / 2, -straight / 2, -1.0):         # top straight, -x
            pts.append((x, radius))
        for a in np.linspace(np.pi / 2, 3 * np.pi / 2, n_arc, endpoint=False):
            pts.append((-straight / 2 + radius * np.cos(a), radius * np.sin(a)))
        center = np.array(pts)
        left, right = _offset_boundaries(center, width / 2)
        blue = _resample(left, spacing)
        yellow = _resample(right, spacing)
        start = center[0]
        orange_big = np.array([[start[0], start[1] + width / 2 + 0.3],
                               [start[0], start[1] - width / 2 - 0.3]])
        return cls(centerline=center,
                   cones={"blue": blue, "yellow": yellow, "orange_big": orange_big},
                   start_pose=(float(start[0]), float(start[1]), 0.0),
                   seed=0, meta={"backend": "test_oval"})


def _offset_boundaries(center, half_width):
    nxt = np.roll(center, -1, axis=0)
    prv = np.roll(center, 1, axis=0)
    tangent = nxt - prv
    tangent /= np.linalg.norm(tangent, axis=1, keepdims=True)
    normal = np.stack([-tangent[:, 1], tangent[:, 0]], axis=1)   # points LEFT
    return center + half_width * normal, center - half_width * normal


def _resample(loop, spacing):
    closed = np.vstack([loop, loop[:1]])
    seg = np.linalg.norm(np.diff(closed, axis=0), axis=1)
    s = np.concatenate([[0], np.cumsum(seg)])
    n = max(int(s[-1] // spacing), 3)
    targets = np.linspace(0, s[-1], n, endpoint=False)
    return np.stack([np.interp(targets, s, closed[:, 0]),
                     np.interp(targets, s, closed[:, 1])], axis=1)
