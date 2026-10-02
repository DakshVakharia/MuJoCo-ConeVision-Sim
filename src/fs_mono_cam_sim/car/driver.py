"""STUB - to be replaced by the car agent.

Contract:
    class CenterlineDriver(track: Track, car_cfg: dict)
        .state_at(t: float) -> CarState   # pure function of sim time t (seconds)
"""
from dataclasses import dataclass

import numpy as np


@dataclass
class CarState:
    x: float
    y: float
    yaw: float            # rad, world frame
    roll: float = 0.0     # rad
    pitch: float = 0.0    # rad, positive = nose down
    v: float = 0.0        # m/s forward
    yaw_rate: float = 0.0 # rad/s


class CenterlineDriver:
    def __init__(self, track, car_cfg):
        c = track.centerline
        closed = np.vstack([c, c[:1]])
        self._pts = closed
        self._s = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(closed, axis=0), axis=1))])
        self.speed = car_cfg.get("driver", {}).get("speed_mps", 8.0)

    def state_at(self, t):
        L = self._s[-1]
        s = (self.speed * t) % L
        x = np.interp(s, self._s, self._pts[:, 0])
        y = np.interp(s, self._s, self._pts[:, 1])
        s2 = (s + 1.0) % L
        x2 = np.interp(s2, self._s, self._pts[:, 0])
        y2 = np.interp(s2, self._s, self._pts[:, 1])
        return CarState(x=float(x), y=float(y), yaw=float(np.arctan2(y2 - y, x2 - x)), v=self.speed)
