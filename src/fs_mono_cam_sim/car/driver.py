"""Kinematic centreline driver.

    CenterlineDriver(track, car_cfg).state_at(t) -> CarState(x, y, yaw, roll, pitch, v, yaw_rate)

state_at is a PURE function of t (everything is precomputed in __init__), so any frame can be
rendered independently and the lap wraps seamlessly.

Pipeline: closed centreline -> periodic cubic spline -> uniform arc-length table (DS metres)
-> optional circular smoothing -> lateral offset -> heading from the chord between the points
lookahead/2 behind and ahead (smooth, no lag) -> speed profile v(s) -> time table t(s).
"""
import math
from dataclasses import dataclass

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.ndimage import gaussian_filter1d

DS = 0.05  # arc-length table resolution (m)


@dataclass
class CarState:
    x: float
    y: float
    yaw: float            # rad, world frame
    roll: float = 0.0     # rad
    pitch: float = 0.0    # rad, positive = nose down
    v: float = 0.0        # m/s forward
    yaw_rate: float = 0.0  # rad/s


def _periodic_arclength_table(centerline, ds, smooth_m):
    c = np.asarray(centerline, float)
    closed = np.vstack([c, c[:1]])
    chord = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(closed, axis=0), axis=1))])
    keep = np.concatenate([[True], np.diff(chord) > 1e-9])           # drop duplicate points
    closed, chord = closed[keep], chord[keep]
    if np.linalg.norm(closed[-1] - closed[0]) > 1e-9:
        closed = np.vstack([closed, closed[:1]])
        chord = np.append(chord, chord[-1] + np.linalg.norm(closed[-1] - closed[-2]))
    else:
        closed[-1] = closed[0]
    spl = CubicSpline(chord, closed, bc_type="periodic")
    fine = np.linspace(0, chord[-1], max(int(chord[-1] / 0.02), 200), endpoint=False)
    P = spl(fine)
    seg = np.linalg.norm(np.diff(np.vstack([P, P[:1]]), axis=0), axis=1)
    s_fine = np.concatenate([[0], np.cumsum(seg)])
    L = s_fine[-1]
    n = int(round(L / ds))
    s = np.arange(n) * (L / n)
    Pe = np.vstack([P, P[:1]])
    pts = np.stack([np.interp(s, s_fine, Pe[:, 0]), np.interp(s, s_fine, Pe[:, 1])], axis=1)
    if smooth_m > 0:
        sig = smooth_m / (L / n)
        pts = np.stack([gaussian_filter1d(pts[:, k], sig, mode="wrap") for k in range(2)], axis=1)
    return pts, L / n, L


class CenterlineDriver:
    def __init__(self, track, car_cfg):
        d = car_cfg.get("driver", {}) or {}
        self.speed = float(d.get("speed_mps", 8.0))
        self.lateral_offset = float(d.get("lateral_offset_m", 0.0))
        lookahead = float(d.get("lookahead_m", 4.0))
        smooth = float(d.get("centerline_smoothing_m", 0.0) or 0.0)
        a_lat = d.get("max_lateral_accel_mps2")
        a_acc = float(d.get("max_accel_mps2", 4.0))
        a_brk = float(d.get("max_brake_mps2", 6.0))
        bm = car_cfg.get("body_motion", {}) or {}
        self._pa = math.radians(float(bm.get("pitch_amplitude_deg", 0.0)))
        self._ra = math.radians(float(bm.get("roll_amplitude_deg", 0.0)))
        self._fb = float(bm.get("frequency_hz", 1.5))

        pts, ds, L = _periodic_arclength_table(track.centerline, DS, smooth)
        n = len(pts)
        self._n, self._ds, self.length = n, ds, n * ds

        # tangent, left normal, curvature of the centreline
        tang = np.roll(pts, -1, axis=0) - np.roll(pts, 1, axis=0)
        tang /= np.linalg.norm(tang, axis=1, keepdims=True)
        nrm = np.stack([-tang[:, 1], tang[:, 0]], axis=1)
        w = max(int(round(1.0 / ds)), 1)                                # 1 m window for curvature
        t1, t0 = np.roll(tang, -w, axis=0), np.roll(tang, w, axis=0)
        dth = np.arctan2(t0[:, 0] * t1[:, 1] - t0[:, 1] * t1[:, 0], (t0 * t1).sum(axis=1))
        kappa = dth / (2 * w * ds)

        path = pts + self.lateral_offset * nrm                          # driven path
        # start position: nearest centreline sample to start_pose, plus start_s_m
        sp = np.asarray(track.start_pose[:2], float)
        s0 = float(np.argmin(np.linalg.norm(pts - sp, axis=1)) * ds)
        self._s_start = (s0 + float(d.get("start_s_m", 0.0))) % self.length

        # heading: chord between points lookahead/2 behind and ahead along the driven path
        half = max(int(round(lookahead / 2 / ds)), 1)
        ahead, behind = np.roll(path, -half, axis=0), np.roll(path, half, axis=0)
        yaw = np.unwrap(np.arctan2(ahead[:, 1] - behind[:, 1], ahead[:, 0] - behind[:, 0]))
        yaw = np.concatenate([yaw, [yaw[0] + 2 * math.pi * round((yaw[-1] - yaw[0]) / (2 * math.pi))]])
        # (after unwrap, a closed loop turns a whole number of 2*pi; keep table periodic up to that)
        self._path = np.vstack([path, path[:1]])
        self._yaw = yaw
        self._s = np.arange(n + 1) * ds

        # speed profile v(s)
        v = np.full(n, self.speed)
        if a_lat:
            k_path = np.abs(kappa / np.maximum(1 - self.lateral_offset * kappa, 0.05))
            v = np.minimum(v, np.sqrt(float(a_lat) / np.maximum(k_path, 1e-6)))
            v = np.maximum(v, 1.0)
            vv = np.tile(v, 3)                                           # 3 laps -> periodic limits
            for i in range(1, len(vv)):
                vv[i] = min(vv[i], math.sqrt(vv[i - 1] ** 2 + 2 * a_acc * ds))
            for i in range(len(vv) - 2, -1, -1):
                vv[i] = min(vv[i], math.sqrt(vv[i + 1] ** 2 + 2 * a_brk * ds))
            v = vv[n:2 * n]
        self._v = np.append(v, v[0])
        # time table (trapezoid in 1/v)
        dt = 2 * ds / (self._v[:-1] + self._v[1:])
        self._t = np.concatenate([[0], np.cumsum(dt)])
        self.lap_time = float(self._t[-1])
        self._t_start = float(np.interp(self._s_start, self._s, self._t))
        # yaw rate table = dyaw/ds * v
        dyaw_ds = np.gradient(self._yaw, ds)
        self._yawrate = dyaw_ds * self._v

    def _s_at(self, t):
        tt = (self._t_start + t) % self.lap_time
        return float(np.interp(tt, self._t, self._s))

    def state_at(self, t):
        s = self._s_at(t)
        x = float(np.interp(s, self._s, self._path[:, 0]))
        y = float(np.interp(s, self._s, self._path[:, 1]))
        yaw = float(np.interp(s, self._s, self._yaw))
        v = float(np.interp(s, self._s, self._v))
        yr = float(np.interp(s, self._s, self._yawrate))
        yaw = (yaw + math.pi) % (2 * math.pi) - math.pi
        w = 2 * math.pi * self._fb
        pitch = self._pa * math.sin(w * t)
        roll = self._ra * math.sin(1.31 * w * t + 0.9)
        return CarState(x=x, y=y, yaw=yaw, roll=roll, pitch=pitch, v=v, yaw_rate=yr)
