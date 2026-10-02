"""Random Formula Student track generation: generate_track(track_cfg) -> Track.

Backends
- third_party: Voronoi-based centreline from mvanlobensels/random-track-generator (vendored, MIT)
- builtin:     self-contained hull/spline generator (builtin.py)
- auto:        third_party, falling back to builtin on any failure

Both backends only produce a raw closed centreline; `_finalize` does everything else (scaling to
the target length, validation, start-line placement, cone placement) so the output contract is
identical for both. Convention: blue cones LEFT of the driving direction, yellow RIGHT, four
big orange cones at the start line (two per side, 5 m apart). The start is at the origin,
heading +x.
"""
import math
import secrets

import numpy as np
from scipy.ndimage import gaussian_filter1d
from scipy.spatial import cKDTree

from .track_types import Track

DS = 0.5                     # centreline sample spacing [m]
ORANGE_GATE_SPACING = 5.0    # distance between the two orange pairs [m]
MIN_CONE_DIST = 1.0          # no two cones closer than this [m]


# --------------------------------------------------------------------------- helpers
def _arclen(loop):
    closed = np.vstack([loop, loop[:1]])
    seg = np.linalg.norm(np.diff(closed, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(seg)])


def _resample_closed(loop, ds):
    """Uniform arc-length resampling of a closed polyline; returns (M,2) and actual spacing."""
    s = _arclen(loop)
    total = s[-1]
    n = max(int(round(total / ds)), 8)
    t = np.linspace(0, total, n, endpoint=False)
    closed = np.vstack([loop, loop[:1]])
    return np.stack([np.interp(t, s, closed[:, 0]), np.interp(t, s, closed[:, 1])], axis=1), total / n


def _smooth_periodic(loop, sigma_m, ds):
    sg = sigma_m / ds
    return np.stack([gaussian_filter1d(loop[:, 0], sg, mode="wrap"),
                     gaussian_filter1d(loop[:, 1], sg, mode="wrap")], axis=1)


def _tangent_curvature(loop, ds):
    d = np.roll(loop, -1, axis=0) - np.roll(loop, 1, axis=0)
    tang = d / np.linalg.norm(d, axis=1, keepdims=True)
    th = np.arctan2(tang[:, 1], tang[:, 0])
    dth = np.roll(th, -1) - np.roll(th, 1)
    dth = (dth + np.pi) % (2 * np.pi) - np.pi
    return tang, dth / (2 * ds)           # signed curvature, +ve = left turn


def _offset_resample(loop, tang, offset, spacing):
    """Offset closed polyline to the left (+offset) and resample at <= spacing along its length."""
    normal = np.stack([-tang[:, 1], tang[:, 0]], axis=1)
    off = loop + offset * normal
    s = _arclen(off)
    n = max(int(math.ceil(s[-1] / spacing)), 3)
    t = np.linspace(0, s[-1], n, endpoint=False)
    closed = np.vstack([off, off[:1]])
    return np.stack([np.interp(t, s, closed[:, 0]), np.interp(t, s, closed[:, 1])], axis=1)


def _loop_ok(cl, ds, width, margin=1.0):
    """No self-intersection / boundary overlap: centreline points that are far apart along the
    track must be farther apart than width + margin in space."""
    n = len(cl)
    k_excl = int(max(12.0, 3.0 * width) / ds)
    pairs = cKDTree(cl).query_pairs(width + margin, output_type="ndarray")
    if len(pairs) == 0:
        return True
    di = np.abs(pairs[:, 0] - pairs[:, 1])
    di = np.minimum(di, n - di)
    return not np.any(di > k_excl)


# --------------------------------------------------------------------------- finalize
def _finalize(raw, cfg, rng):
    """Turn a raw closed centreline into a validated Track, or None if it must be rejected."""
    width = float(cfg.get("track_width_m", 3.5))
    spacing = float(cfg.get("cone_spacing_m", 4.0))
    target = float(cfg.get("target_length_m", 300.0))
    rmin = float(cfg.get("min_corner_radius_m", 5.0))
    bp = cfg.get("backend_params") or {}
    k_range = bp.get("scale_range", [0.6, 1.8])

    if raw is None or not np.isfinite(raw).all():
        return None
    cl, ds = _resample_closed(np.asarray(raw, float), DS)
    k = target / (ds * len(cl))                # uniform scale so length == target
    if not (k_range[0] <= k <= k_range[1]):
        return None
    cl, ds = _resample_closed(cl * k, DS)
    cl = _smooth_periodic(cl, 1.0, ds)
    cl, ds = _resample_closed(cl, DS)

    tang, kappa = _tangent_curvature(cl, ds)
    if np.abs(kappa).max() > 1.0 / (rmin * 0.98):
        return None
    if abs(abs(np.sum(kappa) * ds) - 2 * np.pi) > 0.3:     # simple loop turns by exactly 2*pi
        return None
    if not _loop_ok(cl, ds, width):
        return None

    if rng.random() < 0.5:                                  # random driving direction
        cl = cl[::-1].copy()
        tang, kappa = _tangent_curvature(cl, ds)

    # ---- start line: straightest window covering [-6 m, gate spacing + 6 m]
    n = len(cl)
    back, fwd = int(6 / ds), int((ORANGE_GATE_SPACING + 6) / ds)
    ak = np.abs(kappa)
    win = np.array([ak[np.arange(i - back, i + fwd + 1) % n].max() for i in range(n)])
    good = np.where(win <= 1.0 / 60.0)[0]
    i0 = int(rng.choice(good)) if len(good) else int(win.argmin())
    cl = np.roll(cl, -i0, axis=0)
    tang = np.roll(tang, -i0, axis=0)

    # normalise pose: start at origin heading +x
    yaw0 = math.atan2(tang[0, 1], tang[0, 0])
    c, s = math.cos(-yaw0), math.sin(-yaw0)
    cl = (cl - cl[0]) @ np.array([[c, -s], [s, c]]).T
    tang, kappa = _tangent_curvature(cl, ds)

    # ---- cones
    half = width / 2.0
    blue = _offset_resample(cl, tang, +half, spacing)
    yellow = _offset_resample(cl, tang, -half, spacing)
    gate = int(round(ORANGE_GATE_SPACING / ds))
    orange = []
    for idx in (0, gate):
        nrm = np.array([-tang[idx, 1], tang[idx, 0]])
        orange.append(cl[idx] + half * nrm)      # left
        orange.append(cl[idx] - half * nrm)      # right
    orange = np.array(orange)

    def _clear(arr):   # drop blue/yellow cones that would collide with the start gate
        return arr[np.min(np.linalg.norm(arr[:, None] - orange[None], axis=2), axis=1) > 2.0]
    blue, yellow = _clear(blue), _clear(yellow)

    # ---- cone validation
    allc = np.vstack([blue, yellow, orange])
    dd, _ = cKDTree(allc).query(allc, k=2)
    if dd[:, 1].min() < MIN_CONE_DIST:
        return None
    dcl, _ = cKDTree(cl).query(allc)
    if dcl.min() < half - 0.2:
        return None

    return Track(
        centerline=cl,
        cones={"blue": blue, "yellow": yellow, "orange_big": orange},
        start_pose=(0.0, 0.0, 0.0),
        meta={"length_m": float(ds * len(cl)),
              "track_width_m": width,
              "cone_spacing_m": spacing,
              "min_radius_m": float(1.0 / max(np.abs(kappa).max(), 1e-9)),
              "direction": "ccw" if np.sum(kappa) > 0 else "cw",
              "n_blue": int(len(blue)), "n_yellow": int(len(yellow))})


# --------------------------------------------------------------------------- backends
def _raw_third_party(rng, cfg):
    from .third_party.random_track_generator import generate_centerline
    target = float(cfg.get("target_length_m", 300.0))
    rmin = float(cfg.get("min_corner_radius_m", 5.0))
    p = (cfg.get("backend_params") or {}).get("third_party", {}) or {}
    # box size so the raw loop is roughly target long (empirical: length ~ length_per_bound*box)
    box = float(p.get("max_bound_m") or target / float(p.get("length_per_bound", 3.0)))
    return generate_centerline(
        rng, n_points=int(p.get("n_points", 30)), n_regions=int(p.get("n_regions", 8)),
        max_bound=box, mode=p.get("mode", "random"),
        min_radius=rmin * float(p.get("radius_margin", 1.05)))


def _raw_builtin(rng, cfg):
    from .builtin import builtin_centerline
    p = (cfg.get("backend_params") or {}).get("builtin", {}) or {}
    return builtin_centerline(
        rng, n_points=tuple(p.get("n_points", (14, 24))),
        displacement=float(p.get("displacement", 0.45)),
        smoothing=float(p.get("smoothing", 1.0)))


_RAW = {"third_party": _raw_third_party, "builtin": _raw_builtin}


def _run_backend(name, cfg, seed, max_attempts):
    raw_fn = _RAW[name]
    for attempt in range(max_attempts):
        rng = np.random.default_rng([seed, attempt])
        try:
            track = _finalize(raw_fn(rng, cfg), cfg, rng)
        except Exception:
            track = None
        if track is not None:
            track.seed = seed
            track.meta.update(backend=name, attempts=attempt + 1)
            return track
    return None


def generate_track(track_cfg):
    """Generate a Track from the `track:` section of config/track.yaml."""
    cfg = dict(track_cfg or {})
    seed = cfg.get("seed")
    seed = secrets.randbelow(2**31) if seed is None else int(seed)
    backend = str(cfg.get("backend", "auto")).lower()
    max_attempts = int((cfg.get("backend_params") or {}).get("max_attempts", 500))

    order = {"auto": ["third_party", "builtin"], "third_party": ["third_party"],
             "builtin": ["builtin"]}.get(backend)
    if order is None:
        raise ValueError(f"unknown track backend {backend!r}")
    for name in order:
        track = _run_backend(name, cfg, seed, max_attempts)
        if track is not None:
            if name != order[0]:
                track.meta["fallback"] = True
            return track
    raise RuntimeError(f"track generation failed for backend={backend!r} seed={seed}")
