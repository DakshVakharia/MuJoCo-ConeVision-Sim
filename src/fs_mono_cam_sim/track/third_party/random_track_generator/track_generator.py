"""Voronoi-based random closed centreline.

Derived from https://github.com/mvanlobensels/random-track-generator (MIT, see LICENSE),
commit 4ad014853493d5500aec9dd7121d6dc8f8c240c0. MODIFIED: see README.md in this directory.
"""
from enum import Enum

import numpy as np
from scipy import signal, spatial, interpolate

from .geometry import closest_node, clockwise_sort, curvature


class Mode(Enum):
    EXPAND = 1   # roundish tracks
    EXTEND = 2   # elongated tracks
    RANDOM = 3   # large/rambling tracks


def _bounded_voronoi(input_points, bounding_box):
    """Voronoi diagram with the points mirrored on the box edges (no infinite regions)."""
    def _mirror(boundary, axis):
        mirrored = np.copy(input_points)
        mirrored[:, axis] = 2 * boundary - mirrored[:, axis]
        return mirrored

    x_min, x_max, y_min, y_max = bounding_box
    points = np.concatenate([input_points, _mirror(x_min, 0), _mirror(x_max, 0),
                             _mirror(y_min, 1), _mirror(y_max, 1)])
    vor = spatial.Voronoi(points)
    return vor


def _select_points(rng, mode, input_points, n_points, n_regions, max_bound):
    if mode == Mode.EXPAND:
        idx = int(rng.integers(0, n_points))
        chosen = [idx]
        for i in range(min(n_regions, n_points) - 1):
            chosen.append(closest_node(input_points[idx], input_points, k=i + 1))
        return np.array(chosen)
    if mode == Mode.EXTEND:
        idx = int(rng.integers(0, n_points))
        heading = rng.uniform(0, np.pi / 2)
        p = input_points[idx]
        d = np.array([np.cos(heading), np.sin(heading)])
        rel = input_points - p
        dist = np.abs(rel[:, 0] * d[1] - rel[:, 1] * d[0])     # distance to the infinite line
        return np.argpartition(dist, n_regions)[:n_regions]
    return rng.integers(0, n_points, n_regions)


def generate_centerline(rng, n_points=30, n_regions=10, max_bound=100.0, mode=Mode.RANDOM,
                        min_radius=3.75, n_samples=1000, max_prune=200):
    """Returns an (n_samples, 2) closed centreline (last point != first), or None on failure.

    rng: numpy Generator. Curvature peaks above 1/min_radius are removed by deleting the
    offending Voronoi vertex and re-fitting the periodic spline (upstream algorithm).
    """
    mode = Mode[mode.upper()] if isinstance(mode, str) else mode
    pts = rng.uniform(0.0, max_bound, (n_points, 2))
    vor = _bounded_voronoi(pts, np.array([0.0, max_bound, 0.0, max_bound]))
    regions = np.array([np.array(r) for r in vor.regions], dtype=object)

    sel = _select_points(rng, mode, pts, n_points, n_regions, max_bound)
    ridx = vor.point_region[sel]
    verts_idx = np.concatenate(regions[ridx])
    verts_idx = verts_idx[verts_idx >= 0]
    verts = np.unique(vor.vertices[verts_idx], axis=0)
    if len(verts) < 5:
        return None
    sv = clockwise_sort(verts)
    sv = np.vstack([sv, sv[0]])

    thr = 1.0 / min_radius
    for _ in range(max_prune):
        if len(sv) < 6:
            return None
        try:
            tck, _u = interpolate.splprep([sv[:, 0], sv[:, 1]], s=0, per=True)
        except Exception:
            return None
        t = np.linspace(0, 1, n_samples)
        x, y = interpolate.splev(t, tck, der=0)
        dx, dy = interpolate.splev(t, tck, der=1)
        ddx, ddy = interpolate.splev(t, tck, der=2)
        k = np.abs(curvature(dx, ddx, dy, ddy))
        peaks, _p = signal.find_peaks(k)
        if len(peaks) == 0 or not np.isfinite(k).all():
            return None
        worst = peaks[k[peaks].argmax()]
        if k[worst] <= thr:
            return np.stack([x[:-1], y[:-1]], axis=1)
        v = closest_node((x[worst], y[worst]), sv, k=0)
        sv = np.delete(sv, v, axis=0)
        if not np.array_equal(sv[0], sv[-1]):
            sv = np.vstack([sv, sv[0]])
    return None
