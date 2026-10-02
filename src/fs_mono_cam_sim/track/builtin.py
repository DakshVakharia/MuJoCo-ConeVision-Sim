"""Self-contained random closed-loop generator (no third-party code).

Algorithm: random points -> convex hull -> displace edge midpoints (creates concavities)
-> push close points apart -> periodic smoothing spline. Returns a raw closed centreline;
all validation (self-intersection, corner radius, length, boundary overlap) is done by
`generator._finalize`, which rejects bad candidates so the caller can retry.
"""
import numpy as np
from scipy import interpolate, spatial


def builtin_centerline(rng, n_points=(14, 24), displacement=0.45, smoothing=1.0,
                       min_point_dist=0.12, box=100.0, n_samples=1000):
    """Return an (n_samples, 2) closed centreline (last point != first), or None."""
    n = int(rng.integers(n_points[0], n_points[1] + 1))
    pts = rng.uniform(0.0, box, (n, 2))
    try:
        hull = spatial.ConvexHull(pts)
    except Exception:
        return None
    p = pts[hull.vertices]                       # counter-clockwise
    if len(p) < 4:
        return None

    # midpoint displacement: add a randomly offset point on every hull edge
    out = []
    for i in range(len(p)):
        a, b = p[i], p[(i + 1) % len(p)]
        out.append(a)
        e = b - a
        L = np.linalg.norm(e)
        nrm = np.array([-e[1], e[0]]) / max(L, 1e-9)
        out.append((a + b) / 2 + nrm * rng.uniform(-1.0, 1.0) * displacement * L * 0.5)
    p = np.array(out)

    # push apart points that are too close (relative to box size)
    dmin = min_point_dist * box
    for _ in range(20):
        moved = False
        for i in range(len(p)):
            for j in range(i + 1, len(p)):
                d = p[j] - p[i]
                dist = np.linalg.norm(d)
                if dist < dmin:
                    push = (d / max(dist, 1e-9)) * (dmin - dist) * 0.5
                    p[j] += push
                    p[i] -= push
                    moved = True
        if not moved:
            break

    p = np.vstack([p, p[:1]])
    try:
        tck, _u = interpolate.splprep([p[:, 0], p[:, 1]], s=smoothing * len(p), per=True)
    except Exception:
        return None
    t = np.linspace(0, 1, n_samples + 1)[:-1]
    x, y = interpolate.splev(t, tck)
    return np.stack([x, y], axis=1)
