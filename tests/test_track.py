import numpy as np
import pytest
from scipy.spatial import cKDTree

from fs_mono_cam_sim.config import load_config
from fs_mono_cam_sim.track.generator import generate_track
from fs_mono_cam_sim.track.track_types import Track

BACKENDS = ["third_party", "builtin"]
SEEDS = list(range(20))


def _cfg(backend, seed):
    cfg = dict(load_config("track"))
    cfg.update(backend=backend, seed=seed)
    return cfg


@pytest.fixture(scope="module")
def tracks():
    return {(b, s): generate_track(_cfg(b, s)) for b in BACKENDS for s in SEEDS}


def _tangent(cl):
    d = np.roll(cl, -1, axis=0) - np.roll(cl, 1, axis=0)
    return d / np.linalg.norm(d, axis=1, keepdims=True)


def _segments_intersect(cl):
    """True if any two non-adjacent segments of the closed polyline cross."""
    a, b = cl, np.roll(cl, -1, axis=0)
    n = len(cl)
    d = b - a

    def cross(u, v):
        return u[..., 0] * v[..., 1] - u[..., 1] * v[..., 0]
    for i in range(n):
        j = np.arange(i + 2, n if i > 0 else n - 1)
        if len(j) == 0:
            continue
        r = a[j] - a[i]
        t = cross(r, d[j]) / np.where(cross(d[i], d[j]) == 0, 1e-12, cross(d[i], d[j]))
        u = cross(r, d[i]) / np.where(cross(d[i], d[j]) == 0, 1e-12, cross(d[i], d[j]))
        if np.any((t > 0) & (t < 1) & (u > 0) & (u < 1)):
            return True
    return False


@pytest.mark.parametrize("backend", BACKENDS)
@pytest.mark.parametrize("seed", SEEDS)
def test_track_valid(tracks, backend, seed):
    cfg = _cfg(backend, seed)
    t = tracks[(backend, seed)]
    w, spacing = cfg["track_width_m"], cfg["cone_spacing_m"]
    cl = t.centerline

    assert t.meta["backend"] == backend
    assert abs(t.length() - cfg["target_length_m"]) < 0.05 * cfg["target_length_m"]
    # closed (last != first, but neighbours are one sample apart), densely sampled
    seg = np.linalg.norm(np.roll(cl, -1, axis=0) - cl, axis=1)
    assert 0.3 < seg.min() and seg.max() < 1.2
    assert not _segments_intersect(cl)

    # curvature radius
    th = np.arctan2(*_tangent(cl).T[::-1])
    dth = (np.roll(th, -1) - np.roll(th, 1) + np.pi) % (2 * np.pi) - np.pi
    assert 1.0 / np.abs(dth / (2 * seg.mean())).max() >= 0.95 * cfg["min_corner_radius_m"]

    # driving direction: start pose follows the centerline tangent, at the start line
    x, y, yaw = t.start_pose
    assert np.allclose(cl[0], [x, y], atol=1e-6)
    assert np.cos(yaw) * _tangent(cl)[0, 0] + np.sin(yaw) * _tangent(cl)[0, 1] > 0.99

    # blue left / yellow right via cross product with the nearest centerline tangent
    tree = cKDTree(cl)
    tang = _tangent(cl)
    for name, sign in (("blue", +1), ("yellow", -1)):
        pts = t.cones[name]
        _, idx = tree.query(pts)
        rel = pts - cl[idx]
        cr = tang[idx, 0] * rel[:, 1] - tang[idx, 1] * rel[:, 0]
        assert np.all(sign * cr > 0), name
        assert np.allclose(np.abs(cr), w / 2, atol=0.35), name   # on the boundary offset

    # orange start gate
    assert len(t.cones["orange_big"]) == 4
    og = t.cones["orange_big"]
    _, idx = tree.query(og)
    cr = tang[idx, 0] * (og - cl[idx])[:, 1] - tang[idx, 1] * (og - cl[idx])[:, 0]
    assert (cr > 0).sum() == 2 and (cr < 0).sum() == 2

    # spacing / width constraints
    allc = np.vstack([t.cones[c] for c in ("blue", "yellow", "orange_big")])
    dd, _ = cKDTree(allc).query(allc, k=2)
    assert dd[:, 1].min() >= 1.0
    dcl, _ = tree.query(allc)
    assert dcl.min() >= w / 2 - 0.2
    for name in ("blue", "yellow"):
        p = t.cones[name]
        gaps = np.linalg.norm(np.roll(p, -1, axis=0) - p, axis=1)
        # consecutive cones (except across the start gate) are no further apart than ~2x spacing
        assert np.sort(gaps)[-2] <= spacing * 1.15 + 1e-6, name


@pytest.mark.parametrize("backend", BACKENDS)
def test_deterministic(backend):
    a = generate_track(_cfg(backend, 5))
    b = generate_track(_cfg(backend, 5))
    c = generate_track(_cfg(backend, 6))
    assert np.array_equal(a.centerline, b.centerline)
    for k in a.cones:
        assert np.array_equal(a.cones[k], b.cones[k])
    assert not (a.centerline.shape == c.centerline.shape and np.allclose(a.centerline, c.centerline))


def test_auto_and_random_seed(tmp_path):
    cfg = _cfg("auto", None)
    t = generate_track(cfg)
    assert t.seed is not None and t.meta["backend"] in BACKENDS
    p = tmp_path / "t.yaml"
    t.save(p)
    t2 = Track.load(p)
    assert t2.cones["blue"].shape == t.cones["blue"].shape


def test_unknown_backend():
    with pytest.raises(ValueError):
        generate_track(_cfg("nope", 1))
