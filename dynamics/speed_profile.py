"""Curvature-limited speed profile for track following."""

import numpy as np


def compute_speed_profile(centerline_points, v_max=10.0, a_lat=8.0, a_accel=4.0, a_brake=6.0):
    """
    Compute a speed profile limited by lateral acceleration and track curvature.

    Uses the formula: v(s) = min(v_max, sqrt(a_lat / |kappa|)) with forward/backward passes
    to limit acceleration and braking.

    Args:
        centerline_points: (N, 2) array of [x, y] track coordinates
        v_max: maximum speed (m/s)
        a_lat: max lateral acceleration (m/s^2)
        a_accel: max forward acceleration (m/s^2)
        a_brake: max braking deceleration (m/s^2)

    Returns:
        dict: {
            's': array of cumulative distance (m),
            'kappa': array of curvature (1/m),
            'v_max_curv': speed limited by curvature (m/s),
            'v_final': final speed profile after accel/brake limits (m/s)
        }
    """
    points = np.array(centerline_points, dtype=np.float64)
    n_points = len(points)

    # Compute arc length and derivatives
    dx = np.diff(points[:, 0])
    dy = np.diff(points[:, 1])
    ds = np.sqrt(dx**2 + dy**2)
    s = np.concatenate([[0], np.cumsum(ds)])  # cumulative distance

    # Compute curvature using numerical differentiation
    # kappa = |d(theta)/ds|
    # theta has one value per SEGMENT (n_points-1 of them); theta_diff is the heading change
    # between consecutive segments, so it has one value per INTERIOR vertex (n_points-2).
    # Normalise each by the two segments meeting at that vertex, then pad both ends by
    # repeating the nearest value so kappa has one value per POINT (n_points), matching s.
    theta = np.arctan2(dy, dx)
    theta_diff = np.diff(theta)
    theta_diff = np.where(np.abs(theta_diff) > np.pi,
                          theta_diff - np.sign(theta_diff) * 2*np.pi,
                          theta_diff)

    ds_pair = (ds[:-1] + ds[1:]) / 2  # n_points-2, matches theta_diff
    kappa_interior = np.abs(theta_diff) / np.maximum(ds_pair, 1e-9)
    kappa = np.concatenate([kappa_interior[:1], kappa_interior, kappa_interior[-1:]])  # n_points

    # Clamp curvature to avoid division issues
    kappa = np.maximum(kappa, 1e-6)

    # Speed limited by curvature
    v_max_curv = np.minimum(v_max, np.sqrt(a_lat / kappa))

    # Forward pass: limit by acceleration
    v_forward = np.zeros(n_points)
    v_forward[0] = 0.1  # Start with small speed
    for i in range(1, n_points):
        # Maximum speed increase given acceleration limit
        ds_i = s[i] - s[i-1]
        if ds_i > 0:
            v_max_accel = np.sqrt(v_forward[i-1]**2 + 2*a_accel*ds_i)
        else:
            v_max_accel = v_forward[i-1]
        v_forward[i] = min(v_max_curv[i], v_max_accel)

    # Backward pass: limit by braking
    v_final = v_forward.copy()
    for i in range(n_points-2, -1, -1):
        ds_i = s[i+1] - s[i]
        if ds_i > 0:
            v_max_brake = np.sqrt(v_final[i+1]**2 + 2*a_brake*ds_i)
        else:
            v_max_brake = v_final[i+1]
        v_final[i] = min(v_final[i], v_max_brake)

    return {
        's': s,
        'kappa': kappa,
        'v_max_curv': v_max_curv,
        'v_final': v_final,
    }


def get_target_speed(profile, distance):
    """
    Get target speed at a given distance along the track.

    Args:
        profile: dict from compute_speed_profile()
        distance: distance along track (m)

    Returns:
        float: target speed (m/s)
    """
    s = profile['s']
    v = profile['v_final']

    # Handle wrap-around for closed loop
    total_distance = s[-1]
    distance = distance % total_distance if total_distance > 0 else 0

    # Linear interpolation
    idx = np.searchsorted(s, distance, side='right') - 1
    idx = np.clip(idx, 0, len(s)-2)

    # Interpolate
    t = (distance - s[idx]) / (s[idx+1] - s[idx]) if s[idx+1] > s[idx] else 0
    return float(v[idx] * (1-t) + v[idx+1] * t)
