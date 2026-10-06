#!/usr/bin/env python3
"""Unit tests for offline dynamics simulation."""

import os
import sys
import numpy as np

# Add chrono DLL path
dll_path = r"C:\Users\daksh\miniforge3\envs\chrono\Library\bin"
if os.path.exists(dll_path):
    os.add_dll_directory(dll_path)
os.environ['PATH'] = dll_path + ";" + os.environ.get('PATH', '')

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from run_offline import quat_to_euler
from speed_profile import compute_speed_profile


def test_quat_to_euler():
    """Test quaternion to Euler angle conversion."""
    print("Test: Quaternion to Euler conversion")

    # Test 1: Identity quaternion [1, 0, 0, 0] -> [0, 0, 0]
    roll, pitch, yaw = quat_to_euler([1, 0, 0, 0])
    assert abs(roll) < 1e-6, f"Identity roll should be 0, got {roll}"
    assert abs(pitch) < 1e-6, f"Identity pitch should be 0, got {pitch}"
    assert abs(yaw) < 1e-6, f"Identity yaw should be 0, got {yaw}"
    print("  ✓ Identity quaternion")

    # Test 2: Roll 90° = [cos(45°), sin(45°), 0, 0]
    roll, pitch, yaw = quat_to_euler([np.cos(np.pi/4), np.sin(np.pi/4), 0, 0])
    assert abs(roll - np.pi/2) < 0.01, f"90° roll should give π/2, got {roll}"
    print("  ✓ Roll 90°")

    # Test 3: Yaw 90° = [cos(45°), 0, 0, sin(45°)]
    roll, pitch, yaw = quat_to_euler([np.cos(np.pi/4), 0, 0, np.sin(np.pi/4)])
    assert abs(yaw - np.pi/2) < 0.01, f"90° yaw should give π/2, got {yaw}"
    print("  ✓ Yaw 90°")

    print("  PASSED\n")


def test_speed_profile():
    """Test speed profile generation."""
    print("Test: Speed profile computation")

    # Create a simple circular track
    n_points = 100
    radius = 50.0
    theta = np.linspace(0, 2*np.pi, n_points)
    centerline = np.column_stack([radius*np.cos(theta), radius*np.sin(theta)])

    profile = compute_speed_profile(centerline, v_max=10.0, a_lat=8.0)

    assert len(profile['s']) == n_points, "Arc length array size mismatch"
    assert len(profile['v_final']) == n_points, "Speed profile size mismatch"
    assert np.all(profile['v_final'] >= 0), "Speed should be non-negative"
    assert np.all(profile['v_final'] <= 10.0), "Speed should not exceed v_max"
    print("  ✓ Profile generation")

    # Check that speed is lower at higher curvature regions
    curvature_low_idx = np.argmin(profile['kappa'])
    curvature_high_idx = np.argmax(profile['kappa'])
    assert profile['v_final'][curvature_low_idx] >= profile['v_final'][curvature_high_idx], \
        "Speed should be higher in low-curvature regions"
    print("  ✓ Curvature-speed relationship")

    print("  PASSED\n")


def test_smoke_run():
    """Smoke test: one lap of a small circular track, then sanity-check the CSV it writes."""
    print("Test: short end-to-end run (1 lap, 15 m radius circle)")

    import tempfile
    import yaml
    tmp = tempfile.gettempdir()
    test_track_yaml = os.path.join(tmp, 'cvsim_test_track.yaml')
    out_csv = os.path.join(tmp, 'cvsim_test_chrono.csv')

    radius = 15.0
    n = 120
    ang = np.linspace(0, 2 * np.pi, n, endpoint=False)
    # circle counter-clockwise starting at (0, 0) heading +y... shift so the start is at the origin heading +x
    cx = radius * np.sin(ang)
    cy = radius * (1 - np.cos(ang))
    track_data = {
        'seed': 0,
        'start_pose': [0.0, 0.0, 0.0],
        'meta': {'length_m': float(2 * np.pi * radius)},
        'centerline': [[float(a), float(b)] for a, b in zip(cx, cy)],
    }
    with open(test_track_yaml, 'w') as f:
        yaml.dump(track_data, f)

    from run_offline import run_simulation

    result = run_simulation(
        track_yaml=test_track_yaml,
        vehicle_name='sedan',
        laps=1,
        v_target=4.0,
        out_csv=out_csv,
        dt=0.001,
        settle_time=1.0,
    )
    assert result['num_rows'] > 1000, f"too few rows: {result['num_rows']}"
    assert result['lap_time'] > 5.0, f"run too short: {result['lap_time']} s"
    print(f"  ✓ Logged {result['num_rows']} rows over {result['lap_time']:.1f} s")

    header_lines = [l for l in open(out_csv) if l.startswith('#')]
    cols = [c.strip() for c in header_lines[-1].lstrip('# ').split(',')]
    data = np.loadtxt(out_csv, delimiter=',', comments='#')
    assert cols == ['t', 'x', 'y', 'z', 'roll', 'pitch', 'yaw', 'v', 'yaw_rate', 'ax', 'ay', 'az', 'wx', 'wy', 'wz'], cols
    assert np.all(np.isfinite(data)), "NaN/inf in output"
    d = dict(zip(cols, data.T))
    assert abs(d['z']).max() < 0.1, "z should be relative to rest"
    assert 9.0 < np.median(d['az']) < 10.5, "az should read gravity"
    assert d['ay'].max() > 1.0, "a left-hand circle must show positive lateral specific force"
    print("  ✓ CSV columns, finite values, z relative, az ~ g, ay > 0 in a left-hand turn")

    print("  PASSED\n")


if __name__ == '__main__':
    print("=" * 60)
    print("Offline Dynamics Tests")
    print("=" * 60 + "\n")

    try:
        test_quat_to_euler()
        test_speed_profile()
        test_smoke_run()

        print("=" * 60)
        print("ALL TESTS PASSED")
        print("=" * 60)
    except AssertionError as e:
        print(f"\nFAILED: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"\nERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
