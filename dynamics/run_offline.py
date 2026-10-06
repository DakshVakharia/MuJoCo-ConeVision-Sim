#!/usr/bin/env python3
"""
Chrono vehicle dynamics simulator: CORRECTED IMU calculation.
Fixed acceleration computation using velocity derivatives and proper rotation transforms.
"""

import os
import sys
import argparse
import numpy as np
import yaml
from scipy import signal

# FIX: Add chrono Library\bin to DLL search path
dll_path = r"C:\Users\daksh\miniforge3\envs\chrono\Library\bin"
if os.path.exists(dll_path):
    os.add_dll_directory(dll_path)
os.environ['PATH'] = dll_path + ";" + os.environ.get('PATH', '')

import pychrono.core as chrono
import pychrono.vehicle as veh

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from vehicle import build_vehicle


def quat_to_euler(quat_wxyz):
    """Convert quaternion [w, x, y, z] to Euler angles [roll, pitch, yaw]."""
    w, x, y, z = quat_wxyz[0], quat_wxyz[1], quat_wxyz[2], quat_wxyz[3]

    sinr_cosp = 2 * (w*x + y*z)
    cosr_cosp = 1 - 2*(x**2 + y**2)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w*y - z*x)
    sinp = np.clip(sinp, -1, 1)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w*z + x*y)
    cosy_cosp = 1 - 2*(y**2 + z**2)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def quat_to_rotation_matrix(quat_wxyz):
    """Convert quaternion [w, x, y, z] to 3x3 rotation matrix (body to world, the standard active rotation)."""
    w, x, y, z = quat_wxyz[0], quat_wxyz[1], quat_wxyz[2], quat_wxyz[3]

    norm = np.sqrt(w**2 + x**2 + y**2 + z**2)
    if norm > 0:
        w, x, y, z = w/norm, x/norm, y/norm, z/norm

    R = np.array([
        [1 - 2*(y**2 + z**2),     2*(x*y - w*z),     2*(x*z + w*y)],
        [    2*(x*y + w*z), 1 - 2*(x**2 + z**2),     2*(y*z - w*x)],
        [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x**2 + y**2)],
    ])

    return R


def build_closed_path(centerline_points):
    """Build closed-loop Bezier path."""
    points_3d = []
    for x, y in centerline_points:
        points_3d.append(chrono.ChVector3d(float(x), float(y), 0.0))
    return chrono.ChBezierCurve(points_3d + points_3d + points_3d)  # 3x for long path


def build_path_follower_driver(vehicle, centerline_points):
    """Build path-follower driver with tuned Sedan gains."""
    path = build_closed_path(centerline_points)
    driver = veh.ChPathFollowerDriver(vehicle, path, "centerline", 5.0)

    driver.GetSteeringController().SetLookAheadDistance(3.5)
    driver.GetSteeringController().SetGains(1.0, 0.0, 0.5)
    driver.GetSpeedController().SetGains(0.5, 0.1, 0.0)

    driver.Initialize()
    return driver


def run_simulation(track_yaml, vehicle_name="sedan", laps=2, v_target=8.0,
                   out_csv=None, dt=0.001, settle_time=7.0,
                   profile=False, a_lat=7.0, a_accel=4.0, a_brake=6.0):
    """Run vehicle dynamics with CORRECTED IMU calculation."""

    print(f"Loading track from {track_yaml}")
    with open(track_yaml) as f:
        track_data = yaml.safe_load(f)

    centerline = np.array(track_data['centerline'])
    start_pose = track_data['start_pose']

    # Compute track length
    if len(centerline) > 1:
        diffs = np.diff(centerline, axis=0)
        distances = np.linalg.norm(diffs, axis=1)
        track_length = float(np.sum(distances))
    else:
        track_length = 300.0

    print(f"Track: {len(centerline)} points, ~{track_length:.1f}m")

    # Build vehicle
    print(f"Building {vehicle_name} vehicle...")
    temp_sys = chrono.ChSystemSMC()
    vehicle, terrain = build_vehicle(temp_sys, vehicle_name,
                                    init_pos=[start_pose[0], start_pose[1], 0.5],
                                    init_yaw=start_pose[2])
    sys_mbs = vehicle.GetSystem()
    sys_mbs.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    print(f"Building path-follower driver...")
    driver = build_path_follower_driver(vehicle, centerline)

    # SETTLE VEHICLE
    print(f"Settling vehicle...")
    settle_steps = int(settle_time / dt)
    settle_data = {'z': [], 'roll': [], 'pitch': []}

    for i in range(settle_steps):
        inputs = veh.DriverInputs()
        inputs.m_steering = 0.0
        inputs.m_throttle = 0.0
        inputs.m_braking = 0.0

        driver.Synchronize(sys_mbs.GetChTime())
        vehicle.Synchronize(sys_mbs.GetChTime(), inputs, terrain)
        terrain.Synchronize(sys_mbs.GetChTime())

        driver.Advance(dt)
        vehicle.Advance(dt)
        terrain.Advance(dt)

        chassis = vehicle.GetChassis()
        pos = chassis.GetPos()
        quat = chassis.GetRot()
        speed = np.linalg.norm([chassis.GetLinearVelocity()[j] for j in range(3)])
        roll, pitch, yaw = quat_to_euler(quat)

        settle_data['z'].append(pos[2])
        settle_data['roll'].append(roll)
        settle_data['pitch'].append(pitch)

        if speed < 1e-3 and i > 1000:
            print(f"  Settled at step {i}")
            break

    # At-rest reference (last 0.5s average)
    n_avg = max(1, int(0.5 / dt))
    at_rest_z = np.mean(settle_data['z'][-n_avg:])
    at_rest_roll = np.mean(settle_data['roll'][-n_avg:])
    at_rest_pitch = np.mean(settle_data['pitch'][-n_avg:])

    print(f"At-rest pose: z={at_rest_z:.4f}m, roll={np.degrees(at_rest_roll):.2f}°, pitch={np.degrees(at_rest_pitch):.2f}°")

    # LOW-PASS FILTER setup
    fs = 1.0 / dt
    nyquist = fs / 2
    cutoff = 20
    sos = signal.butter(2, cutoff/nyquist, output='sos')

    # Optional corner-limited speed profile: fast on straights, slow for corners.
    # v(s) = min(v_target, sqrt(a_lat / curvature)), then limited by acceleration and braking.
    v_loop = None
    if profile:
        from scipy.spatial import cKDTree
        from speed_profile import compute_speed_profile
        n0 = len(centerline)
        prof = compute_speed_profile(np.vstack([centerline] * 3), v_max=v_target,
                                     a_lat=a_lat, a_accel=a_accel, a_brake=a_brake)
        v_loop = prof['v_final'][n0:2 * n0]      # the middle copy of 3 tiled laps is the steady-state lap
        tree = cKDTree(centerline)
        ds_pt = track_length / n0
        print(f"Speed profile: a_lat={a_lat} m/s^2 -> target speeds {v_loop.min():.1f}..{v_loop.max():.1f} m/s")
    avg_speed = 0.55 * v_target if profile else v_target

    # Simulate laps - COLLECT DATA ONLY
    max_sim_time = (track_length / avg_speed) * laps * 1.5 + 10.0
    max_steps = int(max_sim_time / dt)

    print(f"Simulating {laps} laps...")

    # Storage arrays
    t_array = []
    x_array = []
    y_array = []
    z_array = []
    roll_array = []
    pitch_array = []
    yaw_array = []
    v_array = []
    yaw_rate_array = []
    vel_world = []  # COLLECT velocity in world frame
    rot_list = []   # COLLECT rotation matrices
    wx_array = []
    wy_array = []
    wz_array = []

    distance_driven = 0.0
    prev_pos = np.array([start_pose[0], start_pose[1]])
    speed_now = 0.0

    for i in range(max_steps):
        t = i * dt

        if v_loop is not None:
            _, idx = tree.query(prev_pos)
            lead = int(0.3 * max(speed_now, 1.0) / ds_pt)           # look 0.3 s ahead so braking starts on time
            desired_speed = v_loop[(idx + lead) % len(v_loop)]
            desired_speed = min(desired_speed, 1.0 + a_accel * t)    # gentle start from standstill
            remaining = track_length * laps - distance_driven
            desired_speed = min(desired_speed, np.sqrt(2.0 * a_brake * max(remaining, 0.0)) + 0.3)  # stop at the end
        # Simple speed ramp (constant speed)
        elif t < 10.0:
            desired_speed = v_target * (t / 10.0)
        elif distance_driven > track_length * laps - (v_target * 10.0):
            desired_speed = v_target * max(0.1, (track_length * laps - distance_driven) / (v_target * 10.0))
        else:
            desired_speed = v_target

        driver.SetDesiredSpeed(desired_speed)

        driver_inputs = driver.GetInputs()
        driver.Synchronize(t)
        vehicle.Synchronize(t, driver_inputs, terrain)
        terrain.Synchronize(t)

        driver.Advance(dt)
        vehicle.Advance(dt)
        terrain.Advance(dt)

        chassis = vehicle.GetChassis()
        pos = chassis.GetPos()
        quat = chassis.GetRot()
        lin_vel = chassis.GetLinearVelocity()
        speed_now = float(np.linalg.norm([lin_vel[0], lin_vel[1], lin_vel[2]]))

        curr_pos = np.array([pos[0], pos[1]])
        ds = np.linalg.norm(curr_pos - prev_pos)
        distance_driven += ds
        prev_pos = curr_pos.copy()

        # Stop when laps completed
        if distance_driven > track_length * laps:
            print(f"  Completed {laps} laps at step {i}")
            break

        # Orientation
        roll, pitch, yaw = quat_to_euler(quat)
        roll_rel = roll - at_rest_roll
        pitch_rel = pitch - at_rest_pitch

        speed = np.linalg.norm(lin_vel)

        # COLLECT data (NOT compute f_body yet)
        t_array.append(t)
        x_array.append(pos[0])
        y_array.append(pos[1])
        z_array.append(pos[2] - at_rest_z)
        roll_array.append(roll_rel)
        pitch_array.append(pitch_rel)
        yaw_array.append(yaw)
        v_array.append(speed)

        # COLLECT velocity and rotation (for post-loop acceleration computation)
        vel_world.append([lin_vel[0], lin_vel[1], lin_vel[2]])
        R_body_to_world = quat_to_rotation_matrix(quat)
        rot_list.append(R_body_to_world)

        # Angular velocity
        try:
            ang_vel_body = chassis.GetAngVelLocal()
        except:
            R_world_to_body = R_body_to_world.T
            ang_vel_parent = np.array([chassis.GetAngularVelocity()[j] for j in range(3)])
            ang_vel_body = R_world_to_body @ ang_vel_parent

        yaw_rate_array.append(ang_vel_body[2])
        wx_array.append(ang_vel_body[0])
        wy_array.append(ang_vel_body[1])
        wz_array.append(ang_vel_body[2])

        if (i+1) % 10000 == 0:
            print(f"  Step {i+1}: t={t:.1f}s, distance={distance_driven:.1f}m")

    # POST-LOOP: Compute acceleration from velocity derivatives
    print(f"Computing acceleration from velocity derivatives...")
    vel = np.asarray(vel_world)
    a_world = np.gradient(vel, dt, axis=0)

    # Apply low-pass filter
    a_world = signal.sosfiltfilt(sos, a_world, axis=0)

    # Add gravity
    a_world[:, 2] += 9.81

    # Transform to body frame: f_body = R^T @ (a_world + g).
    # rot_list holds R = body -> world (the standard quaternion matrix; despite its docstring,
    # quat_to_rotation_matrix is NOT world -> body). einsum 'nji,nj->ni' computes out_i = sum_j R_ji a_j,
    # i.e. R^T a: pass R as is. (Transposing R first would apply R, not R^T: wrong.)
    rot_array = np.asarray(rot_list)
    f_body = np.einsum('nji,nj->ni', rot_array, a_world)

    ax_array = f_body[:, 0]
    ay_array = f_body[:, 1]
    az_array = f_body[:, 2]

    # Write CSV
    if out_csv:
        print(f"Writing CSV to {out_csv}")
        os.makedirs(os.path.dirname(out_csv) or '.', exist_ok=True)

        # lap_time is the DURATION of the rows, not distance
        lap_time = t_array[-1] if len(t_array) > 0 else 0.0

        with open(out_csv, 'w') as f:
            f.write(f"# lap_time={lap_time:.9f}\n")
            f.write("# loop=0\n")
            f.write("# source=chrono\n")
            f.write("# t,x,y,z,roll,pitch,yaw,v,yaw_rate,ax,ay,az,wx,wy,wz\n")

            for i in range(len(t_array)):
                f.write(f"{t_array[i]:.9f},"
                       f"{x_array[i]:.9f},"
                       f"{y_array[i]:.9f},"
                       f"{z_array[i]:.9f},"
                       f"{roll_array[i]:.9f},"
                       f"{pitch_array[i]:.9f},"
                       f"{yaw_array[i]:.9f},"
                       f"{v_array[i]:.9f},"
                       f"{yaw_rate_array[i]:.9f},"
                       f"{ax_array[i]:.9f},"
                       f"{ay_array[i]:.9f},"
                       f"{az_array[i]:.9f},"
                       f"{wx_array[i]:.9f},"
                       f"{wy_array[i]:.9f},"
                       f"{wz_array[i]:.9f}\n")

        print(f"  Wrote {len(t_array)} rows")

        # Verify correlations
        print(f"\nVerifying IMU correlations...")
        # Compute filtered dv/dt for correlation check
        dv_dt = np.gradient(np.linalg.norm(vel, axis=1), dt)
        dv_dt_filtered = signal.sosfiltfilt(sos, dv_dt)

        # Compute v*yaw_rate for lateral acceleration check
        v_mag = np.linalg.norm(vel, axis=1)
        v_yaw_rate = v_mag * np.array(yaw_rate_array)

        # Correlations
        corr_ax = np.corrcoef(ax_array, dv_dt_filtered)[0, 1]
        corr_ay = np.corrcoef(ay_array, v_yaw_rate)[0, 1]

        print(f"  ax vs dv/dt correlation: {corr_ax:.3f} (expect >0.9)")
        print(f"  ay vs v*yaw_rate correlation: {corr_ay:.3f} (expect >0.9)")
        print(f"  ax range: {ax_array.min():.2f}..{ax_array.max():.2f} m/s^2")
        print(f"  ay range: {ay_array.min():.2f}..{ay_array.max():.2f} m/s^2")
        print(f"  az range: {az_array.min():.2f}..{az_array.max():.2f} m/s^2")

    return {
        'lap_time': lap_time,
        'num_rows': len(t_array),
    }


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--track', default='generated/bundle/track.yaml')
    parser.add_argument('--laps', type=int, default=2)
    parser.add_argument('--vmax', type=float, default=8.0,
                        help='target speed (m/s); with --profile it is the top speed on straights')
    parser.add_argument('--profile', action='store_true',
                        help='corner-limited speed profile: fast on straights, slower for corners')
    parser.add_argument('--a-lat', type=float, default=7.0, help='lateral acceleration budget for corners (m/s^2)')
    parser.add_argument('--a-accel', type=float, default=4.0, help='max acceleration (m/s^2)')
    parser.add_argument('--a-brake', type=float, default=6.0, help='max braking (m/s^2)')
    parser.add_argument('--out', default='generated/chrono/trajectory_chrono.csv')

    args = parser.parse_args()

    result = run_simulation(
        track_yaml=args.track,
        vehicle_name='sedan',
        laps=args.laps,
        v_target=args.vmax,
        out_csv=args.out,
        profile=args.profile,
        a_lat=args.a_lat,
        a_accel=args.a_accel,
        a_brake=args.a_brake,
    )

    print(f"\nSimulation complete!")
    print(f"  Lap time: {result['lap_time']:.1f}s")
    print(f"  Rows logged: {result['num_rows']}")
