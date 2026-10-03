#!/usr/bin/env python3
"""Interactive MuJoCo 3D viewer: the car drives the generated track in real time (no ROS).

Mouse: left-drag rotate, right-drag pan, scroll zoom. Double-click a body to select it,
Ctrl+right-drag... see MuJoCo viewer help (F1). Use the Camera dropdown (left panel) to switch
to the car's front camera.
"""
import argparse
import time

import mujoco
import mujoco.viewer

from fs_mono_cam_sim.car.camera_geometry import car_mocap_pose
from fs_mono_cam_sim.car.driver import CenterlineDriver
from fs_mono_cam_sim.config import CONFIG_DIR, load_all
from fs_mono_cam_sim.scene.builder import build_scene_xml
from fs_mono_cam_sim.track.generator import generate_track


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config-dir", default=str(CONFIG_DIR))
    ap.add_argument("--track-seed", type=int, default=None, help="override track.yaml seed")
    ap.add_argument("--speed", type=float, default=1.0, help="playback speed factor")
    args = ap.parse_args()

    cfg = load_all(args.config_dir)
    if args.track_seed is not None:
        cfg["track"]["seed"] = args.track_seed
    track = generate_track(cfg["track"])
    model = mujoco.MjModel.from_xml_string(build_scene_xml(track, cfg))
    data = mujoco.MjData(model)
    driver = CenterlineDriver(track, cfg["car"])
    car = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "car")
    print(f"track seed={track.seed} length={track.length():.0f} m; close the window to quit")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING     # chase cam following the car
        viewer.cam.trackbodyid = car
        viewer.cam.distance, viewer.cam.elevation, viewer.cam.azimuth = 9.0, -20.0, 180.0
        t0 = time.perf_counter()
        while viewer.is_running():
            t = (time.perf_counter() - t0) * args.speed
            pos, quat = car_mocap_pose(driver.state_at(t))
            data.mocap_pos[0], data.mocap_quat[0] = pos, quat
            mujoco.mj_forward(model, data)
            viewer.sync()
            time.sleep(1 / 60)


if __name__ == "__main__":
    main()
