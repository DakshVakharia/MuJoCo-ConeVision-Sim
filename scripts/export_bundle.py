#!/usr/bin/env python3
"""Export everything the C++ runtime node needs into one self-contained folder.

generated/bundle/
  scene.xml, meshes/, textures/   MuJoCo scene (relative paths, loads from its own folder)
  trajectory.csv                  one lap of car poses vs sim time (1 kHz), see header
  track.yaml                      the generated track (for reference / other tools)
  config/*.yaml                   copy of the configs used (the C++ node reads these)
  golden/                         Python reference frames for the C++ golden test (--golden)
    frame_<k>.json                car state + noise-free detections
    frame_<k>.png                 RGB (visual reference only)

By default, uses the kinematic CenterlineDriver with 6-column trajectory.csv.
With --tilt-model, includes pitch/roll/IMU data in the CSV (extended columns).
With --trajectory PATH, uses an external trajectory CSV (must validate and copy).
"""
import argparse
import json
import math
import shutil
from pathlib import Path

import numpy as np

from conevision_sim.car.driver import CenterlineDriver
from conevision_sim.car.trajectory_io import (
    read_trajectory_csv, write_trajectory_csv, kinematic_columns, TrajectoryDriver
)
from conevision_sim.config import CONFIG_DIR, GENERATED_DIR, load_all
from conevision_sim.scene.builder import write_scene
from conevision_sim.track.generator import generate_track

TRAJ_RATE_HZ = 1000.0


def write_trajectory(driver, path, tilt_cfg=None, source="kinematic"):
    """Generate and write trajectory.csv.

    If tilt_cfg is provided and enabled, writes full tilt-model columns.
    Otherwise writes the 6-column kinematic CSV (t,x,y,yaw,v,yaw_rate).
    """
    if tilt_cfg and tilt_cfg.get("enabled"):
        # Use kinematic_columns with tilt model
        columns = kinematic_columns(driver, rate_hz=TRAJ_RATE_HZ, tilt=tilt_cfg)
        source_desc = f"{source} (tilt_model enabled)"
    else:
        # Legacy 6-column format
        columns = kinematic_columns(driver, rate_hz=TRAJ_RATE_HZ, tilt=None)
        # Only keep the 6 legacy columns
        columns = {k: v for k, v in columns.items() if k in ["t", "x", "y", "yaw", "v", "yaw_rate"]}
        source_desc = source

    lap = driver.lap_time
    write_trajectory_csv(path, columns, lap_time=lap, loop=1, source=source_desc)
    return lap, len(columns["t"])


def write_golden(cfg, track, out_dir, frames, driver=None):
    """Noise-free Python detections for the given frame indices (reference for the C++ port)."""
    import cv2
    from conevision_sim.render.sim import Simulation

    cfg = {**cfg, "perception": {**cfg["perception"], "bbox": {
        **cfg["perception"]["bbox"], "center_jitter_px": 0.0, "size_jitter_frac": 0.0,
        "dropout_prob": 0.0}}}
    out_dir.mkdir(parents=True, exist_ok=True)
    sim = Simulation(cfg, track=track, driver=driver)
    try:
        for k in frames:
            f = sim.frame_at_index(k)
            st = f.car_state
            json.dump({
                "k": k, "t": f.t,
                "car_state": {
                    "x": st.x, "y": st.y, "z": st.z,
                    "yaw": st.yaw, "pitch": st.pitch, "roll": st.roll,
                    "v": st.v, "yaw_rate": st.yaw_rate,
                    "ax": st.ax, "ay": st.ay, "az": st.az,
                    "wx": st.wx, "wy": st.wy, "wz": st.wz,
                },
                "detections": [{"cls": d.cls, "index": d.index, "x_min": d.x_min, "y_min": d.y_min,
                                "x_max": d.x_max, "y_max": d.y_max,
                                "visible_pixels": d.visible_pixels, "range_m": d.range_m}
                               for d in f.detections],
            }, open(out_dir / f"frame_{k:05d}.json", "w"), indent=1)
            cv2.imwrite(str(out_dir / f"frame_{k:05d}.png"), f.rgb[..., ::-1])
    finally:
        sim.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config-dir", default=str(CONFIG_DIR))
    ap.add_argument("--out", default=str(GENERATED_DIR / "bundle"))
    ap.add_argument("--track-seed", type=int, default=None, help="override track.yaml seed")
    ap.add_argument("--trajectory", type=str, default=None,
                    help="use external trajectory CSV instead of kinematic CenterlineDriver")
    ap.add_argument("--tilt-model", action="store_true",
                    help="enable tilt_model in output (adds pitch/roll/IMU columns to trajectory.csv)")
    ap.add_argument("--golden", action="store_true", help="also write Python reference frames")
    ap.add_argument("--golden-frames", default="0,30,60,90,120,240,480")
    args = ap.parse_args()

    cfg = load_all(args.config_dir)
    if args.track_seed is not None:
        cfg["track"]["seed"] = args.track_seed

    # Force tilt_model.enabled if --tilt-model is given
    if args.tilt_model:
        if "tilt_model" not in cfg["car"]:
            cfg["car"]["tilt_model"] = {}
        cfg["car"]["tilt_model"]["enabled"] = True

    out = Path(args.out)
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True)

    track = generate_track(cfg["track"])
    track.save(out / "track.yaml")
    write_scene(track, cfg, out / "scene.xml")

    # Determine which driver and trajectory to use
    if args.trajectory:
        # Load external trajectory CSV, validate it
        print(f"Loading trajectory from {args.trajectory}...")
        traj_data = read_trajectory_csv(args.trajectory)
        driver = TrajectoryDriver(traj_data, car_cfg=cfg["car"])
        lap, n = driver.lap_time, len(driver.columns["t"])
        source_name = Path(args.trajectory).stem
        # Copy the trajectory CSV into the bundle
        shutil.copy(args.trajectory, out / "trajectory.csv")
        print(f"Copied trajectory.csv from {args.trajectory} (source={traj_data.source})")
    else:
        # Generate kinematic trajectory
        driver = CenterlineDriver(track, cfg["car"])
        lap, n = write_trajectory(driver, out / "trajectory.csv",
                                   tilt_cfg=cfg["car"].get("tilt_model"),
                                   source="kinematic")
        source_name = "kinematic"

    shutil.copytree(args.config_dir, out / "config")
    print(f"track seed={track.seed} length={track.length():.0f} m, lap {lap:.1f} s ({n} poses)")
    print(f"trajectory source={source_name}")

    if args.golden:
        write_golden(cfg, track, out / "golden", [int(k) for k in args.golden_frames.split(",")],
                     driver=driver)
        print("golden frames:", args.golden_frames)
    print("wrote", out)


if __name__ == "__main__":
    main()
