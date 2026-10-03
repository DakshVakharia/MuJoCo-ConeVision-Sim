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

Body pitch/roll oscillation is NOT in the CSV (it is not lap-periodic); the C++ side recomputes it
from car.yaml body_motion exactly as car/driver.py does:
    pitch = A_p * sin(2*pi*f*t),  roll = A_r * sin(1.31*2*pi*f*t + 0.9)
"""
import argparse
import json
import math
import shutil
from pathlib import Path

import numpy as np

from fs_mono_cam_sim.car.driver import CenterlineDriver
from fs_mono_cam_sim.config import CONFIG_DIR, GENERATED_DIR, load_all
from fs_mono_cam_sim.scene.builder import write_scene
from fs_mono_cam_sim.track.generator import generate_track

TRAJ_RATE_HZ = 1000.0


def write_trajectory(driver, path):
    """Rows t,x,y,yaw,v,yaw_rate for t in [0, lap_time]; yaw is unwrapped so the C++ side can
    interpolate linearly. The last row (t = lap_time) equals the first row with yaw + n*2*pi."""
    lap = driver.lap_time
    n = int(math.ceil(lap * TRAJ_RATE_HZ))
    ts = np.linspace(0.0, lap, n + 1)
    rows = []
    for t in ts:
        st = driver.state_at(float(t))
        rows.append((t, st.x, st.y, st.yaw, st.v, st.yaw_rate))
    a = np.array(rows)
    a[:, 3] = np.unwrap(a[:, 3])
    header = (f"lap_time={lap:.9f}\n"
              "one lap of car base_link poses in the world frame; pitch/roll come from car.yaml\n"
              "t,x,y,yaw,v,yaw_rate")
    np.savetxt(path, a, delimiter=",", fmt="%.9f", header=header, comments="# ")
    return lap, len(a)


def write_golden(cfg, track, out_dir, frames):
    """Noise-free Python detections for the given frame indices (reference for the C++ port)."""
    import cv2
    from fs_mono_cam_sim.render.sim import Simulation

    cfg = {**cfg, "perception": {**cfg["perception"], "bbox": {
        **cfg["perception"]["bbox"], "center_jitter_px": 0.0, "size_jitter_frac": 0.0,
        "dropout_prob": 0.0}}}
    out_dir.mkdir(parents=True, exist_ok=True)
    sim = Simulation(cfg, track=track)
    try:
        for k in frames:
            f = sim.frame_at_index(k)
            st = f.car_state
            json.dump({
                "k": k, "t": f.t,
                "car_state": {"x": st.x, "y": st.y, "yaw": st.yaw, "pitch": st.pitch, "roll": st.roll},
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
    ap.add_argument("--golden", action="store_true", help="also write Python reference frames")
    ap.add_argument("--golden-frames", default="0,30,60,90,120,240,480")
    args = ap.parse_args()

    cfg = load_all(args.config_dir)
    if args.track_seed is not None:
        cfg["track"]["seed"] = args.track_seed
    out = Path(args.out)
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True)

    track = generate_track(cfg["track"])
    track.save(out / "track.yaml")
    write_scene(track, cfg, out / "scene.xml")
    lap, n = write_trajectory(CenterlineDriver(track, cfg["car"]), out / "trajectory.csv")
    shutil.copytree(args.config_dir, out / "config")
    print(f"track seed={track.seed} length={track.length():.0f} m, lap {lap:.1f} s ({n} poses)")
    if args.golden:
        write_golden(cfg, track, out / "golden", [int(k) for k in args.golden_frames.split(",")])
        print("golden frames:", args.golden_frames)
    print("wrote", out)


if __name__ == "__main__":
    main()
