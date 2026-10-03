#!/usr/bin/env python
"""Generate a random FS track and save it as YAML plus a top-down PNG preview (OpenCV only).

  PYTHONPATH=src python scripts/generate_track.py --config config/track.yaml --seed 7 \
      --out generated/track.yaml --plot generated/track.png [--backend builtin]
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from conevision_sim.config import load_yaml  # noqa: E402
from conevision_sim.track.generator import generate_track  # noqa: E402

COLORS_BGR = {"blue": (230, 120, 20), "yellow": (0, 215, 255),
              "orange": (0, 140, 255), "orange_big": (0, 90, 255)}


def plot_track(track, path, size=1000, margin=40):
    pts = np.vstack([track.centerline] + [c for c in track.cones.values() if len(c)])
    lo, hi = pts.min(0), pts.max(0)
    scale = (size - 2 * margin) / max((hi - lo).max(), 1e-6)
    w = int((hi[0] - lo[0]) * scale + 2 * margin)
    h = int((hi[1] - lo[1]) * scale + 2 * margin)

    def px(p):   # world (x right, y up) -> image (u right, v down)
        p = np.atleast_2d(p)
        return np.stack([(p[:, 0] - lo[0]) * scale + margin,
                         h - ((p[:, 1] - lo[1]) * scale + margin)], axis=1)

    img = np.full((h, w, 3), 255, np.uint8)
    cl = px(track.centerline).astype(np.int32)
    cv2.polylines(img, [cl], True, (200, 200, 200), 1, cv2.LINE_AA)
    for name, col in COLORS_BGR.items():
        r = 6 if name == "orange_big" else 4
        for p in px(track.cones[name]):
            cv2.circle(img, (int(p[0]), int(p[1])), r, col, -1, cv2.LINE_AA)
            cv2.circle(img, (int(p[0]), int(p[1])), r, (40, 40, 40), 1, cv2.LINE_AA)
    x, y, yaw = track.start_pose
    a = px([x, y])[0]
    b = px([x + 8 * np.cos(yaw), y + 8 * np.sin(yaw)])[0]
    cv2.arrowedLine(img, tuple(int(v) for v in a), tuple(int(v) for v in b), (0, 160, 0), 2,
                    tipLength=0.3)
    m = track.meta
    txt = (f"{m.get('backend')} seed={track.seed} L={m.get('length_m', 0):.0f}m "
           f"Rmin={m.get('min_radius_m', 0):.1f}m {m.get('direction')}")
    cv2.putText(img, txt, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1, cv2.LINE_AA)
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), img)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--config", default=str(ROOT / "config" / "track.yaml"))
    ap.add_argument("--seed", type=int, default=None, help="override track.seed")
    ap.add_argument("--backend", default=None, help="override track.backend")
    ap.add_argument("--out", default=str(ROOT / "generated" / "track.yaml"))
    ap.add_argument("--plot", default=None, help="PNG preview path")
    args = ap.parse_args()

    cfg = load_yaml(args.config)
    cfg = cfg.get("track", cfg)
    if args.seed is not None:
        cfg["seed"] = args.seed
    if args.backend:
        cfg["backend"] = args.backend
    track = generate_track(cfg)
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    track.save(args.out)
    print(f"saved {args.out}: backend={track.meta['backend']} seed={track.seed} "
          f"length={track.meta['length_m']:.1f} m, blue={len(track.cones['blue'])} "
          f"yellow={len(track.cones['yellow'])} orange_big={len(track.cones['orange_big'])}")
    if args.plot:
        plot_track(track, args.plot)
        print(f"saved {args.plot}")


if __name__ == "__main__":
    main()
