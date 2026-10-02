#!/usr/bin/env python
"""Render the car camera + fake-YOLO boxes without ROS.

Examples:
  PYTHONPATH=src python scripts/preview.py --seconds 5 --out generated/preview.mp4
  PYTHONPATH=src python scripts/preview.py --seconds 2 --show
  PYTHONPATH=src python scripts/preview.py --seconds 10 --yolo-dir generated/yolo --yolo-every 6
"""
import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from fs_mono_cam_sim.config import CONFIG_DIR, GENERATED_DIR, load_all  # noqa: E402
from fs_mono_cam_sim.render.sim import Simulation  # noqa: E402
from fs_mono_cam_sim.track.track_types import CONE_CLASSES  # noqa: E402


def draw_boxes(rgb, detections, outline_colors):
    """Returns a BGR image with boxes (class colour), label and range drawn."""
    img = np.ascontiguousarray(rgb[..., ::-1])
    for d in detections:
        r, g, b = outline_colors[d.cls]
        col = (int(b * 255), int(g * 255), int(r * 255))
        p0, p1 = (int(round(d.x_min)), int(round(d.y_min))), (int(round(d.x_max)), int(round(d.y_max)))
        cv2.rectangle(img, p0, p1, col, 2)
        label = f"{d.cls} {d.range_m:.1f}m"
        cv2.putText(img, label, (p0[0], max(12, p0[1] - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (255, 255, 255), 3, cv2.LINE_AA)
        cv2.putText(img, label, (p0[0], max(12, p0[1] - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    col, 1, cv2.LINE_AA)
    return img


def yolo_lines(detections, w, h):
    """YOLO txt: 'class cx cy w h' normalised, class id = index in CONE_CLASSES."""
    lines = []
    for d in detections:
        cx, cy = (d.x_min + d.x_max) / 2 / w, (d.y_min + d.y_max) / 2 / h
        bw, bh = (d.x_max - d.x_min) / w, (d.y_max - d.y_min) / h
        lines.append(f"{CONE_CLASSES.index(d.cls)} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}")
    return lines


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, default=5.0, help="simulated seconds to render")
    ap.add_argument("--out", default=None, help="output mp4 path (default: none)")
    ap.add_argument("--show", action="store_true", help="show a live window (q/Esc quits)")
    ap.add_argument("--config-dir", default=str(CONFIG_DIR))
    ap.add_argument("--seed", type=int, default=None, help="bbox noise seed")
    ap.add_argument("--yolo-dir", default=None,
                    help="dump images/ + labels/ (YOLO txt) + classes.txt into this folder")
    ap.add_argument("--yolo-every", type=int, default=1, help="dump every Nth frame")
    ap.add_argument("--frames-png", default=None, help="also save every 30th annotated frame here")
    args = ap.parse_args()

    cfg = load_all(args.config_dir)
    sim = Simulation(cfg, seed=args.seed)
    cam = cfg["camera"]
    fps = float(cam["fps"])
    colors = cfg["perception"]["bbox"]["outline_colors"]
    n = int(round(args.seconds * fps))

    writer = None
    if args.out:
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        writer = cv2.VideoWriter(args.out, cv2.VideoWriter_fourcc(*"mp4v"), fps,
                                 (int(cam["width"]), int(cam["height"])))
    if args.yolo_dir:
        yd = Path(args.yolo_dir)
        (yd / "images").mkdir(parents=True, exist_ok=True)
        (yd / "labels").mkdir(parents=True, exist_ok=True)
        (yd / "classes.txt").write_text("\n".join(CONE_CLASSES) + "\n")
    if args.frames_png:
        Path(args.frames_png).mkdir(parents=True, exist_ok=True)

    sums = {}
    t_wall = time.perf_counter()
    for k in range(n):
        f = sim.frame_at_index(k)
        for key, v in f.timings.items():
            sums[key] = sums.get(key, 0.0) + v
        need_img = writer or args.show or args.frames_png
        bgr = draw_boxes(f.rgb, f.detections, colors) if need_img else None
        if writer:
            writer.write(bgr)
        if args.frames_png and k % 30 == 0:
            cv2.imwrite(str(Path(args.frames_png) / f"frame_{k:05d}.png"), bgr)
        if args.yolo_dir and k % args.yolo_every == 0:
            stem = f"{k:06d}"
            cv2.imwrite(str(yd / "images" / f"{stem}.jpg"), f.rgb[..., ::-1],
                        [int(cv2.IMWRITE_JPEG_QUALITY), int(cam.get("jpeg_quality", 90))])
            (yd / "labels" / f"{stem}.txt").write_text(
                "\n".join(yolo_lines(f.detections, cam["width"], cam["height"])) + "\n")
        if args.show:
            cv2.imshow("fs camera", bgr)
            if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                n = k + 1
                break
    wall = time.perf_counter() - t_wall
    if writer:
        writer.release()
    if args.show:
        cv2.destroyAllWindows()

    print(f"{n} frames at {int(cam['width'])}x{int(cam['height'])} ({n / fps:.1f} s simulated)")
    print(f"wall time {wall:.2f}s -> {n / wall:.1f} fps overall (incl. drawing/encoding)")
    pipeline = sums.get("total", 1e-9)
    print(f"render pipeline only: {n / pipeline:.1f} fps")
    for key in ("step", "rgb", "seg", "bbox", "total"):
        if key in sums:
            print(f"  {key:6s} {1000 * sums[key] / n:7.2f} ms/frame")
    if args.out:
        print("wrote", args.out)
    if args.yolo_dir:
        print("wrote YOLO dataset to", args.yolo_dir)


if __name__ == "__main__":
    main()
