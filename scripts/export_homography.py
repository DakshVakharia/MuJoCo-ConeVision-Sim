#!/usr/bin/env python
"""Export the camera's exact ground-plane homography in the YAML format used by the sibling
Monocular-Depth-perception-for-Cones repo (H maps pixel (u, v, 1) -> ground (x fwd, y left) metres).

    PYTHONPATH=src python scripts/export_homography.py [--out generated/homography.yaml]
"""
import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from conevision_sim.car.camera_geometry import ground_homography, intrinsics  # noqa: E402
from conevision_sim.config import load_config  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", default=str(ROOT / "generated" / "homography.yaml"))
    args = ap.parse_args()
    cam = load_config("camera")
    H = ground_homography(cam)
    fx, fy, cx, cy, w, h = intrinsics(cam)
    lines = [
        "# Ground-plane homography exported by MuJoCo-ConeVision-Sim (scripts/export_homography.py)",
        "# Maps image pixel (u, v, 1) -> ground (x forward, y left, 1) in metres, base_link frame",
        "# (origin on the ground under the car centre), flat ground, car level.",
        f"# Camera: {w}x{h} px, fx={fx:.4f} fy={fy:.4f} cx={cx:.4f} cy={cy:.4f}",
        f"# Mount (base_link): x={cam['x']} y={cam['y']} z={cam['z']} m, "
        f"roll={cam.get('roll_deg', 0)} pitch={cam.get('pitch_deg', 0)} (+down) "
        f"yaw={cam.get('yaw_deg', 0)} (+left) deg",
        "homography_matrix:",
    ]
    lines += [f"- [{', '.join(repr(float(v)) for v in row)}]" for row in H]
    lines += ["units_to_metres: 1.0", "swap_xy: false", "frame_origin:", "  x: 0.0", "  y: 0.0", ""]
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text("\n".join(lines), encoding="utf-8")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
