#!/usr/bin/env python3
"""Acceptance test for a Chrono trajectory CSV (docs/trajectory_contract.md).

    python dynamics/check_trajectory.py generated/chrono/trajectory_chrono.csv \
        --track generated/bundle/track.yaml --laps 2

Prints PASS / FAIL / WARN per check and exits 1 if any FAIL. Needs only numpy + pyyaml.
A trajectory is only usable by the simulator if every FAIL line passes.
"""
import argparse
import sys

import numpy as np
import yaml

REQUIRED = ["t", "x", "y", "z", "roll", "pitch", "yaw", "v", "yaw_rate",
            "ax", "ay", "az", "wx", "wy", "wz"]
G = 9.81


def load(path):
    header, names = [], None
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            if line.startswith("#"):
                header.append(line.strip("# \r\n"))
            else:
                break
    names = [c.strip() for c in header[-1].split(",")]
    data = np.loadtxt(path, delimiter=",", comments="#")
    return header, {n: data[:, i] for i, n in enumerate(names)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--track", required=True)
    ap.add_argument("--laps", type=float, required=True, help="number of laps the run should cover")
    ap.add_argument("--max-dev", type=float, default=1.5, help="max allowed distance from centreline (m)")
    a = ap.parse_args()

    results = []

    def check(level, ok, name, detail=""):
        tag = "PASS" if ok else level
        results.append((tag, name, detail))
        print(f"[{tag}] {name}  {detail}")

    header, c = load(a.csv)
    miss = [k for k in REQUIRED if k not in c]
    check("FAIL", not miss, "all 15 columns present", f"missing: {miss}" if miss else "")
    if miss:
        sys.exit(1)
    check("FAIL", any(h.replace(" ", "") == "loop=0" for h in header), "header has loop=0")
    check("FAIL", any(h.startswith("source=chrono") for h in header), "header has source=chrono")

    t = c["t"]
    check("FAIL", np.all(np.isfinite(np.column_stack([c[k] for k in REQUIRED]))), "no NaN/inf")
    check("FAIL", np.allclose(np.diff(t), 1e-3, atol=1e-6), "uniform 1 kHz time", f"dt range {np.diff(t).min():.6f}..{np.diff(t).max():.6f}")

    # At rest at the very start. A car that starts accelerating at t=0 will legitimately show
    # a small pitch onset within the first few rows, so this only checks the first 0.05 s
    # (50 rows): enough to catch a bad at-rest reference, not so long it penalises a normal launch.
    n0 = int(0.05 / 1e-3)
    check("FAIL", np.abs(c["z"][:n0]).max() < 0.02 and np.degrees(np.abs(c["roll"][:n0])).max() < 0.3
          and np.degrees(np.abs(c["pitch"][:n0])).max() < 0.5,
          "relative to at-rest pose: z, roll, pitch ~ 0 in the first 0.05 s",
          f"z {c['z'][:n0].min():.3f}..{c['z'][:n0].max():.3f} m, roll max {np.degrees(np.abs(c['roll'][:n0])).max():.2f} deg, pitch max {np.degrees(np.abs(c['pitch'][:n0])).max():.2f} deg")
    check("FAIL", np.abs(c["z"]).max() < 0.08, "z bounce small (|z| < 8 cm)", f"|z| max {np.abs(c['z']).max():.3f} m")
    rp = np.degrees(max(np.abs(c["roll"]).max(), np.abs(c["pitch"]).max()))
    check("FAIL", rp < 4.0, "roll/pitch stay below 4 deg on a flat track", f"max {rp:.2f} deg")

    # IMU specific force: gravity included, body frame
    az = c["az"]
    check("FAIL", 9.0 < np.median(az) < 10.5, "median az ~ 9.81 (gravity in specific force)", f"median {np.median(az):.2f}")
    check("FAIL", np.percentile(az, 0.5) > 5.0 and np.percentile(az, 99.5) < 14.0, "az stays physical (0.5..99.5 pct within 5..14)",
          f"{np.percentile(az, 0.5):.1f}..{np.percentile(az, 99.5):.1f}, extremes {az.min():.1f}..{az.max():.1f}")
    check("FAIL", max(np.percentile(np.abs(c['ax']), 99.5), np.percentile(np.abs(c['ay']), 99.5)) < 12.0,
          "ax, ay are physical (99.5 pct below 12 m/s^2)",
          f"ax {c['ax'].min():.1f}..{c['ax'].max():.1f}, ay {c['ay'].min():.1f}..{c['ay'].max():.1f}")
    check("FAIL", c["ax"].max() > 0.3 and c["ax"].min() < -0.3 or c["ax"].std() > 0.2,
          "ax has both signs (accelerating and braking) or real variation", f"ax std {c['ax'].std():.2f}")

    # motion
    v = c["v"]
    check("FAIL", 1.0 < v.mean() < 15.0 and v.max() < 20.0, "speed plausible", f"mean {v.mean():.2f}, max {v.max():.2f} m/s")
    vxy = np.hypot(np.gradient(c["x"], t), np.gradient(c["y"], t))
    check("FAIL", abs(vxy.mean() - v.mean()) < 0.5, "v column matches d(x,y)/dt", f"{v.mean():.2f} vs {vxy.mean():.2f}")

    # on the track
    tr = yaml.safe_load(open(a.track))
    cl = np.array(tr["centerline"], float)
    xs, ys = c["x"][::50], c["y"][::50]
    dev = np.array([np.min(np.hypot(cl[:, 0] - x, cl[:, 1] - y)) for x, y in zip(xs, ys)])
    check("FAIL", dev.max() < a.max_dev, f"stays within {a.max_dev} m of the centreline", f"max {dev.max():.2f} m, mean {dev.mean():.2f} m")
    L = np.sum(np.linalg.norm(np.diff(np.vstack([cl, cl[:1]]), axis=0), axis=1))
    driven = np.sum(np.hypot(np.diff(c["x"]), np.diff(c["y"])))
    check("FAIL", 0.9 * a.laps * L < driven < 1.15 * a.laps * L, f"drove about {a.laps:g} laps",
          f"driven {driven:.0f} m, expected {a.laps * L:.0f} m (track {L:.0f} m)")

    # tilt per g (warnings only)
    k = slice(2000, None)
    ay_g = c["ay"][k] / G
    ax_g = c["ax"][k] / G
    if ay_g.std() > 0.02:
        s = np.polyfit(ay_g, np.degrees(c["roll"][k]), 1)[0]
        check("WARN", 0.1 < s < 3.0, "roll gradient 0.1..3 deg per g (expected ~0.4-1.5)", f"{s:.2f} deg/g")
    if ax_g.std() > 0.02:
        s = np.polyfit(-ax_g, np.degrees(c["pitch"][k]), 1)[0]
        check("WARN", 0.1 < s < 3.0, "pitch gradient 0.1..3 deg per g (expected ~0.3-1.5)", f"{s:.2f} deg/g")

    fails = [r for r in results if r[0] == "FAIL"]
    warns = [r for r in results if r[0] == "WARN"]
    print(f"\n{len(results) - len(fails) - len(warns)} passed, {len(warns)} warnings, {len(fails)} FAILED")
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
