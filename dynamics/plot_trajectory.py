#!/usr/bin/env python3
"""Plots for a trajectory CSV (sanity-check what Chrono/the kinematic driver produced).

    .venv/Scripts/python dynamics/plot_trajectory.py generated/chrono/trajectory_chrono.csv \
        --track generated/bundle/track.yaml --out-dir generated/chrono

Pure matplotlib (Agg backend, no window). Needs only numpy, matplotlib, pyyaml.
"""
import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml

G = 9.81


def load(path):
    header = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            if line.startswith("#"):
                header.append(line.strip("# \r\n"))
            else:
                break
    names = [c.strip() for c in header[-1].split(",")]
    data = np.loadtxt(path, delimiter=",", comments="#")
    return {n: data[:, i] for i, n in enumerate(names)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--track", required=True)
    ap.add_argument("--out-dir", required=True)
    a = ap.parse_args()

    c = load(a.csv)
    t = c["t"]
    out = Path(a.out_dir)
    out.mkdir(parents=True, exist_ok=True)

    tr = yaml.safe_load(open(a.track))
    cl = np.array(tr["centerline"], float)
    cones = tr.get("cones", {})

    # 1. path vs centreline + cones
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.plot(*np.vstack([cl, cl[:1]]).T, "k--", lw=1, label="centreline")
    for cls, color in [("blue", "tab:blue"), ("yellow", "gold"), ("orange_big", "orange")]:
        pts = np.array(cones.get(cls, []))
        if len(pts):
            ax.scatter(pts[:, 0], pts[:, 1], s=10, c=color, label=cls)
    ax.plot(c["x"], c["y"], "r-", lw=1.5, label="driven path")
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.legend(fontsize=8)
    ax.set_title("Path vs track")
    fig.savefig(out / "path.png", dpi=110)
    plt.close(fig)

    # 2. speed vs time
    fig, ax = plt.subplots(figsize=(9, 3))
    ax.plot(t, c["v"], lw=1)
    ax.set_xlabel("t (s)"); ax.set_ylabel("speed (m/s)")
    ax.set_title("Speed vs time")
    fig.savefig(out / "speed.png", dpi=110)
    plt.close(fig)

    # 3. pitch vs ax/g
    fig, ax = plt.subplots(figsize=(6, 5))
    ax.scatter(-c["ax"] / G, np.degrees(c["pitch"]), s=1, alpha=0.3)
    s = np.polyfit(-c["ax"] / G, np.degrees(c["pitch"]), 1)[0]
    ax.set_xlabel("braking g (-ax/g)"); ax.set_ylabel("pitch (deg, +nose down)")
    ax.set_title(f"Pitch vs longitudinal g  (slope {s:.2f} deg/g)")
    fig.savefig(out / "pitch_vs_g.png", dpi=110)
    plt.close(fig)

    # 4. roll vs ay/g
    fig, ax = plt.subplots(figsize=(6, 5))
    ax.scatter(c["ay"] / G, np.degrees(c["roll"]), s=1, alpha=0.3)
    s = np.polyfit(c["ay"] / G, np.degrees(c["roll"]), 1)[0]
    ax.set_xlabel("lateral g (ay/g)"); ax.set_ylabel("roll (deg, +right down)")
    ax.set_title(f"Roll vs lateral g  (slope {s:.2f} deg/g)")
    fig.savefig(out / "roll_vs_g.png", dpi=110)
    plt.close(fig)

    print("wrote", out / "path.png", out / "speed.png", out / "pitch_vs_g.png", out / "roll_vs_g.png")


if __name__ == "__main__":
    main()
