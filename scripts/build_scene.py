"""Build the MuJoCo scene XML for a track.

    python scripts/build_scene.py                                  # test oval -> generated/scene.xml
    python scripts/build_scene.py --track generated/track.yaml --out generated/scene.xml --view
"""
import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from conevision_sim import config                      # noqa: E402
from conevision_sim.scene.builder import write_scene   # noqa: E402
from conevision_sim.track.track_types import Track     # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--track", default=None, help="track YAML (default: built-in test oval)")
    ap.add_argument("--out", default=str(ROOT / "generated" / "scene.xml"))
    ap.add_argument("--view", action="store_true", help="open the MuJoCo passive viewer")
    a = ap.parse_args()

    track = Track.load(a.track) if a.track else Track.make_test_oval()
    cfg = config.load_all()
    t0 = time.time()
    out = write_scene(track, cfg, a.out)
    print(f"wrote {out} ({len(track.all_cones())} cones) in {time.time() - t0:.2f}s")

    if a.view:
        import mujoco
        import mujoco.viewer
        import numpy as np
        m = mujoco.MjModel.from_xml_path(str(out))
        d = mujoco.MjData(m)
        x, y, yaw = track.start_pose
        d.mocap_pos[0] = [x, y, 0]
        d.mocap_quat[0] = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
        mujoco.mj_forward(m, d)
        mujoco.viewer.launch(m, d)


if __name__ == "__main__":
    main()
