"""STUB - to be replaced by the track-generator agent.

Contract: generate_track(track_cfg: dict) -> Track
"""
from .track_types import Track


def generate_track(track_cfg):
    t = Track.make_test_oval(width=track_cfg.get("track_width_m", 3.5),
                             spacing=track_cfg.get("cone_spacing_m", 4.0))
    t.seed = track_cfg.get("seed")
    return t
