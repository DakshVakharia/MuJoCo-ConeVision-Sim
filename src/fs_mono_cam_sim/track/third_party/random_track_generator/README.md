# random-track-generator (vendored, modified)

- Upstream: https://github.com/mvanlobensels/random-track-generator
- Commit:   4ad014853493d5500aec9dd7121d6dc8f8c240c0 (branch `main`, v1.1.0)
- License:  MIT (see `LICENSE`, (c) 2024 Mees van Loeben Sels)

Files:
- `geometry.py`        unmodified from upstream.
- `track_generator.py` derived from upstream `track_generator.py`. Only the Voronoi-based
  centreline generation (`_create_track` steps 1-6) is kept. Changes: returns the smoothed
  centreline instead of cones; `shapely` dependency removed (EXTEND mode distance computed with
  numpy; self-crossing / boundary checks are done by `../../generator.py`); curvature limit and
  seed are parameters; the bare `except: continue` retry loop was removed; the Voronoi-region
  selection can no longer loop forever.
- `track.py` / `tracks/*.yaml` / gpxpy export from upstream are NOT vendored (not needed).
