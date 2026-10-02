# FS Monocular Camera Sim

MuJoCo simulator of a Formula Student car whose **only sensor is one fast, long-range monocular camera**
(no stereo, no LiDAR). It generates a random cone track, builds a MuJoCo scene with FS cones, trees and
roadside objects, drives an F1-style car around it and publishes over ROS Noetic:

1. the camera feed: `sensor_msgs/Image` (rgb8) and/or JPEG `sensor_msgs/CompressedImage`, at a configurable fps/resolution/quality
2. "YOLO-style" bounding boxes as `foxglove_msgs/ImageMarkerArray`, computed exactly from MuJoCo's
   segmentation render (no neural network; occlusion-aware), with an optional detector-noise model.

The output is drop-in compatible with
[Monocular-Depth-perception-for-Cones](https://github.com/DakshVakharia/Monocular-Depth-perception-for-Cones).

## Quick start (no ROS needed)
```bash
python -m venv .venv
.venv/Scripts/pip install -r requirements.txt     # Linux: .venv/bin/pip
export PYTHONPATH=src
python scripts/generate_track.py --seed 7 --out generated/track.yaml --plot generated/track.png
python scripts/build_scene.py --track generated/track.yaml --out generated/scene.xml --view
python scripts/preview.py --seconds 8 --out generated/preview.mp4          # boxes drawn on the feed
python scripts/preview.py --seconds 60 --yolo-dir generated/yolo --yolo-every 6   # YOLO-format dataset
python scripts/export_homography.py                                         # exact ground H for the camera
python -m pytest tests
```

## ROS
Put the repo in a catkin workspace (`catkin_make`, package `fs_mono_cam_sim`), install the Python deps for
ROS's python3, then:
```bash
roslaunch fs_mono_cam_sim sim.launch seed:=3            # MUJOCO_GL=egl by default (headless)
```
| Topic (default) | Type | Notes |
|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | rgb8, `publish_raw` |
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | jpeg, `jpeg_quality` |
| `/camera/bounding_boxes` | `foxglove_msgs/ImageMarkerArray` | one POLYGON marker per cone: 4 box corners, `outline_color` = class colour, `ns` = class, same stamp as the image |
| `/sim/ground_truth/odom`, `/sim/imu`, `/clock` | optional | off by default |

To feed the depth-perception pipeline, set `compressed_topic: /zed2/zed_node/left/image_rect_color_compressed`
and `bbox_topic: /stereo_cone_perception/bounding_boxes` in `config/perception.yaml` (or remap in the launch
file), and give it `generated/homography.yaml` from `export_homography.py` as its H.

## Configuration (everything lives in `config/`)
| File | What |
|---|---|
| `track.yaml` | seed, backend (vendored [mvanlobensels/random-track-generator](https://github.com/mvanlobensels/random-track-generator) or builtin), width, cone spacing, length, min radius |
| `scene.yaml` | ground/track textures, sky, sun, shadows, haze, cone geometry/colours, scenery counts and spacing |
| `car.yaml` | car dimensions/colours, driving speed/offset/speed profile, body pitch/roll oscillation |
| `camera.yaml` | resolution, fx/fy/cx/cy (or hfov), mount x/y/z + roll/pitch/yaw, clip range, **fps**, **jpeg quality** |
| `perception.yaml` | bbox filters (min pixels, max range) and noise (jitter, dropout, latency), outline colours, ROS topics |

## Notes
- **Box bottom = near edge of the cone's base plate**, as with a real detector — not the cone centre.
  A homography that uses the box bottom therefore reads ~0.1 m short.
- Rendering speed is GPU-bound. On Windows laptops make sure `python.exe` runs on the discrete GPU
  (Settings → Display → Graphics → High performance); sun shadows (`scene.lighting.shadows`) roughly halve fps.
- The car is driven kinematically along the centreline (no tyre physics).
- Scenery meshes: Kenney CC0 packs, see `assets/scenery/ATTRIBUTION.md`. Track generator: MIT, see its LICENSE.

## License
MIT
