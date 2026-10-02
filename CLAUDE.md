# CLAUDE.md

MuJoCo simulator for a Formula Student car whose ONLY sensor is one fast, long-range monocular
camera (no stereo, no LiDAR). It generates a random cone track, builds a MuJoCo scene (cones,
trees, roadside objects, car), drives the car around the centreline, renders the camera and
publishes over ROS Noetic (rospy):
1. the RGB feed (raw `sensor_msgs/Image` and/or JPEG `CompressedImage`) at a configurable fps/quality
2. "YOLO-like" bounding boxes as `foxglove_msgs/ImageMarkerArray`, computed exactly from MuJoCo's
   segmentation render (no neural net), with optional noise.

Sibling project: `Monocular-Depth-perception-for-Cones` (ROS package `homographic_mono_cam_depth_perception`)
consumes these topics. Its `cone_base_detector` reads `marker.points` (bbox polygon, pixel coords) and
classifies colour from `marker.outline_color` RGB thresholds — see config/perception.yaml.

## Environment
- Dev machine is Windows: MuJoCo/Python run locally in `.venv` (`.venv/Scripts/python`), ROS does NOT.
  Everything except `scripts/sim_node.py` must run and be tested without ROS.
- Run code with `PYTHONPATH=src` (or `pip install -e .`). Tests: `.venv/Scripts/python -m pytest tests`.
- Headless Linux: `MUJOCO_GL=egl`.
- Generated files (track YAML, scene XML) go to `generated/` (gitignored).

## Layout & ownership
```
config/            track.yaml scene.yaml car.yaml camera.yaml perception.yaml   (ALL tunables live here)
src/fs_mono_cam_sim/
  config.py        load_config(name) / load_all()  -> dict per yaml
  track/           track_types.py (Track contract), generator.py (generate_track), third-party adapter
  scene/           builder.py (build_scene_xml), cone meshes, scenery assets
  car/             car_model.py (add_car + camera mount), driver.py (CenterlineDriver), camera_geometry.py
  render/          renderer.py (CameraRenderer), bboxes.py, sim loop
scripts/           generate_track.py, build_scene.py, preview.py (no ROS), sim_node.py (ROS), export_homography.py
assets/            scenery/ (CC0 meshes + ATTRIBUTION.md), car/
launch/sim.launch
tests/
```

## Contracts (do not change signatures without updating every user)
- **Track** (`track/track_types.py`): `centerline (N,2)` closed loop in driving order; `cones` dict
  class -> (M,2) with classes `blue` (LEFT), `yellow` (RIGHT), `orange`, `orange_big` (start line);
  `start_pose (x,y,yaw)`; `save/load` YAML; `Track.make_test_oval()` fixture.
- `track.generator.generate_track(track_cfg) -> Track`
- `scene.builder.build_scene_xml(track, cfg) -> str` (cfg = `config.load_all()`).
  Every cone is a body named `cone_<class>_<index>`; all its geoms are children of that body.
  Visual-only geoms use `contype="0" conaffinity="0"`. `<statistic extent="1">` so `visual/map znear/zfar`
  are in metres. `visual/global offwidth/offheight` >= camera resolution.
- `car.car_model.add_car(worldbody, asset, car_cfg, cam_cfg) -> body`: mocap body named `car`
  (origin = base_link: ground under car centre, x fwd, y left, z up) containing the camera named
  `cam_cfg["name"]`. Camera pose from camera.yaml (pitch +ve = down, yaw +ve = left).
- `car.driver.CenterlineDriver(track, car_cfg).state_at(t) -> CarState(x, y, yaw, roll, pitch, v, yaw_rate)`
  — pure function of sim time.
- Image coordinates: pixels, origin top-left, u right, v down, full camera resolution.
- Car pose -> MuJoCo: `car.camera_geometry.car_mocap_pose(state) -> (pos, quat_wxyz)` into `data.mocap_*[0]`.
- Camera intrinsics go into MuJoCo via `resolution/sensorsize/focalpixel/principalpixel`;
  `principalpixel = (w/2 - cx, h/2 - cy)` (sign verified against renders, < 1 px error).

## Gotchas learned during the build
- `statistic center` is the track centre (not 0 0 0) because the sun shadow map is centred there; extent stays 1.
- Ground plane is 2 cm below the asphalt strip top (z=0); cones/car stand at z=0.
- Cones have an axis-aligned square base plate: a bbox's bottom edge is the plate's near edge, not the
  cone centre (~half plate width closer). Tests compare against projected plate corners.
- Rendering on this Windows laptop defaults to the Intel iGPU (~15-20 fps full pipeline at 1280x720);
  RGB + segmentation share one GL context with two mjrContexts (separate contexts were ~3x slower).
- `build_scene_xml` without `out_dir` writes textures to `generated/scene_assets/textures`.
- `scripts/sim_node.py` is untested against real ROS; message construction is in `render/rosmsgs.py`
  (tested with fake msg classes).
- Third-party track generator is vendored and modified in `track/third_party/random_track_generator/`
  (see its README for upstream commit and changes).
