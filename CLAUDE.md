# CLAUDE.md

MuJoCo ConeVision Sim (repo `MuJoCo-ConeVision-Sim`, ROS package / Python module `conevision_sim`,
C++ namespace `cvsim`): camera-perception simulator for a vehicle whose ONLY sensor is one fast,
long-range monocular camera (no stereo, no LiDAR). Tracks/cones follow Formula Student Driverless
conventions; the project is not affiliated with FS. It generates a random cone track, builds a MuJoCo scene (cones,
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
src/conevision_sim/
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

## C++ runtime (720p @ 120 fps target)
- Python exports a bundle once: `scripts/export_bundle.py [--golden]` -> `generated/bundle/`
  (scene.xml+meshes+textures, trajectory.csv = one lap of poses at 1 kHz, config/, golden/).
- `cpp/` = standalone CMake lib `conevision_core` (+ `cvsim_bench`, `cvsim_test_golden`); public API in
  `cpp/include/conevision_sim/*.h` is a port of render/{renderer,bboxes,sim}.py.
  `cpp/ros/sim_node.cpp` = roscpp node (render thread owns Simulation/GL; publish thread;
  `ros.time_mode` realtime|lockstep). Root `CMakeLists.txt` is catkin and does add_subdirectory(cpp).
- Windows build: `cmd //c build\\build_cpp.bat` (NMake via VS 18 vcvars64; first configure:
  `cmake -G "NMake Makefiles" -S cpp -B build/cpp -DCMAKE_BUILD_TYPE=Release -DMUJOCO_DIR=<repo>/.deps/mujoco-3.14.0`).
  Verify: `build/cpp/cvsim_test_golden.exe generated/bundle` (C++ boxes == Python, 0 px) and
  `build/cpp/cvsim_bench.exe generated/bundle 600` (RTX 4060: ~4.4 ms/frame, ~230 fps).
- Exes export NvOptimusEnabled so they use the NVIDIA GPU on Optimus laptops. EGL path (Linux) untested.
- The ROS node can't be built here; check it with g++ -fsyntax-only against `build/mock_ros/include`.
- `visualization_msgs/ImageMarker::POLYGON` = 3 (not 4).

## Chrono vehicle dynamics (`dynamics/`, branch feature/chrono-dynamics)
Purpose: realistic car tilt (pitch/roll), z and IMU data for the camera. Offline only: Chrono writes
`trajectory.csv` (contract: `docs/trajectory_contract.md`), the renderer replays it.
- Env: conda env `chrono` (Python 3.12, PyChrono 10) at `C:/Users/daksh/miniforge3/envs/chrono`, NOT the repo `.venv`.
  ALWAYS put `<env>/Library/bin` on PATH before importing pychrono, else a "Failed loading SDL3 library" dialog blocks
  the process forever (SDL2.dll there is a shim that needs SDL3.dll). Git Bash:
  `export PATH="/c/Users/daksh/miniforge3/envs/chrono/Library/bin:/c/Users/daksh/miniforge3/envs/chrono:$PATH"`.
- Run: `python dynamics/run_offline.py --track generated/bundle/track.yaml --laps 2 --vmax 8 --out generated/chrono/trajectory_chrono.csv`,
  then the acceptance test `python dynamics/check_trajectory.py <csv> --track generated/bundle/track.yaml --laps 2` (use the .venv python),
  plots `dynamics/plot_trajectory.py`, and `scripts/export_bundle.py --trajectory <csv> --golden --out generated/bundle_chrono`.
- Chrono API facts (verified): load `sedan/vehicle/Sedan_Vehicle.json` (NOT `sedan/Sedan.json`: a spec file that crashes
  WheeledVehicle) plus engine/transmission/tire JSON via `veh.GetVehicleDataFile`; `chrono.ChBezierCurve(list_of_ChVector3d)`;
  `veh.ChPathFollowerDriver(vehicle, path, name, speed)` with `SetDesiredSpeed`. `chassis.GetPosDt2()` returns ~0, so do NOT use it:
  derive acceleration from the logged velocity (np.gradient + 20 Hz zero-phase low-pass), then f_body = R^T (a + [0,0,9.81]).
  `quat_to_rotation_matrix` in run_offline.py is body->world; einsum('nji,nj->ni', R, a) gives R^T a (do not transpose R first).
- Two agent reports claimed success while the data was wrong. Always run `check_trajectory.py` and compare ax/ay with
  d(v)/dt and v*yaw_rate computed from the same file before trusting a Chrono CSV.
- The stock Sedan is a road car (roll ~1.35 deg/g, pitch ~0.9 deg/g). `dynamics/adsdv/REVIEW.md` lists corrected FS-class
  parameters (mostly assumptions; NOT the real ADS-DV). Editing a JSON copy to those values is not done yet.
- Chrono runs are single runs (loop=0): the car stops at the end of the CSV (the renderer clamps).

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
