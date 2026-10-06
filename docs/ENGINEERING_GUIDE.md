# Engineering guide: how MuJoCo ConeVision Sim works

Audience: an engineer who wants to read, run, debug and modify this code without help.
The beginner version is `docs/WALKTHROUGH.md`; the data format is specified in
`docs/trajectory_contract.md`; ROS/Ubuntu build notes are in `docs/ros_cpp_node.md`.

Everything below was checked against the code at the time of writing. Where something is *not*
verified, it says so (section 13).

---

## 1. What the system is

A simulator that produces, for a vehicle carrying **one monocular camera**:

1. camera images (RGB) at a configurable resolution / fps / JPEG quality,
2. "YOLO-style" 2D bounding boxes of traffic cones, computed exactly (no neural net),
3. IMU and odometry from the same motion, so a downstream depth pipeline can correct for tilt,

with exact ground truth for everything (cone positions, car pose). The downstream consumer is the
sibling repo `Monocular-Depth-perception-for-Cones` (homography-based cone distance).

### 1.1 The three execution contexts

```
 OFFLINE, once per track (Python)                    RUNTIME, every frame
 --------------------------------                    ---------------------------------------------
 generate_track  -> Track                            Python reference:  render/sim.py Simulation
 build_scene_xml -> scene.xml (+meshes, textures)    C++ (fast):        cpp/ conevision_core + node
 driver / Chrono -> trajectory.csv                        |
        |                                                 v
        +---- scripts/export_bundle.py --> bundle/ --> ROS topics (image, boxes, IMU, odom, /clock)
```

* **Offline Python** creates the world. It never runs per frame.
* **Runtime** loads the *bundle* (a folder) and, for frame k at time `t = k / fps`: looks up the car
  pose, writes it to MuJoCo, renders RGB + a segmentation image, turns the segmentation into boxes,
  and publishes.
* There are **two implementations of the runtime**: Python (`src/conevision_sim/render/`, ~50 ms/frame,
  used for tools and as the *reference*) and C++ (`cpp/`, ~4-5 ms/frame, the real-time node). They are
  kept equal by a **golden test** (section 9.3).

### 1.2 Why the split (design decisions you will be asked about)

| Decision | Reason |
|---|---|
| Python for the world, C++ for the frame loop | World building is run once and benefits from numpy/scipy and a track generator that is Python. The frame loop at 120 fps leaves 8.3 ms, which Python cannot meet (GL render + readback + box extraction ~50 ms in Python). |
| Boxes from a segmentation render, not projection | Occlusion and image borders come out right for free; the box is exactly what is *visible*. |
| Pose comes from a replayed file (`trajectory.csv`), not live physics | Reproducible (same run every time, golden test possible), keeps the renderer simple. Cost: the car cannot react to anything (no closed loop). |
| Chrono runs offline | Real suspension/tyre physics only matter for the *tilt signal*; running it live would add timing risk for no benefit to perception testing. |
| One agreed CSV format for all producers | Lets the kinematic driver, Chrono and any future vehicle model plug in unchanged. |

---

## 2. Repository map

Line counts are approximate. "Read" = worth reading in full; "Skim" = understand the purpose only.

```
config/                       5 YAML files: every tunable                                 Read (200 lines)
docs/                         trajectory_contract.md, ros_cpp_node.md, WALKTHROUGH.md, this file
src/conevision_sim/           the Python package (import conevision_sim)
  config.py            30     YAML loading                                                Read
  track/
    track_types.py    102     Track dataclass: THE data contract                          Read
    generator.py      228     generate_track(); _finalize() validates and places cones    Read
    builtin.py         59     fallback centreline generator                               Skim
    third_party/              vendored random-track-generator (MIT) + LICENSE             Skim
  scene/
    builder.py        241     build_scene_xml(), write_scene()                            Read
    cones.py          122     procedural striped FS cone meshes                           Skim
    scenery.py        237     deterministic placement of trees/objects                    Skim
    textures.py        53     procedural ground textures (PNG)                            Skip
    asset_import.py   210     one-off converter for the Kenney CC0 OBJ packs              Skip
  car/
    camera_geometry.py 159    pinhole maths, frames, ground homography (NO MuJoCo)        Read, twice
    car_model.py      225     procedural car + camera attributes for MuJoCo               Read camera_attributes()
    driver.py         148     CenterlineDriver: kinematic "train on rails"; CarState      Read
    trajectory_io.py  445     CSV reader/writer, TrajectoryDriver, kinematic_columns      Read
  render/
    sim.py             93     Simulation: pose -> mj_forward -> render -> boxes           Read
    renderer.py        69     RGB + segmentation render (one GL context, 2 mjrContexts)   Read
    bboxes.py         133     segmentation image -> Detection list (+noise/filters)       Read
    rosmsgs.py        104     ROS message builders that take message classes as args      Skim
scripts/                      CLI entry points (section 8)
cpp/                          C++ runtime (section 10)
  include/conevision_sim/     5 public headers (the C++ API)                              Read
  src/                        config, trajectory, renderer, bboxes, simulation, GL ctx    Read
  ros/sim_node.cpp   502      the roscpp node                                             Read
  test/ tools/                golden test, trajectory test, benchmark                     Read test_golden
dynamics/                     Chrono vehicle dynamics (section 11)
tests/                        pytest suites (section 9)
assets/scenery/               25 converted CC0 meshes + ATTRIBUTION.md + catalog.json
launch/                       sim.launch (Python node), sim_cpp.launch (C++ node)
CMakeLists.txt, package.xml   catkin package `conevision_sim`; root CMake add_subdirectory(cpp)
generated/                    outputs (gitignored): bundle/, chrono/, previews
```

Naming map (the project was renamed; you will see all three):

| What | Name |
|---|---|
| GitHub repo | `MuJoCo-ConeVision-Sim` |
| ROS package / Python module | `conevision_sim` |
| C++ namespace / library / executables | `cvsim` / `conevision_core` / `cvsim_bench`, `cvsim_test_golden`, `cvsim_test_trajectory`, `conevision_sim_node` |
| Local folder on the dev machine | `FS-Monocular-Camera-Sim` (rename pending) |

---

## 3. Conventions (read this before any code; most bugs are here)

### 3.1 Frames

| Frame | Definition |
|---|---|
| **world** | MuJoCo world = the track frame. x, y on the ground, **z up**. Track starts at the origin heading +x. |
| **base_link** | Attached to the car. Origin = the ground point under the car centre. **x forward, y left, z up.** |
| **camera (optical)** | **x right, y down, z forward** (OpenCV). The projection formula uses this frame. |
| **MuJoCo camera** | looks along **-z**, x right, y up. (`car_model.camera_xyaxes` converts: x axis = optical x, up axis = -optical y.) |
| **pixel** | u right, v down, origin top-left, full camera resolution. |

### 3.2 Angles and signs (used by Python, C++, Chrono and the CSV)

* Orientation of the car: `R_world_base = Rz(yaw) · Ry(pitch) · Rx(roll)` (`camera_geometry.rpy_matrix`,
  `render/sim.py euler_to_quat`, C++ `Trajectory::mocap_pose`: all the same formula).
* **yaw** + = turning left (counter-clockwise from above).
* **pitch** + = **nose down** (rotation about +y).
* **roll** + = **right side down** (rotation about +x).
* **Camera extrinsics** in `camera.yaml`: `pitch_deg` + tilts the camera **down**, `yaw_deg` + turns it **left**.
* Roll/pitch in a trajectory are **relative to the at-rest pose** (a parked car reads 0, 0).

### 3.3 Units and time

SI everywhere (metres, seconds, radians in code and CSV; degrees only in YAML names ending `_deg`).
Frame `k` has `t = k / camera.fps` exactly; sim time is never wall-clock inside the renderer.

### 3.4 IMU semantics

`ax, ay, az` are **specific force in the body frame, gravity included**: a level stationary car reads
`(0, 0, +9.81)`. In a hard corner at 0.9 g, `az` is still ~9.8 (it does not shrink). Formula used by
the producers: `f_body = R^T (a_world + [0,0,g])`.

### 3.5 Cones

Class names: `blue` (left of travel), `yellow` (right), `orange` (small), `orange_big` (start gate).
MuJoCo body name: `cone_<class>_<index>`. The box code **depends on this naming**.
A cone's body origin is on the ground; its base is a square plate, so a detection's bottom edge is the
plate's near edge, not the cone centre (about 0.1 m bias for distance).

---

## 4. Data contracts

### 4.1 `Track` (`track/track_types.py`)

| Field | Type | Meaning |
|---|---|---|
| `centerline` | (N,2) array | closed loop in driving order, last point != first, ~0.5 m spacing |
| `cones` | dict class -> (M,2) | cone positions |
| `start_pose` | (x, y, yaw) | car at the start line (always (0, 0, 0) from `generate_track`) |
| `seed`, `meta` | | reproducibility / backend name, length, min radius, direction |

`Track.save/load` use YAML; `Track.make_test_oval()` is a tiny fixture used by tests.

### 4.2 `trajectory.csv`

Specified in `docs/trajectory_contract.md`. Summary: 1 kHz rows of `t,x,y,z,roll,pitch,yaw,v,yaw_rate,ax,ay,az,wx,wy,wz`;
columns found **by name** from the last `# ` header line; header carries `lap_time=`, `loop=` (1 wraps,
0 clamps) and `source=`. If the file has no roll/pitch columns, the legacy sinusoidal `body_motion`
from `car.yaml` is added; otherwise nothing is added.

### 4.3 The bundle (`scripts/export_bundle.py` output)

```
generated/bundle/
  scene.xml  meshes/  textures/     MuJoCo scene (relative paths; loads from its own folder)
  trajectory.csv                    car motion
  track.yaml                        the Track (for tools / Chrono)
  config/                           copy of config/*.yaml used (the C++ node reads these)
  golden/frame_<k:05d>.json + .png  Python reference detections for the C++ golden test (e.g. frame_00060.json)
```

The C++ runtime only reads a bundle. It never generates a track or a scene.

### 4.4 ROS outputs (defaults from `config/perception.yaml`)

| Topic | Type | Notes |
|---|---|---|
| `/camera/image_raw` (+ `/compressed`) | `sensor_msgs/Image` rgb8 (+ JPEG) | C++ uses `image_transport`; JPEG quality from `camera.jpeg_quality` |
| `/camera/bounding_boxes` | `foxglove_msgs/ImageMarkerArray` | one `ImageMarker` per cone, `type = POLYGON` (**value 3**, not 4), 4 corners `(xmin,ymin),(xmax,ymin),(xmax,ymax),(xmin,ymax)`, `ns` = class, `id` = cone index, `outline_color` = class colour |
| `/zed2/zed_node/imu/data` | `sensor_msgs/Imu` | orientation from roll/pitch/yaw; angular velocity `(wx,wy,wz)`; linear acceleration `(ax,ay,az)` |
| `/slam/state` | `nav_msgs/Odometry` | `map -> base_link` |
| `/clock` | `rosgraph_msgs/Clock` | in `lockstep` mode |

Compatibility with the depth repo's `cone_base_detector`: it takes the maximum y over `marker.points`
(the box bottom), averages x at that y, subtracts `y_offset_pixels` (5 px), and classifies colour
from `outline_color` thresholds. The colours in `perception.yaml` satisfy those thresholds, and
`tests/test_bboxes.py::test_marker_array_matches_cone_base_detector_expectations` checks the logic.

---

## 5. Python modules in detail

### 5.1 `config.py`

`load_config(name)` returns the section under the top-level key `name` when the file has only that key
(camera, car, track, scene); `perception.yaml` has several top-level keys (`bbox`, `render`, `seed`, `ros`)
so it returns the whole dict. `load_all()` returns `{track, scene, car, camera, perception}`, the
`cfg` object that nearly every function takes.

### 5.2 Track generation (`track/`)

`generate_track(track_cfg)` -> `Track`:
1. pick a backend (`auto` = vendored Voronoi generator `third_party`, falling back to `builtin` after
   `max_attempts` failures);
2. each backend only produces a **raw closed centreline**; `_finalize(raw, cfg, rng)` does the rest and
   returns `None` to reject a candidate:
   * resample to 0.5 m, scale uniformly to `target_length_m` (reject if scale outside `scale_range`),
     smooth (Gaussian, sigma 1 m);
   * reject if max curvature > 1/(0.98 * `min_corner_radius_m`), if total turning is not 2*pi, or if the
     loop passes within `track_width + 1 m` of itself away from its neighbourhood (`_loop_ok`);
   * random driving direction; start at the **straightest window**; rotate/translate so the start is at
     (0, 0) heading +x;
   * cones: offset the centreline by +/- width/2 and resample at <= `cone_spacing_m` (blue left,
     yellow right); four `orange_big` gate cones 5 m apart; drop blue/yellow cones within 2 m of the gate;
   * validate: no two cones closer than 1 m, none closer than `width/2 - 0.2` to the centreline.
3. `seed: null` picks a random seed (stored in `track.seed`).

Note it returns a **path, not a trajectory**: no time, no speed.

### 5.3 Scene building (`scene/`)

`build_scene_xml(track, cfg, out_dir=None)` assembles an MJCF string with `xml.etree`:
`<compiler>`, `<option>`, `<statistic extent="1">` (so `znear/zfar` are metres), `<visual>` (offscreen
size >= camera resolution, shadow size, MSAA samples, fog from `scene.haze`, `znear/zfar` from the camera),
`<asset>` (sky gradient, ground texture, cone meshes, scenery meshes, car meshes), `<worldbody>`
(sun, ground plane sized from track bounds + margin, an asphalt **strip** mesh following the centreline
with half-widths from the nearest cones, cones, scenery, the car).
* Cones: `cones.add_cone_assets` builds one mesh per class (plate + truncated cone + stripes baked into a
  palette texture); `add_cone_body` places each as body `cone_<class>_<i>` with only visual geoms
  (`contype=0 conaffinity=0`).
* Scenery: `scenery.place_scenery` is **deterministic** (`scene.scenery.seed`): random positions between
  `min/max_dist_from_track_m`, never within `cone_clearance_m` of a cone or overlapping another object;
  falls back to procedural trees/boxes if the CC0 assets are missing.
* The car: `car_model.add_car(worldbody, asset, car_cfg, cam_cfg)` adds a **mocap body named `car`**
  (index 0 in `data.mocap_*`) containing the visual-only car and the camera.
* `write_scene(track, cfg, path)` writes the XML plus `meshes/` and `textures/` next to it using relative
  paths, so the folder is relocatable (this is what the bundle uses).

### 5.4 Camera and motion (`car/`)

**`camera_geometry.py`: the maths you must understand.**

```
R_base_cam = Rz(yaw) Ry(pitch) Rx(roll) @ R_OPT      (extrinsics from camera.yaml; R_OPT maps the optical axes into base_link)
T_wc       = T_world_base(car_state) @ T_base_cam     (camera pose in the world; T_world_base uses x, y, yaw, pitch, roll)
p_cam      = R_wc^T (P_world - t_wc)
u = fx * X/Z + cx        v = fy * Y/Z + cy             (X, Y, Z = p_cam)
```
`project_world_points` implements exactly that. `ground_homography(cam_cfg, car_state=None)` returns the
exact 3x3 `H` mapping a pixel to a flat-ground point `(x fwd, y left)` in metres in the gravity-aligned
base_link frame: with `pixel ~ K R^T (X - t)` and `X = (gx, gy, 0)`, `M = K [R^T[:,0], R^T[:,1], -R^T t]`,
`H = M^-1` (normalised so `H[2,2] = 1`). If `car_state` is given, its roll/pitch (not yaw) tilt the camera
relative to the ground. `scripts/export_homography.py` writes it in the depth repo's YAML format.

Why tilt matters (derive it once): the ground distance of a point seen `a` below the horizon from height `h`
is `d = h / tan(a)`, so a pitch error `da` moves it by about `d^2/h * da`. With `h = 1.15 m`, `da = 0.5 deg`
the shift is ~0.8 m at 10 m, ~3 m at 20 m, ~7 m at 30 m.

Gotcha: `T_world_base` sets the base_link z to 0, so `car_mocap_pose()` ignores `CarState.z`. The
Python `Simulation.step_to` adds `state.z` itself; C++ `Trajectory::mocap_pose` copies `z`.

**`car_model.py`.** `add_car` builds a procedural open-wheel car (lofted meshes, wings, wheels, halo,
helmet; all visual-only). `camera_attributes(cam_cfg)` is the important function: it emits MuJoCo's
`resolution`, `sensorsize` (arbitrary 1e-3 per pixel), `focalpixel = (fx, fy)` and
`principalpixel = (w/2 - cx, h/2 - cy)` (**sign opposite to cx - w/2**, verified by test) so MuJoCo's
render matches the pinhole model to < 1 px (`tests/test_camera.py::test_projection_matches_render`).

**`driver.py`.** `CarState` is the pose/motion record (`x, y, yaw, roll, pitch, v, yaw_rate` plus
`z, ax, ay, az, wx, wy, wz` with defaults). `CenterlineDriver(track, car_cfg).state_at(t)` is a **pure
function of t** (everything is precomputed in `__init__`): periodic cubic spline of the centreline ->
uniform arc-length table -> optional smoothing and lateral offset -> heading from the chord between points
`lookahead/2` behind and ahead -> speed profile `v(s)` (constant, or `min(speed, sqrt(a_lat/curvature))`
with accel/brake passes) -> time table. It is kinematic: no forces, no tilt (except the optional sine
`body_motion`).

**`trajectory_io.py`.**
* `read_trajectory_csv` / `write_trajectory_csv`: contract-compliant I/O (`TrajectoryData` holds columns,
  `lap_time`, `loop`, `source`).
* `TrajectoryDriver(csv_or_data, car_cfg).state_at(t)`: linear interpolation of every column, `loop=1`
  wraps (`t mod lap_time`), `loop=0` clamps, yaw wrapped on output, missing columns default per contract.
  This is the Python mirror of C++ `Trajectory::state_at`.
* `kinematic_columns(driver, rate_hz, tilt)`: samples a `CenterlineDriver` into CSV columns. With
  `tilt.enabled` it adds a cheap **acceleration-based tilt model** (no Chrono): `pitch = -k_p * a_long/g`
  (braking => positive, nose down), `roll = k_r * a_lat/g`, each through a 2nd-order filter
  (`natural_freq_hz`, `damping`), run over 3 laps keeping the middle one so the lap closes; the IMU
  columns come from `f_body = (Ry(pitch) Rx(roll))^T (a_long, a_lat, g)`.

### 5.5 Rendering and boxes (`render/`)

**`Simulation` (`sim.py`)**: the per-frame pipeline.

```
frame(t):  step_to(t)            driver.state_at(t) -> data.mocap_pos[0] = (x, y, z); data.mocap_quat[0] = euler_to_quat(yaw, pitch, roll)
                                 mujoco.mj_forward(model, data)         (kinematics only, no dynamics step)
           renderer.render_rgb(data)
           renderer.render_segmentation(data)
           compute_bboxes(model, data, seg, cam_cfg, bbox_cfg, rng)
           -> Frame(t, rgb, detections, car_state, timings)
```
`Simulation(cfg, track=None, seed=None, driver=None)` builds the track, scene, model, driver (a given
`driver`, else `TrajectoryDriver` if `cfg["car"]["trajectory_csv"]` is set, else `CenterlineDriver`) and
renderer. `frame_at_index(k)` uses `t = k / fps`.

**`CameraRenderer` (`renderer.py`)**: one OpenGL context with **two** `mjrContext`s: RGB (with MSAA from
the scene or `render.msaa_samples`) and segmentation (no MSAA, because ids must not blend). The
segmentation pass sets `mjRND_SEGMENT | mjRND_IDCOLOR`; each geom is then drawn flat in a colour that
encodes `segid + 1` as a little-endian 24-bit number (0 = background). `render_segmentation` decodes with
a lookup table `segid + 1 -> geom id` and flips vertically (GL origin is bottom-left).
(Two separate GL contexts were measured ~3x slower than one context with two mjrContexts.)

**`compute_bboxes` (`bboxes.py`)**:
1. `geom_to_cone[geom_id]` -> cone body id (or -1), from body names (`_tables`, cached per model);
2. `pixel_stats`: per cone body, pixel count and min/max x, y (Python: sort + `reduceat`);
3. box = `(xmin, ymin, xmax+1, ymax+1)` (inclusive extent + 1 = pixel *edges*);
4. filters from `perception.yaml bbox`: `min_visible_pixels`, `min_box_height_px`, `max_range_m`
   (range = |body xpos - camera pos|), `dropout_prob`; noise: `center_jitter_px`, `size_jitter_frac`
   (seeded RNG); results sorted nearest first.
A `Detection` carries `cls, index, body, x_min..y_max, visible_pixels, range_m`.

**`rosmsgs.py`**: message construction as pure functions that receive the message classes as
arguments, so they are unit-testable without ROS (`tests/test_bboxes.py` uses fake classes).

---

## 6. End-to-end traces

### 6.1 "Make a bundle with Chrono tilt"
```
generate_track(cfg.track)        -> Track (seed 42 by default)
Chrono run (dynamics/)           -> generated/chrono/trajectory_chrono.csv     (loop=0, source=chrono)
export_bundle.py --trajectory X --golden --out generated/bundle_chrono
    write_scene(...)             -> scene.xml, meshes/, textures/
    copy X                       -> trajectory.csv
    Simulation(cfg, driver=TrajectoryDriver(X)).frame_at_index(k) for the golden frames
                                 -> golden/frame_<k>.json (detections + car_state) and .png
cvsim_test_golden generated/bundle_chrono   C++ must reproduce the golden boxes
```
Important: the Chrono run and the bundle must use the **same track** (same seed): the CSV is in the
track frame.

### 6.2 "What changes if I set `camera.pitch_deg` from 4 to 10?"
`camera_geometry.R_base_cam` (extrinsics) -> `car_model.camera_xyaxes` (MuJoCo camera axes written into the
scene XML) -> the scene must be rebuilt (the bundle contains the old camera). `ground_homography` changes
too, so re-run `export_homography.py` if the depth pipeline uses it. Nothing in the C++ code needs editing.

### 6.3 One frame in the C++ node
`render_thread` (owns the `Simulation`, hence the GL context): `acquire` a `Frame` from the pool ->
`sim->frame(k, *f)` (state -> `mj_forward` -> `CameraRenderer::render` -> `BoxComputer::compute`) -> push to
the ready queue. Main thread: pop -> build `Image` (move the pixel buffer in, no copy), `ImageMarkerArray`,
IMU, odometry, `/clock` -> publish -> release the frame. See section 10.6.

---

## 7. Configuration reference

| File | Keys that matter |
|---|---|
| `camera.yaml` | `name`, `width`, `height`, `fx fy cx cy` (or `hfov_deg` with fx/fy null), mount `x y z`, `roll_deg pitch_deg yaw_deg`, `near_clip far_clip`, `fps`, `jpeg_quality` |
| `perception.yaml` | `bbox.*` (filters, noise, latency, `outline_colors`), `render.msaa_samples`, `render.seg_scale` (C++ only), `seed`, `ros.*` (topics, `frame_id`, `imu_frame_id`, `publish_*`, `realtime_factor`, `time_mode` realtime/lockstep) |
| `track.yaml` | `seed`, `backend`, `track_width_m`, `cone_spacing_m`, `target_length_m`, `min_corner_radius_m`, `backend_params.*` |
| `scene.yaml` | `ground` (texture, margin, `track_strip`), `lighting` (sun, shadows, MSAA), `sky`, `haze`, `cones` (sizes, stripes, colours), `scenery` (counts, seed, distances, `use_assets`) |
| `car.yaml` | geometry and colours, `driver.*` (speed, offset, lookahead, `max_lateral_accel_mps2`), `body_motion.*` (legacy sine tilt), `tilt_model.*` (accel-based tilt) |

Changing a YAML value takes effect for: Python scripts immediately; the C++ node only after
re-running `export_bundle.py` (the bundle holds a *copy* of `config/` and a built scene).

---

## 8. Scripts

| Script | Purpose | Notable flags |
|---|---|---|
| `generate_track.py` | make a track, save YAML and a PNG preview | `--seed --backend --out --plot` |
| `build_scene.py` | write `scene.xml` | `--track --out --view` |
| `export_bundle.py` | everything the C++ node needs | `--trajectory CSV --tilt-model --golden --golden-frames --out --track-seed` |
| `export_homography.py` | exact ground homography in the depth repo's YAML format | `--out` |
| `preview.py` | render with boxes drawn (window and/or MP4), YOLO-format export | `--seconds --show --out --trajectory --speed --yolo-dir --frames-png` |
| `view_sim.py` | MuJoCo 3D viewer, chase camera | `--trajectory --speed --track-seed` |
| `sim_node.py` | the Python ROS node (reference/fallback) | rosparams `~config_dir ~seed` |

Run Python with `PYTHONPATH=src .venv/Scripts/python scripts/<name>.py ...` (or `pip install -e .`).
(`preview.py --speed N` renders every Nth frame; it is playback speed, not car speed.)

---

## 9. Tests: what each one proves

### 9.1 Python (`PYTHONPATH=src .venv/Scripts/python -m pytest tests -q` -> 98 pass)

| File | What it guarantees |
|---|---|
| `test_track.py` | tracks are valid (closed, no self-intersection, blue left / yellow right, spacing, width), deterministic per seed, both backends |
| `test_scene.py` | XML loads, every cone has exactly one body with the right name, scenery exclusion zones and determinism, segmentation maps cone pixels to cone bodies |
| `test_camera.py` | projection agrees with the MuJoCo render (< 1 px) incl. off-centre cx/cy and fx != fy; homography round trip and agreement with rendered ground points |
| `test_car.py` | driver continuity and lap wrap, stays within the track, speed slows in corners, camera does not see the car |
| `test_bboxes.py` | exact boxes, occlusion, border truncation, filters, seeded noise, ROS marker layout vs `cone_base_detector` rules, boxes vs projected cone base |
| `test_trajectory_io.py` | CSV round trip, legacy 6-column files, interpolation, loop/clamp, defaults, tilt model signs and lap closure |

### 9.2 C++ (`build/cpp/cvsim_test_trajectory.exe`, `cvsim_test_golden.exe <bundle>`)
`test_trajectory` mirrors the Python reader tests (columns by name, CRLF, loop vs clamp, defaults).

### 9.3 The golden test (the most important test)
For each `golden/frame_<k>.json` in a bundle, C++ renders frame `k` with noise off and requires: the same
set of `(class, index)` detections (a cone with few visible pixels may differ, "border cases" are
skipped), every box within 1 px, range within 1 cm. A second pass at `seg_scale = 0.5` uses a looser
tolerance (5 px). Passing means the C++ port equals the Python reference on that bundle, including
the pose pipeline (tilt, z, yaw).

### 9.4 Chrono acceptance (`dynamics/check_trajectory.py`)
18 pass/fail checks on a Chrono CSV (columns, 1 kHz, relative-to-rest, z, IMU plausibility, on-track,
laps) plus tilt-gradient warnings. **A trajectory is usable only if it exits 0.**

---

## 10. The C++ runtime in detail

### 10.1 Build

* Standalone (Windows or Linux, no ROS): `cmake -S cpp -B build/cpp -DMUJOCO_DIR=<mujoco release>`;
  on this machine `cmd //c "build\\build_cpp.bat"` (NMake via the Visual Studio vcvars64).
  Fetches yaml-cpp, GLFW (Windows) and nlohmann_json through CMake `FetchContent`.
* Catkin (Ubuntu + ROS Noetic): the root `CMakeLists.txt` does `add_subdirectory(cpp)` and builds
  `conevision_sim_node`. MuJoCo is found through `MUJOCO_DIR`.
* Targets: `conevision_core` (static library), `cvsim_bench`, `cvsim_test_golden`, `cvsim_test_trajectory`,
  `conevision_sim_node`.
* On Windows Optimus laptops, `cvsim_bench` and `cvsim_test_golden` export the `NvOptimusEnabled` symbol so
  they use the NVIDIA GPU. Other programs (e.g. the Python viewers) use whichever GPU Windows assigns to
  `python.exe`, which is the Intel iGPU by default (set it in Windows graphics settings).

### 10.2 Public API (`cpp/include/conevision_sim/`)

| Header | Content |
|---|---|
| `config.h` | `SimConfig` (+ `CameraConfig`, `BBoxConfig`, `RenderConfig`, `RosConfig`, `BodyMotion`); `load_config(dir)` reads camera/perception/car YAML |
| `trajectory.h` | `CarState`; `Trajectory(csv, body_motion)`; `state_at(t)`, `mocap_pose(state, pos, quat)` |
| `renderer.h` | `GlContext::create()` (EGL on Linux, hidden GLFW window on Windows); `CameraRenderer::render(data, rgb, seg)` |
| `bboxes.h` | `Detection`; `BoxComputer(model, cfg, seed).compute(data, cam_id, seg, w, h)` |
| `simulation.h` | `Simulation(bundle_dir, config_dir="", seed_override=-1)`; `frame(k, Frame&)`; `Frame` holds `rgb`, `detections`, `car`, `timing` |

### 10.3 `renderer.cpp`
Same design as `renderer.py`: one GL context, `ctx_rgb` (MSAA) and `ctx_seg` (none). The segmentation pass
may use a smaller viewport (`render.seg_scale`, 0.5 = ~4x cheaper), recorded in `SegImage.scale`;
boxes are scaled back to camera pixels. Decode: `code = r | g<<8 | b<<16`, `geom = lut[code]`, rows flipped
so row 0 is the top. All buffers are preallocated; the only per-frame allocation is the detection vector.

### 10.4 `bboxes.cpp`
Same filters/noise/order as Python, but per-body count/min/max in a **single pass** over the pixels (no
sort), using a geom-to-cone-body table built from the body names. `std::mt19937_64` instead of numpy's RNG, so
**noise realisations differ from Python** (the golden test therefore runs with noise off).

### 10.5 `trajectory.cpp`, `simulation.cpp`
`Trajectory` parses the header (`lap_time`, `loop`), takes column names from the last `# ` line, and
interpolates exactly as in section 4.2. `Simulation::frame(k, out)`: `t = k / fps` -> `state_at(t)` ->
`mocap_pose` -> `mj_forward` -> `render` -> `compute`, filling `out.timing`.

### 10.6 `ros/sim_node.cpp`: threads and time
* **Render thread** creates and owns the `Simulation` (a GL context is bound to a thread), loops
  `acquire frame -> sim->frame(k) -> push_ready`.
* **Main thread** pops frames, builds and publishes the messages (`image_transport` for the image, so
  raw + `/compressed` + others come from plugins and the JPEG is only encoded if somebody subscribes), then
  releases the frame to the pool. `FramePool` (3 frames) bounds latency and avoids allocations; the pixel
  buffer is *moved* into the `Image` message and moved back if nothing kept it.
* **`time_mode: realtime`**: frame k is due at `wall0 + k / (fps * realtime_factor)`; if the renderer is
  more than one period late it **skips frame indices** to stay real-time and warns. Stamps are
  `start_time + k/fps`.
* **`time_mode: lockstep`**: never skips; publishes `/clock = k/fps` before each frame; run all nodes with
  `/use_sim_time:=true`. `~lockstep_throttle` additionally paces to the realtime factor.
* `latency_ms`: boxes are held in a queue and published later than the image (image stamp unchanged).
* Params: `~bundle_dir`, `~config_dir`, `~seed` (override of the box-noise seed), `~time_mode`.

---

## 11. Chrono vehicle dynamics (`dynamics/`)

Purpose: realistic tilt (and z, IMU) from braking and cornering. **Offline only**: it writes a
`trajectory.csv`; the renderer replays it.

### 11.1 Environment
A separate conda environment (`chrono`, Python 3.12, PyChrono 10) at
`C:/Users/daksh/miniforge3/envs/chrono`. **Before importing pychrono, `<env>/Library/bin` must be on
PATH**, otherwise the SDL2 compatibility shim cannot find `SDL3.dll` and shows a modal
"Failed loading SDL3 library" dialog that blocks the process. `run_offline.py` does this itself
(`os.add_dll_directory` plus PATH); for ad-hoc commands in Git Bash:
`export PATH="/c/Users/daksh/miniforge3/envs/chrono/Library/bin:/c/Users/daksh/miniforge3/envs/chrono:$PATH"`.

### 11.2 Files

| File | Role |
|---|---|
| `vehicle.py` | `build_vehicle`: the **car** (no driving logic): `WheeledVehicle` from `sedan/vehicle/Sedan_Vehicle.json`, engine/transmission/tyre JSON, rigid terrain |
| `drivers.py`, `run_offline.build_path_follower_driver` | the **driver**: `ChPathFollowerDriver` over a `ChBezierCurve` built from the centreline |
| `speed_profile.py` | `compute_speed_profile`: `v = min(vmax, sqrt(a_lat/kappa))` with accel/brake passes |
| `run_offline.py` | `run_simulation(...)`: settle, drive, log at 1 kHz, post-process IMU, write the CSV |
| `check_trajectory.py` | acceptance test; `plot_trajectory.py` plots; `test_offline.py` unit tests |
| `adsdv/` | researched FS-class parameters (`PARAMETERS.md`, `adsdv_params.yaml`), `REVIEW.md` (corrections), `CHRONO_MAPPING.md` (which JSON keys to edit) |

### 11.3 What `run_simulation` does
1. Load `track.yaml`; build the vehicle at `start_pose`; build the path follower.
2. **Settle** (default 7 s, no inputs); average z/roll/pitch over the last 0.5 s = the at-rest reference.
3. Loop at `dt = 1 ms`: set the desired speed (constant ramp, or the **speed profile** with `--profile`),
   `driver/vehicle/terrain.Synchronize` then `Advance`, record position, quaternion, world velocity,
   body angular velocity; stop after `laps * track_length` driven.
4. Post-process: `a_world = d(velocity)/dt`, zero-phase 20 Hz low-pass (`sosfiltfilt`), add `[0,0,9.81]`,
   rotate with `R^T` to get body-frame `ax, ay, az`.
5. Write the CSV (`loop=0`, `source=chrono`, `lap_time` = duration of the rows).

### 11.4 Verified API facts / traps
* Load `sedan/vehicle/Sedan_Vehicle.json`, **not** `sedan/Sedan.json` (a spec file; loading it crashes).
* `chrono.ChBezierCurve(list_of_ChVector3d)`; `veh.ChPathFollowerDriver(vehicle, path, name, speed)`;
  `driver.SetDesiredSpeed(v)`.
* `chassis.GetPosDt2()` returned ~0; **derive acceleration from the logged velocity**.
* `quat_to_rotation_matrix` in `run_offline.py` is body -> world; `einsum('nji,nj->ni', R, a)` = `R^T a`.

### 11.5 Reading a Chrono result
Run `check_trajectory.py`, then compare `ax` with the filtered `dv/dt` and `ay` with `v * yaw_rate` computed
from the same file (they should correlate strongly with slope ~1). Tilt gradients for the stock sedan:
roll ~1.35 deg/g, pitch ~0.9 deg/g. The stock Sedan is a softer road car than an FS car (expected
0.4-1.5 deg/g); `adsdv/REVIEW.md` lists the corrected parameters, but the vehicle JSON has not been edited yet.

---

## 12. Recipes

| I want to... | Do this |
|---|---|
| Change the camera lens or mounting | edit `config/camera.yaml`, rebuild the scene (`export_bundle.py`), re-export the homography |
| Add a cone class | add the colour to `scene.yaml cones.colors` and `perception.yaml bbox.outline_colors`, extend `CONE_CLASSES` (track_types), the stripe tables in `cones.py`, and the class list handling in `bboxes.py`/`bboxes.cpp` |
| Add detector noise | extend `compute_bboxes` and `BoxComputer::compute` **together**, then regenerate the golden frames |
| Publish another topic | C++: `sim_node.cpp` (publisher + builder from `Frame`); Python mirror: `rosmsgs.py` + `scripts/sim_node.py` |
| Use a different vehicle in Chrono | add a builder next to `build_vehicle`; keep the output CSV contract |
| Drive faster | Chrono: `run_offline.py --vmax 16 --profile --a-lat 7` (**not yet run to completion**); kinematic: `car.yaml driver.speed_mps` |
| Check a C++ change did not break anything | rebuild, run `cvsim_test_golden` on a fresh bundle |

---

## 13. Known issues and honest status

Verified working on the Windows dev machine: Python simulator and 98 tests; C++ runtime builds, matches
Python (golden), runs ~226 fps at 1280x720 on the RTX 4060 (the Python path ~15-20 fps, ~50 ms/frame);
Chrono stock Sedan completes 2 laps within 0.6 m of the centreline with plausible tilt.

**Not verified / open:**
1. **The ROS node has never run on real ROS** (Windows cannot run Noetic); it is only type-checked against
   stand-in headers. First real test is on Ubuntu. Headless **EGL** (Linux GL context) is also untested.
2. Noetic uses Python 3.8: the Python node would need `mujoco==3.2.3` (not tested).
3. **Python/C++ edge case:** if a trajectory has only one of `roll`/`pitch`, Python uses the CSV value for
   that angle and 0 for the other; C++ falls back to the sinusoids for both. Real trajectories have both.
4. IMU lateral accuracy on the real track is moderate (`ay` correlation ~0.77 with `v * yaw_rate`); worse on
   tiny-radius test circles. Cause not identified.
5. Chrono run is a single run (`loop=0`): the car stops at the end; the viewers loop it for display.
6. The Sedan is a road car; the FS-class parameters in `dynamics/adsdv/` are mostly **assumptions**, and not
   the real ADS-DV (its mass/springs/dampers are not public). The JSON edit has not been done.
7. `--profile` (corner-limited speed) in `run_offline.py` was added but its run was stopped before it
   finished, so it is untested.
8. The simulated ground is perfectly flat; no slopes, kerbs or bumps.
9. The vertical IMU axis was wrong in an early version (`az = sqrt(g^2 - ax^2 - ay^2)`); fixed. If you see
   `az` collapsing in corners, that bug is back.

---

## 14. Suggested reading order (about 3-4 hours)

1. `config/*.yaml`, sections 3-4 of this guide, `docs/trajectory_contract.md` (30 min)
2. `track/track_types.py`, then `track/generator.py::_finalize` (30 min)
3. `car/camera_geometry.py` (45 min; derive the homography yourself)
4. `car/driver.py`, `car/trajectory_io.py::TrajectoryDriver` (30 min)
5. `scene/builder.py::build_scene_xml` (30 min)
6. `render/sim.py`, `render/renderer.py`, `render/bboxes.py` (45 min)
7. `cpp/include/conevision_sim/*.h`, then `cpp/src/renderer.cpp`, `bboxes.cpp`, `trajectory.cpp` (45 min)
8. `cpp/ros/sim_node.cpp` (30 min)
9. `dynamics/run_offline.py`, `dynamics/check_trajectory.py` (30 min)
10. `tests/test_camera.py`, `tests/test_bboxes.py`, `cpp/test/test_golden.cpp`: they restate the guarantees.

### Exercises with checkable answers
1. Compute by hand the homography distance error for a 0.3 deg pitch error at 15 m (`d^2/h * da`), then
   confirm with `ground_homography(cam_cfg, car_state=CarState(..., pitch=0.3 deg))` and `pixel_to_ground`.
2. Set `center_jitter_px: 3`, run `preview.py`, and find which function adds the jitter (`compute_bboxes`).
3. Make the camera look 5 deg left (`yaw_deg: 5`); predict which of `ground_homography` / the scene XML /
   the C++ node change, then verify.
4. Run `export_bundle.py --tilt-model --golden --out generated/bundle_tilt`, then `cvsim_test_golden` on it.
5. Break the golden test on purpose (flip the sign of pitch in `Trajectory::mocap_pose`), watch it fail,
   and read the failure output.
