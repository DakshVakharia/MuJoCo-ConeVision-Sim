# MuJoCo ConeVision Sim: a walkthrough for someone who knows nothing

Read this top to bottom once. It starts with the problem, then follows one camera frame through the
whole system, then explains each part, how to run things, and what is and isn't finished.

---

## 1. The problem in plain words

A driverless race car has to know **where the traffic cones are**. Cones mark the track: blue on the
left, yellow on the right. If the car knows where they are, it can steer between them.

Most teams use LiDAR (a laser scanner) or a stereo camera (two cameras, like two eyes) to measure
distance. **Your earlier project used only one ordinary camera.** One camera can't measure distance
directly, so you used a trick (a *homography*, section 5) that works if the ground is flat.

Two problems appeared:
1. **You no longer have the car or the recorded data**, so you can't test or improve the idea.
2. You can't tell *why* it makes mistakes when you only have real data, because on real data you don't
   know the true distances exactly.

**This project is a fake world that fixes both.** It builds a random race track on the computer, puts a
virtual car with a virtual camera on it, drives around, and gives out the camera pictures *plus the exact
truth* (where every cone really is). Your depth code can be tested on it as often as you like, and every
error can be measured precisely.

---

## 2. The big picture: one frame's journey

Everything the project does, for each picture the camera "takes", in order:

```
 time t  ──►  where is the car at time t?   (a path with timing: the "trajectory")
         ──►  place the car in the 3D world (MuJoCo)
         ──►  draw what the camera sees      (two drawings: colour picture + "who is who" picture)
         ──►  from the "who is who" picture, work out a box around every cone
         ──►  publish: picture, boxes, plus IMU and odometry   (over ROS)
```

That's it. The rest of this document explains each arrow and how the world got built in the first place.

The part that runs once, before the sim starts, is done in **Python**:
make the track, build the 3D world, work out the car's movement, save it all in a folder (the "bundle").
The part that runs for every picture is done in **C++** (a faster language), because at 120 pictures a
second you have only 8 milliseconds per picture.

---

## 3. Vocabulary (look things up here)

| Word | Meaning |
|---|---|
| **Pixel** | One dot of an image. A 1280x720 image has 921,600 of them. |
| **fps** | Frames (pictures) per second. 120 fps = a picture every 8.3 ms. |
| **MuJoCo** | A physics + 3D drawing engine from DeepMind. We use it mainly to *draw* the world from the camera. |
| **MJCF / scene XML** | MuJoCo's text file describing the world (ground, cones, car, camera). Python writes it. |
| **Camera intrinsics** | Numbers describing the camera's lens: focal length (fx, fy) and image centre (cx, cy), in pixels. |
| **Camera extrinsics** | Where the camera sits on the car and which way it points (x, y, z and roll/pitch/yaw angles). |
| **Bounding box** | A rectangle drawn around an object in an image. A "YOLO" detector outputs these. |
| **YOLO** | A neural network that finds objects in images. We don't need it: the sim computes boxes exactly. |
| **Segmentation image** | A second picture where each pixel's colour says *which object* it shows, instead of its real colour. |
| **Homography** | A 3x3 matrix that converts an image pixel on the ground into a ground position in metres. |
| **Roll / pitch / yaw** | Tilt angles: roll = leaning sideways, pitch = nose up/down, yaw = turning left/right. |
| **IMU** | A sensor measuring acceleration and rotation. Tells the car how it's tilting. |
| **Path vs trajectory** | A path is only *where* (a line). A trajectory is path + *when* (position at each moment in time). |
| **Kinematic vs dynamic** | Kinematic = movement described by pure geometry. Dynamic = movement caused by forces (engine, brakes, tyres, springs). |
| **ROS** | A framework where programs ("nodes") send messages ("topics") to each other. Used in robots. |
| **Topic** | A named channel, e.g. `/camera/image_raw`. One node publishes, others subscribe. |
| **Chrono** | A physics library with real vehicle models (suspension, tyres). Planned to make the car tilt realistically. |
| **conda / venv** | Two ways to keep each project's Python packages separate. |
| **CMake** | The tool that builds C++ programs. |
| **Golden test** | A test that checks a new version gives exactly the same answers as a trusted reference. |

---

## 4. The folder map

Repo on GitHub: `DakshVakharia/MuJoCo-ConeVision-Sim`. On your disk it is currently
`C:\Users\daksh\FS-Monocular-Camera-Sim` (the folder rename is still pending).

```
config/          the 5 YAML files with every setting you can change      <- start here
src/conevision_sim/                  the Python package
   track/        makes the random track and cone positions
   scene/        builds the 3D world (cones, ground, sky, trees)
   car/          the car's shape, camera maths, and movement
   render/       draws the camera view, finds boxes, makes ROS messages
scripts/         things you run: preview, viewer, export, ROS node
cpp/             the fast C++ version of the per-frame work + the ROS node
dynamics/        (new) Chrono vehicle physics experiments
docs/            documentation, including this file
tests/           Python tests
assets/          3D models of trees, barriers, ... (free CC0 licence)
generated/       things the programs create (ignored by git)
```

---

## 5. The ideas you need, one at a time

### 5.1 How a camera turns 3D points into pixels
Imagine a cone at position (X, Y, Z) in the *camera's* coordinates (X right, Y down, Z forward).
Its pixel position is:

```
u = fx * X / Z + cx          v = fy * Y / Z + cy
```
Far things (big Z) land near the image centre; near things land far from it. `fx, fy, cx, cy` are the
intrinsics. To get X, Y, Z you first move the cone's world position into the camera's frame using the
car's position and the camera's mounting (extrinsics). All of this is in
`src/conevision_sim/car/camera_geometry.py`. A test checks this maths against MuJoCo's own drawing:
the two agree within 0.8 pixel.

### 5.2 How one camera estimates distance (the homography idea)
If a cone stands on **flat ground** and you know the camera's height `h`, then the lower the cone's base
appears in the picture, the closer it is. A homography encodes that exactly: it maps the pixel of the cone's
*base* (where it touches the ground) to a ground position.

**Why distance errors grow so fast with range:** a pixel row far away covers a lot of ground. The ground
distance is `d = h / tan(angle below the horizon)`. If the camera tilts by a small angle `a` (in
radians) the estimated distance shifts by about `d^2 / h * a`. With the camera 1.15 m high and a tilt of
0.5 degrees:

| Real distance | Error from a 0.5 degree tilt |
|---|---|
| 10 m | about 0.8 m |
| 20 m | about 3 m |
| 30 m | about 7 m |

That's why the car's tilt matters, and why the IMU (section 5.7) is used to correct it.

### 5.3 Boxes from a "who is who" picture
MuJoCo can draw the scene twice:
1. normal colours (the camera image),
2. a **segmentation** image, where every pixel is coloured by the id of the object it shows.

Every cone is named `cone_<colour>_<number>` in the scene. For each cone, the box is simply the
smallest rectangle containing all pixels with that cone's id: `x_min, y_min, x_max, y_max`. Because it
uses what is actually *visible*, a cone hidden behind another cone gets a smaller box, as with a real
detector. Code: `src/conevision_sim/render/bboxes.py` (Python) and `cpp/src/bboxes.cpp` (C++, the same
logic).

Optional realism (set in `config/perception.yaml`): jitter, random misses (dropout), delay (latency).

**Gotcha you will meet:** the box bottom is the near edge of the cone's square base plate, not the exact
middle of the cone. That biases distance estimates by roughly 0.1 m. A real detector has the same
effect.

### 5.4 The track
`track/generator.py` makes a random closed loop (300 m by default) using an open-source generator
(Voronoi-based, MIT licence, from `mvanlobensels/random-track-generator`). It then:
- puts blue cones on the left edge and yellow on the right edge, every 4 m, 3.5 m apart,
- adds four big orange cones at the start line.

The result is a `Track` object: `centerline` (the path along the middle), `cones`, and `start_pose`.
Note: the generator gives a **path**, not a trajectory.

### 5.5 The car's movement today ("kinematic")
`car/driver.py` takes the centreline and works out where the car is at any time `t`: like a train on a
track. It's smooth and repeatable, but it has no physics, so the car never leans. At 8 m/s one lap
takes 37.4 s.

### 5.6 Why a real vehicle model (Chrono) is being added
A real car dips its nose when braking and leans in corners. This changes where cones appear in the
image, and your depth pipeline is supposed to correct for it. To *test* that correction, the sim needs
realistic tilt. Chrono simulates a car with suspension and tyres and a "driver" that follows our
track. Its output (position + tilt over time) goes into `trajectory.csv`, and the camera moves
accordingly.

### 5.7 The IMU
An IMU measures **acceleration** (accelerometer) and **rotation speed** (gyroscope). It cannot tell
gravity from motion, so a parked car reads about +9.81 m/s^2 on the vertical axis, and still about 9.81
in a corner. Your depth pipeline listens to `/zed2/zed_node/imu/data` and uses roll and pitch from it.

We use these sign rules everywhere (written down in `docs/trajectory_contract.md`):
**pitch positive = nose down; roll positive = right side down; yaw positive = turning left.**

---

## 6. The data file that ties it together: `trajectory.csv`

One table of the car's movement over time, 1000 rows per second. Columns:
`t, x, y, z, roll, pitch, yaw, v, yaw_rate, ax, ay, az, wx, wy, wz`.
Two possible producers: the simple Python driver (now) or Chrono (new). Two consumers: the C++
renderer and a Python reader. The full rules are in `docs/trajectory_contract.md`. Having one
agreed file format is what let four helpers work at the same time.

---

## 7. The "bundle": what Python hands to C++

`python scripts/export_bundle.py` creates `generated/bundle/`:
```
scene.xml, meshes/, textures/   the 3D world
trajectory.csv                  the car's movement
track.yaml                      the track
config/                         a copy of the settings
golden/                         reference results from Python, used to test C++
```
The C++ program loads only this folder. It never makes a track itself.

---

## 8. Python vs C++: why both

| | Python version | C++ version |
|---|---|---|
| Time per frame at 1280x720 | about 50 ms (15-20 fps) | about 4-5 ms (200-280 fps) on your RTX 4060 |
| Role | reference, tools, preview | the real-time node |
| Where | `src/conevision_sim/render/` | `cpp/` |

120 fps needs less than 8.3 ms per frame, so only C++ qualifies. The Python version stays as the trusted
reference: the **golden test** (`build/cpp/cvsim_test_golden.exe generated/bundle`) checks that C++ gives
the same boxes as Python (they match to 0.00 pixels).

---

## 9. How to run things (Windows, from the repo folder in Git Bash)

```bash
# Watch the car drive in 3D (a window opens; pick the `front_cam` camera in the left panel)
PYTHONPATH=src .venv/Scripts/python scripts/view_sim.py

# See the camera view with boxes drawn, saved as a video
PYTHONPATH=src .venv/Scripts/python scripts/preview.py --seconds 8 --out generated/preview.mp4

# Make a new random track and see it from above
PYTHONPATH=src .venv/Scripts/python scripts/generate_track.py --seed 7 --plot generated/track.png

# Export the bundle for C++
PYTHONPATH=src .venv/Scripts/python scripts/export_bundle.py --golden

# Build the C++ part, test it, time it
cmd //c "build\\build_cpp.bat"
build/cpp/cvsim_test_golden.exe generated/bundle
build/cpp/cvsim_bench.exe generated/bundle 600

# All Python tests
PYTHONPATH=src .venv/Scripts/python -m pytest tests -q
```

`PYTHONPATH=src` just tells Python where the package lives.

---

## 10. The settings you can play with (`config/`)

| File | Change this to... |
|---|---|
| `camera.yaml` | resolution, lens (fx, fy, cx, cy), where/how the camera is mounted, **fps**, JPEG quality |
| `perception.yaml` | box noise/dropout/latency, box colours, ROS topic names, `time_mode`, IMU/odometry on/off |
| `track.yaml` | seed (same seed = same track), length, width, cone spacing |
| `scene.yaml` | ground texture, sun, shadows (they halve the speed), number of trees |
| `car.yaml` | car size, speed, optional `tilt_model` (cheap fake tilt without Chrono) |

Good first experiment: in `camera.yaml` change `pitch_deg` from 4 to 10 and re-run `preview.py`. You'll
see the horizon move up.

---

## 11. ROS: how it talks to your depth code

ROS is a way for programs to exchange messages. Our node publishes:

| Topic | What | Used by |
|---|---|---|
| `/camera/image_raw` (+ `/compressed`) | the picture | your pipeline's image input |
| `/camera/bounding_boxes` | one polygon per cone, with its colour | `cone_base_detector` |
| `/zed2/zed_node/imu/data` | tilt + motion | `h_matrix_publisher` (tilt correction) |
| `/slam/state` | the car's position | `track_map_visualizer` |

Run the C++ node with `roslaunch conevision_sim sim_cpp.launch` on **Ubuntu 20.04 + ROS Noetic**.
ROS Noetic doesn't run on Windows, so the node has only been checked there against stand-in headers, not
run for real. Build instructions: `docs/ros_cpp_node.md`.

Two time modes: `realtime` (drops frames if the computer is too slow) and `lockstep` (never drops, time
advances exactly 1/fps per frame).

---

## 12. What is verified, and what is not

**Verified by running it on this laptop**
- The Python simulator and its tests (98 pass).
- The C++ render code builds, matches Python exactly, and runs at 200+ fps.
- The new trajectory file readers (C++ and Python), including the IMU data.

**Not yet verified**
- The ROS node running on real ROS (needs your Ubuntu machine).
- Headless rendering on Linux ("EGL").
- The Chrono vehicle run: an agent is working on it as this is written. Check `dynamics/README.md`.

**Honest limitations**
- The ground is perfectly flat (no slopes or bumps).
- The vehicle parameters are typical Formula Student values, many of them assumptions
  (`dynamics/adsdv/REVIEW.md`). It is **not** the real ADS-DV car.
- The random tracks are fairly round, with few tight hairpins.

---

## 13. A note on how this was built (read before an interview)

Most code was written by AI helpers working in parallel; I reviewed their output and ran the checks.
Several of them had errors that review caught, which is itself worth knowing:
- An early test used the cone's centre where the box actually measures the base plate edge.
- An IMU formula made the vertical reading collapse in corners (fixed, `trajectory_io.py`).
- A helper's report gave wrong statistics that its own output file contradicted.
- A Chrono helper blamed Windows path slashes for a crash; the real cause was loading the wrong file.

The lesson: **don't trust a summary; run the check.** If you can explain sections 5, 6 and 8 from memory,
you understand the project well enough to defend it.

---

## 14. Suggested reading order for the code

1. `config/*.yaml` and `docs/trajectory_contract.md`
2. `track/track_types.py`, then `track/generator.py`
3. `car/camera_geometry.py` (the maths), then `car/driver.py`
4. `scene/builder.py`
5. `render/sim.py`, `render/renderer.py`, `render/bboxes.py`
6. `car/trajectory_io.py`
7. `cpp/src/simulation.cpp`, `cpp/src/bboxes.cpp`, `cpp/ros/sim_node.cpp`
8. `tests/`: they show what each part promises.

## 15. Exercises (learn by changing things)

1. Change `pitch_deg` in `camera.yaml`; predict what happens before you run `preview.py`.
2. Set `center_jitter_px: 3` in `perception.yaml` and watch the boxes wobble.
3. Run `generate_track.py` with five seeds; find the one with the sharpest corner.
4. In `scene.yaml` turn `shadows: true`; measure the speed change with `cvsim_bench`.
5. Enable the `tilt_model` in `car.yaml`, export, and plot roll against time.
6. Compute by hand the distance error for a 0.3 degree tilt at 15 m (use section 5.2), then check it
   with the Python code.

---

## 16. What comes next

1. Finish the Chrono run and compare its tilt with the expected 0.4 to 0.5 degrees per g.
2. Run the C++ node on Ubuntu and feed your depth pipeline.
3. Write an **evaluation script**: compare the pipeline's distance estimates with the true ones, per range,
   with tilt correction on and off. That table is the real result of the whole project.
4. Later: make the car drive from its own camera (closed-loop), which needs Chrono running live.
