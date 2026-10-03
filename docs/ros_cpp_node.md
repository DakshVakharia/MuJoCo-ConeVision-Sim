# C++ ROS node (`conevision_sim_node`)

Runtime node for 1280x720 @ 120 fps. Python stays the offline tool; it writes a *bundle* that the C++
node loads: `generated/bundle/{scene.xml, trajectory.csv, config/}`.

Target: Ubuntu 20.04, ROS Noetic, NVIDIA GPU. Only the Python bundle export needs Python.

## Build (Ubuntu 20.04)

```bash
# 1. MuJoCo 3.14.0 (not a rosdep key)
cd ~ && wget https://github.com/google-deepmind/mujoco/releases/download/3.14.0/mujoco-3.14.0-linux-x86_64.tar.gz
tar xf mujoco-3.14.0-linux-x86_64.tar.gz
export MUJOCO_DIR=$HOME/mujoco-3.14.0          # put it in ~/.bashrc

# 2. workspace
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone <this repo> conevision_sim            # branch feature/cpp-render-node
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -y  # roscpp, image_transport, compressed_image_transport,
                                                 # foxglove_msgs, yaml-cpp, EGL dev headers ...
catkin_make -DCMAKE_BUILD_TYPE=Release -DMUJOCO_DIR=$MUJOCO_DIR
#   or: catkin build -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

`foxglove_msgs` must be installed (`ros-noetic-foxglove-msgs`; rosdep resolves it if available,
otherwise build it from source in the workspace). The binary gets an RPATH to `$MUJOCO_DIR/lib`, so
`LD_LIBRARY_PATH` is not needed.

## Export the bundle (offline, Python)

```bash
cd ~/catkin_ws/src/conevision_sim
pip3 install -r requirements.txt
python3 scripts/export_bundle.py                 # -> generated/bundle/
```

Re-export whenever the track/seed/car config changes. Topics, fps, JPEG quality, `time_mode`, etc. are
read from `generated/bundle/config/*.yaml` (copied from `config/`), or from `~config_dir` if set.

## Run

```bash
roscore &
roslaunch conevision_sim sim_cpp.launch                       # realtime, bundle in generated/bundle
roslaunch conevision_sim sim_cpp.launch lockstep:=true        # deterministic, publishes /clock, use_sim_time
roslaunch conevision_sim sim_cpp.launch bundle_dir:=/path/to/bundle
rosrun conevision_sim conevision_sim_node _bundle_dir:=/path/to/bundle
```

Private params: `~bundle_dir`, `~config_dir` (default `<bundle>/config`), `~time_mode`
(`realtime|lockstep`, overrides `ros.time_mode`), `~lockstep_throttle` (pace lockstep to
`realtime_factor`). `~seed` is **not** supported by the C++ node (warning only): the bbox seed is the
`seed:` key of the bundle's `perception.yaml`; change it there.

### Topics

| topic (from `perception.yaml`) | type |
|---|---|
| `image_topic` (`/camera/image_raw`) | `sensor_msgs/Image` rgb8 (raw) |
| `image_topic/compressed` | `sensor_msgs/CompressedImage` jpeg, quality = `camera.jpeg_quality` |
| `bbox_topic` | `foxglove_msgs/ImageMarkerArray` (POLYGON per cone, ns = class, id = cone index) |
| `odom_topic`, `imu_topic` | optional (`publish_odom`, `publish_imu`) |
| `/clock` | `publish_clock: true` or lockstep |

Images go through `image_transport`, so every installed plugin (`/compressed`, `/theora`...) appears.
`publish_raw` / `publish_compressed` are informational for the C++ node: a transport costs nothing
until somebody subscribes (the JPEG is only encoded when `/compressed` has subscribers). To really
disable one use the standard `<image_topic>/disable_pub_plugins` param.

### Time modes

* `realtime`: frame k is due at `wall0 + k/(fps*realtime_factor)`. If rendering falls behind, frame
  indices are skipped (sim time keeps tracking wall time) and a throttled warning prints the achieved
  fps. Stamps = node start time + k/fps (or exactly k/fps with `publish_clock`).
* `lockstep`: never skips; `/clock` = k/fps is published before each frame's messages. Runs as fast as
  the GPU allows. Use with `use_sim_time:=true` (the launch arg `lockstep:=true` sets it).

Boxes honour `bbox.latency_ms` (stamped with the image time, published later; wall-clock delay in
realtime, sim-clock delay in lockstep).

## Verify

```bash
rostopic hz /camera/image_raw                    # ~120 Hz
rostopic hz /camera/image_raw/compressed
rostopic echo -n1 /camera/bounding_boxes | head -40
rqt_image_view /camera/image_raw/compressed
```

The node logs every 5 s: achieved fps and mean ms per stage (step / render / bbox / total).

## GPU / EGL

`MUJOCO_GL` is **not** used: the C++ core creates its own headless EGL context and prefers an NVIDIA
device (`EGL_EXT_device_enumeration`). Requirements:

* NVIDIA driver with its GL/EGL userspace (`libnvidia-gl-<ver>`, installed with the driver packages)
  and `libegl1`.
* If another vendor's ICD (Mesa) wins, point the loader at NVIDIA:
  `export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json`
* In containers: `--gpus all -e NVIDIA_DRIVER_CAPABILITIES=graphics,compute,utility`.
* Check with `eglinfo` (package `mesa-utils-extra`) that an NVIDIA device is listed.

## Feeding the depth-perception pipeline

`Monocular-Depth-perception-for-Cones` expects `/zed2/zed_node/left/image_rect_color_compressed`
(CompressedImage) and boxes on `/stereo_cone_perception/bounding_boxes`. With image_transport the JPEG
topic is `<image_topic>/compressed`, so uncomment the remaps in `launch/sim_cpp.launch`:

```xml
<remap from="/camera/image_raw/compressed" to="/zed2/zed_node/left/image_rect_color_compressed"/>
<remap from="/camera/bounding_boxes"        to="/stereo_cone_perception/bounding_boxes"/>
```

(Check with `rostopic list` that the remap took effect; otherwise edit `image_topic`/`bbox_topic` in
the bundle's `perception.yaml`.)

## 120 fps tips

* Raw 1280x720 rgb8 is 2.76 MB/frame = ~330 MB/s at 120 Hz. Over TCPROS (loopback) this is heavy; prefer
  subscribing to `/compressed` (JPEG, ~100-250 KB/frame) unless a node needs raw.
* Raw consumers in the same process should be nodelets: the node publishes a shared pointer
  (`sensor_msgs::ImagePtr`), so intraprocess subscribers (nodelet manager built on this node, or
  `image_transport` in the same executable) get zero copies.
* Lower the JPEG cost with `camera.jpeg_quality` (80-85) - encoding runs on the publish thread.
* GPU cost: `render.msaa_samples: 0` or `4`, and `render.seg_scale: 0.5` (segmentation ~4x cheaper,
  box error <= ~1 px).
* The renderer and publisher run on separate threads with a 3-frame pool; if `skipped` rises in the
  log, check which stage dominates the "mean ms" line.
* Use the `performance` CPU governor and Release build (`-DCMAKE_BUILD_TYPE=Release`).
