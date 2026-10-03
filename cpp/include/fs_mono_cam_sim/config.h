// Runtime configuration, read from the same config/*.yaml files the Python side uses.
#pragma once

#include <array>
#include <map>
#include <string>

namespace fsim {

struct CameraConfig {            // config/camera.yaml -> camera:
  std::string name = "front_cam";
  int width = 1280, height = 720;
  double fps = 120.0;
  int jpeg_quality = 90;
};

struct BBoxConfig {              // config/perception.yaml -> bbox:
  int min_visible_pixels = 25;
  double min_box_height_px = 6.0;
  double max_range_m = 60.0;
  double center_jitter_px = 0.0;
  double size_jitter_frac = 0.0;
  double dropout_prob = 0.0;
  double latency_ms = 0.0;
  std::map<std::string, std::array<float, 3>> outline_colors;  // class -> rgb 0..1
};

struct RenderConfig {            // config/perception.yaml -> render:
  int msaa_samples = -1;         // -1 = use the scene XML value (Python: null)
  double seg_scale = 1.0;        // segmentation resolution factor (0.5 = half res, boxes scaled back)
};

struct RosConfig {               // config/perception.yaml -> ros:
  std::string frame_id = "front_cam_optical";
  bool publish_raw = true;
  std::string image_topic = "/camera/image_raw";   // image_transport base topic
  bool publish_compressed = true;                  // (Python uses compressed_topic; C++ uses <image_topic>/compressed)
  std::string bbox_topic = "/camera/bounding_boxes";
  bool publish_odom = false;
  std::string odom_topic = "/sim/ground_truth/odom";
  bool publish_imu = false;
  std::string imu_topic = "/sim/imu";
  double realtime_factor = 1.0;
  bool publish_clock = false;
  std::string time_mode = "realtime";              // realtime | lockstep
};

struct BodyMotion {              // config/car.yaml -> car: body_motion:
  double pitch_amplitude_deg = 0.0;
  double roll_amplitude_deg = 0.0;
  double frequency_hz = 1.5;
};

struct SimConfig {
  CameraConfig camera;
  BBoxConfig bbox;
  RenderConfig render;
  RosConfig ros;
  BodyMotion body_motion;
  unsigned long long seed = 0;   // perception.yaml top-level `seed` (bbox noise)
};

// Load camera.yaml, perception.yaml and car.yaml from `config_dir`. Missing keys keep defaults.
// Throws std::runtime_error on unreadable files.
SimConfig load_config(const std::string& config_dir);

}  // namespace fsim
