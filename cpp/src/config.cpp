// YAML config loading. Key names match config/camera.yaml, perception.yaml and car.yaml.
#include "fs_mono_cam_sim/config.h"

#include <stdexcept>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace fsim {
namespace {

YAML::Node load_file(const std::string& path) {
  try {
    return YAML::LoadFile(path);
  } catch (const std::exception& e) {
    throw std::runtime_error("cannot read config '" + path + "': " + e.what());
  }
}

// Value at node[key], or `def` when the key is missing or `null` (Python's None).
template <typename T>
T get(const YAML::Node& node, const char* key, const T& def) {
  if (!node || !node.IsMap()) return def;
  const YAML::Node v = node[key];
  if (!v || v.IsNull()) return def;
  return v.as<T>();
}

}  // namespace

SimConfig load_config(const std::string& config_dir) {
  const std::string dir = config_dir.empty() || config_dir.back() == '/' || config_dir.back() == '\\'
                              ? config_dir : config_dir + "/";
  SimConfig c;

  const YAML::Node cam = load_file(dir + "camera.yaml")["camera"];
  c.camera.name = get<std::string>(cam, "name", c.camera.name);
  c.camera.width = get<int>(cam, "width", c.camera.width);
  c.camera.height = get<int>(cam, "height", c.camera.height);
  c.camera.fps = get<double>(cam, "fps", c.camera.fps);
  c.camera.jpeg_quality = get<int>(cam, "jpeg_quality", c.camera.jpeg_quality);

  const YAML::Node per = load_file(dir + "perception.yaml");
  const YAML::Node bb = per["bbox"];
  c.bbox.min_visible_pixels = get<int>(bb, "min_visible_pixels", c.bbox.min_visible_pixels);
  c.bbox.min_box_height_px = get<double>(bb, "min_box_height_px", c.bbox.min_box_height_px);
  c.bbox.max_range_m = get<double>(bb, "max_range_m", c.bbox.max_range_m);
  c.bbox.center_jitter_px = get<double>(bb, "center_jitter_px", c.bbox.center_jitter_px);
  c.bbox.size_jitter_frac = get<double>(bb, "size_jitter_frac", c.bbox.size_jitter_frac);
  c.bbox.dropout_prob = get<double>(bb, "dropout_prob", c.bbox.dropout_prob);
  c.bbox.latency_ms = get<double>(bb, "latency_ms", c.bbox.latency_ms);
  if (bb && bb["outline_colors"] && bb["outline_colors"].IsMap()) {
    for (const auto& kv : bb["outline_colors"]) {
      const auto v = kv.second.as<std::vector<float>>();
      if (v.size() == 3) c.bbox.outline_colors[kv.first.as<std::string>()] = {v[0], v[1], v[2]};
    }
  }

  const YAML::Node rd = per["render"];
  c.render.msaa_samples = get<int>(rd, "msaa_samples", c.render.msaa_samples);
  c.render.seg_scale = get<double>(rd, "seg_scale", c.render.seg_scale);
  if (c.render.seg_scale <= 0.0 || c.render.seg_scale > 1.0)
    throw std::runtime_error("render.seg_scale must be in (0, 1]");
  c.seed = get<unsigned long long>(per, "seed", c.seed);

  const YAML::Node ros = per["ros"];
  c.ros.frame_id = get<std::string>(ros, "frame_id", c.ros.frame_id);
  c.ros.publish_raw = get<bool>(ros, "publish_raw", c.ros.publish_raw);
  c.ros.image_topic = get<std::string>(ros, "image_topic", c.ros.image_topic);
  c.ros.publish_compressed = get<bool>(ros, "publish_compressed", c.ros.publish_compressed);
  c.ros.bbox_topic = get<std::string>(ros, "bbox_topic", c.ros.bbox_topic);
  c.ros.publish_odom = get<bool>(ros, "publish_odom", c.ros.publish_odom);
  c.ros.odom_topic = get<std::string>(ros, "odom_topic", c.ros.odom_topic);
  c.ros.publish_imu = get<bool>(ros, "publish_imu", c.ros.publish_imu);
  c.ros.imu_topic = get<std::string>(ros, "imu_topic", c.ros.imu_topic);
  c.ros.realtime_factor = get<double>(ros, "realtime_factor", c.ros.realtime_factor);
  c.ros.publish_clock = get<bool>(ros, "publish_clock", c.ros.publish_clock);
  c.ros.time_mode = get<std::string>(ros, "time_mode", c.ros.time_mode);

  const YAML::Node bm = load_file(dir + "car.yaml")["car"]["body_motion"];
  c.body_motion.pitch_amplitude_deg = get<double>(bm, "pitch_amplitude_deg", c.body_motion.pitch_amplitude_deg);
  c.body_motion.roll_amplitude_deg = get<double>(bm, "roll_amplitude_deg", c.body_motion.roll_amplitude_deg);
  c.body_motion.frequency_hz = get<double>(bm, "frequency_hz", c.body_motion.frequency_hz);
  return c;
}

}  // namespace fsim
