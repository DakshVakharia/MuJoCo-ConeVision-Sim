// Test/bench helper: copy <bundle>/config to a temp dir with perception.yaml patched
// (noise off and/or a different render.seg_scale), so the Simulation can be built with it.
#pragma once

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <regex>
#include <sstream>
#include <string>

namespace cvsim_tools {

namespace fs = std::filesystem;

struct ConfigOverride {
  bool noise_off = false;
  double seg_scale = -1.0;   // < 0: keep the file's value
};

// Returns the new config dir (created under the system temp dir, tagged by `tag`).
inline std::string make_config(const std::string& bundle_dir, const ConfigOverride& ov, const std::string& tag) {
  const fs::path src = fs::path(bundle_dir) / "config";
  const fs::path dst = fs::temp_directory_path() / ("cvsim_cfg_" + tag);
  fs::create_directories(dst);
  for (const char* f : {"camera.yaml", "car.yaml"})
    fs::copy_file(src / f, dst / f, fs::copy_options::overwrite_existing);

  std::ifstream in(src / "perception.yaml");
  std::stringstream ss;
  ss << in.rdbuf();
  std::string txt = ss.str();
  if (ov.noise_off) {
    for (const char* key : {"center_jitter_px", "size_jitter_frac", "dropout_prob"})
      txt = std::regex_replace(txt, std::regex(std::string("(\\n\\s*") + key + ":)[^\\n#]*"), "$1 0.0 ");
  }
  if (ov.seg_scale > 0) {
    txt = std::regex_replace(txt, std::regex("\\n\\s*seg_scale:[^\\n]*"), "");   // drop an existing one
    char buf[64];
    std::snprintf(buf, sizeof buf, "$1\n  seg_scale: %g", ov.seg_scale);
    txt = std::regex_replace(txt, std::regex("(\\nrender:)"), buf);
  }
  std::ofstream(dst / "perception.yaml") << txt;
  return dst.string();
}

}  // namespace cvsim_tools
