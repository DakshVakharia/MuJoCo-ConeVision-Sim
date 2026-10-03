// cvsim_test_golden <bundle_dir>
// Compares C++ detections with the Python reference frames in <bundle>/golden/frame_*.json
// (written by `scripts/export_bundle.py --golden`, noise-free). Runs twice: full-res segmentation
// (tight tolerance) and render.seg_scale = 0.5 (looser). Exit code 0 = pass, 1 = fail.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "../tools/config_override.h"
#include "conevision_sim/simulation.h"

#ifdef _WIN32
// Ask Optimus/PowerXpress laptops to run this exe on the discrete GPU (read by the driver at load).
extern "C" {
__declspec(dllexport) unsigned long NvOptimusEnabled = 1;
__declspec(dllexport) int AmdPowerXpressRequestHighPerformance = 1;
}
#endif

namespace fs = std::filesystem;
using json = nlohmann::json;

namespace {

struct Result {
  int frames = 0, matched = 0, border_skipped = 0, failures = 0;
  double max_box_err = 0, max_range_err = 0, max_pose_err = 0;
};

// Returns the number of failures for this pass; prints details of every failure.
Result run_pass(const std::string& bundle, double seg_scale, double box_tol_px) {
  Result r;
  cvsim_tools::ConfigOverride ov;
  ov.noise_off = true;
  ov.seg_scale = seg_scale;
  const std::string cfg_dir = cvsim_tools::make_config(bundle, ov, "golden");
  cvsim::Simulation sim(bundle, cfg_dir);
  const int min_px = sim.config().bbox.min_visible_pixels;
  const double min_h = sim.config().bbox.min_box_height_px;

  std::vector<fs::path> files;
  for (const auto& e : fs::directory_iterator(fs::path(bundle) / "golden"))
    if (e.path().extension() == ".json") files.push_back(e.path());
  std::sort(files.begin(), files.end());

  cvsim::Frame frame;
  for (const auto& path : files) {
    std::ifstream in(path);
    const json g = json::parse(in);
    const long long k = g["k"];
    sim.frame(k, frame);
    ++r.frames;

    const auto& cs = g["car_state"];
    const double pe = std::max({std::fabs(cs["x"].get<double>() - frame.car.x),
                                std::fabs(cs["y"].get<double>() - frame.car.y),
                                std::fabs(cs["yaw"].get<double>() - frame.car.yaw)});
    r.max_pose_err = std::max(r.max_pose_err, pe);
    if (pe > 1e-3) {
      std::printf("  [frame %lld] FAIL car pose differs by %.2e\n", k, pe);
      ++r.failures;
    }

    std::map<std::pair<std::string, int>, const cvsim::Detection*> mine;
    for (const auto& d : frame.detections) mine[{d.cls, d.index}] = &d;

    for (const auto& pd : g["detections"]) {
      const std::pair<std::string, int> key{pd["cls"].get<std::string>(), pd["index"].get<int>()};
      const int py_vis = pd["visible_pixels"];
      const double py_h = pd["y_max"].get<double>() - pd["y_min"].get<double>();
      auto it = mine.find(key);
      if (it == mine.end()) {
        if (py_vis < 2 * min_px || py_h < min_h + 2) {   // border case: right at a filter threshold
          ++r.border_skipped;
        } else {
          std::printf("  [frame %lld] FAIL missing %s_%d (python: %d px, h %.0f)\n", k, key.first.c_str(),
                      key.second, py_vis, py_h);
          ++r.failures;
        }
        continue;
      }
      const cvsim::Detection& cd = *it->second;
      const double e = std::max({std::fabs(cd.x_min - pd["x_min"].get<double>()),
                                 std::fabs(cd.y_min - pd["y_min"].get<double>()),
                                 std::fabs(cd.x_max - pd["x_max"].get<double>()),
                                 std::fabs(cd.y_max - pd["y_max"].get<double>())});
      const double re = std::fabs(cd.range_m - pd["range_m"].get<double>());
      r.max_box_err = std::max(r.max_box_err, e);
      r.max_range_err = std::max(r.max_range_err, re);
      ++r.matched;
      if (e > box_tol_px || re > 0.01) {
        std::printf("  [frame %lld] FAIL %s_%d box err %.2f px (tol %.1f), range err %.4f m"
                    " (signed dx0 %+.1f dy0 %+.1f dx1 %+.1f dy1 %+.1f, python %d px)\n", k,
                    key.first.c_str(), key.second, e, box_tol_px, re,
                    cd.x_min - pd["x_min"].get<double>(), cd.y_min - pd["y_min"].get<double>(),
                    cd.x_max - pd["x_max"].get<double>(), cd.y_max - pd["y_max"].get<double>(), py_vis);
        ++r.failures;
      }
      mine.erase(it);
    }
    for (const auto& kv : mine) {                       // detections Python does not have
      const cvsim::Detection& cd = *kv.second;
      if (cd.visible_pixels < 2 * min_px || (cd.y_max - cd.y_min) < min_h + 2) {
        ++r.border_skipped;
      } else {
        std::printf("  [frame %lld] FAIL extra %s_%d (c++: %d px)\n", k, cd.cls.c_str(), cd.index,
                    cd.visible_pixels);
        ++r.failures;
      }
    }
  }
  return r;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <bundle_dir>\n", argv[0]);
    return 2;
  }
  const std::string bundle = argv[1];
  int failures = 0;
  try {
    struct Pass { const char* name; double scale, tol; };
    // tolerance for scale s: +-2 px plus 1.5 seg pixels (1.5/s camera px); thin plate corners of far cones can
    // vanish from a half-res raster, so the shrink is slightly more than one seg pixel per side
    for (const Pass p : {Pass{"full-res (seg_scale 1.0)", 1.0, 1.0},
                         Pass{"seg_scale 0.5", 0.5, 2.0 + 1.5 / 0.5}}) {
      std::printf("== %s, box tolerance %.1f px\n", p.name, p.tol);
      const Result r = run_pass(bundle, p.scale, p.tol);
      std::printf("   %d frames, %d detections matched, %d border cases skipped, %d failures\n", r.frames,
                  r.matched, r.border_skipped, r.failures);
      std::printf("   max box error %.2f px, max range error %.2e m, max car pose error %.2e\n",
                  r.max_box_err, r.max_range_err, r.max_pose_err);
      failures += r.failures;
    }
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
  std::printf(failures ? "RESULT: FAIL (%d)\n" : "RESULT: PASS\n", failures);
  return failures ? 1 : 0;
}
