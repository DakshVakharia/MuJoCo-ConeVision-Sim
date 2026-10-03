// fsim_bench <bundle_dir> [frames=600] [seg_scale]
// Runs the full per-frame pipeline and prints per-stage timings. Frame 60 is saved as bench_frame60.ppm.
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <numeric>
#include <string>
#include <vector>

#include "../src/gl_internal.h"
#include "config_override.h"
#include "fs_mono_cam_sim/simulation.h"

#ifdef _WIN32
// Ask Optimus/PowerXpress laptops to run this exe on the discrete GPU (read by the driver at load).
extern "C" {
__declspec(dllexport) unsigned long NvOptimusEnabled = 1;
__declspec(dllexport) int AmdPowerXpressRequestHighPerformance = 1;
}
#endif

namespace {

struct Stat {
  std::vector<double> v;
  void add(double x) { v.push_back(x); }
  double mean() const { return v.empty() ? 0 : std::accumulate(v.begin(), v.end(), 0.0) / v.size(); }
  double p95() const {
    if (v.empty()) return 0;
    std::vector<double> s = v;
    std::sort(s.begin(), s.end());
    return s[std::min(s.size() - 1, static_cast<size_t>(0.95 * s.size()))];
  }
  void print(const char* name) const { std::printf("  %-8s mean %7.3f ms   p95 %7.3f ms\n", name, mean(), p95()); }
};

void write_ppm(const char* path, const fsim::RgbImage& im) {
  if (FILE* f = std::fopen(path, "wb")) {
    std::fprintf(f, "P6\n%d %d\n255\n", im.width, im.height);
    std::fwrite(im.data.data(), 1, im.data.size(), f);
    std::fclose(f);
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <bundle_dir> [frames=600] [seg_scale]\n", argv[0]);
    return 2;
  }
  const int frames = argc > 2 ? std::atoi(argv[2]) : 600;
  try {
    fsim_tools::ConfigOverride ov;
    if (argc > 3) ov.seg_scale = std::atof(argv[3]);
    const std::string cfg_dir = argc > 3 ? fsim_tools::make_config(argv[1], ov, "bench") : "";
    fsim::Simulation sim(argv[1], cfg_dir);
    std::printf("GL_RENDERER: %s\n", fsim::gl_renderer_string().c_str());
    std::printf("camera %dx%d, seg_scale %.2f, %d frames\n", sim.config().camera.width,
                sim.config().camera.height, sim.config().render.seg_scale, frames);

    fsim::Frame f;
    for (int k = 0; k < 30; ++k) sim.frame(k, f);   // warm-up (shader compile, driver caches)

    Stat step, render, bbox, total;
    size_t ndet = 0;
    const auto t0 = std::chrono::steady_clock::now();
    for (int k = 0; k < frames; ++k) {
      sim.frame(k, f);
      step.add(f.timing.step_ms);
      render.add(f.timing.render_ms);
      bbox.add(f.timing.bbox_ms);
      total.add(f.timing.total_ms);
      ndet += f.detections.size();
      if (k == 60) {
        write_ppm("bench_frame60.ppm", f.rgb);
        std::printf("frame 60: %zu detections (saved bench_frame60.ppm)\n", f.detections.size());
      }
    }
    const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::printf("per-stage timings:\n");
    step.print("step");
    render.print("render");
    bbox.print("bbox");
    total.print("total");
    std::printf("achieved %.1f fps (%.1f detections/frame avg); target 120 fps = 8.33 ms\n",
                frames / wall, frames ? static_cast<double>(ndet) / frames : 0.0);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
  return 0;
}
