// Bundle loader + per-frame pipeline: pose -> mj_forward -> render -> boxes.
// Port of src/conevision_sim/render/sim.py for the C++ runtime.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include "conevision_sim/bboxes.h"
#include "conevision_sim/config.h"
#include "conevision_sim/renderer.h"
#include "conevision_sim/trajectory.h"

namespace cvsim {

struct Frame {
  long long k = 0;                 // frame index; t = k / fps exactly
  double t = 0;
  RgbImage rgb;
  std::vector<Detection> detections;
  CarState car;
  struct Timing { double step_ms = 0, render_ms = 0, bbox_ms = 0, total_ms = 0; } timing;
};

class Simulation {
 public:
  // bundle_dir: output of scripts/export_bundle.py (scene.xml, trajectory.csv, config/).
  // config_dir: empty = <bundle_dir>/config.
  // seed_override: >= 0 replaces perception.yaml `seed` (bbox noise RNG); < 0 = use the config.
  explicit Simulation(const std::string& bundle_dir, const std::string& config_dir = "",
                      long long seed_override = -1);
  ~Simulation();
  Simulation(const Simulation&) = delete;
  Simulation& operator=(const Simulation&) = delete;

  // Renders frame k (t = k / camera.fps). Reuses `out`'s buffers. Must be called from one thread.
  void frame(long long k, Frame& out);

  const SimConfig& config() const { return cfg_; }
  double fps() const { return cfg_.camera.fps; }
  const mjModel* model() const { return m_; }

 private:
  SimConfig cfg_;
  mjModel* m_ = nullptr;
  mjData* d_ = nullptr;
  std::unique_ptr<Trajectory> traj_;
  std::unique_ptr<CameraRenderer> renderer_;
  std::unique_ptr<BoxComputer> boxes_;
  SegImage seg_;
};

}  // namespace cvsim
