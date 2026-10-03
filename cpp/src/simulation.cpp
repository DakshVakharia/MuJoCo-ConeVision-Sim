// Per-frame pipeline, port of render/sim.py: pose -> mj_forward -> render -> boxes.
#include "conevision_sim/simulation.h"

#include <chrono>
#include <stdexcept>

namespace cvsim {
namespace {
using Clock = std::chrono::steady_clock;
double ms_between(Clock::time_point a, Clock::time_point b) {
  return std::chrono::duration<double, std::milli>(b - a).count();
}
}  // namespace

Simulation::Simulation(const std::string& bundle_dir, const std::string& config_dir,
                       long long seed_override) {
  const std::string root = bundle_dir.empty() || bundle_dir.back() == '/' || bundle_dir.back() == '\\'
                               ? bundle_dir : bundle_dir + "/";
  cfg_ = load_config(config_dir.empty() ? root + "config" : config_dir);
  if (seed_override >= 0) cfg_.seed = static_cast<unsigned long long>(seed_override);

  char err[1000] = "";
  m_ = mj_loadXML((root + "scene.xml").c_str(), nullptr, err, sizeof(err));
  if (!m_) throw std::runtime_error(std::string("cannot load scene.xml: ") + err);
  d_ = mj_makeData(m_);
  if (m_->nmocap < 1) throw std::runtime_error("scene has no mocap body for the car");

  traj_ = std::make_unique<Trajectory>(root + "trajectory.csv", cfg_.body_motion);
  renderer_ = std::make_unique<CameraRenderer>(m_, cfg_);
  boxes_ = std::make_unique<BoxComputer>(m_, cfg_.bbox, cfg_.seed);
}

Simulation::~Simulation() {
  boxes_.reset();
  renderer_.reset();       // frees GL objects while the model still exists
  if (d_) mj_deleteData(d_);
  if (m_) mj_deleteModel(m_);
}

void Simulation::frame(long long k, Frame& out) {
  const auto t0 = Clock::now();
  out.k = k;
  out.t = static_cast<double>(k) / cfg_.camera.fps;
  out.car = traj_->state_at(out.t);
  double pos[3], quat[4];
  Trajectory::mocap_pose(out.car, pos, quat);
  for (int i = 0; i < 3; ++i) d_->mocap_pos[i] = pos[i];
  for (int i = 0; i < 4; ++i) d_->mocap_quat[i] = quat[i];
  mj_forward(m_, d_);
  const auto t1 = Clock::now();

  renderer_->render(d_, out.rgb, seg_);
  const auto t2 = Clock::now();

  out.detections = boxes_->compute(d_, renderer_->camera_id(), seg_, cfg_.camera.width, cfg_.camera.height);
  const auto t3 = Clock::now();

  out.timing.step_ms = ms_between(t0, t1);
  out.timing.render_ms = ms_between(t1, t2);
  out.timing.bbox_ms = ms_between(t2, t3);
  out.timing.total_ms = ms_between(t0, t3);
}

}  // namespace cvsim
