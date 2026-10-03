// Port of CenterlineDriver.state_at(): the lap is pre-sampled at 1 kHz in trajectory.csv, so we just
// interpolate. Pitch/roll body motion is a pure function of t and is recomputed here.
#include "fs_mono_cam_sim/trajectory.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace fsim {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;
}  // namespace

Trajectory::Trajectory(const std::string& csv_path, const BodyMotion& body_motion) : motion_(body_motion) {
  std::ifstream in(csv_path);
  if (!in) throw std::runtime_error("cannot open trajectory '" + csv_path + "'");
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    if (line[0] == '#') {
      const auto p = line.find("lap_time=");
      if (p != std::string::npos) lap_time_ = std::stod(line.substr(p + 9));
      continue;
    }
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream ss(line);
    double t, x, y, yaw, v, yr;
    if (!(ss >> t >> x >> y >> yaw >> v >> yr)) throw std::runtime_error("bad trajectory row: " + line);
    t_.push_back(t); x_.push_back(x); y_.push_back(y);
    yaw_.push_back(yaw); v_.push_back(v); yaw_rate_.push_back(yr);
  }
  if (t_.size() < 2) throw std::runtime_error("trajectory has fewer than 2 rows");
  if (lap_time_ <= 0) lap_time_ = t_.back();
}

CarState Trajectory::state_at(double t) const {
  // t mod lap_time (Python's % is always non-negative)
  double tt = std::fmod(t, lap_time_);
  if (tt < 0) tt += lap_time_;
  // segment [i, i+1] containing tt
  size_t i = static_cast<size_t>(std::upper_bound(t_.begin(), t_.end(), tt) - t_.begin());
  i = i == 0 ? 0 : std::min(i - 1, t_.size() - 2);
  const double span = t_[i + 1] - t_[i];
  const double a = span > 0 ? std::min(std::max((tt - t_[i]) / span, 0.0), 1.0) : 0.0;
  auto lerp = [&](const std::vector<double>& v) { return v[i] + a * (v[i + 1] - v[i]); };

  CarState s;
  s.x = lerp(x_);
  s.y = lerp(y_);
  s.v = lerp(v_);
  s.yaw_rate = lerp(yaw_rate_);
  const double yaw = lerp(yaw_);                    // unwrapped in the CSV -> safe to interpolate
  double w = std::fmod(yaw + kPi, 2 * kPi);         // wrap to [-pi, pi)
  if (w < 0) w += 2 * kPi;
  s.yaw = w - kPi;

  const double om = 2 * kPi * motion_.frequency_hz;
  s.pitch = motion_.pitch_amplitude_deg * kDeg2Rad * std::sin(om * t);
  s.roll = motion_.roll_amplitude_deg * kDeg2Rad * std::sin(1.31 * om * t + 0.9);
  return s;
}

void Trajectory::mocap_pose(const CarState& s, double pos[3], double quat[4]) {
  pos[0] = s.x; pos[1] = s.y; pos[2] = 0.0;
  // R = Rz(yaw) Ry(pitch) Rx(roll) as a quaternion (w, x, y, z); same as sim.py euler_to_quat.
  const double cy = std::cos(s.yaw / 2), sy = std::sin(s.yaw / 2);
  const double cp = std::cos(s.pitch / 2), sp = std::sin(s.pitch / 2);
  const double cr = std::cos(s.roll / 2), sr = std::sin(s.roll / 2);
  quat[0] = cr * cp * cy + sr * sp * sy;
  quat[1] = sr * cp * cy - cr * sp * sy;
  quat[2] = cr * sp * cy + sr * cp * sy;
  quat[3] = cr * cp * sy - sr * sp * cy;
}

}  // namespace fsim
