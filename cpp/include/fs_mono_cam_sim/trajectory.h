// Car pose vs sim time, from <bundle>/trajectory.csv written by scripts/export_bundle.py.
#pragma once

#include <string>
#include <vector>

#include "fs_mono_cam_sim/config.h"

namespace fsim {

struct CarState {
  double x = 0, y = 0, yaw = 0;   // world frame; yaw wrapped to [-pi, pi)
  double roll = 0, pitch = 0;     // rad; pitch +ve = nose down
  double v = 0, yaw_rate = 0;     // m/s, rad/s
};

class Trajectory {
 public:
  // Parses the CSV (header `# lap_time=...`, rows t,x,y,yaw_unwrapped,v,yaw_rate). Throws on error.
  Trajectory(const std::string& csv_path, const BodyMotion& body_motion);

  // Pure function of sim time t >= 0. Linear interpolation at (t mod lap_time); pitch/roll as in
  // car/driver.py:  w = 2*pi*f;  pitch = A_p*sin(w*t);  roll = A_r*sin(1.31*w*t + 0.9)  (A in rad).
  CarState state_at(double t) const;

  double lap_time() const { return lap_time_; }

  // MuJoCo mocap pose for the car body (z = 0): quat (w,x,y,z) of R = Rz(yaw) Ry(pitch) Rx(roll),
  // identical to render/sim.py euler_to_quat.
  static void mocap_pose(const CarState& s, double pos[3], double quat[4]);

 private:
  double lap_time_ = 0;
  std::vector<double> t_, x_, y_, yaw_, v_, yaw_rate_;
  BodyMotion motion_;
};

}  // namespace fsim
