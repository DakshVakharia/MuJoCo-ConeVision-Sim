// Car pose vs sim time, from <bundle>/trajectory.csv written by scripts/export_bundle.py.
#pragma once

#include <string>
#include <vector>

#include "conevision_sim/config.h"

namespace cvsim {

struct CarState {
  double x = 0, y = 0, z = 0, yaw = 0;                    // world frame; yaw wrapped to [-pi, pi)
  double roll = 0, pitch = 0;                             // rad; pitch +ve = nose down, roll +ve = right side down
  double v = 0, yaw_rate = 0;                             // m/s, rad/s
  double ax = 0, ay = 0, az = 9.81;                       // m/s^2; body frame specific force (gravity included)
  double wx = 0, wy = 0, wz = 0;                          // rad/s; body frame angular velocity
};

class Trajectory {
 public:
  // Parses the CSV. Header lines starting with '# ' may include lap_time=..., loop=..., source=...
  // The LAST '# ' line before the data contains column names (by name matching). Supports:
  // legacy 6-column (t,x,y,yaw,v,yaw_rate) and new columns (z,roll,pitch,ax,ay,az,wx,wy,wz).
  // Missing columns use defaults per the contract. If roll AND pitch are absent, body_motion sinusoids apply.
  // Throws on error. Tolerates CRLF line endings.
  Trajectory(const std::string& csv_path, const BodyMotion& body_motion);

  // Pure function of sim time t >= 0. Linear interpolation of all columns at the sample time.
  // loop=1: t' = t mod lap_time (wrap). loop=0: t' = clamp(t, 0, last_t).
  // yaw is wrapped to [-pi, pi) on output (stored unwrapped in CSV for linear interp).
  CarState state_at(double t) const;

  double lap_time() const { return lap_time_; }
  bool loop() const { return loop_; }

  // MuJoCo mocap pose: pos = (x, y, z from CarState), quat (w,x,y,z) of R = Rz(yaw) Ry(pitch) Rx(roll),
  // identical to render/sim.py euler_to_quat.
  static void mocap_pose(const CarState& s, double pos[3], double quat[4]);

 private:
  double lap_time_ = 0;
  bool loop_ = true;
  bool has_roll_ = false, has_pitch_ = false;
  std::vector<double> t_, x_, y_, z_, yaw_, roll_, pitch_, v_, yaw_rate_, ax_, ay_, az_, wx_, wy_, wz_;
  BodyMotion motion_;
};

}  // namespace cvsim
