// Reads trajectory.csv with flexible column format (by-name matching from header line).
// Supports both legacy 6-column and new full format with optional columns.
// Line endings: both LF and CRLF are tolerated.
#include "conevision_sim/trajectory.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace cvsim {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;

// Strips trailing '\r' from a string (CRLF support).
void trim_crlf(std::string& s) {
  if (!s.empty() && s.back() == '\r') s.pop_back();
}

// Parses the header line "# t,x,y,..." and returns a map from column name to column index.
std::map<std::string, int> parse_header(const std::string& header_line) {
  std::map<std::string, int> cols;
  // Remove the "# " prefix
  std::string names = header_line;
  if (names.size() >= 2 && names[0] == '#' && names[1] == ' ') {
    names = names.substr(2);
  } else if (names.size() >= 1 && names[0] == '#') {
    names = names.substr(1);
  }
  trim_crlf(names);

  std::istringstream ss(names);
  std::string col;
  int idx = 0;
  while (std::getline(ss, col, ',')) {
    // Trim whitespace from col
    col.erase(0, col.find_first_not_of(" \t"));
    col.erase(col.find_last_not_of(" \t") + 1);
    if (!col.empty()) {
      cols[col] = idx;
      ++idx;
    }
  }
  return cols;
}
}  // namespace

Trajectory::Trajectory(const std::string& csv_path, const BodyMotion& body_motion)
    : motion_(body_motion), loop_(true) {
  std::ifstream in(csv_path);
  if (!in) throw std::runtime_error("cannot open trajectory '" + csv_path + "'");

  std::string line;
  std::string header_line;
  std::vector<std::string> data_lines;

  // First pass: find header lines and data
  while (std::getline(in, line)) {
    trim_crlf(line);
    if (line.empty()) continue;
    if (line[0] == '#') {
      // Parse special header fields
      if (line.find("lap_time=") != std::string::npos) {
        const auto p = line.find("lap_time=");
        lap_time_ = std::stod(line.substr(p + 9));
      }
      if (line.find("loop=") != std::string::npos) {
        const auto p = line.find("loop=");
        const int loop_val = std::stoi(line.substr(p + 5));
        loop_ = (loop_val != 0);
      }
      // Remember the last "# " line for column headers
      if (line.size() >= 2 && line[0] == '#' && line[1] == ' ') {
        header_line = line;
      }
      continue;
    }
    data_lines.push_back(line);
  }

  if (header_line.empty()) throw std::runtime_error("no column header found (expected '# t,x,y,...')");
  if (data_lines.empty()) throw std::runtime_error("no data rows found");

  // Parse column mapping
  const auto cols = parse_header(header_line);

  // Check required columns
  if (cols.find("t") == cols.end() || cols.find("x") == cols.end() ||
      cols.find("y") == cols.end() || cols.find("yaw") == cols.end()) {
    throw std::runtime_error("trajectory missing required columns: need t, x, y, yaw");
  }

  const int col_t = cols.at("t");
  const int col_x = cols.at("x");
  const int col_y = cols.at("y");
  const int col_yaw = cols.at("yaw");
  const int col_z = cols.find("z") != cols.end() ? cols.at("z") : -1;
  const int col_roll = cols.find("roll") != cols.end() ? cols.at("roll") : -1;
  const int col_pitch = cols.find("pitch") != cols.end() ? cols.at("pitch") : -1;
  const int col_v = cols.find("v") != cols.end() ? cols.at("v") : -1;
  const int col_yaw_rate = cols.find("yaw_rate") != cols.end() ? cols.at("yaw_rate") : -1;
  const int col_ax = cols.find("ax") != cols.end() ? cols.at("ax") : -1;
  const int col_ay = cols.find("ay") != cols.end() ? cols.at("ay") : -1;
  const int col_az = cols.find("az") != cols.end() ? cols.at("az") : -1;
  const int col_wx = cols.find("wx") != cols.end() ? cols.at("wx") : -1;
  const int col_wy = cols.find("wy") != cols.end() ? cols.at("wy") : -1;
  const int col_wz = cols.find("wz") != cols.end() ? cols.at("wz") : -1;

  has_roll_ = (col_roll >= 0);
  has_pitch_ = (col_pitch >= 0);

  // Parse data rows
  for (const auto& data_line : data_lines) {
    std::replace(const_cast<std::string&>(data_line).begin(), const_cast<std::string&>(data_line).end(), ',', ' ');
    std::istringstream ss(data_line);
    std::vector<double> fields;
    double val;
    while (ss >> val) {
      fields.push_back(val);
    }

    if (fields.size() < static_cast<size_t>(col_yaw + 1)) {
      throw std::runtime_error("trajectory row has too few columns: " + data_line);
    }

    const double t = fields[col_t];
    const double x = fields[col_x];
    const double y = fields[col_y];
    const double yaw = fields[col_yaw];
    const double z = col_z >= 0 && col_z < static_cast<int>(fields.size()) ? fields[col_z] : 0.0;
    const double roll = col_roll >= 0 && col_roll < static_cast<int>(fields.size()) ? fields[col_roll] : 0.0;
    const double pitch = col_pitch >= 0 && col_pitch < static_cast<int>(fields.size()) ? fields[col_pitch] : 0.0;
    const double v = col_v >= 0 && col_v < static_cast<int>(fields.size()) ? fields[col_v] : 0.0;
    const double yaw_rate = col_yaw_rate >= 0 && col_yaw_rate < static_cast<int>(fields.size()) ? fields[col_yaw_rate] : 0.0;
    const double ax = col_ax >= 0 && col_ax < static_cast<int>(fields.size()) ? fields[col_ax] : 0.0;
    const double ay = col_ay >= 0 && col_ay < static_cast<int>(fields.size()) ? fields[col_ay] : 0.0;
    const double az = col_az >= 0 && col_az < static_cast<int>(fields.size()) ? fields[col_az] : 9.81;
    const double wx = col_wx >= 0 && col_wx < static_cast<int>(fields.size()) ? fields[col_wx] : 0.0;
    const double wy = col_wy >= 0 && col_wy < static_cast<int>(fields.size()) ? fields[col_wy] : 0.0;
    const double wz = col_wz >= 0 && col_wz < static_cast<int>(fields.size()) ? fields[col_wz] : yaw_rate;

    t_.push_back(t);
    x_.push_back(x);
    y_.push_back(y);
    z_.push_back(z);
    yaw_.push_back(yaw);
    roll_.push_back(roll);
    pitch_.push_back(pitch);
    v_.push_back(v);
    yaw_rate_.push_back(yaw_rate);
    ax_.push_back(ax);
    ay_.push_back(ay);
    az_.push_back(az);
    wx_.push_back(wx);
    wy_.push_back(wy);
    wz_.push_back(wz);
  }

  if (t_.size() < 2) throw std::runtime_error("trajectory has fewer than 2 rows");
  if (lap_time_ <= 0) lap_time_ = t_.back();
}

CarState Trajectory::state_at(double t) const {
  // Compute sample time based on loop mode
  double tt;
  if (loop_) {
    // loop=1: wrap around using modulo
    tt = std::fmod(t, lap_time_);
    if (tt < 0) tt += lap_time_;
  } else {
    // loop=0: clamp to [0, last_t]
    tt = std::max(0.0, std::min(t, t_.back()));
  }

  // Find segment [i, i+1] containing tt
  size_t i = static_cast<size_t>(std::upper_bound(t_.begin(), t_.end(), tt) - t_.begin());
  i = i == 0 ? 0 : std::min(i - 1, t_.size() - 2);
  const double span = t_[i + 1] - t_[i];
  const double a = span > 0 ? std::min(std::max((tt - t_[i]) / span, 0.0), 1.0) : 0.0;

  auto lerp = [&](const std::vector<double>& v) { return v[i] + a * (v[i + 1] - v[i]); };

  CarState s;
  s.x = lerp(x_);
  s.y = lerp(y_);
  s.z = lerp(z_);
  s.v = lerp(v_);
  s.yaw_rate = lerp(yaw_rate_);
  s.ax = lerp(ax_);
  s.ay = lerp(ay_);
  s.az = lerp(az_);
  s.wx = lerp(wx_);
  s.wy = lerp(wy_);
  s.wz = lerp(wz_);

  // Interpolate yaw (unwrapped in CSV, so linear interpolation is safe)
  const double yaw = lerp(yaw_);
  // Wrap yaw to [-pi, pi)
  double w = std::fmod(yaw + kPi, 2 * kPi);
  if (w < 0) w += 2 * kPi;
  s.yaw = w - kPi;

  // Roll and pitch: use CSV values if present, otherwise apply body_motion sinusoids
  if (has_roll_ && has_pitch_) {
    s.roll = lerp(roll_);
    s.pitch = lerp(pitch_);
  } else {
    // Legacy: apply body_motion sinusoids (only if both roll AND pitch are absent)
    const double om = 2 * kPi * motion_.frequency_hz;
    s.pitch = motion_.pitch_amplitude_deg * kDeg2Rad * std::sin(om * t);
    s.roll = motion_.roll_amplitude_deg * kDeg2Rad * std::sin(1.31 * om * t + 0.9);
  }

  return s;
}

void Trajectory::mocap_pose(const CarState& s, double pos[3], double quat[4]) {
  pos[0] = s.x; pos[1] = s.y; pos[2] = s.z;
  // R = Rz(yaw) Ry(pitch) Rx(roll) as a quaternion (w, x, y, z); same as sim.py euler_to_quat.
  const double cy = std::cos(s.yaw / 2), sy = std::sin(s.yaw / 2);
  const double cp = std::cos(s.pitch / 2), sp = std::sin(s.pitch / 2);
  const double cr = std::cos(s.roll / 2), sr = std::sin(s.roll / 2);
  quat[0] = cr * cp * cy + sr * sp * sy;
  quat[1] = sr * cp * cy - cr * sp * sy;
  quat[2] = cr * sp * cy + sr * cp * sy;
  quat[3] = cr * cp * sy - sr * sp * cy;
}

}  // namespace cvsim
