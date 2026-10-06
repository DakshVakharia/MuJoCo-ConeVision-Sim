// Test trajectory.h: CSV parsing with flexible columns, loop modes, defaults.
// Exit code 0 = all tests pass, 1 = failures.
#include <cassert>
#include <cmath>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include "conevision_sim/trajectory.h"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTol = 1e-6;

// Helper to write a CSV file and return its path.
std::string write_csv(const std::string& content) {
  static int counter = 0;
  std::ostringstream path_os;
  path_os << "cvsim_test_trajectory_" << counter++ << ".csv";
  std::string path = (std::filesystem::temp_directory_path() / path_os.str()).string();
  std::ofstream out(path);
  out << content;
  out.close();
  return path;
}

// Wrap yaw to [-pi, pi).
double wrap_yaw(double yaw) {
  double w = std::fmod(yaw + kPi, 2 * kPi);
  if (w < 0) w += 2 * kPi;
  return w - kPi;
}

bool nearly_equal(double a, double b, double tol = kTol) {
  return std::fabs(a - b) < tol;
}

// Test 1: Old 6-column format with body_motion sinusoids
void test_6column_legacy() {
  std::printf("Test 1: 6-column legacy format with body_motion sinusoids...\n");
  std::string csv = R"(# lap_time=1.0
# t,x,y,yaw,v,yaw_rate
0.0,0.0,0.0,0.0,0.0,0.0
1.0,10.0,5.0,0.5,2.0,0.1
)";
  std::string path = write_csv(csv);

  try {
    cvsim::BodyMotion motion;
    motion.pitch_amplitude_deg = 2.0;
    motion.roll_amplitude_deg = 3.0;
    motion.frequency_hz = 1.0;

    cvsim::Trajectory traj(path, motion);
    assert(nearly_equal(traj.lap_time(), 1.0));
    assert(traj.loop() == true);  // default

    // At t=0.5 (mid), state should be interpolated
    auto s = traj.state_at(0.5);
    assert(nearly_equal(s.x, 5.0));      // (0 + 10) / 2
    assert(nearly_equal(s.y, 2.5));      // (0 + 5) / 2
    assert(nearly_equal(s.z, 0.0));      // default
    assert(nearly_equal(s.v, 1.0));      // (0 + 2) / 2

    // Body motion sinusoids should apply (roll and pitch not in CSV)
    const double om = 2 * kPi * 1.0;
    const double expected_pitch = 2.0 * kPi / 180.0 * std::sin(om * 0.5);
    assert(nearly_equal(s.pitch, expected_pitch, 1e-5));

    // Defaults for acceleration/angular
    assert(nearly_equal(s.ax, 0.0));
    assert(nearly_equal(s.ay, 0.0));
    assert(nearly_equal(s.az, 9.81));
    assert(nearly_equal(s.wx, 0.0));
    assert(nearly_equal(s.wy, 0.0));
    assert(nearly_equal(s.wz, 0.05));  // yaw_rate interpolated

    std::printf("  PASS\n");
  } catch (const std::exception& e) {
    std::printf("  FAIL: %s\n", e.what());
    return;
  }
}

// Test 2: New full-column format with loop=0 (clamping)
void test_full_format_clamp() {
  std::printf("Test 2: Full-column format with loop=0 (clamping)...\n");
  std::string csv = R"(# lap_time=2.0
# loop=0
# source=test
# t,x,y,z,roll,pitch,yaw,v,yaw_rate,ax,ay,az,wx,wy,wz
0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,9.81,0.0,0.0,0.0
1.0,10.0,5.0,1.0,0.1,0.05,1.0,2.0,0.1,1.0,2.0,9.5,0.01,0.02,0.1
2.0,20.0,10.0,0.5,0.2,0.1,2.0,3.0,0.15,2.0,3.0,9.2,0.02,0.04,0.15
)";
  std::string path = write_csv(csv);

  try {
    cvsim::BodyMotion motion;
    cvsim::Trajectory traj(path, motion);

    assert(nearly_equal(traj.lap_time(), 2.0));
    assert(traj.loop() == false);  // loop=0

    // At t=1.0 (exact row 1)
    auto s = traj.state_at(1.0);
    assert(nearly_equal(s.x, 10.0));
    assert(nearly_equal(s.y, 5.0));
    assert(nearly_equal(s.z, 1.0));
    assert(nearly_equal(s.roll, 0.1));
    assert(nearly_equal(s.pitch, 0.05));
    assert(nearly_equal(s.ax, 1.0));
    assert(nearly_equal(s.az, 9.5));
    assert(nearly_equal(s.wz, 0.1));

    // Clamping: beyond end, should clamp to last row
    auto s_clamp = traj.state_at(10.0);
    assert(nearly_equal(s_clamp.x, 20.0));
    assert(nearly_equal(s_clamp.y, 10.0));
    assert(nearly_equal(s_clamp.z, 0.5));

    // Before start, should clamp to first row
    auto s_before = traj.state_at(-1.0);
    assert(nearly_equal(s_before.x, 0.0));
    assert(nearly_equal(s_before.y, 0.0));

    std::printf("  PASS\n");
  } catch (const std::exception& e) {
    std::printf("  FAIL: %s\n", e.what());
    return;
  }
}

// Test 3: Wrapping with loop=1
void test_wrapping_loop() {
  std::printf("Test 3: Wrapping with loop=1...\n");
  std::string csv = R"(# lap_time=1.0
# loop=1
# t,x,y,yaw,v,yaw_rate
0.0,0.0,0.0,0.0,1.0,0.0
1.0,10.0,0.0,6.283185307,1.0,0.0
)";
  std::string path = write_csv(csv);

  try {
    cvsim::BodyMotion motion;
    cvsim::Trajectory traj(path, motion);

    assert(traj.loop() == true);

    // At t=0.5 (mid-cycle)
    auto s = traj.state_at(0.5);
    assert(nearly_equal(s.x, 5.0));

    // At t=1.5 (wrapped to 0.5)
    auto s_wrap = traj.state_at(1.5);
    assert(nearly_equal(s_wrap.x, 5.0));

    // Yaw should wrap to [-pi, pi) but the CSV has yaw=6.283... (=2pi) at last row
    // which is almost equivalent to 0.0
    auto s_yaw = traj.state_at(1.0);
    // Unwrapped yaw at t=1 is 6.283..., wrapped to [-pi, pi) should be close to 0
    assert(nearly_equal(s_yaw.yaw, 0.0, 0.01));

    std::printf("  PASS\n");
  } catch (const std::exception& e) {
    std::printf("  FAIL: %s\n", e.what());
    return;
  }
}

// Test 4: Missing optional columns (should use defaults)
void test_missing_columns_defaults() {
  std::printf("Test 4: Missing optional columns with defaults...\n");
  std::string csv = R"(# lap_time=1.0
# t,x,y,yaw,v
0.0,0.0,0.0,0.0,1.0
0.5,5.0,2.5,0.5,1.5
1.0,10.0,5.0,1.0,2.0
)";
  std::string path = write_csv(csv);

  try {
    cvsim::BodyMotion motion;
    cvsim::Trajectory traj(path, motion);

    auto s = traj.state_at(0.5);
    assert(nearly_equal(s.x, 5.0));
    assert(nearly_equal(s.y, 2.5));
    assert(nearly_equal(s.yaw, 0.5));

    // Missing columns should have defaults
    assert(nearly_equal(s.z, 0.0));        // default z
    assert(nearly_equal(s.yaw_rate, 0.0)); // default yaw_rate
    assert(nearly_equal(s.ax, 0.0));       // default ax
    assert(nearly_equal(s.ay, 0.0));       // default ay
    assert(nearly_equal(s.az, 9.81));      // default az
    assert(nearly_equal(s.wx, 0.0));       // default wx
    assert(nearly_equal(s.wy, 0.0));       // default wy
    assert(nearly_equal(s.wz, 0.0));       // default wz (yaw_rate missing)

    std::printf("  PASS\n");
  } catch (const std::exception& e) {
    std::printf("  FAIL: %s\n", e.what());
    return;
  }
}

// Test 5: CRLF line endings
void test_crlf_endings() {
  std::printf("Test 5: CRLF line endings...\n");
  std::string csv = "# lap_time=1.0\r\n# t,x,y,yaw,v,yaw_rate\r\n0.0,0.0,0.0,0.0,0.0,0.0\r\n1.0,10.0,5.0,0.5,2.0,0.1\r\n";
  std::string path = write_csv(csv);

  try {
    cvsim::BodyMotion motion;
    cvsim::Trajectory traj(path, motion);

    assert(nearly_equal(traj.lap_time(), 1.0));
    auto s = traj.state_at(0.5);
    assert(nearly_equal(s.x, 5.0));

    std::printf("  PASS\n");
  } catch (const std::exception& e) {
    std::printf("  FAIL: %s\n", e.what());
    return;
  }
}

// Test 6: Yaw wrapping
void test_yaw_wrapping() {
  std::printf("Test 6: Yaw wrapping to [-pi, pi)...\n");
  std::string csv = R"(# lap_time=1.0
# t,x,y,yaw,v,yaw_rate
0.0,0.0,0.0,-2.0,1.0,0.0
1.0,10.0,0.0,4.0,1.0,0.0
)";
  std::string path = write_csv(csv);

  try {
    cvsim::BodyMotion motion;
    cvsim::Trajectory traj(path, motion);

    // At t=0, yaw=-2.0 (already in [-pi, pi))
    auto s0 = traj.state_at(0.0);
    assert(nearly_equal(s0.yaw, -2.0));

    // At t=1.0, yaw=4.0 (outside [-pi, pi), should wrap)
    auto s1 = traj.state_at(1.0);
    double expected_yaw = wrap_yaw(4.0);  // should be ~-2.283
    assert(nearly_equal(s1.yaw, expected_yaw, 1e-4));

    std::printf("  PASS\n");
  } catch (const std::exception& e) {
    std::printf("  FAIL: %s\n", e.what());
    return;
  }
}

// Test 7: Interpolation at exact midpoint
void test_midpoint_interpolation() {
  std::printf("Test 7: Interpolation at exact midpoint...\n");
  std::string csv = R"(# lap_time=2.0
# loop=0
# t,x,y,yaw,z,ax,ay,az,wz
0.0,0.0,0.0,0.0,0.0,0.0,0.0,9.81,0.0
2.0,20.0,10.0,0.0,2.0,4.0,6.0,8.0,1.0
)";
  std::string path = write_csv(csv);

  try {
    cvsim::BodyMotion motion;
    cvsim::Trajectory traj(path, motion);

    auto s = traj.state_at(1.0);  // exact midpoint
    assert(nearly_equal(s.x, 10.0));   // (0 + 20) / 2
    assert(nearly_equal(s.y, 5.0));    // (0 + 10) / 2
    assert(nearly_equal(s.z, 1.0));    // (0 + 2) / 2
    assert(nearly_equal(s.ax, 2.0));   // (0 + 4) / 2
    assert(nearly_equal(s.ay, 3.0));   // (0 + 6) / 2
    assert(nearly_equal(s.az, 8.905)); // (9.81 + 8) / 2
    assert(nearly_equal(s.wz, 0.5));   // (0 + 1) / 2

    std::printf("  PASS\n");
  } catch (const std::exception& e) {
    std::printf("  FAIL: %s\n", e.what());
    return;
  }
}

}  // namespace

int main() {
  std::printf("Running trajectory tests...\n\n");

  test_6column_legacy();
  test_full_format_clamp();
  test_wrapping_loop();
  test_missing_columns_defaults();
  test_crlf_endings();
  test_yaw_wrapping();
  test_midpoint_interpolation();

  std::printf("\nAll tests completed.\n");
  return 0;
}
