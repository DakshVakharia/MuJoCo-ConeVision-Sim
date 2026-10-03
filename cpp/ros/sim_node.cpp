// conevision_sim_node: C++ (roscpp) runtime for the MuJoCo Formula Student camera simulator.
//
// Port of scripts/sim_node.py. Offline tooling stays in Python (scripts/export_bundle.py writes the
// bundle); this node loads the bundle and renders 1280x720 @ 120 fps.
//
// Threads:
//   render thread  - creates + owns the Simulation (the GL context is thread-bound), paces frames
//                    according to ros.time_mode and fills pooled Frame buffers.
//   main thread    - consumes ready frames, builds/publishes all ROS messages (JPEG compression runs
//                    inside image_transport's publish call on this thread).
// Frames travel through a pool of kPoolSize Frame objects: free queue -> render -> ready queue ->
// publish -> free queue. The pool bounds latency (the renderer blocks/skips when the publisher lags).
//
// Params (private): ~bundle_dir, ~config_dir, ~time_mode (overrides ros.time_mode),
//                   ~lockstep_throttle (bool, lockstep only: also pace to realtime_factor),
//                   ~seed (NOT supported by the Simulation API; a warning is logged).
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <deque>
#include <exception>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <foxglove_msgs/ImageMarkerArray.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/ImageMarker.h>

#include "conevision_sim/config.h"
#include "conevision_sim/simulation.h"
#include "conevision_sim/trajectory.h"

namespace {

using SteadyClock = std::chrono::steady_clock;
constexpr int kPoolSize = 3;

// ---------------------------------------------------------------- frame pool / queues
class FramePool {
 public:
  FramePool() : frames_(kPoolSize) {
    for (auto& f : frames_) free_.push_back(&f);
  }
  // Renderer side: blocks for a free frame; nullptr when stopped.
  cvsim::Frame* acquire() {
    std::unique_lock<std::mutex> lk(m_);
    cv_free_.wait(lk, [&] { return stop_ || !free_.empty(); });
    if (stop_) return nullptr;
    auto* f = free_.front();
    free_.pop_front();
    return f;
  }
  void push_ready(cvsim::Frame* f) {
    {
      std::lock_guard<std::mutex> lk(m_);
      ready_.push_back(f);
    }
    cv_ready_.notify_one();
  }
  // Publisher side: waits up to `timeout`; nullptr on timeout or stop.
  cvsim::Frame* pop_ready(std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lk(m_);
    cv_ready_.wait_for(lk, timeout, [&] { return stop_ || !ready_.empty(); });
    if (ready_.empty()) return nullptr;
    auto* f = ready_.front();
    ready_.pop_front();
    return f;
  }
  void release(cvsim::Frame* f) {
    {
      std::lock_guard<std::mutex> lk(m_);
      free_.push_back(f);
    }
    cv_free_.notify_one();
  }
  void stop() {
    {
      std::lock_guard<std::mutex> lk(m_);
      stop_ = true;
    }
    cv_free_.notify_all();
    cv_ready_.notify_all();
  }

 private:
  std::vector<cvsim::Frame> frames_;
  std::deque<cvsim::Frame*> free_, ready_;
  std::mutex m_;
  std::condition_variable cv_free_, cv_ready_;
  bool stop_ = false;
};

// ---------------------------------------------------------------- shared state with render thread
struct Shared {
  std::mutex m;
  std::condition_variable cv;
  bool sim_ready = false;      // Simulation constructed (or failed)
  bool go = false;             // main thread finished advertising; start rendering
  std::string error;           // non-empty if the render thread died
  std::atomic<bool> stop{false};
  std::atomic<long long> skipped{0};
  std::atomic<long long> rendered{0};
};

// ---------------------------------------------------------------- message builders
// Same layout as rosmsgs.build_marker_array.
visualization_msgs::ImageMarker make_marker(const cvsim::Detection& det, const cvsim::BBoxConfig& bc,
                                            const ros::Time& stamp, const std::string& frame_id,
                                            const ros::Duration& lifetime) {
  visualization_msgs::ImageMarker m;
  m.header.stamp = stamp;
  m.header.frame_id = frame_id;
  m.ns = det.cls;
  m.id = det.index;
  m.type = visualization_msgs::ImageMarker::POLYGON;
  m.action = visualization_msgs::ImageMarker::ADD;
  m.scale = 2.0;
  auto it = bc.outline_colors.find(det.cls);
  if (it != bc.outline_colors.end()) {
    m.outline_color.r = it->second[0];
    m.outline_color.g = it->second[1];
    m.outline_color.b = it->second[2];
  }
  m.outline_color.a = 1.0f;
  m.filled = 0;
  const double xs[4] = {det.x_min, det.x_max, det.x_max, det.x_min};
  const double ys[4] = {det.y_min, det.y_min, det.y_max, det.y_max};
  m.points.resize(4);
  for (int i = 0; i < 4; ++i) {
    m.points[i].x = xs[i];
    m.points[i].y = ys[i];
    m.points[i].z = 0.0;
  }
  m.lifetime = lifetime;
  return m;
}

// ---------------------------------------------------------------- render thread
void render_thread_main(const std::string& bundle_dir, const std::string& config_dir, long long seed,
                        bool lockstep, bool throttle, FramePool* pool, Shared* sh) {
  std::unique_ptr<cvsim::Simulation> sim;
  try {
    sim.reset(new cvsim::Simulation(bundle_dir, config_dir, seed));  // GL context created on THIS thread
  } catch (const std::exception& e) {
    std::lock_guard<std::mutex> lk(sh->m);
    sh->error = std::string("Simulation init failed: ") + e.what();
    sh->sim_ready = true;
    sh->cv.notify_all();
    return;
  }
  {
    std::lock_guard<std::mutex> lk(sh->m);
    sh->sim_ready = true;
    sh->cv.notify_all();
  }
  {
    std::unique_lock<std::mutex> lk(sh->m);
    sh->cv.wait(lk, [&] { return sh->go || sh->stop.load(); });
  }
  if (sh->stop) return;

  const double fps = sim->fps();
  const double rtf = std::max(1e-3, sim->config().ros.realtime_factor);
  const double period_s = 1.0 / (fps * rtf);  // wall seconds per frame
  const bool paced = !lockstep || throttle;
  const auto wall0 = SteadyClock::now();
  auto due_of = [&](long long k) {
    return wall0 + std::chrono::duration_cast<SteadyClock::duration>(
                       std::chrono::duration<double>(k * period_s));
  };
  long long k = 0;
  try {
    while (!sh->stop) {
      if (paced) {
        auto due = due_of(k);
        const auto now = SteadyClock::now();
        if (!lockstep) {
          // Real-time: if more than one period late, jump ahead so sim time tracks wall time.
          const double late_s = std::chrono::duration<double>(now - due).count();
          if (late_s > period_s) {
            const double elapsed = std::chrono::duration<double>(now - wall0).count();
            const long long k_new = static_cast<long long>(std::ceil(elapsed / period_s));
            sh->skipped += (k_new - k);
            ROS_WARN_THROTTLE(5.0,
                              "renderer is %.0f ms behind real time: skipped %lld frames so far "
                              "(achieved %.1f fps of %.1f requested)",
                              late_s * 1e3, static_cast<long long>(sh->skipped.load()),
                              sh->rendered.load() / std::max(1e-3, elapsed), fps * rtf);
            k = k_new;
            due = due_of(k);
          }
        }
        if (due > now) std::this_thread::sleep_until(due);
      }
      cvsim::Frame* f = pool->acquire();
      if (!f) break;
      sim->frame(k, *f);
      pool->push_ready(f);
      ++sh->rendered;
      ++k;
    }
  } catch (const std::exception& e) {
    {
      std::lock_guard<std::mutex> lk(sh->m);
      sh->error = std::string("render thread: ") + e.what();
    }
    sh->stop = true;
    pool->stop();
  }
  // Simulation (and its GL context) is destroyed here, on the thread that created it.
}

}  // namespace

// ---------------------------------------------------------------- main
int main(int argc, char** argv) {
  ros::init(argc, argv, "conevision_sim");
  ros::NodeHandle nh, pnh("~");

  std::string bundle_dir, config_dir, time_mode_override;
  pnh.param<std::string>("bundle_dir", bundle_dir, "");
  pnh.param<std::string>("config_dir", config_dir, "");
  pnh.param<std::string>("time_mode", time_mode_override, "");
  bool lockstep_throttle = false;
  pnh.param<bool>("lockstep_throttle", lockstep_throttle, false);
  if (bundle_dir.empty()) {
    const std::string pkg = ros::package::getPath("conevision_sim");
    if (pkg.empty()) {
      ROS_FATAL("~bundle_dir not set and package conevision_sim not found");
      return 1;
    }
    bundle_dir = pkg + "/generated/bundle";
  }
  if (config_dir.empty()) config_dir = bundle_dir + "/config";

  // ~seed: optional override of perception.yaml `seed` (bbox noise). The launch file passes it as a
  // string ("" = not set), so accept both int and string params.
  long long seed = -1;
  int seed_int = 0;
  std::string seed_str;
  if (pnh.getParam("seed", seed_int)) {
    seed = seed_int;
  } else if (pnh.getParam("seed", seed_str) && !seed_str.empty()) {
    try {
      seed = std::stoll(seed_str);
    } catch (const std::exception&) {
      ROS_FATAL("~seed must be an integer, got '%s'", seed_str.c_str());
      return 1;
    }
  }

  // The Simulation re-reads the config itself; we load it here for topics/modes (pure file parsing).
  cvsim::SimConfig cfg;
  try {
    cfg = cvsim::load_config(config_dir);
  } catch (const std::exception& e) {
    ROS_FATAL("cannot load config from '%s': %s", config_dir.c_str(), e.what());
    return 1;
  }
  const std::string time_mode = time_mode_override.empty() ? cfg.ros.time_mode : time_mode_override;
  if (time_mode != "realtime" && time_mode != "lockstep") {
    ROS_FATAL("time_mode must be 'realtime' or 'lockstep', got '%s'", time_mode.c_str());
    return 1;
  }
  const bool lockstep = (time_mode == "lockstep");
  const bool publish_clock = cfg.ros.publish_clock || lockstep;
  if (lockstep) {
    bool use_sim_time = false;
    ros::param::get("/use_sim_time", use_sim_time);
    if (!use_sim_time)
      ROS_WARN("lockstep mode: set /use_sim_time:=true for all nodes (this node publishes /clock)");
  }

  FramePool pool;
  Shared sh;
  std::thread render(render_thread_main, bundle_dir, config_dir, seed, lockstep, lockstep_throttle, &pool,
                     &sh);
  auto shutdown_all = [&] {
    sh.stop = true;
    pool.stop();
    {
      std::lock_guard<std::mutex> lk(sh.m);
      sh.go = true;
    }
    sh.cv.notify_all();
    if (render.joinable()) render.join();
  };
  {
    std::unique_lock<std::mutex> lk(sh.m);
    sh.cv.wait(lk, [&] { return sh.sim_ready; });
    if (!sh.error.empty()) {
      const std::string err = sh.error;
      lk.unlock();
      ROS_FATAL("%s", err.c_str());
      shutdown_all();
      return 1;
    }
  }

  const double fps = cfg.camera.fps;
  const double rtf = cfg.ros.realtime_factor;

  // JPEG quality for image_transport's compressed plugin (must be set before advertise).
  const std::string img_topic_resolved = nh.resolveName(cfg.ros.image_topic);
  nh.setParam(img_topic_resolved + "/compressed/format", std::string("jpeg"));
  nh.setParam(img_topic_resolved + "/compressed/jpeg_quality", cfg.camera.jpeg_quality);

  // image_transport advertises raw + every installed plugin (/compressed, /theora ...). Subscribers
  // decide what is encoded: the JPEG is only produced if somebody subscribes to /compressed.
  // publish_raw / publish_compressed in the config are informational here; to hard-disable a
  // transport use the standard `<image_topic>/disable_pub_plugins` param.
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub_image = it.advertise(cfg.ros.image_topic, 2);
  ros::Publisher pub_box = nh.advertise<foxglove_msgs::ImageMarkerArray>(cfg.ros.bbox_topic, 10);
  ros::Publisher pub_odom, pub_imu, pub_clock;
  if (cfg.ros.publish_odom) pub_odom = nh.advertise<nav_msgs::Odometry>(cfg.ros.odom_topic, 2);
  if (cfg.ros.publish_imu) pub_imu = nh.advertise<sensor_msgs::Imu>(cfg.ros.imu_topic, 2);
  if (publish_clock) pub_clock = nh.advertise<rosgraph_msgs::Clock>("/clock", 1);

  // Stamps: with /clock published by us they are exactly k/fps; otherwise start_time + k/fps.
  const ros::Time stamp_base = publish_clock ? ros::Time(0, 0) : ros::Time::now();
  const ros::Duration box_life(std::max(2.0 / fps, 0.05));
  const double latency_s = cfg.bbox.latency_ms / 1000.0;

  ROS_INFO("conevision_sim: %dx%d @ %.1f fps, time_mode=%s (x%.2f)%s, jpeg_quality=%d, bundle=%s",
           cfg.camera.width, cfg.camera.height, fps, time_mode.c_str(), rtf,
           publish_clock ? ", publishing /clock" : "", cfg.camera.jpeg_quality, bundle_dir.c_str());

  {
    std::lock_guard<std::mutex> lk(sh.m);
    sh.go = true;
  }
  sh.cv.notify_all();

  // Delayed boxes (latency_ms). `due` is in wall seconds (realtime) or sim seconds (lockstep).
  struct Pending {
    double due;
    foxglove_msgs::ImageMarkerArray msg;
  };
  std::deque<Pending> pending;
  const auto wall_start = SteadyClock::now();
  double last_sim_t = 0.0;
  auto domain_now = [&]() {
    return lockstep ? last_sim_t
                    : std::chrono::duration<double>(SteadyClock::now() - wall_start).count();
  };
  auto flush_pending = [&]() {
    const double now = domain_now();
    while (!pending.empty() && pending.front().due <= now) {
      pub_box.publish(pending.front().msg);
      pending.pop_front();
    }
  };

  // stats window
  auto win_start = SteadyClock::now();
  long long win_frames = 0;
  double sum_step = 0, sum_render = 0, sum_bbox = 0, sum_total = 0;

  while (ros::ok() && !sh.stop) {
    ros::spinOnce();
    cvsim::Frame* f = pool.pop_ready(std::chrono::milliseconds(2));
    if (!f) {
      flush_pending();
      continue;
    }
    last_sim_t = f->t;
    const ros::Time stamp = stamp_base + ros::Duration(f->t);

    if (publish_clock) {
      rosgraph_msgs::Clock c;
      c.clock = ros::Time(0, 0) + ros::Duration(f->t);
      pub_clock.publish(c);
    }

    // Image: move the pixel buffer into the message (no copy), take it back if nobody kept it.
    sensor_msgs::ImagePtr img(new sensor_msgs::Image);
    img->header.stamp = stamp;
    img->header.frame_id = cfg.ros.frame_id;
    img->height = f->rgb.height;
    img->width = f->rgb.width;
    img->encoding = "rgb8";
    img->is_bigendian = 0;
    img->step = 3 * f->rgb.width;
    img->data.swap(f->rgb.data);
    pub_image.publish(img);  // raw + compressed (JPEG encoded inside, on this thread if subscribed)
    if (img.use_count() == 1) img->data.swap(f->rgb.data);  // else an intraprocess subscriber owns it

    foxglove_msgs::ImageMarkerArray arr;
    arr.markers.reserve(f->detections.size());
    for (const auto& det : f->detections)
      arr.markers.push_back(make_marker(det, cfg.bbox, stamp, cfg.ros.frame_id, box_life));
    if (latency_s <= 0.0) {
      pub_box.publish(arr);
    } else {
      pending.push_back({domain_now() + latency_s, std::move(arr)});
    }
    flush_pending();

    if (pub_odom || pub_imu) {
      double pos[3], quat[4];  // quat = (w, x, y, z), same as euler_to_quat(yaw, pitch, roll)
      cvsim::Trajectory::mocap_pose(f->car, pos, quat);
      if (pub_odom) {
        nav_msgs::Odometry o;
        o.header.stamp = stamp;
        o.header.frame_id = "map";
        o.child_frame_id = "base_link";
        o.pose.pose.position.x = f->car.x;
        o.pose.pose.position.y = f->car.y;
        o.pose.pose.position.z = 0.0;
        o.pose.pose.orientation.w = quat[0];
        o.pose.pose.orientation.x = quat[1];
        o.pose.pose.orientation.y = quat[2];
        o.pose.pose.orientation.z = quat[3];
        o.twist.twist.linear.x = f->car.v;
        o.twist.twist.angular.z = f->car.yaw_rate;
        pub_odom.publish(o);
      }
      if (pub_imu) {
        sensor_msgs::Imu m;
        m.header.stamp = stamp;
        m.header.frame_id = "base_link";
        m.orientation.w = quat[0];
        m.orientation.x = quat[1];
        m.orientation.y = quat[2];
        m.orientation.z = quat[3];
        m.angular_velocity.z = f->car.yaw_rate;
        pub_imu.publish(m);
      }
    }

    ++win_frames;
    sum_step += f->timing.step_ms;
    sum_render += f->timing.render_ms;
    sum_bbox += f->timing.bbox_ms;
    sum_total += f->timing.total_ms;
    pool.release(f);

    const auto now = SteadyClock::now();
    const double win_s = std::chrono::duration<double>(now - win_start).count();
    if (win_s >= 5.0) {
      const double n = static_cast<double>(std::max<long long>(1, win_frames));
      ROS_INFO("%.1f fps published | mean ms: step %.2f, render %.2f, bbox %.2f, total %.2f | "
               "t=%.2f s, skipped %lld",
               win_frames / win_s, sum_step / n, sum_render / n, sum_bbox / n, sum_total / n,
               last_sim_t, static_cast<long long>(sh.skipped.load()));
      win_start = now;
      win_frames = 0;
      sum_step = sum_render = sum_bbox = sum_total = 0;
    }
  }

  shutdown_all();
  const bool failed = !sh.error.empty();
  if (failed) ROS_FATAL("%s", sh.error.c_str());
  ros::shutdown();
  return failed ? 1 : 0;
}
