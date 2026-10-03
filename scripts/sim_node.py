#!/usr/bin/env python3
"""ROS Noetic node: publishes the simulated camera feed + fake-YOLO boxes.

Publishes (see config/perception.yaml `ros:`):
  image_topic       sensor_msgs/Image              rgb8            (publish_raw)
  compressed_topic  sensor_msgs/CompressedImage    jpeg            (publish_compressed)
  bbox_topic        foxglove_msgs/ImageMarkerArray one POLYGON ImageMarker per cone
  odom_topic        nav_msgs/Odometry              ground truth    (publish_odom)
  imu_topic         sensor_msgs/Imu                                (publish_imu)
  /clock            rosgraph_msgs/Clock            sim time        (publish_clock)

Private params: ~config_dir (default: package config dir), ~seed (bbox noise seed override).
All ROS imports are inside main(), so this file compiles/imports without ROS.
"""
import collections
import sys
import time
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))


def main():
    import cv2
    import rospy
    from foxglove_msgs.msg import ImageMarkerArray
    from geometry_msgs.msg import Point
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import CompressedImage, Image, Imu
    from std_msgs.msg import ColorRGBA
    from visualization_msgs.msg import ImageMarker

    from fs_mono_cam_sim.config import CONFIG_DIR, load_all
    from fs_mono_cam_sim.render import rosmsgs
    from fs_mono_cam_sim.render.sim import Simulation

    rospy.init_node("fs_mono_cam_sim")
    config_dir = rospy.get_param("~config_dir", str(CONFIG_DIR)) or str(CONFIG_DIR)
    seed = rospy.get_param("~seed", None)
    if seed in ("", None):
        seed = None
    cfg = load_all(config_dir)
    ros = cfg["perception"]["ros"]
    bbox_cfg = cfg["perception"]["bbox"]
    colors = bbox_cfg["outline_colors"]
    frame_id = ros.get("frame_id", "front_cam_optical")
    latency = float(bbox_cfg.get("latency_ms", 0)) / 1000.0
    rtf = float(ros.get("realtime_factor", 1.0))
    publish_clock = bool(ros.get("publish_clock", False))
    jpeg_q = int(cfg["camera"].get("jpeg_quality", 90))

    msgs = SimpleNamespace(ImageMarker=ImageMarker, ImageMarkerArray=ImageMarkerArray,
                           Point=Point, ColorRGBA=ColorRGBA)
    sim = Simulation(cfg, seed=int(seed) if seed is not None else None)
    fps = sim.fps

    q = 2
    pub_raw = rospy.Publisher(ros["image_topic"], Image, queue_size=q) if ros.get("publish_raw", True) else None
    pub_cmp = (rospy.Publisher(ros["compressed_topic"], CompressedImage, queue_size=q)
               if ros.get("publish_compressed", True) else None)
    pub_box = rospy.Publisher(ros["bbox_topic"], ImageMarkerArray, queue_size=10)
    pub_odom = (rospy.Publisher(ros.get("odom_topic", "/sim/ground_truth/odom"), Odometry, queue_size=q)
                if ros.get("publish_odom") else None)
    pub_imu = (rospy.Publisher(ros.get("imu_topic", "/sim/imu"), Imu, queue_size=q)
               if ros.get("publish_imu") else None)
    pub_clock = rospy.Publisher("/clock", Clock, queue_size=1) if publish_clock else None

    t_start = rospy.Time.now()          # with use_sim_time this is the /clock time (0 until we publish)
    pending = collections.deque()       # (wall due time, ImageMarkerArray) for latency_ms
    box_life = rospy.Duration.from_sec(max(2.0 / fps, 0.05))
    rospy.loginfo("fs_mono_cam_sim: %dx%d @ %.0f fps (x%.2f), seed=%s", sim.renderer.width,
                  sim.renderer.height, fps, rtf, seed)

    def stamp_of(t):
        return rospy.Time.from_sec(t) if publish_clock else t_start + rospy.Duration.from_sec(t)

    def flush_boxes(now_wall):
        while pending and pending[0][0] <= now_wall:
            _, arr = pending.popleft()
            pub_box.publish(arr)

    k = 0
    wall0 = time.monotonic()
    while not rospy.is_shutdown():
        t = k / fps
        f = sim.frame(t)
        stamp = stamp_of(t)
        if pub_clock is not None:
            pub_clock.publish(Clock(clock=stamp))
        if pub_raw is not None:
            pub_raw.publish(rosmsgs.build_image_msg(Image, f.rgb, stamp, frame_id))
        if pub_cmp is not None:
            pub_cmp.publish(rosmsgs.build_compressed_msg(CompressedImage, f.rgb, stamp, frame_id, jpeg_q, cv2))
        arr = rosmsgs.build_marker_array(msgs, f.detections, colors, stamp, frame_id, lifetime=box_life)
        due = time.monotonic() + latency
        pending.append((due, arr))
        if pub_odom is not None:
            pub_odom.publish(rosmsgs.build_odom_msg(Odometry, f.car_state, stamp))
        if pub_imu is not None:
            pub_imu.publish(rosmsgs.build_imu_msg(Imu, f.car_state, stamp))
        flush_boxes(time.monotonic())
        k += 1
        # pace on wall time (time.monotonic, so it also works when /use_sim_time is set): frame k is due at wall0 + k / (fps * rtf)
        target = wall0 + k / (fps * rtf)
        while True:
            now = time.monotonic()
            flush_boxes(now)
            if now >= target or rospy.is_shutdown():
                break
            time.sleep(min(0.001, target - now))
        if now - target > 1.0:                     # fell far behind: do not try to catch up in a burst
            wall0 += now - target
    sim.close()


if __name__ == "__main__":
    main()
