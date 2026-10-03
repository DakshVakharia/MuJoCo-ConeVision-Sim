"""Pure message-building helpers for the ROS node.

None of these import ROS: every function receives the message classes/modules it needs, so they can be
unit-tested with fake message classes on a machine without ROS.
"""
import numpy as np

# ImageMarker constants (visualization_msgs/ImageMarker)
IMAGE_MARKER_POLYGON = 3   # visualization_msgs/ImageMarker: CIRCLE 0, LINE_STRIP 1, LINE_LIST 2, POLYGON 3, POINTS 4


def box_corners(det):
    """Corners in the order the sibling cone_base_detector expects (top-left first, clockwise)."""
    return [(det.x_min, det.y_min), (det.x_max, det.y_min),
            (det.x_max, det.y_max), (det.x_min, det.y_max)]


def build_marker_array(msgs, detections, outline_colors, stamp, frame_id, lifetime=None):
    """foxglove_msgs/ImageMarkerArray with one POLYGON ImageMarker per detection.

    msgs: object with attributes ImageMarker, ImageMarkerArray, Point, ColorRGBA.
    The array is empty when there are no detections.
    """
    arr = msgs.ImageMarkerArray()
    for det in detections:
        m = msgs.ImageMarker()
        m.header.stamp = stamp
        m.header.frame_id = frame_id
        m.ns = det.cls
        m.id = int(det.index)
        m.type = getattr(msgs.ImageMarker, "POLYGON", IMAGE_MARKER_POLYGON)
        m.action = getattr(msgs.ImageMarker, "ADD", 0)
        m.scale = 2.0
        r, g, b = outline_colors[det.cls]
        m.outline_color = msgs.ColorRGBA(r=float(r), g=float(g), b=float(b), a=1.0)
        m.filled = 0
        pts = []
        for (x, y) in box_corners(det):
            p = msgs.Point()
            p.x, p.y, p.z = float(x), float(y), 0.0
            pts.append(p)
        m.points = pts
        if lifetime is not None:
            m.lifetime = lifetime
        arr.markers.append(m)
    return arr


def build_image_msg(Image, rgb, stamp, frame_id):
    """sensor_msgs/Image rgb8 from an HxWx3 uint8 array."""
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = int(rgb.shape[0]), int(rgb.shape[1])
    msg.encoding = "rgb8"
    msg.is_bigendian = 0
    msg.step = int(rgb.shape[1] * 3)
    msg.data = np.ascontiguousarray(rgb).tobytes()
    return msg


def build_compressed_msg(CompressedImage, rgb, stamp, frame_id, quality, cv2):
    """sensor_msgs/CompressedImage (jpeg). OpenCV wants BGR."""
    ok, buf = cv2.imencode(".jpg", np.ascontiguousarray(rgb[..., ::-1]),
                           [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        raise RuntimeError("jpeg encoding failed")
    msg = CompressedImage()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.format = "jpeg"
    msg.data = buf.tobytes()
    return msg


def build_odom_msg(Odometry, st, stamp):
    """Ground-truth pose in frame "map", child "base_link"."""
    from .sim import euler_to_quat
    msg = Odometry()
    msg.header.stamp = stamp
    msg.header.frame_id = "map"
    msg.child_frame_id = "base_link"
    msg.pose.pose.position.x = float(st.x)
    msg.pose.pose.position.y = float(st.y)
    msg.pose.pose.position.z = 0.0
    w, x, y, z = euler_to_quat(st.yaw, st.pitch, st.roll)
    q = msg.pose.pose.orientation
    q.w, q.x, q.y, q.z = float(w), float(x), float(y), float(z)
    msg.twist.twist.linear.x = float(st.v)
    msg.twist.twist.angular.z = float(st.yaw_rate)
    return msg


def build_imu_msg(Imu, st, stamp, frame_id="base_link"):
    """Orientation from roll/pitch/yaw, angular velocity z = yaw_rate (no accel/gyro noise)."""
    from .sim import euler_to_quat
    msg = Imu()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    w, x, y, z = euler_to_quat(st.yaw, st.pitch, st.roll)
    q = msg.orientation
    q.w, q.x, q.y, q.z = float(w), float(x), float(y), float(z)
    msg.angular_velocity.z = float(st.yaw_rate)
    return msg
