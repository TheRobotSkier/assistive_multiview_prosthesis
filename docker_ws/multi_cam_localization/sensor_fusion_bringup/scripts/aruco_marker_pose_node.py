#!/usr/bin/env python3

import ast
import math
import re
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from tf2_ros import TransformBroadcaster


def T_inv(T):
    R = T[:3, :3]
    t = T[:3, 3]
    Tout = np.eye(4)
    Tout[:3, :3] = R.T
    Tout[:3, 3] = -R.T @ t
    return Tout


def rot_to_quat_xyzw(R):
    tr = np.trace(R)
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s

    q = np.array([qx, qy, qz, qw], dtype=float)
    q /= np.linalg.norm(q)
    return q


def odom_to_T(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    x, y, z, w = q.x, q.y, q.z, q.w

    R = np.array([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w,     2*x*z + 2*y*w],
        [2*x*y + 2*z*w,     1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w,     2*y*z + 2*x*w,     1 - 2*x*x - 2*y*y],
    ], dtype=float)

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def fill_pose(pose_msg, T):
    qx, qy, qz, qw = rot_to_quat_xyzw(T[:3, :3])
    pose_msg.position.x = float(T[0, 3])
    pose_msg.position.y = float(T[1, 3])
    pose_msg.position.z = float(T[2, 3])
    pose_msg.orientation.x = float(qx)
    pose_msg.orientation.y = float(qy)
    pose_msg.orientation.z = float(qz)
    pose_msg.orientation.w = float(qw)


def parse_list_from_line(text, key):
    m = re.search(rf"{key}:\s*(\[[^\n]+\])", text)
    if not m:
        raise RuntimeError(f"Could not find '{key}' in calibration file")
    return ast.literal_eval(m.group(1))


def parse_T_block(text, key):
    lines = text.splitlines()
    for i, line in enumerate(lines):
        if line.strip().startswith(f"{key}:"):
            rows = []
            for j in range(i + 1, min(i + 8, len(lines))):
                s = lines[j].strip()
                if s.startswith("- [") or s.startswith("["):
                    s = s[1:].strip() if s.startswith("-") else s
                    rows.append(ast.literal_eval(s))
                    if len(rows) == 4:
                        return np.array(rows, dtype=float)
    raise RuntimeError(f"Could not find '{key}' matrix in calibration file")


def load_kalibr_imucam(path):
    text = Path(path).read_text()
    fx, fy, cx, cy = parse_list_from_line(text, "intrinsics")
    dist = parse_list_from_line(text, "distortion_coeffs")
    T_cam_imu = parse_T_block(text, "T_cam_imu")

    K = np.array([
        [fx, 0.0, cx],
        [0.0, fy, cy],
        [0.0, 0.0, 1.0],
    ], dtype=float)

    D = np.array(dist, dtype=float).reshape(-1, 1)
    return K, D, T_cam_imu


def marker_object_points(size_m):
    s = float(size_m)
    h = s / 2.0

    # Corner order matches OpenCV ArUco: top-left, top-right, bottom-right, bottom-left.
    # Marker frame is centered on marker, with x right, y down in the printed marker plane.
    return np.array([
        [-h, -h, 0.0],
        [ h, -h, 0.0],
        [ h,  h, 0.0],
        [-h,  h, 0.0],
    ], dtype=np.float32)


class ArucoMarkerPoseNode(Node):
    def __init__(self):
        super().__init__("aruco_marker_pose_node")

        self.declare_parameter("config_file", "")
        config_file = self.get_parameter("config_file").value
        if not config_file:
            raise RuntimeError("Parameter config_file is required")

        self.config = yaml.safe_load(Path(config_file).read_text())

        aruco_cfg = self.config["aruco"]
        self.dictionary_name = aruco_cfg.get("dictionary", "DICT_6X6_1000")
        self.default_marker_size_m = float(aruco_cfg.get("default_marker_size_m", 0.154))

        self.map_frame = self.config["frames"].get("map_frame", "marker_map")
        self.camera_frame = self.config["frames"].get("camera_frame", "camera_color_optical_frame")
        self.imu_frame = self.config["frames"].get("imu_frame", "imu")

        calib_path = self.config["calibration"]["kalibr_imucam_chain"]
        self.K, self.D, self.T_cam_imu = load_kalibr_imucam(calib_path)

        self.image_topic = self.config["topics"].get("image", "/head/d435i_head/color/image_raw")
        self.odom_topic = self.config["topics"].get("openvins_odom", "/ov_msckf/odomimu")

        reanchor_cfg = self.config.get("reanchoring", {})
        self.reanchor_enabled = bool(reanchor_cfg.get("enabled", True))
        self.update_continuously = bool(reanchor_cfg.get("update_continuously", True))

        self.markers = {}
        for marker_id_str, marker_cfg in self.config["markers"].items():
            marker_id = int(marker_id_str)
            T_map_marker = np.array(marker_cfg["T_map_marker"], dtype=float)
            size_m = float(marker_cfg.get("size_m", self.default_marker_size_m))
            self.markers[marker_id] = {
                "size_m": size_m,
                "T_map_marker": T_map_marker,
            }

        if not hasattr(cv2, "aruco"):
            raise RuntimeError("cv2.aruco is not available. Install OpenCV with aruco support.")

        dict_id = getattr(cv2.aruco, self.dictionary_name)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)

        if hasattr(cv2.aruco, "DetectorParameters"):
            self.aruco_params = cv2.aruco.DetectorParameters()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters_create()

        # Use legacy detectMarkers call for now.
        # This avoids segfaults seen with some OpenCV/Python/ROS image combinations.
        self.aruco_detector = None

        self.tf_broadcaster = TransformBroadcaster(self)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_cb, sensor_qos)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 20)

        self.camera_pose_pub = self.create_publisher(PoseStamped, "/head/marker_pose/camera_pose", 10)
        self.imu_pose_pub = self.create_publisher(PoseStamped, "/head/marker_pose/imu_pose", 10)
        self.corrected_odom_pub = self.create_publisher(Odometry, "/head/marker_pose/ov_corrected_odom", 20)
        self.active_marker_pub = self.create_publisher(Int32, "/head/marker_pose/active_marker_id", 10)

        self.last_odom_msg = None
        self.last_T_global_imu = None
        self.T_map_global = None

        self.get_logger().info(f"Using marker config: {config_file}")
        self.get_logger().info(f"Image topic: {self.image_topic}")
        self.get_logger().info(f"OpenVINS odom topic: {self.odom_topic}")
        self.get_logger().info(f"Dictionary: {self.dictionary_name}")
        self.get_logger().info(f"Known marker IDs: {sorted(self.markers.keys())}")
        self.get_logger().info(f"Default marker size: {self.default_marker_size_m:.4f} m")

    def detect_markers(self, gray):
        # Conservative OpenCV call path.
        # Do not pass DetectorParameters here.
        # On the Jetson OpenCV 4.6.0 Python binding this can segfault on live ROS images.
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray,
            self.aruco_dict,
        )
        return corners, ids

    def image_msg_to_gray(self, msg):
        # Avoid cv_bridge for raw sensor_msgs/Image conversion.
        # Expected RealSense topic encoding is usually rgb8.
        enc = msg.encoding.lower()
        h = msg.height
        w = msg.width
        step = msg.step

        data = np.frombuffer(msg.data, dtype=np.uint8)

        if enc in ("rgb8", "bgr8"):
            channels = 3
            rows = data.reshape((h, step))
            img = rows[:, :w * channels].reshape((h, w, channels))
            img = np.ascontiguousarray(img)

            if enc == "rgb8":
                return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            else:
                return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        if enc in ("mono8", "8uc1"):
            rows = data.reshape((h, step))
            gray = rows[:, :w]
            return np.ascontiguousarray(gray)

        self.get_logger().warning(f"Unsupported image encoding for marker detector: {msg.encoding}")
        return None

    def image_cb(self, msg):
        gray = self.image_msg_to_gray(msg)
        if gray is None:
            return

        try:
            corners, ids = self.detect_markers(gray)
        except Exception as exc:
            self.get_logger().warning(f"ArUco detection failed: {exc}")
            return

        if ids is None or len(ids) == 0:
            return

        best = None

        for idx, marker_id_arr in enumerate(ids):
            marker_id = int(marker_id_arr[0])
            if marker_id not in self.markers:
                continue

            image_points = corners[idx].reshape(4, 2).astype(np.float32)
            area = abs(cv2.contourArea(image_points))

            size_m = self.markers[marker_id]["size_m"]
            obj_points = marker_object_points(size_m)

            ok, rvec, tvec = cv2.solvePnP(
                obj_points,
                image_points,
                self.K,
                self.D,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )

            if not ok:
                continue

            R_cam_marker, _ = cv2.Rodrigues(rvec)
            T_cam_marker = np.eye(4)
            T_cam_marker[:3, :3] = R_cam_marker
            T_cam_marker[:3, 3] = tvec.reshape(3)

            T_marker_cam = T_inv(T_cam_marker)
            T_map_marker = self.markers[marker_id]["T_map_marker"]
            T_map_cam = T_map_marker @ T_marker_cam
            T_map_imu = T_map_cam @ self.T_cam_imu

            if best is None or area > best["area"]:
                best = {
                    "marker_id": marker_id,
                    "area": area,
                    "T_map_cam": T_map_cam,
                    "T_map_imu": T_map_imu,
                }

        if best is None:
            return

        stamp = msg.header.stamp
        marker_id = best["marker_id"]

        self.publish_pose(self.camera_pose_pub, stamp, self.map_frame, best["T_map_cam"])
        self.publish_pose(self.imu_pose_pub, stamp, self.map_frame, best["T_map_imu"])

        id_msg = Int32()
        id_msg.data = marker_id
        self.active_marker_pub.publish(id_msg)

        self.publish_tf(stamp, self.map_frame, self.camera_frame + "_from_marker", best["T_map_cam"])
        self.publish_tf(stamp, self.map_frame, self.imu_frame + "_from_marker", best["T_map_imu"])

        if self.reanchor_enabled and self.last_T_global_imu is not None:
            if self.T_map_global is None or self.update_continuously:
                self.T_map_global = best["T_map_imu"] @ T_inv(self.last_T_global_imu)
                self.get_logger().debug(f"Updated marker_map -> OpenVINS global correction using marker {marker_id}")

    def odom_cb(self, msg):
        self.last_odom_msg = msg
        self.last_T_global_imu = odom_to_T(msg)

        if self.T_map_global is None:
            return

        T_map_imu_corrected = self.T_map_global @ self.last_T_global_imu

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.map_frame
        out.child_frame_id = self.imu_frame + "_openvins_corrected"

        fill_pose(out.pose.pose, T_map_imu_corrected)

        # Copy covariance and twist for now. Later we can rotate covariances into marker_map if needed.
        out.pose.covariance = msg.pose.covariance
        out.twist = msg.twist

        self.corrected_odom_pub.publish(out)
        self.publish_tf(msg.header.stamp, self.map_frame, out.child_frame_id, T_map_imu_corrected)

    def publish_pose(self, pub, stamp, frame_id, T):
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        fill_pose(msg.pose, T)
        pub.publish(msg)

    def publish_tf(self, stamp, parent, child, T):
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = parent
        tf.child_frame_id = child
        tf.transform.translation.x = float(T[0, 3])
        tf.transform.translation.y = float(T[1, 3])
        tf.transform.translation.z = float(T[2, 3])

        qx, qy, qz, qw = rot_to_quat_xyzw(T[:3, :3])
        tf.transform.rotation.x = float(qx)
        tf.transform.rotation.y = float(qy)
        tf.transform.rotation.z = float(qz)
        tf.transform.rotation.w = float(qw)

        self.tf_broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    node = ArucoMarkerPoseNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()