#!/usr/bin/env python3
"""Cam2 Hand Tracker — ChArUco-based camera tracking for the digital twin.

Subscribes to color images from both cameras, detects a ChArUco board,
computes the 3D pose of cam2 relative to cam1, and publishes world→wrist_link
TF so the URDF hand follows cam2's physical motion.

When the board drops out (occluded), repeats the last known good transform.
On cold start, the static fallback world→wrist_link identity keeps the TF
chain connected.

Subscribes:
  /cam{1,2}/d435_{1,2}/color/image_raw
  /cam{1,2}/d435_{1,2}/color/camera_info
  /tf_static, /tf  (for realsense internal TF chain + static world TFs)

Publishes:
  /tf  (world→wrist_link)
"""

import math
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from geometry_msgs.msg import TransformStamped


# ── Math helpers (from charuco_tf_node) ────────────────────────────────────

def _rvec_tvec_to_matrix(rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(tvec, dtype=np.float64).reshape(3)
    return T


def _invert_transform_matrix(T):
    T_inv = np.eye(4, dtype=np.float64)
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


def _msg_to_matrix(tf: TransformStamped) -> np.ndarray:
    t = tf.transform.translation
    q = tf.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
        [    2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
        [    2*x*z - 2*y*w,     2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y],
    ], dtype=np.float64)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = [t.x, t.y, t.z]
    return T


def _matrix_to_tf(T: np.ndarray, parent: str, child: str, stamp) -> TransformStamped:
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.transform.translation.x = float(T[0, 3])
    msg.transform.translation.y = float(T[1, 3])
    msg.transform.translation.z = float(T[2, 3])
    R = T[:3, :3]
    trace = np.trace(R)
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    msg.transform.rotation.x = float(qx)
    msg.transform.rotation.y = float(qy)
    msg.transform.rotation.z = float(qz)
    msg.transform.rotation.w = float(qw)
    return msg


def _rpy_to_matrix(roll, pitch, yaw, tx, ty, tz):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    R = np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr],
    ], dtype=np.float64)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = [tx, ty, tz]
    return T


# ── Per-camera state ────────────────────────────────────────────────────────

class _CamState:
    def __init__(self):
        self.K: np.ndarray | None = None  # 3x3 intrinsic matrix
        self.D: np.ndarray | None = None  # distortion coefficients
        self.color_frame: str = ""
        self.link_frame: str = ""
        self.have_info = False


# ── Main node ───────────────────────────────────────────────────────────────

class Cam2HandTracker(Node):
    """Tracks cam2 position via ChArUco board and publishes world→wrist_link TF."""

    def __init__(self):
        super().__init__("cam2_hand_tracker")

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("cam1_color_frame", "cam1camera_color_optical_frame")
        self.declare_parameter("cam1_link", "cam1camera_link")
        self.declare_parameter("cam1_depth_frame", "cam1camera_depth_optical_frame")
        self.declare_parameter("cam2_color_frame", "cam2camera_color_optical_frame")
        self.declare_parameter("cam2_link", "cam2camera_link")
        self.declare_parameter("cam1_image_topic", "/cam1/d435_1/color/image_raw")
        self.declare_parameter("cam1_info_topic", "/cam1/d435_1/color/camera_info")
        self.declare_parameter("cam2_image_topic", "/cam2/d435_2/color/image_raw")
        self.declare_parameter("cam2_info_topic", "/cam2/d435_2/color/camera_info")
        self.declare_parameter("board_frame", "charuco_board")

        # ChArUco board config
        self.declare_parameter("squares_x", 5)
        self.declare_parameter("squares_y", 7)
        self.declare_parameter("square_length", 0.035)
        self.declare_parameter("marker_length", 0.026)
        self.declare_parameter("aruco_dictionary", "DICT_4X4_50")
        self.declare_parameter("min_charuco_corners", 4)

        # Wrist→cam2 offset
        self.declare_parameter("wrist_cam2_tx", -0.04)
        self.declare_parameter("wrist_cam2_ty", -0.01)
        self.declare_parameter("wrist_cam2_tz", 0.20)
        self.declare_parameter("wrist_cam2_roll", 1.57)
        self.declare_parameter("wrist_cam2_pitch", 0.0)
        self.declare_parameter("wrist_cam2_yaw", 1.57)
        self.declare_parameter("publish_rate", 15.0)

        # Read params
        self._world = self.get_parameter("world_frame").value
        self._cam1_color = self.get_parameter("cam1_color_frame").value
        self._cam1_link = self.get_parameter("cam1_link").value
        self._cam1_depth = self.get_parameter("cam1_depth_frame").value
        self._cam2_color = self.get_parameter("cam2_color_frame").value
        self._cam2_link = self.get_parameter("cam2_link").value
        self._board_frame = self.get_parameter("board_frame").value

        self._T_cam2_wrist = np.linalg.inv(_rpy_to_matrix(
            self.get_parameter("wrist_cam2_roll").value,
            self.get_parameter("wrist_cam2_pitch").value,
            self.get_parameter("wrist_cam2_yaw").value,
            self.get_parameter("wrist_cam2_tx").value,
            self.get_parameter("wrist_cam2_ty").value,
            self.get_parameter("wrist_cam2_tz").value,
        ))

        # ── ChArUco board ───────────────────────────────────────────────
        dict_id = getattr(cv2.aruco, self.get_parameter("aruco_dictionary").value)
        self._aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        self._board = cv2.aruco.CharucoBoard(
            (self.get_parameter("squares_x").value,
             self.get_parameter("squares_y").value),
            self.get_parameter("square_length").value,
            self.get_parameter("marker_length").value,
            self._aruco_dict,
        )
        self._detector = cv2.aruco.CharucoDetector(self._board)
        self._min_corners = self.get_parameter("min_charuco_corners").value

        # ── Subscriptions ───────────────────────────────────────────────
        self._bridge = CvBridge()
        self._cam1 = _CamState()
        self._cam2 = _CamState()
        self._cam1.color_frame = self._cam1_color
        self._cam1.link_frame = self._cam1_link
        self._cam2.color_frame = self._cam2_color
        self._cam2.link_frame = self._cam2_link

        self.create_subscription(CameraInfo,
            self.get_parameter("cam1_info_topic").value, self._cam1_info_cb, 10)
        self.create_subscription(CameraInfo,
            self.get_parameter("cam2_info_topic").value, self._cam2_info_cb, 10)
        self.create_subscription(Image,
            self.get_parameter("cam1_image_topic").value, self._cam1_image_cb, 10)
        self.create_subscription(Image,
            self.get_parameter("cam2_image_topic").value, self._cam2_image_cb, 10)

        # ── TF ──────────────────────────────────────────────────────────
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)

        # ── State ───────────────────────────────────────────────────────
        self._last_board_cam1: np.ndarray | None = None  # 4x4 board→cam1color
        self._last_board_cam2: np.ndarray | None = None  # 4x4 board→cam2color
        self._last_good_T: np.ndarray | None = None
        self._tracking_ok = False
        self._lost_count = 0

        # ── Timer ───────────────────────────────────────────────────────
        rate = self.get_parameter("publish_rate").value
        self.create_timer(1.0 / rate, self._track_and_publish)

        self.get_logger().info(
            f"Cam2HandTracker: ChArUco board tracking → {self._world}→wrist_link at {rate} Hz"
        )

    # ── Camera info callbacks ───────────────────────────────────────────

    def _cam1_info_cb(self, msg: CameraInfo):
        self._cam1.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self._cam1.D = np.array(msg.d, dtype=np.float64)
        self._cam1.have_info = True

    def _cam2_info_cb(self, msg: CameraInfo):
        self._cam2.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self._cam2.D = np.array(msg.d, dtype=np.float64)
        self._cam2.have_info = True

    # ── Image callbacks ─────────────────────────────────────────────────

    def _cam1_image_cb(self, msg: Image):
        T = self._detect_board(msg, self._cam1, "cam1")
        if T is not None:
            self._last_board_cam1 = T

    def _cam2_image_cb(self, msg: Image):
        T = self._detect_board(msg, self._cam2, "cam2")
        if T is not None:
            self._last_board_cam2 = T

    def _detect_board(self, msg: Image, state: _CamState, label: str) -> np.ndarray | None:
        """Detect ChArUco board in image, return 4x4 T_board_coloroptical."""
        if state.K is None or state.D is None:
            return None
        try:
            img = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            return None
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        corners, ids, marker_ids, _ = self._detector.detectBoard(gray)
        if ids is None or len(ids) < self._min_corners:
            return None

        objp, imgp = self._board.matchImagePoints(corners, ids)
        if objp is None or imgp is None or len(objp) < self._min_corners:
            return None

        ok, rvec, tvec = cv2.solvePnP(objp, imgp, state.K, state.D)
        if not ok:
            return None

        # OpenCV returns camera→board. Invert to get board→color_optical.
        T_cam_board = _rvec_tvec_to_matrix(rvec, tvec)
        T_board_color = _invert_transform_matrix(T_cam_board)
        return T_board_color

    # ── Tracking loop ───────────────────────────────────────────────────

    def _track_and_publish(self):
        now = self.get_clock().now().to_msg()

        T = self._compute_world_wrist()
        if T is not None:
            self._last_good_T = T
            self._lost_count = 0
            if not self._tracking_ok:
                self._tracking_ok = True
                self.get_logger().info("ChArUco tracking ACTIVE — hand follows cam2")
        elif self._last_good_T is not None:
            T = self._last_good_T
            self._lost_count += 1
            if self._tracking_ok and self._lost_count > 30:
                self._tracking_ok = False
                self.get_logger().warn("ChArUco tracking LOST — repeating last position",
                                       throttle_duration_sec=5.0)
        else:
            return  # cold start — static fallback handles it

        wrist_tf = _matrix_to_tf(T, self._world, "wrist_link", now)
        self._tf_broadcaster.sendTransform(wrist_tf)

    def _compute_world_wrist(self) -> np.ndarray | None:
        """Compose: world→cam1depth→cam1link→cam1color→board→cam2color→cam2link→wrist."""
        T_board_cam1 = self._last_board_cam1
        T_board_cam2 = self._last_board_cam2
        if T_board_cam1 is None or T_board_cam2 is None:
            return None

        # cam1color → board
        T_cam1color_board = _invert_transform_matrix(T_board_cam1)
        # cam1color → board → cam2color
        T_cam1color_cam2color = T_cam1color_board @ T_board_cam2

        # Get realsense static: cam1link → cam1color and cam2color → cam2link
        try:
            tf_cam1link_cam1color = self._tf_buffer.lookup_transform(
                self._cam1_link, self._cam1_color, Time(),
                timeout=Duration(seconds=0.3))
            tf_cam2color_cam2link = self._tf_buffer.lookup_transform(
                self._cam2_color, self._cam2_link, Time(),
                timeout=Duration(seconds=0.3))
        except Exception:
            return None
        T_cam1link_cam1color = _msg_to_matrix(tf_cam1link_cam1color)
        T_cam2color_cam2link = _msg_to_matrix(tf_cam2color_cam2link)

        # cam1link → cam1color → cam2color → cam2link
        T_cam1link_cam2link = T_cam1link_cam1color @ T_cam1color_cam2color @ T_cam2color_cam2link

        # world → cam1link via cam1depth
        try:
            tf_world_cam1depth = self._tf_buffer.lookup_transform(
                self._world, self._cam1_depth, Time(),
                timeout=Duration(seconds=0.3))
            tf_cam1link_cam1depth = self._tf_buffer.lookup_transform(
                self._cam1_link, self._cam1_depth, Time(),
                timeout=Duration(seconds=0.3))
        except Exception:
            return None
        T_world_cam1depth = _msg_to_matrix(tf_world_cam1depth)
        T_cam1link_cam1depth = _msg_to_matrix(tf_cam1link_cam1depth)
        T_cam1depth_cam1link = _invert_transform_matrix(T_cam1link_cam1depth)
        T_world_cam1link = T_world_cam1depth @ T_cam1depth_cam1link

        # world → cam1link → cam2link → wrist
        T_world_cam2link = T_world_cam1link @ T_cam1link_cam2link
        T_world_wrist = T_world_cam2link @ self._T_cam2_wrist
        return T_world_wrist


def main(args=None):
    rclpy.init(args=args)
    node = Cam2HandTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
