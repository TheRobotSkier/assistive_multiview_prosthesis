#!/usr/bin/env python3

# ============================================================
# ChArUco TF node for two RealSense cameras in ROS 2
#
# Purpose:
# 1. Subscribe to color images and camera_info from two cameras
# 2. Detect a printed ChArUco board in each color image
# 3. Estimate the pose of the board relative to each camera
# 4. Convert that pose into ROS TF transforms
# 5. Publish:
#    - charuco_board -> d435_1_link
#    - charuco_board -> d435_2_link
#
# Why publish to *_link instead of *_color_optical_frame?
# -------------------------------------------------------
# RealSense already publishes its own internal TF tree:
#   d435_X_link -> d435_X_color_frame -> d435_X_color_optical_frame
#   d435_X_link -> d435_X_depth_frame -> d435_X_depth_optical_frame
#
# If we directly republish d435_X_color_optical_frame under charuco_board,
# we would break the existing RealSense TF tree.
#
# Instead, we:
# 1. Estimate board -> color_optical from the ChArUco board
# 2. Look up the existing RealSense transform color_optical <- link
# 3. Compose them to get board -> link
# 4. Publish board -> link
#
# This keeps the RealSense TF chain intact and allows RViz to transform
# depth / point cloud data into the charuco_board frame.
# ============================================================

import os
import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from cv_bridge import CvBridge


# ============================================================
# Helper functions for transform math
# ============================================================

def rvec_tvec_to_matrix(rvec, tvec):
    """
    Convert OpenCV pose output (rvec, tvec) into a 4x4 homogeneous matrix.

    OpenCV returns:
      - rvec: Rodrigues rotation vector
      - tvec: translation vector

    We convert this into:
      T = [ R  t ]
          [ 0  1 ]

    where:
      R is 3x3 rotation matrix
      t is 3x1 translation vector
    """
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(tvec, dtype=np.float64).reshape(3)
    return T


def invert_transform_matrix(T):
    """
    Invert a rigid-body 4x4 transform.

    If:
      T = [ R  t ]
          [ 0  1 ]

    then:
      T^-1 = [ R^T   -R^T t ]
             [  0       1   ]

    This is used because OpenCV gives us "board wrt camera",
    but for ROS TF we want "board -> camera" or later "board -> link".
    """
    T_inv = np.eye(4, dtype=np.float64)
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


def quaternion_to_rotation_matrix(x, y, z, w):
    """
    Convert quaternion (x, y, z, w) into a 3x3 rotation matrix.

    This is used when converting an existing ROS TF transform
    into matrix form so we can compose transforms numerically.
    """
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    R = np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz),       2.0 * (xz + wy)],
        [2.0 * (xy + wz),       1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy),       2.0 * (yz + wx),       1.0 - 2.0 * (xx + yy)],
    ], dtype=np.float64)
    return R


def transform_msg_to_matrix(tf_msg):
    """
    Convert a ROS TransformStamped into a 4x4 homogeneous matrix.

    This lets us combine TF transforms with our OpenCV-estimated transforms
    using normal matrix multiplication.
    """
    t = tf_msg.transform.translation
    q = tf_msg.transform.rotation

    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = quaternion_to_rotation_matrix(q.x, q.y, q.z, q.w)
    T[:3, 3] = np.array([t.x, t.y, t.z], dtype=np.float64)
    return T


def rotation_matrix_to_quaternion(R):
    """
    Convert a 3x3 rotation matrix into quaternion (x, y, z, w).

    This is needed because ROS TF messages use quaternions,
    while most of our transform math is done in matrix form.
    """
    q = np.empty(4, dtype=np.float64)
    trace = np.trace(R)

    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        q[3] = 0.25 * s
        q[0] = (R[2, 1] - R[1, 2]) / s
        q[1] = (R[0, 2] - R[2, 0]) / s
        q[2] = (R[1, 0] - R[0, 1]) / s
    else:
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            q[3] = (R[2, 1] - R[1, 2]) / s
            q[0] = 0.25 * s
            q[1] = (R[0, 1] + R[1, 0]) / s
            q[2] = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            q[3] = (R[0, 2] - R[2, 0]) / s
            q[0] = (R[0, 1] + R[1, 0]) / s
            q[1] = 0.25 * s
            q[2] = (R[1, 2] + R[2, 1]) / s
        else:
            s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            q[3] = (R[1, 0] - R[0, 1]) / s
            q[0] = (R[0, 2] + R[2, 0]) / s
            q[1] = (R[1, 2] + R[2, 1]) / s
            q[2] = 0.25 * s

    return q[0], q[1], q[2], q[3]

# ============================================================
# Per-camera state
# ============================================================

class CameraState:
    """
    Holds all per-camera state needed by the node.

    This avoids duplicating the same variables for cam1 and cam2.
    """
    def __init__(self):
        # Intrinsic calibration matrix from CameraInfo or fallback
        self.camera_matrix = None

        # Distortion coefficients from CameraInfo or fallback
        self.dist_coeffs = None

        # Optical frame name, e.g. d435_1_color_optical_frame
        self.frame_id = None

        # Camera link frame, e.g. d435_1_link
        self.link_frame = None

        # True if ROS CameraInfo has been received
        self.have_camera_info = False

        # Used to avoid printing the same warning repeatedly
        self.fallback_warned = False
        self.tf_warned = False
        self.tf_warn_time = None   # wall-clock time of last TF warning (for periodic re-warn)

        # Detection diagnostic counters (reset each time a transform is published)
        self.diag_no_markers = 0
        self.diag_few_corners = 0
        self.diag_pose_failed = 0

        # Cache the existing RealSense transform:
        # color_optical <- link
        # so we don't look it up every frame
        self.cached_T_color_link = None

# ============================================================
# Main ROS 2 node
# ============================================================

class CharucoTfNode(Node):
    def __init__(self):
        # Allow node name override via env var to prevent duplicate name conflicts
        node_name = os.environ.get('CHARUCO_NODE_NAME', 'charuco_tf_node')
        super().__init__(node_name)

        # cv_bridge converts ROS Image messages to OpenCV images
        self.bridge = CvBridge()

        # Broadcaster publishes our output transforms into TF
        self.tf_broadcaster = TransformBroadcaster(self)

        # Buffer + listener let us read existing TF transforms
        # that RealSense is already publishing
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ------------------------------------------------------------
        # ChArUco board configuration
        # These values must match the printed board exactly
        # ------------------------------------------------------------
        self.declare_parameter('board_frame', 'charuco_board')
        self.declare_parameter('squares_x', 5)
        self.declare_parameter('squares_y', 7)
        self.declare_parameter('square_length', 0.035) # in meters, e.g. 0.035 for 35 mm
        self.declare_parameter('marker_length', 0.026) # in meters, e.g. 0.026 for 26 mm
        self.declare_parameter('aruco_dictionary', 'DICT_4X4_50')
        self.declare_parameter('min_charuco_corners', 4)
        self.declare_parameter('publish_debug_optical_frames', True)

        # ------------------------------------------------------------
        # Startup behavior
        #
        # startup_grace_sec:
        #   During early startup, we suppress warnings because topics
        #   and TF may not be ready yet.
        #
        # fallback_after_sec:
        #   If camera_info still has not arrived after this time,
        #   we are allowed to use hard-coded fallback intrinsics.
        #
        # tf_lookup_timeout_sec:
        #   How long to wait during each tf2 lookup.
        # ------------------------------------------------------------
        self.declare_parameter('startup_grace_sec', 5.0)
        self.declare_parameter('fallback_after_sec', 5.0)
        self.declare_parameter('tf_lookup_timeout_sec', 1.0)

        # ------------------------------------------------------------
        # Fallback intrinsics
        #
        # These are used only if camera_info does not arrive in time.
        # In normal operation, live CameraInfo is preferred.
        # ------------------------------------------------------------
        self.declare_parameter('use_fallback_intrinsics', True)

        self.declare_parameter('cam1_k', [
            615.3892211914062, 0.0, 324.1833190917969,
            0.0, 615.7371215820312, 242.41490173339844,
            0.0, 0.0, 1.0
        ])
        self.declare_parameter('cam1_d', [0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('cam1_frame_id', 'd435_1_color_optical_frame')
        self.declare_parameter('cam1_link_frame', 'd435_1_link')

        self.declare_parameter('cam2_k', [
            615.4778442382812, 0.0, 316.2964782714844,
            0.0, 615.7003784179688, 244.2735137939453,
            0.0, 0.0, 1.0
        ])
        self.declare_parameter('cam2_d', [0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('cam2_frame_id', 'd435_2_color_optical_frame')
        self.declare_parameter('cam2_link_frame', 'd435_2_link')

        # Read parameter values into normal variables
        self.board_frame = self.get_parameter('board_frame').value
        squares_x = self.get_parameter('squares_x').value
        squares_y = self.get_parameter('squares_y').value
        square_length = self.get_parameter('square_length').value
        marker_length = self.get_parameter('marker_length').value
        dict_name = self.get_parameter('aruco_dictionary').value
        self.min_charuco_corners = self.get_parameter('min_charuco_corners').value
        self.publish_debug_optical_frames = self.get_parameter('publish_debug_optical_frames').value

        self.startup_grace_sec = float(self.get_parameter('startup_grace_sec').value)
        self.fallback_after_sec = float(self.get_parameter('fallback_after_sec').value)
        self.tf_lookup_timeout_sec = float(self.get_parameter('tf_lookup_timeout_sec').value)

        self.use_fallback_intrinsics = self.get_parameter('use_fallback_intrinsics').value

        # ------------------------------------------------------------
        # Create the OpenCV ArUco dictionary and ChArUco board object
        #
        # The board object holds the known geometry of the printed board.
        # This is what makes metric pose estimation possible.
        # ------------------------------------------------------------
        aruco_dict_id = getattr(cv2.aruco, dict_name)
        self.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_id)
        self.board = cv2.aruco.CharucoBoard_create(
            squares_x, squares_y, square_length, marker_length, self.aruco_dict
        )
        self.detector_params = cv2.aruco.DetectorParameters_create()

        # One state object per camera to hold intrinsics, frame IDs, cached TFs, etc.
        self.cam1 = CameraState()
        self.cam2 = CameraState()

        # Record node start time for timeout/grace-period logic
        self.start_time = self.get_clock().now()

        # ------------------------------------------------------------
        # ROS subscriptions
        # We subscribe to:
        #   - CameraInfo for intrinsics
        #   - color image for ChArUco detection
        # ------------------------------------------------------------
        self.create_subscription(CameraInfo, '/cam1/d435_1/camera/color/camera_info', self.cam1_info_cb, 10)
        self.create_subscription(CameraInfo, '/cam2/d435_2/camera/color/camera_info', self.cam2_info_cb, 10)
        self.create_subscription(Image, '/cam1/d435_1/camera/color/image_raw', self.cam1_image_cb, 10)
        self.create_subscription(Image, '/cam2/d435_2/camera/color/image_raw', self.cam2_image_cb, 10)

        self.get_logger().info('charuco_tf_node started')

    def seconds_since_start(self):
        """
        Return elapsed time since node startup in seconds.

        Used for:
        - warning grace period
        - deciding when fallback intrinsics may be used
        """
        return (self.get_clock().now() - self.start_time).nanoseconds / 1e9

    def cam1_info_cb(self, msg: CameraInfo):
        """CameraInfo callback for camera 1."""
        self._camera_info_to_state(msg, self.cam1, 'cam1')

    def cam2_info_cb(self, msg: CameraInfo):
        """CameraInfo callback for camera 2."""
        self._camera_info_to_state(msg, self.cam2, 'cam2')

    def _camera_info_to_state(self, msg: CameraInfo, state: CameraState, cam_name: str):
        """
        Copy CameraInfo into the camera state.

        We extract:
        - camera matrix K
        - distortion coefficients D
        - optical frame ID

        We also infer the link frame from the optical frame name.
        Example:
          d435_1_color_optical_frame -> d435_1_link
        """
        state.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        state.dist_coeffs = np.array(msg.d, dtype=np.float64)
        state.frame_id = msg.header.frame_id

        if state.frame_id.endswith('_color_optical_frame'):
            state.link_frame = state.frame_id.replace('_color_optical_frame', '_link')
        else:
            state.link_frame = self.get_parameter(f'{cam_name}_link_frame').value

        state.have_camera_info = True

    def apply_fallback_intrinsics(self, state: CameraState, cam_name: str):
        """
        Use hard-coded intrinsics if CameraInfo has not arrived in time.

        This is a fallback path, mainly for robustness and debugging.
        In normal operation, live CameraInfo should be used instead.
        """
        if not self.use_fallback_intrinsics:
            return False

        if self.seconds_since_start() < self.fallback_after_sec:
            return False

        if cam_name == 'cam1':
            k = self.get_parameter('cam1_k').value
            d = self.get_parameter('cam1_d').value
            frame_id = self.get_parameter('cam1_frame_id').value
            link_frame = self.get_parameter('cam1_link_frame').value
        elif cam_name == 'cam2':
            k = self.get_parameter('cam2_k').value
            d = self.get_parameter('cam2_d').value
            frame_id = self.get_parameter('cam2_frame_id').value
            link_frame = self.get_parameter('cam2_link_frame').value
        else:
            return False

        state.camera_matrix = np.array(k, dtype=np.float64).reshape(3, 3)
        state.dist_coeffs = np.array(d, dtype=np.float64)
        state.frame_id = frame_id
        state.link_frame = link_frame

        # Warn only once per camera
        if not state.fallback_warned:
            self.get_logger().warn(
                f'{cam_name}: using fallback intrinsics because camera_info did not arrive within '
                f'{self.fallback_after_sec:.1f} s'
            )
            state.fallback_warned = True

        return True

    def get_or_cache_color_from_link_tf(self, state: CameraState, cam_name: str):
        """
        Look up and cache the existing RealSense TF:
            color_optical <- link

        Why cache?
        ----------
        This transform is static for a given camera, so there is no need
        to query tf2 every frame once we have it.

        Returns:
            4x4 matrix T_color_link
        """
        # If already cached, reuse it
        if state.cached_T_color_link is not None:
            return state.cached_T_color_link

        if state.frame_id is None or state.link_frame is None:
            return None

        try:
            tf_color_link = self.tf_buffer.lookup_transform(
                state.frame_id,         # target frame
                state.link_frame,       # source frame
                Time(),
                timeout=Duration(seconds=self.tf_lookup_timeout_sec)
            )
            state.cached_T_color_link = transform_msg_to_matrix(tf_color_link)
            # Reset warning state on success
            state.tf_warned = False
            state.tf_warn_time = None
            return state.cached_T_color_link
        except Exception as e:
            # Suppress startup warning spam during grace period
            now = self.get_clock().now().nanoseconds / 1e9
            if self.seconds_since_start() >= self.startup_grace_sec:
                # Re-warn periodically (every 30 s) rather than just once
                if not state.tf_warned:
                    self.get_logger().warn(
                        f'{cam_name}: could not look up existing TF {state.frame_id} <- {state.link_frame}: {e}'
                    )
                    state.tf_warned = True
                    state.tf_warn_time = now
                elif state.tf_warn_time is not None and (now - state.tf_warn_time) > 30.0:
                    self.get_logger().warn(
                        f'{cam_name}: still cannot look up TF {state.frame_id} <- {state.link_frame} '
                        f'(last warned {now - state.tf_warn_time:.0f}s ago): {e}'
                    )
                    state.tf_warn_time = now
            return None

    def cam1_image_cb(self, msg: Image):
        """Image callback for camera 1."""
        self.process_image(msg, self.cam1, 'cam1')

    def cam2_image_cb(self, msg: Image):
        """Image callback for camera 2."""
        self.process_image(msg, self.cam2, 'cam2')

    def process_image(self, msg: Image, state: CameraState, cam_name: str):
        """
        Main ChArUco processing pipeline for one camera.

        Steps:
        1. Make sure intrinsics are available
        2. Make sure existing RealSense TF is available
        3. Convert ROS Image to OpenCV image
        4. Detect ArUco markers
        5. Interpolate ChArUco corners
        6. Estimate ChArUco board pose
        7. Convert board pose to board -> color_optical
        8. Compose with existing TF to get board -> link
        9. Publish TF
        """
        
        # Ensure we have intrinsics, using fallback if needed
        if state.camera_matrix is None or state.dist_coeffs is None or state.frame_id is None or state.link_frame is None:
            ok = self.apply_fallback_intrinsics(state, cam_name)
            if not ok:
                return

        # Ensure we have the existing RealSense transform color_optical <- link
        T_color_link = self.get_or_cache_color_from_link_tf(state, cam_name)
        if T_color_link is None:
            return

        # Convert ROS Image -> OpenCV BGR image
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'{cam_name}: cv_bridge failed: {e}')
            return

        # ChArUco / ArUco detection typically works on grayscale images
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # ------------------------------------------------------------
        # Step 1: detect ArUco markers
        # marker_corners: image-space corners of each marker
        # marker_ids: IDs of detected markers
        # ------------------------------------------------------------
        marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.detector_params
        )

        # If no markers are found, no board pose can be estimated
        if marker_ids is None or len(marker_ids) == 0:
            state.diag_no_markers += 1
            if state.diag_no_markers == 1 or state.diag_no_markers % 30 == 0:
                self.get_logger().warn(
                    f'{cam_name}: no ArUco markers detected in frame '
                    f'(seen {state.diag_no_markers} frames without markers)'
                )
            return

        # ------------------------------------------------------------
        # Step 2: interpolate ChArUco chessboard corners
        #
        # These corners are typically more accurate for pose estimation
        # than marker corners alone.
        # ------------------------------------------------------------
        num, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
            marker_corners,
            marker_ids,
            gray,
            self.board,
            cameraMatrix=state.camera_matrix,
            distCoeffs=state.dist_coeffs
        )

        # Require a minimum number of detected ChArUco corners
        found = 0 if (charuco_ids is None or charuco_corners is None) else int(num)
        if found < self.min_charuco_corners:
            state.diag_few_corners += 1
            if state.diag_few_corners == 1 or state.diag_few_corners % 30 == 0:
                self.get_logger().warn(
                    f'{cam_name}: insufficient ChArUco corners ({found} < {self.min_charuco_corners}) '
                    f'(seen {state.diag_few_corners} frames with too few corners)'
                )
            return

        # ------------------------------------------------------------
        # Step 3: estimate board pose
        #
        # OpenCV returns the pose of the board with respect to the camera.
        # In other words: "board wrt camera".
        # ------------------------------------------------------------
        ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
            charuco_corners,
            charuco_ids,
            self.board,
            state.camera_matrix,
            state.dist_coeffs,
            None,
            None
        )

        if not ok:
            state.diag_pose_failed += 1
            if state.diag_pose_failed == 1 or state.diag_pose_failed % 30 == 0:
                self.get_logger().warn(
                    f'{cam_name}: estimatePoseCharucoBoard failed '
                    f'(seen {state.diag_pose_failed} pose failures)'
                )
            return

        # Convert OpenCV pose output into 4x4 matrix form
        T_camera_board = rvec_tvec_to_matrix(rvec, tvec)

        # Invert it to get board -> color_optical
        T_board_color = invert_transform_matrix(T_camera_board)

        # ------------------------------------------------------------
        # Optional debug TF:
        # publish board -> color_optical_from_charuco
        #
        # This is useful for debugging the raw board pose estimate
        # without touching the original RealSense TF tree.
        # ------------------------------------------------------------
        if self.publish_debug_optical_frames:
            debug_child = f'{state.frame_id}_from_charuco'
            self.publish_tf(
                stamp=msg.header.stamp,
                parent_frame=self.board_frame,
                child_frame=debug_child,
                T=T_board_color
            )

        # ------------------------------------------------------------
        # Compose transforms:
        #
        # We already have:
        #   T_board_color  = board -> color_optical
        #   T_color_link   = color_optical -> link
        #
        # Therefore:
        #   T_board_link = T_board_color @ T_color_link
        #
        # This gives us:
        #   board -> link
        # ------------------------------------------------------------
        T_board_link = T_board_color @ T_color_link

        # Validate the composed transform before publishing
        if not self._validate_transform(T_board_link, cam_name):
            return

        # Reset diagnostic counters on successful publish
        state.diag_no_markers = 0
        state.diag_few_corners = 0
        state.diag_pose_failed = 0

        # Publish the final useful TF:
        #   charuco_board -> d435_X_link
        self.publish_tf(
            stamp=msg.header.stamp,
            parent_frame=self.board_frame,
            child_frame=state.link_frame,
            T=T_board_link
        )


    def _validate_transform(self, T: np.ndarray, cam_name: str) -> bool:
        """
        Validate a 4x4 homogeneous transform before publishing.

        Checks:
        - No NaN or Inf values
        - Rotation part is non-zero (not a degenerate estimate)

        Returns True if the transform is valid.
        """
        if not np.all(np.isfinite(T)):
            self.get_logger().error(
                f'{cam_name}: computed transform contains NaN or Inf — skipping publish'
            )
            return False

        # Check that the rotation part is not near-zero (would indicate degenerate pose)
        R_norm = np.linalg.norm(T[:3, :3])
        if R_norm < 1e-6:
            self.get_logger().error(
                f'{cam_name}: computed transform has near-zero rotation (norm={R_norm:.2e}) — skipping publish'
            )
            return False

        return True

    def publish_tf(self, stamp, parent_frame: str, child_frame: str, T: np.ndarray):
        """
        Publish a 4x4 transform matrix as a ROS TF transform.

        Inputs:
            stamp        : ROS timestamp from the source image
            parent_frame : TF parent frame
            child_frame  : TF child frame
            T            : 4x4 homogeneous transform
        """
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = parent_frame
        tf_msg.child_frame_id = child_frame

        tf_msg.transform.translation.x = float(T[0, 3])
        tf_msg.transform.translation.y = float(T[1, 3])
        tf_msg.transform.translation.z = float(T[2, 3])

        qx, qy, qz, qw = rotation_matrix_to_quaternion(T[:3, :3])
        tf_msg.transform.rotation.x = float(qx)
        tf_msg.transform.rotation.y = float(qy)
        tf_msg.transform.rotation.z = float(qz)
        tf_msg.transform.rotation.w = float(qw)

        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    """
    Standard ROS 2 Python node entry point.
    """
    rclpy.init(args=args)
    node = CharucoTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()