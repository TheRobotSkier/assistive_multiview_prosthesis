#!/usr/bin/env python3
"""Point cloud fusion node — transforms dual-camera clouds to world frame, merges,
filters, downsamples, and publishes a unified cloud.

Pipeline:
  1. Subscribe to both camera point clouds (approximate time sync)
  2. Transform both to world frame via TF2
  3. Concatenate into a single cloud
  4. Distance filter — remove points >max_distance from arm_frame origin
  5. Hand/arm bbox removal — AABB crop per pruning box (from camera_mounts.yaml)
  6. Voxel downsampling — numpy-based grid filter
  7. Publish on /fused_pointcloud

Topics
------
Subscribe:
  cam1_topic  (param)  sensor_msgs/PointCloud2   — camera 1 depth/color/points
  cam2_topic  (param)  sensor_msgs/PointCloud2   — camera 2 depth/color/points

Publish:
  /fused_pointcloud                    sensor_msgs/PointCloud2
  /pointcloud_fusion/hand_removal_bbox visualization_msgs/Marker
"""

import threading
from pathlib import Path

import numpy as np
import rclpy
import tf2_ros
import tf2_sensor_msgs  # noqa: F401 — registers do_transform_cloud
import yaml
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker

try:
    import message_filters
    from message_filters import ApproximateTimeSynchronizer

    _HAS_MSG_FILTERS = True
except ImportError:
    _HAS_MSG_FILTERS = False


# ---------------------------------------------------------------------------
# PointCloud2 helpers
# ---------------------------------------------------------------------------


def _parse_cloud(msg: PointCloud2):
    """Extract xyz (N,3) float32 and rgb_packed (N,) uint32 from a PointCloud2.

    Returns (xyz, rgb_packed) where rgb_packed is 0x00RRGGBB per point.
    If the cloud has no rgb field, rgb_packed is all zeros.
    """
    n = msg.width * msg.height
    if n == 0:
        return np.zeros((0, 3), dtype=np.float32), np.zeros(0, dtype=np.uint32)

    step = msg.point_step
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, step)
    fields = {f.name: f for f in msg.fields}

    def _col_f32(name: str) -> np.ndarray:
        off = fields[name].offset
        return np.frombuffer(raw[:, off : off + 4].copy().tobytes(), dtype=np.float32)

    xyz = np.column_stack([_col_f32("x"), _col_f32("y"), _col_f32("z")])

    rgb_packed = np.zeros(n, dtype=np.uint32)
    if "rgb" in fields:
        rgb_packed = _col_f32("rgb").view(np.uint32).copy()

    return xyz, rgb_packed


def _build_cloud(xyz: np.ndarray, rgb_packed: np.ndarray, header) -> PointCloud2:
    """Build an XYZRGB PointCloud2 from numpy arrays.

    xyz:         (N, 3) float32
    rgb_packed:  (N,) uint32  — 0x00RRGGBB encoding
    """
    n = xyz.shape[0]
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = n
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16  # 4 fields * 4 bytes
    msg.row_step = 16 * n
    msg.is_dense = True

    # Interleave xyz and rgb into a single byte buffer
    buf = np.zeros((n, 4), dtype=np.float32)
    buf[:, :3] = xyz
    buf[:, 3] = rgb_packed.view(np.float32)
    msg.data = np.ascontiguousarray(buf).tobytes()
    return msg


# ---------------------------------------------------------------------------
# Filtering helpers
# ---------------------------------------------------------------------------


def _distance_filter(
    xyz: np.ndarray, center: np.ndarray, max_dist: float
) -> np.ndarray:
    """Return boolean mask: True for points within max_dist of center."""
    diff = xyz - center
    dists_sq = np.sum(diff * diff, axis=1)
    return dists_sq <= (max_dist * max_dist)


def _bbox_filter(
    xyz_arm: np.ndarray, bbox_min: np.ndarray, bbox_max: np.ndarray
) -> np.ndarray:
    """Return boolean mask: True for points OUTSIDE the AABB (to keep)."""
    inside = np.all(
        (xyz_arm >= bbox_min) & (xyz_arm <= bbox_max),
        axis=1,
    )
    return ~inside  # keep points outside the bbox


def _voxel_downsample(xyz: np.ndarray, rgb_packed: np.ndarray, voxel_size: float):
    """Voxel grid downsampling. Returns (xyz_down, rgb_down) with one centroid per voxel.

    xyz:         (N, 3) float32
    rgb_packed:  (N,) uint32  — 0x00RRGGBB encoding
    voxel_size:  side length of each voxel cube in metres
    """
    if voxel_size <= 0.0 or len(xyz) == 0:
        return xyz, rgb_packed

    inv = 1.0 / voxel_size
    voxel_idx = np.floor(xyz * inv).astype(np.int64)
    _, unique_idx, inverse = np.unique(
        voxel_idx,
        axis=0,
        return_index=True,
        return_inverse=True,
    )

    n_voxels = len(unique_idx)
    summed_xyz = np.zeros((n_voxels, 3), dtype=np.float64)
    counts = np.zeros(n_voxels, dtype=np.int32)

    np.add.at(summed_xyz, inverse, xyz.astype(np.float64))
    np.add.at(counts, inverse, 1)

    xyz_out = (summed_xyz / counts[:, None]).astype(np.float32)

    # Per-channel RGB averaging to avoid carry propagation between channels
    # when averaging packed 0x00RRGGBB integers.
    r_ch = ((rgb_packed >> 16) & 0xFF).astype(np.uint64)
    g_ch = ((rgb_packed >> 8) & 0xFF).astype(np.uint64)
    b_ch = (rgb_packed & 0xFF).astype(np.uint64)

    summed_r = np.zeros(n_voxels, dtype=np.uint64)
    summed_g = np.zeros(n_voxels, dtype=np.uint64)
    summed_b = np.zeros(n_voxels, dtype=np.uint64)

    np.add.at(summed_r, inverse, r_ch)
    np.add.at(summed_g, inverse, g_ch)
    np.add.at(summed_b, inverse, b_ch)

    counts_u64 = counts.astype(np.uint64)
    avg_r = (summed_r / counts_u64).astype(np.uint32)
    avg_g = (summed_g / counts_u64).astype(np.uint32)
    avg_b = (summed_b / counts_u64).astype(np.uint32)

    rgb_out = (avg_r << 16) | (avg_g << 8) | avg_b
    return xyz_out, rgb_out


def _voxel_sample_xyz(
    xyz: np.ndarray, voxel_size: float, max_points: int
) -> np.ndarray:
    """Downsample XYZ points for registration and cap to max_points."""
    if len(xyz) == 0:
        return xyz.astype(np.float32)
    finite = np.isfinite(xyz).all(axis=1)
    sample = xyz[finite].astype(np.float32)
    if len(sample) == 0:
        return sample
    if voxel_size > 0.0:
        dummy_rgb = np.zeros(len(sample), dtype=np.uint32)
        sample, _ = _voxel_downsample(sample, dummy_rgb, voxel_size)
    if max_points > 0 and len(sample) > max_points:
        rng = np.random.default_rng()
        sample = sample[rng.choice(len(sample), max_points, replace=False)]
    return sample.astype(np.float32)


def _voxel_sample_xyzrgb(
    xyz: np.ndarray,
    rgb_packed: np.ndarray,
    voxel_size: float,
    max_points: int,
) -> tuple[np.ndarray, np.ndarray]:
    """Downsample XYZ + RGB for registration and cap to max_points.

    Returns (xyz_sample, rgb_sample) with matching lengths.
    """
    if len(xyz) == 0:
        return (
            np.zeros((0, 3), dtype=np.float32),
            np.zeros(0, dtype=np.uint32),
        )
    finite = np.isfinite(xyz).all(axis=1)
    s_xyz = xyz[finite].astype(np.float32)
    s_rgb = rgb_packed[finite].copy()
    if len(s_xyz) == 0:
        return s_xyz, s_rgb
    if voxel_size > 0.0:
        s_xyz, s_rgb = _voxel_downsample(s_xyz, s_rgb, voxel_size)
    if max_points > 0 and len(s_xyz) > max_points:
        rng = np.random.default_rng()
        keep = rng.choice(len(s_xyz), max_points, replace=False)
        s_xyz = s_xyz[keep]
        s_rgb = s_rgb[keep]
    return s_xyz.astype(np.float32), s_rgb


def _rgb_to_lab(rgb_packed: np.ndarray) -> np.ndarray:
    """Convert packed 0x00RRGGBB to approximate Lab (N,3) float32.

    Uses a fast sRGB→linear→XYZ→Lab approximation suitable for
    perceptual color distance without needing a full colour-science lib.
    """
    r = ((rgb_packed >> 16) & 0xFF).astype(np.float32) / 255.0
    g = ((rgb_packed >> 8) & 0xFF).astype(np.float32) / 255.0
    b = (rgb_packed & 0xFF).astype(np.float32) / 255.0
    # sRGB -> linear
    r = np.where(r > 0.04045, ((r + 0.055) / 1.055) ** 2.4, r / 12.92)
    g = np.where(g > 0.04045, ((g + 0.055) / 1.055) ** 2.4, g / 12.92)
    b = np.where(b > 0.04045, ((b + 0.055) / 1.055) ** 2.4, b / 12.92)
    # linear -> XYZ (D65)
    x = 0.4124564 * r + 0.3575761 * g + 0.1804375 * b
    y = 0.2126729 * r + 0.7151522 * g + 0.0721750 * b
    z = 0.0193339 * r + 0.1191920 * g + 0.9503041 * b
    # XYZ -> Lab
    xn, yn, zn = 0.95047, 1.0, 1.08883
    x /= xn
    y /= yn
    z /= zn
    eps = 0.008856
    kappa = 903.3
    fx = np.where(x > eps, np.cbrt(x), (kappa * x + 16.0) / 116.0)
    fy = np.where(y > eps, np.cbrt(y), (kappa * y + 16.0) / 116.0)
    fz = np.where(z > eps, np.cbrt(z), (kappa * z + 16.0) / 116.0)
    L = 116.0 * fy - 16.0
    a = 500.0 * (fx - fy)
    b_out = 200.0 * (fy - fz)
    return np.column_stack([L, a, b_out]).astype(np.float32)


def _estimate_rigid_transform(src: np.ndarray, tgt: np.ndarray):
    """Return R,t that maps src onto tgt using Kabsch/SVD."""
    if len(src) < 3 or len(tgt) < 3 or len(src) != len(tgt):
        return None
    src_f = src.astype(np.float64)
    tgt_f = tgt.astype(np.float64)
    src_mean = np.mean(src_f, axis=0)
    tgt_mean = np.mean(tgt_f, axis=0)
    src_c = src_f - src_mean
    tgt_c = tgt_f - tgt_mean
    H = src_c.T @ tgt_c
    try:
        U, _, Vt = np.linalg.svd(H)
    except np.linalg.LinAlgError:
        return None
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[2, :] *= -1
        R = Vt.T @ U.T
    t = tgt_mean - R @ src_mean
    return R, t


def _compose_transform(
    R_new: np.ndarray, t_new: np.ndarray, R_old: np.ndarray, t_old: np.ndarray
):
    """Compose transforms: new(old(x))."""
    return R_new @ R_old, (R_new @ t_old) + t_new


def _rotation_angle_rad(R: np.ndarray) -> float:
    trace = float(np.clip((np.trace(R) - 1.0) * 0.5, -1.0, 1.0))
    return float(np.arccos(trace))


def _extract_rotation_translation(t) -> tuple[np.ndarray, np.ndarray]:
    """Extract rotation matrix (3,3) and translation vector (3,) from a TF2 transform."""
    qx = t.transform.rotation.x
    qy = t.transform.rotation.y
    qz = t.transform.rotation.z
    qw = t.transform.rotation.w

    r00 = 1 - 2 * (qy * qy + qz * qz)
    r01 = 2 * (qx * qy - qz * qw)
    r02 = 2 * (qx * qz + qy * qw)
    r10 = 2 * (qx * qy + qz * qw)
    r11 = 1 - 2 * (qx * qx + qz * qz)
    r12 = 2 * (qy * qz - qx * qw)
    r20 = 2 * (qx * qz - qy * qw)
    r21 = 2 * (qy * qz + qx * qw)
    r22 = 1 - 2 * (qx * qx + qy * qy)

    R = np.array([[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]], dtype=np.float64)
    t_vec = np.array(
        [
            t.transform.translation.x,
            t.transform.translation.y,
            t.transform.translation.z,
        ],
        dtype=np.float64,
    )
    return R, t_vec


def _transform_points_to_frame(
    xyz: np.ndarray, tf_buffer, target_frame: str, source_frame: str, stamp
) -> np.ndarray | None:
    """Transform an (N,3) xyz array from source_frame to target_frame using TF2.

    Returns transformed (N,3) float32 or None if TF lookup fails.
    """
    try:
        t = tf_buffer.lookup_transform(target_frame, source_frame, stamp)
    except Exception:
        return None

    R, t_vec = _extract_rotation_translation(t)
    transformed = (xyz.astype(np.float64) @ R.T) + t_vec
    return transformed.astype(np.float32)


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------


class PointCloudFusionNode(Node):
    """Fuses dual-camera point clouds in world frame with filtering."""

    def __init__(self):
        super().__init__("pointcloud_fusion")

        # ── Declare parameters ────────────────────────────────────────────
        self.declare_parameter("target_frame", "marker_map")
        self.declare_parameter("cam1_topic", "/head/d435i_head/depth/color/points")
        self.declare_parameter("cam2_topic", "/arm/d435i_arm/depth/color/points")
        self.declare_parameter("arm_frame", "arm_d435i_arm_depth_frame")
        self.declare_parameter("max_distance", 2.0)
        self.declare_parameter("voxel_size", 0.005)
        self.declare_parameter("bbox_min", [-0.30, -0.10, -0.10])
        self.declare_parameter("bbox_max", [0.22, 0.10, 0.12])
        self.declare_parameter("enable_downsampling", True)
        self.declare_parameter("enable_distance_filter", True)
        self.declare_parameter("enable_hand_removal", True)
        self.declare_parameter("output_topic", "/fused_pointcloud")
        self.declare_parameter("sync_tolerance_s", 0.1)
        self.declare_parameter("require_both_cameras", False)
        self.declare_parameter("fallback_merge_rate_hz", 15.0)
        self.declare_parameter("cloud_max_age_s", 0.5)
        self.declare_parameter("mounts_config_path", "")
        self.declare_parameter("active_mount", "8_cm_cam_mount")
        self.declare_parameter(
            "bbox_fallback_mode", "cache"
        )  # skip | cache | conservative
        self.declare_parameter("bbox_cache_max_age_s", 2.0)
        self.declare_parameter("wait_for_tf", True)
        self.declare_parameter("tf_ready_check_interval", 2.0)
        self.declare_parameter("bbox_lookup_timeout_s", 0.02)
        self.declare_parameter("max_processing_age_s", 5.0)

        # ── Quality-first registration parameters ────────────────────────
        self.declare_parameter("enable_quality_registration", True)
        self.declare_parameter("registration_voxel_size", 0.03)
        self.declare_parameter("registration_max_points", 500)
        self.declare_parameter("global_max_iterations", 250)
        self.declare_parameter("global_candidate_count", 120)
        self.declare_parameter("global_correspondence_distance_m", 0.08)
        self.declare_parameter("global_min_correspondences", 30)
        self.declare_parameter("global_min_fitness", 0.08)
        self.declare_parameter("global_max_correction_m", 0.5)
        self.declare_parameter("global_max_correction_deg", 30.0)
        self.declare_parameter("icp_max_iterations", 20)
        self.declare_parameter("icp_correspondence_distance_m", 0.05)
        self.declare_parameter("icp_min_correspondences", 30)
        self.declare_parameter("icp_min_fitness", 0.10)
        self.declare_parameter("icp_max_rmse_m", 0.10)
        self.declare_parameter("icp_max_correction_m", 0.5)
        self.declare_parameter("icp_max_correction_deg", 30.0)
        self.declare_parameter("icp_convergence_translation_m", 0.001)
        self.declare_parameter("icp_convergence_rotation_deg", 0.2)

        # ── Color-guided registration parameters ─────────────────────────
        self.declare_parameter("color_weight", 0.0005)
        self.declare_parameter("color_inlier_threshold", 50.0)

        # ── RANSAC alignment parameters ─────────────────────────────────
        # Disabled by default for live operation: TF already places both
        # clouds in target_frame, and per-frame RANSAC can be expensive and
        # can introduce visible lag if it chases weak/ambiguous geometry.
        self.declare_parameter("enable_ransac_alignment", True)
        self.declare_parameter("ransac_min_correspondences", 8)
        self.declare_parameter("ransac_max_iterations", 100)
        self.declare_parameter("ransac_inlier_distance_m", 0.03)
        self.declare_parameter("ransac_downsample_max_points", 500)
        self.declare_parameter("ransac_min_inlier_ratio", 0.08)
        self.declare_parameter("ransac_max_correction_m", 1.0)
        self.declare_parameter("ransac_max_correction_deg", 360.0)

        # ── Read parameters ───────────────────────────────────────────────
        self._target_frame = self.get_parameter("target_frame").value
        self._arm_frame = self.get_parameter("arm_frame").value
        self._max_distance = self.get_parameter("max_distance").value
        self._voxel_size = self.get_parameter("voxel_size").value
        self._bbox_min = np.array(
            self.get_parameter("bbox_min").value, dtype=np.float32
        )
        self._bbox_max = np.array(
            self.get_parameter("bbox_max").value, dtype=np.float32
        )
        self._enable_downsampling = self.get_parameter("enable_downsampling").value
        self._enable_distance_filter = self.get_parameter(
            "enable_distance_filter"
        ).value
        self._enable_hand_removal = self.get_parameter("enable_hand_removal").value
        cam1_topic = self.get_parameter("cam1_topic").value
        cam2_topic = self.get_parameter("cam2_topic").value
        output_topic = self.get_parameter("output_topic").value
        sync_tol = self.get_parameter("sync_tolerance_s").value
        self._require_both = self.get_parameter("require_both_cameras").value
        fallback_rate = max(
            float(self.get_parameter("fallback_merge_rate_hz").value), 1.0
        )
        self._cloud_max_age = float(self.get_parameter("cloud_max_age_s").value)
        self._bbox_fallback_mode = self.get_parameter("bbox_fallback_mode").value
        self._bbox_cache_max_age = float(
            self.get_parameter("bbox_cache_max_age_s").value
        )
        self._enable_ransac = self.get_parameter("enable_ransac_alignment").value
        self._ransac_min_corr = self.get_parameter("ransac_min_correspondences").value
        self._ransac_max_iter = self.get_parameter("ransac_max_iterations").value
        self._ransac_inlier_dist = self.get_parameter("ransac_inlier_distance_m").value
        self._ransac_max_points = self.get_parameter(
            "ransac_downsample_max_points"
        ).value
        self._ransac_min_inlier_ratio = float(
            self.get_parameter("ransac_min_inlier_ratio").value
        )
        self._ransac_max_correction = float(
            self.get_parameter("ransac_max_correction_m").value
        )
        self._ransac_max_correction_rad = np.deg2rad(
            float(self.get_parameter("ransac_max_correction_deg").value)
        )
        self._wait_for_tf = self.get_parameter("wait_for_tf").value
        self._bbox_lookup_timeout = float(
            self.get_parameter("bbox_lookup_timeout_s").value
        )
        self._max_processing_age = float(
            self.get_parameter("max_processing_age_s").value
        )
        self._tf_ready_check_interval = float(
            self.get_parameter("tf_ready_check_interval").value
        )
        self._enable_quality_registration = self.get_parameter(
            "enable_quality_registration"
        ).value
        self._registration_voxel_size = float(
            self.get_parameter("registration_voxel_size").value
        )
        self._registration_max_points = int(
            self.get_parameter("registration_max_points").value
        )
        self._global_max_iter = int(self.get_parameter("global_max_iterations").value)
        self._global_candidate_count = int(
            self.get_parameter("global_candidate_count").value
        )
        self._global_corr_dist = float(
            self.get_parameter("global_correspondence_distance_m").value
        )
        self._global_min_corr = int(
            self.get_parameter("global_min_correspondences").value
        )
        self._global_min_fitness = float(self.get_parameter("global_min_fitness").value)
        self._global_max_correction = float(
            self.get_parameter("global_max_correction_m").value
        )
        self._global_max_correction_rad = np.deg2rad(
            float(self.get_parameter("global_max_correction_deg").value)
        )
        self._icp_max_iter = int(self.get_parameter("icp_max_iterations").value)
        self._icp_corr_dist = float(
            self.get_parameter("icp_correspondence_distance_m").value
        )
        self._icp_min_corr = int(self.get_parameter("icp_min_correspondences").value)
        self._icp_min_fitness = float(self.get_parameter("icp_min_fitness").value)
        self._icp_max_rmse = float(self.get_parameter("icp_max_rmse_m").value)
        self._icp_max_correction = float(
            self.get_parameter("icp_max_correction_m").value
        )
        self._icp_max_correction_rad = np.deg2rad(
            float(self.get_parameter("icp_max_correction_deg").value)
        )
        self._icp_conv_translation = float(
            self.get_parameter("icp_convergence_translation_m").value
        )
        self._icp_conv_rotation_rad = np.deg2rad(
            float(self.get_parameter("icp_convergence_rotation_deg").value)
        )
        self._color_weight = float(self.get_parameter("color_weight").value)
        self._color_inlier_threshold = float(
            self.get_parameter("color_inlier_threshold").value
        )

        # ── Load pruning boxes from camera_mounts.yaml or fall back to params ─
        mounts_config = self.get_parameter("mounts_config_path").value
        if mounts_config:
            self._pruning_boxes = self._load_pruning_boxes(
                mounts_config, self.get_parameter("active_mount").value
            )
            for i, (frame, bmin, bmax) in enumerate(self._pruning_boxes):
                self.get_logger().info(
                    f"Pruning box {i}: frame={frame}, "
                    f"min={bmin.tolist()}, max={bmax.tolist()}"
                )
        else:
            self._pruning_boxes = [(self._arm_frame, self._bbox_min, self._bbox_max)]
            self.get_logger().info(
                f"Pruning box 0 (fallback): frame={self._arm_frame}, "
                f"min={self._bbox_min.tolist()}, max={self._bbox_max.tolist()}"
            )

        # ── TF2 ───────────────────────────────────────────────────────────
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── Publishers ────────────────────────────────────────────────────
        self._pub = self.create_publisher(PointCloud2, output_topic, 5)
        self._bbox_marker_pub = self.create_publisher(
            Marker, "/pointcloud_fusion/hand_removal_bbox", 1
        )

        # ── Subscriptions ─────────────────────────────────────────────────
        # Use RELIABLE QoS — RealSense publishers use RELIABLE
        cloud_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Always set up individual subscriptions for the fallback timer merge.
        self._cam1_cloud = None
        self._cam2_cloud = None
        self._cam1_stamp = None
        self._cam2_stamp = None
        self._input_seq = 0
        self._processing_active = False
        self._processing_started_ns = 0
        self._lock = threading.Lock()
        self.create_subscription(PointCloud2, cam1_topic, self._cb_cam1, cloud_qos)
        self.create_subscription(PointCloud2, cam2_topic, self._cb_cam2, cloud_qos)

        if _HAS_MSG_FILTERS and self._require_both:
            # Synchronizer-only mode: require both clouds to arrive together.
            sub1 = message_filters.Subscriber(
                self, PointCloud2, cam1_topic, qos_profile=cloud_qos
            )
            sub2 = message_filters.Subscriber(
                self, PointCloud2, cam2_topic, qos_profile=cloud_qos
            )
            self._sync = ApproximateTimeSynchronizer(
                [sub1, sub2], queue_size=5, slop=sync_tol
            )
            self._sync.registerCallback(self._synced_callback)
            self._fallback_timer = None
            self.get_logger().info(
                f"Using ApproximateTimeSynchronizer (tolerance={sync_tol}s, require_both=True)"
            )
        else:
            # Fallback timer merge: process whichever clouds are fresh.
            self._sync = None
            self._fallback_timer = self.create_timer(
                1.0 / fallback_rate, self._timer_merge
            )
            mode_desc = (
                "require_both=True but message_filters unavailable"
                if self._require_both
                else "require_both=False"
            )
            self.get_logger().info(
                f"Using timer-based merge ({mode_desc}, "
                f"rate={fallback_rate}Hz, max_age={self._cloud_max_age}s)"
            )

        # ── BBox visualization timer ──────────────────────────────────────
        self.create_timer(1.0, self._publish_bbox_marker)

        # ── Stats ─────────────────────────────────────────────────────────
        self._stats_lock = threading.Lock()
        self._stats = {
            "published": 0,
            "cam1_only": 0,
            "dual": 0,
            "distance_removed": 0,
            "bbox_removed": 0,
            "bbox_skipped": 0,
            "bbox_cache_hits": 0,
            "processing_busy_skips": 0,
            "processing_age_drops": 0,
            "processing_ms_sum": 0.0,
            "processing_ms_max": 0.0,
            "ransac_used": 0,
            "ransac_skipped": 0,
            "ransac_inliers": 0,
            "registration_used": 0,
            "registration_skipped": 0,
            "global_used": 0,
            "icp_used": 0,
            "registration_fitness_sum": 0.0,
            "registration_rmse_sum": 0.0,
            "tf_fail": {},
        }
        self._last_publish_time = self.get_clock().now()
        self.create_timer(10.0, self._log_stats)

        # ── Bbox transform cache ───────────────────────────────────────────
        # Caches the most recent successful transform for each pruning box
        # frame as (R, t_vec, timestamp_ns). Used when fresh TF lookup fails
        # and bbox_fallback_mode == "cache".
        self._bbox_transform_cache: dict[
            str, tuple
        ] = {}  # frame -> (R, t_vec, cache_time_ns)

        # ── Bbox health tracking ───────────────────────────────────────────
        self._bbox_attempts = 0
        self._bbox_successes = 0
        self._bbox_health_window_start = self.get_clock().now()
        self.create_timer(30.0, self._check_bbox_health)

        # ── TF wait gate ───────────────────────────────────────────────────
        # When wait_for_tf is True, the fusion node waits for the TF tree to
        # be fully connected (marker_map -> depth_optical_frame) before
        # processing any clouds.  This eliminates the 40-80s startup race
        # where every cloud TF lookup fails.
        self._tf_ready = not self._wait_for_tf  # if gate disabled, ready immediately
        self._tf_ready_time = None  # set when _tf_ready transitions to True
        self._node_start_time = None
        if self._wait_for_tf:
            self._node_start_time = self.get_clock().now()
            # Check immediately, then periodically.
            self._check_tf_ready()
            self._tf_ready_timer = self.create_timer(
                self._tf_ready_check_interval, self._check_tf_ready
            )
            self.get_logger().info(
                f"TF wait gate active: waiting for TF tree to connect "
                f"(checking every {self._tf_ready_check_interval:.1f}s)"
            )
        else:
            self._tf_ready_timer = None
            self.get_logger().info(
                "TF wait gate DISABLED (wait_for_tf=False) — "
                "processing clouds immediately"
            )

        ransac_str = (
            f"RANSAC alignment: ON (min_corr={self._ransac_min_corr}, "
            f"iter={self._ransac_max_iter}, dist={self._ransac_inlier_dist}m)"
            if self._enable_ransac
            else "RANSAC alignment: OFF"
        )
        registration_str = (
            f"quality_registration=ON (voxel={self._registration_voxel_size}m, "
            f"max_points={self._registration_max_points}, "
            f"global_iter={self._global_max_iter}, "
            f"global_candidates={self._global_candidate_count}, "
            f"icp_iter={self._icp_max_iter}, "
            f"color_weight={self._color_weight}, "
            f"color_inlier_threshold={self._color_inlier_threshold})"
            if self._enable_quality_registration
            else "quality_registration=OFF"
        )
        self.get_logger().info(
            f"Pointcloud fusion: {cam1_topic} + {cam2_topic} -> {output_topic} "
            f"(target_frame={self._target_frame}, arm_frame={self._arm_frame}, "
            f"max_dist={self._max_distance}m, voxel={self._voxel_size}m, "
            f"pruning_boxes={len(self._pruning_boxes)}, "
            f"bbox_fallback={self._bbox_fallback_mode}, "
            f"bbox_timeout={self._bbox_lookup_timeout:.3f}s, "
            f"max_processing_age={self._max_processing_age:.2f}s, "
            f"{registration_str}, {ransac_str})"
        )

    # ── Synced callback (message_filters) ────────────────────────────────

    def _synced_callback(self, msg1: PointCloud2, msg2: PointCloud2):
        """Called when both clouds arrive within sync tolerance."""
        if not self._tf_ready:
            return

        now = self.get_clock().now()
        with self._lock:
            self._cam1_cloud = msg1
            self._cam2_cloud = msg2
            self._cam1_stamp = now
            self._cam2_stamp = now
            self._input_seq += 1
            if self._processing_active:
                with self._stats_lock:
                    self._stats["processing_busy_skips"] += 1
                return

        self._start_latest_processing(now)

    # ── Fallback individual callbacks ────────────────────────────────────

    def _cb_cam1(self, msg: PointCloud2):
        with self._lock:
            self._cam1_cloud = msg
            self._cam1_stamp = self.get_clock().now()
            self._input_seq += 1

    def _cb_cam2(self, msg: PointCloud2):
        with self._lock:
            self._cam2_cloud = msg
            self._cam2_stamp = self.get_clock().now()
            self._input_seq += 1

    def _timer_merge(self):
        """Start one latest-cloud processing job when the worker is idle.

        Incoming clouds are stored by the callbacks with depth=1 QoS.  If
        processing is slower than camera rate, intermediate frames are skipped;
        after each publish the next worker snapshots the newest cached clouds.
        """
        if not self._tf_ready:
            return
        now = self.get_clock().now()
        with self._lock:
            if self._processing_active:
                active_age = 0.0
                if self._processing_started_ns:
                    active_age = (now.nanoseconds - self._processing_started_ns) / 1e9
                with self._stats_lock:
                    self._stats["processing_busy_skips"] += 1
                if active_age > 1.0:
                    self.get_logger().warn(
                        f"Fusion processing still active after {active_age:.2f}s; "
                        "will process newest cached clouds when it finishes",
                        throttle_duration_sec=2.0,
                    )
                return

        self._start_latest_processing(now)

    def _start_latest_processing(self, now=None):
        """Snapshot newest cached clouds and launch one background worker."""
        if now is None:
            now = self.get_clock().now()

        with self._lock:
            if self._processing_active:
                return False

            c1, s1 = self._cam1_cloud, self._cam1_stamp
            c2, s2 = self._cam2_cloud, self._cam2_stamp

            clouds = []
            newest_stamp = None
            if c1 is not None and s1 is not None:
                age = (now - s1).nanoseconds / 1e9
                if age <= self._cloud_max_age:
                    clouds.append(c1)
                    newest_stamp = (
                        s1
                        if newest_stamp is None or s1 > newest_stamp
                        else newest_stamp
                    )
            if c2 is not None and s2 is not None:
                age = (now - s2).nanoseconds / 1e9
                if age <= self._cloud_max_age:
                    clouds.append(c2)
                    newest_stamp = (
                        s2
                        if newest_stamp is None or s2 > newest_stamp
                        else newest_stamp
                    )

            if not clouds:
                return False

            self._processing_active = True
            self._processing_started_ns = now.nanoseconds

        # Process in a daemon thread so the executor is not blocked.  Only one
        # worker is allowed at a time; new arrivals update the cached snapshot
        # for the next cycle rather than invalidating the current publish.
        t = threading.Thread(
            target=self._process_clouds_worker,
            args=(clouds, newest_stamp),
            daemon=True,
        )
        t.start()
        return True

    def _process_clouds_worker(self, clouds: list[PointCloud2], newest_stamp):
        start = self.get_clock().now()
        try:
            self._process_clouds(clouds, newest_stamp)
        finally:
            duration_ms = (self.get_clock().now() - start).nanoseconds / 1e6
            with self._stats_lock:
                self._stats["processing_ms_sum"] += float(duration_ms)
                self._stats["processing_ms_max"] = max(
                    self._stats["processing_ms_max"], float(duration_ms)
                )
            with self._lock:
                self._processing_active = False
                self._processing_started_ns = 0
            self._start_latest_processing()

    # ── Core processing pipeline ─────────────────────────────────────────

    def _process_clouds(self, clouds: list[PointCloud2], newest_stamp=None):
        """Transform, merge, filter, downsample, and publish."""
        # ── Step 1: Transform all clouds to target frame ──────────────
        transformed: list[PointCloud2] = []
        for cloud in clouds:
            if cloud.header.frame_id == self._target_frame:
                transformed.append(cloud)
                continue
            try:
                t = self._tf_buffer.lookup_transform(
                    self._target_frame,
                    cloud.header.frame_id,
                    rclpy.time.Time(),
                )
                transformed.append(tf2_sensor_msgs.do_transform_cloud(cloud, t))
            except Exception as exc:
                frame = cloud.header.frame_id
                with self._stats_lock:
                    self._stats["tf_fail"][frame] = (
                        self._stats["tf_fail"].get(frame, 0) + 1
                    )
                self.get_logger().warn(
                    f"TF transform failed for {frame}: {exc}",
                    throttle_duration_sec=10.0,
                )

        if not transformed:
            # ── First batch after TF ready: log diagnostic ─────────
            if self._tf_ready and self._tf_ready_time is not None:
                age_s = (self.get_clock().now() - self._tf_ready_time).nanoseconds / 1e9
                if age_s < 5.0:
                    self.get_logger().info(
                        f"TF tree is connected but first cloud batch had "
                        f"no successful transforms — retrying on next batch"
                    )
            return

        # ── Step 2: Registration alignment (optional) ───────────────────
        ransac_inliers = 0
        ransac_used = False
        registration_result = None
        if self._enable_quality_registration and len(transformed) == 2:
            head_xyz, head_rgb = _parse_cloud(transformed[0])
            arm_xyz, arm_rgb = _parse_cloud(transformed[1])
            if len(head_xyz) >= 3 and len(arm_xyz) >= 3:
                aligned_arm, registration_result = self._quality_register(
                    head_xyz, arm_xyz, head_rgb, arm_rgb
                )
                if registration_result["used"]:
                    transformed[1] = _build_cloud(
                        aligned_arm, arm_rgb, transformed[1].header
                    )
        elif self._enable_ransac and len(transformed) == 2:
            head_xyz, _ = _parse_cloud(transformed[0])
            arm_xyz, arm_rgb = _parse_cloud(transformed[1])
            if len(head_xyz) >= 3 and len(arm_xyz) >= 3:
                aligned_arm, ransac_used, ransac_inliers = self._ransac_align(
                    head_xyz, arm_xyz
                )
                if ransac_used:
                    # Rebuild the arm PointCloud2 with aligned points
                    aligned_msg = _build_cloud(
                        aligned_arm, arm_rgb, transformed[1].header
                    )
                    transformed[1] = aligned_msg

        # ── Step 3: Concatenate clouds ────────────────────────────────
        if len(transformed) == 1:
            xyz_all, rgb_all = _parse_cloud(transformed[0])
            stamp = transformed[0].header.stamp
            with self._stats_lock:
                self._stats["cam1_only"] += 1
        else:
            # Verify field compatibility
            ref_fields = [(f.name, f.datatype, f.count) for f in transformed[0].fields]
            for i, c in enumerate(transformed[1:], 1):
                chk = [(f.name, f.datatype, f.count) for f in c.fields]
                if chk != ref_fields:
                    self.get_logger().warn(
                        f"Cloud {i} field mismatch — using first cloud only",
                        throttle_duration_sec=10.0,
                    )
                    transformed = [transformed[0]]
                    break

            parts_xyz = []
            parts_rgb = []
            for c in transformed:
                xyz, rgb = _parse_cloud(c)
                if len(xyz) > 0:
                    parts_xyz.append(xyz)
                    parts_rgb.append(rgb)

            if not parts_xyz:
                return

            xyz_all = np.concatenate(parts_xyz, axis=0)
            rgb_all = np.concatenate(parts_rgb, axis=0)
            stamp = transformed[0].header.stamp
            with self._stats_lock:
                self._stats["dual"] += 1
                if registration_result is not None:
                    if registration_result["used"]:
                        self._stats["registration_used"] += 1
                        self._stats["registration_fitness_sum"] += registration_result[
                            "fitness"
                        ]
                        self._stats["registration_rmse_sum"] += registration_result[
                            "rmse"
                        ]
                        if registration_result["global_used"]:
                            self._stats["global_used"] += 1
                        if registration_result["icp_used"]:
                            self._stats["icp_used"] += 1
                    else:
                        self._stats["registration_skipped"] += 1
                if ransac_used:
                    self._stats["ransac_used"] += 1
                    self._stats["ransac_inliers"] += ransac_inliers
                elif self._enable_ransac and not self._enable_quality_registration:
                    self._stats["ransac_skipped"] += 1

        if len(xyz_all) == 0:
            return

        # Filter NaN/Inf
        valid = np.isfinite(xyz_all).all(axis=1)
        xyz_all = xyz_all[valid]
        rgb_all = rgb_all[valid]

        # ── Step 3: Distance filter (2m from arm) ────────────────────
        if self._enable_distance_filter:
            arm_pos = self._get_frame_origin_in_target(self._arm_frame)
            if arm_pos is not None:
                mask = _distance_filter(xyz_all, arm_pos, self._max_distance)
                removed = len(xyz_all) - np.sum(mask)
                xyz_all = xyz_all[mask]
                rgb_all = rgb_all[mask]
                with self._stats_lock:
                    self._stats["distance_removed"] += int(removed)
            else:
                self.get_logger().warn(
                    f"Cannot look up {self._arm_frame} in {self._target_frame} "
                    f"for distance filter — skipping",
                    throttle_duration_sec=5.0,
                )

        if len(xyz_all) == 0:
            return

        # ── Step 4: Hand/arm bbox removal (multiple pruning boxes) ─────
        if self._enable_hand_removal:
            # Use latest-available TF for bbox lookups (rclpy.time.Time()).
            for frame, bbox_min, bbox_max in self._pruning_boxes:
                self._bbox_attempts += 1

                # Try non-blocking TF lookup with a short timeout; falls back to cache
                transform_result = self._lookup_bbox_transform(frame, xyz_all)

                if transform_result is not None:
                    R, t_vec, xyz_box = transform_result
                    # Cache the successful transform
                    cache_time = self.get_clock().now().nanoseconds
                    self._bbox_transform_cache[frame] = (R, t_vec, cache_time)
                    self._bbox_successes += 1

                    keep = _bbox_filter(xyz_box, bbox_min, bbox_max)
                    removed = len(xyz_all) - np.sum(keep)
                    xyz_all = xyz_all[keep]
                    rgb_all = rgb_all[keep]
                    with self._stats_lock:
                        self._stats["bbox_removed"] += int(removed)
                    # Throttled success logging — only visible when bbox removal
                    # is actually filtering hand/arm points.
                    if removed > 0 and self._bbox_successes % 50 == 1:
                        self.get_logger().info(
                            f"Bbox removal via fresh TF removed {removed} points "
                            f"from {frame}",
                            throttle_duration_sec=10.0,
                        )
                else:
                    # Fresh lookup failed — apply fallback strategy
                    handled = self._bbox_fallback(
                        frame, bbox_min, bbox_max, xyz_all, rgb_all
                    )
                    if handled is not None:
                        xyz_all, rgb_all = handled[0], handled[1]
                    # else: bbox_skipped already counted in _bbox_fallback

        if len(xyz_all) == 0:
            return

        # ── Step 5: Voxel downsampling ────────────────────────────────
        if self._enable_downsampling and self._voxel_size > 0.0:
            xyz_all, rgb_all = _voxel_downsample(xyz_all, rgb_all, self._voxel_size)

        if len(xyz_all) == 0:
            return

        # ── Step 6: Build and publish ─────────────────────────────────
        if newest_stamp is not None:
            result_age_s = (self.get_clock().now() - newest_stamp).nanoseconds / 1e9
            if result_age_s > self._max_processing_age:
                with self._stats_lock:
                    self._stats["processing_age_drops"] += 1
                self.get_logger().warn(
                    f"Dropping stale fused cloud result age={result_age_s:.2f}s "
                    f"> max_processing_age_s={self._max_processing_age:.2f}s",
                    throttle_duration_sec=2.0,
                )
                return

        header = transformed[0].header if transformed else None
        if header is None:
            return
        header.frame_id = self._target_frame
        # Stamp with host clock so downstream consumers (twist propagation,
        # segmentation) see a fresh timestamp relative to their own clock.
        # The original cloud stamp comes from the Jetson and can be seconds
        # behind the host clock due to network transit + processing.
        header.stamp = self.get_clock().now().to_msg()

        out_msg = _build_cloud(xyz_all, rgb_all, header)
        self._pub.publish(out_msg)
        with self._stats_lock:
            self._stats["published"] += 1
        self._last_publish_time = self.get_clock().now()

    # ── Quality registration alignment ───────────────────────────────────

    def _quality_register(
        self,
        target_xyz: np.ndarray,
        source_xyz: np.ndarray,
        target_rgb: np.ndarray,
        source_rgb: np.ndarray,
    ):
        """Coarse global alignment followed by ICP refinement.

        The input clouds are already transformed into target_frame by TF.  This
        method treats TF/OpenVINS as a rough initial guess, then estimates an
        additional source->target correction from voxelized samples.  It returns
        the full-resolution source cloud after applying the accepted correction.

        RGB data is used to improve correspondence quality when available.
        """
        result = {
            "used": False,
            "global_used": False,
            "icp_used": False,
            "fitness": 0.0,
            "rmse": 0.0,
        }
        has_color = (
            target_rgb is not None
            and source_rgb is not None
            and len(target_rgb) == len(target_xyz)
            and len(source_rgb) == len(source_xyz)
            and np.any(target_rgb != 0)
            and np.any(source_rgb != 0)
        )

        if has_color:
            target_sample, target_sample_rgb = _voxel_sample_xyzrgb(
                target_xyz,
                target_rgb,
                self._registration_voxel_size,
                self._registration_max_points,
            )
            source_sample, source_sample_rgb = _voxel_sample_xyzrgb(
                source_xyz,
                source_rgb,
                self._registration_voxel_size,
                self._registration_max_points,
            )
            target_lab = _rgb_to_lab(target_sample_rgb)
            source_lab = _rgb_to_lab(source_sample_rgb)
        else:
            target_sample = _voxel_sample_xyz(
                target_xyz,
                self._registration_voxel_size,
                self._registration_max_points,
            )
            source_sample = _voxel_sample_xyz(
                source_xyz,
                self._registration_voxel_size,
                self._registration_max_points,
            )
            target_lab = None
            source_lab = None

        min_needed = max(3, min(self._global_min_corr, self._icp_min_corr))
        if len(target_sample) < min_needed or len(source_sample) < min_needed:
            self.get_logger().warn(
                f"Registration skipped: insufficient samples "
                f"target={len(target_sample)} source={len(source_sample)}",
                throttle_duration_sec=5.0,
            )
            return source_xyz, result

        # Log initial cloud separation for diagnostics
        tgt_centroid = np.mean(target_sample, axis=0)
        src_centroid = np.mean(source_sample, axis=0)
        initial_offset = float(np.linalg.norm(tgt_centroid - src_centroid))
        self.get_logger().info(
            f"Registration input: target={len(target_sample)} "
            f"source={len(source_sample)} pts, "
            f"centroid_offset={initial_offset:.3f}m, "
            f"color={has_color}",
            throttle_duration_sec=5.0,
        )

        R_total = np.eye(3, dtype=np.float64)
        t_total = np.zeros(3, dtype=np.float64)

        # ── Coarse global alignment ──────────────────────────────────────
        # Always use the combined XYZ+colour-weighted tree approach.
        # Colour helps steer correspondences away from false matches
        # (e.g. a red point matching a blue one), but spatial position is
        # the primary signal.  The pure colour-only path was removed because
        # it produced false matches with large rotations when OpenVINS
        # jumped and similarly-coloured points existed on different surfaces.
        global_fit = self._coarse_global_align(
            target_sample,
            source_sample,
            target_lab=target_lab if has_color else None,
            source_lab=source_lab if has_color else None,
        )

        if global_fit is not None:
            Rg, tg, g_fitness, g_rmse = global_fit
            self.get_logger().info(
                f"Coarse global: fitness={g_fitness:.3f} rmse={g_rmse:.4f} "
                f"trans={np.linalg.norm(tg):.4f}m",
                throttle_duration_sec=5.0,
            )
            if self._registration_transform_allowed(
                Rg, tg, self._global_max_correction, self._global_max_correction_rad
            ):
                R_total, t_total = Rg, tg
                result["global_used"] = True
                result["fitness"] = g_fitness
                result["rmse"] = g_rmse
            else:
                self.get_logger().warn(
                    "Global registration rejected: correction exceeds configured bounds",
                    throttle_duration_sec=2.0,
                )
        else:
            self.get_logger().warn(
                f"Coarse global alignment found no acceptable hypothesis "
                f"(offset={initial_offset:.2f}m)",
                throttle_duration_sec=5.0,
            )

        # ── ICP refinement ───────────────────────────────────────────────
        # Only run ICP if coarse gave us a reasonable starting point.
        # ICP from identity when clouds are meters apart is pointless.
        if result["global_used"] or initial_offset < 0.15:
            icp_fit = self._icp_refine(
                target_sample,
                source_sample,
                R_total,
                t_total,
                target_lab=target_lab if has_color else None,
                source_lab=source_lab if has_color else None,
            )
            if icp_fit is not None:
                Ri, ti, i_fitness, i_rmse = icp_fit
                self.get_logger().info(
                    f"ICP refine: fitness={i_fitness:.3f} rmse={i_rmse:.4f} "
                    f"trans={np.linalg.norm(ti):.4f}m",
                    throttle_duration_sec=5.0,
                )
                if self._registration_transform_allowed(
                    Ri, ti, self._icp_max_correction, self._icp_max_correction_rad
                ):
                    R_total, t_total = Ri, ti
                    result["icp_used"] = True
                    result["fitness"] = i_fitness
                    result["rmse"] = i_rmse
                else:
                    self.get_logger().warn(
                        "ICP registration rejected: correction exceeds configured bounds",
                        throttle_duration_sec=2.0,
                    )
            else:
                self.get_logger().warn(
                    "ICP refinement failed to converge",
                    throttle_duration_sec=5.0,
                )
        else:
            self.get_logger().info(
                "Skipping ICP: no coarse result and clouds too far apart",
                throttle_duration_sec=5.0,
            )

        result["used"] = result["global_used"] or result["icp_used"]
        if not result["used"]:
            self.get_logger().warn(
                "Registration skipped: no acceptable global/ICP correction found",
                throttle_duration_sec=3.0,
            )
            return source_xyz, result

        # ── Sanity check: centroid must get closer ─────────────────────────
        # If the registration moves the source centroid farther from the
        # target centroid, the correction is physically wrong (e.g. the
        # color-only correspondence found a false match with a large rotation).
        src_centroid = np.mean(source_xyz, axis=0).astype(np.float64)
        tgt_centroid = np.mean(target_xyz, axis=0).astype(np.float64)
        new_src_centroid = (src_centroid @ R_total.T) + t_total
        before_dist = float(np.linalg.norm(tgt_centroid - src_centroid))
        after_dist = float(np.linalg.norm(tgt_centroid - new_src_centroid))
        if after_dist > before_dist * 1.1:  # allow 10% tolerance
            self.get_logger().warn(
                f"Registration rejected: centroid moved farther apart "
                f"({before_dist:.3f}m -> {after_dist:.3f}m)",
                throttle_duration_sec=3.0,
            )
            result["used"] = False
            result["global_used"] = False
            result["icp_used"] = False
            return source_xyz, result

        aligned = (source_xyz.astype(np.float64) @ R_total.T) + t_total
        self.get_logger().info(
            f"Registration applied: global={result['global_used']} "
            f"icp={result['icp_used']} fitness={result['fitness']:.3f} "
            f"rmse={result['rmse']:.3f} trans={np.linalg.norm(t_total):.3f}m "
            f"rot={np.rad2deg(_rotation_angle_rad(R_total)):.1f}deg",
            throttle_duration_sec=2.0,
        )
        return aligned.astype(np.float32), result

    # NOTE: _coarse_color_only is currently unused (see _quality_register).
    # It was removed from the active path because pure colour-based
    # correspondence produced false matches with large rotations when
    # OpenVINS jumped.  Kept for potential future use if a better
    # colour-only strategy is designed.
    def _coarse_color_only(
        self,
        target_sample: np.ndarray,
        source_sample: np.ndarray,
        target_lab: np.ndarray,
        source_lab: np.ndarray,
    ):
        """Coarse alignment using pure color correspondence in Lab space.

        This is designed for the case where the two clouds are far apart
        spatially (e.g. OpenVINS jumped 2+ metres) so spatial nearest-neighbour
        correspondences are meaningless.  Instead we find correspondences based
        solely on color similarity, estimate a rigid transform from those, and
        score it.

        The algorithm:
        1. Build a KD-tree on target Lab colors.
        2. For each RANSAC iteration, pick 3 source points.
        3. Find their nearest color neighbors in the target.
        4. Estimate rigid transform from the 3 source->target pairs.
        5. Score: apply transform to all source points, find spatial NN in
           target, count inliers within correspondence distance.
        6. Return the best transform.
        """
        rng = np.random.default_rng()
        target_tree = cKDTree(target_sample)
        target_lab_tree = cKDTree(target_lab)
        best = None
        best_inliers = 0
        best_rmse = float("inf")
        max_src = len(source_sample)
        if max_src < 3 or len(target_sample) < 3:
            return None

        corr_dist = self._global_corr_dist
        color_inlier_threshold = self._color_inlier_threshold

        # Score against a subset for speed
        max_score_points = max(3, min(self._global_candidate_count, max_src))
        if max_src > max_score_points:
            score_idx = rng.choice(max_src, max_score_points, replace=False)
        else:
            score_idx = np.arange(max_src)
        score_sample = source_sample[score_idx]
        score_lab = source_lab[score_idx]

        min_score_inliers = max(3, int(np.ceil(self._global_min_fitness * len(score_sample))))

        # Use fewer iterations for color-only since it's more exploratory
        max_iter = max(1, min(self._global_max_iter, 500))

        for _ in range(max_iter):
            idx = rng.choice(max_src, 3, replace=False)
            src_pts = source_sample[idx].astype(np.float64)
            src_lab = source_lab[idx]

            # Find nearest COLOR neighbors (ignoring position entirely)
            _, nn_idx = target_lab_tree.query(src_lab.astype(np.float64), k=1)
            tgt_pts = target_sample[nn_idx].astype(np.float64)

            # Check that the 3 source points are not degenerate
            if (
                np.linalg.norm(src_pts[0] - src_pts[1]) < 0.015
                or np.linalg.norm(src_pts[0] - src_pts[2]) < 0.015
                or np.linalg.norm(src_pts[1] - src_pts[2]) < 0.015
            ):
                continue

            estimated = _estimate_rigid_transform(src_pts, tgt_pts)
            if estimated is None:
                continue
            R, t = estimated

            # Score: transform score subset and check spatial overlap
            transformed = (score_sample.astype(np.float64) @ R.T) + t
            dists, nn_dists_idx = target_tree.query(transformed, k=1)
            spatial_mask = dists < corr_dist

            # Also check color consistency of the correspondences
            tgt_lab_corr = target_lab[nn_dists_idx]
            color_dists = np.sqrt(np.sum((score_lab - tgt_lab_corr) ** 2, axis=1))
            color_mask = color_dists < color_inlier_threshold
            inlier_mask = spatial_mask & color_mask

            inliers = int(np.sum(inlier_mask))
            if inliers < min_score_inliers:
                continue
            rmse = float(np.sqrt(np.mean(dists[inlier_mask] ** 2)))
            if inliers > best_inliers or (inliers == best_inliers and rmse < best_rmse):
                best_inliers = inliers
                best_rmse = rmse
                best = (R, t)
                if inliers >= int(0.8 * len(score_sample)):
                    break

        if best is None:
            return None

        # Validate on full sample
        R_best, t_best = best
        full_transformed = (source_sample.astype(np.float64) @ R_best.T) + t_best
        full_dists, full_nn_idx = target_tree.query(full_transformed, k=1)
        full_spatial_mask = full_dists < corr_dist
        full_tgt_lab = target_lab[full_nn_idx]
        full_color_dists = np.sqrt(
            np.sum((source_lab - full_tgt_lab) ** 2, axis=1)
        )
        full_color_mask = full_color_dists < color_inlier_threshold
        full_inlier_mask = full_spatial_mask & full_color_mask
        full_inliers = int(np.sum(full_inlier_mask))
        fitness = full_inliers / max(len(source_sample), 1)
        if full_inliers < min(self._global_min_corr, len(source_sample)):
            return None
        if fitness < self._global_min_fitness:
            return None
        full_rmse = float(np.sqrt(np.mean(full_dists[full_inlier_mask] ** 2)))
        return R_best, t_best, float(fitness), full_rmse

    def _coarse_global_align(
        self,
        target_sample: np.ndarray,
        source_sample: np.ndarray,
        target_lab: np.ndarray | None = None,
        source_lab: np.ndarray | None = None,
    ):
        """RANSAC-style coarse alignment on voxelized samples.

        When target_lab / source_lab are provided, correspondences are
        filtered by color similarity in Lab space.  This dramatically
        improves registration when the scene has distinctive colors (e.g.
        colored objects, textured surfaces) even when the geometric
        structure is ambiguous.

        Keep scoring bounded: evaluating every hypothesis against every
        registration point is too slow in Python.  Hypotheses are generated
        from the full samples, then scored against a capped deterministic
        subset before the best candidate is checked against thresholds.
        """
        rng = np.random.default_rng()
        target_tree = cKDTree(target_sample)
        best = None
        best_inliers = 0
        best_rmse = float("inf")
        best_color_score = -1.0
        max_src = len(source_sample)
        if max_src < 3 or len(target_sample) < 3:
            return None

        use_color = (
            target_lab is not None
            and source_lab is not None
            and len(target_lab) == len(target_sample)
            and len(source_lab) == len(source_sample)
        )

        # If color is available, build a combined XYZ+weighted-Lab tree
        # for finding better initial correspondences.
        color_weight = self._color_weight if use_color else 0.0
        if use_color and color_weight > 0.0:
            # Normalize Lab to comparable scale as XYZ (in metres).
            # Lab delta-E ranges 0-100; scale so color_weight controls
            # the relative importance.
            tgt_combined = np.hstack(
                [target_sample, target_lab[:, :3] * color_weight]
            )
            src_combined = np.hstack(
                [source_sample, source_lab[:, :3] * color_weight]
            )
            combined_tree = cKDTree(tgt_combined)
        else:
            combined_tree = None

        corr_dist = self._global_corr_dist
        color_inlier_threshold = self._color_inlier_threshold
        score_sample = source_sample
        score_lab = source_lab if use_color else None
        max_score_points = max(3, min(self._global_candidate_count, len(source_sample)))
        if len(score_sample) > max_score_points:
            keep_idx = rng.choice(len(score_sample), max_score_points, replace=False)
            score_sample = score_sample[keep_idx]
            if score_lab is not None:
                score_lab = score_lab[keep_idx]

        min_score_inliers = max(
            3,
            int(np.ceil(self._global_min_fitness * len(score_sample))),
        )
        iterations_run = 0
        for _ in range(max(1, self._global_max_iter)):
            iterations_run += 1
            idx = rng.choice(max_src, 3, replace=False)
            src_pts = source_sample[idx].astype(np.float64)
            if (
                np.linalg.norm(src_pts[0] - src_pts[1]) < 0.015
                or np.linalg.norm(src_pts[0] - src_pts[2]) < 0.015
                or np.linalg.norm(src_pts[1] - src_pts[2]) < 0.015
            ):
                continue

            # Find correspondences using combined tree if color is available,
            # otherwise fall back to pure spatial nearest neighbor.
            if combined_tree is not None:
                src_query = np.hstack(
                    [
                        src_pts,
                        source_lab[idx].astype(np.float64) * color_weight,
                    ]
                )
                _, nn_idx = combined_tree.query(src_query, k=1)
            else:
                _, nn_idx = target_tree.query(src_pts, k=1)
            tgt_pts = target_sample[nn_idx]
            estimated = _estimate_rigid_transform(src_pts, tgt_pts)
            if estimated is None:
                continue
            R, t = estimated
            transformed = (score_sample.astype(np.float64) @ R.T) + t
            dists, nn_dists_idx = target_tree.query(transformed, k=1)
            spatial_mask = dists < corr_dist

            # Spatial inliers only — color is a soft tiebreaker below
            inlier_mask = spatial_mask
            inliers = int(np.sum(inlier_mask))
            if inliers < min_score_inliers:
                continue
            rmse = float(np.sqrt(np.mean(dists[inlier_mask] ** 2)))

            # Soft color tiebreaker: when spatial inliers and RMSE are
            # tied, prefer the hypothesis with better color consistency.
            color_inliers = 0
            if use_color and score_lab is not None:
                tgt_lab_corr = target_lab[nn_dists_idx[inlier_mask]]
                src_lab_corr = score_lab[inlier_mask]
                color_dists = np.sqrt(
                    np.sum((src_lab_corr - tgt_lab_corr) ** 2, axis=1)
                )
                color_inliers = int(np.sum(color_dists < color_inlier_threshold))

            if (
                inliers > best_inliers
                or (inliers == best_inliers and rmse < best_rmse)
                or (inliers == best_inliers and abs(rmse - best_rmse) < 1e-6
                    and color_inliers > best_color_score)
            ):
                best_inliers = inliers
                best_rmse = rmse
                best_color_score = color_inliers
                best = (R, t)
                if inliers >= int(0.8 * len(score_sample)):
                    break

        if best is None:
            self.get_logger().debug(
                f"Coarse align: no hypothesis accepted after "
                f"{iterations_run} iterations "
                f"(target={len(target_sample)} src={len(source_sample)} "
                f"score_pts={len(score_sample)} min_inliers={min_score_inliers} "
                f"use_color={use_color})"
            )
            return None

        R_best, t_best = best
        full_transformed = (source_sample.astype(np.float64) @ R_best.T) + t_best
        full_dists, full_nn_idx = target_tree.query(full_transformed, k=1)
        full_spatial_mask = full_dists < corr_dist
        full_inlier_mask = full_spatial_mask
        full_inliers = int(np.sum(full_inlier_mask))
        fitness = full_inliers / max(len(source_sample), 1)
        if full_inliers < min(self._global_min_corr, len(source_sample)):
            self.get_logger().debug(
                f"Coarse align: full validation failed — "
                f"inliers={full_inliers} < min_corr={self._global_min_corr}"
            )
            return None
        if fitness < self._global_min_fitness:
            self.get_logger().debug(
                f"Coarse align: fitness={fitness:.3f} < min={self._global_min_fitness}"
            )
            return None
        full_rmse = float(np.sqrt(np.mean(full_dists[full_inlier_mask] ** 2)))
        return R_best, t_best, float(fitness), full_rmse

    def _icp_refine(
        self,
        target_sample: np.ndarray,
        source_sample: np.ndarray,
        R_init: np.ndarray,
        t_init: np.ndarray,
        target_lab: np.ndarray | None = None,
        source_lab: np.ndarray | None = None,
    ):
        """Point-to-point ICP refinement on voxelized samples.

        When target_lab / source_lab are provided, correspondences are
        additionally filtered by color similarity in Lab space.
        """
        target_tree = cKDTree(target_sample)
        R_total = R_init.copy()
        t_total = t_init.copy()
        last_rmse = None
        best_fitness = 0.0
        best_rmse = float("inf")

        use_color = (
            target_lab is not None
            and source_lab is not None
            and len(target_lab) == len(target_sample)
            and len(source_lab) == len(source_sample)
        )
        color_inlier_threshold = self._color_inlier_threshold

        for _ in range(max(1, self._icp_max_iter)):
            transformed = (source_sample.astype(np.float64) @ R_total.T) + t_total
            dists, nn_idx = target_tree.query(transformed, k=1)
            mask = dists < self._icp_corr_dist

            corr_count = int(np.sum(mask))
            if corr_count < self._icp_min_corr:
                return None

            src_corr = transformed[mask]
            tgt_corr = target_sample[nn_idx[mask]]
            update = _estimate_rigid_transform(src_corr, tgt_corr)
            if update is None:
                return None
            R_delta, t_delta = update
            R_total, t_total = _compose_transform(R_delta, t_delta, R_total, t_total)

            rmse = float(np.sqrt(np.mean(dists[mask] ** 2)))
            fitness = corr_count / max(len(source_sample), 1)
            best_fitness, best_rmse = fitness, rmse
            if last_rmse is not None and abs(last_rmse - rmse) < 1e-5:
                break
            last_rmse = rmse
            if (
                np.linalg.norm(t_delta) < self._icp_conv_translation
                and _rotation_angle_rad(R_delta) < self._icp_conv_rotation_rad
            ):
                break

        if best_fitness < self._icp_min_fitness or best_rmse > self._icp_max_rmse:
            return None
        return R_total, t_total, float(best_fitness), float(best_rmse)

    def _registration_transform_allowed(
        self,
        R: np.ndarray,
        t: np.ndarray,
        max_translation: float,
        max_rotation_rad: float,
    ) -> bool:
        if np.linalg.norm(t) > max_translation:
            return False
        if _rotation_angle_rad(R) > max_rotation_rad:
            return False
        return True

    # ── RANSAC alignment ────────────────────────────────────────────────

    def _ransac_align(self, target_xyz: np.ndarray, source_xyz: np.ndarray):
        """RANSAC-based rigid registration of source onto target.

        Uses 3-point correspondences with SVD (Kabsch) to find the best
        rigid transform, then counts inliers via nearest-neighbour search.

        Parameters
        ----------
        target_xyz : (N,3) float32 — points to align TO (head cloud)
        source_xyz : (M,3) float32 — points to align FROM (arm cloud)

        Returns
        -------
        aligned : (M,3) float32 — source points transformed to target frame
        ransac_used : bool       — True if alignment was applied
        inliers : int           — number of inlier correspondences found
        """
        n_src = min(len(source_xyz), self._ransac_max_points)
        n_tgt = min(len(target_xyz), self._ransac_max_points)

        # Downsample randomly for speed
        rng = np.random.default_rng()
        src_sample = source_xyz[rng.choice(len(source_xyz), n_src, replace=False)]
        tgt_sample = target_xyz[rng.choice(len(target_xyz), n_tgt, replace=False)]

        # Build KD-Tree on target for fast nearest-neighbor queries
        tree = cKDTree(tgt_sample)

        best_inliers = 0
        best_R = np.eye(3, dtype=np.float64)
        best_t = np.zeros(3, dtype=np.float64)

        for _ in range(self._ransac_max_iter):
            # Pick 3 random non-collinear points from source
            idx = rng.choice(n_src, 3, replace=False)
            src_pts = src_sample[idx].astype(np.float64)

            # Skip if points are too close (degenerate triangle)
            d01 = np.linalg.norm(src_pts[0] - src_pts[1])
            d02 = np.linalg.norm(src_pts[0] - src_pts[2])
            d12 = np.linalg.norm(src_pts[1] - src_pts[2])
            if d01 < 0.01 or d02 < 0.01 or d12 < 0.01:
                continue

            # Find nearest neighbours in target
            _, nn_idx = tree.query(src_pts)
            tgt_pts = tgt_sample[nn_idx].astype(np.float64)

            # Compute rigid transform (Kabsch / SVD)
            src_mean = np.mean(src_pts, axis=0)
            tgt_mean = np.mean(tgt_pts, axis=0)
            src_c = src_pts - src_mean
            tgt_c = tgt_pts - tgt_mean
            H = src_c.T @ tgt_c
            U, S, Vt = np.linalg.svd(H)
            R = Vt.T @ U.T
            if np.linalg.det(R) < 0:
                Vt[2, :] *= -1
                R = Vt.T @ U.T
            t = tgt_mean - R @ src_mean

            # Count inliers on source sample (fast)
            aligned_sample = (src_sample.astype(np.float64) @ R.T) + t
            dists, _ = tree.query(aligned_sample, k=1)
            inliers = int(np.sum(dists < self._ransac_inlier_dist))

            if inliers > best_inliers:
                best_inliers = inliers
                best_R, best_t = R, t

                # Early exit if we have enough inliers
                if best_inliers >= 3 * self._ransac_min_corr:
                    break

        inlier_ratio = best_inliers / max(n_src, 1)
        correction_t = float(np.linalg.norm(best_t))
        trace = float(np.clip((np.trace(best_R) - 1.0) * 0.5, -1.0, 1.0))
        correction_angle = float(np.arccos(trace))

        if best_inliers >= self._ransac_min_corr:
            if inlier_ratio < self._ransac_min_inlier_ratio:
                self.get_logger().debug(
                    f"RANSAC skipped: inlier_ratio={inlier_ratio:.3f} "
                    f"< {self._ransac_min_inlier_ratio:.3f}"
                )
                return source_xyz, False, best_inliers
            if correction_t > self._ransac_max_correction:
                self.get_logger().warn(
                    f"RANSAC correction rejected: translation={correction_t:.3f}m "
                    f"> {self._ransac_max_correction:.3f}m",
                    throttle_duration_sec=2.0,
                )
                return source_xyz, False, best_inliers
            if correction_angle > self._ransac_max_correction_rad:
                self.get_logger().warn(
                    f"RANSAC correction rejected: rotation="
                    f"{np.rad2deg(correction_angle):.1f}deg > "
                    f"{np.rad2deg(self._ransac_max_correction_rad):.1f}deg",
                    throttle_duration_sec=2.0,
                )
                return source_xyz, False, best_inliers

            aligned = (source_xyz.astype(np.float64) @ best_R.T) + best_t
            return aligned.astype(np.float32), True, best_inliers

        return source_xyz, False, best_inliers

    def _lookup_bbox_transform(self, frame: str, xyz_all: np.ndarray):
        """Try to look up the transform for a bbox pruning box.

        Returns (R, t_vec, xyz_box) on success, or None if lookup fails.
        Uses rclpy.time.Time() (= latest available) with a 500 ms timeout
        to accommodate multi-edge TF chains (6+ edges for palm_frame ->
        marker_map).  This matches the timeout used by tf_pipeline_diagnostics
        which checks the same chains successfully.
        """
        try:
            t = self._tf_buffer.lookup_transform(
                frame,
                self._target_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self._bbox_lookup_timeout),
            )
        except Exception as e:
            self.get_logger().debug(
                f"TF bbox lookup failed for {frame} -> {self._target_frame}: {e}"
            )
            return None

        R, t_vec = _extract_rotation_translation(t)
        xyz_box = (xyz_all.astype(np.float64) @ R.T) + t_vec
        return R, t_vec, xyz_box.astype(np.float32)

    def _bbox_fallback(self, frame: str, bbox_min, bbox_max, xyz_all, rgb_all):
        """Handle bbox removal when fresh TF lookup fails.

        Returns (xyz_all, rgb_all) if fallback was applied, or None if
        the pruning box was skipped entirely.
        """
        if self._bbox_fallback_mode == "cache":
            cached = self._bbox_transform_cache.get(frame)
            if cached is not None:
                R, t_vec, cache_time_ns = cached
                age_s = (self.get_clock().now().nanoseconds - cache_time_ns) / 1e9
                if age_s <= self._bbox_cache_max_age:
                    # Apply cached transform
                    xyz_box = (xyz_all.astype(np.float64) @ R.T) + t_vec
                    xyz_box = xyz_box.astype(np.float32)
                    keep = _bbox_filter(xyz_box, bbox_min, bbox_max)
                    removed = len(xyz_all) - np.sum(keep)
                    xyz_all = xyz_all[keep]
                    rgb_all = rgb_all[keep]
                    with self._stats_lock:
                        self._stats["bbox_removed"] += int(removed)
                        self._stats["bbox_cache_hits"] += 1
                    # Also count cache hits as functional successes
                    self._bbox_successes += 1
                    if removed > 0:
                        if self._bbox_successes % 50 == 1:
                            self.get_logger().info(
                                f"Bbox removal via cache removed {removed} points "
                                f"from {frame} (cache age={age_s:.2f}s)",
                                throttle_duration_sec=10.0,
                            )
                    self.get_logger().warn(
                        f"Using cached transform for {frame} "
                        f"(age={age_s:.2f}s) — fresh lookup failed",
                        throttle_duration_sec=5.0,
                    )
                    return xyz_all, rgb_all
                else:
                    with self._stats_lock:
                        self._stats["bbox_skipped"] += 1
                    self.get_logger().warn(
                        f"Cannot transform to {frame} for bbox removal — "
                        f"cached transform too old ({age_s:.1f}s > "
                        f"{self._bbox_cache_max_age}s), skipping",
                        throttle_duration_sec=5.0,
                    )
                    return None
            # No cache available
            with self._stats_lock:
                self._stats["bbox_skipped"] += 1
            self.get_logger().warn(
                f"Cannot transform to {frame} for bbox removal — "
                f"no cached transform available, skipping",
                throttle_duration_sec=5.0,
            )
            return None

        elif self._bbox_fallback_mode == "conservative":
            # Don't publish if bbox removal can't run — safest option.
            # Return empty arrays to signal the caller to abort.
            self.get_logger().warn(
                f"Cannot transform to {frame} for bbox removal — "
                f"conservative mode: dropping fused cloud",
                throttle_duration_sec=5.0,
            )
            return np.zeros((0, 3), dtype=np.float32), rgb_all[:0]

        else:  # "skip" mode (original behavior)
            with self._stats_lock:
                self._stats["bbox_skipped"] += 1
            self.get_logger().warn(
                f"Cannot transform to {frame} for bbox removal — skipping",
                throttle_duration_sec=5.0,
            )
            return None

    def _check_bbox_health(self):
        """Log an ERROR if bbox removal success rate is too low over a 30s window."""
        now = self.get_clock().now()
        elapsed = (now - self._bbox_health_window_start).nanoseconds / 1e9
        if self._bbox_attempts > 0 and elapsed >= 25.0:
            success_rate = self._bbox_successes / self._bbox_attempts
            if success_rate < 0.9:
                self.get_logger().error(
                    f"Bbox removal success rate is {success_rate:.0%} over "
                    f"{elapsed:.0f}s ({self._bbox_successes}/"
                    f"{self._bbox_attempts} attempts). "
                    f"Fused cloud quality is degraded — hand/arm may not be "
                    f"filtered. Check TF tree connectivity for pruning box frames."
                )
            # Reset window
            self._bbox_attempts = 0
            self._bbox_successes = 0
            self._bbox_health_window_start = now

    # ── TF helpers ────────────────────────────────────────────────────────

    def _get_frame_origin_in_target(self, frame: str) -> np.ndarray | None:
        """Get the origin of `frame` in target_frame. Returns (3,) float32 or None."""
        try:
            t = self._tf_buffer.lookup_transform(
                self._target_frame, frame, rclpy.time.Time()
            )
            return np.array(
                [
                    t.transform.translation.x,
                    t.transform.translation.y,
                    t.transform.translation.z,
                ],
                dtype=np.float32,
            )
        except Exception:
            return None

    # ── TF wait gate ───────────────────────────────────────────────────────

    # Depth optical frames checked by the gate.  Either one connecting is
    # sufficient to start fusion (head is preferred but may never initialize
    # — arm is more reliable in practice).
    _GATE_DEPTH_FRAMES = [
        "head_d435i_head_depth_optical_frame",
        "arm_d435i_arm_depth_optical_frame",
    ]

    def _check_tf_ready(self):
        """Check whether the TF tree is fully connected.

        Uses ``can_transform`` with **zero timeout** to avoid blocking
        the single-threaded executor.  A non-zero timeout (even 1 s)
        starves the /tf subscription callback, preventing the TF buffer
        from ever populating — which is why the gate never opened in v18.

        Opens the gate as soon as **either** camera chain is connected —
        the fusion node can operate single-camera (require_both=False).
        """
        if self._tf_ready:
            # Already ready — cancel the timer if it still exists.
            if self._tf_ready_timer is not None:
                self._tf_ready_timer.cancel()
                self._tf_ready_timer = None
            return

        elapsed = 0.0
        if self._node_start_time is not None:
            elapsed = (self.get_clock().now() - self._node_start_time).nanoseconds / 1e9

        diag_parts = []

        for depth_frame in self._GATE_DEPTH_FRAMES:
            try:
                connected = self._tf_buffer.can_transform(
                    self._target_frame,
                    depth_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.0),
                )
            except Exception as exc:
                diag_parts.append(f"{depth_frame}: can_transform threw '{exc}'")
                connected = False

            if connected:
                # ── This camera's TF tree is connected! ──────────────────
                self._tf_ready = True
                self._tf_ready_time = self.get_clock().now()

                # Cancel the polling timer.
                if self._tf_ready_timer is not None:
                    self._tf_ready_timer.cancel()
                    self._tf_ready_timer = None

                self.get_logger().warn(
                    f"TF tree connected via {depth_frame!r} — "
                    f"starting point cloud fusion "
                    f"(waited {elapsed:.1f}s since node startup)"
                )

                # Reset bbox health tracking to avoid 0% success rate
                # from the startup period polluting the health metrics.
                self._bbox_attempts = 0
                self._bbox_successes = 0
                self._bbox_health_window_start = self.get_clock().now()
                return
            else:
                # Get a more specific diagnostic from lookup_transform
                try:
                    self._tf_buffer.lookup_transform(
                        self._target_frame,
                        depth_frame,
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=0.0),
                    )
                    diag_parts.append(
                        f"{depth_frame}: can_transform=False but "
                        f"lookup_transform succeeded (unexpected)"
                    )
                except Exception as exc2:
                    diag_parts.append(f"{depth_frame}: {exc2}")

        # Neither camera's chain is ready yet — log at warn level every
        # ~10 seconds so we can see what's happening without spamming.
        if elapsed < 5.0 or int(elapsed) % 10 == 0:
            self.get_logger().warn(
                f"TF wait gate: not connected after {elapsed:.0f}s — "
                + "; ".join(diag_parts)
            )

    # ── BBox visualization ────────────────────────────────────────────────

    def _publish_bbox_marker(self):
        """Publish a semi-transparent cube per pruning box for RViz visualization."""
        if self._bbox_marker_pub.get_subscription_count() == 0:
            return

        colors = [
            (1.0, 0.3, 0.3, 0.3),  # red for box 0
            (1.0, 0.6, 0.0, 0.3),  # orange for box 1
        ]
        for i, (frame, bbox_min, bbox_max) in enumerate(self._pruning_boxes):
            marker = Marker()
            marker.header.frame_id = frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = f"pruning_box_{i}"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            center = (bbox_min + bbox_max) / 2.0
            scale = bbox_max - bbox_min
            marker.pose.position.x = float(center[0])
            marker.pose.position.y = float(center[1])
            marker.pose.position.z = float(center[2])
            marker.pose.orientation.w = 1.0
            marker.scale.x = float(scale[0])
            marker.scale.y = float(scale[1])
            marker.scale.z = float(scale[2])

            r, g, b, a = colors[i % len(colors)]
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = a

            marker.lifetime.sec = 5  # Expire if node dies

            self._bbox_marker_pub.publish(marker)

    # ── Pruning box loading from camera_mounts.yaml ──────────────────

    @staticmethod
    def _load_pruning_boxes(config_path: str, mount_name: str) -> list:
        """Load pruning boxes from camera_mounts.yaml.

        Returns a list of (frame_id, bbox_min, bbox_max) tuples.
        """
        with open(config_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)

        boxes = []

        # Box 1: bounding_box in palm_frame
        bb = data["bounding_box"]
        corner = bb["palm_to_corner"]["translation"]
        opp_offset = bb["corner_to_opposite"]["translation"]
        c = np.array([corner["x"], corner["y"], corner["z"]], dtype=np.float32)
        o = c + np.array(
            [opp_offset["x"], opp_offset["y"], opp_offset["z"]], dtype=np.float32
        )
        boxes.append(
            (
                "palm_frame",
                np.minimum(c, o),
                np.maximum(c, o),
            )
        )

        # Box 2: cam_bounding_box in screw frame (if present)
        cam_key = f"cam_bounding_box_{mount_name.split('_')[0]}cm"
        # Try exact key first, then fall back to 8cm
        cam_bb = data.get("cam_bounding_box_8cm")
        if cam_bb is None:
            cam_bb = data.get(cam_key)
        if cam_bb is not None:
            c1_dict = cam_bb["screw_to_bbcam1"]["translation"]
            c2_dict = cam_bb["screw_to_bbcam2"]["translation"]
            c1 = np.array([c1_dict["x"], c1_dict["y"], c1_dict["z"]], dtype=np.float32)
            c2 = np.array([c2_dict["x"], c2_dict["y"], c2_dict["z"]], dtype=np.float32)
            screw_frame = f"d435i_arm_bottom_screw_frame_{mount_name}"
            boxes.append(
                (
                    screw_frame,
                    np.minimum(c1, c2),
                    np.maximum(c1, c2),
                )
            )

        return boxes

    # ── Stats logging ─────────────────────────────────────────────────────

    def _log_stats(self):
        now = self.get_clock().now()
        with self._stats_lock:
            stats_snapshot = dict(self._stats)
            stats_snapshot["tf_fail"] = dict(self._stats["tf_fail"])
        since_last = (now - self._last_publish_time).nanoseconds / 1e9
        tf_fail_str = ""
        if stats_snapshot["tf_fail"]:
            tf_fail_str = (
                " tf_fail={"
                + ", ".join(
                    f"{k}:{v}" for k, v in sorted(stats_snapshot["tf_fail"].items())
                )
                + "}"
            )
        ransac_str = ""
        if self._enable_ransac:
            avg_inliers = stats_snapshot["ransac_inliers"] / max(
                stats_snapshot["ransac_used"], 1
            )
            ransac_str = (
                f" ransac_used={stats_snapshot['ransac_used']}"
                f" ransac_skipped={stats_snapshot['ransac_skipped']}"
                f" ransac_avg_inliers={avg_inliers:.0f}"
            )
        registration_str = ""
        if self._enable_quality_registration:
            avg_fitness = stats_snapshot["registration_fitness_sum"] / max(
                stats_snapshot["registration_used"], 1
            )
            avg_rmse = stats_snapshot["registration_rmse_sum"] / max(
                stats_snapshot["registration_used"], 1
            )
            registration_str = (
                f" registration_used={stats_snapshot['registration_used']}"
                f" registration_skipped={stats_snapshot['registration_skipped']}"
                f" global_used={stats_snapshot['global_used']}"
                f" icp_used={stats_snapshot['icp_used']}"
                f" registration_avg_fitness={avg_fitness:.3f}"
                f" registration_avg_rmse={avg_rmse:.3f}"
            )
        processing_avg = stats_snapshot["processing_ms_sum"] / max(
            stats_snapshot["published"] + stats_snapshot["processing_age_drops"], 1
        )
        self.get_logger().info(
            f"Stats: published={stats_snapshot['published']} "
            f"(dual={stats_snapshot['dual']}, cam1_only={stats_snapshot['cam1_only']}) "
            f"dist_removed={stats_snapshot['distance_removed']} "
            f"bbox_removed={stats_snapshot['bbox_removed']}"
            f" bbox_skipped={stats_snapshot['bbox_skipped']}"
            f" bbox_cache_hits={stats_snapshot['bbox_cache_hits']}"
            f" busy_skips={stats_snapshot['processing_busy_skips']}"
            f" age_drops={stats_snapshot['processing_age_drops']}"
            f" proc_avg_ms={processing_avg:.1f}"
            f" proc_max_ms={stats_snapshot['processing_ms_max']:.1f}"
            f"{registration_str}"
            f"{ransac_str}"
            f"{tf_fail_str}"
            f" last_publish_ago={since_last:.1f}s"
        )

        # Stall diagnostic: if nothing published this interval, explain why.
        if stats_snapshot["published"] == 0:
            with self._lock:
                c1, s1 = self._cam1_cloud, self._cam1_stamp
                c2, s2 = self._cam2_cloud, self._cam2_stamp

            c1_age = (now - s1).nanoseconds / 1e9 if s1 else None
            c2_age = (now - s2).nanoseconds / 1e9 if s2 else None

            parts = []
            if c1 is None or c1_age is None or c1_age > self._cloud_max_age:
                parts.append(
                    f"cam1: no cloud"
                    if c1 is None or c1_age is None
                    else f"cam1: stale ({c1_age:.1f}s)"
                )
            else:
                parts.append(f"cam1: fresh ({c1_age:.2f}s)")
                # Cloud is fresh but TF failed — diagnose which chain link is missing
                try:
                    self._tf_buffer.lookup_transform(
                        self._target_frame, c1.header.frame_id, rclpy.time.Time()
                    )
                except Exception as e:
                    parts.append(f"cam1 TF: {e}")

            if c2 is None or c2_age is None or c2_age > self._cloud_max_age:
                parts.append(
                    f"cam2: no cloud"
                    if c2 is None or c2_age is None
                    else f"cam2: stale ({c2_age:.1f}s)"
                )
            else:
                parts.append(f"cam2: fresh ({c2_age:.2f}s)")
                try:
                    self._tf_buffer.lookup_transform(
                        self._target_frame, c2.header.frame_id, rclpy.time.Time()
                    )
                except Exception as e:
                    parts.append(f"cam2 TF: {e}")

            self.get_logger().warn(
                f"Stall diagnostic (no publish for {since_last:.0f}s): {'; '.join(parts)}"
            )

        # Reset per-interval counters
        with self._stats_lock:
            self._stats = {
                "published": 0,
                "cam1_only": 0,
                "dual": 0,
                "distance_removed": 0,
                "bbox_removed": 0,
                "bbox_skipped": 0,
                "bbox_cache_hits": 0,
                "processing_busy_skips": 0,
                "processing_age_drops": 0,
                "processing_ms_sum": 0.0,
                "processing_ms_max": 0.0,
                "ransac_used": 0,
                "ransac_skipped": 0,
                "ransac_inliers": 0,
                "registration_used": 0,
                "registration_skipped": 0,
                "global_used": 0,
                "icp_used": 0,
                "registration_fitness_sum": 0.0,
                "registration_rmse_sum": 0.0,
                "tf_fail": {},
            }


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudFusionNode()
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
