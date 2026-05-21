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
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker
import tf2_ros
import tf2_sensor_msgs  # noqa: F401 — registers do_transform_cloud

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
        return np.frombuffer(raw[:, off:off + 4].copy().tobytes(), dtype=np.float32)

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
        PointField(name="x",   offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name="y",   offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name="z",   offset=8,  datatype=PointField.FLOAT32, count=1),
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

def _distance_filter(xyz: np.ndarray, center: np.ndarray, max_dist: float) -> np.ndarray:
    """Return boolean mask: True for points within max_dist of center."""
    diff = xyz - center
    dists_sq = np.sum(diff * diff, axis=1)
    return dists_sq <= (max_dist * max_dist)


def _bbox_filter(xyz_arm: np.ndarray, bbox_min: np.ndarray, bbox_max: np.ndarray) -> np.ndarray:
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
        voxel_idx, axis=0, return_index=True, return_inverse=True,
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


def _transform_points_to_frame(xyz: np.ndarray, tf_buffer, target_frame: str,
                                source_frame: str, stamp) -> np.ndarray | None:
    """Transform an (N,3) xyz array from source_frame to target_frame using TF2.

    Returns transformed (N,3) float32 or None if TF lookup fails.
    """
    try:
        t = tf_buffer.lookup_transform(target_frame, source_frame, stamp)
    except Exception:
        return None

    tx = t.transform.translation.x
    ty = t.transform.translation.y
    tz = t.transform.translation.z
    qx = t.transform.rotation.x
    qy = t.transform.rotation.y
    qz = t.transform.rotation.z
    qw = t.transform.rotation.w

    # Quaternion to rotation matrix
    r00 = 1 - 2*(qy*qy + qz*qz)
    r01 = 2*(qx*qy - qz*qw)
    r02 = 2*(qx*qz + qy*qw)
    r10 = 2*(qx*qy + qz*qw)
    r11 = 1 - 2*(qx*qx + qz*qz)
    r12 = 2*(qy*qz - qx*qw)
    r20 = 2*(qx*qz - qy*qw)
    r21 = 2*(qy*qz + qx*qw)
    r22 = 1 - 2*(qx*qx + qy*qy)

    R = np.array([[r00, r01, r02],
                   [r10, r11, r12],
                   [r20, r21, r22]], dtype=np.float64)
    t_vec = np.array([tx, ty, tz], dtype=np.float64)

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

        # ── Read parameters ───────────────────────────────────────────────
        self._target_frame = self.get_parameter("target_frame").value
        self._arm_frame = self.get_parameter("arm_frame").value
        self._max_distance = self.get_parameter("max_distance").value
        self._voxel_size = self.get_parameter("voxel_size").value
        self._bbox_min = np.array(self.get_parameter("bbox_min").value, dtype=np.float32)
        self._bbox_max = np.array(self.get_parameter("bbox_max").value, dtype=np.float32)
        self._enable_downsampling = self.get_parameter("enable_downsampling").value
        self._enable_distance_filter = self.get_parameter("enable_distance_filter").value
        self._enable_hand_removal = self.get_parameter("enable_hand_removal").value
        cam1_topic = self.get_parameter("cam1_topic").value
        cam2_topic = self.get_parameter("cam2_topic").value
        output_topic = self.get_parameter("output_topic").value
        sync_tol = self.get_parameter("sync_tolerance_s").value
        self._require_both = self.get_parameter("require_both_cameras").value
        fallback_rate = max(float(self.get_parameter("fallback_merge_rate_hz").value), 1.0)
        self._cloud_max_age = float(self.get_parameter("cloud_max_age_s").value)

        # ── Load pruning boxes from camera_mounts.yaml or fall back to params ─
        mounts_config = self.get_parameter("mounts_config_path").value
        if mounts_config:
            self._pruning_boxes = self._load_pruning_boxes(
                mounts_config, self.get_parameter("active_mount").value)
            for i, (frame, bmin, bmax) in enumerate(self._pruning_boxes):
                self.get_logger().info(
                    f"Pruning box {i}: frame={frame}, "
                    f"min={bmin.tolist()}, max={bmax.tolist()}")
        else:
            self._pruning_boxes = [
                (self._arm_frame, self._bbox_min, self._bbox_max)]
            self.get_logger().info(
                f"Pruning box 0 (fallback): frame={self._arm_frame}, "
                f"min={self._bbox_min.tolist()}, max={self._bbox_max.tolist()}")

        # ── TF2 ───────────────────────────────────────────────────────────
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── Publishers ────────────────────────────────────────────────────
        self._pub = self.create_publisher(PointCloud2, output_topic, 5)
        self._bbox_marker_pub = self.create_publisher(
            Marker, "/pointcloud_fusion/hand_removal_bbox", 1)

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
        self._lock = threading.Lock()
        self.create_subscription(
            PointCloud2, cam1_topic, self._cb_cam1, cloud_qos)
        self.create_subscription(
            PointCloud2, cam2_topic, self._cb_cam2, cloud_qos)

        if _HAS_MSG_FILTERS and self._require_both:
            # Synchronizer-only mode: require both clouds to arrive together.
            sub1 = message_filters.Subscriber(self, PointCloud2, cam1_topic, qos_profile=cloud_qos)
            sub2 = message_filters.Subscriber(self, PointCloud2, cam2_topic, qos_profile=cloud_qos)
            self._sync = ApproximateTimeSynchronizer(
                [sub1, sub2], queue_size=5, slop=sync_tol)
            self._sync.registerCallback(self._synced_callback)
            self._fallback_timer = None
            self.get_logger().info(
                f"Using ApproximateTimeSynchronizer (tolerance={sync_tol}s, require_both=True)")
        else:
            # Fallback timer merge: process whichever clouds are fresh.
            self._sync = None
            self._fallback_timer = self.create_timer(
                1.0 / fallback_rate, self._timer_merge)
            mode_desc = "require_both=True but message_filters unavailable" \
                if self._require_both else "require_both=False"
            self.get_logger().info(
                f"Using timer-based merge ({mode_desc}, "
                f"rate={fallback_rate}Hz, max_age={self._cloud_max_age}s)")

        # ── BBox visualization timer ──────────────────────────────────────
        self.create_timer(1.0, self._publish_bbox_marker)

        # ── Stats ─────────────────────────────────────────────────────────
        self._stats_lock = threading.Lock()
        self._stats = {"published": 0, "cam1_only": 0, "dual": 0,
                       "distance_removed": 0, "bbox_removed": 0,
                       "tf_fail": {}}
        self._last_publish_time = self.get_clock().now()
        self.create_timer(10.0, self._log_stats)

        self.get_logger().info(
            f"Pointcloud fusion: {cam1_topic} + {cam2_topic} -> {output_topic} "
            f"(target_frame={self._target_frame}, arm_frame={self._arm_frame}, "
            f"max_dist={self._max_distance}m, voxel={self._voxel_size}m, "
            f"pruning_boxes={len(self._pruning_boxes)})"
        )

    # ── Synced callback (message_filters) ────────────────────────────────

    def _synced_callback(self, msg1: PointCloud2, msg2: PointCloud2):
        """Called when both clouds arrive within sync tolerance."""
        self._process_clouds([msg1, msg2])

    # ── Fallback individual callbacks ────────────────────────────────────

    def _cb_cam1(self, msg: PointCloud2):
        with self._lock:
            self._cam1_cloud = msg
            self._cam1_stamp = self.get_clock().now()

    def _cb_cam2(self, msg: PointCloud2):
        with self._lock:
            self._cam2_cloud = msg
            self._cam2_stamp = self.get_clock().now()

    def _timer_merge(self):
        """Process whichever clouds are fresh enough.

        Runs processing in a background thread so the executor can continue
        receiving cloud callbacks and TF updates without starvation.
        """
        now = self.get_clock().now()
        with self._lock:
            c1, s1 = self._cam1_cloud, self._cam1_stamp
            c2, s2 = self._cam2_cloud, self._cam2_stamp

        clouds = []
        if c1 is not None and s1 is not None:
            age = (now - s1).nanoseconds / 1e9
            if age <= self._cloud_max_age:
                clouds.append(c1)
        if c2 is not None and s2 is not None:
            age = (now - s2).nanoseconds / 1e9
            if age <= self._cloud_max_age:
                clouds.append(c2)
        if clouds:
            # Process in a daemon thread so the executor is not blocked.
            t = threading.Thread(target=self._process_clouds, args=(clouds,), daemon=True)
            t.start()

    # ── Core processing pipeline ─────────────────────────────────────────

    def _process_clouds(self, clouds: list[PointCloud2]):
        """Transform, merge, filter, downsample, and publish."""
        # ── Step 1: Transform all clouds to target frame ──────────────
        transformed: list[PointCloud2] = []
        for cloud in clouds:
            if cloud.header.frame_id == self._target_frame:
                transformed.append(cloud)
                continue
            try:
                t = self._tf_buffer.lookup_transform(
                    self._target_frame, cloud.header.frame_id,
                    rclpy.time.Time(),
                )
                transformed.append(tf2_sensor_msgs.do_transform_cloud(cloud, t))
            except Exception as exc:
                frame = cloud.header.frame_id
                with self._stats_lock:
                    self._stats["tf_fail"][frame] = self._stats["tf_fail"].get(frame, 0) + 1
                self.get_logger().warn(
                    f"TF transform failed for {frame}: {exc}",
                    throttle_duration_sec=10.0,
                )

        if not transformed:
            return

        # ── Step 2: Concatenate clouds ────────────────────────────────
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
            for frame, bbox_min, bbox_max in self._pruning_boxes:
                xyz_box = _transform_points_to_frame(
                    xyz_all, self._tf_buffer, frame,
                    self._target_frame, rclpy.time.Time(),
                )
                if xyz_box is not None:
                    keep = _bbox_filter(xyz_box, bbox_min, bbox_max)
                    removed = len(xyz_all) - np.sum(keep)
                    xyz_all = xyz_all[keep]
                    rgb_all = rgb_all[keep]
                    with self._stats_lock:
                        self._stats["bbox_removed"] += int(removed)
                else:
                    self.get_logger().warn(
                        f"Cannot transform to {frame} for bbox removal — skipping",
                        throttle_duration_sec=5.0,
                    )

        if len(xyz_all) == 0:
            return

        # ── Step 5: Voxel downsampling ────────────────────────────────
        if self._enable_downsampling and self._voxel_size > 0.0:
            xyz_all, rgb_all = _voxel_downsample(xyz_all, rgb_all, self._voxel_size)

        if len(xyz_all) == 0:
            return

        # ── Step 6: Build and publish ─────────────────────────────────
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

    # ── TF helpers ────────────────────────────────────────────────────────

    def _get_frame_origin_in_target(self, frame: str) -> np.ndarray | None:
        """Get the origin of `frame` in target_frame. Returns (3,) float32 or None."""
        try:
            t = self._tf_buffer.lookup_transform(
                self._target_frame, frame, rclpy.time.Time())
            return np.array([
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
            ], dtype=np.float32)
        except Exception:
            return None

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
        o = c + np.array([opp_offset["x"], opp_offset["y"], opp_offset["z"]],
                         dtype=np.float32)
        boxes.append((
            "palm_frame",
            np.minimum(c, o),
            np.maximum(c, o),
        ))

        # Box 2: cam_bounding_box in screw frame (if present)
        cam_key = f"cam_bounding_box_{mount_name.split('_')[0]}cm"
        # Try exact key first, then fall back to 8cm
        cam_bb = data.get("cam_bounding_box_8cm")
        if cam_bb is None:
            cam_bb = data.get(cam_key)
        if cam_bb is not None:
            c1_dict = cam_bb["screw_to_bbcam1"]["translation"]
            c2_dict = cam_bb["screw_to_bbcam2"]["translation"]
            c1 = np.array([c1_dict["x"], c1_dict["y"], c1_dict["z"]],
                          dtype=np.float32)
            c2 = np.array([c2_dict["x"], c2_dict["y"], c2_dict["z"]],
                          dtype=np.float32)
            screw_frame = f"d435i_arm_bottom_screw_frame_{mount_name}"
            boxes.append((
                screw_frame,
                np.minimum(c1, c2),
                np.maximum(c1, c2),
            ))

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
            tf_fail_str = " tf_fail={" + ", ".join(
                f"{k}:{v}" for k, v in sorted(stats_snapshot["tf_fail"].items())
            ) + "}"
        self.get_logger().info(
            f"Stats: published={stats_snapshot['published']} "
            f"(dual={stats_snapshot['dual']}, cam1_only={stats_snapshot['cam1_only']}) "
            f"dist_removed={stats_snapshot['distance_removed']} "
            f"bbox_removed={stats_snapshot['bbox_removed']}"
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
                parts.append(f"cam1: no cloud"
                             if c1 is None or c1_age is None
                             else f"cam1: stale ({c1_age:.1f}s)")
            else:
                parts.append(f"cam1: fresh ({c1_age:.2f}s)")
                # Cloud is fresh but TF failed — diagnose which chain link is missing
                try:
                    self._tf_buffer.lookup_transform(
                        self._target_frame, c1.header.frame_id, rclpy.time.Time())
                except Exception as e:
                    parts.append(f"cam1 TF: {e}")

            if c2 is None or c2_age is None or c2_age > self._cloud_max_age:
                parts.append(f"cam2: no cloud"
                             if c2 is None or c2_age is None
                             else f"cam2: stale ({c2_age:.1f}s)")
            else:
                parts.append(f"cam2: fresh ({c2_age:.2f}s)")
                try:
                    self._tf_buffer.lookup_transform(
                        self._target_frame, c2.header.frame_id, rclpy.time.Time())
                except Exception as e:
                    parts.append(f"cam2 TF: {e}")

            self.get_logger().warn(
                f"Stall diagnostic (no publish for {since_last:.0f}s): {'; '.join(parts)}"
            )

        # Reset per-interval counters
        with self._stats_lock:
            self._stats = {"published": 0, "cam1_only": 0, "dual": 0,
                           "distance_removed": 0, "bbox_removed": 0,
                           "tf_fail": {}}


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
