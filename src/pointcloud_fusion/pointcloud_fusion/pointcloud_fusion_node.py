#!/usr/bin/env python3
"""Point cloud fusion node — transforms dual-camera clouds to world frame, merges,
filters, downsamples, and publishes a unified cloud.

Pipeline:
  1. Subscribe to both camera point clouds (approximate time sync)
  2. Transform both to world frame via TF2
  3. Concatenate into a single cloud
  4. Distance filter — remove points >max_distance from arm_frame origin
  5. Hand/arm bbox removal — AABB crop in arm_frame
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

import numpy as np
import rclpy
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
    rgb_packed:  (N,) uint32
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
    summed_rgb = np.zeros(n_voxels, dtype=np.uint64)
    counts = np.zeros(n_voxels, dtype=np.int32)

    np.add.at(summed_xyz, inverse, xyz.astype(np.float64))
    np.add.at(summed_rgb, inverse, rgb_packed.astype(np.uint64))
    np.add.at(counts, inverse, 1)

    xyz_out = (summed_xyz / counts[:, None]).astype(np.float32)
    rgb_out = (summed_rgb / counts.astype(np.uint64)).astype(np.uint32)
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
        self.declare_parameter("target_frame", "world")
        self.declare_parameter("cam1_topic", "/head/d435i_head/depth/color/points")
        self.declare_parameter("cam2_topic", "/arm/d435i_arm/depth/color/points")
        self.declare_parameter("arm_frame", "wrist_link")
        self.declare_parameter("max_distance", 2.0)
        self.declare_parameter("voxel_size", 0.005)
        self.declare_parameter("bbox_min", [-0.30, -0.10, -0.10])
        self.declare_parameter("bbox_max", [0.22, 0.10, 0.12])
        self.declare_parameter("enable_downsampling", True)
        self.declare_parameter("enable_distance_filter", True)
        self.declare_parameter("enable_hand_removal", True)
        self.declare_parameter("output_topic", "/fused_pointcloud")
        self.declare_parameter("sync_tolerance_s", 0.1)

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

        # ── TF2 ───────────────────────────────────────────────────────────
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── Publishers ────────────────────────────────────────────────────
        self._pub = self.create_publisher(PointCloud2, output_topic, 5)
        self._bbox_marker_pub = self.create_publisher(
            Marker, "/pointcloud_fusion/hand_removal_bbox", 1)

        # ── Subscriptions ─────────────────────────────────────────────────
        # Use BEST_EFFORT QoS — RealSense publishers use BEST_EFFORT
        cloud_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        if _HAS_MSG_FILTERS:
            sub1 = message_filters.Subscriber(self, PointCloud2, cam1_topic, qos_profile=cloud_qos)
            sub2 = message_filters.Subscriber(self, PointCloud2, cam2_topic, qos_profile=cloud_qos)
            self._sync = ApproximateTimeSynchronizer(
                [sub1, sub2], queue_size=5, slop=sync_tol)
            self._sync.registerCallback(self._synced_callback)
            self.get_logger().info(
                f"Using ApproximateTimeSynchronizer (tolerance={sync_tol}s)")
        else:
            # Fallback: subscribe individually, process latest of each
            self._cam1_cloud = None
            self._cam2_cloud = None
            self._lock = threading.Lock()
            self.create_subscription(
                PointCloud2, cam1_topic, self._cb_cam1, cloud_qos)
            self.create_subscription(
                PointCloud2, cam2_topic, self._cb_cam2, cloud_qos)
            self.create_timer(1.0 / 15.0, self._timer_merge)
            self._sync = None
            self.get_logger().info(
                "message_filters unavailable — using timer-based merge fallback")

        # ── BBox visualization timer ──────────────────────────────────────
        self.create_timer(1.0, self._publish_bbox_marker)

        # ── Stats ─────────────────────────────────────────────────────────
        self._stats = {"published": 0, "cam1_only": 0, "dual": 0,
                       "distance_removed": 0, "bbox_removed": 0}
        self.create_timer(10.0, self._log_stats)

        self.get_logger().info(
            f"Pointcloud fusion: {cam1_topic} + {cam2_topic} -> {output_topic} "
            f"(target_frame={self._target_frame}, arm_frame={self._arm_frame}, "
            f"max_dist={self._max_distance}m, voxel={self._voxel_size}m)"
        )

    # ── Synced callback (message_filters) ────────────────────────────────

    def _synced_callback(self, msg1: PointCloud2, msg2: PointCloud2):
        """Called when both clouds arrive within sync tolerance."""
        self._process_clouds([msg1, msg2])

    # ── Fallback individual callbacks ────────────────────────────────────

    def _cb_cam1(self, msg: PointCloud2):
        with self._lock:
            self._cam1_cloud = msg

    def _cb_cam2(self, msg: PointCloud2):
        with self._lock:
            self._cam2_cloud = msg

    def _timer_merge(self):
        with self._lock:
            c1 = self._cam1_cloud
            c2 = self._cam2_cloud
        clouds = []
        if c1 is not None:
            clouds.append(c1)
        if c2 is not None:
            clouds.append(c2)
        if clouds:
            self._process_clouds(clouds)

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
                if self._tf_buffer.can_transform(
                    self._target_frame, cloud.header.frame_id,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.05),
                ):
                    t = self._tf_buffer.lookup_transform(
                        self._target_frame, cloud.header.frame_id,
                        rclpy.time.Time(),
                    )
                    transformed.append(tf2_sensor_msgs.do_transform_cloud(cloud, t))
                else:
                    self.get_logger().warn(
                        f"TF not available: {cloud.header.frame_id} -> {self._target_frame}",
                        throttle_duration_sec=5.0,
                    )
            except Exception as exc:
                self.get_logger().warn(
                    f"TF transform failed for {cloud.header.frame_id}: {exc}",
                    throttle_duration_sec=5.0,
                )

        if not transformed:
            return

        # ── Step 2: Concatenate clouds ────────────────────────────────
        if len(transformed) == 1:
            xyz_all, rgb_all = _parse_cloud(transformed[0])
            stamp = transformed[0].header.stamp
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
                self._stats["distance_removed"] += int(removed)
            else:
                self.get_logger().warn(
                    f"Cannot look up {self._arm_frame} in {self._target_frame} "
                    f"for distance filter — skipping",
                    throttle_duration_sec=5.0,
                )

        if len(xyz_all) == 0:
            return

        # ── Step 4: Hand/arm bbox removal ─────────────────────────────
        if self._enable_hand_removal:
            xyz_arm = _transform_points_to_frame(
                xyz_all, self._tf_buffer, self._arm_frame,
                self._target_frame, rclpy.time.Time(),
            )
            if xyz_arm is not None:
                keep = _bbox_filter(xyz_arm, self._bbox_min, self._bbox_max)
                removed = len(xyz_all) - np.sum(keep)
                xyz_all = xyz_all[keep]
                rgb_all = rgb_all[keep]
                self._stats["bbox_removed"] += int(removed)
            else:
                self.get_logger().warn(
                    f"Cannot transform to {self._arm_frame} for bbox removal — skipping",
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

        out_msg = _build_cloud(xyz_all, rgb_all, header)
        self._pub.publish(out_msg)
        self._stats["published"] += 1

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
        """Publish the hand removal bbox as a semi-transparent cube in arm_frame."""
        if self._bbox_marker_pub.get_subscription_count() == 0:
            return

        marker = Marker()
        marker.header.frame_id = self._arm_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "hand_removal_bbox"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # Center and scale from bbox_min/max
        center = (self._bbox_min + self._bbox_max) / 2.0
        scale = self._bbox_max - self._bbox_min
        marker.pose.position.x = float(center[0])
        marker.pose.position.y = float(center[1])
        marker.pose.position.z = float(center[2])
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(scale[0])
        marker.scale.y = float(scale[1])
        marker.scale.z = float(scale[2])

        # Semi-transparent red
        marker.color.r = 1.0
        marker.color.g = 0.3
        marker.color.b = 0.3
        marker.color.a = 0.3

        marker.lifetime.sec = 2  # Expire if node dies

        self._bbox_marker_pub.publish(marker)

    # ── Stats logging ─────────────────────────────────────────────────────

    def _log_stats(self):
        self.get_logger().info(
            f"Stats: published={self._stats['published']} "
            f"(dual={self._stats['dual']}, cam1_only={self._stats['cam1_only']}) "
            f"dist_removed={self._stats['distance_removed']} "
            f"bbox_removed={self._stats['bbox_removed']}"
        )


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
