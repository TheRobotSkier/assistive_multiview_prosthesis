#!/usr/bin/env python3
"""Bridge OpenVINS camera frames to RealSense frame trees.

The Jetson camera stack publishes two useful but disconnected TF trees:

    marker_map -> head_imu -> head_cam0
    head_d435i_head_link -> head_d435i_head_depth_frame -> ...

PointCloud2 messages use the RealSense depth optical frames, while OpenVINS
owns the marker_map tree.  This node publishes the missing parent transform:

    head_cam0 -> head_d435i_head_link
    arm_cam0  -> arm_d435i_arm_link

**Key insight**: OpenVINS uses the RealSense color camera as its tracking
camera (cam0).  This means ``head_cam0`` and ``head_d435i_head_color_optical_frame``
are the **same physical camera**.  The bridge edge ``cam0 -> link`` is therefore
just the known RealSense extrinsic ``T(color_optical -> link)``, which is a
static property of the D435i hardware.

The node publishes this extrinsic directly from the RealSense static chain
(no aruco marker detection needed).  It also broadcasts the full nominal
D435/D435i static fan-out via StaticTransformBroadcaster once at startup.

A slow liveness timer (~0.2 Hz) re-sends only the bridge-edge transforms on
/tf as a workaround for CycloneDDS /tf_static latch unreliability with
late-joining nodes.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformBroadcaster, TransformListener


def _quaternion_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def _matrix_to_quaternion(r: np.ndarray) -> tuple[float, float, float, float]:
    trace = float(np.trace(r))

    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (r[2, 1] - r[1, 2]) / s
        qy = (r[0, 2] - r[2, 0]) / s
        qz = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        qw = (r[2, 1] - r[1, 2]) / s
        qx = 0.25 * s
        qy = (r[0, 1] + r[1, 0]) / s
        qz = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        qw = (r[0, 2] - r[2, 0]) / s
        qx = (r[0, 1] + r[1, 0]) / s
        qy = 0.25 * s
        qz = (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        qw = (r[1, 0] - r[0, 1]) / s
        qx = (r[0, 2] + r[2, 0]) / s
        qy = (r[1, 2] + r[2, 1]) / s
        qz = 0.25 * s

    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 0.0:
        return 0.0, 0.0, 0.0, 1.0
    return qx / norm, qy / norm, qz / norm, qw / norm


def _transform_to_matrix(transform: TransformStamped) -> np.ndarray:
    t = transform.transform.translation
    q = transform.transform.rotation
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = _quaternion_to_matrix(q.x, q.y, q.z, q.w)
    out[:3, 3] = np.array([t.x, t.y, t.z], dtype=np.float64)
    return out


def _matrix_to_transform(
    matrix: np.ndarray,
    parent_frame: str,
    child_frame: str,
    stamp,
) -> TransformStamped:
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent_frame
    msg.child_frame_id = child_frame

    msg.transform.translation.x = float(matrix[0, 3])
    msg.transform.translation.y = float(matrix[1, 3])
    msg.transform.translation.z = float(matrix[2, 3])

    qx, qy, qz, qw = _matrix_to_quaternion(matrix[:3, :3])
    msg.transform.rotation.x = float(qx)
    msg.transform.rotation.y = float(qy)
    msg.transform.rotation.z = float(qz)
    msg.transform.rotation.w = float(qw)
    return msg


def _make_transform_from_xyz_rpy(
    x: float, y: float, z: float,
    roll: float, pitch: float, yaw: float,
    parent_frame: str, child_frame: str, stamp,
) -> TransformStamped:
    """Build a TransformStamped from xyz and rpy (URDF convention: Rz*Ry*Rx)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    # R = Rz(yaw) * Ry(pitch) * Rx(roll)
    R = np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )

    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = np.array([x, y, z], dtype=np.float64)

    return _matrix_to_transform(T, parent_frame, child_frame, stamp)


# Nominal D435/D435i extrinsics from realsense2_description URDF.
# These are approximate (factory calibration overrides them on the device).
# Format: (parent_suffix, child_suffix, x, y, z, roll, pitch, yaw)
# All optical children share the same rpy(-pi/2, 0, -pi/2).
_OPTICAL_RPY = (-math.pi / 2.0, 0.0, -math.pi / 2.0)
_NOMINAL_STATIC_EDGES: list[tuple[str, str, float, float, float, float, float, float]] = [
    # link -> sensor frames
    ("link", "depth_frame", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    ("link", "color_frame", 0.0, 0.015, 0.0, 0.0, 0.0, 0.0),
    ("link", "accel_frame", -0.01174, -0.00552, 0.0051, 0.0, 0.0, 0.0),
    ("link", "gyro_frame", -0.01174, -0.00552, 0.0051, 0.0, 0.0, 0.0),
    ("link", "imu_frame", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    # sensor frames -> optical frames
    ("depth_frame", "depth_optical_frame", 0.0, 0.0, 0.0, *_OPTICAL_RPY),
    ("color_frame", "color_optical_frame", 0.0, 0.0, 0.0, *_OPTICAL_RPY),
    ("accel_frame", "accel_optical_frame", 0.0, 0.0, 0.0, *_OPTICAL_RPY),
    ("gyro_frame", "gyro_optical_frame", 0.0, 0.0, 0.0, *_OPTICAL_RPY),
    ("imu_frame", "imu_optical_frame", 0.0, 0.0, 0.0, *_OPTICAL_RPY),
]


@dataclass(frozen=True)
class BridgeSpec:
    name: str
    parent_frame: str
    link_frame: str
    anchor_frame: str
    anchor_frame_mode: str
    fallback_optical_frame: str

    @property
    def color_optical_frame(self) -> str | None:
        """Return the RealSense color_optical_frame.

        If the anchor frame IS the color_optical_frame (e.g.
        ``head_d435i_head_color_optical_frame``), return it directly.
        Otherwise, strip known suffixes (``_from_marker``, ``_body_display``).
        """
        if not self.anchor_frame:
            return None
        # Direct match: anchor is already the color_optical_frame.
        if "color_optical_frame" in self.anchor_frame and "_" not in self.anchor_frame.split("color_optical_frame", 1)[1]:
            return self.anchor_frame
        # Legacy: strip suffix.
        for suffix in ("_from_marker", "_body_display"):
            if self.anchor_frame.endswith(suffix):
                return self.anchor_frame[: -len(suffix)]
        return None


class OpenVinsRealSenseTfBridge(Node):
    """Publishes the missing OpenVINS camera -> RealSense link transforms
    and the nominal RealSense static fan-out."""

    def __init__(self):
        super().__init__("openvins_realsense_tf_bridge")

        self.declare_parameter("camera_names", ["head", "arm"])
        self.declare_parameter("parent_frames", ["head_cam0", "arm_cam0"])
        self.declare_parameter(
            "link_frames", ["head_d435i_head_link", "arm_d435i_arm_link"]
        )
        self.declare_parameter(
            "anchor_frames",
            [
                "head_d435i_head_color_optical_frame_from_marker",
                "arm_d435i_arm_color_optical_frame_from_marker",
            ],
        )
        self.declare_parameter("anchor_frame_modes", ["link", "link"])
        self.declare_parameter(
            "fallback_optical_frames",
            [
                "head_d435i_head_depth_optical_frame",
                "arm_d435i_arm_depth_optical_frame",
            ],
        )
        self.declare_parameter("publish_rate_hz", 15.0)
        self.declare_parameter("publish_nominal_static_chain", True)

        self._publish_nominal_static = bool(
            self.get_parameter("publish_nominal_static_chain").value
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._specs = self._load_specs()
        self._warned: set[str] = set()
        self._published_from: dict[str, str] = {}
        self._bridge_tfs: list[TransformStamped] = []  # resolved bridge-edge TFs

        # Pre-build the nominal static chain so we can send it efficiently.
        self._nominal_static_tfs: list[TransformStamped] = []
        if self._publish_nominal_static:
            self._build_and_send_nominal_static_chain()

        # Phase 1: fast startup timer to resolve bridge extrinsics.
        rate = max(float(self.get_parameter("publish_rate_hz").value), 1.0)
        self._startup_timer = self.create_timer(1.0 / rate, self._startup_tick)

        # Phase 2: slow liveness timer (created after startup completes).
        self._liveness_timer = None

        # Periodic diagnostic for unresolved bridge transforms.
        self._resolve_fail_counts: dict[str, int] = {}
        self.create_timer(10.0, self._log_diagnostics)

        summary = ", ".join(
            f"{spec.parent_frame}->{spec.link_frame}" for spec in self._specs
        )
        self.get_logger().info(
            f"OpenVINS/RealSense TF bridge active: {summary}. "
            f"Static chain publishing={'ON' if self._publish_nominal_static else 'OFF'}"
        )

    def _load_specs(self) -> list[BridgeSpec]:
        names = list(self.get_parameter("camera_names").value)
        parents = list(self.get_parameter("parent_frames").value)
        links = list(self.get_parameter("link_frames").value)
        anchors = list(self.get_parameter("anchor_frames").value)
        modes = list(self.get_parameter("anchor_frame_modes").value)
        fallbacks = list(self.get_parameter("fallback_optical_frames").value)

        lengths = {
            len(names),
            len(parents),
            len(links),
            len(anchors),
            len(modes),
            len(fallbacks),
        }
        if len(lengths) != 1:
            raise ValueError(
                "camera_names, parent_frames, link_frames, anchor_frames, "
                "anchor_frame_modes, and fallback_optical_frames must have equal length"
            )

        specs = []
        for name, parent, link, anchor, mode, fallback in zip(
            names, parents, links, anchors, modes, fallbacks
        ):
            mode = str(mode).lower()
            if mode not in ("link", "optical"):
                raise ValueError(
                    f"anchor_frame_mode for {name!r} must be 'link' or 'optical'"
                )
            specs.append(
                BridgeSpec(
                    name=str(name),
                    parent_frame=str(parent),
                    link_frame=str(link),
                    anchor_frame=str(anchor),
                    anchor_frame_mode=mode,
                    fallback_optical_frame=str(fallback),
                )
            )
        return specs

    def _build_and_send_nominal_static_chain(self):
        """Publish the nominal D435/D435i static fan-out once via /tf_static."""
        stamp = self.get_clock().now().to_msg()
        for spec in self._specs:
            prefix = spec.link_frame
            if not prefix.endswith("_link"):
                self.get_logger().warn(
                    f"link_frame {spec.link_frame!r} does not end with '_link'; "
                    "skipping nominal static chain for this camera"
                )
                continue
            base = prefix[:-5]  # strip trailing "_link"
            for parent_suffix, child_suffix, x, y, z, roll, pitch, yaw in _NOMINAL_STATIC_EDGES:
                parent = f"{base}_{parent_suffix}"
                child = f"{base}_{child_suffix}"
                tf_msg = _make_transform_from_xyz_rpy(
                    x, y, z, roll, pitch, yaw,
                    parent, child, stamp,
                )
                self._nominal_static_tfs.append(tf_msg)

        if self._nominal_static_tfs:
            self._static_tf_broadcaster.sendTransform(self._nominal_static_tfs)
            self.get_logger().info(
                f"Published nominal static chain: {len(self._nominal_static_tfs)} transforms "
                f"({len(self._nominal_static_tfs) // len(self._specs)} per camera)"
            )

    def _lookup_matrix(self, target_frame: str, source_frame: str) -> np.ndarray | None:
        """Non-blocking TF lookup — returns 4x4 matrix or None."""
        try:
            tf_msg = self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
            )
            return _transform_to_matrix(tf_msg)
        except Exception:
            return None

    def _startup_tick(self):
        """Phase 1: resolve bridge extrinsics at high rate, then switch to liveness."""
        stamp = self.get_clock().now().to_msg()
        all_resolved = True

        for spec in self._specs:
            matrix, source = self._resolve_bridge_transform(spec)
            if matrix is None:
                all_resolved = False
                self._warn_once(
                    spec.name,
                    f"waiting for color_optical->link extrinsic "
                    f"for {spec.parent_frame}->{spec.link_frame}",
                )
                continue

            tf_msg = _matrix_to_transform(
                matrix,
                parent_frame=spec.parent_frame,
                child_frame=spec.link_frame,
                stamp=stamp,
            )

            # Publish on /tf_static once per spec (idempotent for static transforms).
            self._static_tf_broadcaster.sendTransform(tf_msg)
            # Also on /tf during startup for late joiners.
            self._tf_broadcaster.sendTransform(tf_msg)

            # Track resolved transforms for the liveness phase.
            already = any(
                t.header.frame_id == tf_msg.header.frame_id
                and t.child_frame_id == tf_msg.child_frame_id
                for t in self._bridge_tfs
            )
            if not already:
                self._bridge_tfs.append(tf_msg)

            if self._published_from.get(spec.name) != source:
                self._published_from[spec.name] = source
                self.get_logger().info(
                    f"{spec.name}: publishing {spec.parent_frame}->{spec.link_frame} "
                    f"from {source}"
                )

        if all_resolved and self._startup_timer is not None:
            self.get_logger().info(
                f"All {len(self._specs)} bridge transform(s) resolved — "
                "switching to liveness mode (0.2 Hz)"
            )
            self._startup_timer.cancel()
            self._startup_timer = None
            self._liveness_timer = self.create_timer(5.0, self._liveness_tick)

    def _liveness_tick(self):
        """Phase 2: re-send only the bridge-edge transforms on /tf at ~0.2 Hz.

        This is a workaround for CycloneDDS /tf_static latch unreliability.
        Late-joining nodes that missed the /tf_static latch can pick up the
        bridge edges from the dynamic /tf topic.
        """
        if not self._bridge_tfs:
            return
        stamp = self.get_clock().now().to_msg()
        for tf_msg in self._bridge_tfs:
            tf_msg.header.stamp = stamp
        self._tf_broadcaster.sendTransform(self._bridge_tfs)

    def _resolve_bridge_transform(
        self, spec: BridgeSpec
    ) -> tuple[np.ndarray | None, str]:
        """Resolve T(cam0 -> link) from the RealSense static chain.

        Since OpenVINS uses the RealSense color camera as its tracking camera,
        cam0 == color_optical_frame.  The bridge edge is simply:

            T(cam0 -> link) = T(color_optical -> link)

        This is a known static extrinsic from the D435i hardware calibration.
        No aruco marker detection is needed.
        """
        color_optical = spec.color_optical_frame
        if color_optical is None:
            return None, ""

        # Look up T(color_optical -> link) from the RealSense static chain.
        # This is the extrinsic between the color camera and the RealSense body.
        matrix = self._lookup_matrix(color_optical, spec.link_frame)
        if matrix is not None:
            return matrix, f"RealSense extrinsic {color_optical}->{spec.link_frame}"

        # Fallback: try depth_optical -> link (same rotation, no translation offset).
        # Less accurate but works before the full static chain is available.
        matrix = self._lookup_matrix(spec.fallback_optical_frame, spec.link_frame)
        if matrix is not None:
            return (
                matrix,
                f"RealSense extrinsic {spec.fallback_optical_frame}->{spec.link_frame} (approx)",
            )

        return None, ""

    def _warn_once(self, key: str, message: str):
        if key in self._warned:
            # Track repeated failures for periodic diagnostics.
            self._resolve_fail_counts[key] = self._resolve_fail_counts.get(key, 0) + 1
            return
        self._warned.add(key)
        self.get_logger().warn(message)

    def _log_diagnostics(self):
        """Periodically log which bridge transforms are still unresolved."""
        unresolved = []
        for spec in self._specs:
            if spec.name in self._published_from:
                continue
            color_optical = spec.color_optical_frame
            unresolved.append(
                f"{spec.name}: color_optical={color_optical}, "
                f"link={spec.link_frame}"
            )
        if unresolved:
            self.get_logger().warn(
                "Unresolved bridge transforms (waiting for RealSense static chain): "
                + "; ".join(unresolved)
            )


def main(args=None):
    rclpy.init(args=args)
    node = OpenVinsRealSenseTfBridge()
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
