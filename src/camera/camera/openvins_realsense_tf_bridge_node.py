#!/usr/bin/env python3
"""Bridge OpenVINS camera frames to RealSense frame trees.

The Jetson camera stack publishes two useful but disconnected TF trees:

    marker_map -> head_imu -> head_cam0
    head_d435i_head_link -> head_d435i_head_depth_frame -> ...

PointCloud2 messages use the RealSense depth optical frames, while OpenVINS
owns the marker_map tree. This node publishes the missing parent transform:

    head_cam0 -> head_d435i_head_link
    arm_cam0  -> arm_d435i_arm_link

It prefers the marker/OpenVINS "body_display" frames when available because
those encode the camera body pose in the marker_map tree. If those frames are
not present yet, it falls back to the static RealSense optical->link transform.
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
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


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


@dataclass(frozen=True)
class BridgeSpec:
    name: str
    parent_frame: str
    link_frame: str
    anchor_frame: str
    anchor_frame_mode: str
    fallback_optical_frame: str


class OpenVinsRealSenseTfBridge(Node):
    """Publishes the missing OpenVINS camera -> RealSense link transforms."""

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
                "head_d435i_head_color_optical_frame_body_display",
                "arm_d435i_arm_color_optical_frame_body_display",
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
        self.declare_parameter("tf_lookup_timeout_s", 0.05)
        self.declare_parameter("fallback_to_optical_assumption", True)

        self._timeout = Duration(
            seconds=float(self.get_parameter("tf_lookup_timeout_s").value)
        )
        self._fallback_enabled = bool(
            self.get_parameter("fallback_to_optical_assumption").value
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._specs = self._load_specs()
        self._warned: set[str] = set()
        self._published_from: dict[str, str] = {}

        rate = max(float(self.get_parameter("publish_rate_hz").value), 1.0)
        self.create_timer(1.0 / rate, self._publish_all)

        summary = ", ".join(
            f"{spec.parent_frame}->{spec.link_frame}" for spec in self._specs
        )
        self.get_logger().info(f"OpenVINS/RealSense TF bridge active: {summary}")

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

    def _lookup_matrix(self, target_frame: str, source_frame: str) -> np.ndarray | None:
        try:
            tf_msg = self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=self._timeout,
            )
            return _transform_to_matrix(tf_msg)
        except Exception:
            return None

    def _publish_all(self):
        stamp = self.get_clock().now().to_msg()
        for spec in self._specs:
            matrix, source = self._resolve_bridge_transform(spec)
            if matrix is None:
                self._warn_once(
                    spec.name,
                    "waiting for TF inputs for "
                    f"{spec.parent_frame}->{spec.link_frame} "
                    f"(anchor={spec.anchor_frame}, fallback={spec.fallback_optical_frame})",
                )
                continue

            tf_msg = _matrix_to_transform(
                matrix,
                parent_frame=spec.parent_frame,
                child_frame=spec.link_frame,
                stamp=stamp,
            )
            self._tf_broadcaster.sendTransform(tf_msg)

            if self._published_from.get(spec.name) != source:
                self._published_from[spec.name] = source
                self.get_logger().info(
                    f"{spec.name}: publishing {spec.parent_frame}->{spec.link_frame} "
                    f"from {source}"
                )

    def _resolve_bridge_transform(
        self, spec: BridgeSpec
    ) -> tuple[np.ndarray | None, str]:
        if spec.anchor_frame:
            parent_to_anchor = self._lookup_matrix(
                spec.parent_frame, spec.anchor_frame
            )
            if parent_to_anchor is not None:
                if spec.anchor_frame_mode == "link":
                    return parent_to_anchor, f"anchor {spec.anchor_frame} as link"

                optical_to_link = self._lookup_matrix(
                    spec.fallback_optical_frame, spec.link_frame
                )
                if optical_to_link is not None:
                    return (
                        parent_to_anchor @ optical_to_link,
                        f"anchor {spec.anchor_frame} as optical + RealSense extrinsic",
                    )

        if not self._fallback_enabled:
            return None, ""

        optical_to_link = self._lookup_matrix(
            spec.fallback_optical_frame, spec.link_frame
        )
        if optical_to_link is None:
            return None, ""
        return optical_to_link, (
            f"fallback assuming {spec.parent_frame} == {spec.fallback_optical_frame}"
        )

    def _warn_once(self, key: str, message: str):
        if key in self._warned:
            return
        self._warned.add(key)
        self.get_logger().warn(message)


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
