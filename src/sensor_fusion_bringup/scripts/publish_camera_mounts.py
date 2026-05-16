#!/usr/bin/env python3
"""
Publish camera mount transforms + bounding box marker for RViz visualization.

Reads camera_mounts.yaml and publishes a TF tree plus a visualization marker.

Tree (all-mount mode):
  world -> palm_frame
    ├── d435i_arm_bottom_screw_frame_<mount>  (inverse of mount transform)
    │     └── d435i_arm_link_<mount>           (URDF offset)
    ├── bb_corner -> bb_opposite               (shared workspace bounding box)

Tree (single-mount mode):
  Same, but only the chosen mount is published.

A CUBE marker is published in palm_frame to visualize the bounding box.
Published at 1 Hz so RViz can latch it.

Usage:
  python3 publish_camera_mounts.py --all              # all mounts
  python3 publish_camera_mounts.py --mount NAME       # single mount
  python3 publish_camera_mounts.py --list             # list mounts
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Point, Quaternion, TransformStamped
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from visualization_msgs.msg import Marker


def _find_config() -> Path:
    candidates = [
        Path(__file__).resolve().parent.parent / "config" / "camera_mounts.yaml",
        Path("/prosthesis_ws/install/sensor_fusion_bringup/share/sensor_fusion_bringup/config/camera_mounts.yaml"),
    ]
    for p in candidates:
        if p.is_file():
            return p
    raise FileNotFoundError("Cannot locate camera_mounts.yaml")


def _q(data: dict) -> list:
    return data["quaternion"]


def _t(data: dict, key: str) -> float:
    return data["translation"][key]


def _quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Convert [x,y,z,w] quaternion to 3x3 rotation matrix."""
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ])


def _invert_transform(translation: dict, quat: list) -> tuple:
    """Invert a rigid transform (screw->palm) to (palm->screw).

    Returns (tx, ty, tz, qx, qy, qz, qw) for the inverted transform.
    """
    qx, qy, qz, qw = quat
    R = _quat_to_rot(qx, qy, qz, qw)
    t = np.array([translation["x"], translation["y"], translation["z"]])
    t_inv = -R.T @ t
    return (float(t_inv[0]), float(t_inv[1]), float(t_inv[2]),
            -qx, -qy, -qz, qw)


def _make_tf(stamp, parent: str, child: str,
             tx: float, ty: float, tz: float,
             qx: float, qy: float, qz: float, qw: float) -> TransformStamped:
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x = tx
    t.transform.translation.y = ty
    t.transform.translation.z = tz
    t.transform.rotation.x = qx
    t.transform.rotation.y = qy
    t.transform.rotation.z = qz
    t.transform.rotation.w = qw
    return t


def _make_bbox_marker(stamp, bb_data: dict) -> Marker:
    """Build a CUBE Marker from bounding box corner data in palm_frame."""
    corner = bb_data["palm_to_corner"]["translation"]
    opp = bb_data["corner_to_opposite"]["translation"]

    opp_in_palm = {
        "x": corner["x"] + opp["x"],
        "y": corner["y"] + opp["y"],
        "z": corner["z"] + opp["z"],
    }
    cx = (corner["x"] + opp_in_palm["x"]) / 2.0
    cy = (corner["y"] + opp_in_palm["y"]) / 2.0
    cz = (corner["z"] + opp_in_palm["z"]) / 2.0
    sx = abs(opp_in_palm["x"] - corner["x"])
    sy = abs(opp_in_palm["y"] - corner["y"])
    sz = abs(opp_in_palm["z"] - corner["z"])

    m = Marker()
    m.header.stamp = stamp
    m.header.frame_id = "palm_frame"
    m.ns = "workspace"
    m.id = 0
    m.type = Marker.CUBE
    m.action = Marker.ADD
    m.pose.position = Point(x=cx, y=cy, z=cz)
    m.pose.orientation.x = 0.0
    m.pose.orientation.y = 0.0
    m.pose.orientation.z = 0.0
    m.pose.orientation.w = 1.0
    m.scale.x = sx
    m.scale.y = sy
    m.scale.z = sz
    m.color.r = 0.2
    m.color.g = 0.8
    m.color.b = 0.2
    m.color.a = 0.15
    return m


_COLORS = [
    (1.0, 0.2, 0.2),
    (0.2, 1.0, 0.2),
    (0.2, 0.4, 1.0),
    (1.0, 0.9, 0.2),
    (1.0, 0.2, 1.0),
]


class CameraMountTFPublisher(Node):
    def __init__(self, config_path: Path, mount_name: str | None = None,
                 publish_all: bool = False):
        super().__init__("camera_mount_tf_publisher")
        self._broadcaster = StaticTransformBroadcaster(self)
        self._marker_pub = self.create_publisher(
            Marker, "/camera_mounts/bounding_box", 10)
        self._stamp = self.get_clock().now().to_msg()

        data = yaml.safe_load(config_path.read_text())
        self._data = data

        if publish_all:
            self._publish_all(data)
        elif mount_name is not None:
            self._publish_single(data, mount_name)

        self._bbox_marker = _make_bbox_marker(self._stamp, data["bounding_box"])
        self._bbox_marker.header.stamp = self.get_clock().now().to_msg()
        self._marker_pub.publish(self._bbox_marker)

        self._timer = self.create_timer(1.0, self._republish_marker)

    def _republish_marker(self):
        self._bbox_marker.header.stamp = self.get_clock().now().to_msg()
        self._marker_pub.publish(self._bbox_marker)

    def _publish_single(self, data: dict, mount_name: str):
        tfs = self._build_shared_tfs(data)
        tfs += self._build_bounding_box_tfs(data)
        tfs += self._build_mount_tfs(data, mount_name, f"_{mount_name}")
        self._broadcaster.sendTransform(tfs)
        self.get_logger().info(
            f"Published TF tree for mount '{mount_name}' + bounding box")

    def _publish_all(self, data: dict):
        tfs = self._build_shared_tfs(data)
        tfs += self._build_bounding_box_tfs(data)
        for name in data["mounts"]:
            tfs += self._build_mount_tfs(data, name, f"_{name}")
        self._broadcaster.sendTransform(tfs)
        names = ", ".join(data["mounts"].keys())
        self.get_logger().info(
            f"Published all mounts ({names})")

    def _build_shared_tfs(self, data: dict) -> list[TransformStamped]:
        s = self._stamp
        return [
            _make_tf(s, "world", "palm_frame",
                     0.0, 0.0, 0.0,
                     0.0, 1.0, 0.0, 0.0),
        ]

    def _build_bounding_box_tfs(self, data: dict) -> list[TransformStamped]:
        s = self._stamp
        tfs = []
        bb = data["bounding_box"]
        pc = bb["palm_to_corner"]
        tfs.append(_make_tf(s, "palm_frame", "bb_corner",
                            _t(pc, "x"), _t(pc, "y"), _t(pc, "z"),
                            *_q(pc)))
        co = bb["corner_to_opposite"]
        tfs.append(_make_tf(s, "bb_corner", "bb_opposite",
                            _t(co, "x"), _t(co, "y"), _t(co, "z"),
                            *_q(co)))
        return tfs

    def _build_mount_tfs(self, data: dict, mount_name: str,
                         suffix: str) -> list[TransformStamped]:
        s = self._stamp
        mount = data["mounts"][mount_name]
        sp = mount["screw_to_palm"]

        screw_frame = f"d435i_arm_bottom_screw_frame{suffix}"
        link_frame = f"d435i_arm_link{suffix}"

        itx, ity, itz, iqx, iqy, iqz, iqw = _invert_transform(
            sp["translation"], sp["quaternion"])

        tfs = []
        tfs.append(_make_tf(s, "palm_frame", screw_frame,
                            itx, ity, itz, iqx, iqy, iqz, iqw))

        sl = data["screw_to_link"]
        tfs.append(_make_tf(s, screw_frame, link_frame,
                            _t(sl, "x"), _t(sl, "y"), _t(sl, "z"),
                            *_q(sl)))
        return tfs


def main():
    parser = argparse.ArgumentParser(
        description="Publish camera mount TFs + bounding box marker")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--mount", "-m", default=None,
                       help="Publish a single mount by name")
    group.add_argument("--all", "-a", action="store_true",
                       help="Publish all mounts simultaneously")
    group.add_argument("--list", "-l", action="store_true",
                       help="List available mounts and exit")
    parser.add_argument("--config", "-c", default=None,
                        help="Path to camera_mounts.yaml")
    args, _ = parser.parse_known_args()

    config_path = Path(args.config) if args.config else _find_config()

    if args.list:
        data = yaml.safe_load(config_path.read_text())
        print("Available mounts:")
        for name, m in data["mounts"].items():
            print(f"  {name}: {m['description']}")
        return

    rclpy.init(args=sys.argv)

    if args.all:
        node = CameraMountTFPublisher(config_path, publish_all=True)
    elif args.mount:
        node = CameraMountTFPublisher(config_path, mount_name=args.mount)
    else:
        node = CameraMountTFPublisher(config_path, publish_all=True)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
