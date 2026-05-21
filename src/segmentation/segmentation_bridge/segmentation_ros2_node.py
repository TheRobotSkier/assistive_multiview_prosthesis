#!/usr/bin/env python3
"""
ROS2 segmentation node (Python 3.12, Jazzy).

Bridges ROS2 topics to the Python 3.8 inference server running on localhost:5678.

Topics
------
Subscribe:
  /segmentation/input_cloud      sensor_msgs/PointCloud2
      The full scene point cloud (XYZ or XYZRGB).
  /segmentation/click_positive   geometry_msgs/PointStamped
      A positive click (foreground). Transformed to cloud frame via TF2.
  /segmentation/click_negative   geometry_msgs/PointStamped
      A negative click (background). Transformed to cloud frame via TF2.
  /segmentation/reset            std_msgs/Empty
      Clear all accumulated clicks and the output cloud.

Publish:
  /segmentation/object_cloud     sensor_msgs/PointCloud2
      Foreground points from the most recent segmentation.
      An empty cloud is published on reset to clear RViz2.

Parameters
----------
  cubeedge      (float, default 0.05) – half-width of the click cube in metres.
  inference_url (str,   default 'http://127.0.0.1:5678') – inference server URL.

Inference is triggered only on click changes (new click or reset-then-click),
not on periodic cloud republishes.
"""

import base64
import threading

import numpy as np
import requests
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Empty
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # noqa: F401 – registers PointStamped transform support


# ---------------------------------------------------------------------------
# PointCloud2 helpers
# ---------------------------------------------------------------------------

def _parse_pointcloud2(msg: PointCloud2):
    """Return (xyz, rgb) numpy arrays from a PointCloud2 message.

    xyz: (N, 3) float32
    rgb: (N, 3) float32 in [0, 1]  – zeros if the cloud has no colour fields.
    """
    n = msg.width * msg.height
    step = msg.point_step
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, step)

    fields = {f.name: f for f in msg.fields}

    def _extract_f32(name: str) -> np.ndarray:
        off = fields[name].offset
        return np.frombuffer(raw[:, off:off + 4].copy().tobytes(), dtype=np.float32)

    xyz = np.column_stack([_extract_f32("x"), _extract_f32("y"), _extract_f32("z")])

    rgb = np.zeros((n, 3), dtype=np.float32)
    if "rgb" in fields:
        # PCL-style packed float32 encoding of 0x00RRGGBB
        packed_int = _extract_f32("rgb").view(np.uint32)
        rgb[:, 0] = ((packed_int >> 16) & 0xFF) / 255.0
        rgb[:, 1] = ((packed_int >> 8) & 0xFF) / 255.0
        rgb[:, 2] = (packed_int & 0xFF) / 255.0

    return xyz.astype(np.float32), rgb


def _build_pointcloud2(xyz: np.ndarray, header) -> PointCloud2:
    """Build an XYZ-only PointCloud2 from an (N, 3) float32 array."""
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = int(xyz.shape[0])
    msg.fields = [
        PointField(name="x", offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8,  datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * msg.width
    msg.is_dense = True
    msg.data = np.ascontiguousarray(xyz, dtype=np.float32).tobytes()
    return msg


def _empty_pointcloud2(header) -> PointCloud2:
    """Return a zero-point PointCloud2 in the same frame (used to clear RViz2)."""
    return _build_pointcloud2(np.zeros((0, 3), dtype=np.float32), header)


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class SegmentationNode(Node):

    def __init__(self):
        super().__init__("segmentation_node")

        self.declare_parameter("cubeedge", 0.05)
        self.declare_parameter("inference_url", "http://127.0.0.1:5678")

        self._lock = threading.Lock()
        self._cloud_xyz: np.ndarray | None = None
        self._cloud_rgb: np.ndarray | None = None
        self._cloud_header = None
        self._pos_clicks: list[list[float]] = []
        self._neg_clicks: list[list[float]] = []

        # TF2 listener — used to transform incoming clicks to the cloud frame
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(
            PointCloud2, "/segmentation/input_cloud", self._cloud_cb, 10)
        self.create_subscription(
            PointStamped, "/segmentation/click_positive", self._pos_click_cb, 10)
        self.create_subscription(
            PointStamped, "/segmentation/click_negative", self._neg_click_cb, 10)
        self.create_subscription(
            Empty, "/segmentation/reset", self._reset_cb, 10)

        self._pub = self.create_publisher(
            PointCloud2, "/segmentation/object_cloud", 10)

        self.get_logger().info("Segmentation node ready.")

    # --- helpers ------------------------------------------------------------

    def _transform_click_to_cloud_frame(self, msg: PointStamped) -> list[float]:
        """Return [x, y, z] of msg transformed into the current cloud frame.

        Raises RuntimeError if TF lookup fails — callers must catch and handle.
        """
        with self._lock:
            cloud_frame = self._cloud_header.frame_id if self._cloud_header else None

        if cloud_frame is None or msg.header.frame_id == cloud_frame:
            return [msg.point.x, msg.point.y, msg.point.z]

        try:
            transformed = self._tf_buffer.transform(msg, cloud_frame, timeout=rclpy.duration.Duration(seconds=0.5))
            return [transformed.point.x, transformed.point.y, transformed.point.z]
        except Exception as exc:
            raise RuntimeError(
                f"TF transform from '{msg.header.frame_id}' to '{cloud_frame}' failed: {exc}"
            ) from exc

    # --- subscribers --------------------------------------------------------

    def _cloud_cb(self, msg: PointCloud2):
        """Store incoming cloud. Does NOT trigger inference — that is click-driven."""
        xyz, rgb = _parse_pointcloud2(msg)
        with self._lock:
            self._cloud_xyz = xyz
            self._cloud_rgb = rgb
            self._cloud_header = msg.header

    def _pos_click_cb(self, msg: PointStamped):
        try:
            pt = self._transform_click_to_cloud_frame(msg)
        except RuntimeError as exc:
            self.get_logger().error(f"Positive click rejected: {exc}")
            return
        with self._lock:
            self._pos_clicks.append(pt)
        self.get_logger().info(f"[+] positive click at ({pt[0]:.3f}, {pt[1]:.3f}, {pt[2]:.3f}) (cloud frame)")
        threading.Thread(target=self._run_inference, daemon=True).start()

    def _neg_click_cb(self, msg: PointStamped):
        try:
            pt = self._transform_click_to_cloud_frame(msg)
        except RuntimeError as exc:
            self.get_logger().error(f"Negative click rejected: {exc}")
            return
        with self._lock:
            self._neg_clicks.append(pt)
        self.get_logger().info(f"[-] negative click at ({pt[0]:.3f}, {pt[1]:.3f}, {pt[2]:.3f}) (cloud frame)")
        threading.Thread(target=self._run_inference, daemon=True).start()

    def _reset_cb(self, _msg):
        with self._lock:
            self._pos_clicks.clear()
            self._neg_clicks.clear()
            header = self._cloud_header
        self.get_logger().info("Clicks reset.")
        # Publish an empty cloud to clear the RViz2 display
        if header is not None:
            empty_msg = _empty_pointcloud2(header)
            empty_msg.header.stamp = self.get_clock().now().to_msg()
            self._pub.publish(empty_msg)

    # --- inference ----------------------------------------------------------

    def _run_inference(self):
        with self._lock:
            if self._cloud_xyz is None:
                self.get_logger().warn("No cloud received yet – ignoring click.")
                return
            xyz = self._cloud_xyz.copy()
            rgb = self._cloud_rgb.copy()
            header = self._cloud_header
            pos_clicks = list(self._pos_clicks)
            neg_clicks = list(self._neg_clicks)

        cubeedge = self.get_parameter("cubeedge").value
        url = self.get_parameter("inference_url").value

        payload = {
            "xyz": base64.b64encode(xyz.tobytes()).decode(),
            "rgb": base64.b64encode(rgb.tobytes()).decode(),
            "positive_clicks": pos_clicks,
            "negative_clicks": neg_clicks,
            "cubeedge": cubeedge,
        }

        try:
            resp = requests.post(f"{url}/segment", json=payload, timeout=120)
            resp.raise_for_status()
            mask = np.array(resp.json()["mask"], dtype=bool)
        except Exception as exc:
            self.get_logger().error(f"Inference request failed: {exc}")
            return

        fg_xyz = xyz[mask]
        out_header = header
        out_header.stamp = self.get_clock().now().to_msg()

        if len(fg_xyz) == 0:
            self.get_logger().warn("Segmentation returned no foreground points.")
            self._pub.publish(_empty_pointcloud2(out_header))
            return

        cloud_msg = _build_pointcloud2(fg_xyz, out_header)
        self._pub.publish(cloud_msg)
        self.get_logger().info(
            f"Published segmented cloud: {len(fg_xyz)}/{len(xyz)} points.")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = SegmentationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

