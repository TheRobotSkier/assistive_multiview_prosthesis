#!/usr/bin/env python3
"""Mock point cloud publisher for testing without a RealSense camera.

Publishes a synthetic point cloud with a small sphere/box "object"
at a configurable position. Useful for testing the segmentation and
preshaping pipeline without hardware.

Publishes:
    /camera/depth/color/points  (PointCloud2) - Synthetic point cloud
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


class MockCloudPublisher(Node):
    def __init__(self):
        super().__init__("mock_cloud_publisher")

        self.declare_parameter("publish_hz", 5.0)
        self.declare_parameter("object_type", "sphere")  # sphere or box
        self.declare_parameter("object_center", [0.4, 0.0, 0.2])
        self.declare_parameter("object_radius", 0.04)
        self.declare_parameter("cloud_width", 640)
        self.declare_parameter("cloud_height", 480)

        publish_hz = self.get_parameter("publish_hz").value
        obj_type = self.get_parameter("object_type").value
        obj_center = self.array_param("object_center")
        obj_radius = self.get_parameter("object_radius").value

        self.publisher = self.create_publisher(PointCloud2, "/camera/depth/color/points", 5)

        # Generate static point cloud with object
        self.cloud_msg = self._generate_cloud(obj_type, obj_center, obj_radius)

        self.timer = self.create_timer(1.0 / publish_hz, self._publish)
        self.get_logger().info(
            f"Mock cloud publisher started. Object: {obj_type} at {obj_center}, "
            f"radius: {obj_radius}m, hz: {publish_hz}"
        )

    def array_param(self, name):
        return np.array(self.get_parameter(name).value, dtype=np.float64)

    def _generate_cloud(self, obj_type, center, radius):
        """Generate a synthetic point cloud with a tabletop and object."""
        width = self.get_parameter("cloud_width").value
        height = self.get_parameter("cloud_height").value

        # Create tabletop plane at z=0
        x = np.linspace(0.1, 0.8, width)
        y = np.linspace(-0.4, 0.4, height)
        xx, yy = np.meshgrid(x, y)
        zz = np.zeros_like(xx)

        # Add object
        if obj_type == "sphere":
            dist = np.sqrt((xx - center[0]) ** 2 + (yy - center[1]) ** 2)
            mask = dist < radius
            zz[mask] = center[2] + np.sqrt(
                np.maximum(radius ** 2 - (xx[mask] - center[0]) ** 2 - (yy[mask] - center[1]) ** 2, 0)
            )
        elif obj_type == "box":
            mask = (
                (np.abs(xx - center[0]) < radius)
                & (np.abs(yy - center[1]) < radius)
            )
            zz[mask] = center[2]

        # Flatten to point list
        points = np.stack([xx.ravel(), yy.ravel(), zz.ravel()], axis=-1).astype(np.float32)

        # Build PointCloud2 message
        header = Header(frame_id="camera_color_optical_frame")
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud = PointCloud2(
            header=header,
            height=height,
            width=width,
            fields=fields,
            is_bigendian=False,
            point_step=12,
            row_step=12 * width,
            is_dense=True,
            data=points.tobytes(),
        )
        return cloud

    def _publish(self):
        self.cloud_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.cloud_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockCloudPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
