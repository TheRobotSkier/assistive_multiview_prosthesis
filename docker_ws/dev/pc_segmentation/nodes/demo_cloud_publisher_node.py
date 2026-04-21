#!/usr/bin/env python3
"""
Loads a PLY file and publishes it as a sensor_msgs/PointCloud2 on
/segmentation/input_cloud with transient-local QoS so RViz2 receives
it even if it subscribes after the first publish.
"""
import argparse
import struct
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def load_ply_numpy(path: str):
    """Load x, y, z, r, g, b from a PLY file using open3d or plyfile fallback."""
    try:
        import open3d as o3d
        pcd = o3d.io.read_point_cloud(path)
        xyz = np.asarray(pcd.points, dtype=np.float32)
        if pcd.has_colors():
            rgb = (np.asarray(pcd.colors) * 255).astype(np.uint8)
        else:
            rgb = np.full((len(xyz), 3), 128, dtype=np.uint8)
        return xyz, rgb
    except Exception as e:
        print(f"[cloud_publisher] open3d failed ({e}), trying plyfile...")

    from plyfile import PlyData
    ply = PlyData.read(path)
    v = ply['vertex']
    xyz = np.column_stack([v['x'], v['y'], v['z']]).astype(np.float32)
    try:
        rgb = np.column_stack([v['red'], v['green'], v['blue']]).astype(np.uint8)
    except Exception:
        rgb = np.full((len(xyz), 3), 128, dtype=np.uint8)
    return xyz, rgb


def make_pointcloud2_msg(xyz: np.ndarray, rgb: np.ndarray, frame_id: str) -> PointCloud2:
    """Build an XYZRGB PointCloud2 message (PCL/RViz2 RGB float packing)."""
    n = len(xyz)
    # Pack RGB as uint32 → reinterpret as float32 (PCL convention)
    rgb32 = (rgb[:, 0].astype(np.uint32) << 16
             | rgb[:, 1].astype(np.uint32) << 8
             | rgb[:, 2].astype(np.uint32))
    rgb_f = rgb32.view(np.float32)

    data = np.zeros(n, dtype=np.dtype([
        ('x', np.float32), ('y', np.float32), ('z', np.float32),
        ('_pad', np.float32),
        ('rgb', np.float32),
    ]))
    data['x'] = xyz[:, 0]
    data['y'] = xyz[:, 1]
    data['z'] = xyz[:, 2]
    data['rgb'] = rgb_f

    msg = PointCloud2()
    msg.header = Header()
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = n
    msg.is_dense = True
    msg.is_bigendian = False
    msg.point_step = data.dtype.itemsize  # 20 bytes
    msg.row_step = msg.point_step * n
    msg.fields = [
        PointField(name='x',   offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y',   offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z',   offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=16, datatype=PointField.FLOAT32, count=1),
    ]
    msg.data = data.tobytes()
    return msg


class CloudPublisher(Node):
    def __init__(self, ply_path: str, frame_id: str, republish_period: float):
        super().__init__('demo_cloud_publisher')
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(PointCloud2, '/segmentation/input_cloud', qos)
        self.get_logger().info(f'Loading PLY: {ply_path}')
        xyz, rgb = load_ply_numpy(ply_path)
        self.msg = make_pointcloud2_msg(xyz, rgb, frame_id)
        self.get_logger().info(f'Loaded {len(xyz)} points. Publishing on /segmentation/input_cloud')
        self._publish()
        self.create_timer(republish_period, self._publish)

    def _publish(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--ply', default='/miahand_ws/src/dev/pc_segmentation/depth_images/D435_img_1.ply')
    parser.add_argument('--frame', default='map')
    parser.add_argument('--republish-period', type=float, default=5.0)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=sys.argv[:1] + ros_args)
    node = CloudPublisher(args.ply, args.frame, args.republish_period)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
