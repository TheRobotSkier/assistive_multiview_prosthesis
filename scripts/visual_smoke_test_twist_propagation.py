#!/usr/bin/env python3
"""Visual smoke test for twist_propagation node.

Publishes mock hand poses and a point cloud so the twist propagation
visualization can be verified in RViz.

Usage:
    1. Launch the node with RViz:
       ros2 launch twist_propagation twist_propagation.launch.py active:=true
    2. In another terminal, run this script:
       python3 scripts/visual_smoke_test_twist_propagation.py

You should see in RViz:
    - Green trajectory line extending from the hand position
    - Semi-transparent red collision spheres along the predicted path
    - A green hit marker sphere when the path intersects the point cloud
    - The point cloud cluster at (0.6, 0.0, 0.5)
"""

import math
import struct
import sys
import time

import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, PointField


def make_cloud(xyz_points, frame_id="camera_front_depth"):
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp.sec = int(time.time())
    msg.header.stamp.nanosec = int((time.time() % 1) * 1e9)
    msg.height = 1
    msg.width = len(xyz_points)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(xyz_points)
    buf = bytearray()
    for pt in xyz_points:
        buf.extend(struct.pack("fff", float(pt[0]), float(pt[1]), float(pt[2])))
    msg.data = bytes(buf)
    msg.is_dense = True
    return msg


def main():
    rclpy.init()
    node = Node("visual_smoke_test")

    pose_pub = node.create_publisher(PoseStamped, "/hand_pose", 10)
    cloud_pub = node.create_publisher(PointCloud2, "/camera/depth/color/points", 10)

    # Wait for subscribers
    print("Waiting for subscribers...")
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.5)
        if pose_pub.get_subscription_count() > 0:
            break

    # Publish a point cloud cluster at x=0.6, y=0.0, z=0.5
    rng = np.random.default_rng(42)
    cluster = rng.normal(loc=[0.6, 0.0, 0.5], scale=0.02, size=(300, 3))
    bg = rng.uniform(low=-1.0, high=2.0, size=(200, 3))
    bg[:, 2] = rng.uniform(0.3, 1.5, size=200)
    pts = np.vstack([cluster, bg])

    print(f"Publishing cloud with {len(pts)} points (cluster at 0.6, 0.0, 0.5)")
    cloud_pub.publish(make_cloud(pts))

    # Publish hand poses moving toward the cluster from x=0.0
    print("Publishing hand poses moving from x=0.0 toward x=0.6 at 0.3 m/s...")
    print("Watch RViz for:")
    print("  - Green trajectory line (predicted path)")
    print("  - Red semi-transparent spheres (collision geometry)")
    print("  - Green hit marker when path intersects cloud")
    print()
    print("Press Ctrl+C to stop.")

    start_x = 0.0
    vx = 0.3
    dt = 0.05
    t = 0.0

    try:
        while rclpy.ok():
            px = start_x + vx * t
            py = 0.0
            pz = 0.5

            msg = PoseStamped()
            msg.header.frame_id = "camera_front_depth"
            msg.header.stamp.sec = int(time.time())
            msg.header.stamp.nanosec = int((time.time() % 1) * 1e9)
            msg.pose.position.x = px
            msg.pose.position.y = py
            msg.pose.position.z = pz
            msg.pose.orientation.w = 1.0
            pose_pub.publish(msg)

            # Re-publish cloud periodically (every 2s)
            if int(t * 20) % 40 == 0:
                cloud_pub.publish(make_cloud(pts))

            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(dt)
            t += dt

            # Reset when we've gone past the target
            if px > 1.0:
                t = 0.0
                print(f"  Resetting hand position to x={start_x}")

    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
