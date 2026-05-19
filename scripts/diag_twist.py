#!/usr/bin/env python3
"""Quick diagnostic for twist propagation issues."""
import os
os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import time

rclpy.init()
node = Node('diag_node')

best_effort = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

reliable = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

results = {}

def on_pose(msg):
    results['pose'] = {
        'stamp': f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec}",
        'frame': msg.header.frame_id,
        'pos': f"({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}, {msg.pose.position.z:.3f})",
    }

def on_cloud_be(msg):
    results['cloud_be'] = {
        'stamp': f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec}",
        'frame': msg.header.frame_id,
        'points': msg.width * msg.height,
    }

def on_cloud_rel(msg):
    results['cloud_rel'] = {
        'stamp': f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec}",
        'frame': msg.header.frame_id,
        'points': msg.width * msg.height,
    }

def on_odom(msg):
    results['odom'] = {
        'stamp': f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec}",
        'frame': msg.header.frame_id,
    }

def on_status(msg):
    results['status'] = msg.data

node.create_subscription(PoseStamped, '/hand_pose', on_pose, 10)
node.create_subscription(PointCloud2, '/arm/d435i_arm/points_marker_map', on_cloud_be, best_effort)
node.create_subscription(PointCloud2, '/arm/d435i_arm/points_marker_map', on_cloud_rel, reliable)
node.create_subscription(Odometry, '/hand_odom', on_odom, 10)
node.create_subscription(String, '/twist_propagation/status', on_status, 10)

print("Spinning for 3 seconds...")
start = time.time()
while time.time() - start < 3.0:
    rclpy.spin_once(node, timeout_sec=0.1)

print("\n=== DIAGNOSTIC RESULTS ===")
now_ns = node.get_clock().now().nanoseconds
now_s = now_ns / 1e9
print(f"Local clock: {now_s:.3f}")

for key, val in sorted(results.items()):
    print(f"\n{key}: {val}")

if 'pose' not in results:
    print("\n!! NO /hand_pose received — relay may not be publishing")
if 'cloud_be' not in results:
    print("\n!! NO cloud via BEST_EFFORT — Jetson publisher issue")
if 'cloud_rel' not in results:
    print("\n!! NO cloud via RELIABLE — expected if publisher is BEST_EFFORT")
if 'cloud_be' in results:
    cloud_stamp = int(results['cloud_be']['stamp'].split('.')[0]) + int(results['cloud_be']['stamp'].split('.')[1]) * 1e-9
    age = now_s - cloud_stamp
    print(f"\nCloud age: {age:.3f}s {'(OK)' if age < 2.0 else '(TOO OLD!)'}")
if 'pose' in results:
    pose_stamp = int(results['pose']['stamp'].split('.')[0]) + int(results['pose']['stamp'].split('.')[1]) * 1e-9
    age = now_s - pose_stamp
    print(f"Pose age: {age:.3f}s {'(OK)' if age < 1.0 else '(TOO OLD!)'}")
if 'status' not in results:
    print("\n!! NO /twist_propagation/status — node timer may be stuck or crashing")

node.destroy_node()
rclpy.shutdown()
