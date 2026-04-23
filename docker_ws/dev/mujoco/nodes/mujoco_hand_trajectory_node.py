#!/usr/bin/env python3
"""Move the MuJoCo hand through a fixed start → end trajectory.

Sequence
--------
1. Wait for /mujoco/hand_pose (confirms InteractiveSystemInterface is active).
2. Instantly teleport the hand to *hand_start* via /mujoco/set_hand_pose.
3. Sleep 1.5 s to let MuJoCo physics settle after the teleport.
4. Publish a 4-second smooth motion to *hand_end* via /mujoco/move_hand.
5. Exit cleanly.

Pose convention
---------------
All poses are expressed as (X, Y, Z, Roll, Pitch, Yaw) in world frame (metres,
radians). RPY uses the standard ROS 'sxyz' extrinsic convention, which matches
tf_transformations.quaternion_from_euler(r, p, y).

hand_start: X=-0.3,   Y=0.5,  Z=0.41,  Roll=2,  Pitch=0,   Yaw=-0.4
hand_end:   X=-0.12, Y=0.1,  Z=0.31,  Roll=3.14, Pitch=-1,  Yaw=0
"""

from __future__ import annotations

import math
import sys
import time

import rclpy
from builtin_interfaces.msg import Time as RosTime
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node

# ---------------------------------------------------------------------------
# Pose definitions
# ---------------------------------------------------------------------------
HAND_START = dict(x=-0.3,   y=0.5,  z=0.41,  roll=2,  pitch=0.0,  yaw=-0.4)
HAND_END   = dict(x=-0.12, y=0.1,  z=0.31,  roll=3.14, pitch=-1.0, yaw=0.0)

TRAJECTORY_DURATION_S = 4.0   # seconds
SETTLE_DELAY_S        = 1.5   # pause between teleport and trajectory
READINESS_TIMEOUT_S   = 60.0  # max wait for sim to come up


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def rpy_to_quat(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """Return (qx, qy, qz, qw) from extrinsic XYZ RPY angles (radians)."""
    cr, sr = math.cos(roll  / 2), math.sin(roll  / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw   / 2), math.sin(yaw   / 2)
    return (
        sr * cp * cy - cr * sp * sy,  # qx
        cr * sp * cy + sr * cp * sy,  # qy
        cr * cp * sy - sr * sp * cy,  # qz
        cr * cp * cy + sr * sp * sy,  # qw
    )


def make_pose(x: float, y: float, z: float,
              roll: float, pitch: float, yaw: float) -> Pose:
    qx, qy, qz, qw = rpy_to_quat(roll, pitch, yaw)
    msg = Pose()
    msg.position.x, msg.position.y, msg.position.z = x, y, z
    msg.orientation.x, msg.orientation.y = qx, qy
    msg.orientation.z, msg.orientation.w = qz, qw
    return msg


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------
class HandTrajectoryNode(Node):
    def __init__(self) -> None:
        super().__init__('mujoco_hand_trajectory')

        self._sim_ready = False
        self._ready_sub = self.create_subscription(
            Pose, '/mujoco/hand_pose', self._on_hand_pose, 10)

        self._set_pose_pub = self.create_publisher(
            Pose, '/mujoco/set_hand_pose', 10)
        self._move_hand_pub = self.create_publisher(
            PoseStamped, '/mujoco/move_hand', 10)

    def _on_hand_pose(self, _msg: Pose) -> None:
        self._sim_ready = True

    # ------------------------------------------------------------------
    def wait_for_sim(self) -> bool:
        self.get_logger().info(
            f'Waiting for simulation (up to {READINESS_TIMEOUT_S:.0f} s)…')
        deadline = time.monotonic() + READINESS_TIMEOUT_S
        while not self._sim_ready:
            if time.monotonic() > deadline:
                return False
            rclpy.spin_once(self, timeout_sec=0.5)
        self.get_logger().info('Simulation is ready.')
        # wait another 5 seconds for sim in case
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.5)
        return True

    def teleport_to_start(self) -> None:
        pose = make_pose(**HAND_START)
        self._set_pose_pub.publish(pose)
        self.get_logger().info(
            f'Teleported hand to start: '
            f'({HAND_START["x"]}, {HAND_START["y"]}, {HAND_START["z"]}) '
            f'RPY=({HAND_START["roll"]}, {HAND_START["pitch"]}, {HAND_START["yaw"]})')

    def send_trajectory(self) -> None:
        msg = PoseStamped()
        # Encode duration in the stamp field (sec + nanosec/1e9 = duration_s)
        dur_sec = int(TRAJECTORY_DURATION_S)
        dur_nsec = int((TRAJECTORY_DURATION_S - dur_sec) * 1e9)
        msg.header.stamp = RosTime(sec=dur_sec, nanosec=dur_nsec)
        msg.pose = make_pose(**HAND_END)
        self._move_hand_pub.publish(msg)
        self.get_logger().info(
            f'Trajectory sent to end: '
            f'({HAND_END["x"]}, {HAND_END["y"]}, {HAND_END["z"]}) '
            f'RPY=({HAND_END["roll"]}, {HAND_END["pitch"]}, {HAND_END["yaw"]}) '
            f'over {TRAJECTORY_DURATION_S:.1f} s.')


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main() -> None:
    rclpy.init()
    node = HandTrajectoryNode()

    if not node.wait_for_sim():
        node.get_logger().fatal(
            f'Simulation did not start within {READINESS_TIMEOUT_S:.0f} s. Aborting.')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    node.teleport_to_start()

    node.get_logger().info(f'Settling for {SETTLE_DELAY_S:.1f} s…')
    time.sleep(SETTLE_DELAY_S)

    node.send_trajectory()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
