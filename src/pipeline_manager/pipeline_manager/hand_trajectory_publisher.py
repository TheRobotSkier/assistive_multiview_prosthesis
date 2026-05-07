#!/usr/bin/env python3
"""Hand Trajectory Publisher — synthetic hand trajectory for testing.

Subscribes to /grasp_preshaping/target_hand_pose and publishes interpolated
poses to /hand_pose at 100 Hz, moving the virtual hand toward the target
at a configurable speed. Designed for automated testing without physical
hand movement.

Subscribes:
    /grasp_preshaping/target_hand_pose  (geometry_msgs/PoseStamped)

Publishes:
    /hand_pose  (geometry_msgs/PoseStamped) — interpolated hand pose
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped


def _quaternion_conjugate(q):
    """Return conjugate of quaternion (w, x, y, z)."""
    return (q[0], -q[1], -q[2], -q[3])


def _quaternion_multiply(a, b):
    """Multiply two quaternions (w, x, y, z)."""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


def _quaternion_slerp(q1, q2, t):
    """Spherical linear interpolation between two quaternions.

    Args:
        q1: Start quaternion (w, x, y, z)
        q2: End quaternion (w, x, y, z)
        t:  Interpolation factor [0, 1]

    Returns:
        Interpolated quaternion (w, x, y, z), normalized.
    """
    dot = q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3]

    # Handle negative dot product — flip the second quaternion
    if dot < 0.0:
        q2 = (-q2[0], -q2[1], -q2[2], -q2[3])
        dot = -dot

    # Clamp to avoid numerical issues at the edges
    dot = min(max(dot, -1.0), 1.0)

    theta = math.acos(dot)
    sin_theta = math.sin(theta)

    # If the angle is very small, use linear interpolation (faster, stable)
    if sin_theta < 1e-6:
        result = (
            q1[0] + t * (q2[0] - q1[0]),
            q1[1] + t * (q2[1] - q1[1]),
            q1[2] + t * (q2[2] - q1[2]),
            q1[3] + t * (q2[3] - q1[3]),
        )
    else:
        a = math.sin((1.0 - t) * theta) / sin_theta
        b = math.sin(t * theta) / sin_theta
        result = (
            a * q1[0] + b * q2[0],
            a * q1[1] + b * q2[1],
            a * q1[2] + b * q2[2],
            a * q1[3] + b * q2[3],
        )

    # Normalize
    norm = math.sqrt(result[0] ** 2 + result[1] ** 2 + result[2] ** 2 + result[3] ** 2)
    if norm > 0.0:
        result = (result[0] / norm, result[1] / norm, result[2] / norm, result[3] / norm)
    return result


def _position_distance(a, b):
    """Euclidean distance between two position tuples (x, y, z)."""
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    dz = a[2] - b[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


class HandTrajectoryPublisher(Node):
    """Publishes interpolated hand poses for testing without hardware."""

    def __init__(self):
        super().__init__("hand_trajectory_publisher")

        # ── Parameters ───────────────────────────────────────────────────
        self.declare_parameter("speed", 0.05)      # m/s
        self.declare_parameter("auto_start", False)

        self._speed = self.get_parameter("speed").value
        self._auto_start = self.get_parameter("auto_start").value

        # ── State ────────────────────────────────────────────────────────
        self._current_pose = PoseStamped()
        self._current_pose.header.frame_id = "world"
        self._current_pose.pose.orientation.w = 1.0

        self._target_pose: PoseStamped | None = None
        self._target_received = False

        # ── Publisher (latched for consistency with hand_pose_publisher) ─
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._publisher = self.create_publisher(PoseStamped, "/hand_pose", latched)

        # ── Subscription ─────────────────────────────────────────────────
        self.create_subscription(
            PoseStamped,
            "/grasp_preshaping/target_hand_pose",
            self._on_target_pose,
            10,
        )

        # ── Timer: always running at 100 Hz ──────────────────────────────
        # - Without target: publishes current pose (if auto_start) or nothing
        # - With target: interpolates toward target each tick
        self._timer = self.create_timer(1.0 / 100.0, self._tick)

        self.get_logger().info(
            f"Hand trajectory publisher started — speed={self._speed} m/s, "
            f"auto_start={self._auto_start}"
        )

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _on_target_pose(self, msg: PoseStamped) -> None:
        """Store the target pose and mark that a target has been received."""
        self._target_received = True
        self._target_pose = msg
        self.get_logger().info(
            f"Target pose received: ({msg.pose.position.x:.3f}, "
            f"{msg.pose.position.y:.3f}, {msg.pose.position.z:.3f})"
        )

    # ── Main loop ─────────────────────────────────────────────────────────

    def _tick(self) -> None:
        """Called at 100 Hz: interpolate toward target and publish."""
        # Step 1: interpolate if we have a target
        if self._target_pose is not None:
            self._interpolate_toward_target()

        # Step 2: decide whether to publish
        if not self._auto_start and not self._target_received:
            return  # Don't publish until we have a target (or auto_start)
        if self._target_pose is not None:
            self._check_arrival()

        # Stamp and publish
        self._current_pose.header.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(self._current_pose)

    # ── Interpolation ─────────────────────────────────────────────────────

    def _interpolate_toward_target(self) -> None:
        """Move current pose one step toward the target pose.

        Position uses linear interpolation (LERP) capped at speed * dt.
        Orientation uses spherical linear interpolation (SLERP).
        """
        target = self._target_pose.pose

        # ── Position ─────────────────────────────────────────────────
        cp = self._current_pose.pose.position
        tp = target.position

        dx = tp.x - cp.x
        dy = tp.y - cp.y
        dz = tp.z - cp.z
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        if dist < 1e-9:
            step_ratio = 1.0  # Already there
        else:
            max_step = self._speed * 0.01  # speed * dt (100 Hz = 0.01 s)
            step_ratio = min(max_step / dist, 1.0)

        cp.x += dx * step_ratio
        cp.y += dy * step_ratio
        cp.z += dz * step_ratio

        # ── Orientation ──────────────────────────────────────────────
        cq = (self._current_pose.pose.orientation.w,
              self._current_pose.pose.orientation.x,
              self._current_pose.pose.orientation.y,
              self._current_pose.pose.orientation.z)
        tq = (target.orientation.w,
              target.orientation.x,
              target.orientation.y,
              target.orientation.z)

        # Determine angular distance to decide slerp factor
        # Use quaternion dot product to get angular distance
        dot_q = cq[0] * tq[0] + cq[1] * tq[1] + cq[2] * tq[2] + cq[3] * tq[3]
        dot_q = min(max(dot_q, -1.0), 1.0)
        angular_dist = math.acos(abs(dot_q))

        if angular_dist < 1e-6:
            slerp_t = 1.0
        else:
            # Scale angular speed proportionally to linear speed
            # Equivalent: rotate at same "speed" as translation
            max_angular_step = self._speed * 0.01 / 0.5  # ~rad/tick for 0.5m reference
            slerp_t = min(max_angular_step / angular_dist, 1.0) if angular_dist > 0 else 1.0

        iq = _quaternion_slerp(cq, tq, slerp_t)

        self._current_pose.pose.orientation.w = iq[0]
        self._current_pose.pose.orientation.x = iq[1]
        self._current_pose.pose.orientation.y = iq[2]
        self._current_pose.pose.orientation.z = iq[3]

    def _check_arrival(self) -> None:
        """Check if close enough to target and snap if so."""
        cp = self._current_pose.pose.position
        tp = self._target_pose.pose.position

        dist = _position_distance((cp.x, cp.y, cp.z),
                                  (tp.x, tp.y, tp.z))

        if dist < 0.005:
            self.get_logger().info(
                f"Arrived at target (distance={dist:.4f}m < 0.005m)"
            )
            # Snap exactly to target
            cp.x = tp.x
            cp.y = tp.y
            cp.z = tp.z
            self._current_pose.pose.orientation = self._target_pose.pose.orientation
            self._target_pose = None  # Clear target — stop interpolating


def main(args=None):
    rclpy.init(args=args)
    node = HandTrajectoryPublisher()
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
