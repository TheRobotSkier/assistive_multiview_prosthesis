"""Haptic controller: wrist ring (motors 0-3) + force feedback (motors 4-7).

Subscribes:
  /wrist/state                          Float64MultiArray [pos_deg, vel_deg_s]
  data_streams/fingers/forces/data      mia_hand_msgs/ForceData
  /pipeline/state                       std_msgs/Int32

Publishes:
  /haptic_band/motors                   Float32MultiArray (8 values, 0-100)

On startup: calls data_streams/fingers/forces/switch service to enable streaming.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float64MultiArray, Int32
from std_srvs.srv import SetBool

_MOTOR_ANGLES = [0.0, 90.0, 180.0, -90.0]   # bracelet A motor positions (deg)
_MIN_THRESHOLD = 5.0                           # % below which motor is silenced

# Pipeline state integer codes (match pipeline_manager/State)
_IDLE = 0
_GRASPING = 4
_HOLDING = 5
_RELEASING = 6


def _circ_dist(a: float, b: float) -> float:
    """Shortest angular distance on a circle, result in [0, 180]."""
    return abs((a - b + 180.0) % 360.0 - 180.0)


def _wrist_ring(angle_deg: float) -> list[float]:
    """Compute motor intensities 0-100 for motors 0-3 given wrist angle."""
    dists = [_circ_dist(angle_deg, m) for m in _MOTOR_ANGLES]
    sorted_idx = sorted(range(4), key=lambda i: dists[i])
    i0, i1 = sorted_idx[0], sorted_idx[1]
    d0, d1 = dists[i0], dists[i1]
    span = d0 + d1
    intensities = [0.0] * 4
    if span < 1e-6:
        intensities[i0] = 100.0
    else:
        intensities[i0] = (d1 / span) * 100.0
        intensities[i1] = (d0 / span) * 100.0
    return [v if v >= _MIN_THRESHOLD else 0.0 for v in intensities]


class HapticControllerNode(Node):
    def __init__(self):
        super().__init__('haptic_controller')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('pipeline_state_topic', '/pipeline/state')
        self.declare_parameter('release_buzz_duration_s', 0.5)
        self.declare_parameter('release_buzz_intensity', 60.0)
        self.declare_parameter('force_max_raw', 500.0)
        self.declare_parameter('force_stale_timeout_s', 1.0)

        self._pipeline_state_topic = self.get_parameter('pipeline_state_topic').value
        self._release_buzz_duration = self.get_parameter('release_buzz_duration_s').value
        self._release_buzz_intensity = self.get_parameter('release_buzz_intensity').value
        self._force_max_raw = self.get_parameter('force_max_raw').value
        self._force_stale_timeout = self.get_parameter('force_stale_timeout_s').value

        # ── State ─────────────────────────────────────────────────────────
        self._wrist_angle: float = 0.0
        self._force_intensities: list[float] = [0.0] * 4  # motors 4-7
        self._pipeline_state: int = _IDLE
        self._prev_pipeline_state: int = _IDLE
        self._force_data_stamp: float = 0.0
        self._release_buzz_end: float = 0.0

        # ── Publishers ────────────────────────────────────────────────────
        self._motors_pub = self.create_publisher(Float32MultiArray, '/haptic_band/motors', 10)

        # ── Subscriptions ─────────────────────────────────────────────────
        self.create_subscription(Float64MultiArray, '/wrist/state', self._on_wrist, 10)
        self.create_subscription(
            Int32, self._pipeline_state_topic, self._on_pipeline_state, 10)

        # Force feedback (optional dep, graceful fallback)
        try:
            from mia_hand_msgs.msg import ForceData
            self.create_subscription(
                ForceData, 'data_streams/fingers/forces/data', self._on_force, 10)
            self._enable_force_stream()
            self.get_logger().info('Force feedback enabled.')
        except ImportError:
            self.get_logger().warn(
                'mia_hand_msgs not found — force feedback (motors 4-7) disabled.')

        self.create_timer(0.05, self._publish)   # 20 Hz

    def _enable_force_stream(self):
        client = self.create_client(SetBool, 'data_streams/fingers/forces/switch')
        if client.wait_for_service(timeout_sec=5.0):
            req = SetBool.Request()
            req.data = True
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            if future.result() and future.result().success:
                self.get_logger().info('Force streaming enabled.')
            else:
                self.get_logger().warn('Force stream switch returned failure or timed out.')
        else:
            self.get_logger().warn(
                'data_streams/fingers/forces/switch service not available — '
                'force feedback disabled.')

    def _on_wrist(self, msg: Float64MultiArray):
        if msg.data:
            self._wrist_angle = float(msg.data[0])

    def _on_pipeline_state(self, msg: Int32):
        self._prev_pipeline_state = self._pipeline_state
        self._pipeline_state = msg.data

        # Detect transition into RELEASING -> start release buzz
        if (self._pipeline_state == _RELEASING
                and self._prev_pipeline_state != _RELEASING):
            self._release_buzz_end = (
                self.get_clock().now().nanoseconds / 1e9
                + self._release_buzz_duration)
            self.get_logger().info('Release buzz triggered')

    def _on_force(self, msg):
        def _scale(raw: int) -> float:
            return min(100.0, max(0.0, abs(raw) / self._force_max_raw * 100.0))
        self._force_intensities = [
            _scale(msg.thumb_nfor),
            _scale(msg.index_nfor),
            _scale(msg.mrl_nfor),
            0.0,   # motor 7 spare
        ]
        self._force_data_stamp = self.get_clock().now().nanoseconds / 1e9

    def _publish(self):
        ring = _wrist_ring(self._wrist_angle)
        now = self.get_clock().now().nanoseconds / 1e9

        # Decide force motor output
        if now < self._release_buzz_end:
            # Release buzz active — override force motors
            force = [self._release_buzz_intensity] * 4
        elif self._pipeline_state in (_GRASPING, _HOLDING):
            # Force feedback active only during GRASPING/HOLDING
            if now - self._force_data_stamp > self._force_stale_timeout:
                # Stale data — zero force motors
                force = [0.0] * 4
            else:
                force = list(self._force_intensities)
        else:
            # Not in a gripping state — zero force motors
            force = [0.0] * 4

        out = Float32MultiArray()
        out.data = [float(v) for v in ring + force]
        self._motors_pub.publish(out)

    def destroy_node(self):
        # Publish zeros before shutting down
        out = Float32MultiArray()
        out.data = [0.0] * 8
        self._motors_pub.publish(out)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HapticControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
