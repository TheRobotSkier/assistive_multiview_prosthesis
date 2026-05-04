"""Haptic controller: wrist ring (motors 0-3) + force feedback (motors 4-7).

Subscribes:
  /wrist/state                          Float64MultiArray [pos_deg, vel_deg_s]
  data_streams/fingers/forces/data      mia_hand_msgs/ForceData

Publishes:
  /haptic_band/motors                   Float32MultiArray (8 values, 0-100)

On startup: calls data_streams/fingers/forces/switch service to enable streaming.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float64MultiArray
from std_srvs.srv import SetBool

_MOTOR_ANGLES = [0.0, 90.0, 180.0, -90.0]   # bracelet A motor positions (deg)
_MIN_THRESHOLD = 5.0                           # % below which motor is silenced
_FORCE_MAX_RAW = 500                           # ADC counts at full scale


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

        self._wrist_angle: float = 0.0
        self._force_intensities: list[float] = [0.0] * 4  # motors 4-7

        self._motors_pub = self.create_publisher(Float32MultiArray, '/haptic_band/motors', 10)
        self.create_subscription(Float64MultiArray, '/wrist/state', self._on_wrist, 10)

        # Try to import ForceData — optional dep, graceful fallback
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

    def _on_force(self, msg):
        def _scale(raw: int) -> float:
            return min(100.0, max(0.0, abs(raw) / _FORCE_MAX_RAW * 100.0))
        self._force_intensities = [
            _scale(msg.thumb_nfor),
            _scale(msg.index_nfor),
            _scale(msg.mrl_nfor),
            0.0,   # motor 7 spare
        ]

    def _publish(self):
        ring = _wrist_ring(self._wrist_angle)
        out = Float32MultiArray()
        out.data = [float(v) for v in ring + self._force_intensities]
        self._motors_pub.publish(out)


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
