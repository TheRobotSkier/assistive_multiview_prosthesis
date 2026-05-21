"""Haptic controller: wrist ring (motors 0-3) + force feedback (motors 4-7).

Subscribes:
  /wrist/state                          Float64MultiArray [pos_deg, vel_deg_s]
  data_streams/fingers/forces/data      mia_hand_msgs/ForceData
  /force_controller/status                mia_hand_msgs/ForceControllerStatus
  /pipeline/state                       std_msgs/Int32
  /haptic_band/connection_status        std_msgs/Bool

Publishes:
  /haptic_band/motors                   Float32MultiArray (8 values, 0-100)

On startup: calls data_streams/fingers/forces/switch service to enable streaming.
"""
from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, Float64MultiArray, Int32
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
        self.declare_parameter('acquired_pulse_intensity', 100.0)
        self.declare_parameter('acquired_pulse_duration_s', 0.3)
        self.declare_parameter('stable_vibration_intensity', 30.0)
        self.declare_parameter('slip_pulse_intensity', 80.0)
        self.declare_parameter('slip_pulse_period_s', 0.1)
        self.declare_parameter('overforce_alarm_intensity', 100.0)
        self.declare_parameter('overforce_threshold', 500.0)
        self.declare_parameter('stale_pulse_intensity', 20.0)
        self.declare_parameter('stale_pulse_period_s', 1.0)

        self._pipeline_state_topic = self.get_parameter('pipeline_state_topic').value
        self._release_buzz_duration = self.get_parameter('release_buzz_duration_s').value
        self._release_buzz_intensity = self.get_parameter('release_buzz_intensity').value
        self._force_max_raw = self.get_parameter('force_max_raw').value
        self._force_stale_timeout = self.get_parameter('force_stale_timeout_s').value
        self._acquired_pulse_intensity = self.get_parameter('acquired_pulse_intensity').value
        self._acquired_pulse_duration = self.get_parameter('acquired_pulse_duration_s').value
        self._stable_vibration_intensity = self.get_parameter('stable_vibration_intensity').value
        self._slip_pulse_intensity = self.get_parameter('slip_pulse_intensity').value
        self._slip_pulse_period = self.get_parameter('slip_pulse_period_s').value
        self._overforce_alarm_intensity = self.get_parameter('overforce_alarm_intensity').value
        self._overforce_threshold = self.get_parameter('overforce_threshold').value
        self._stale_pulse_intensity = self.get_parameter('stale_pulse_intensity').value
        self._stale_pulse_period = self.get_parameter('stale_pulse_period_s').value

        # ── State ─────────────────────────────────────────────────────────
        self._wrist_angle: float = 0.0
        self._force_intensities: list[float] = [0.0] * 4  # motors 4-7
        self._pipeline_state: int = _IDLE
        self._prev_pipeline_state: int = _IDLE
        self._force_data_stamp: float = 0.0
        self._release_buzz_end: float = 0.0
        self._acquired_pulse_end: float = 0.0
        self._connection_ok: bool = True
        self._force_stable: bool = False
        self._slip_detected: bool = False
        self._overforce: bool = False

        # ── Publishers ────────────────────────────────────────────────────
        self._motors_pub = self.create_publisher(Float32MultiArray, '/haptic_band/motors', 10)

        # ── Subscriptions ─────────────────────────────────────────────────
        self.create_subscription(Float64MultiArray, '/wrist/state', self._on_wrist, 10)
        self.create_subscription(
            Int32, self._pipeline_state_topic, self._on_pipeline_state, 10)
        self.create_subscription(
            Bool, '/haptic_band/connection_status', self._on_connection_status, 10)

        # Force feedback (optional dep, graceful fallback)
        try:
            from mia_hand_msgs.msg import ForceData, ForceControllerStatus
            self.create_subscription(
                ForceData, 'data_streams/fingers/forces/data', self._on_force, 10)
            self.create_subscription(
                ForceControllerStatus, '/force_controller/status', self._on_force_status, 10)
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

        # Detect transition into GRASPING -> start acquired pulse
        if (self._pipeline_state == _GRASPING
                and self._prev_pipeline_state != _GRASPING):
            self._acquired_pulse_end = (
                self.get_clock().now().nanoseconds / 1e9
                + self._acquired_pulse_duration)
            self.get_logger().info('Acquired grasp — haptic pulse triggered')

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
            0.0,   # motor 7 normal by default
        ]
        max_force = max(abs(msg.thumb_nfor), abs(msg.index_nfor), abs(msg.mrl_nfor))
        self._overforce = max_force > self._overforce_threshold
        self._force_data_stamp = self.get_clock().now().nanoseconds / 1e9

    def _on_force_status(self, msg) -> None:
        self._force_stable = msg.force_stable
        self._slip_detected = msg.slip_detected

    def _on_connection_status(self, msg: Bool) -> None:
        self._connection_ok = msg.data
        if not self._connection_ok:
            self.get_logger().warn('Haptic band reported disconnected')

    @staticmethod
    def _pulse(now: float, period: float, intensity: float) -> float:
        """Square wave pulse: on for half period, off for half period."""
        return intensity if int(now / (period / 2.0)) % 2 == 0 else 0.0

    @staticmethod
    def _gentle_vibration(now: float, intensity: float) -> float:
        """Gentle 4 Hz vibration between 60% and 100% of intensity."""
        phase = int(now * 4.0) % 2
        return intensity * (1.0 if phase == 0 else 0.6)

    def _publish(self):
        ring = _wrist_ring(self._wrist_angle)
        now = self.get_clock().now().nanoseconds / 1e9

        # Decide force motor output based on priority
        if not self._connection_ok:
            # DISCONNECTED: zero all motors
            force = [0.0] * 4
            self.get_logger().warn('Haptic band disconnected — zeroing motors')
        elif now < self._release_buzz_end:
            # RELEASE: release buzz active
            force = [self._release_buzz_intensity] * 4
        elif self._overforce:
            # OVERFORCE: intense alarm vibration
            force = [self._overforce_alarm_intensity] * 4
            force[3] = 100.0  # motor 7 = 100
        elif self._slip_detected:
            # SLIP: rapid pulsing
            p = self._pulse(now, self._slip_pulse_period, self._slip_pulse_intensity)
            force = [p] * 3 + [50.0]  # motor 7 = 50
        elif now < self._acquired_pulse_end:
            # ACQUIRED: brief strong pulse on all force motors
            force = [self._acquired_pulse_intensity] * 4
        elif now - self._force_data_stamp > self._force_stale_timeout:
            # STALE: slow intermittent pulse
            p = self._pulse(now, self._stale_pulse_period, self._stale_pulse_intensity)
            force = [p] * 4
        elif self._force_stable and self._pipeline_state == _HOLDING:
            # STABLE: gentle steady vibration during HOLDING
            v = self._gentle_vibration(now, self._stable_vibration_intensity)
            force = [v] * 4
        elif self._pipeline_state in (_GRASPING, _HOLDING):
            # Normal force feedback active only during GRASPING/HOLDING
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
