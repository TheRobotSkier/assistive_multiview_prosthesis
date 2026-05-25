#!/usr/bin/env python3
"""Force Controller Node for Mia Hand — position-based PI force regulation.

Handoff Protocol (Proximity Controller -> Force Controller):
    When the pipeline transitions from APPROACHING to GRASPING:
      1. The proximity controller has already sent full closure positions to
         the ``*_pos_ff_controller/commands`` topics.
      2. This node wakes up, activates force streaming via the Mia Hand driver
         service, and reads the current joint positions from ``/joint_states``.
      3. It uses those positions as its baseline and begins making incremental
         adjustments based on force sensor feedback.
      4. No explicit handshake is needed — the proximity controller stops
         publishing during GRASPING/HOLDING (pipeline state gate), so only
         this node commands the joints during those states.

Subscriptions:
    data_streams/fingers/forces/data  (mia_hand_msgs/ForceData)
        Real force sensor data: 6 int32 fields (3 normal + 3 tangential).
    /pipeline/state  (std_msgs/Int32)
        Pipeline state machine state.
    /joint_states  (sensor_msgs/JointState)
        Current joint positions for incremental adjustments.

Service clients:
    data_streams/fingers/forces/switch  (std_srvs/SetBool)
        Activate/deactivate force streaming on the Mia Hand driver.

Publishers:
    /thumb_pos_ff_controller/commands  (std_msgs/Float64MultiArray)
    /index_pos_ff_controller/commands  (std_msgs/Float64MultiArray)
    /mrl_pos_ff_controller/commands    (std_msgs/Float64MultiArray)
    /force_controller/status           (mia_hand_msgs/ForceControllerStatus)

Parameters:
    target_force_min       (double) — Min target normal force in raw sensor units
    target_force_max       (double) — Max target normal force in raw sensor units
    kp                     (double) — Proportional gain (rad / raw force unit)
    ki                     (double) — Integral gain
    update_rate_hz         (double) — Control loop frequency
    max_position_step      (double) — Max position change per tick (rad)
    integral_limit         (double) — Anti-windup: max integral accumulator
    stability_window       (double) — Seconds within tolerance before stable
    stability_tolerance    (double) — Raw force units tolerance for stability
    slip_threshold         (double) — Tangential force rate-of-change threshold
    max_force_emergency    (double) — Emergency release threshold (raw units)
    stale_data_timeout_s   (double) — Seconds without force data before warning
    emergency_backoff_factor (double) — Multiplier for max_step on emergency release
    force_filter_window    (int)    — Moving-average window for force readings
    force_data_topic       (str)    — Force sensor data topic
    force_stream_switch_service (str) — Service to activate/deactivate force streaming
    pipeline_state_topic   (str)    — Pipeline state topic
    joint_states_topic     (str)    — Joint states topic
    thumb_cmd_topic        (str)    — Thumb position command topic
    index_cmd_topic        (str)    — Index position command topic
    mrl_cmd_topic          (str)    — MRL position command topic
    force_status_topic     (str)    — Force controller status publish topic
"""

from __future__ import annotations

import time
from collections import deque
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from mia_hand_msgs.msg import ForceData, ForceControllerStatus
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray, Int32
from std_srvs.srv import SetBool


# Pipeline states (must match pipeline_manager)
STATE_IDLE = 0
STATE_TWISTING = 1
STATE_SEGMENTING = 2
STATE_PLANNING = 3
STATE_APPROACHING = 4
STATE_GRASPING = 5
STATE_HOLDING = 6
STATE_VOLITIONAL = 7
STATE_RELEASING = 8

STATE_NAMES = {
    STATE_IDLE: "IDLE",
    STATE_TWISTING: "TWISTING",
    STATE_SEGMENTING: "SEGMENTING",
    STATE_PLANNING: "PLANNING",
    STATE_APPROACHING: "APPROACHING",
    STATE_GRASPING: "GRASPING",
    STATE_HOLDING: "HOLDING",
    STATE_VOLITIONAL: "VOLITIONAL",
    STATE_RELEASING: "RELEASING",
}

# Joint names tracked from /joint_states
JOINT_NAMES = ["j_thumb_fle", "j_index_fle", "j_mrl_fle"]


class ForceControllerNode(Node):
    """PI force controller that regulates fingertip normal forces."""

    def __init__(self) -> None:
        super().__init__("force_controller")

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter("target_force_min", 50.0)
        self.declare_parameter("target_force_max", 200.0)
        self.declare_parameter("kp", 0.01)
        self.declare_parameter("ki", 0.001)
        self.declare_parameter("update_rate_hz", 10.0)
        self.declare_parameter("max_position_step", 0.05)
        self.declare_parameter("integral_limit", 50.0)
        self.declare_parameter("stability_window", 1.0)
        self.declare_parameter("stability_tolerance", 30.0)
        self.declare_parameter("slip_threshold", 100.0)
        self.declare_parameter("max_force_emergency", 500.0)
        self.declare_parameter("stale_data_timeout_s", 0.5)
        self.declare_parameter("emergency_backoff_factor", 2.0)
        self.declare_parameter("force_filter_window", 5)
        self.declare_parameter("manual_adjust_topic", "/force_controller/manual_adjust")
        self.declare_parameter("manual_adjust_step", 10.0)

        # Topic / service name parameters
        self.declare_parameter("force_data_topic", "data_streams/fingers/forces/data")
        self.declare_parameter("force_stream_switch_service", "data_streams/fingers/forces/switch")
        self.declare_parameter("pipeline_state_topic", "/pipeline/state")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("thumb_cmd_topic", "/thumb_pos_ff_controller/commands")
        self.declare_parameter("index_cmd_topic", "/index_pos_ff_controller/commands")
        self.declare_parameter("mrl_cmd_topic", "/mrl_pos_ff_controller/commands")
        self.declare_parameter("force_status_topic", "/force_controller/status")
        self.declare_parameter("joint_position_limits", [3.14, 3.14, 3.14])
        self.declare_parameter("joint_position_min", [0.0, 0.0, 0.0])

        self._target_min = self.get_parameter("target_force_min").value
        self._target_max = self.get_parameter("target_force_max").value
        self._target_mid = (self._target_min + self._target_max) / 2.0
        self._kp = self.get_parameter("kp").value
        self._ki = self.get_parameter("ki").value
        self._max_step = self.get_parameter("max_position_step").value
        self._stability_window = self.get_parameter("stability_window").value
        self._stability_tol = self.get_parameter("stability_tolerance").value
        self._slip_thresh = self.get_parameter("slip_threshold").value
        self._max_force_emerg = self.get_parameter("max_force_emergency").value
        self._filter_win = max(1, self.get_parameter("force_filter_window").value)
        self._integral_limit = self.get_parameter("integral_limit").value
        self._stale_timeout = self.get_parameter("stale_data_timeout_s").value
        self._emergency_backoff = self.get_parameter("emergency_backoff_factor").value
        rate_hz = self.get_parameter("update_rate_hz").value
        self._dt = 1.0 / rate_hz

        # Topic / service names
        self._force_data_topic = self.get_parameter("force_data_topic").value
        self._stream_switch_service = self.get_parameter("force_stream_switch_service").value
        self._pipeline_state_topic = self.get_parameter("pipeline_state_topic").value
        self._joint_states_topic = self.get_parameter("joint_states_topic").value
        self._thumb_cmd_topic = self.get_parameter("thumb_cmd_topic").value
        self._index_cmd_topic = self.get_parameter("index_cmd_topic").value
        self._mrl_cmd_topic = self.get_parameter("mrl_cmd_topic").value
        self._force_status_topic = self.get_parameter("force_status_topic").value
        self._joint_pos_limits = list(self.get_parameter("joint_position_limits").value)
        self._joint_pos_min = list(self.get_parameter("joint_position_min").value)
        self._manual_adjust_topic = self.get_parameter("manual_adjust_topic").value
        self._manual_adjust_step = self.get_parameter("manual_adjust_step").value

        # ── Internal state ────────────────────────────────────────────────
        self._pipeline_state: int = STATE_IDLE
        self._prev_pipeline_state: int = STATE_IDLE
        self._streaming_active: bool = False
        self._controller_active: bool = False

        # Current joint positions [thumb, index, mrl] in radians
        self._joint_pos: list[float] = [0.0, 0.0, 0.0]
        self._joint_pos_received: bool = False

        # Force data — raw readings
        self._normal_forces: list[float] = [0.0, 0.0, 0.0]
        self._tangential_forces: list[float] = [0.0, 0.0, 0.0]
        self._prev_tangential_forces: list[float] = [0.0, 0.0, 0.0]
        self._force_data_received: bool = False
        self._last_force_time: float = 0.0

        # Filtered force buffers (moving average per finger)
        self._normal_bufs: list[deque[float]] = [
            deque(maxlen=self._filter_win) for _ in range(3)
        ]
        self._tangential_bufs: list[deque[float]] = [
            deque(maxlen=self._filter_win) for _ in range(3)
        ]

        # PI controller state per finger
        self._integral: list[float] = [0.0, 0.0, 0.0]

        # Stability tracking
        self._stable_since: Optional[float] = None
        self._force_stable: bool = False
        self._slip_detected: bool = False

        # Manual force target adjustment (from volitional mode)
        self._manual_adjust_offset: float = 0.0
        self._target_base: float = self._target_mid  # stored for clarity

        # ── Service clients ───────────────────────────────────────────────
        self._stream_switch = self.create_client(
            SetBool, self._stream_switch_service
        )

        # ── Subscriptions ─────────────────────────────────────────────────
        self.create_subscription(
            ForceData,
            self._force_data_topic,
            self._on_forces,
            10,
        )
        self.create_subscription(
            Int32,
            self._pipeline_state_topic,
            self._on_pipeline_state,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_subscription(
            JointState,
            self._joint_states_topic,
            self._on_joint_states,
            10,
        )

        # Manual force target adjustment (from volitional mode)
        self.create_subscription(
            Float64,
            self._manual_adjust_topic,
            self._on_manual_adjust,
            10,
        )

        # ── Publishers ────────────────────────────────────────────────────
        self._thumb_pub = self.create_publisher(
            Float64MultiArray, self._thumb_cmd_topic, 10
        )
        self._index_pub = self.create_publisher(
            Float64MultiArray, self._index_cmd_topic, 10
        )
        self._mrl_pub = self.create_publisher(
            Float64MultiArray, self._mrl_cmd_topic, 10
        )
        self._status_pub = self.create_publisher(
            ForceControllerStatus, self._force_status_topic, 10
        )

        # ── Control timer ─────────────────────────────────────────────────
        self.create_timer(self._dt, self._control_tick)

        self.get_logger().info(
            f"Force controller started. Target range: [{self._target_min}, "
            f"{self._target_max}] raw units, kp={self._kp}, ki={self._ki}, "
            f"rate={rate_hz} Hz"
        )

    # ── Callbacks ──────────────────────────────────────────────────────────

    def _on_forces(self, msg: ForceData) -> None:
        """Store force sensor readings and update filter buffers."""
        raw_normal = [float(msg.thumb_nfor), float(msg.index_nfor), float(msg.mrl_nfor)]
        raw_tangential = [
            float(msg.thumb_tfor),
            float(msg.index_tfor),
            float(msg.mrl_tfor),
        ]

        # Push into filter buffers
        for i in range(3):
            self._normal_bufs[i].append(raw_normal[i])
            self._tangential_bufs[i].append(raw_tangential[i])

        # Compute filtered (moving average) values
        self._prev_tangential_forces = self._tangential_forces[:]
        self._normal_forces = [
            sum(self._normal_bufs[i]) / len(self._normal_bufs[i]) for i in range(3)
        ]
        self._tangential_forces = [
            sum(self._tangential_bufs[i]) / len(self._tangential_bufs[i])
            for i in range(3)
        ]

        self._force_data_received = True
        self._last_force_time = time.monotonic()

    def _on_pipeline_state(self, msg: Int32) -> None:
        """Track pipeline state and activate/deactivate controller."""
        new_state = msg.data
        if new_state == self._pipeline_state:
            return

        old_state = self._pipeline_state
        self._pipeline_state = new_state
        self.get_logger().info(
            f"Pipeline state: {STATE_NAMES.get(old_state, '?')} -> "
            f"{STATE_NAMES.get(new_state, '?')}"
        )

        # Activate on entering GRASPING
        if new_state == STATE_GRASPING and old_state in (
            STATE_APPROACHING,
            STATE_PLANNING,
        ):
            self._activate_controller()

        # Deactivate on leaving GRASPING/HOLDING/VOLITIONAL
        if old_state in (STATE_GRASPING, STATE_HOLDING, STATE_VOLITIONAL) and new_state not in (
            STATE_GRASPING,
            STATE_HOLDING,
            STATE_VOLITIONAL,
        ):
            self._deactivate_controller()

        # Full reset on RELEASING
        if new_state == STATE_RELEASING:
            self._reset_controller()

    def _on_joint_states(self, msg: JointState) -> None:
        """Track current joint positions for incremental adjustments."""
        for i, name in enumerate(JOINT_NAMES):
            try:
                idx = msg.name.index(name)
                self._joint_pos[i] = msg.position[idx]
            except (ValueError, IndexError):
                pass
        self._joint_pos_received = True

    def _on_manual_adjust(self, msg: Float64) -> None:
        """Adjust the target force midpoint by a delta.

        Positive values increase target force (tighten grasp).
        Negative values decrease target force (loosen grasp).
        The cumulative offset is clamped so target stays in [min, min+2*step].
        """
        self._manual_adjust_offset += msg.data
        # Clamp offset so effective target never exceeds reasonable bounds
        max_offset = self._target_max - self._target_base
        min_offset = self._target_min - self._target_base
        self._manual_adjust_offset = max(min_offset, min(max_offset, self._manual_adjust_offset))
        self.get_logger().info(
            f"Manual adjust: delta={msg.data:.1f}, cumulative_offset="
            f"{self._manual_adjust_offset:.1f}, effective_target="
            f"{self._target_base + self._manual_adjust_offset:.1f}"
        )

    # ── Controller lifecycle ───────────────────────────────────────────────

    def _activate_controller(self) -> None:
        """Activate force streaming and initialize controller state."""
        self.get_logger().info("Activating force controller...")
        self._controller_active = True
        self._force_stable = False
        self._slip_detected = False
        self._stable_since = None
        self._integral = [0.0, 0.0, 0.0]

        # Activate force streaming via service
        if not self._stream_switch.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                "Force stream switch service not available. "
                "Force data may not flow."
            )
            return

        if not self._streaming_active:
            req = SetBool.Request()
            req.data = True
            future = self._stream_switch.call_async(req)

            def on_response(fut: object) -> None:
                try:
                    result = fut.result()
                    if result.success:
                        self._streaming_active = True
                        self.get_logger().info("Force streaming activated.")
                    else:
                        self.get_logger().warn(
                            f"Force streaming activation failed: {result.message}"
                        )
                except Exception as exc:
                    self.get_logger().error(
                        f"Force streaming service error: {exc}"
                    )

            future.add_done_callback(on_response)

    def _deactivate_controller(self) -> None:
        """Deactivate force streaming and reset controller state."""
        self.get_logger().info("Deactivating force controller...")
        self._controller_active = False
        self._force_stable = False
        self._slip_detected = False
        self._stable_since = None
        self._integral = [0.0, 0.0, 0.0]

        # Deactivate force streaming
        if self._streaming_active and self._stream_switch.service_is_ready():
            req = SetBool.Request()
            req.data = False
            self._stream_switch.call_async(req)
            self._streaming_active = False
            self.get_logger().info("Force streaming deactivated.")

    def _reset_controller(self) -> None:
        """Full reset — called on RELEASING."""
        self._deactivate_controller()
        self._normal_forces = [0.0, 0.0, 0.0]
        self._tangential_forces = [0.0, 0.0, 0.0]
        self._prev_tangential_forces = [0.0, 0.0, 0.0]
        self._force_data_received = False
        for buf in self._normal_bufs:
            buf.clear()
        for buf in self._tangential_bufs:
            buf.clear()

    # ── Control loop ───────────────────────────────────────────────────────

    def _control_tick(self) -> None:
        """Main control loop — PI force regulation."""
        # Only regulate during GRASPING, HOLDING, and VOLITIONAL
        if self._pipeline_state not in (STATE_GRASPING, STATE_HOLDING, STATE_VOLITIONAL):
            self._publish_status()
            return

        if not self._controller_active:
            self._publish_status()
            return

        if not self._joint_pos_received:
            self.get_logger().warn(
                "Control tick skipped: valid joint feedback not available."
            )
            status = ForceControllerStatus()
            status.active = False
            status.current_normal_forces = self._normal_forces
            status.current_tangential_forces = self._tangential_forces
            status.force_errors = [0.0, 0.0, 0.0]
            status.force_stable = self._force_stable
            status.slip_detected = self._slip_detected
            status.state = STATE_NAMES.get(self._pipeline_state, "UNKNOWN")
            self._status_pub.publish(status)
            return

        # Check for stale force data (no data for >2 seconds)
        now = time.monotonic()
        if self._force_data_received and (now - self._last_force_time) > self._stale_timeout:
            self.get_logger().error(
                "Force data stale (no message for >2s). Skipping commands."
            )
            self._publish_status()
            return

        if not self._force_data_received:
            # No data yet — wait for it
            self._publish_status()
            return

        # Emergency force release
        if any(f > self._max_force_emerg for f in self._normal_forces):
            self.get_logger().warn(
                f"EMERGENCY: Force exceeded {self._max_force_emerg}! "
                f"Forces: {self._normal_forces} — backing off"
            )
            self._emergency_release()
            self._publish_status()
            return

        # Slip detection: tangential force rate-of-change
        self._slip_detected = False
        for i in range(3):
            tang_rate = abs(
                self._tangential_forces[i] - self._prev_tangential_forces[i]
            ) / self._dt
            if tang_rate > self._slip_thresh:
                self._slip_detected = True
                self.get_logger().warn(
                    f"Slip detected on finger {i}: tangential rate = {tang_rate:.1f}"
                )
                break

        # PI control for each finger
        new_positions = self._joint_pos[:]
        force_errors = [0.0, 0.0, 0.0]

        for i in range(3):
            # Compute error: positive = need more force (close more)
            error = self._target_base + self._manual_adjust_offset - self._normal_forces[i]
            force_errors[i] = error

            # Update integral with anti-windup
            self._integral[i] += error * self._dt
            self._integral[i] = max(
                -self._integral_limit, min(self._integral_limit, self._integral[i])
            )

            # PI output
            adjustment = self._kp * error + self._ki * self._integral[i]

            # Slip response: if slip detected, close more aggressively
            if self._slip_detected:
                adjustment += self._max_step

            # Clamp to max step
            adjustment = max(-self._max_step, min(self._max_step, adjustment))

            # Closing = positive position direction
            new_positions[i] = self._joint_pos[i] + adjustment

        # Clamp to joint limits
        for i in range(3):
            new_positions[i] = max(
                self._joint_pos_min[i],
                min(self._joint_pos_limits[i], new_positions[i]),
            )

        # Publish position commands
        self._publish_position_commands(new_positions)

        # Check stability
        self._update_stability(force_errors)

        # Publish status
        self._publish_status(force_errors=force_errors)

    def _emergency_release(self) -> None:
        """Back off all fingers by 2x max step to release force."""
        new_positions = [
            self._joint_pos[i] - self._emergency_backoff * self._max_step for i in range(3)
        ]
        self._publish_position_commands(new_positions)

    def _update_stability(self, force_errors: list[float]) -> None:
        """Track whether forces have been within tolerance for stability_window."""
        now = time.monotonic()
        all_within = all(abs(e) <= self._stability_tol for e in force_errors)

        if all_within:
            if self._stable_since is None:
                self._stable_since = now
            elif (now - self._stable_since) >= self._stability_window:
                if not self._force_stable:
                    self.get_logger().info("Forces stabilized.")
                self._force_stable = True
        else:
            self._stable_since = None
            self._force_stable = False

    # ── Publishing helpers ─────────────────────────────────────────────────

    def _publish_position_commands(self, positions: list[float]) -> None:
        """Publish position commands to the three ForwardCommandController topics."""
        for i, (pub, pos) in enumerate(zip(
            [self._thumb_pub, self._index_pub, self._mrl_pub], positions
        )):
            assert self._joint_pos_min[i] <= pos <= self._joint_pos_limits[i], (
                f"Position for joint {i} out of bounds: {pos} not in "
                f"[{self._joint_pos_min[i]}, {self._joint_pos_limits[i]}]"
            )
            msg = Float64MultiArray()
            msg.data = [pos]
            pub.publish(msg)

    def _publish_status(
        self, force_errors: Optional[list[float]] = None
    ) -> None:
        """Publish ForceControllerStatus for pipeline manager."""
        status = ForceControllerStatus()
        status.active = self._controller_active
        status.current_normal_forces = self._normal_forces
        status.current_tangential_forces = self._tangential_forces
        status.force_errors = force_errors if force_errors else [0.0, 0.0, 0.0]
        status.force_stable = self._force_stable
        status.slip_detected = self._slip_detected
        status.state = STATE_NAMES.get(self._pipeline_state, "UNKNOWN")
        self._status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = ForceControllerNode()
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
