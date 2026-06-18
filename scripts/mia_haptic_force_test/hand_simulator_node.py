#!/usr/bin/env python3
"""Hand/wrist simulator backend for offline testing.

Publishes the raw Mia hardware streams the rest of the stack already
knows how to read, plus a ``/wrist/state`` topic so the supervisor's
``_wrist_target_complete()`` gate does not fall back to
``assume_target_on_timeout`` after the 10s default timeout.

When ``mock_hardware:=true`` is passed to the launch file, the
``force_input_node`` is also pointed at ``/hand_sim/joint_states`` and
``/hand_sim/forces`` (the simulator's convenience passthrough) so the
controller's effort fallback has real data.

Subscribes to:
    /group_vel_ff_controller/commands   Float64MultiArray
    /group_pos_ff_controller/commands   Float64MultiArray
    /wrist/set_position                 Float64MultiArray [deg, accel]

Publishes:
    data_streams/fingers/forces/data    ForceData
    data_streams/motors/positions/data  MotorData
    data_streams/motors/speeds/data     MotorData
    data_streams/motors/currents/data   MotorData
    data_streams/joints/positions/data  JointData
    data_streams/joints/speeds/data     JointData
    /hand_sim/joint_states              JointState
    /hand_sim/forces                    Float32MultiArray
    /wrist/state                        Float64MultiArray [deg, vel]
"""

from __future__ import annotations

import argparse
import os
import signal
import sys
import threading
import time
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Float64MultiArray

from mia_hand_msgs.msg import ForceData, JointData, MotorData

from scripts.mia_haptic_force_test.common.constants import (
    FINGER_COUNT,
    FINGER_JOINTS,
    TOPIC_HW_FINGER_FORCES,
    TOPIC_HW_JOINT_POSITIONS,
    TOPIC_HW_JOINT_SPEEDS,
    TOPIC_HW_MOTOR_CURRENTS,
    TOPIC_HW_MOTOR_POSITIONS,
    TOPIC_HW_MOTOR_SPEEDS,
)
from scripts.mia_haptic_force_test.common.hand_simulation import (
    HandSimState,
    default_sim_config,
    sim_config_from,
    step_hand_simulation,
)

_REPO_ROOT: str = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if __name__ == "__main__" and __package__ is None and _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)


# ── Scaling constants for raw hardware message fields ─────────────────────

_ENCODER_TICKS_PER_RAD: float = 1000.0  # arbitrary scale; matches legacy
_CURRENT_SCALE_N: float = 10.0           # mA per N of normal force
_MAX_ENCODER_TICKS: int = 2**31 - 1


def _clamp_int(value: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, int(value)))


def _rad_to_ticks(rad: float) -> int:
    ticks = int(round(rad * _ENCODER_TICKS_PER_RAD))
    return _clamp_int(ticks, -_MAX_ENCODER_TICKS, _MAX_ENCODER_TICKS)


def _rad_s_to_ticks(rad_s: float) -> int:
    return _rad_to_ticks(rad_s)


def _force_to_current(force_n: float) -> int:
    return _clamp_int(int(round(force_n * _CURRENT_SCALE_N)), 0, 2**15 - 1)


class HandSimulatorNode(Node):
    """Deterministic hand/wrist simulator for offline tests."""

    def __init__(self, config_path: Optional[str] = None) -> None:
        super().__init__("hand_simulator_node")
        self._config_path = config_path or os.path.join(
            _REPO_ROOT, "config", "mia_haptic_force_test.yaml"
        )
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("dt_s", 0.01)
        self.declare_parameter("wrist_topic", "/wrist/state")
        self.declare_parameter("sim_joint_states_topic", "/hand_sim/joint_states")
        self.declare_parameter("sim_forces_topic", "/hand_sim/forces")

        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._dt = float(self.get_parameter("dt_s").value) or (1.0 / max(rate_hz, 1.0))
        self._period = 1.0 / max(rate_hz, 1.0)
        self._wrist_topic = str(self.get_parameter("wrist_topic").value)
        self._sim_joint_states_topic = str(
            self.get_parameter("sim_joint_states_topic").value
        )
        self._sim_forces_topic = str(self.get_parameter("sim_forces_topic").value)

        # ── Load simulator config from YAML ─────────────────────────────
        cfg: dict[str, Any] = {}
        try:
            import yaml

            with open(self._config_path) as f:
                raw = yaml.safe_load(f) or {}
            cfg = sim_config_from(raw)
        except Exception as exc:
            self.get_logger().warn(
                f"Config load failed ({self._config_path}): {exc}; using defaults"
            )
            cfg = default_sim_config()
        self._cfg = cfg

        # ── Sim state ───────────────────────────────────────────────────
        self._lock = threading.Lock()
        self._state = HandSimState(
            finger_positions_rad=cfg["open_rad"],
            wrist_position_deg=cfg["wrist_open_deg"],
        )
        self._finger_vel_cmd: tuple[float, ...] = (0.0, 0.0, 0.0)
        self._finger_pos_cmd: tuple[float, ...] = cfg["open_rad"]
        self._wrist_target_deg: Optional[float] = None
        self._wrist_accel_deg_s2: float = float(cfg["wrist_acceleration_deg_s2"])

        # ── Publishers ──────────────────────────────────────────────────
        self._forces_pub = self.create_publisher(
            ForceData, TOPIC_HW_FINGER_FORCES, 10
        )
        self._motor_pos_pub = self.create_publisher(
            MotorData, TOPIC_HW_MOTOR_POSITIONS, 10
        )
        self._motor_speed_pub = self.create_publisher(
            MotorData, TOPIC_HW_MOTOR_SPEEDS, 10
        )
        self._motor_current_pub = self.create_publisher(
            MotorData, TOPIC_HW_MOTOR_CURRENTS, 10
        )
        self._joint_pos_pub = self.create_publisher(
            JointData, TOPIC_HW_JOINT_POSITIONS, 10
        )
        self._joint_speed_pub = self.create_publisher(
            JointData, TOPIC_HW_JOINT_SPEEDS, 10
        )
        self._joint_effort_pub = self.create_publisher(
            JointData, "data_streams/joints/efforts/data", 10
        )
        self._wrist_state_pub = self.create_publisher(
            Float64MultiArray, self._wrist_topic, 10
        )
        self._sim_joint_states_pub = self.create_publisher(
            JointState, self._sim_joint_states_topic, 10
        )
        self._sim_forces_pub = self.create_publisher(
            Float32MultiArray, self._sim_forces_topic, 10
        )

        # ── Subscriptions ───────────────────────────────────────────────
        self.create_subscription(
            Float64MultiArray,
            "/group_vel_ff_controller/commands",
            self._on_vel_cmd,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "/group_pos_ff_controller/commands",
            self._on_pos_cmd,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "/wrist/set_position",
            self._on_wrist_cmd,
            10,
        )

        # ── Background simulation thread ─────────────────────────────────
        self._running = True
        self._thread = threading.Thread(
            target=self._loop, daemon=True, name="hand_simulator_loop"
        )
        self._thread.start()
        self.get_logger().info(
            f"HandSimulatorNode started: publish_rate_hz={rate_hz}, dt={self._dt}s"
        )

    # ── Command callbacks ────────────────────────────────────────────────

    def _on_vel_cmd(self, msg: Float64MultiArray) -> None:
        with self._lock:
            cmds = list(msg.data[:FINGER_COUNT]) + [0.0] * (FINGER_COUNT - len(msg.data))
            self._finger_vel_cmd = tuple(float(c) for c in cmds)

    def _on_pos_cmd(self, msg: Float64MultiArray) -> None:
        with self._lock:
            cmds = list(msg.data[:FINGER_COUNT]) + [0.0] * (FINGER_COUNT - len(msg.data))
            self._finger_pos_cmd = tuple(float(c) for c in cmds)

    def _on_wrist_cmd(self, msg: Float64MultiArray) -> None:
        with self._lock:
            if not msg.data:
                self._wrist_target_deg = None
                return
            self._wrist_target_deg = float(msg.data[0])
            if len(msg.data) >= 2:
                self._wrist_accel_deg_s2 = float(msg.data[1])

    # ── Main loop ────────────────────────────────────────────────────────

    def _step(self) -> None:
        with self._lock:
            vel_cmd = self._finger_vel_cmd
            pos_cmd = self._finger_pos_cmd
            wrist_target = self._wrist_target_deg
            wrist_accel = self._wrist_accel_deg_s2
        new_state = step_hand_simulation(
            self._state,
            finger_vel_cmd=vel_cmd,
            finger_pos_cmd=pos_cmd,
            wrist_target_deg=wrist_target,
            wrist_acceleration_deg_s2=wrist_accel,
            dt_s=self._dt,
            cfg=self._cfg,
        )
        with self._lock:
            self._state = new_state

    def _publish(self) -> None:
        with self._lock:
            state = self._state
        now = self.get_clock().now().to_msg()

        # ── Raw hardware streams ────────────────────────────────────────
        forces = state.finger_forces_n
        velocities = state.finger_velocities_rad_s
        positions = state.finger_positions_rad

        force_msg = ForceData(
            thumb_nfor=int(round(forces[0])),
            index_nfor=int(round(forces[1])),
            mrl_nfor=int(round(forces[2])),
            thumb_tfor=0,
            index_tfor=0,
            mrl_tfor=0,
        )
        self._forces_pub.publish(force_msg)

        motor_pos = MotorData(
            thumb_data=_rad_to_ticks(positions[0]),
            index_data=_rad_to_ticks(positions[1]),
            mrl_data=_rad_to_ticks(positions[2]),
        )
        self._motor_pos_pub.publish(motor_pos)

        motor_speed = MotorData(
            thumb_data=_rad_s_to_ticks(velocities[0]),
            index_data=_rad_s_to_ticks(velocities[1]),
            mrl_data=_rad_s_to_ticks(velocities[2]),
        )
        self._motor_speed_pub.publish(motor_speed)

        motor_current = MotorData(
            thumb_data=_force_to_current(forces[0]),
            index_data=_force_to_current(forces[1]),
            mrl_data=_force_to_current(forces[2]),
        )
        self._motor_current_pub.publish(motor_current)

        joint_pos = JointData(
            thumb_data=float(positions[0]),
            index_data=float(positions[1]),
            mrl_data=float(positions[2]),
        )
        self._joint_pos_pub.publish(joint_pos)

        joint_speed = JointData(
            thumb_data=float(velocities[0]),
            index_data=float(velocities[1]),
            mrl_data=float(velocities[2]),
        )
        self._joint_speed_pub.publish(joint_speed)

        joint_effort = JointData(
            thumb_data=float(forces[0]),
            index_data=float(forces[1]),
            mrl_data=float(forces[2]),
        )
        self._joint_effort_pub.publish(joint_effort)

        # ── /wrist/state ────────────────────────────────────────────────
        wrist_msg = Float64MultiArray()
        wrist_msg.data = [float(state.wrist_position_deg), float(state.wrist_velocity_deg_s)]
        self._wrist_state_pub.publish(wrist_msg)

        # ── /hand_sim/joint_states ──────────────────────────────────────
        js = JointState()
        js.header.stamp = now
        js.name = list(FINGER_JOINTS)
        js.position = [float(p) for p in positions]
        js.velocity = [float(v) for v in velocities]
        js.effort = [float(f) for f in forces]
        self._sim_joint_states_pub.publish(js)

        # ── /hand_sim/forces ────────────────────────────────────────────
        sim_forces = Float32MultiArray()
        sim_forces.data = [float(f) for f in forces]
        self._sim_forces_pub.publish(sim_forces)

    def _loop(self) -> None:
        next_time = time.monotonic()
        while rclpy.ok() and self._running:
            t0 = time.monotonic()
            self._step()
            self._publish()
            next_time += self._period
            remaining = next_time - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)
            else:
                # Catch up: skip ahead to avoid busy-looping.
                next_time = time.monotonic()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def destroy_node(self) -> None:
        self.stop()
        super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--config-path", default=None)
    known, remaining = parser.parse_known_args(args or [])
    rclpy.init(args=remaining)
    node = HandSimulatorNode(config_path=known.config_path)

    def _signal_handler(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _signal_handler)

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
