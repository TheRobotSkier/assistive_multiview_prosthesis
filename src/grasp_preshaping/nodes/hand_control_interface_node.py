#!/usr/bin/env python3
"""MIA Hand Control Interface — single ROS node for all hand commands.

Subscribes:
  /hand_control/preshape   Float64MultiArray  [wrist_deg, thumb, index, mrl]
                           NaN entries → use defaults from config YAML.
  /hand_control/close      Empty               trigger velocity-based force-aware closure.
  /hand_control/reset       Empty               return hand to resting (0.0 rad) pose.
  /joint_states             JointState          force/position feedback during closure.

Publishes:
  /group_pos_ff_controller/commands  Float64MultiArray   [thumb, index, mrl]
  /group_vel_ff_controller/commands  Float64MultiArray   [thumb_vel, index_vel, mrl_vel]
  /wrist/set_position                Float64MultiArray   [deg, accel]

Services used:
  /controller_manager/load_controller
  /controller_manager/switch_controller

Config: reads the same static_grasp_test.yaml used by make test-static-grasp
  plus a 'default_preshape' section for fallback values.
"""

import math
import os
import subprocess
import time
from pathlib import Path
from typing import List, Optional

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Float64MultiArray, Empty
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState

FINGER_JOINTS = ["j_thumb_fle", "j_index_fle", "j_mrl_fle"]
FINGER_COUNT = 3


def _nan_or(val, default):
    if val is None or (isinstance(val, float) and math.isnan(val)):
        return default
    return float(val)


# ── helper: ros2 CLI subprocess wrappers ──────────────────────────────────────

def _ros(*args, timeout=10, check=True):
    cmd = ["ros2"] + list(args)
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    if check and result.returncode != 0:
        raise RuntimeError(f"ros2 {' '.join(args)} failed: {result.stderr.strip()}")
    return result.stdout.strip()


def _load_controller(name: str):
    subprocess.run(
        ["ros2", "control", "load_controller", "--set-state", "inactive", name],
        capture_output=True, text=True,
    )


def _switch_controllers(activate: List[str], deactivate: List[str]):
    req = (
        "{"
        f"activate_controllers: [{', '.join(repr(c) for c in activate)}], "
        f"deactivate_controllers: [{', '.join(repr(c) for c in deactivate)}], "
        "strictness: 1, activate_asap: true, "
        "timeout: {sec: 10, nanosec: 0}}"
    )
    cmd = [
        "ros2", "service", "call", "/controller_manager/switch_controller",
        "controller_manager_msgs/srv/SwitchController", req,
    ]
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
    if result.returncode != 0 or "ok=True" not in result.stdout:
        req2 = req.replace("strictness: 1", "strictness: 2")
        cmd[-1] = req2
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
        if result.returncode != 0 or "ok=True" not in result.stdout:
            raise RuntimeError(f"Controller switch failed: {result.stdout.strip()}")


# ── the node ──────────────────────────────────────────────────────────────────

class HandControlInterfaceNode(Node):
    def __init__(self):
        super().__init__("hand_control_interface")

        # ---- parameters ----
        self.declare_parameter("config_file", "")
        self._load_config()

        # ---- publishers ----
        self._pos_pub = self.create_publisher(
            Float64MultiArray, "/group_pos_ff_controller/commands", 10)
        self._vel_pub = self.create_publisher(
            Float64MultiArray, "/group_vel_ff_controller/commands", 10)
        self._wrist_pub = self.create_publisher(
            Float64MultiArray, "/wrist/set_position", 10)

        # ---- subscriptions ----
        self.create_subscription(
            Float64MultiArray, "/hand_control/preshape",
            self._on_preshape, 10)
        self.create_subscription(
            Empty, "/hand_control/close",
            self._on_close, 10)
        self.create_subscription(
            Empty, "/hand_control/reset",
            self._on_reset, 10)
        self.create_subscription(
            JointState, "/joint_states",
            self._on_joint_states, 10)

        # ---- state ----
        self._positions = [0.0] * FINGER_COUNT
        self._efforts = [0.0] * FINGER_COUNT
        self._closing = False
        self._abort_close = False

        self.get_logger().info(
            "HandControlInterface ready — "
            f"force_thresholds={self._force_thresholds}  "
            f"stop_positions={self._stop_positions}"
        )

    # ── config ────────────────────────────────────────────────────────────────

    def _load_config(self):
        config_path = self.get_parameter("config_file").value or ""
        if not config_path:
            candidates = [
                Path("/prosthesis_ws") / "config" / "static_grasp_test.yaml",
                Path(__file__).resolve().parents[3] / "config" / "static_grasp_test.yaml",
            ]
            for c in candidates:
                if c.exists():
                    config_path = str(c)
                    break
        if not config_path:
            self.get_logger().warn("No config found; using hardcoded defaults.")
            cfg = {}
        else:
            self.get_logger().info(f"Loading config from {config_path}")
            cfg = yaml.safe_load(Path(config_path).read_text()) or {}

        # velocity ramp
        self._v_start = float(cfg.get("closing_velocity_start", 0.3))
        self._v_end = float(cfg.get("closing_velocity_end", 0.05))
        self._decay_steps = int(cfg.get("decay_steps", 10))
        self._step_interval = float(cfg.get("step_interval_s", 0.5))
        self._relaxed_wait = float(cfg.get("relaxed_wait_s", 2.0))

        # per-finger thresholds
        sp = cfg.get("stop_positions", {})
        ft = cfg.get("force_thresholds", {})
        self._stop_positions = [
            float(sp.get(n, 2.0)) for n in FINGER_JOINTS]
        self._force_thresholds = [
            float(ft.get(n, 400.0)) for n in FINGER_JOINTS]

        # default preshape
        dp = cfg.get("default_preshape", {})
        self._default_wrist_deg = float(dp.get("wrist_deg", 0.0))
        self._default_thumb = float(dp.get("thumb_closure", 0.3))
        self._default_index = float(dp.get("index_closure", 0.3))
        self._default_mrl = float(dp.get("mrl_closure", 0.3))

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _on_joint_states(self, msg: JointState):
        try:
            for i, jname in enumerate(FINGER_JOINTS):
                idx = msg.name.index(jname)
                self._positions[i] = float(msg.position[idx])
                if idx < len(msg.effort):
                    self._efforts[i] = float(msg.effort[idx])
        except ValueError:
            pass

    def _on_preshape(self, msg: Float64MultiArray):
        data = list(msg.data) if msg.data else []
        wrist_deg = _nan_or(data[0] if len(data) > 0 else None,
                            self._default_wrist_deg)
        thumb = _nan_or(data[1] if len(data) > 1 else None,
                        self._default_thumb)
        index = _nan_or(data[2] if len(data) > 2 else None,
                        self._default_index)
        mrl = _nan_or(data[3] if len(data) > 3 else None,
                       self._default_mrl)

        self.get_logger().info(
            f"Preshape: wrist={wrist_deg:.1f}°  "
            f"thumb={thumb:.3f}  index={index:.3f}  mrl={mrl:.3f}")

        # Cancel any running closure
        if self._closing:
            self._abort_close = True
            self._closing = False

        # Ensure position controller is active
        _switch_controllers(["group_pos_ff_controller"],
                           ["group_vel_ff_controller"])

        # Publish wrist
        wrist_msg = Float64MultiArray()
        wrist_msg.data = [wrist_deg, 180.0]  # 180 deg/s² acceleration
        self._wrist_pub.publish(wrist_msg)

        # Publish finger positions
        pos_msg = Float64MultiArray()
        pos_msg.data = [thumb, index, mrl]
        self._pos_pub.publish(pos_msg)

    def _on_close(self, _msg: Empty):
        if self._closing:
            self.get_logger().warn("Close requested but already closing")
            return
        self.get_logger().info("GRASP CLOSE — starting velocity-based closure")
        self._closing = True
        self._abort_close = False
        self._run_velocity_closure()

    def _on_reset(self, _msg: Empty):
        self.get_logger().info("RESET — returning to resting pose")
        self._abort_close = True
        self._closing = False

        # Zero velocity first
        vel_msg = Float64MultiArray()
        vel_msg.data = [0.0, 0.0, 0.0]
        self._vel_pub.publish(vel_msg)

        # Switch to position, command open
        _switch_controllers(["group_pos_ff_controller"],
                           ["group_vel_ff_controller"])

        pos_msg = Float64MultiArray()
        pos_msg.data = [0.0, 0.0, 0.0]
        self._pos_pub.publish(pos_msg)

        self.get_logger().info("Reset complete — hand at relaxed position")

    # ── velocity-based force-aware closure ────────────────────────────────────

    def _compute_ramp(self):
        if self._decay_steps <= 1:
            return [self._v_start]
        ramp = []
        for i in range(self._decay_steps):
            t = i / (self._decay_steps - 1)
            ramp.append(self._v_start + t * (self._v_end - self._v_start))
        return ramp

    def _run_velocity_closure(self):
        # Load and switch to velocity controller
        _load_controller("group_vel_ff_controller")
        _switch_controllers(["group_vel_ff_controller"],
                           ["group_pos_ff_controller"])

        ramp = self._compute_ramp()
        stop_reason = None

        # ---- velocity ramp ----
        for step_idx, vel in enumerate(ramp):
            if self._abort_close:
                stop_reason = "ABORTED"
                break

            self._publish_velocity(vel)
            self._sleep_spinning(self._step_interval)

            p = self._positions
            f = self._efforts

            self.get_logger().info(
                f"  step {step_idx+1:2d}/{self._decay_steps}  vel={vel:.3f}  "
                f"pos=[{p[0]:.3f} {p[1]:.3f} {p[2]:.3f}]  "
                f"force=[{f[0]:.0f} {f[1]:.0f} {f[2]:.0f}]"
            )

            stop_reason = self._check_stop_conditions(p, f)
            if stop_reason:
                break

        # ---- post-ramp: continue at v_end ----
        if stop_reason is None:
            self._publish_velocity(self._v_end)
            while not self._abort_close:
                self._sleep_spinning(self._step_interval)
                p = self._positions
                f = self._efforts

                self.get_logger().info(
                    f"  post-ramp  vel={self._v_end:.3f}  "
                    f"pos=[{p[0]:.3f} {p[1]:.3f} {p[2]:.3f}]  "
                    f"force=[{f[0]:.0f} {f[1]:.0f} {f[2]:.0f}]"
                )

                stop_reason = self._check_stop_conditions(p, f)
                if stop_reason:
                    break

        # ---- stop ----
        self._publish_velocity(0.0)

        # Switch back to position controller for holding
        _switch_controllers(["group_pos_ff_controller"],
                           ["group_vel_ff_controller"])

        self._closing = False
        self.get_logger().info(
            f"Closure complete — {stop_reason or 'unknown'}  "
            f"final pos=[{self._positions[0]:.3f} {self._positions[1]:.3f} {self._positions[2]:.3f}]  "
            f"final force=[{self._efforts[0]:.0f} {self._efforts[1]:.0f} {self._efforts[2]:.0f}]"
        )

    def _check_stop_conditions(self, p, f):
        for i, name in enumerate(FINGER_JOINTS):
            if f[i] >= self._force_thresholds[i]:
                return f"FORCE on {name} ({f[i]:.0f} >= {self._force_thresholds[i]})"
        for i, name in enumerate(FINGER_JOINTS):
            if p[i] >= self._stop_positions[i]:
                return f"STOP_POS on {name} ({p[i]:.3f} >= {self._stop_positions[i]:.2f})"
        return None

    def _publish_velocity(self, v):
        msg = Float64MultiArray()
        msg.data = [v, v, v]
        self._vel_pub.publish(msg)

    def _sleep_spinning(self, duration):
        deadline = time.time() + duration
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)


def main(args=None):
    rclpy.init(args=args)
    node = HandControlInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
