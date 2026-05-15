#!/usr/bin/env python3
"""Static grasp test with velocity-based force-aware closure.

Launches ros2_control, resets the hand, then closes fingers using a velocity
ramp (start speed → end speed over N discrete steps). At every step, forces
and positions are checked. If any finger exceeds its force threshold or
stop position, ALL fingers are stopped immediately.
"""
import os
import sys
import time
import math
import subprocess
from pathlib import Path

import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

FINGER_JOINTS = ["j_thumb_fle", "j_index_fle", "j_mrl_fle"]
FINGER_COUNT = 3


def ros(*args, timeout=10, check=True):
    """Run a ros2 CLI command and return stdout, or raise on failure."""
    cmd = ["ros2"] + list(args)
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    if check and result.returncode != 0:
        raise RuntimeError(f"ros2 {' '.join(args)} failed: {result.stderr.strip()}")
    return result.stdout.strip()


def wait_for_controller_manager(timeout_s=30):
    """Block until /controller_manager/list_controllers is available."""
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            out = ros("service", "list", timeout=5)
            if "/controller_manager/list_controllers" in out:
                return True
        except Exception:
            pass
        time.sleep(1)
    return False


def load_and_switch_controllers(activate, deactivate):
    """Load a controller if needed, then switch using ros2 CLI."""
    # First ensure the velocity controller is loaded and configured
    for ctrl in activate:
        result = subprocess.run(
            ["ros2", "control", "load_controller", "--set-state", "inactive", ctrl],
            capture_output=True, text=True,
        )
        # load_controller succeeds even if already loaded, ignore errors about "already loaded"

    # Now switch
    req = (
        "{"
        f"activate_controllers: [{', '.join(repr(c) for c in activate)}], "
        f"deactivate_controllers: [{', '.join(repr(c) for c in deactivate)}], "
        "strictness: 2, "               # STRICT
        "activate_asap: true, "
        "timeout: {sec: 10, nanosec: 0}"
        "}"
    )
    cmd = [
        "ros2", "service", "call", "/controller_manager/switch_controller",
        "controller_manager_msgs/srv/SwitchController", req,
    ]
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
    if result.returncode != 0:
        raise RuntimeError(f"Controller switch failed: {result.stderr.strip()}")
    if "ok=True" not in result.stdout:
        # Try BEST_EFFORT as fallback
        req2 = req.replace("strictness: 2", "strictness: 1")
        cmd[-1] = req2
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
        if "ok=True" not in result.stdout:
            raise RuntimeError(f"Controller switch not ok: {result.stdout.strip()}")
    return True

def reset_to_position_control():
    """Switch back to position control for safety."""
    subprocess.run(
        ["ros2", "control", "set_controller_state", "group_vel_ff_controller", "inactive"],
        capture_output=True, text=True,
    )
    subprocess.run(
        ["ros2", "control", "set_controller_state", "group_pos_ff_controller", "active"],
        capture_output=True, text=True,
    )


class GraspTestNode(Node):
    """Minimal ROS node that publishes velocity commands and reads joint states."""

    def __init__(self):
        super().__init__("grasp_test_node")

        self._vel_pub = self.create_publisher(
            Float64MultiArray, "/group_vel_ff_controller/commands", 10
        )
        self._pos_pub = self.create_publisher(
            Float64MultiArray, "/group_pos_ff_controller/commands", 10
        )

        self._positions = [0.0] * FINGER_COUNT
        self._efforts = [0.0] * FINGER_COUNT
        self._got_data = False

        self.create_subscription(
            JointState, "/joint_states", self._on_joint_states, 10
        )

    def _on_joint_states(self, msg: JointState):
        try:
            for i, jname in enumerate(FINGER_JOINTS):
                idx = msg.name.index(jname)
                self._positions[i] = float(msg.position[idx])
                if idx < len(msg.effort):
                    self._efforts[i] = float(msg.effort[idx])
            self._got_data = True
        except ValueError:
            pass

    def publish_velocity(self, v: float):
        msg = Float64MultiArray()
        msg.data = [v, v, v]
        self._vel_pub.publish(msg)

    def publish_position(self, pos: float):
        msg = Float64MultiArray()
        msg.data = [pos, pos, pos]
        self._pos_pub.publish(msg)

    def spin_once(self):
        rclpy.spin_once(self, timeout_sec=0.05)


def load_config(path: str) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def compute_velocity_ramp(v_start, v_end, decay_steps):
    """Return list of velocities, one per step."""
    if decay_steps <= 1:
        return [v_start]
    ramp = []
    for i in range(decay_steps):
        t = i / (decay_steps - 1)
        ramp.append(v_start + t * (v_end - v_start))
    return ramp


def main():
    config_path = os.environ.get(
        "GRASP_TEST_CONFIG", "/prosthesis_ws/config/static_grasp_test.yaml"
    )
    config = load_config(config_path)

    v_start = float(config["closing_velocity_start"])
    v_end = float(config["closing_velocity_end"])
    decay_steps = int(config["decay_steps"])
    step_interval = float(config["step_interval_s"])
    relaxed_wait = float(config["relaxed_wait_s"])

    sp = config["stop_positions"]
    ft = config["force_thresholds"]
    stop_positions = [float(sp[n]) for n in FINGER_JOINTS]
    force_thresholds = [float(ft[n]) for n in FINGER_JOINTS]

    ramp = compute_velocity_ramp(v_start, v_end, decay_steps)

    print("=" * 50)
    print("  Velocity-Based Force-Aware Static Grasp Test")
    print("=" * 50)
    print(f"Velocity ramp : {v_start:.3f} → {v_end:.3f} rad/s over {decay_steps} steps")
    print(f"Step interval : {step_interval}s  (total ramp: {decay_steps * step_interval:.1f}s)")
    print(f"Stop positions: thumb={stop_positions[0]:.2f}  index={stop_positions[1]:.2f}  mrl={stop_positions[2]:.2f} rad")
    print(f"Force thresholds: thumb={force_thresholds[0]}  index={force_thresholds[1]}  mrl={force_thresholds[2]} raw ADC")
    print(f"Relaxed wait : {relaxed_wait}s")
    print()

    # ---- Wait for controller_manager ----
    print("[1] Waiting for controller_manager...")
    if not wait_for_controller_manager():
        print("FATAL: controller_manager not available")
        return 1
    print("     Ready.")

    # ---- Init rclpy ----
    rclpy.init()
    node = GraspTestNode()

    # ---- Reset to relaxed ----
    print(f"[2] Resetting hand to relaxed position (0.0 rad)...")
    node.publish_position(0.0)
    time.sleep(1.0)
    node.spin_once()
    print(f"     Positions: {[f'{p:.3f}' for p in node._positions]}")
    print(f"     Idle forces: {[f'{e:.0f}' for e in node._efforts]}")
    print(f"     Waiting {relaxed_wait}s...")
    for _ in range(int(relaxed_wait)):
        time.sleep(1)
        node.spin_once()

    # ---- Switch to velocity controller ----
    print("[3] Switching to velocity controller (group_vel_ff_controller)...")
    load_and_switch_controllers(["group_vel_ff_controller"], ["group_pos_ff_controller"])
    time.sleep(1.0)
    print("     Velocity controller active.")

    # ---- Velocity ramp closure ----
    print("[4] Closing with velocity ramp — monitoring forces and positions...")
    print()

    stop_reason = None
    current_vel = 0.0

    # Ramp phase
    for step_idx, vel in enumerate(ramp):
        current_vel = vel
        node.publish_velocity(vel)
        node.spin_once()

        p = node._positions
        f = node._efforts

        # Check forces
        for i, name in enumerate(FINGER_JOINTS):
            if f[i] >= force_thresholds[i]:
                stop_reason = f"FORCE CONTACT on {name} ({f[i]:.0f} >= {force_thresholds[i]})"
                break

        # Check positions
        if stop_reason is None:
            for i, name in enumerate(FINGER_JOINTS):
                if p[i] >= stop_positions[i]:
                    stop_reason = f"STOP POSITION reached on {name} ({p[i]:.3f} >= {stop_positions[i]:.2f})"
                    break

        print(
            f"  step {step_idx+1:2d}/{decay_steps}  vel={vel:.3f}  "
            f"pos=[{p[0]:.3f} {p[1]:.3f} {p[2]:.3f}]  "
            f"force=[{f[0]:.0f} {f[1]:.0f} {f[2]:.0f}]"
        )

        if stop_reason:
            break

        # Wait for step interval, spinning to collect data
        deadline = time.time() + step_interval
        while time.time() < deadline:
            node.spin_once()
            time.sleep(0.05)

    # ---- Post-ramp: continue at v_end if no stop yet ----
    if stop_reason is None:
        print()
        print(f"[5] Ramp complete. Continuing at velocity {v_end:.3f} rad/s...")
        node.publish_velocity(v_end)

        post_step = 0
        while stop_reason is None:
            post_step += 1
            deadline = time.time() + step_interval
            while time.time() < deadline:
                node.spin_once()
                time.sleep(0.05)

            p = node._positions
            f = node._efforts

            for i, name in enumerate(FINGER_JOINTS):
                if f[i] >= force_thresholds[i]:
                    stop_reason = f"FORCE CONTACT on {name} ({f[i]:.0f} >= {force_thresholds[i]})"
                    break
            if stop_reason is None:
                for i, name in enumerate(FINGER_JOINTS):
                    if p[i] >= stop_positions[i]:
                        stop_reason = f"STOP POSITION reached on {name} ({p[i]:.3f} >= {stop_positions[i]:.2f})"
                        break

            print(
                f"  post {post_step:3d}  vel={v_end:.3f}  "
                f"pos=[{p[0]:.3f} {p[1]:.3f} {p[2]:.3f}]  "
                f"force=[{f[0]:.0f} {f[1]:.0f} {f[2]:.0f}]"
            )

    # ---- STOP ----
    print()
    print("=" * 50)
    print(f"  STOP: {stop_reason or 'unknown'}")
    print(f"  Final positions: [{node._positions[0]:.3f} {node._positions[1]:.3f} {node._positions[2]:.3f}]")
    print(f"  Final forces:    [{node._efforts[0]:.0f} {node._efforts[1]:.0f} {node._efforts[2]:.0f}]")
    print("=" * 50)

    # Zero velocity
    node.publish_velocity(0.0)
    time.sleep(0.5)

    # Switch back to position controller for safe return
    print()
    print("[6] Switching back to position controller...")
    try:
        reset_to_position_control()
    except Exception:
        pass

    print("[7] Returning hand to relaxed position...")
    node.publish_position(0.0)
    time.sleep(2.0)

    node.destroy_node()
    rclpy.shutdown()
    print("Test complete.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
