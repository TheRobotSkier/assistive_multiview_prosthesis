#!/usr/bin/env python3
"""Static grasp test with velocity-based force-aware closure.

Launches ros2_control, resets the hand, then closes fingers using a velocity
ramp (start speed → end speed over N discrete steps). At every step, forces
and positions are checked. If any finger exceeds its force threshold or
stop position, ALL fingers are stopped immediately.
"""
import os
import re
import sys
import time
import signal
import subprocess

import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

FINGER_JOINTS = ["j_thumb_fle", "j_index_fle", "j_mrl_fle"]
FINGER_COUNT = 3
POSITION_CONTROLLERS = [
    "group_pos_ff_controller",
    "thumb_pos_ff_controller",
    "index_pos_ff_controller",
    "mrl_pos_ff_controller",
]
VELOCITY_CONTROLLERS = [
    "group_vel_ff_controller",
    "thumb_vel_ff_controller",
    "index_vel_ff_controller",
    "mrl_vel_ff_controller",
]


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


def _env_flag(name: str, default: bool = True) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() not in {"0", "false", "no", "off"}


def _env_float(name: str, default: float) -> float:
    value = os.environ.get(name)
    if value is None or value.strip() == "":
        return default
    return float(value)


def start_controller_manager_if_needed(serial_port: str):
    """Start Mia hand ros2_control unless another controller manager is present."""
    if wait_for_controller_manager(timeout_s=2):
        print("     Found existing controller_manager.")
        return None

    if not _env_flag("GRASP_TEST_START_CONTROLLER", default=True):
        return None

    use_mock_hardware = _env_flag("GRASP_TEST_USE_MOCK_HARDWARE", default=False)
    cmd = [
        "ros2",
        "launch",
        "mia_hand_ros2_control",
        "mia_hand_system_interface_launch.py",
        f"serial_port:={serial_port}",
        "rviz2_gui:=false",
        "controller:=group_pos_ff_controller",
        f"use_mock_hardware:={'true' if use_mock_hardware else 'false'}",
    ]
    if use_mock_hardware:
        print("     Starting Mia hand ros2_control launch with mock hardware...")
    else:
        print("     Starting Mia hand ros2_control launch...")
    return subprocess.Popen(cmd, start_new_session=True)


def stop_controller_manager(proc):
    if proc is None or proc.poll() is not None:
        return
    os.killpg(proc.pid, signal.SIGINT)
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGTERM)
        proc.wait(timeout=5)


def competing_controllers(active):
    active_set = set(active)
    return [c for c in POSITION_CONTROLLERS + VELOCITY_CONTROLLERS if c not in active_set]


def controller_states():
    try:
        out = ros(
            "service", "call",
            "/controller_manager/list_controllers",
            "controller_manager_msgs/srv/ListControllers",
            "{}",
            timeout=5,
            check=False,
        )
    except Exception:
        return {}

    states = {}
    for name, state in re.findall(r"name='([^']+)'.*?state='([^']+)'", out):
        states[name] = state
    return states


def ensure_controller_loaded(ctrl):
    states = controller_states()
    if ctrl in states:
        return

    load_req = f"{{name: '{ctrl}'}}"
    load_out = ros(
        "service", "call",
        "/controller_manager/load_controller",
        "controller_manager_msgs/srv/LoadController",
        load_req,
        timeout=10,
        check=False,
    )
    if "ok=True" not in load_out:
        raise RuntimeError(f"Failed to load controller {ctrl}: {load_out.strip()}")

    configure_out = ros(
        "service", "call",
        "/controller_manager/configure_controller",
        "controller_manager_msgs/srv/ConfigureController",
        load_req,
        timeout=10,
        check=False,
    )
    if "ok=True" not in configure_out:
        raise RuntimeError(
            f"Failed to configure controller {ctrl}: {configure_out.strip()}"
        )


def load_and_switch_controllers(activate, deactivate):
    """Load a controller if needed, then switch using ros2 CLI."""
    if not wait_for_controller_manager(timeout_s=5):
        raise RuntimeError("controller_manager service is not available")

    states = controller_states()
    activate = [ctrl for ctrl in activate if states.get(ctrl) != "active"]
    deactivate = [ctrl for ctrl in deactivate if states.get(ctrl) == "active"]

    if not activate and not deactivate:
        return True

    # First ensure requested controllers are loaded and configured.
    for ctrl in activate:
        ensure_controller_loaded(ctrl)

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
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
    except subprocess.TimeoutExpired as exc:
        raise RuntimeError("Timed out switching controllers") from exc
    if result.returncode != 0:
        raise RuntimeError(f"Controller switch failed: {result.stderr.strip()}")
    if "ok=True" not in result.stdout:
        # Try BEST_EFFORT as fallback
        req2 = req.replace("strictness: 2", "strictness: 1")
        cmd[-1] = req2
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError("Timed out switching controllers") from exc
        if "ok=True" not in result.stdout:
            raise RuntimeError(f"Controller switch not ok: {result.stdout.strip()}")
    return True

def reset_to_position_control():
    """Switch back to position control for safety."""
    load_and_switch_controllers(
        ["group_pos_ff_controller"],
        competing_controllers(["group_pos_ff_controller"]),
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


def compute_hold_velocity(force, target, deadzone, min_overshoot, max_overshoot, max_velocity):
    error = target - force
    abs_error = abs(error)
    if abs_error <= deadzone:
        return 0.0

    if max_overshoot <= 0.0:
        return max_velocity if error > 0.0 else -max_velocity

    scaled_error = min(max(abs_error, min_overshoot), max_overshoot)
    velocity = max_velocity * (scaled_error / max_overshoot)
    return velocity if error > 0.0 else -velocity


def check_force_contact(efforts, force_thresholds):
    for i, name in enumerate(FINGER_JOINTS):
        if efforts[i] >= force_thresholds[i]:
            return f"FORCE CONTACT on {name} ({efforts[i]:.0f} >= {force_thresholds[i]})"
    return None


def check_stop_position(positions, stop_positions):
    for i, name in enumerate(FINGER_JOINTS):
        if positions[i] >= stop_positions[i]:
            return f"STOP POSITION reached on {name} ({positions[i]:.3f} >= {stop_positions[i]:.2f})"
    return None


def spin_until(node, deadline, force_thresholds, stop_positions=None):
    while time.time() < deadline:
        node.spin_once()
        contact_reason = check_force_contact(node._efforts, force_thresholds)
        if contact_reason:
            return contact_reason, None

        if stop_positions is not None:
            stop_reason = check_stop_position(node._positions, stop_positions)
            if stop_reason:
                return None, stop_reason

        time.sleep(0.05)
    return None, None


def main():
    controller_proc = None
    rclpy_started = False
    node = None

    config_path = os.environ.get(
        "GRASP_TEST_CONFIG", "/prosthesis_ws/config/static_grasp_test.yaml"
    )
    serial_port = os.environ.get("MIA_PORT", "/dev/ttyUSB0")
    config = load_config(config_path)

    v_start = float(config["closing_velocity_start"])
    v_end = float(config["closing_velocity_end"])
    decay_steps = int(config["decay_steps"])
    step_interval = float(config["step_interval_s"])
    relaxed_wait = float(config["relaxed_wait_s"])
    max_closing_duration = _env_float(
        "GRASP_TEST_MAX_CLOSING_DURATION_S",
        float(config.get("max_closing_duration_s", 20.0)),
    )

    sp = config["stop_positions"]
    ft = config["force_thresholds"]
    hold_cfg = config.get("force_hold", {})
    stop_positions = [float(sp[n]) for n in FINGER_JOINTS]
    force_thresholds = [float(ft[n]) for n in FINGER_JOINTS]
    if os.environ.get("GRASP_TEST_FORCE_THRESHOLD", "").strip():
        force_thresholds = [
            _env_float("GRASP_TEST_FORCE_THRESHOLD", force_thresholds[0])
        ] * FINGER_COUNT

    hold_targets_cfg = hold_cfg.get("target_forces", ft)
    hold_targets = [float(hold_targets_cfg[n]) for n in FINGER_JOINTS]
    if os.environ.get("GRASP_TEST_FORCE_HOLD_TARGET", "").strip():
        hold_targets = [
            _env_float("GRASP_TEST_FORCE_HOLD_TARGET", hold_targets[0])
        ] * FINGER_COUNT

    hold_duration = _env_float(
        "GRASP_TEST_FORCE_HOLD_DURATION_S",
        float(hold_cfg.get("duration_s", 10.0)),
    )
    hold_deadzone = float(hold_cfg.get("deadzone", 20.0))
    hold_min_overshoot = float(hold_cfg.get("min_overshoot", hold_deadzone))
    hold_max_overshoot = float(hold_cfg.get("max_overshoot", 150.0))
    hold_max_velocity = float(hold_cfg.get("max_velocity", 0.08))

    ramp = compute_velocity_ramp(v_start, v_end, decay_steps)

    print("=" * 50)
    print("  Velocity-Based Force-Aware Static Grasp Test")
    print("=" * 50)
    print(f"Velocity ramp : {v_start:.3f} → {v_end:.3f} rad/s over {decay_steps} steps")
    print(f"Step interval : {step_interval}s  (total ramp: {decay_steps * step_interval:.1f}s)")
    print(f"Stop positions: thumb={stop_positions[0]:.2f}  index={stop_positions[1]:.2f}  mrl={stop_positions[2]:.2f} rad")
    print(f"Force thresholds: thumb={force_thresholds[0]}  index={force_thresholds[1]}  mrl={force_thresholds[2]} raw ADC")
    print(f"Hold targets: thumb={hold_targets[0]}  index={hold_targets[1]}  mrl={hold_targets[2]} raw ADC")
    print(f"Hold duration: {hold_duration:.1f}s  deadzone={hold_deadzone:.1f}  max_vel={hold_max_velocity:.3f} rad/s")
    print(f"Max close time: {max_closing_duration:.1f}s")
    print(f"Relaxed wait : {relaxed_wait}s")
    print()

    try:
        # ---- Start/wait for controller_manager ----
        print("[1] Starting/waiting for controller_manager...")
        controller_proc = start_controller_manager_if_needed(serial_port)
        if not wait_for_controller_manager():
            print("FATAL: controller_manager not available")
            return 1
        print("     Ready.")

        # ---- Init rclpy ----
        rclpy.init()
        rclpy_started = True
        node = GraspTestNode()

        # ---- Reset to relaxed ----
        print("[2] Resetting hand to relaxed position (0.0 rad)...")
        load_and_switch_controllers(
            ["group_pos_ff_controller"],
            competing_controllers(["group_pos_ff_controller"]),
        )
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
        load_and_switch_controllers(
            ["group_vel_ff_controller"],
            competing_controllers(["group_vel_ff_controller"]),
        )
        time.sleep(1.0)
        print("     Velocity controller active.")

        # ---- Velocity ramp closure ----
        print("[4] Closing with velocity ramp — monitoring forces and positions...")
        print()

        stop_reason = None
        contact_reason = None
        close_started_at = time.time()
        # Ramp phase
        for step_idx, vel in enumerate(ramp):
            node.publish_velocity(vel)
            node.spin_once()

            p = node._positions
            f = node._efforts

            contact_reason = check_force_contact(f, force_thresholds)

            # Check positions
            if contact_reason is None:
                stop_reason = check_stop_position(p, stop_positions)

            print(
                f"  step {step_idx+1:2d}/{decay_steps}  vel={vel:.3f}  "
                f"pos=[{p[0]:.3f} {p[1]:.3f} {p[2]:.3f}]  "
                f"force=[{f[0]:.0f} {f[1]:.0f} {f[2]:.0f}]"
            )

            if contact_reason or stop_reason:
                break

            # Wait for step interval, spinning to collect data
            deadline = time.time() + step_interval
            contact_reason, stop_reason = spin_until(
                node, deadline, force_thresholds, stop_positions
            )
            if (
                contact_reason is None
                and stop_reason is None
                and time.time() - close_started_at >= max_closing_duration
            ):
                stop_reason = (
                    f"MAX CLOSING DURATION reached ({max_closing_duration:.1f}s)"
                )

            if contact_reason or stop_reason:
                break

        # ---- Post-ramp: continue at v_end if no stop yet ----
        if contact_reason is None and stop_reason is None:
            print()
            print(f"[5] Ramp complete. Continuing at velocity {v_end:.3f} rad/s...")
            node.publish_velocity(v_end)

            post_step = 0
            while contact_reason is None and stop_reason is None:
                post_step += 1
                deadline = time.time() + step_interval
                contact_reason, stop_reason = spin_until(
                    node, deadline, force_thresholds, stop_positions
                )
                if (
                    contact_reason is None
                    and stop_reason is None
                    and time.time() - close_started_at >= max_closing_duration
                ):
                    stop_reason = (
                        f"MAX CLOSING DURATION reached ({max_closing_duration:.1f}s)"
                    )

                p = node._positions
                f = node._efforts

                print(
                    f"  post {post_step:3d}  vel={v_end:.3f}  "
                    f"pos=[{p[0]:.3f} {p[1]:.3f} {p[2]:.3f}]  "
                    f"force=[{f[0]:.0f} {f[1]:.0f} {f[2]:.0f}]"
                )

        # ---- Force hold ----
        if contact_reason is not None:
            print()
            print(f"[5] {contact_reason}. Holding target force for {hold_duration:.1f}s...")
            hold_started_at = time.time()
            hold_step = 0
            while time.time() - hold_started_at < hold_duration:
                hold_step += 1
                hold_velocities = [
                    compute_hold_velocity(
                        node._efforts[i],
                        hold_targets[i],
                        hold_deadzone,
                        hold_min_overshoot,
                        hold_max_overshoot,
                        hold_max_velocity,
                    )
                    for i in range(FINGER_COUNT)
                ]
                msg = Float64MultiArray()
                msg.data = hold_velocities
                node._vel_pub.publish(msg)

                deadline = time.time() + step_interval
                spin_until(node, deadline, [float("inf")] * FINGER_COUNT)

                p = node._positions
                f = node._efforts
                print(
                    f"  hold {hold_step:3d}  "
                    f"vel=[{hold_velocities[0]:+.3f} {hold_velocities[1]:+.3f} {hold_velocities[2]:+.3f}]  "
                    f"force=[{f[0]:.0f} {f[1]:.0f} {f[2]:.0f}]  "
                    f"target=[{hold_targets[0]:.0f} {hold_targets[1]:.0f} {hold_targets[2]:.0f}]"
                )
            stop_reason = f"FORCE HOLD complete after {hold_duration:.1f}s"

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

        print("Test complete.")
        return 0
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy_started:
            rclpy.shutdown()
        stop_controller_manager(controller_proc)


if __name__ == "__main__":
    sys.exit(main())
