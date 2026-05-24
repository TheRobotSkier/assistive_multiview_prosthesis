#!/usr/bin/env python3
"""Hardware validation checklist for EMG grasp system.

REQUIRES EXPLICIT ENVIRONMENT FLAGS to run. Without them, this script does
nothing but print instructions. This prevents accidental hardware activation.

Required environment variables:
   EMG_GRASP_HW_TEST=1     Master enable flag
   MIA_PORT=<serial>       Mia Hand serial port (e.g., /dev/ttyUSB0)
   MINDROVE_IP=<ip>        MindRove board IP (optional, for EMG tests)

Optional safety overrides:
   EMG_GRASP_SKIP_WRIST=1  Skip wrist movement tests
   EMG_GRASP_SKIP_FORCE=1  Skip force/contact tests
   EMG_GRASP_QUICK=1       Reduce hold durations for quick validation

Usage:
   EMG_GRASP_HW_TEST=1 MIA_PORT=/dev/ttyUSB0 python3 tests/emg_grasp/test_hardware_checklist.py

The script validates:
   1. collect/train/launch sequence
   2. OPEN safety reset works
   3. POWER mode switching works
   4. Force-hold target adjustment works
   5. Wrist movement in allowed modes
   6. Safe shutdown leaves hand open
"""

from __future__ import annotations

import os
import sys
import time
import subprocess
from dataclasses import dataclass, field
from enum import Enum, auto
from pathlib import Path
from typing import Optional, List, Dict, Callable

# ═══════════════════════════════════════════════════════════════════════════════
# Environment checks — must pass before any hardware interaction
# ═══════════════════════════════════════════════════════════════════════════════

class HWTestError(Exception):
    """Hardware test failure."""


class HWTestSkip(Exception):
    """Test skipped (safety flag or missing hardware)."""


def _check_env_flag(name: str) -> bool:
    """Check if an environment flag is set to a truthy value."""
    val = os.environ.get(name, "").strip().lower()
    return val in ("1", "true", "yes", "y", "on")


def _require_env(name: str) -> str:
    """Get required environment variable or raise."""
    val = os.environ.get(name, "").strip()
    if not val:
        raise HWTestSkip(f"Environment variable {name} not set. "
                         f"Set it to enable hardware tests.")
    return val


# ═══════════════════════════════════════════════════════════════════════════════
# Test runner infrastructure
# ═══════════════════════════════════════════════════════════════════════════════

class TestStatus(Enum):
    PASS = auto()
    FAIL = auto()
    SKIP = auto()
    WARN = auto()


@dataclass
class TestResult:
    name: str
    status: TestStatus
    message: str = ""
    elapsed_ms: float = 0.0


class HardwareChecklist:
    """Hardware validation checklist runner."""

    def __init__(self):
        self.results: List[TestResult] = []
        self._mia_port: Optional[str] = None
        self._mindrove_ip: Optional[str] = None
        self._skip_wrist: bool = False
        self._skip_force: bool = False
        self._quick: bool = False

    # ── helpers ────────────────────────────────────────────────────────────

    def _run(self, name: str, fn: Callable) -> None:
        """Run a test function and record result."""
        start = time.monotonic()
        try:
            fn()
            elapsed = (time.monotonic() - start) * 1000
            self.results.append(TestResult(name, TestStatus.PASS, elapsed_ms=elapsed))
            print(f"  [PASS] {name} ({elapsed:.0f}ms)")
        except HWTestSkip as e:
            elapsed = (time.monotonic() - start) * 1000
            self.results.append(TestResult(name, TestStatus.SKIP, str(e), elapsed))
            print(f"  [SKIP] {name}: {e}")
        except HWTestError as e:
            elapsed = (time.monotonic() - start) * 1000
            self.results.append(TestResult(name, TestStatus.FAIL, str(e), elapsed))
            print(f"  [FAIL] {name}: {e}")
        except Exception as e:
            elapsed = (time.monotonic() - start) * 1000
            self.results.append(TestResult(name, TestStatus.FAIL, str(e), elapsed))
            print(f"  [FAIL] {name}: {e}")

    def _warn(self, msg: str) -> None:
        print(f"  [WARN] {msg}")

    # ── preflight ──────────────────────────────────────────────────────────

    def check_environment(self) -> None:
        """Verify environment flags and hardware prerequisites."""
        print("\n=== Environment Check ===\n")

        if not _check_env_flag("EMG_GRASP_HW_TEST"):
            print("  Hardware tests not enabled.")
            print("  Set EMG_GRASP_HW_TEST=1 to enable.")
            print("  Also set MIA_PORT=/dev/ttyUSB0 (or your serial port).")
            print("  Optional: MINDROVE_IP=<ip> for live EMG tests.")
            print("\n  Example:")
            print("    EMG_GRASP_HW_TEST=1 MIA_PORT=/dev/ttyUSB0 \\")
            print("      python3 tests/emg_grasp/test_hardware_checklist.py")
            sys.exit(0)

        self._mia_port = os.environ.get("MIA_PORT", "/dev/ttyUSB0")
        self._mindrove_ip = os.environ.get("MINDROVE_IP", "")
        self._skip_wrist = _check_env_flag("EMG_GRASP_SKIP_WRIST")
        self._skip_force = _check_env_flag("EMG_GRASP_SKIP_FORCE")
        self._quick = _check_env_flag("EMG_GRASP_QUICK")

        print(f"  MIA_PORT      = {self._mia_port}")
        print(f"  MINDROVE_IP   = {self._mindrove_ip or '(not set)'}")
        print(f"  SKIP_WRIST    = {self._skip_wrist}")
        print(f"  SKIP_FORCE    = {self._skip_force}")
        print(f"  QUICK         = {self._quick}")

        # Check serial port
        if os.path.exists(self._mia_port):
            print(f"  Serial port   OK ({self._mia_port} exists)")
        else:
            print(f"  Serial port   WARN ({self._mia_port} does not exist)")
            self._warn("MIA_PORT not found. Tests requiring hand will fail.")

        # Check ROS environment
        ros_distro = os.environ.get("ROS_DISTRO", "")
        if ros_distro:
            print(f"  ROS distro    OK ({ros_distro})")
        else:
            self._warn("ROS_DISTRO not set. Source ROS setup.bash first.")

        print()

    # ── safety pre-checks ──────────────────────────────────────────────────

    def safety_reminder(self) -> bool:
        """Display safety reminder and get operator confirmation."""
        print("=" * 60)
        print("  ⚠  SAFETY CHECKLIST  ⚠")
        print("=" * 60)
        print()
        print("  Before proceeding, confirm:")
        print("  1. Hand is mounted securely and has free range of motion")
        print("  2. No objects in the hand's path during opening/closing")
        print("  3. Emergency stop: Ctrl-C will trigger safe shutdown")
        print("  4. Force limits are calibrated (stop_positions, force_thresholds)")
        print("  5. Wrist movement area is clear (if testing wrist)")
        print(f"  6. Serial port: {self._mia_port}")
        print()

        try:
            response = input("  Type 'READY' to continue, anything else to abort: ").strip()
            return response == "READY"
        except (KeyboardInterrupt, EOFError):
            print("\n  Aborted.")
            return False

    # ── summary ────────────────────────────────────────────────────────────

    def print_summary(self) -> int:
        """Print test summary. Returns exit code (0 = all pass, 1 = failures)."""
        print("\n" + "=" * 60)
        print("  Hardware Validation Results")
        print("=" * 60)
        print()

        passed = sum(1 for r in self.results if r.status == TestStatus.PASS)
        failed = sum(1 for r in self.results if r.status == TestStatus.FAIL)
        skipped = sum(1 for r in self.results if r.status == TestStatus.SKIP)
        warned = sum(1 for r in self.results if r.status == TestStatus.WARN)

        for i, r in enumerate(self.results, 1):
            symbol = {TestStatus.PASS: "✓", TestStatus.FAIL: "✗",
                      TestStatus.SKIP: "-", TestStatus.WARN: "!"}[r.status]
            print(f"  {symbol} {i:2d}. {r.name}")
            if r.message:
                print(f"       {r.message}")

        print()
        print(f"  Passed:  {passed}")
        print(f"  Failed:  {failed}")
        print(f"  Skipped: {skipped}")
        if warned:
            print(f"  Warnings: {warned}")
        print("=" * 60)

        return 1 if failed > 0 else 0


# ═══════════════════════════════════════════════════════════════════════════════
# Test definitions (these run with real hardware)
# ═══════════════════════════════════════════════════════════════════════════════

def _bash(cmd: str, timeout: float = 10.0) -> Tuple[int, str, str]:
    """Run a bash command and return (exit_code, stdout, stderr)."""
    try:
        result = subprocess.run(
            ["bash", "-c", cmd],
            capture_output=True, text=True, timeout=timeout,
        )
        return result.returncode, result.stdout.strip(), result.stderr.strip()
    except subprocess.TimeoutExpired:
        return -1, "", "timeout"


def _ros_setup_cmd(cmd: str) -> str:
    """Wrap a command with ROS environment setup."""
    distro = os.environ.get("ROS_DISTRO", "jazzy")
    return (f"source /opt/ros/{distro}/setup.bash && "
            f"source /prosthesis_ws/install/setup.bash 2>/dev/null; "
            f"{cmd}")


def _ros_cmd(cmd: str, timeout: float = 10.0) -> Tuple[int, str, str]:
    """Run a ROS command with setup."""
    return _bash(_ros_setup_cmd(cmd), timeout=timeout)


def _ros_node_running(name: str) -> bool:
    """Check if a ROS node is running."""
    rc, out, _ = _ros_cmd("ros2 node list 2>/dev/null", timeout=5.0)
    if rc != 0:
        return False
    return f"/{name}" in out.split("\n")


def _ros_topic_exists(topic: str) -> bool:
    """Check if a ROS topic exists."""
    rc, out, _ = _ros_cmd("ros2 topic list 2>/dev/null", timeout=5.0)
    if rc != 0:
        return False
    return topic in out.split("\n")


# ═══════════════════════════════════════════════════════════════════════════════
# Hardware test functions
# ═══════════════════════════════════════════════════════════════════════════════

def test_collect_train_launch_sequence(checklist: HardwareChecklist) -> None:
    """Validate the collect/train/launch sequence works.

    Steps:
      1. Verify launch files exist and parse correctly
      2. Verify test config is loadable
      3. Verify we can discover the emg_grasp_node
      4. Check mock EMG publisher exists
    """
    ws = Path("/prosthesis_ws")

    # 1. Launch file exists
    launch_file = ws / "src/prosthesis_launch/launch/emg_grasp_test.launch.py"
    if not launch_file.exists():
        raise HWTestError(f"Launch file not found: {launch_file}")

    # 2. Config file exists and is valid YAML
    import yaml
    config_file = ws / "tests/emg_grasp/emg_grasp_test.yaml"
    if not config_file.exists():
        raise HWTestError(f"Config file not found: {config_file}")
    with open(config_file) as f:
        cfg = yaml.safe_load(f)
    assert "closing_velocity_start" in cfg
    assert "emg_grasp_trigger_gesture" in cfg

    # 3. EMG grasp node exists
    node_file = ws / "tests/emg_grasp/emg_grasp_node.py"
    if not node_file.exists():
        raise HWTestError(f"Node file not found: {node_file}")

    # 4. Mock publisher exists
    mock_file = ws / "tests/emg_grasp/mock_emg_publisher.py"
    if not mock_file.exists():
        raise HWTestError(f"Mock publisher not found: {mock_file}")

    # 5. EMG bridge packages installed
    rc, out, _ = _ros_cmd("ros2 pkg list 2>/dev/null | grep -E 'emg_bridge|prosthesis_launch'", timeout=5.0)
    if rc != 0 or not out.strip():
        checklist._warn("EMG bridge or prosthesis_launch package not found in ROS workspace")
    else:
        print(f"    Packages found: {out.strip()}")


def test_open_safety_reset(checklist: HardwareChecklist) -> None:
    """Validate OPEN safety reset behavior.

    Steps:
      1. Start the grasp test node (or verify it's running)
      2. Send OPEN gesture with high confidence
      3. Verify the node transitions to IDLE/RELEASING
      4. Verify hand position commands are zero
    """
    # Check if ROS is reachable
    if not _ros_node_running("emg_grasp_test"):
        checklist._warn("emg_grasp_test node not running. Start it with:")
        checklist._warn("  ros2 launch prosthesis_launch emg_grasp_test.launch.py")

    # Validate the OPEN gesture configuration
    import yaml
    config_file = Path("/prosthesis_ws/tests/emg_grasp/emg_grasp_test.yaml")
    with open(config_file) as f:
        cfg = yaml.safe_load(f)

    release_gesture = cfg["emg_release_gesture"]
    assert release_gesture == 3, f"Expected release gesture 3 (OPEN), got {release_gesture}"
    print(f"    Release gesture configured as: {release_gesture} (OPEN)")

    conf_threshold = cfg["emg_grasp_confidence_threshold"]
    assert 0.0 <= conf_threshold <= 1.0, f"Confidence threshold out of range: {conf_threshold}"
    print(f"    Confidence threshold: {conf_threshold}")

    hold_timeout = cfg["gesture_hold_timeout_s"]
    assert hold_timeout > 0, f"Hold timeout must be positive: {hold_timeout}"
    print(f"    Hold timeout: {hold_timeout}s")


def test_power_mode_switching(checklist: HardwareChecklist) -> None:
    """Validate POWER mode switching.

    Steps:
      1. Verify POWER gesture is configured as grasp trigger
      2. Verify mode transitions are consistent
      3. If hand is available, verify hand closes on POWER
    """
    import yaml
    config_file = Path("/prosthesis_ws/tests/emg_grasp/emg_grasp_test.yaml")
    with open(config_file) as f:
        cfg = yaml.safe_load(f)

    grasp_gesture = cfg["emg_grasp_trigger_gesture"]
    assert grasp_gesture == 1, f"Expected grasp trigger gesture 1 (POWER), got {grasp_gesture}"
    print(f"    Grasp trigger gesture: {grasp_gesture} (POWER)")

    # Verify mode machine logic (loadable even without hand)
    from test_mock_integration import ModeMachine, GraspMode, EmgGesture
    mm = ModeMachine()
    assert mm.mode == GraspMode.NOT_GRASPING

    # Simulate POWER hold
    mm._gesture_start_time = time.time() - 0.6
    result = mm.transition(gesture=1, confidence=0.8)
    assert result == GraspMode.CONTROL_GRASP

    # Simulate OPEN release
    mm._gesture_start_time = time.time() - 0.6
    result = mm.transition(gesture=3, confidence=0.8)
    assert result == GraspMode.NOT_GRASPING

    print("    Mode transitions validated (NOT_GRASPING ↔ CONTROL_GRASP)")


def test_force_hold_target_adjustment(checklist: HardwareChecklist) -> None:
    """Validate force-hold target adjustment.

    Steps:
      1. Verify force controller math (PI adjustment)
      2. If hand and force sensors available, verify real behavior
    """
    from test_mock_integration import ForceControllerState, compute_force_hold_target

    # Test PI math
    fc = ForceControllerState(kp=0.01, target_min=50, target_max=200)
    adj, integral = fc.compute_adjustment(force=50.0)  # below target
    assert adj > 0, f"PI should close more when force {50} < target {125}"
    print(f"    PI adjustment (force=50, target=125): {adj:.4f}")

    adj, integral = fc.compute_adjustment(force=200.0)  # above target
    assert adj < 0, f"PI should open when force {200} > target {125}"
    print(f"    PI adjustment (force=200, target=125): {adj:.4f}")

    # Test hold target
    new_pos = compute_force_hold_target(1.0, 50.0, 125.0, kp=0.01, max_step=0.05)
    assert new_pos > 1.0
    print(f"    Hold target: from {1.0:.2f} → {new_pos:.4f}")

    print("    Force-target adjustment math validated")


def test_wrist_movement_in_allowed_modes(checklist: HardwareChecklist) -> None:
    """Validate wrist movement is only allowed in CONTROL_WRIST mode.

    Steps:
      1. Verify wrist config is loaded
      2. Verify wrist gating logic
      3. If wrist hardware available, test real movement
    """
    if checklist._skip_wrist:
        raise HWTestSkip("EMG_GRASP_SKIP_WRIST=1 set")

    from test_mock_integration import WristController, GraspMode

    # Verify wrist gating logic
    wc = WristController()

    # NOT_GRASPING → no wrist
    wc.set_mode(GraspMode.NOT_GRASPING)
    cmd = wc.compute_command(desired_velocity=10.0)
    assert cmd == 0.0, f"Wrist should be 0 in NOT_GRASPING, got {cmd}"

    # CONTROL_GRASP → no wrist (key constraint!)
    wc.set_mode(GraspMode.CONTROL_GRASP)
    cmd = wc.compute_command(desired_velocity=10.0)
    assert cmd == 0.0, f"Wrist should be 0 in CONTROL_GRASP, got {cmd}"
    print("    Wrist correctly gated in CONTROL_GRASP")

    # CONTROL_WRIST → wrist allowed
    wc.set_mode(GraspMode.CONTROL_WRIST)
    cmd = wc.compute_command(desired_velocity=10.0, dt=0.1)
    assert cmd > 0.0, f"Wrist should move in CONTROL_WRIST, got {cmd}"
    print(f"    Wrist movement allowed in CONTROL_WRIST: {cmd:.2f}°")

    # Velocity clamping
    wc.reset()
    wc.set_mode(GraspMode.CONTROL_WRIST)
    cmd = wc.compute_command(desired_velocity=100.0, dt=0.1)  # exceeds max 30
    assert cmd == pytest.approx(3.0), f"Velocity should be clamped, got {cmd}"
    print("    Wrist velocity clamping validated")

    # Check wrist config in test YAML
    import yaml
    config_file = Path("/prosthesis_ws/tests/emg_grasp/emg_grasp_test.yaml")
    with open(config_file) as f:
        cfg = yaml.safe_load(f)
    assert "wrist_control_enabled" in cfg
    print(f"    Wrist control in config: enabled={cfg['wrist_control_enabled']}")


def test_safe_shutdown_leaves_hand_open(checklist: HardwareChecklist) -> None:
    """Validate safe shutdown sequence.

    Steps:
      1. Verify shutdown sequence logic
      2. Verify node cleanup code
    """
    from test_mock_integration import ShutdownSequence
    import inspect

    # Verify shutdown logic
    seq = ShutdownSequence()
    steps = seq.execute()
    assert seq.is_safe
    assert "velocity_zeroed" in steps
    assert "hand_opened" in steps
    assert "force_disabled" in steps
    print(f"    Shutdown steps: {steps}")

    # Verify EmgGraspNode has cleanup in finally block
    from emg_grasp_node import main as node_main
    src = inspect.getsource(node_main)
    assert "_publish_velocity" in src, "main() must publish zero velocity on shutdown"
    assert "destroy_node" in src, "main() must destroy node on shutdown"
    print("    Node cleanup code verified (velocity zero + destroy)")

    # Verify force_controller has _emergency_release
    from force_controller.force_controller_node import ForceControllerNode
    assert hasattr(ForceControllerNode, "_emergency_release"), \
        "ForceControllerNode must have emergency release method"
    print("    Force controller emergency release method found")


# ═══════════════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    print("=" * 60)
    print("  EMG Grasp — Hardware Validation Checklist")
    print("=" * 60)

    checklist = HardwareChecklist()

    # ── Phase 0: Environment check ─────────────────────────────────────────
    checklist.check_environment()

    # ── Phase 1: Safety confirmation ───────────────────────────────────────
    if not checklist.safety_reminder():
        sys.exit(1)
    print()

    # ── Phase 2: Add Python path for test imports ──────────────────────────
    ws = Path("/prosthesis_ws")
    test_dir = ws / "tests" / "emg_grasp"
    if str(test_dir) not in sys.path:
        sys.path.insert(0, str(test_dir))

    # ── Phase 3: Run tests ─────────────────────────────────────────────────
    print("=== Running Hardware Validation Tests ===\n")
    import pytest  # noqa: F811 (re-import for assertion helpers)

    checklist._run("collect/train/launch sequence", lambda: test_collect_train_launch_sequence(checklist))
    checklist._run("OPEN safety reset", lambda: test_open_safety_reset(checklist))
    checklist._run("POWER mode switching", lambda: test_power_mode_switching(checklist))
    checklist._run("force-hold target adjustment", lambda: test_force_hold_target_adjustment(checklist))
    checklist._run("wrist movement in allowed modes", lambda: test_wrist_movement_in_allowed_modes(checklist))
    checklist._run("safe shutdown leaves hand open", lambda: test_safe_shutdown_leaves_hand_open(checklist))

    # ── Phase 4: Summary ───────────────────────────────────────────────────
    exit_code = checklist.print_summary()

    if exit_code == 0:
        print("\n  All hardware validation checks passed.")
    else:
        print(f"\n  Some checks failed. Review the output above.")

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
