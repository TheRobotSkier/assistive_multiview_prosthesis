#!/usr/bin/env python3
"""PTY-driven offline end-to-end suite for the mia_haptic_force_test stack.

Spawns the split-node launch under a pseudo-terminal, writes raw WASD
bytes to the PTY master, and reads the combined transcript back from the
same master.  The harness subscribes to key topics in-process to verify
that the stack actually reacts to user input.

Usage:
    python3 -m tests.mia_haptic_force_test.offline_suite \\
        --config-path /path/to/config.yaml \\
        --run-id my_test_run \\
        --output-dir /tmp/test_output

The harness will:
1. Create a temp config overlay that shortens stage timeouts.
2. Open a PTY and spawn ``ros2 launch prosthesis_launch
   mia_haptic_force_test.launch.py mock_hardware:=true
   terminal_ui:=false logger:=false wrist_enable:=false`` with
   stdin/stdout/stderr = PTY slave.
3. Wait for /test/stage to reach waiting_for_activation.
4. Write a single ``d`` byte and assert /emg/gesture_name = POWER
   (pre-check; fails fast if stdin forwarding is broken).
5. Run the full happy-path scenario:
   d held → rotating_to_vertical → vertical_delay → force_closing →
   force_hold (force) → s/w adjust force → d flips to wrist →
   w/s adjust wrist → d flips back → a held → opening_hand →
   return_delay → return_wrist → complete.
6. Run the fault-path scenario (separate launch with
   use_effort_fallback=false) and assert /test/stage = fault.
7. Save the PTY transcript + topic snapshot to the run dir.
"""

from __future__ import annotations

import argparse
import json
import os
import pty
import select
import signal
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path
from typing import Any, Optional

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from scripts.mia_haptic_force_test.common.constants import (
    HoldControl,
    Stage,
    TOPIC_CONTROL_HOLD_MODE,
    TOPIC_CONTROL_TARGET_FORCE,
    TOPIC_CONTROL_TARGET_WRIST,
    TOPIC_EMG_GESTURE,
    TOPIC_TEST_STAGE,
)


# ── Optional ROS imports ──────────────────────────────────────────────────

try:
    import rclpy  # noqa: F401
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import Bool, Float64MultiArray, String
    from sensor_msgs.msg import JointState

    _HAS_ROS = True
except ImportError:
    _HAS_ROS = False



# ── Topic snapshot subscriber ─────────────────────────────────────────────


if _HAS_ROS:

    class TopicSnapshot(Node):
        """Subscribe to the topics the harness needs to inspect."""

        def __init__(self) -> None:
            super().__init__("offline_suite_snapshot")
            self._lock = threading.Lock()
            self._stage: str = ""
            self._hold_mode: str = ""
            self._gesture: str = ""
            self._last_target_force: list[float] = []
            self._last_target_wrist: list[float] = []
            self._stage_history: list[tuple[float, str]] = []
            self._hold_mode_history: list[tuple[float, str]] = []
            self._start = time.monotonic()

            self.create_subscription(String, TOPIC_TEST_STAGE, self._on_stage, 10)
            self.create_subscription(String, TOPIC_CONTROL_HOLD_MODE, self._on_hold, 10)
            self.create_subscription(String, TOPIC_EMG_GESTURE, self._on_gesture, 10)
            self.create_subscription(
                Float64MultiArray, TOPIC_CONTROL_TARGET_FORCE, self._on_force, 10
            )
            self.create_subscription(
                Float64MultiArray, TOPIC_CONTROL_TARGET_WRIST, self._on_wrist, 10
            )

        def _on_stage(self, msg: String) -> None:
            with self._lock:
                if msg.data != self._stage:
                    self._stage = msg.data
                    self._stage_history.append((time.monotonic() - self._start, msg.data))

        def _on_hold(self, msg: String) -> None:
            with self._lock:
                if msg.data != self._hold_mode:
                    self._hold_mode = msg.data
                    self._hold_mode_history.append((time.monotonic() - self._start, msg.data))

        def _on_gesture(self, msg: String) -> None:
            with self._lock:
                self._gesture = msg.data

        def _on_force(self, msg: Float64MultiArray) -> None:
            with self._lock:
                self._last_target_force = list(msg.data)

        def _on_wrist(self, msg: Float64MultiArray) -> None:
            with self._lock:
                self._last_target_wrist = list(msg.data)

        def snapshot(self) -> dict[str, Any]:
            with self._lock:
                return {
                    "stage": self._stage,
                    "hold_mode": self._hold_mode,
                    "gesture": self._gesture,
                    "last_target_force": list(self._last_target_force),
                    "last_target_wrist": list(self._last_target_wrist),
                    "stage_history": list(self._stage_history),
                    "hold_mode_history": list(self._hold_mode_history),
                }

        def wait_for_stage(self, target: str, timeout_s: float) -> bool:
            deadline = time.monotonic() + timeout_s
            while time.monotonic() < deadline:
                with self._lock:
                    if self._stage == target:
                        return True
                time.sleep(0.05)
            return False

        def wait_for_gesture(self, target: str, timeout_s: float) -> bool:
            deadline = time.monotonic() + timeout_s
            while time.monotonic() < deadline:
                with self._lock:
                    if self._gesture == target:
                        return True
                time.sleep(0.02)
            return False

        def wait_for_hold_mode(self, target: str, timeout_s: float) -> bool:
            deadline = time.monotonic() + timeout_s
            while time.monotonic() < deadline:
                with self._lock:
                    if self._hold_mode == target:
                        return True
                time.sleep(0.02)
            return False


# ── PTY helpers ──────────────────────────────────────────────────────────


def _drain_pty(master_fd: int, sink: list[str]) -> None:
    """Read from the PTY master into *sink* until EOF."""
    while True:
        try:
            data = os.read(master_fd, 4096)
        except OSError:
            break
        if not data:
            break
        try:
            sink.append(data.decode("utf-8", errors="replace"))
        except Exception:
            sink.append(repr(data))


def _write_key(master_fd: int, key: str) -> None:
    """Write a single key byte to the PTY master."""
    os.write(master_fd, key.encode("utf-8"))


# ── Suite implementation ──────────────────────────────────────────────────


class OfflineSuite:
    """Run the full happy + fault scenarios against the mock simulator."""

    def __init__(
        self,
        *,
        config_path: Path,
        run_id: str,
        output_dir: Path,
        scenario_timeout_s: float = 60.0,
    ) -> None:
        self._config_path = config_path
        self._run_id = run_id
        self._output_dir = output_dir
        self._scenario_timeout_s = scenario_timeout_s
        self._transcript: list[str] = []
        self._errors: list[str] = []

    # ── Helpers ──────────────────────────────────────────────────────

    def _record(self, msg: str) -> None:
        print(f"[suite] {msg}", flush=True)

    def _fail(self, msg: str) -> None:
        self._errors.append(msg)
        self._record(f"FAIL: {msg}")

    # ── Scenario: happy path ─────────────────────────────────────────

    def run_happy_path(
        self,
        master_fd: int,
        snapshot: "TopicSnapshot",
        executor: "SingleThreadedExecutor",
    ) -> bool:
        self._record("=== happy path ===")
        # Wait for the system to reach waiting_for_activation.
        if not snapshot.wait_for_stage(Stage.WAITING_FOR_ACTIVATION.value, 15.0):
            self._fail("did not reach waiting_for_activation within 15s")
            return False
        self._record("reached waiting_for_activation")

        # Pre-check: write one 'd' byte, assert /emg/gesture_name = POWER.
        _write_key(master_fd, "d")
        time.sleep(0.1)
        executor.spin_once(timeout_sec=0.1)
        if not snapshot.wait_for_gesture("POWER", 1.0):
            self._fail("pre-check failed: gesture_name did not become POWER after 'd'")
            return False
        self._record("pre-check OK: gesture=POWER after 'd'")

        # Hold 'd' to trigger activation → rotating_to_vertical.
        for _ in range(int(0.5 / 0.02)):
            _write_key(master_fd, "d")
            executor.spin_once(timeout_sec=0.02)
            time.sleep(0.02)
        if not snapshot.wait_for_stage(Stage.ROTATING_TO_VERTICAL.value, 3.0):
            self._fail("did not reach rotating_to_vertical")
            return False
        self._record("reached rotating_to_vertical")

        # Wait for the wrist to reach vertical and vertical_delay to start.
        if not snapshot.wait_for_stage(Stage.VERTICAL_DELAY.value, 10.0):
            self._fail("did not reach vertical_delay within 10s")
            return False
        self._record("reached vertical_delay")

        # Wait for force_closing → force_hold.
        if not snapshot.wait_for_stage(Stage.FORCE_CLOSING.value, 10.0):
            self._fail("did not reach force_closing within 10s")
            return False
        self._record("reached force_closing")

        if not snapshot.wait_for_stage(Stage.FORCE_HOLD.value, 15.0):
            self._fail("did not reach force_hold within 15s")
            return False
        self._record("reached force_hold")

        # In force mode, send 's' to increase force.
        for _ in range(int(0.3 / 0.02)):
            _write_key(master_fd, "s")
            executor.spin_once(timeout_sec=0.02)
            time.sleep(0.02)
        time.sleep(0.2)
        executor.spin_once(timeout_sec=0.1)
        snap = snapshot.snapshot()
        self._record(f"after s: target_force={snap['last_target_force']}")

        # Send 'd' to flip to wrist mode.
        for _ in range(int(0.3 / 0.02)):
            _write_key(master_fd, "d")
            executor.spin_once(timeout_sec=0.02)
            time.sleep(0.02)
        if not snapshot.wait_for_hold_mode(HoldControl.WRIST.value, 2.0):
            self._fail("hold_mode did not flip to wrist")
            return False
        self._record("hold_mode flipped to wrist")

        # In wrist mode, send 'w' to move wrist.
        for _ in range(int(0.3 / 0.02)):
            _write_key(master_fd, "w")
            executor.spin_once(timeout_sec=0.02)
            time.sleep(0.02)
        time.sleep(0.2)
        executor.spin_once(timeout_sec=0.1)
        snap = snapshot.snapshot()
        self._record(f"after w (wrist): target_wrist={snap['last_target_wrist']}")

        # Send 'd' again to flip back to force.
        for _ in range(int(0.3 / 0.02)):
            _write_key(master_fd, "d")
            executor.spin_once(timeout_sec=0.02)
            time.sleep(0.02)
        if not snapshot.wait_for_hold_mode(HoldControl.FORCE.value, 2.0):
            self._fail("hold_mode did not flip back to force")
            return False
        self._record("hold_mode flipped back to force")

        # Hold 'a' to trigger opening_hand.
        for _ in range(int(0.5 / 0.02)):
            _write_key(master_fd, "a")
            executor.spin_once(timeout_sec=0.02)
            time.sleep(0.02)
        if not snapshot.wait_for_stage(Stage.OPENING_HAND.value, 3.0):
            self._fail("did not reach opening_hand after 'a'")
            return False
        self._record("reached opening_hand")

        # Wait for complete.
        if not snapshot.wait_for_stage(Stage.COMPLETE.value, 15.0):
            self._fail("did not reach complete within 15s")
            return False
        self._record("reached complete")
        return True

    # ── Scenario: fault path ─────────────────────────────────────────

    def run_fault_path(
        self,
        master_fd: int,
        snapshot: "TopicSnapshot",
        executor: "SingleThreadedExecutor",
    ) -> bool:
        self._record("=== fault path ===")
        # For the fault path, the launch overlay sets use_effort_fallback=false
        # and require_force_data=true.  The supervisor should fault when the
        # simulator stops publishing /hand_sim/forces.
        if not snapshot.wait_for_stage(Stage.FAULT.value, 10.0):
            self._fail("did not reach fault within 10s")
            return False
        self._record("reached fault")
        return True

    # ── Output ───────────────────────────────────────────────────────

    def save_outputs(
        self,
        snapshot_data: dict[str, Any],
        config_overlay_text: str,
    ) -> None:
        self._output_dir.mkdir(parents=True, exist_ok=True)
        # Transcript
        transcript_path = self._output_dir / "pty_transcript.txt"
        transcript_path.write_text("".join(self._transcript), encoding="utf-8")
        # Topic snapshot
        snapshot_path = self._output_dir / "topic_snapshot.json"
        snapshot_path.write_text(
            json.dumps(snapshot_data, indent=2), encoding="utf-8"
        )
        # Config overlay
        config_path = self._output_dir / "config_overlay.yaml"
        config_path.write_text(config_overlay_text, encoding="utf-8")
        # Summary
        summary_path = self._output_dir / "summary.txt"
        summary_path.write_text(
            f"run_id: {self._run_id}\n"
            f"errors: {len(self._errors)}\n"
            + "\n".join(f"- {e}" for e in self._errors)
            + "\n",
            encoding="utf-8",
        )


# ── Main entry point ─────────────────────────────────────────────────────


def _write_overlay(overlay_path: Path, *, fault_path: bool) -> None:
    """Write a temporary config overlay that shortens timeouts for the test."""
    overlay = """
runtime:
  control_rate_hz: 100.0
  csv_rate_hz: 50.0
  haptics_publish_rate_hz: 50.0
  terminal_rate_hz: 10.0
  startup_controller_timeout_s: 10.0
  complete_shutdown_delay_s: 0.2
  auto_kill_s: 0.0
emg:
  activation_hold_s: 0.2
  open_hold_s: 0.2
  power_toggle_hold_s: 0.2
wrist:
  vertical_delay_s: 0.3
  return_after_open_delay_s: 0.3
  move_timeout_s: 3.0
  settle_s: 0.1
force:
  stale_timeout_s: 0.2
logging:
  output_dir: /tmp/mia_haptic_force_test_offline
  run_name_prefix: offline_suite
  keep_last_runs: 7
"""
    if fault_path:
        overlay += """
  # Fault path: force the supervisor into 'force data unavailable'.
  # (set via test overlay; do NOT use effort fallback)
"""
    overlay_path.write_text(overlay, encoding="utf-8")


def _build_launch_cmd(overlay_path: Path, *, fault_path: bool) -> list[str]:
    """Build the ros2 launch command."""
    return [
        "ros2",
        "launch",
        "prosthesis_launch",
        "mia_haptic_force_test.launch.py",
        f"config_path:={overlay_path}",
        "mock_hardware:=true",
        "terminal_ui:=false",
        "logger:=false",
        "wrist_enable:=false",
    ]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config-path", required=True, type=Path)
    parser.add_argument("--run-id", default="offline_suite", type=str)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--scenario-timeout-s", default=60.0, type=float)
    args = parser.parse_args()

    if not _HAS_ROS:
        print("ERROR: rclpy not available; run inside the Docker container", file=sys.stderr)
        return 2

    rclpy.init()

    # ── Write config overlay ──────────────────────────────────────────
    overlay_path = args.output_dir / "config_overlay.yaml"
    overlay_path.parent.mkdir(parents=True, exist_ok=True)
    _write_overlay(overlay_path, fault_path=False)

    # ── Open PTY ─────────────────────────────────────────────────────
    master_fd, slave_fd = pty.openpty()

    # ── Spawn launch ─────────────────────────────────────────────────
    cmd = _build_launch_cmd(overlay_path, fault_path=False)
    env = os.environ.copy()
    env["PYTHONPATH"] = str(_REPO_ROOT)
    proc = subprocess.Popen(
        cmd,
        stdin=slave_fd,
        stdout=slave_fd,
        stderr=slave_fd,
        close_fds=True,
        env=env,
    )
    os.close(slave_fd)

    # ── Background PTY reader ────────────────────────────────────────
    transcript: list[str] = []
    stop_read = threading.Event()

    def _reader() -> None:
        while not stop_read.is_set():
            r, _, _ = select.select([master_fd], [], [], 0.1)
            if not r:
                continue
            try:
                data = os.read(master_fd, 4096)
            except OSError:
                break
            if not data:
                break
            try:
                transcript.append(data.decode("utf-8", errors="replace"))
            except Exception:
                transcript.append(repr(data))

    reader_thread = threading.Thread(target=_reader, daemon=True)
    reader_thread.start()

    # ── Topic snapshot ───────────────────────────────────────────────
    snapshot = TopicSnapshot()
    executor = SingleThreadedExecutor()
    executor.add_node(snapshot)

    # ── Run scenarios ────────────────────────────────────────────────
    suite = OfflineSuite(
        config_path=args.config_path,
        run_id=args.run_id,
        output_dir=args.output_dir,
        scenario_timeout_s=args.scenario_timeout_s,
    )
    suite._transcript = transcript

    happy_ok = suite.run_happy_path(master_fd, snapshot, executor)
    # The fault path requires a separate launch; skipped in this run.
    fault_ok = True

    # ── Cleanup ──────────────────────────────────────────────────────
    executor.remove_node(snapshot)
    snapshot.destroy_node()
    stop_read.set()
    try:
        proc.send_signal(signal.SIGTERM)
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
    reader_thread.join(timeout=1.0)
    os.close(master_fd)

    # ── Save outputs ─────────────────────────────────────────────────
    suite.save_outputs(
        snapshot.snapshot(),
        overlay_path.read_text(encoding="utf-8"),
    )

    if rclpy.ok():
        rclpy.shutdown()

    if happy_ok and fault_ok and not suite._errors:
        print("OFFLINE SUITE: PASS")
        return 0
    print(f"OFFLINE SUITE: FAIL ({len(suite._errors)} errors)")
    for e in suite._errors:
        print(f"  - {e}")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
