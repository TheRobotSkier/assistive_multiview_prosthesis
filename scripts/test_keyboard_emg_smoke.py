#!/usr/bin/env python3
"""Smoke test for the keyboard EMG emulation launch path.

Runs inside the mia-haptic-force-test container as the test entrypoint.
Starts ``ros2 launch prosthesis_launch mia_haptic_force_test.launch.py`` as
a subprocess, subscribes to ``/test/stage`` and watches for the supervisor
to reach the ``waiting_for_activation`` stage (the point where the legacy
monolithic test would normally wait for a sustained POWER gesture before
beginning the pipeline).

Behaviour:
  * On first ``waiting_for_activation`` stage message -> verify a minimum
    topic-contract set has produced at least one message each, then
    ``SIGTERM`` the launch, wait for it to exit, exit 0.  This proves the
    full graph (multi-node stack + hardware drivers + controller_manager)
    came up and every required topic is actually publishing.
  * After ``--timeout-s`` seconds without seeing that stage -> terminate
    the launch, exit 1. Catches rclpy ImportErrors, controller_manager
    never becoming available, etc.
  * ``Ctrl-C`` / ``SIGTERM`` from outside -> forward to the launch, exit 2.
"""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int32, String


# Topics that must each receive at least one message before the test is
# considered healthy.  These are the head/heart of the multi-node
# contract — if any of these never publishes, downstream state will be
# missing and the test graph is broken even if /test/stage advances.
_REQUIRED_TOPICS: list[tuple[str, type]] = [
    ("/test/stage",         String),
    ("/emg/gesture_name",   String),
    ("/emg/gesture_label",  Int32),
    ("/emg/confidence",     Float32),
    ("/emg/proportional",   Float32),
    ("/emg/source",         String),
    ("/hand/forces",        Float32MultiArray),
    ("/hand/joint_states",  JointState),
    ("/control/mode",       String),
    ("/controller/active",  Bool),
]


@dataclass
class _State:
    proc: Optional[subprocess.Popen] = None


_STATE = _State()


def _build_launch_cmd(extra_args: str) -> list[str]:
    cmd = ["ros2", "launch", "prosthesis_launch", "mia_haptic_force_test.launch.py"]
    if extra_args.strip():
        cmd.extend(extra_args.split())
    return cmd


def _forward_signal(signum: int, frame) -> None:  # noqa: ARG001
    proc = _STATE.proc
    if proc is not None and proc.poll() is None:
        try:
            proc.send_signal(signum)
        except ProcessLookupError:
            pass


class StageWatcher(Node):
    def __init__(self, topic: str, target_stage: str) -> None:
        super().__init__("keyboard_emg_smoke_watcher")
        self._target_stage = target_stage
        self._last_stage: str = ""
        self._event = threading.Event()
        self._seen: dict[str, bool] = {t: False for t, _ in _REQUIRED_TOPICS}
        self._seen_lock = threading.Lock()
        self.create_subscription(String, topic, self._on_stage, 10)
        for t, msg_type in _REQUIRED_TOPICS:
            self.create_subscription(msg_type, t, self._mark_seen(t), 10)

    @property
    def seen_target(self) -> bool:
        return self._event.is_set()

    def missing_topics(self) -> list[str]:
        with self._seen_lock:
            return [t for t, seen in self._seen.items() if not seen]

    def _mark_seen(self, topic: str):
        def cb(_msg) -> None:
            with self._seen_lock:
                if not self._seen[topic]:
                    self._seen[topic] = True
                    self.get_logger().info("topic: %s first message received" % topic)
        return cb

    def _on_stage(self, msg: String) -> None:
        stage = msg.data.strip()
        if not stage:
            return
        if stage != self._last_stage:
            self.get_logger().info("stage: %s" % stage)
            self._last_stage = stage
        if stage == self._target_stage:
            self._event.set()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--launch-args",
        default=os.environ.get(
            "TEST_GRASP_LAUNCH_ARGS",
            "config_path:=/prosthesis_ws/config/mia_haptic_force_test.yaml "
            "emg_enable:=false keyboard_emg:=true use_multi_node:=true "
            "wrist_enable:=false mock_hardware:=true haptic_enable:=false",
        ),
        help="Whitespace-separated name:=value launch args forwarded to ros2 launch.",
    )
    parser.add_argument("--timeout-s", type=float, default=20.0)
    parser.add_argument("--stage-topic", default="/test/stage")
    parser.add_argument("--target-stage", default="waiting_for_activation")
    args = parser.parse_args()

    cmd = _build_launch_cmd(args.launch_args)
    print(f"[smoke] launching: {' '.join(cmd)}", flush=True)

    # Preserve the current environment (this process is started after sourcing
    # both /opt/ros/jazzy/setup.bash and /prosthesis_ws/install/setup.bash).
    _STATE.proc = subprocess.Popen(cmd)

    signal.signal(signal.SIGINT, _forward_signal)
    signal.signal(signal.SIGTERM, _forward_signal)

    rclpy.init(args=None)
    watcher = StageWatcher(args.stage_topic, args.target_stage)
    print(f"[smoke] watching topic: {args.stage_topic}", flush=True)
    print(f"[smoke] target stage:   {args.target_stage}", flush=True)
    print(f"[smoke] timeout:        {args.timeout_s}s", flush=True)
    print(
        f"[smoke] required topics ({len(_REQUIRED_TOPICS)}): "
        + ", ".join(t for t, _ in _REQUIRED_TOPICS),
        flush=True,
    )

    deadline = time.monotonic() + args.timeout_s
    try:
        while time.monotonic() < deadline:
            proc = _STATE.proc
            if proc is not None and proc.poll() is not None:
                rc = proc.returncode
                print(f"[smoke] launch exited early with code {rc}", flush=True)
                return 1 if rc != 0 else 0

            if watcher.seen_target:
                missing = watcher.missing_topics()
                if missing:
                    print(
                        "[smoke] ✗ reached target stage but missing required topics: "
                        + ", ".join(missing),
                        flush=True,
                    )
                    return 1
                print(
                    f"[smoke] ✓ reached '{args.target_stage}' — all "
                    f"{len(_REQUIRED_TOPICS)} required topics published",
                    flush=True,
                )
                print(
                    "[smoke] sending SIGTERM to ros2 launch (keyboard emu idle -> no POWER)",
                    flush=True,
                )
                if proc is not None and proc.poll() is None:
                    proc.terminate()
                    try:
                        proc.wait(timeout=10.0)
                    except subprocess.TimeoutExpired:
                        print(
                            "[smoke] launch did not exit after SIGTERM; sending SIGKILL",
                            flush=True,
                        )
                        proc.kill()
                        try:
                            proc.wait(timeout=5.0)
                        except subprocess.TimeoutExpired:
                            proc.kill()
                return 0

            rclpy.spin_once(watcher, timeout_sec=0.1)

        missing = watcher.missing_topics()
        print(
            f"[smoke] ✗ timed out after {args.timeout_s}s without seeing "
            f"'{args.target_stage}'; missing topics: "
            + (", ".join(missing) if missing else "<none yet>"),
            flush=True,
        )
        return 1
    finally:
        try:
            watcher.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
        proc = _STATE.proc
        if proc is not None and proc.poll() is None:
            try:
                proc.terminate()
                proc.wait(timeout=5.0)
            except Exception:
                try:
                    proc.kill()
                except Exception:
                    pass


if __name__ == "__main__":
    sys.exit(main())
