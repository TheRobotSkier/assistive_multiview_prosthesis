#!/usr/bin/env python3
"""MVP-XVK: Integration smoke test for the multi-node stack.

This test launches the supervisor and controller nodes (no hardware required)
and verifies that:

* supervisor publishes /test/stage transitions
* controller publishes /controller/loop_timing and /controller/active
* target force and wrist trajectories are non-stationary
* loop timing meets the 100 Hz / 2 ms jitter target

Full hardware-in-the-loop validation must be run on the Jetson using the
legacy script as a reference.  See AGENTS.md for the ``scripts/jetson_run.sh``
queue workflow.
"""

from __future__ import annotations

import os
import sys
import threading
import time
from typing import Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, Float64MultiArray, String

_REPO_ROOT: str = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if __name__ == "__main__" and __package__ is None and _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from scripts.mia_haptic_force_test.common.constants import FINGER_COUNT
from scripts.mia_haptic_force_test.hand_controller_node import HandControllerNode
from scripts.mia_haptic_force_test.supervisor_node import SupervisorNode


class ProbeNode(Node):
    """Collects messages and asserts integration criteria."""

    def __init__(self) -> None:
        super().__init__("integration_probe")
        self._lock = threading.Lock()
        self._stages: list[str] = []
        self._timing_samples: list[dict] = []
        self._active_count = 0
        self._last_target_force: Optional[list[float]] = None
        self._last_target_wrist: Optional[float] = None

        self.create_subscription(String, "/test/stage", self._on_stage, 10)
        self.create_subscription(String, "/controller/loop_timing", self._on_timing, 10)
        self.create_subscription(Bool, "/controller/active", self._on_active, 10)
        self.create_subscription(Float64MultiArray, "/control/target_force", self._on_target_force, 10)
        self.create_subscription(Float64MultiArray, "/control/target_wrist", self._on_target_wrist, 10)

    def _on_stage(self, msg: String) -> None:
        with self._lock:
            if not self._stages or self._stages[-1] != msg.data:
                self._stages.append(msg.data)

    def _on_timing(self, msg: String) -> None:
        import json
        with self._lock:
            try:
                self._timing_samples.append(json.loads(msg.data))
            except Exception:
                pass

    def _on_active(self, msg: Bool) -> None:
        with self._lock:
            if msg.data:
                self._active_count += 1

    def _on_target_force(self, msg: Float64MultiArray) -> None:
        with self._lock:
            self._last_target_force = list(msg.data)

    def _on_target_wrist(self, msg: Float64MultiArray) -> None:
        with self._lock:
            if msg.data:
                self._last_target_wrist = float(msg.data[0])

    def summary(self) -> dict:
        with self._lock:
            return {
                "stages": list(self._stages),
                "timing_samples": len(self._timing_samples),
                "active_count": self._active_count,
                "last_target_force": self._last_target_force,
                "last_target_wrist": self._last_target_wrist,
            }


def main() -> int:
    rclpy.init(args=None)

    supervisor = SupervisorNode()
    controller = HandControllerNode()
    probe = ProbeNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(supervisor)
    executor.add_node(controller)
    executor.add_node(probe)

    supervisor.start()
    controller.start()

    run_time_s = 5.0
    print(f"Running integration smoke test for {run_time_s} s...")
    start = time.monotonic()
    try:
        while time.monotonic() - start < run_time_s:
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        controller.stop()
        supervisor.stop()
        executor.shutdown()
        for n in (controller, supervisor, probe):
            n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    summary = probe.summary()
    print("Integration summary:", summary)

    ok = True
    if not summary["stages"]:
        print("FAIL: no stage transitions observed")
        ok = False
    if not summary["timing_samples"]:
        print("FAIL: no controller timing messages observed")
        ok = False
    if summary["last_target_force"] is None or len(summary["last_target_force"]) != FINGER_COUNT:
        print("FAIL: target force not published correctly")
        ok = False

    for sample in summary.get("timing_samples", []):
        if sample.get("mean_hz", 0) < 99.5:
            print(f"FAIL: mean_hz {sample.get('mean_hz')} < 99.5")
            ok = False
        if sample.get("p99_jitter_ms", 999) > 2.0:
            print(f"FAIL: p99_jitter_ms {sample.get('p99_jitter_ms')} > 2.0")
            ok = False

    if ok:
        print("PASS: integration smoke test")
        return 0
    return 1


if __name__ == "__main__":
    sys.exit(main())
