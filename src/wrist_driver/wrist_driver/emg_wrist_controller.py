#!/usr/bin/env python3
"""EMG-driven bounded wrist velocity controller.

Subscribes to EMG gesture/confidence topics and the pipeline state to decide
when to move the wrist.  Since the wrist driver is position-oriented, this
controller emulates velocity control by sending bounded position increments
at a configurable command rate.

Activation:
  FLEXION / EXTENSION gestures move the wrist ONLY when the pipeline is in a
  wrist-permissive state (IDLE, RELEASING by default — configurable).
  In control-grasp modes (APPROACHING, GRASPING, HOLDING) wrist motion is
  suppressed; FLEXION/EXTENSION is reserved for other consumers (e.g. force
  target adjustment).

Stop conditions (any of these halts wrist motion immediately):
  - Gesture changes to REST (neutral) or to a non-wrist gesture.
  - OPEN gesture (release override).
  - Pipeline mode changes away from the configured wrist-permissive set.
  - Wrist state data is stale (hardware disconnected / timeout).
  - Wrist position reaches the configured min or max limit.
  - Node shutdown.

Subscribes:
  /emg/gesture_label   std_msgs/Int32
  /emg/confidence      std_msgs/Float32
  /emg/gesture_name    std_msgs/String (optional, for debug logging)
  /pipeline/state      std_msgs/Int32
  /wrist/state         std_msgs/Float64MultiArray [position_deg, velocity_deg_s]

Publishes:
  /wrist/set_position  std_msgs/Float64MultiArray [position_deg, acceleration_deg_s2]
"""

from __future__ import annotations

import sys
import time
from typing import Optional, Set

import yaml

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64MultiArray, Float32, Int32, String
except ModuleNotFoundError:
    # Stubs for environments without ROS2 installed (unit tests)
    class Node:  # type: ignore[no-redef,misc]
        pass

    class Float64MultiArray:  # type: ignore[no-redef]
        def __init__(self, data=None):
            self.data = data if data is not None else []

    class Float32:  # type: ignore[no-redef]
        def __init__(self, data=0.0):
            self.data = data

    class Int32:  # type: ignore[no-redef]
        def __init__(self, data=0):
            self.data = data

    class String:  # type: ignore[no-redef]
        def __init__(self, data=""):
            self.data = data


# ── Gesture label constants (mirrored from emg_bridge/run_classifier.py) ──────
GESTURE_REST = 0
GESTURE_POWER = 1
GESTURE_PINCH = 2
GESTURE_OPEN = 3
GESTURE_POINT = 4
GESTURE_FLEXION = 5
GESTURE_EXTENSION = 6

GESTURE_NAMES = {
    0: "REST",
    1: "POWER",
    2: "PINCH",
    3: "OPEN",
    4: "POINT",
    5: "FLEXION",
    6: "EXTENSION",
}

# ── Pipeline state constants (mirrored from pipeline_manager) ─────────────────
PIPELINE_IDLE = 0
PIPELINE_SEGMENTING = 1
PIPELINE_PLANNING = 2
PIPELINE_APPROACHING = 3
PIPELINE_GRASPING = 4
PIPELINE_HOLDING = 5
PIPELINE_RELEASING = 6

PIPELINE_STATE_NAMES = {
    0: "IDLE",
    1: "SEGMENTING",
    2: "PLANNING",
    3: "APPROACHING",
    4: "GRASPING",
    5: "HOLDING",
    6: "RELEASING",
}

# Default wrist-permissive pipeline states (wrist motion allowed)
DEFAULT_WRIST_PERMISSIVE_STATES: Set[int] = {
    PIPELINE_IDLE,
    PIPELINE_RELEASING,
}


class EmgWristController(Node):
    """Bounded wrist velocity controller driven by EMG gestures.

    Translates sustained FLEXION/EXTENSION gestures into position increments
    sent to the wrist driver at a fixed command rate.  Motion is bounded by
    configurable min/max angles and gated by pipeline state.
    """

    def __init__(self):
        super().__init__("emg_wrist_controller")

        # ── Config ─────────────────────────────────────────────────────────
        config_path = self.declare_parameter(
            "config_path", "/prosthesis_ws/tests/emg_grasp/emg_grasp_test.yaml"
        ).value
        with open(config_path) as f:
            cfg = yaml.safe_load(f)

        wc = cfg.get("wrist_control", {})
        self._enabled: bool = bool(wc.get("enabled", False))
        self._flexion_gesture: int = int(wc.get("flexion_gesture", GESTURE_FLEXION))
        self._extension_gesture: int = int(wc.get("extension_gesture", GESTURE_EXTENSION))
        self._confidence_threshold: float = float(wc.get("confidence_threshold", 0.55))
        self._positive_speed_deg_s: float = float(wc.get("positive_speed_deg_s", 30.0))
        self._negative_speed_deg_s: float = float(wc.get("negative_speed_deg_s", -30.0))
        self._step_size_deg: float = float(wc.get("step_size_deg", 1.5))
        self._command_rate_hz: float = float(wc.get("command_rate_hz", 20.0))
        self._min_position_deg: float = float(wc.get("min_position_deg", -90.0))
        self._max_position_deg: float = float(wc.get("max_position_deg", 90.0))
        self._stale_state_timeout_s: float = float(wc.get("stale_state_timeout_s", 0.5))
        self._stop_on_mode_change: bool = bool(wc.get("stop_on_mode_change", True))
        self._fail_preflight_on_missing_wrist: bool = bool(
            wc.get("fail_preflight_on_missing_wrist", False)
        )
        self._accel_deg_s2: float = float(wc.get("wrist_accel_deg_s2", 180.0))
        self._wrist_permissive_states: Set[int] = set(
            int(s) for s in wc.get("wrist_permissive_states",
                                    list(DEFAULT_WRIST_PERMISSIVE_STATES))
        )

        # Topic overrides from config (with sensible defaults)
        self._gesture_topic = cfg.get("emg_gesture_topic", "/emg/gesture_label")
        self._confidence_topic = cfg.get("emg_confidence_topic", "/emg/confidence")
        self._gesture_name_topic = cfg.get("emg_gesture_name_topic", "/emg/gesture_name")
        self._pipeline_state_topic = cfg.get("pipeline_state_topic", "/pipeline/state")
        self._wrist_state_topic = cfg.get("wrist_state_topic", "/wrist/state")
        self._wrist_cmd_topic = cfg.get("wrist_cmd_topic", "/wrist/set_position")

        self._release_gesture = int(cfg.get("emg_release_gesture", GESTURE_OPEN))

        if not self._enabled:
            self.get_logger().info("Wrist control disabled in config — node is idle")
            return

        # ── Internal state ──────────────────────────────────────────────────
        self._current_gesture: int = GESTURE_REST
        self._current_confidence: float = 0.0
        self._current_gesture_name: str = "REST"
        self._pipeline_state: int = PIPELINE_IDLE
        self._prev_pipeline_state: int = PIPELINE_IDLE
        self._wrist_position_deg: float = 0.0
        self._wrist_velocity_deg_s: float = 0.0
        self._last_wrist_state_time: float = 0.0
        self._wrist_state_received: bool = False
        self._active_command_deg: Optional[float] = None
        self._stopped: bool = False

        # ── Subscribers ────────────────────────────────────────────────────
        self.create_subscription(Int32, self._gesture_topic, self._on_gesture, 10)
        self.create_subscription(Float32, self._confidence_topic, self._on_confidence, 10)
        self.create_subscription(String, self._gesture_name_topic, self._on_gesture_name, 10)
        self.create_subscription(Int32, self._pipeline_state_topic, self._on_pipeline_state, 10)
        self.create_subscription(
            Float64MultiArray, self._wrist_state_topic, self._on_wrist_state, 10
        )

        # ── Publisher ──────────────────────────────────────────────────────
        self._wrist_cmd_pub = self.create_publisher(
            Float64MultiArray, self._wrist_cmd_topic, 10
        )

        # ── Timer ──────────────────────────────────────────────────────────
        self._dt = 1.0 / self._command_rate_hz
        self._timer = self.create_timer(self._dt, self._control_loop)

        self.get_logger().info(
            f"EMG wrist controller started — "
            f"flexion={self._flexion_gesture}, extension={self._extension_gesture}, "
            f"command_rate={self._command_rate_hz} Hz, "
            f"bounds=[{self._min_position_deg}, {self._max_position_deg}] deg, "
            f"permissive_states={[PIPELINE_STATE_NAMES.get(s, str(s)) for s in sorted(self._wrist_permissive_states)]}"
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _on_gesture(self, msg: Int32):
        prev = self._current_gesture
        self._current_gesture = msg.data
        if prev != msg.data:
            self.get_logger().debug(
                f"Gesture: {GESTURE_NAMES.get(prev, str(prev))} -> "
                f"{GESTURE_NAMES.get(msg.data, str(msg.data))}"
            )

    def _on_confidence(self, msg: Float32):
        self._current_confidence = msg.data

    def _on_gesture_name(self, msg: String):
        self._current_gesture_name = msg.data

    def _on_pipeline_state(self, msg: Int32):
        self._prev_pipeline_state = self._pipeline_state
        self._pipeline_state = msg.data

    def _on_wrist_state(self, msg: Float64MultiArray):
        if len(msg.data) >= 1:
            self._wrist_position_deg = float(msg.data[0])
        if len(msg.data) >= 2:
            self._wrist_velocity_deg_s = float(msg.data[1])
        self._last_wrist_state_time = time.time()
        self._wrist_state_received = True

    # ── Control logic ─────────────────────────────────────────────────────────

    def _is_wrist_permissive(self) -> bool:
        """Return True if the current pipeline state allows wrist motion."""
        return self._pipeline_state in self._wrist_permissive_states

    def _check_stop_conditions(self) -> Optional[str]:
        """Evaluate all stop conditions.  Returns reason string if stop needed."""

        # Disabled — never run
        if not self._enabled:
            return "disabled"

        # Missing wrist hardware — discovered during preflight
        if not self._wrist_state_received:
            if self._fail_preflight_on_missing_wrist:
                self.get_logger().error(
                    "No wrist state received — wrist hardware appears missing. "
                    "Set fail_preflight_on_missing_wrist=false to suppress this error."
                )
                raise RuntimeError("Wrist hardware missing — preflight failed")
            else:
                self.get_logger().warn(
                    "No wrist state received yet — wrist may be missing. "
                    "Wrist control disabled until state arrives."
                )
                self._enabled = False
                return "missing wrist hardware"

        # Stale wrist state
        if self._last_wrist_state_time > 0:
            age = time.time() - self._last_wrist_state_time
            if age > self._stale_state_timeout_s:
                return f"stale wrist state ({age:.2f}s > {self._stale_state_timeout_s}s)"

        # Gesture is REST (neutral)
        if self._current_gesture == GESTURE_REST:
            return "neutral EMG (REST)"

        # Gesture is OPEN (release override)
        if self._current_gesture == self._release_gesture:
            return f"release gesture ({GESTURE_NAMES.get(self._release_gesture, str(self._release_gesture))})"

        # Gesture is not a wrist-controlling gesture
        if self._current_gesture not in (self._flexion_gesture, self._extension_gesture):
            return (
                f"non-wrist gesture "
                f"({GESTURE_NAMES.get(self._current_gesture, str(self._current_gesture))})"
            )

        # Confidence below threshold
        if self._current_confidence < self._confidence_threshold:
            return (
                f"low confidence ({self._current_confidence:.2f} < "
                f"{self._confidence_threshold})"
            )

        # Mode changed (if stop-on-mode-change is enabled)
        # Checked BEFORE permissive-state so mode-change fires when transitioning
        # to a non-permissive state (more specific reason than "not in permissive").
        if self._stop_on_mode_change and self._pipeline_state != self._prev_pipeline_state:
            return (
                f"mode changed: {PIPELINE_STATE_NAMES.get(self._prev_pipeline_state)} "
                f"-> {PIPELINE_STATE_NAMES.get(self._pipeline_state)}"
            )

        # Not in a wrist-permissive pipeline state
        if not self._is_wrist_permissive():
            return (
                f"pipeline state {PIPELINE_STATE_NAMES.get(self._pipeline_state, str(self._pipeline_state))} "
                f"not in permissive set"
            )

        # At position limit
        if self._current_gesture == self._flexion_gesture:
            if self._wrist_position_deg >= self._max_position_deg:
                return f"at max position ({self._wrist_position_deg:.1f} >= {self._max_position_deg})"
        elif self._current_gesture == self._extension_gesture:
            if self._wrist_position_deg <= self._min_position_deg:
                return f"at min position ({self._wrist_position_deg:.1f} <= {self._min_position_deg})"

        return None  # No stop condition

    def _control_loop(self):
        """Main control loop — runs at the configured command rate."""
        stop_reason = self._check_stop_conditions()

        if stop_reason is not None:
            if not self._stopped:
                self.get_logger().info(f"Wrist motion stopped: {stop_reason}")
                self._stopped = True
                self._active_command_deg = None
            # Do NOT send any command when stopped
            return

        # ── Active wrist motion ────────────────────────────────────────────
        self._stopped = False

        # Determine direction and compute new position
        if self._current_gesture == self._flexion_gesture:
            step = self._step_size_deg
        elif self._current_gesture == self._extension_gesture:
            step = -self._step_size_deg
        else:
            return  # Should not happen (caught by stop conditions)

        # Compute target: current position + step, clamped to bounds
        target = self._wrist_position_deg + step
        target = max(self._min_position_deg, min(self._max_position_deg, target))

        # Publish position command with acceleration
        msg = Float64MultiArray()
        msg.data = [float(target), float(self._accel_deg_s2)]
        self._wrist_cmd_pub.publish(msg)
        self._active_command_deg = target

    # ── Public API (for testing) ──────────────────────────────────────────────

    @property
    def active_command_deg(self) -> Optional[float]:
        return self._active_command_deg

    @property
    def stopped(self) -> bool:
        return self._stopped

    @property
    def wrist_position_deg(self) -> float:
        return self._wrist_position_deg

    @property
    def current_gesture(self) -> int:
        return self._current_gesture

    @property
    def pipeline_state(self) -> int:
        return self._pipeline_state

    @property
    def enabled(self) -> bool:
        return self._enabled

    # ── Static helpers (for testing) ──────────────────────────────────────────

    @staticmethod
    def check_stop_conditions(
        *,
        enabled: bool,
        current_gesture: int,
        current_confidence: float,
        pipeline_state: int,
        prev_pipeline_state: int,
        wrist_position_deg: float,
        last_wrist_state_time: float,
        wrist_state_received: bool,
        fail_preflight_on_missing_wrist: bool,
        flexion_gesture: int,
        extension_gesture: int,
        release_gesture: int,
        confidence_threshold: float,
        min_position_deg: float,
        max_position_deg: float,
        stale_state_timeout_s: float,
        stop_on_mode_change: bool,
        wrist_permissive_states: Set[int],
        now: float,
    ) -> Optional[str]:
        """Static version of stop-condition evaluation (no ROS dependencies).

        Returns a reason string if any stop condition fires, else None.
        """
        if not enabled:
            return "disabled"

        if not wrist_state_received:
            if fail_preflight_on_missing_wrist:
                raise RuntimeError("Wrist hardware missing — preflight failed")
            return "missing wrist hardware"

        if last_wrist_state_time > 0:
            age = now - last_wrist_state_time
            if age > stale_state_timeout_s:
                return f"stale wrist state ({age:.2f}s > {stale_state_timeout_s}s)"

        if current_gesture == GESTURE_REST:
            return "neutral EMG (REST)"

        if current_gesture == release_gesture:
            return f"release gesture ({GESTURE_NAMES.get(release_gesture, str(release_gesture))})"

        if current_gesture not in (flexion_gesture, extension_gesture):
            return (
                f"non-wrist gesture "
                f"({GESTURE_NAMES.get(current_gesture, str(current_gesture))})"
            )

        if current_confidence < confidence_threshold:
            return f"low confidence ({current_confidence:.2f} < {confidence_threshold})"

        if stop_on_mode_change and pipeline_state != prev_pipeline_state:
            return (
                f"mode changed: "
                f"{PIPELINE_STATE_NAMES.get(prev_pipeline_state)} -> "
                f"{PIPELINE_STATE_NAMES.get(pipeline_state)}"
            )

        if pipeline_state not in wrist_permissive_states:
            return (
                f"pipeline state {PIPELINE_STATE_NAMES.get(pipeline_state, str(pipeline_state))} "
                f"not in permissive set"
            )

        if current_gesture == flexion_gesture and wrist_position_deg >= max_position_deg:
            return f"at max position ({wrist_position_deg:.1f} >= {max_position_deg})"

        if current_gesture == extension_gesture and wrist_position_deg <= min_position_deg:
            return f"at min position ({wrist_position_deg:.1f} <= {min_position_deg})"

        return None

    @staticmethod
    def compute_wrist_command(
        *,
        current_gesture: int,
        wrist_position_deg: float,
        step_size_deg: float,
        min_position_deg: float,
        max_position_deg: float,
        flexion_gesture: int = GESTURE_FLEXION,
        extension_gesture: int = GESTURE_EXTENSION,
    ) -> Optional[float]:
        """Compute the next wrist position command given a gesture.

        Returns the clamped target position in degrees, or None if gesture
        is not a wrist-controlling gesture.
        """
        if current_gesture == flexion_gesture:
            target = wrist_position_deg + step_size_deg
        elif current_gesture == extension_gesture:
            target = wrist_position_deg - step_size_deg
        else:
            return None
        return max(min_position_deg, min(max_position_deg, target))


def main(args=None):
    rclpy.init(args=args)
    node = EmgWristController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
