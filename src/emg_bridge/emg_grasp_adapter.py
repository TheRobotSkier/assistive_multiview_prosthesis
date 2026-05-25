#!/usr/bin/env python3
"""EMG grasp adapter: consumes trained classifier output and exposes normalized signal.

Subscribes to:
  /emg/gesture_name    std_msgs/String
  /emg/confidence       std_msgs/Float32
  /emg/proportional     std_msgs/Float32

Normalizes gestures to canonical set: OPEN, POWER, FLEXION, EXTENSION.
Ignores low-confidence or stale data. Tracks hold duration.
"""
from __future__ import annotations

import time
import threading
from dataclasses import dataclass, field
from typing import Dict, Optional

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32, String
    from rclpy.qos import QoSProfile, DurabilityPolicy
    HAS_ROS = True
except ImportError:
    HAS_ROS = False

    # Stubs so type-checking works without ROS installed
    class Node:  # type: ignore[misc]
        pass

    class Float32:  # type: ignore[no-redef]
        def __init__(self, data: float = 0.0):
            self.data = data

    class String:  # type: ignore[no-redef]
        def __init__(self, data: str = ""):
            self.data = data


@dataclass
class EmgSignal:
    """Normalized EMG signal ready for the grasp state machine.

    Attributes:
        normalized_gesture: canonical gesture name (NEUTRAL, OPEN, POWER,
            FLEXION, EXTENSION).
        confidence: classifier confidence of the raw gesture [0.0, 1.0].
        control_value: proportional / intensity value [0.0, 1.0].
        timestamp: time stamp of the last processed message (seconds).
        hold_duration: how long the current raw gesture has been held
            continuously (seconds).
        is_valid: True only when the signal passes all confidence, stale,
            and safety gates.
        raw_gesture: original gesture name from the classifier.
    """
    normalized_gesture: str = "NEUTRAL"
    confidence: float = 0.0
    control_value: float = 0.0
    timestamp: float = 0.0
    hold_duration: float = 0.0
    is_valid: bool = False
    raw_gesture: str = "REST"


class EmgGraspAdapter:
    """Adapter between raw EMG classifier output and canonical grasp signals.

    Configurable gesture aliases allow legacy names (POINT, PINCH) to map to
    canonical wrist names (FLEXION, EXTENSION).  The adapter enforces
    confidence thresholds, stale-data timeouts, and an OPEN safety gate so
    that low-quality or ambiguous EMG cannot accidentally command motion.

    Usage (standalone):
        adapter = EmgGraspAdapter(aliases={"PINCH": "FLEXION", "POINT": "EXTENSION"})
        signal = adapter.update_raw(gesture_name="PINCH", confidence=0.9, proportional=0.5)

    Usage (ROS 2):
        adapter = EmgGraspAdapter(node=ros_node)
        signal = adapter.get_signal()   # poll in your control loop
    """

    # Canonical gestures recognized by the grasp controller
    CANONICAL_GESTURES = frozenset(["OPEN", "POWER", "FLEXION", "EXTENSION"])
    NEUTRAL_GESTURE = "NEUTRAL"

    def __init__(
        self,
        *,
        aliases: Optional[Dict[str, str]] = None,
        confidence_threshold: float = 0.55,
        stale_timeout_s: float = 0.5,
        open_min_control: float = 0.7,
        open_min_hold_s: float = 1.0,
        node: Optional[Node] = None,
    ):
        """
        Args:
            aliases: mapping from raw gesture name -> canonical gesture name.
                     e.g. {"PINCH": "FLEXION", "POINT": "EXTENSION"}
            confidence_threshold: minimum confidence for a gesture to be valid.
            stale_timeout_s: if no new EMG data arrives within this window,
                             the adapter returns a neutral signal.
            open_min_control: minimum proportional value for OPEN to be valid.
            open_min_hold_s: minimum hold duration for OPEN to be valid.
            node: optional ROS 2 Node.  If provided, the adapter sets up
                  subscriptions automatically.
        """
        self._aliases: Dict[str, str] = aliases if aliases is not None else {}
        self._confidence_threshold = confidence_threshold
        self._stale_timeout_s = stale_timeout_s
        self._open_min_control = open_min_control
        self._open_min_hold_s = open_min_hold_s

        # Thread safety for shared state
        self._lock = threading.Lock()

        # Internal state
        self._current_raw: str = "REST"
        self._current_conf: float = 0.0
        self._current_prop: float = 0.0
        self._current_signal: EmgSignal = EmgSignal()
        self._last_msg_time: float = time.time()
        self._gesture_start_time: float = time.time()

        # ROS subscriptions (if node provided)
        self._node = node
        if node is not None:
            self._setup_ros(node)

    # ── ROS setup (only when used inside a ROS node) ──────────────────────────

    def _setup_ros(self, node: Node) -> None:
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(String, "/emg/gesture_name", self._on_ros_name, qos)
        node.create_subscription(Float32, "/emg/confidence", self._on_ros_conf, qos)
        node.create_subscription(Float32, "/emg/proportional", self._on_ros_prop, qos)

    def _on_ros_name(self, msg: String) -> None:
        self.update_raw(gesture_name=msg.data)

    def _on_ros_conf(self, msg: Float32) -> None:
        self.update_raw(confidence=msg.data)

    def _on_ros_prop(self, msg: Float32) -> None:
        self.update_raw(proportional=msg.data)

    # ── Public update API ─────────────────────────────────────────────────────

    def update_raw(
        self,
        *,
        gesture_name: Optional[str] = None,
        confidence: Optional[float] = None,
        proportional: Optional[float] = None,
    ) -> EmgSignal:
        """Feed new raw data (from ROS or a test harness) and return current signal.

        Thread-safe.
        """
        now = time.time()
        with self._lock:
            if gesture_name is not None:
                upper = gesture_name.upper().strip()
                if upper != self._current_raw:
                    self._current_raw = upper
                    self._gesture_start_time = now
            if confidence is not None:
                self._current_conf = float(confidence)
            if proportional is not None:
                self._current_prop = float(proportional)
            self._last_msg_time = now
            self._current_signal = self._compute_signal(now)
        return self._current_signal

    def get_signal(self) -> EmgSignal:
        """Return the current normalized signal (thread-safe).

        Re-evaluates stale-data condition on every call.
        """
        now = time.time()
        with self._lock:
            self._current_signal = self._compute_signal(now)
            return self._current_signal

    # ── Normalization & safety logic ─────────────────────────────────────────

    def _compute_signal(self, now: float) -> EmgSignal:
        # ── stale data guard ─────────────────────────────────────────────────
        if (now - self._last_msg_time) > self._stale_timeout_s:
            return EmgSignal(
                normalized_gesture=self.NEUTRAL_GESTURE,
                confidence=0.0,
                control_value=0.0,
                timestamp=now,
                hold_duration=0.0,
                is_valid=False,
                raw_gesture=self._current_raw,
            )

        # ── gesture name alias mapping ─────────────────────────────────────
        raw = self._current_raw
        canonical = self._aliases.get(raw, raw)

        # Anything outside the canonical set (including REST) becomes NEUTRAL
        if canonical not in self.CANONICAL_GESTURES:
            canonical = self.NEUTRAL_GESTURE

        hold = now - self._gesture_start_time

        # ── low-confidence guard ───────────────────────────────────────────
        if self._current_conf < self._confidence_threshold:
            return EmgSignal(
                normalized_gesture=self.NEUTRAL_GESTURE,
                confidence=self._current_conf,
                control_value=self._current_prop,
                timestamp=now,
                hold_duration=hold,
                is_valid=False,
                raw_gesture=raw,
            )

        # ── OPEN safety gate ─────────────────────────────────────────────────
        # OPEN is the "release / open hand" command.  We require a strong
        # proportional signal and a sustained hold so that spurious classifier
        # flickers cannot accidentally open the hand during grasp.
        if canonical == "OPEN":
            if self._current_prop < self._open_min_control or hold < self._open_min_hold_s:
                return EmgSignal(
                    normalized_gesture=self.NEUTRAL_GESTURE,
                    confidence=self._current_conf,
                    control_value=self._current_prop,
                    timestamp=now,
                    hold_duration=hold,
                    is_valid=False,
                    raw_gesture=raw,
                )

        # ── valid canonical gesture ──────────────────────────────────────────
        # NEUTRAL (including REST and unmapped gestures) is never a valid command.
        is_valid = canonical != self.NEUTRAL_GESTURE
        return EmgSignal(
            normalized_gesture=canonical,
            confidence=self._current_conf,
            control_value=self._current_prop,
            timestamp=now,
            hold_duration=hold,
            is_valid=is_valid,
            raw_gesture=raw,
        )
