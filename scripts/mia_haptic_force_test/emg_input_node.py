#!/usr/bin/env python3
"""MVP-HJ3 + MVP-8UV.4: emg_input_node.

Wraps the existing emg_bridge board reader and classifier, predicts gesture at
a configurable rate and publishes on /emg/* topics.

Publishes::

    /emg/gesture_name  (std_msgs/String)   human-readable gesture name
    /emg/gesture_label (std_msgs/Int32)   numeric class label
    /emg/confidence    (std_msgs/Float32)  classifier confidence
    /emg/proportional  (std_msgs/Float32)  continuous 0-1 proportional value
    /emg/source        (std_msgs/String)   "hardware" or "simulated"
    /emg/age           (std_msgs/Float32)  seconds since last valid EMG reading
"""

from __future__ import annotations

import argparse
import os
import signal
import sys
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, String

_REPO_ROOT: str = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if __name__ == "__main__" and __package__ is None and _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from scripts.mia_haptic_force_test.common.constants import GESTURES, REST_GESTURE_LABEL, TOPIC_EMG_GESTURE

# Default EMG hardware settings (overridable via config YAML under emg:)
_DEFAULT_BOARD_IP: str = "10.27.30.3"
_DEFAULT_BOARD_PORT: int = 4210
_DEFAULT_CLASSIFIER_MODEL_PATH: str = os.path.join(_REPO_ROOT, "models")
_DEFAULT_PUBLISH_RATE_HZ: float = 50.0


class EmgInputNode(Node):
    """MindRove EMG input + gesture prediction node."""

    def __init__(self, config_path: Optional[str] = None) -> None:
        super().__init__("emg_input_node")
        self._config_path = config_path or os.path.join(
            _REPO_ROOT, "config", "mia_haptic_force_test.yaml"
        )

        # ── Load EMG config from YAML ────────────────────────────────────────
        emg_cfg = self._load_emg_config()
        self._board_ip: str = str(emg_cfg.get("board_ip", _DEFAULT_BOARD_IP))
        self._board_port: int = int(emg_cfg.get("board_port", _DEFAULT_BOARD_PORT))
        self._classifier_model_path: str = str(
            emg_cfg.get("classifier_model_path", _DEFAULT_CLASSIFIER_MODEL_PATH)
        )
        publish_rate_hz = float(
            emg_cfg.get("publish_rate_hz", _DEFAULT_PUBLISH_RATE_HZ)
        )

        self.declare_parameter("publish_rate_hz", publish_rate_hz)
        self.declare_parameter("use_simulated_gestures", False)
        self.declare_parameter("simulated_gesture", "REST")
        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._dt = 1.0 / max(rate_hz, 1.0)

        # ── Publishers ───────────────────────────────────────────────────────
        self._gesture_pub = self.create_publisher(String, TOPIC_EMG_GESTURE, 10)
        self._label_pub = self.create_publisher(Int32, "/emg/gesture_label", 10)
        self._conf_pub = self.create_publisher(Float32, "/emg/confidence", 10)
        self._prop_pub = self.create_publisher(Float32, "/emg/proportional", 10)
        self._source_pub = self.create_publisher(String, "/emg/source", 10)
        self._age_pub = self.create_publisher(Float32, "/emg/age", 10)

        # ── State ────────────────────────────────────────────────────────────
        self._source: str = "simulated"
        self._last_reading_time: float = 0.0
        self._use_simulated = bool(self.get_parameter("use_simulated_gestures").value)
        self._simulated_gesture = str(self.get_parameter("simulated_gesture").value)

        # ── Hardware components (set by _init_hardware) ──────────────────────
        self._reader = None
        self._classifier_pipe = None
        self._filter = None
        self._ring_buffer = None
        self._smoother = None
        self._clf_predict = None
        self._compute_features = None
        self._window_step: int = 50
        self._gesture_names: list[str] = []
        self._confidence_threshold: float = 0.55

        # ── Hardware init ────────────────────────────────────────────────────
        if not self._use_simulated:
            try:
                self._init_hardware()
                self._source = "hardware"
                self.get_logger().info(
                    f"MindRove hardware initialised "
                    f"(board={self._board_ip}:{self._board_port}, "
                    f"model={self._classifier_model_path})"
                )
            except Exception as exc:
                self.get_logger().warn(
                    f"Failed to initialise MindRove hardware ({exc}); "
                    f"falling back to simulated REST"
                )
                self._use_simulated = True
                self._source = "simulated"

        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name="emg_loop")
        self._thread.start()

    # ── Config loading ───────────────────────────────────────────────────────

    def _load_emg_config(self) -> dict:
        """Load the ``emg:`` section from the YAML config file."""
        import yaml

        try:
            with open(self._config_path) as f:
                raw = yaml.safe_load(f) or {}
        except Exception as exc:
            self.get_logger().warn(
                f"Config load failed ({self._config_path}): {exc}; using defaults"
            )
            return {}
        return raw.get("emg", {})

    # ── Hardware initialisation ──────────────────────────────────────────────

    def _init_hardware(self) -> None:
        """Initialise board reader, classifier, and preprocessing pipeline.

        Raises on any failure so the caller can fall back to simulated mode.
        """
        from emg_bridge.board_reader import BoardReader
        from emg_bridge import classifier as clf_mod
        from emg_bridge.classifier import PredictionSmoother, predict as clf_predict
        from emg_bridge.config import (
            CONFIDENCE_THRESHOLD,
            GESTURE_NAMES,
            N_CHANNELS,
            PREDICTION_SMOOTHING_FRAMES,
            WINDOW_LEN,
            WINDOW_STEP,
        )
        from emg_bridge.features import compute_features
        from emg_bridge.preprocessing import OnlineFilter, RingBuffer

        # Board
        self._reader = BoardReader(
            ip_address=self._board_ip,
            ip_port=self._board_port,
        )
        self._reader.connect()

        # Classifier pipeline
        self._classifier_pipe = clf_mod.load(self._classifier_model_path)

        # Preprocessing
        n_channels = N_CHANNELS
        self._filter = OnlineFilter(n_channels)
        self._ring_buffer = RingBuffer(WINDOW_LEN, n_channels)
        self._smoother = PredictionSmoother(PREDICTION_SMOOTHING_FRAMES)
        self._clf_predict = clf_predict
        self._compute_features = compute_features
        self._window_step = WINDOW_STEP
        self._gesture_names = list(GESTURE_NAMES)
        self._confidence_threshold = CONFIDENCE_THRESHOLD

        # Boot: fill ring buffer with initial samples
        self.get_logger().info("Filling EMG ring buffer ...")
        while not self._ring_buffer.is_full():
            chunk = self._reader.read(self._window_step)
            filtered = self._filter.process(chunk)
            self._ring_buffer.push(filtered)
        self._last_reading_time = time.monotonic()
        self.get_logger().info("EMG ring buffer ready")

    # ── Prediction ───────────────────────────────────────────────────────────

    def _predict(self) -> tuple[str, int, float, float]:
        """Return (gesture_name, label, confidence, proportional)."""
        if self._use_simulated:
            name = self._simulated_gesture
            label = GESTURES.get(name, REST_GESTURE_LABEL)
            return name, label, 1.0, 0.0

        try:
            # Read next chunk of raw EMG samples from the board
            chunk = self._reader.read(self._window_step)
            self._last_reading_time = time.monotonic()

            # Filter and push into ring buffer
            filtered = self._filter.process(chunk)
            self._ring_buffer.push(filtered)

            if not self._ring_buffer.is_full():
                return "REST", REST_GESTURE_LABEL, 0.0, 0.0

            # Extract features from the full window
            window = self._ring_buffer.get_window().T  # (N_channels, WINDOW_LEN)
            features = self._compute_features(window)

            # Classify
            label, confidence, _probs = self._clf_predict(
                self._classifier_pipe,
                features,
                threshold=self._confidence_threshold,
            )
            smoothed_label = self._smoother.update(label)

            # Map label → gesture name
            name = "REST"
            if 0 <= smoothed_label < len(self._gesture_names):
                name = self._gesture_names[smoothed_label]

            return name, smoothed_label, float(confidence), 0.0
        except Exception as exc:
            self.get_logger().warn(f"Prediction failed: {exc}")
            return "REST", REST_GESTURE_LABEL, 0.0, 0.0

    # ── Main loop ────────────────────────────────────────────────────────────

    def _loop(self) -> None:
        while rclpy.ok() and self._running:
            t0 = time.monotonic()
            name, label, confidence, proportional = self._predict()

            self._gesture_pub.publish(String(data=name))
            self._label_pub.publish(Int32(data=label))
            self._conf_pub.publish(Float32(data=confidence))
            self._prop_pub.publish(Float32(data=proportional))

            # Publish EMG source ("hardware" or "simulated")
            self._source_pub.publish(String(data=self._source))

            # Publish EMG age (seconds since last valid reading)
            now = time.monotonic()
            age = (now - self._last_reading_time) if self._last_reading_time > 0.0 else -1.0
            self._age_pub.publish(Float32(data=age))

            elapsed = time.monotonic() - t0
            remaining = self._dt - elapsed
            if remaining > 0.0:
                time.sleep(remaining)

    # ── Lifecycle ────────────────────────────────────────────────────────────

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._reader is not None:
            try:
                self._reader.disconnect()
            except Exception:
                pass

    def destroy_node(self) -> None:
        self.stop()
        super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--config-path", default=None)
    known, remaining = parser.parse_known_args(args or [])
    rclpy.init(args=remaining)
    node = EmgInputNode(config_path=known.config_path)

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
