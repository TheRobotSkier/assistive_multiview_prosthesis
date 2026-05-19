#!/usr/bin/env python3
"""EMG Armband Node — MindRove WiFi board hardware interface.

Runs the MindRove EMG inference pipeline in a background thread and publishes
gesture + proportional control data on a ROS2 topic so that emg_parser_node
can route it to the hand controller.

Pipeline:
    BoardReader → OnlineFilter → RingBuffer
    → feature extraction (MAV/RMS/WL/ZC/SSC/VAR × 8 ch, 48 features total)
    → LDA/SVM classifier (loaded from model_dir)
    → PredictionSmoother (majority-vote, 3 frames)
    → proportional control (RMS normalised with per-gesture calibration)

Publishes:
    <emg_data_topic>  (mia_hand_msgs/EmgData)
        .proportional  – normalised EMG intensity 0.0–1.0
        .gesture       – gesture name string (REST/POWER/PINCH/OPEN/POINT)

Parameters (may also be set via the emg_node: section of emg.yaml):
    model_dir            (string) – directory containing classifier.pkl and
                                    prop_calibration.pkl.
                                    Default: /emg_ws/src/emg_armband/models/
    config_file          (string) – path to emg.yaml; empty = auto-detect
    emg_data_topic       (string) – topic to publish on (default /emg/data)
    publish_rate         (double) – Hz (default 10.0, matches WINDOW_STEP cadence)
    confidence_threshold (double) – confidence below which REST is returned
                                    (default 0.55)
    smoothing_frames     (int)    – majority-vote window in frames (default 3)
"""

import threading
from pathlib import Path

import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from mia_hand_msgs.msg import EmgData

from emg_classifier import classifier as clf_mod
from emg_classifier import proportional as prop_mod
from emg_classifier.board_reader import BoardReader
from emg_classifier.classifier import PredictionSmoother, predict
from emg_classifier.config import (
    CONFIDENCE_THRESHOLD,
    GESTURE_NAMES,
    N_CHANNELS,
    PREDICTION_SMOOTHING_FRAMES,
    WINDOW_LEN,
    WINDOW_STEP,
)
from emg_classifier.features import compute_features
from emg_classifier.preprocessing import OnlineFilter, RingBuffer


class EmgNode(Node):
    def __init__(self) -> None:
        super().__init__("emg_node")

        # ── parameters ──────────────────────────────────────────────────────
        self.declare_parameter("config_file", "")
        self.declare_parameter("model_dir", "/emg_ws/src/emg_armband/models/")
        self.declare_parameter("emg_data_topic", "/emg/data")
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("confidence_threshold", CONFIDENCE_THRESHOLD)
        self.declare_parameter("smoothing_frames", PREDICTION_SMOOTHING_FRAMES)

        self._load_config()

        model_dir = Path(self.get_parameter("model_dir").value)
        topic = self.get_parameter("emg_data_topic").value
        rate = self.get_parameter("publish_rate").value
        self._conf_threshold: float = float(
            self.get_parameter("confidence_threshold").value
        )
        self._smooth_frames: int = int(
            self.get_parameter("smoothing_frames").value
        )

        # ── publisher ───────────────────────────────────────────────────────
        self._pub = self.create_publisher(EmgData, topic, 10)

        # ── shared state (written by inference thread, read by publish timer)
        self._lock = threading.Lock()
        self._proportional: float = 0.0
        self._gesture: str = "REST"

        # ── load models ──────────────────────────────────────────────────────
        self.get_logger().info(f"Loading classifier from {model_dir} …")
        try:
            self._pipe = clf_mod.load(model_dir)
            self._calibration = prop_mod.load(model_dir)
        except FileNotFoundError as exc:
            self.get_logger().fatal(str(exc))
            raise

        if self._calibration is None:
            self.get_logger().warn(
                "prop_calibration.pkl not found — using fallback RMS normalisation."
            )
        else:
            self.get_logger().info("Proportional calibration loaded.")

        # ── inference thread ─────────────────────────────────────────────────
        self._running = True
        self._hw_thread = threading.Thread(
            target=self._inference_loop, daemon=True
        )
        self._hw_thread.start()

        # ── publish timer ─────────────────────────────────────────────────────
        self._timer = self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            f"emg_node started  topic={topic}  rate={rate:.1f} Hz"
        )

    # ── config loading ────────────────────────────────────────────────────────

    def _load_config(self) -> None:
        config_path = self.get_parameter("config_file").value or ""
        if not config_path:
            for candidate in [
                Path("/emg_ws/src/emg_armband/config/emg.yaml"),
                Path(__file__).resolve().parents[3] / "config" / "emg.yaml",
            ]:
                if candidate.exists():
                    config_path = str(candidate)
                    break

        if not config_path:
            self.get_logger().warn("emg.yaml not found; using parameter defaults.")
            return

        self.get_logger().info(f"Loading EMG config from {config_path}")
        cfg = yaml.safe_load(Path(config_path).read_text()) or {}
        node_cfg = cfg.get("emg_node", {})

        for param, key in [
            ("model_dir", "model_dir"),
            ("emg_data_topic", "emg_data_topic"),
            ("publish_rate", "publish_rate"),
            ("confidence_threshold", "confidence_threshold"),
            ("smoothing_frames", "smoothing_frames"),
        ]:
            val = node_cfg.get(key)
            if val is not None:
                try:
                    self.set_parameters(
                        [rclpy.parameter.Parameter(param, value=val)]
                    )
                except Exception as exc:
                    self.get_logger().warn(
                        f"Could not apply config key {key}: {exc}"
                    )

    # ── inference loop (background thread) ───────────────────────────────────

    def _inference_loop(self) -> None:
        self.get_logger().info("Connecting to MindRove WiFi board …")
        reader = BoardReader()
        try:
            reader.connect()
        except Exception as exc:
            self.get_logger().error(f"Failed to connect to MindRove board: {exc}")
            return

        self.get_logger().info(
            f"Connected — {reader.sampling_rate} Hz  {N_CHANNELS} channels"
        )

        filt = OnlineFilter(N_CHANNELS)
        ring = RingBuffer(WINDOW_LEN, N_CHANNELS)
        smoother = PredictionSmoother(self._smooth_frames)

        # Fill the ring buffer before starting inference
        self.get_logger().info("Filling ring buffer …")
        while not ring.is_full() and self._running:
            chunk = reader.read(WINDOW_STEP)
            ring.push(filt.process(chunk))

        self.get_logger().info("Buffer full — inference running.")

        try:
            while self._running:
                chunk = reader.read(WINDOW_STEP)
                ring.push(filt.process(chunk))

                if not ring.is_full():
                    continue

                window = ring.get_window().T  # (N_channels, WINDOW_LEN)
                features = compute_features(window)

                label, _conf, _probs = predict(
                    self._pipe, features, threshold=self._conf_threshold
                )
                smoothed = smoother.update(label)

                prop_val = prop_mod.compute_proportional(
                    window, smoothed, self._calibration
                )
                gesture_name = (
                    GESTURE_NAMES[smoothed]
                    if 0 <= smoothed < len(GESTURE_NAMES)
                    else "REST"
                )

                with self._lock:
                    self._proportional = prop_val
                    self._gesture = gesture_name

        except Exception as exc:
            self.get_logger().error(f"Inference loop error: {exc}")
        finally:
            reader.disconnect()
            self.get_logger().info("Board disconnected.")

    # ── publish loop ──────────────────────────────────────────────────────────

    def _publish(self) -> None:
        msg = EmgData()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        with self._lock:
            msg.proportional = float(self._proportional)
            msg.gesture = str(self._gesture)
        self._pub.publish(msg)

    def destroy_node(self) -> None:
        self._running = False
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EmgNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
