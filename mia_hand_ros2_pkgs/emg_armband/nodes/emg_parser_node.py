#!/usr/bin/env python3
"""EMG Parser Node — routes raw EMG signals to ROS topics based on emg.yaml.

For every incoming EmgData message the node:
  1. Always publishes the gesture name on the gesture topic.
  2. For the active gesture, looks up the output list from emg.yaml and
     publishes a Float32 value on each configured topic according to the rules:

     Standard (streaming) mode:
       proportional >= threshold  →  publish (binary ? 1.0 : proportional)
       proportional <  threshold  →  publish 0.0  if send_zero=true, else nothing

     Edge-triggered mode (send_on_change=true):
       Rising edge  (inactive → active)  →  publish value (with optional change_timeout cooldown)
       Falling edge (active  → inactive) →  publish 0.0   if send_zero=true, else nothing

     activation_count: require N consecutive above-threshold frames before "active"

Subscribes:
  <emg_data_topic>   (mia_hand_msgs/EmgData)

Publishes:
  <gesture_topic>    (std_msgs/String)    – current gesture, every message
  <per-gesture topics> (std_msgs/Float32) – routed per emg.yaml

Parameters:
  config_file      (string) – path to emg.yaml (searched automatically if empty)
  emg_data_topic   (string) – override input topic  (default: read from yaml)
  gesture_topic    (string) – override gesture topic (default: read from yaml)
"""

import time
from collections import defaultdict
from pathlib import Path
from typing import Any

import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from mia_hand_msgs.msg import EmgData


class EmgParserNode(Node):
    def __init__(self) -> None:
        super().__init__("emg_parser_node")

        # ── parameters ──────────────────────────────────────────────────────
        self.declare_parameter("config_file", "")
        self.declare_parameter("emg_data_topic", "")
        self.declare_parameter("gesture_topic", "")

        self._cfg = self._load_config()
        parser_cfg: dict[str, Any] = self._cfg.get("emg_parser", {})

        # Resolve topics: explicit parameter > yaml > hard default
        def _topic(param: str, yaml_key: str, default: str) -> str:
            v = self.get_parameter(param).value
            return v if v else parser_cfg.get(yaml_key, default)

        emg_topic = _topic("emg_data_topic", "emg_data_topic", "/emg/data")
        gesture_topic = _topic("gesture_topic", "gesture_topic", "/emg/gesture")

        # ── gesture publisher ────────────────────────────────────────────────
        self._gesture_pub = self.create_publisher(String, gesture_topic, 10)

        # ── per-gesture output publishers ────────────────────────────────────
        # Structure: { gesture_name: [ { pub, binary, threshold, send_zero,
        #                                send_on_change, change_timeout,
        #                                activation_count, _state… }, … ] }
        self._outputs: dict[str, list[dict]] = defaultdict(list)
        gestures_cfg: dict = parser_cfg.get("gestures", {})

        for gesture_name, gesture_data in gestures_cfg.items():
            outputs = (gesture_data or {}).get("outputs", []) or []
            for out in outputs:
                topic = out.get("topic")
                if not topic:
                    self.get_logger().warn(
                        f"Gesture '{gesture_name}' has an output with no topic — skipping."
                    )
                    continue
                pub = self.create_publisher(Float32, topic, 10)
                self._outputs[gesture_name].append(
                    {
                        "pub": pub,
                        "topic": topic,
                        # ── output behaviour ───────────────────────────────
                        "binary": bool(out.get("binary", False)),
                        "threshold": float(out.get("threshold", 0.0)),
                        "send_zero": bool(out.get("send_zero", True)),
                        # ── edge / debounce options ────────────────────────
                        # send_on_change: publish only on rising/falling edge
                        "send_on_change": bool(out.get("send_on_change", False)),
                        # change_timeout: minimum seconds between two rising-edge fires
                        "change_timeout": float(out.get("change_timeout", 0.0)),
                        # activation_count: consecutive above-threshold frames required
                        "activation_count": int(out.get("activation_count", 0)),
                        # ── per-output runtime state ───────────────────────
                        "_prev_active": False,
                        "_active_frames": 0,
                        "_last_fire_time": 0.0,
                    }
                )
                self.get_logger().info(
                    f"  gesture '{gesture_name}' → {topic} "
                    f"(binary={out.get('binary', False)}, "
                    f"threshold={out.get('threshold', 0.0)}, "
                    f"send_zero={out.get('send_zero', True)}, "
                    f"send_on_change={out.get('send_on_change', False)}, "
                    f"change_timeout={out.get('change_timeout', 0.0)}, "
                    f"activation_count={out.get('activation_count', 0)})"
                )

        # ── last active gesture (for cross-gesture state reset) ──────────────
        self._last_gesture: str = ""

        # ── subscription ─────────────────────────────────────────────────────
        self._sub = self.create_subscription(
            EmgData, emg_topic, self._on_emg, 10
        )

        self.get_logger().info(
            f"emg_parser_node ready  "
            f"input={emg_topic}  gesture_out={gesture_topic}  "
            f"gestures configured: {list(self._outputs.keys())}"
        )

    # ── config ───────────────────────────────────────────────────────────────

    def _load_config(self) -> dict:
        config_path = self.get_parameter("config_file").value or ""
        if not config_path:
            candidates = [
                Path("/emg_ws/src/emg_armband/config/emg.yaml"),
                Path(__file__).resolve().parents[3] / "config" / "emg.yaml",
            ]
            for c in candidates:
                if c.exists():
                    config_path = str(c)
                    break

        if not config_path:
            self.get_logger().warn(
                "emg.yaml not found; emg_parser_node will do nothing useful. "
                "Pass config_file parameter or place emg.yaml in the package config/ dir."
            )
            return {}

        self.get_logger().info(f"Loading EMG config from {config_path}")
        return yaml.safe_load(Path(config_path).read_text()) or {}

    # ── helpers ──────────────────────────────────────────────────────────────

    def _reset_gesture_outputs(self, gesture: str, now: float) -> None:
        """Reset per-output state when a gesture becomes inactive.

        If an output was in the active state and send_on_change + send_zero are
        both set, we emit a falling-edge 0.0 so downstream consumers see a
        clean de-activation even when the gesture changed mid-hold.
        """
        for out in self._outputs.get(gesture, []):
            if out["_prev_active"] and out["send_on_change"] and out["send_zero"]:
                f_msg = Float32()
                f_msg.data = 0.0
                out["pub"].publish(f_msg)
            out["_active_frames"] = 0
            out["_prev_active"] = False

    # ── callback ─────────────────────────────────────────────────────────────

    def _on_emg(self, msg: EmgData) -> None:
        gesture: str = msg.gesture
        proportional: float = float(msg.proportional)
        now: float = time.monotonic()

        # 1. Always publish gesture
        g_msg = String()
        g_msg.data = gesture
        self._gesture_pub.publish(g_msg)

        # 2. If gesture changed, reset state for the previous gesture's outputs
        if gesture != self._last_gesture and self._last_gesture:
            self._reset_gesture_outputs(self._last_gesture, now)
        self._last_gesture = gesture

        # 3. Route proportional signal for this gesture
        for out in self._outputs.get(gesture, []):
            above: bool = proportional >= out["threshold"]

            # ── activation_count debounce ───────────────────────────────────
            if out["activation_count"] > 0:
                if above:
                    out["_active_frames"] += 1
                else:
                    out["_active_frames"] = 0
                fire: bool = out["_active_frames"] >= out["activation_count"]
            else:
                fire = above

            f_msg = Float32()

            if out["send_on_change"]:
                # ── edge-triggered mode ─────────────────────────────────────
                rising  = fire and not out["_prev_active"]
                falling = (not fire) and out["_prev_active"]

                if rising:
                    cooldown_ok = (
                        out["change_timeout"] <= 0.0
                        or (now - out["_last_fire_time"]) >= out["change_timeout"]
                    )
                    if cooldown_ok:
                        f_msg.data = 1.0 if out["binary"] else proportional
                        out["pub"].publish(f_msg)
                        out["_last_fire_time"] = now

                if falling and out["send_zero"]:
                    f_msg.data = 0.0
                    out["pub"].publish(f_msg)

            else:
                # ── streaming mode (original behaviour) ────────────────────
                if fire:
                    f_msg.data = 1.0 if out["binary"] else proportional
                    out["pub"].publish(f_msg)
                elif out["send_zero"]:
                    f_msg.data = 0.0
                    out["pub"].publish(f_msg)

            out["_prev_active"] = fire


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EmgParserNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
