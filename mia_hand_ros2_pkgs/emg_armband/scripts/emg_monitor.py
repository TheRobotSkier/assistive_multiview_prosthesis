#!/usr/bin/env python3
"""Live terminal monitor for the EMG ROS2 output topics.

Reads emg.yaml to discover every topic published by emg_node and
emg_parser_node, then subscribes to all of them and renders a compact,
auto-refreshing terminal dashboard.

Topics monitored:
    /emg/data    (mia_hand_msgs/EmgData)  – gesture label + proportional value
    /emg/gesture (std_msgs/String)         – current gesture string
    All Float32 output topics listed under emg_parser.gestures in emg.yaml

Must be run after sourcing the ROS2 workspace:
    source /emg_ws/install/setup.bash
    python scripts/emg_monitor.py
    python scripts/emg_monitor.py --config /emg_ws/src/emg_armband/config/emg.yaml
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path

import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from mia_hand_msgs.msg import EmgData


# ── ANSI helpers ──────────────────────────────────────────────────────────────

def _bold(s: str) -> str:   return f"\033[1m{s}\033[0m"
def _green(s: str) -> str:  return f"\033[92m{s}\033[0m"
def _yellow(s: str) -> str: return f"\033[93m{s}\033[0m"
def _cyan(s: str) -> str:   return f"\033[96m{s}\033[0m"
def _dim(s: str) -> str:    return f"\033[2m{s}\033[0m"
def _red(s: str) -> str:    return f"\033[91m{s}\033[0m"

_UP = "\033[A"
_CLEAR_LINE = "\033[2K"


def _prop_bar(val: float, width: int = 22) -> str:
    filled = int(max(0.0, min(1.0, val)) * width)
    bar = "▓" * filled + "░" * (width - filled)
    return f"[{bar}] {val:.3f}"


def _float_bar(val: float, width: int = 14) -> str:
    filled = int(max(0.0, min(1.0, abs(val))) * width)
    bar = "█" * filled + "·" * (width - filled)
    return f"[{bar}] {val:+.3f}"


# ── Monitor node ──────────────────────────────────────────────────────────────

class EmgMonitorNode(Node):
    """Subscribes to all EMG output topics and refreshes a terminal display."""

    def __init__(self, config_path: str) -> None:
        super().__init__("emg_monitor")

        # ── parse config ──────────────────────────────────────────────────────
        cfg = yaml.safe_load(Path(config_path).read_text()) or {}
        parser_cfg = cfg.get("emg_parser", {})

        emg_data_topic: str = parser_cfg.get("emg_data_topic", "/emg/data")
        gesture_topic: str = parser_cfg.get("gesture_topic", "/emg/gesture")

        # Collect unique output Float32 topics in yaml order
        self._output_topics: list[str] = []
        gestures_cfg: dict = parser_cfg.get("gestures", {}) or {}
        for gesture_name, gesture_body in gestures_cfg.items():
            for output in (gesture_body or {}).get("outputs", []):
                t = output.get("topic")
                if t and t not in self._output_topics:
                    self._output_topics.append(t)

        # ── shared state (written in subscription callbacks, read in display) ─
        self._lock = threading.Lock()
        self._gesture: str = "—"
        self._proportional: float = 0.0
        self._float_values: dict[str, float] = {t: 0.0 for t in self._output_topics}
        self._last_emg_stamp: float = 0.0   # monotonic time of last EmgData msg

        # ── subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(EmgData, emg_data_topic, self._on_emg, 10)
        self.create_subscription(String,  gesture_topic,  self._on_gesture, 10)

        for topic in self._output_topics:
            # Bind topic name into the closure so each callback captures its own `t`
            def _make_cb(t: str):
                def cb(msg: Float32) -> None:
                    with self._lock:
                        self._float_values[t] = msg.data
                return cb
            self.create_subscription(Float32, topic, _make_cb(topic), 10)

        # ── display timer ─────────────────────────────────────────────────────
        self._first_draw: bool = True
        self._n_display_lines: int = 0
        self.create_timer(0.1, self._draw)   # 10 Hz refresh

        self.get_logger().info(
            f"Monitoring {emg_data_topic}, {gesture_topic}"
            + (f", and {len(self._output_topics)} Float32 output topics"
               if self._output_topics else "")
        )

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _on_emg(self, msg: EmgData) -> None:
        with self._lock:
            self._gesture = msg.gesture
            self._proportional = msg.proportional
            self._last_emg_stamp = time.monotonic()

    def _on_gesture(self, _msg: String) -> None:
        pass  # already captured via EmgData; kept for completeness

    # ── display ───────────────────────────────────────────────────────────────

    def _draw(self) -> None:
        with self._lock:
            gesture = self._gesture
            prop = self._proportional
            float_vals = dict(self._float_values)
            stale = (time.monotonic() - self._last_emg_stamp) > 2.0

        lines: list[str] = []
        lines.append(_bold("─" * 58))
        lines.append(
            f"  {_bold('EMG Monitor')}  "
            + _dim(f"{'(waiting for data…)' if stale else 'live'}")
            + "  "
            + _dim("Ctrl-C to stop")
        )
        lines.append(_bold("─" * 58))

        if stale:
            gesture_str = _yellow("waiting…")
        else:
            gesture_str = (_dim(gesture) if gesture == "REST" else _green(_bold(gesture)))

        lines.append(f"  {_bold('Gesture')}    : {gesture_str}")
        lines.append(f"  {_bold('Proportional')}: {_prop_bar(prop)}")

        if float_vals:
            lines.append("")
            lines.append(f"  {_bold('Output topics:')}")
            for topic, val in float_vals.items():
                # Show only the last path segment for brevity
                label = topic.rsplit("/", 1)[-1]
                bar = _float_bar(val)
                # Dim topics that are zero (likely inactive)
                row = f"    {_cyan(label):<26}  {bar}"
                lines.append(_dim(row) if val == 0.0 else row)

        lines.append(_bold("─" * 58))

        # Erase previous output and redraw
        n = len(lines)
        if not self._first_draw:
            sys.stdout.write((_UP + _CLEAR_LINE) * self._n_display_lines)

        sys.stdout.write("\n".join(lines) + "\n")
        sys.stdout.flush()
        self._n_display_lines = n
        self._first_draw = False


# ── entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Live terminal monitor for EMG ROS2 output topics"
    )
    parser.add_argument(
        "--config",
        default="/emg_ws/src/emg_armband/config/emg.yaml",
        help="Path to emg.yaml  (default: /emg_ws/src/emg_armband/config/emg.yaml)",
    )
    args = parser.parse_args()

    if not Path(args.config).exists():
        print(f"Config file not found: {args.config}", file=sys.stderr)
        sys.exit(1)

    rclpy.init()
    node = EmgMonitorNode(args.config)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("\nMonitor stopped.")


if __name__ == "__main__":
    main()
