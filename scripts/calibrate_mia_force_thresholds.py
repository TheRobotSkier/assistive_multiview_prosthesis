#!/usr/bin/env python3
"""Calibrate MIA hand fingertip force thresholds from idle strain gauge readings.

Subscribes to /joint_states and records effort values for the three flexion
joints (thumb, index, MRL) while the hand is at rest — no contact, no load.

After the recording window elapses, the script computes per-finger statistics
and prints a YAML snippet with recommended force thresholds ready to paste
into prosthesis_config.yaml.
"""

from __future__ import annotations

import argparse
import math
import signal
import sys
import time
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


FINGER_JOINTS = ["j_thumb_fle", "j_index_fle", "j_mrl_fle"]


def _summarise(values: List[float], label: str) -> Dict[str, float]:
    n = len(values)
    if n == 0:
        return {"min": 0.0, "max": 0.0, "mean": 0.0, "stddev": 0.0,
                "contact_threshold": 50.0, "spike_threshold": 15.0}

    mean = sum(values) / n
    var = max(0.0, sum((v - mean) ** 2 for v in values) / n)
    stddev = math.sqrt(var)

    return {
        "min": min(values),
        "max": max(values),
        "mean": mean,
        "stddev": stddev,
        "contact_threshold": mean + 3.0 * stddev,
        "spike_threshold": 2.0 * stddev,
    }


class ForceCalibrator(Node):

    def __init__(self, duration_s: float = 5.0):
        super().__init__("calibrate_mia_force_thresholds")
        self._duration_s = duration_s
        self._start_time: Optional[float] = None
        self._done = False

        self._samples: Dict[str, List[float]] = {j: [] for j in FINGER_JOINTS}

        self._sub = self.create_subscription(
            JointState, "/joint_states", self._on_joint_states, 10
        )

        self._timer = self.create_timer(0.1, self._check_done)

        self.get_logger().info(
            f"Recording idle forces for {duration_s:.1f} s. "
            "Keep the hand open and unloaded."
        )

    def _on_joint_states(self, msg: JointState) -> None:
        if self._start_time is None:
            self._start_time = time.time()

        for i, joint_name in enumerate(FINGER_JOINTS):
            try:
                idx = msg.name.index(joint_name)
            except ValueError:
                continue
            if idx < len(msg.effort):
                self._samples[joint_name].append(float(msg.effort[idx]))

    def _check_done(self) -> None:
        if self._done:
            return
        if self._start_time is None:
            return

        elapsed = time.time() - self._start_time
        remaining = max(0.0, self._duration_s - elapsed)
        if remaining > 0:
            self.get_logger().debug(
                f"Recording... {remaining:.1f} s remaining"
            )
            return

        self._done = True
        self._print_report()
        rclpy.shutdown()

    def _print_report(self) -> None:
        print()
        print("=" * 60)
        print("  MIA Hand Force Calibration Report")
        print("=" * 60)
        print()

        stats: Dict[str, Dict[str, float]] = {}
        for j in FINGER_JOINTS:
            display = j.split("_")[1]  # thumb / index / mrl
            stats[display] = _summarise(self._samples[j], display)

        for finger_label in ["thumb", "index", "mrl"]:
            s = stats[finger_label]
            count = len(self._samples[f"j_{finger_label}_fle"])
            print(f"  {finger_label.capitalize()} ({count} samples):")
            print(f"    min               : {s['min']:8.1f}")
            print(f"    max               : {s['max']:8.1f}")
            print(f"    mean              : {s['mean']:8.1f}")
            print(f"    stddev            : {s['stddev']:8.1f}")
            print(f"    contact_threshold : {s['contact_threshold']:8.1f}")
            print(f"    spike_threshold   : {s['spike_threshold']:8.1f}")
            print()

        # Use the maximum recommended threshold across fingers as the safe
        # single-value threshold for each profile.  Per-finger thresholds could
        # be tighter but the current config uses one value per profile.
        max_contact = max(
            stats[f]["contact_threshold"] for f in ["thumb", "index", "mrl"]
        )
        max_spike = max(
            stats[f]["spike_threshold"] for f in ["thumb", "index", "mrl"]
        )

        print("-" * 60)
        print("  Suggested YAML snippet for prosthesis_config.yaml")
        print("-" * 60)
        print()
        label_width = max(len("contact_force_spike_threshold"),
                          len("contact_force_threshold"))

        def _emit(profile_name: str, contact_scale: float = 1.0, spike_scale: float = 1.0):
            ct = f"{max_contact * contact_scale:.1f}"
            st = f"{max_spike * spike_scale:.1f}"
            print(f"  {profile_name}:")
            print(f"    contact_force_threshold:       {ct:>{label_width - 11}}")
            print(f"    contact_force_spike_threshold: {st:>{label_width - 15}}")

        print("  profiles:")
        _emit("soft")
        _emit("strong", contact_scale=2.0, spike_scale=2.0)

        print()
        print("=" * 60)
        print(f"  Raw values are ADC units — NOT Newtons.")
        print(f"  See docs/mia_force_calibration.md for interpretation.")
        print("=" * 60)


def _make_argparser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Calibrate MIA hand force thresholds from idle strain gauge data."
    )
    p.add_argument(
        "--duration", "-d",
        type=float,
        default=5.0,
        help="Recording duration in seconds (default: 5.0)",
    )
    p.add_argument(
        "--output", "-o",
        type=str,
        default="",
        help="Optional file path to save the YAML snippet (default: print only)",
    )
    return p


def main():
    parser = _make_argparser()
    args = parser.parse_args()

    rclpy.init(args=sys.argv)

    node = ForceCalibrator(duration_s=args.duration)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if args.output:
        print(f"\nResults written to {args.output} not yet implemented — "
              "redirect stdout instead:  ... > my_thresholds.yaml")


if __name__ == "__main__":
    main()
