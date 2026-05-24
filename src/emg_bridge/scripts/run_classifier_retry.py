#!/usr/bin/env python3
"""Retry wrapper for the live EMG classifier."""

from __future__ import annotations

import argparse
import subprocess
import time


EMG_TOPICS = {"/emg/gesture_label", "/emg/confidence"}


def _ros2_topic_list() -> set[str]:
    try:
        result = subprocess.run(
            ["ros2", "topic", "list"],
            check=False,
            capture_output=True,
            text=True,
            timeout=5,
        )
    except Exception:
        return set()

    if result.returncode != 0:
        return set()

    return {line.strip() for line in result.stdout.splitlines() if line.strip()}


def _ros2_topic_info(topic: str) -> tuple[int, int]:
    try:
        result = subprocess.run(
            ["ros2", "topic", "info", topic],
            check=False,
            capture_output=True,
            text=True,
            timeout=5,
        )
    except Exception:
        return (0, 0)

    if result.returncode != 0:
        return (0, 0)

    publishers = 0
    subscribers = 0
    for line in result.stdout.splitlines():
        line = line.strip()
        if line.startswith("Publisher count:"):
            publishers = int(line.split(":", 1)[1].strip())
        elif line.startswith("Subscription count:"):
            subscribers = int(line.split(":", 1)[1].strip())
    return (publishers, subscribers)


def external_emg_available(
    *,
    topic_info_func=_ros2_topic_info,
    topics: set[str] | None = None,
) -> bool:
    available_topics = topics if topics is not None else _ros2_topic_list()
    if not EMG_TOPICS.issubset(available_topics):
        return False

    return all(topic_info_func(topic)[0] > 0 for topic in EMG_TOPICS)


def main() -> None:
    parser = argparse.ArgumentParser(description="Retry the live EMG classifier")
    parser.add_argument("--model-dir", required=True, help="Directory containing trained EMG models")
    parser.add_argument(
        "--config",
        type=str,
        default="",
        help="Path to EMG experiment config YAML file",
    )
    parser.add_argument(
        "--retry-delay",
        type=float,
        default=5.0,
        help="Seconds between retry attempts",
    )
    args = parser.parse_args()

    try:
        while True:
            if external_emg_available():
                print(
                    "External EMG topics already available; skipping local classifier startup.",
                    flush=True,
                )
                time.sleep(args.retry_delay)
                continue

            cmd = [
                "ros2",
                "run",
                "emg_bridge",
                "run_classifier",
                "--model-dir",
                args.model_dir,
            ]
            if args.config and args.config.strip():
                cmd.extend(["--config", args.config.strip()])
            result = subprocess.run(cmd, check=False)
            if result.returncode == 0:
                return

            print(
                f"Local EMG classifier exited with code {result.returncode}; retrying in {args.retry_delay:.0f} s.",
                flush=True,
            )
            time.sleep(args.retry_delay)
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
