#!/usr/bin/env python3
"""Runtime planning for the EMG grasp-test launch."""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
import time
from dataclasses import dataclass


EMG_TOPICS = {"/emg/gesture_label", "/emg/confidence"}
HAND_NODES = {"/controller_manager"}
WRIST_NODE = "/wrist_driver"
_ROS2_TIMEOUT = 5


@dataclass(frozen=True)
class LaunchPlan:
    launch_mia_hand: bool
    use_mock_hardware: bool
    launch_wrist_driver: bool
    launch_local_emg: bool


def _ros2_list(kind: str) -> set[str]:
    try:
        result = subprocess.run(
            ["ros2", kind, "list"],
            check=False,
            capture_output=True,
            text=True,
            timeout=_ROS2_TIMEOUT,
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
            timeout=_ROS2_TIMEOUT,
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


def _host_ipv4_addresses() -> set[str]:
    try:
        result = subprocess.run(
            ["ip", "-4", "addr", "show"],
            check=False,
            capture_output=True,
            text=True,
            timeout=5,
        )
    except Exception:
        return set()

    if result.returncode != 0:
        return set()

    return set(re.findall(r"inet\s+(\d+\.\d+\.\d+\.\d+)", result.stdout))


def peer_config_matches_host(peer_config_path: str) -> bool:
    if not os.path.exists(peer_config_path):
        return False

    try:
        with open(peer_config_path, "r", encoding="utf-8") as handle:
            config_text = handle.read()
    except OSError:
        return False

    configured_addresses = set(re.findall(r'address="([^"]+)"', config_text))
    if not configured_addresses:
        return False

    return bool(configured_addresses & _host_ipv4_addresses())


def decide_launch_plan(
    *,
    available_nodes: set[str],
    available_topics: set[str],
    emg_topic_publishers: dict[str, int] | None = None,
    hand_serial_exists: bool,
    wrist_serial_exists: bool,
) -> LaunchPlan:
    topic_publishers = emg_topic_publishers or {}
    controller_running = bool(HAND_NODES & available_nodes)
    wrist_running = WRIST_NODE in available_nodes
    emg_running = all(
        topic in available_topics and topic_publishers.get(topic, 0) > 0
        for topic in EMG_TOPICS
    )

    launch_mia_hand = not controller_running
    use_mock_hardware = launch_mia_hand and not hand_serial_exists
    launch_wrist_driver = not wrist_running and wrist_serial_exists
    launch_local_emg = not emg_running

    return LaunchPlan(
        launch_mia_hand=launch_mia_hand,
        use_mock_hardware=use_mock_hardware,
        launch_wrist_driver=launch_wrist_driver,
        launch_local_emg=launch_local_emg,
    )


def probe_launch_plan(
    *,
    hand_port: str,
    wrist_port: str,
    discovery_attempts: int = 3,
    discovery_delay_s: float = 1.0,
) -> LaunchPlan:
    nodes: set[str] = set()
    topics: set[str] = set()
    for attempt in range(discovery_attempts):
        nodes.update(_ros2_list("node"))
        topics.update(_ros2_list("topic"))
        if attempt < discovery_attempts - 1:
            time.sleep(discovery_delay_s)

    emg_topic_publishers = {
        topic: _ros2_topic_info(topic)[0]
        for topic in EMG_TOPICS
        if topic in topics
    }

    return decide_launch_plan(
        available_nodes=nodes,
        available_topics=topics,
        emg_topic_publishers=emg_topic_publishers,
        hand_serial_exists=os.path.exists(hand_port),
        wrist_serial_exists=os.path.exists(wrist_port),
    )


def _bool_arg(value: bool) -> str:
    return "true" if value else "false"


def build_launch_command(
    *,
    plan: LaunchPlan,
    config_path: str,
    model_dir: str,
    hand_port: str,
    wrist_port: str,
) -> list[str]:
    return [
        "ros2",
        "launch",
        "prosthesis_launch",
        "emg_grasp_test.launch.py",
        f"launch_mia_hand:={_bool_arg(plan.launch_mia_hand)}",
        f"use_mock_hardware:={_bool_arg(plan.use_mock_hardware)}",
        f"launch_wrist_driver:={_bool_arg(plan.launch_wrist_driver)}",
        f"emg:={_bool_arg(plan.launch_local_emg)}",
        f"serial_port:={hand_port}",
        f"wrist_serial_port:={wrist_port}",
        f"config_path:={config_path}",
        f"model_dir:={model_dir}",
    ]


def main() -> None:
    parser = argparse.ArgumentParser(description="Plan and launch EMG grasp test")
    parser.add_argument(
        "--config-path",
        default="/prosthesis_ws/tests/emg_grasp/emg_grasp_test.yaml",
        help="Path to the EMG grasp test config YAML",
    )
    parser.add_argument(
        "--model-dir",
        default="/prosthesis_ws/models",
        help="Directory containing trained EMG models",
    )
    parser.add_argument(
        "--serial-port",
        default="/dev/ttyUSB0",
        help="Mia Hand serial device path",
    )
    parser.add_argument(
        "--wrist-serial-port",
        default="/dev/ttyUSB1",
        help="Wrist serial device path",
    )
    parser.add_argument(
        "--cyclonedds-uri",
        default="/tmp/cyclonedds_peer.xml",
        help="CycloneDDS peer config file to validate before launch",
    )
    args = parser.parse_args()

    if not peer_config_matches_host(args.cyclonedds_uri):
        os.environ.pop("CYCLONEDDS_URI", None)
        print(
            f"CycloneDDS peer config {args.cyclonedds_uri} does not match this host; using default discovery.",
            flush=True,
        )

    plan = probe_launch_plan(
        hand_port=args.serial_port,
        wrist_port=args.wrist_serial_port,
    )

    print(
        "Launch plan: "
        f"mia_hand={'local' if plan.launch_mia_hand else 'reuse'} "
        f"mock_hardware={plan.use_mock_hardware} "
        f"wrist_driver={'local' if plan.launch_wrist_driver else 'skip'} "
        f"local_emg={'start' if plan.launch_local_emg else 'reuse'}",
        flush=True,
    )

    command = build_launch_command(
        plan=plan,
        config_path=args.config_path,
        model_dir=args.model_dir,
        hand_port=args.serial_port,
        wrist_port=args.wrist_serial_port,
    )
    os.execvp(command[0], command)


if __name__ == "__main__":
    main()
