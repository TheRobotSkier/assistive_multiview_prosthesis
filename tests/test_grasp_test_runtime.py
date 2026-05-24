"""Tests for grasp-test launch planning."""

from __future__ import annotations

import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "scripts"))

import emg_grasp_test_runtime as runtime


def test_missing_devices_use_mock_hardware_and_local_emg() -> None:
    plan = runtime.decide_launch_plan(
        available_nodes=set(),
        available_topics=set(),
        hand_serial_exists=False,
        wrist_serial_exists=False,
    )

    assert plan.launch_mia_hand is True
    assert plan.use_mock_hardware is True
    assert plan.launch_wrist_driver is False
    assert plan.launch_local_emg is True


def test_existing_ros_components_are_not_relaunched() -> None:
    plan = runtime.decide_launch_plan(
        available_nodes={"/controller_manager", "/wrist_driver"},
        available_topics={"/emg/gesture_label", "/emg/confidence"},
        emg_topic_publishers={"/emg/gesture_label": 1, "/emg/confidence": 1},
        hand_serial_exists=True,
        wrist_serial_exists=True,
    )

    assert plan.launch_mia_hand is False
    assert plan.use_mock_hardware is False
    assert plan.launch_wrist_driver is False
    assert plan.launch_local_emg is False


def test_available_devices_launch_real_stack() -> None:
    plan = runtime.decide_launch_plan(
        available_nodes=set(),
        available_topics=set(),
        hand_serial_exists=True,
        wrist_serial_exists=True,
    )

    assert plan.launch_mia_hand is True
    assert plan.use_mock_hardware is False
    assert plan.launch_wrist_driver is True
    assert plan.launch_local_emg is True


def test_topics_without_publishers_still_launch_local_emg() -> None:
    plan = runtime.decide_launch_plan(
        available_nodes=set(),
        available_topics={"/emg/gesture_label", "/emg/confidence"},
        emg_topic_publishers={"/emg/gesture_label": 0, "/emg/confidence": 0},
        hand_serial_exists=True,
        wrist_serial_exists=True,
    )

    assert plan.launch_local_emg is True


def test_peer_config_matches_host_when_address_present(monkeypatch) -> None:
    monkeypatch.setattr(runtime, "_host_ipv4_addresses", lambda: {"10.42.0.1"})

    path = REPO_ROOT / "config" / "cyclonedds_peer.xml"
    assert runtime.peer_config_matches_host(str(path)) is True


def test_makefile_runs_grasp_test_container_directly() -> None:
    makefile_text = (REPO_ROOT / "Makefile").read_text()
    target_start = makefile_text.index("up-grasp-test:")
    target_end = makefile_text.index("down-grasp-test:", target_start)
    target_body = makefile_text[target_start:target_end]

    assert "$(DOCKER_CMD) run" in target_body
    assert "$(MAKE) logs-grasp-test" in target_body
    assert "podman-compose --profile grasp_test up -d grasp_test" not in target_body
