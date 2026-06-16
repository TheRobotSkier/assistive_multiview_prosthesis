#!/usr/bin/env python3

from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def test_emg_workflow_uses_dedicated_emg_service():
    workflow = (ROOT / "scripts" / "emg_latency_workflow.sh").read_text(encoding="utf-8")
    compose = (ROOT / "docker" / "docker-compose.yml").read_text(encoding="utf-8")

    assert 'CONTAINER_NAME="emg"' in workflow
    assert '    emg:' in compose
    assert 'dockerfile: docker/Dockerfile.emg' in compose


def test_emg_service_uses_ros_core_slim_image_and_host_targets_exist():
    dockerfile = (ROOT / "docker" / "Dockerfile.emg").read_text(encoding="utf-8")
    makefile = (ROOT / "Makefile").read_text(encoding="utf-8")

    assert "FROM docker.io/library/ros:jazzy-ros-core-noble" in dockerfile
    assert "ros-jazzy-rclpy" in dockerfile
    assert "ros-jazzy-std-msgs" in dockerfile
    assert "emg-dev:" in makefile
    assert "emg-shell:" in makefile


def test_emg_dockerfile_handles_existing_uid_user_mapping():
    dockerfile = (ROOT / "docker" / "Dockerfile.emg").read_text(encoding="utf-8")

    assert "if id -u ${USER_UID} >/dev/null 2>&1; then" in dockerfile
    assert "usermod -l prosthesis" in dockerfile


def test_docs_describe_emg_service_and_ros_core_base():
    validation = (ROOT / "docs" / "reference" / "validation-and-workflows.md").read_text(encoding="utf-8")
    control = (ROOT / "docs" / "architecture" / "control-and-actuation.md").read_text(encoding="utf-8")

    assert "make emg-dev" in validation
    assert "ros:jazzy-ros-core-noble" in validation
    assert "dedicated EMG container" in control
