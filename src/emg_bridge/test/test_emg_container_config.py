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


def test_workflow_exec_path_does_not_force_dash_it_flags():
    workflow = (ROOT / "scripts" / "emg_latency_workflow.sh").read_text(encoding="utf-8")

    assert "exec --user prosthesis -it" not in workflow


def test_workflow_does_not_fetch_remote_by_default():
    workflow = (ROOT / "scripts" / "emg_latency_workflow.sh").read_text(encoding="utf-8")

    assert 'FETCH_REMOTE="${EMG_FETCH_REMOTE:-false}"' in workflow
    assert 'Fetching origin/asger_dev with gh-authenticated git...' not in workflow


def test_emg_dockerfile_installs_cyclonedds_rmw_runtime():
    dockerfile = (ROOT / "docker" / "Dockerfile.emg").read_text(encoding="utf-8")

    assert "ros-jazzy-cyclonedds" in dockerfile
    assert "ros-jazzy-rmw-cyclonedds-cpp" in dockerfile


def test_notrain_workflow_target_reuses_existing_model():
    workflow = (ROOT / "scripts" / "emg_latency_workflow.sh").read_text(encoding="utf-8")
    makefile = (ROOT / "Makefile").read_text(encoding="utf-8")

    assert 'EMG_SKIP_TRAIN="${EMG_SKIP_TRAIN:-false}"' in workflow
    assert "emg-latency-workflow-notrain" in makefile
