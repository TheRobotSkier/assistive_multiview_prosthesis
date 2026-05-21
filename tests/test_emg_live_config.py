"""Tests for live EMG config reloading behavior in pipeline_manager."""

from __future__ import annotations

import sys
import time
from pathlib import Path
from unittest.mock import MagicMock

import pytest
import yaml


# ── ROS mocks (must be before importing the node module) ──────────────────────
class _FakeQoSProfile:
    def __init__(self, *a, **k): pass

class _FakeDurabilityPolicy:
    TRANSIENT_LOCAL = 1

class _FakeNode:
    def __init__(self, name): self._name = name
    def declare_parameter(self, name, value): pass
    def get_parameter(self, name):
        m = MagicMock()
        m.value = self._params.get(name)
        return m
    def create_publisher(self, *a, **k): return MagicMock()
    def create_subscription(self, *a, **k): return None
    def create_client(self, *a, **k): return MagicMock()
    def create_timer(self, *a, **k): return MagicMock()
    def get_logger(self): return MagicMock()

sys.modules["rclpy"] = MagicMock()
sys.modules["rclpy.node"] = MagicMock()
sys.modules["rclpy.node"].Node = _FakeNode
sys.modules["rclpy.qos"] = MagicMock()
sys.modules["rclpy.qos"].QoSProfile = _FakeQoSProfile
sys.modules["rclpy.qos"].DurabilityPolicy = _FakeDurabilityPolicy
sys.modules["sensor_msgs.msg"] = MagicMock()
sys.modules["std_msgs.msg"] = MagicMock()
sys.modules["std_srvs.srv"] = MagicMock()
sys.modules["mia_hand_msgs.msg"] = MagicMock()

from pipeline_manager.pipeline_manager_node import PipelineManagerNode


# ── Test helper ───────────────────────────────────────────────────────────────

def _make_node(params: dict) -> PipelineManagerNode:
    """Construct a PipelineManagerNode with all internals mocked."""
    class _TestNode(PipelineManagerNode):
        def __init__(self, params):
            # Skip the real ROS Node.__init__ entirely
            self._params = params
            self._state = 0  # State.IDLE
            self._history = []
            self._grasp_type = 0
            self._latest_confidence = 0.0
            self._latest_joint_positions = {}
            self._release_confidence_frames = 0
            self._release_monitor_timer = None
            self._release_monitor_start_time = 0.0
            self._force_emergency = False

            self._confidence_threshold = params["confidence_threshold"]
            self._release_confidence_threshold = params["release_confidence_threshold"]
            self._grasp_gestures = params["grasp_gestures"]
            self._release_gesture = params["release_gesture"]
            self._release_debounce_frames = params["release_debounce_frames"]
            self._abort_gestures = params["abort_gestures"]
            self._emergency_stop_gesture = params["emergency_stop_gesture"]
            self._manual_tighten_gesture = params["manual_tighten_gesture"]
            self._manual_loosen_gesture = params["manual_loosen_gesture"]
            self._release_open_position = params["release_open_position"]
            self._release_joint_states_topic = params["release_joint_states_topic"]
            self._release_timeout_s = params["release_timeout_s"]
            self._release_joint_threshold = params["release_joint_threshold"]
            self._emg_live_config_path = params["emg_live_config_path"]
            self._live_config_mtime = None

    return _TestNode(params)


@pytest.fixture
def base_params():
    return {
        "confidence_threshold": 0.55,
        "release_confidence_threshold": 0.25,
        "grasp_gestures": [1, 2, 4],
        "release_gesture": 3,
        "release_debounce_frames": 3,
        "release_open_position": [0.0, 0.0, 0.0],
        "release_joint_states_topic": "/joint_states",
        "release_timeout_s": 3.0,
        "release_joint_threshold": 0.1,
        "abort_gestures": [0],
        "emergency_stop_gesture": -1,
        "manual_tighten_gesture": -1,
        "manual_loosen_gesture": -1,
        "emg_live_config_path": "/tmp/test_emg_live.yaml",
    }


# ── Tests ─────────────────────────────────────────────────────────────────────

class TestLiveConfigAbsentFileFallback:
    """mvp-9om.2: When live config is absent, node should use ROS parameters."""

    def test_absent_file_uses_ros_params(self, base_params, tmp_path):
        base_params["emg_live_config_path"] = str(tmp_path / "does_not_exist.yaml")
        node = _make_node(base_params)
        node._load_live_config_at_startup()
        assert node._confidence_threshold == 0.55
        assert node._release_confidence_threshold == 0.25
        assert node._grasp_gestures == [1, 2, 4]
        assert node._release_gesture == 3


class TestLiveConfigStartupOverride:
    """mvp-9om.2: When live config exists at startup, it should override ROS params."""

    def test_valid_startup_override(self, base_params, tmp_path):
        config_path = tmp_path / "emg_live.yaml"
        config_path.write_text(yaml.safe_dump({
            "confidence_threshold": 0.70,
            "release_confidence_threshold": 0.20,
            "grasp_gestures": [1],
            "release_gesture": 3,
        }))
        base_params["emg_live_config_path"] = str(config_path)
        base_params["confidence_threshold"] = 0.55

        node = _make_node(base_params)
        node._load_live_config_at_startup()
        assert node._confidence_threshold == 0.70
        assert node._release_confidence_threshold == 0.20
        assert node._grasp_gestures == [1]
        assert node._release_gesture == 3


class TestLiveConfigMtimeReload:
    """mvp-9om.3: Config should reload when file mtime changes."""

    def test_mtime_reload_applies_changes(self, base_params, tmp_path):
        config_path = tmp_path / "emg_live.yaml"
        config_path.write_text(yaml.safe_dump({
            "confidence_threshold": 0.60,
        }))
        base_params["emg_live_config_path"] = str(config_path)

        node = _make_node(base_params)
        node._load_live_config_at_startup()
        assert node._confidence_threshold == 0.60

        time.sleep(0.1)
        config_path.write_text(yaml.safe_dump({
            "confidence_threshold": 0.80,
        }))

        node._poll_live_config()
        assert node._confidence_threshold == 0.80


class TestLiveConfigInvalidEditPreservation:
    """mvp-9om.4: Invalid config edits should preserve previous runtime values."""

    def test_invalid_value_keeps_previous(self, base_params, tmp_path):
        config_path = tmp_path / "emg_live.yaml"
        config_path.write_text(yaml.safe_dump({
            "confidence_threshold": 0.60,
        }))
        base_params["emg_live_config_path"] = str(config_path)

        node = _make_node(base_params)
        node._load_live_config_at_startup()
        assert node._confidence_threshold == 0.60

        time.sleep(0.1)
        config_path.write_text(yaml.safe_dump({
            "confidence_threshold": "not_a_number",
        }))

        node._poll_live_config()
        assert node._confidence_threshold == 0.60

    def test_out_of_range_value_keeps_previous(self, base_params, tmp_path):
        config_path = tmp_path / "emg_live.yaml"
        config_path.write_text(yaml.safe_dump({
            "confidence_threshold": 0.60,
        }))
        base_params["emg_live_config_path"] = str(config_path)

        node = _make_node(base_params)
        node._load_live_config_at_startup()

        time.sleep(0.1)
        config_path.write_text(yaml.safe_dump({
            "confidence_threshold": 1.5,
        }))

        node._poll_live_config()
        assert node._confidence_threshold == 0.60

    def test_malformed_yaml_graceful(self, base_params, tmp_path):
        config_path = tmp_path / "emg_live.yaml"
        config_path.write_text("confidence_threshold: [0.55  # malformed yaml")
        base_params["emg_live_config_path"] = str(config_path)

        node = _make_node(base_params)
        node._load_live_config_at_startup()
        assert node._confidence_threshold == 0.55
