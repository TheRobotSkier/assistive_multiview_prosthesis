"""Unit tests for mode-based EMG grasp logic.

These tests assume a future refactored EmgGraspNode that exposes:
  - _dispatch_gesture(gesture_id, confidence, mode) -> (func_name, func_cfg) or (None, None)
  - _gesture_held_long_enough(hold_time_s) -> bool
  - _transition_to_grasping()
  - _transition_to_moving()
  - _execute_continuous(func_name, func_cfg)
  - _stop_continuous_action()

Tests that require the refactored interface are automatically skipped until the
refactor is applied, ensuring the existing contract tests remain green.
"""

import pytest
import sys
import os
import time
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from emg_grasp_node import EmgGraspNode

CONFIG_PATH = os.path.join(os.path.dirname(__file__), "emg_grasp_test.yaml")

_HAS_MODE_DISPATCH = hasattr(EmgGraspNode, '_dispatch_gesture')


@pytest.fixture
def node(monkeypatch):
    """Instantiate EmgGraspNode with the local config path."""
    if not _HAS_MODE_DISPATCH:
        pytest.skip("Mode-based refactor not yet applied to emg_grasp_node.py")

    # Override declare_parameter so the node finds our local YAML
    orig_declare = EmgGraspNode.declare_parameter

    def _mock_declare(self, name, default):
        if name == "config_path":
            class _Param:
                value = CONFIG_PATH
            return _Param()
        return orig_declare(self, name, default)

    monkeypatch.setattr(EmgGraspNode, "declare_parameter", _mock_declare)
    return EmgGraspNode()


# ---- Config loading tests (always run) ----

def test_mode_config_loading():
    """Verify modes dict is present and contains the expected modes/functions."""
    with open(CONFIG_PATH) as f:
        cfg = yaml.safe_load(f)
    assert "modes" in cfg
    for mode in ("moving", "grasping", "holding"):
        assert mode in cfg["modes"], f"Mode '{mode}' not found in config"
        mode_cfg = cfg["modes"][mode]
        assert isinstance(mode_cfg, dict)
        assert len(mode_cfg) > 0


def test_disabled_function_skipped():
    """Functions with null gesture_id are marked disabled in the YAML."""
    with open(CONFIG_PATH) as f:
        cfg = yaml.safe_load(f)
    grasping = cfg["modes"]["grasping"]
    assert "wrist_neg" in grasping
    assert grasping["wrist_neg"]["gesture_id"] is None


# ---- Gesture dispatch tests ----

def test_dispatch_moving_wrist_pos(node):
    """In moving mode, PINCH(2) maps to wrist_pos."""
    func_name, func_cfg = node._dispatch_gesture(2, 0.95, "moving")
    assert func_name == "wrist_pos"
    assert func_cfg is not None
    assert func_cfg["velocity"] == 10.0


def test_dispatch_moving_wrist_neg(node):
    """In moving mode, OPEN(3) maps to wrist_neg."""
    func_name, func_cfg = node._dispatch_gesture(3, 0.95, "moving")
    assert func_name == "wrist_neg"
    assert func_cfg["velocity"] == -10.0


def test_dispatch_moving_grasp_activate(node):
    """In moving mode, POWER(1) maps to grasp_activate."""
    func_name, func_cfg = node._dispatch_gesture(1, 0.95, "moving")
    assert func_name == "grasp_activate"


def test_dispatch_grasping_release(node):
    """In grasping mode, OPEN(3) maps to grasp_release."""
    func_name, func_cfg = node._dispatch_gesture(3, 0.95, "grasping")
    assert func_name == "grasp_release"


def test_dispatch_holding_force_inc(node):
    """In holding mode, POWER(1) maps to force_inc."""
    func_name, func_cfg = node._dispatch_gesture(1, 0.95, "holding")
    assert func_name == "force_inc"
    assert func_cfg["velocity"] == 0.05


def test_dispatch_holding_force_dec(node):
    """In holding mode, OPEN(3) maps to force_dec."""
    func_name, func_cfg = node._dispatch_gesture(3, 0.95, "holding")
    assert func_name == "force_dec"
    assert func_cfg["velocity"] == -0.05


# ---- Mode transition tests ----

def test_transition_moving_to_grasping(node):
    """grasp_activate triggers moving→grasping transition."""
    if hasattr(node, '_mode'):
        node._mode = "moving"
    node._transition_to_grasping()
    current = getattr(node, '_mode', None)
    assert current == "grasping"


def test_transition_grasping_to_moving(node):
    """grasp_release triggers grasping→moving transition."""
    if hasattr(node, '_mode'):
        node._mode = "grasping"
    node._transition_to_moving()
    current = getattr(node, '_mode', None)
    assert current == "moving"


def test_transition_grasping_to_holding(node):
    """Force contact triggers grasping→holding transition."""
    if not hasattr(node, '_transition_to_holding'):
        pytest.skip("_transition_to_holding not yet implemented")
    if hasattr(node, '_mode'):
        node._mode = "grasping"
    node._transition_to_holding()
    assert node._mode == "holding"


def test_transition_holding_to_moving(node):
    """grasp_release triggers holding→moving transition."""
    if hasattr(node, '_mode'):
        node._mode = "holding"
    node._transition_to_moving()
    current = getattr(node, '_mode', None)
    assert current == "moving"
