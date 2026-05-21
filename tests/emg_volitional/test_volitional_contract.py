"""Unit tests for EMG volitional controller logic (no ROS required)."""

import pytest
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from emg_volitional_node import EmgVolitionalNode


def test_lookup_velocity_known_gesture():
    vels = {0: 0.0, 1: 0.3, 3: -0.3}
    assert EmgVolitionalNode.lookup_velocity(1, 0.8, vels, 0.5, 0) == pytest.approx(0.3)
    assert EmgVolitionalNode.lookup_velocity(3, 0.8, vels, 0.5, 0) == pytest.approx(-0.3)


def test_lookup_velocity_rest_is_stop():
    vels = {0: 0.0, 1: 0.3}
    assert EmgVolitionalNode.lookup_velocity(0, 0.8, vels, 0.5, 0) == pytest.approx(0.0)


def test_lookup_velocity_emergency_stop():
    vels = {0: 0.0, 1: 0.3}
    # Even with high confidence, emergency stop gesture returns 0.0
    assert EmgVolitionalNode.lookup_velocity(0, 0.99, vels, 0.5, emergency_stop_gesture=0) == 0.0


def test_lookup_velocity_low_confidence():
    vels = {1: 0.3}
    assert EmgVolitionalNode.lookup_velocity(1, 0.3, vels, 0.5, 0) == 0.0


def test_lookup_velocity_unknown_gesture():
    vels = {0: 0.0, 1: 0.3}
    assert EmgVolitionalNode.lookup_velocity(99, 0.8, vels, 0.5, 0) == 0.0


def test_clamp_velocity_at_max_limit():
    positions = [3.0, 3.0, 3.0]  # at max
    assert EmgVolitionalNode.clamp_velocity_by_position_limits(0.3, positions, 0.0, 3.0) == 0.0


def test_clamp_velocity_at_min_limit():
    positions = [0.0, 0.0, 0.0]  # at min
    assert EmgVolitionalNode.clamp_velocity_by_position_limits(-0.3, positions, 0.0, 3.0) == 0.0


def test_clamp_velocity_within_limits():
    positions = [1.5, 1.5, 1.5]
    assert EmgVolitionalNode.clamp_velocity_by_position_limits(0.3, positions, 0.0, 3.0) == pytest.approx(0.3)
    assert EmgVolitionalNode.clamp_velocity_by_position_limits(-0.3, positions, 0.0, 3.0) == pytest.approx(-0.3)
