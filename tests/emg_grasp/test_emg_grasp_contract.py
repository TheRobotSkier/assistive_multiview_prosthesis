"""Unit tests for EMG grasp logic (no ROS required).

Tests the velocity ramp computation and stop condition logic in isolation.
"""

import pytest
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from emg_grasp_node import EmgGraspNode


def test_velocity_ramp_linear():
    ramp = EmgGraspNode._compute_velocity_ramp(0.3, 0.1, 5)
    assert len(ramp) == 5
    assert ramp[0] == pytest.approx(0.3)
    assert ramp[-1] == pytest.approx(0.1)
    assert ramp[2] == pytest.approx(0.2)


def test_velocity_ramp_single_step():
    ramp = EmgGraspNode._compute_velocity_ramp(0.5, 0.1, 1)
    assert ramp == [0.5]


def test_check_stop_conditions_force():
    reason = EmgGraspNode.check_stop_conditions(
        [0.5, 0.5, 0.5], [200, 350, 200], [1.5, 1.5, 1.5], [300, 300, 300]
    )
    assert reason is not None
    assert "FORCE CONTACT on j_index_fle" in reason


def test_check_stop_conditions_position():
    reason = EmgGraspNode.check_stop_conditions(
        [0.5, 1.6, 0.5], [200, 200, 200], [1.5, 1.5, 1.5], [300, 300, 300]
    )
    assert reason is not None
    assert "STOP POSITION reached on j_index_fle" in reason


def test_check_stop_conditions_none():
    reason = EmgGraspNode.check_stop_conditions(
        [0.5, 0.5, 0.5], [200, 200, 200], [1.5, 1.5, 1.5], [300, 300, 300]
    )
    assert reason is None
