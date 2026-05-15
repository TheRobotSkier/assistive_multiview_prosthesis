"""Tests for PipelineManagerNode state machine without ROS hardware."""

from __future__ import annotations

import enum
import sys
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

_PROJECT_ROOT = Path(__file__).resolve().parents[1]
_SRC = _PROJECT_ROOT / "src" / "pipeline_manager" / "pipeline_manager"
sys.path.insert(0, str(_SRC))

# Duplicate constants to avoid importing pipeline_manager_node at module level
# (it imports rclpy which may not be installed).


class State(enum.IntEnum):
    IDLE = 0
    SEGMENTING = 1
    PLANNING = 2
    APPROACHING = 3
    GRASPING = 4
    HOLDING = 5
    RELEASING = 6


GESTURE_OPEN = 3
GESTURE_POWER = 1


def make_node():
    """Create a PipelineManagerNode with mocked ROS infrastructure."""
    with (
        patch.dict(sys.modules, {
            'rclpy': MagicMock(),
            'rclpy.node': MagicMock(),
            'rclpy.qos': MagicMock(),
        }),
        patch('builtins.super'),
    ):
        from std_msgs.msg import Int32, String, Float32  # noqa: F811
        from pipeline_manager_node import PipelineManagerNode

        node = object.__new__(PipelineManagerNode)

        # Manually init attributes (skipping ROS __init__)
        node._confidence_threshold = 0.55
        node._grasp_gestures = [GESTURE_POWER]
        node._release_gesture = GESTURE_OPEN
        node._state = State.IDLE
        node._history = []
        node._grasp_type = 0

        node._state_pub = MagicMock()
        node._state_name_pub = MagicMock()
        node.get_logger = MagicMock(return_value=MagicMock())

        def fake_create_timer(period, callback):
            mock_timer = MagicMock()
            mock_timer.callback = callback
            return mock_timer

        node.create_timer = MagicMock(side_effect=fake_create_timer)
        node._compute_client = MagicMock()
        node._publish_state()

        return node


class TestTransitionFix:

    def test_transition_does_not_throw_name_error(self):
        node = make_node()
        result = node._transition(State.SEGMENTING, "test transition")
        assert result is True
        assert node._state == State.SEGMENTING
        assert len(node._history) == 1

    def test_transition_to_same_state_returns_false(self):
        node = make_node()
        node._transition(State.SEGMENTING, "first")
        result = node._transition(State.SEGMENTING, "same state")
        assert result is False
        assert len(node._history) == 1

    def test_transition_name_in_log_message(self):
        node = make_node()
        node._transition(State.SEGMENTING, "check log")
        logger = node.get_logger.return_value
        logger.info.assert_called_once()
        msg = logger.info.call_args[0][0]
        assert "IDLE -> SEGMENTING" in msg
        assert "check log" in msg


class TestReleaseTimerFix:

    def _push_to_holding(self, node):
        node._transition(State.SEGMENTING, "grasp trigger")
        node._transition(State.PLANNING, "seg done")
        node._transition(State.APPROACHING, "preshaped")
        node._transition(State.GRASPING, "proximity")
        node._transition(State.HOLDING, "contact stable")

    def test_open_gesture_in_holding_transitions_to_releasing(self):
        node = make_node()
        self._push_to_holding(node)
        assert node._state == State.HOLDING

        node._on_emg_gesture(MagicMock(data=GESTURE_OPEN))

        assert node._state == State.RELEASING
        timer_calls = [
            c for c in node.create_timer.call_args_list
            if len(c[0]) >= 2 and c[0][1] == node._on_release_timer
        ]
        assert len(timer_calls) == 1

    def test_release_timer_callback_transitions_to_idle_and_cancels(self):
        node = make_node()
        self._push_to_holding(node)
        node._on_emg_gesture(MagicMock(data=GESTURE_OPEN))
        assert node._state == State.RELEASING

        node._on_release_timer()

        assert node._state == State.IDLE
        node._release_timer.cancel.assert_called_once()

    def test_open_gesture_in_idle_does_nothing(self):
        node = make_node()
        assert node._state == State.IDLE

        node._on_emg_gesture(MagicMock(data=GESTURE_OPEN))

        assert node._state == State.IDLE
        assert len(node._history) == 0

    def test_release_timer_no_double_cancel_if_called_again(self):
        node = make_node()
        self._push_to_holding(node)
        node._on_emg_gesture(MagicMock(data=GESTURE_OPEN))
        node._on_release_timer()

        assert node._state == State.IDLE
        cancel_count = node._release_timer.cancel.call_count
        assert cancel_count == 1

        node._on_release_timer()
        assert node._state == State.IDLE
        assert node._release_timer.cancel.call_count == cancel_count + 1
