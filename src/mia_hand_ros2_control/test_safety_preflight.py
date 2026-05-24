#!/usr/bin/env python3
"""Unit tests for EmgSafetyPreflight — does NOT require ROS or hardware.

These tests validate:
  - Parameter defaults and edge cases
  - Preflight error conditions (service timeouts, missing controllers)
  - Runtime safety threshold logic (stale data, max force, command rate)
  - Cleanup idempotency
  - Signal handler registration
  - Process PID registration and cleanup
"""

from __future__ import annotations

import math
import os
import signal
import time
from unittest import mock

import pytest

# Import the module under test (requires rclpy to be importable)
pytest.importorskip("rclpy")
pytest.importorskip("std_srvs")
pytest.importorskip("sensor_msgs")

from mia_hand_ros2_control.emg_safety_preflight import (
    EmgSafetyPreflight,
    PreflightError,
    SafetyViolation,
    _DEFAULT_PARAMS,
    _SAFE_OPEN_POSITIONS,
    _JOINT_NAMES,
)


# ── Fixtures ────────────────────────────────────────────────────────────────

@pytest.fixture
def node_mock():
    """Create a mock rclpy Node with parameter support."""
    with mock.patch("rclpy.node.Node", autospec=True) as MockNode:
        instance = MockNode.return_value
        instance.get_logger.return_value = mock.MagicMock()

        # Parameter storage
        _params = dict(_DEFAULT_PARAMS)

        def _get_param(key):
            if hasattr(instance, "_mock_params") and key in instance._mock_params:
                return mock.MagicMock(value=instance._mock_params[key])
            return mock.MagicMock(value=_params.get(key))

        instance.get_parameter = mock.MagicMock(side_effect=_get_param)

        def _declare_param(key, default):
            if not hasattr(instance, "_mock_params"):
                instance._mock_params = {}
            instance._mock_params[key] = default

        instance.declare_parameter = mock.MagicMock(side_effect=_declare_param)

        # For publisher info
        instance.get_publishers_info_by_topic = mock.MagicMock(
            return_value=[]
        )

        yield instance


@pytest.fixture
def safety(node_mock):
    """Create an EmgSafetyPreflight wrapping a mock node."""
    return EmgSafetyPreflight(node=node_mock)


# ── Test: construction and defaults ─────────────────────────────────────────

class TestConstruction:
    """Verify the module can be constructed with default parameters."""

    def test_creates_with_node(self, node_mock):
        """EmgSafetyPreflight wraps an existing node correctly."""
        s = EmgSafetyPreflight(node=node_mock)
        assert s._node is node_mock
        assert s._own_node is False

    def test_creates_standalone(self):
        """EmgSafetyPreflight creates its own internal node when None."""
        with mock.patch("rclpy.node.Node", autospec=True) as MockNode:
            s = EmgSafetyPreflight(node=None)
            assert s._own_node is True
            MockNode.assert_called_once_with("emg_safety_preflight")

    def test_default_parameters_exist(self, safety):
        """All expected default parameter keys are present."""
        assert "stale_emg_timeout_s" in _DEFAULT_PARAMS
        assert "stale_force_timeout_s" in _DEFAULT_PARAMS
        assert "max_force_limit" in _DEFAULT_PARAMS
        assert "max_wrist_error_deg" in _DEFAULT_PARAMS
        assert "max_command_rate_hz" in _DEFAULT_PARAMS
        assert "safe_open_positions" in _DEFAULT_PARAMS

    def test_required_controllers_list(self, safety):
        """The default required_controllers list is non-empty."""
        req = _DEFAULT_PARAMS["required_controllers"]
        assert len(req) >= 3
        assert "joint_state_broadcaster" in req


# ── Test: preflight errors ─────────────────────────────────────────────────

class TestPreflightErrors:
    """Verify preflight checks fail with descriptive messages."""

    def test_play_service_not_available_raises(self, safety, node_mock):
        """PreflightError when play service is unreachable."""
        client_mock = mock.MagicMock()
        client_mock.wait_for_service.return_value = False
        node_mock.create_client.return_value = client_mock

        with pytest.raises(PreflightError, match="not available"):
            safety._preflight_play()

    def test_play_service_timeout_raises(self, safety, node_mock):
        """PreflightError when play service times out."""
        import rclpy

        client_mock = mock.MagicMock()
        client_mock.wait_for_service.return_value = True
        node_mock.create_client.return_value = client_mock

        # Create a future that is never done
        future_mock = mock.MagicMock()
        future_mock.done.return_value = False

        # rclpy.ok() must return True for spin_until_future_complete to spin
        with mock.patch.object(rclpy, "ok", return_value=True):
            with mock.patch.object(rclpy, "spin_until_future_complete",
                                   return_value=None):
                with pytest.raises(PreflightError, match="timed out"):
                    safety._preflight_play()

    def test_play_service_failure_message(self, safety, node_mock):
        """PreflightError includes the server's failure message."""
        import rclpy

        client_mock = mock.MagicMock()
        client_mock.wait_for_service.return_value = True
        node_mock.create_client.return_value = client_mock

        future_mock = mock.MagicMock()
        future_mock.done.return_value = True
        result_mock = mock.MagicMock()
        result_mock.success = False
        result_mock.message = "Hand not connected"
        future_mock.result.return_value = result_mock

        with mock.patch.object(rclpy, "spin_until_future_complete",
                               return_value=future_mock):
            with pytest.raises(PreflightError, match="Hand not connected"):
                safety._preflight_play()

    def test_controller_manager_not_available(self, safety, node_mock):
        """PreflightError when controller_manager service is unreachable."""
        client_mock = mock.MagicMock()
        client_mock.wait_for_service.return_value = False
        node_mock.create_client.return_value = client_mock

        with pytest.raises(PreflightError, match="not available"):
            safety._preflight_controllers()

    def test_required_controller_not_active(self, safety, node_mock):
        """PreflightError when a required controller is inactive."""
        import rclpy

        client_mock = mock.MagicMock()
        client_mock.wait_for_service.return_value = True
        node_mock.create_client.return_value = client_mock

        future_mock = mock.MagicMock()
        future_mock.done.return_value = True

        result_mock = mock.MagicMock()
        # Only joint_state_broadcaster is active; positions are not
        ctrl_active = mock.MagicMock()
        ctrl_active.name = "joint_state_broadcaster"
        ctrl_active.state = "active"
        ctrl_inactive = mock.MagicMock()
        ctrl_inactive.name = "thumb_pos_ff_controller"
        ctrl_inactive.state = "inactive"
        result_mock.controller = [ctrl_active, ctrl_inactive]
        future_mock.result.return_value = result_mock

        with mock.patch.object(rclpy, "spin_until_future_complete",
                               return_value=future_mock):
            with pytest.raises(PreflightError, match="not active"):
                safety._preflight_controllers()

    def test_missing_controller_in_list(self, safety, node_mock):
        """PreflightError when a required controller is entirely absent."""
        import rclpy

        client_mock = mock.MagicMock()
        client_mock.wait_for_service.return_value = True
        node_mock.create_client.return_value = client_mock

        future_mock = mock.MagicMock()
        future_mock.done.return_value = True

        result_mock = mock.MagicMock()
        result_mock.controller = []  # no controllers at all
        future_mock.result.return_value = result_mock

        with mock.patch.object(rclpy, "spin_until_future_complete",
                               return_value=future_mock):
            with pytest.raises(PreflightError, match="not found"):
                safety._preflight_controllers()

    def test_command_source_conflict_detected(self, safety, node_mock):
        """PreflightError when too many publishers on a command topic."""
        # Mock get_publishers_info_by_topic to return many publishers
        pub_mocks = []
        for i in range(5):
            pm = mock.MagicMock()
            pm.node_name = f"node_{i}"
            pub_mocks.append(pm)
        node_mock.get_publishers_info_by_topic.return_value = pub_mocks

        with pytest.raises(PreflightError, match="Command-source conflicts"):
            safety._preflight_command_conflicts()


# ── Test: runtime safety thresholds ─────────────────────────────────────────

class TestRuntimeSafety:
    """Verify runtime safety check logic."""

    def test_stale_emg_raises_safety_violation(self, safety):
        """SafetyViolation when EMG data exceeds stale timeout."""
        safety._emg_data_received = True
        safety._last_emg_time = time.monotonic() - 10.0  # far in the past

        with pytest.raises(SafetyViolation, match="stale"):
            safety._check_stale_emg()

    def test_stale_emg_not_checked_before_data(self, safety):
        """No violation if EMG has never been received (preflight pending)."""
        safety._emg_data_received = False
        safety._last_emg_time = time.monotonic() - 10.0

        # Should not raise
        safety._check_stale_emg()

    def test_stale_force_emergency_raises(self, safety):
        """SafetyViolation when force data is stale beyond emergency threshold."""
        safety._force_data_received = True
        safety._last_force_time = time.monotonic() - 10.0  # > 2s emergency

        with pytest.raises(SafetyViolation, match="STALE EMERGENCY"):
            safety._check_stale_force()

    def test_stale_force_warning_only_below_emergency(self, safety, node_mock):
        """Only a warning when force is stale but below emergency threshold."""
        safety._force_data_received = True
        safety._last_force_time = time.monotonic() - 0.7  # > 0.5 warning, < 2.0 emergency

        # Should NOT raise — only logs a warning
        safety._check_stale_force()
        # The warning should have been logged
        assert node_mock.get_logger.return_value.warn.called

    def test_max_force_exceeded_raises(self, safety, node_mock):
        """SafetyViolation when force exceeds max_force_limit."""
        safety._force_data_received = True
        safety._max_observed_force = 600.0  # > 500 default

        with pytest.raises(SafetyViolation, match="Max force exceeded"):
            safety._check_max_force()

    def test_max_force_within_limit_passes(self, safety):
        """No violation when force is within limits."""
        safety._force_data_received = True
        safety._max_observed_force = 200.0  # < 500 default

        # Should not raise
        safety._check_max_force()


# ── Test: cleanup ───────────────────────────────────────────────────────────

class TestCleanup:
    """Verify cleanup behavior."""

    def test_cleanup_is_idempotent(self, safety, node_mock):
        """Cleanup can be called multiple times without error."""
        safety.cleanup()
        safety.cleanup()  # second call should be a no-op
        # Should not raise

    def test_cleanup_stops_runtime_monitor(self, safety, node_mock):
        """Cleanup stops the runtime timer."""
        # Create a mock timer
        timer_mock = mock.MagicMock()
        safety._runtime_timer = timer_mock
        safety._runtime_active = True

        safety.cleanup()

        timer_mock.cancel.assert_called_once()
        assert safety._runtime_active is False

    def test_cleanup_registers_as_done(self, safety):
        """After cleanup, _cleaned_up flag is True."""
        assert safety._cleaned_up is False
        safety.cleanup()
        assert safety._cleaned_up is True

    def test_open_hand_publishes_safe_positions(self, safety, node_mock):
        """Cleanup publishes safe positions to all three finger controllers."""
        pub_mock = mock.MagicMock()
        node_mock.create_publisher.return_value = pub_mock

        safety._open_hand_safe()

        # Should have created 3 publishers (thumb, index, mrl)
        assert node_mock.create_publisher.call_count == 3
        # Each publisher should have had publish() called
        assert pub_mock.publish.call_count == 3

    def test_zero_velocity_publishes_to_velocity_topics(self, safety, node_mock):
        """Cleanup zero-velocity publishes to velocity controller topics."""
        pub_mock = mock.MagicMock()
        node_mock.create_publisher.return_value = pub_mock

        safety._zero_velocity()

        # Should have created 3 publishers
        assert node_mock.create_publisher.call_count == 3
        assert pub_mock.publish.call_count == 3

    def test_kill_launched_processes_sends_sigterm(self, safety):
        """Registered PIDs receive SIGTERM during cleanup."""
        safety.register_child_pid(12345)

        with mock.patch("os.kill") as mock_kill:
            safety._kill_launched_processes()
            mock_kill.assert_called_once_with(12345, signal.SIGTERM)

    def test_kill_launched_processes_noop_when_empty(self, safety):
        """No error when there are no launched processes."""
        with mock.patch("os.kill") as mock_kill:
            safety._kill_launched_processes()
            mock_kill.assert_not_called()

    def test_kill_launched_processes_handles_exited_pids(self, safety):
        """Does not raise when a PID has already exited."""
        safety.register_child_pid(99999)

        with mock.patch("os.kill", side_effect=ProcessLookupError):
            # Should not raise
            safety._kill_launched_processes()

    def test_cleanup_step_continues_on_failure(self, safety):
        """Individual cleanup steps log errors but don't abort the sequence."""
        # Make _zero_velocity raise
        safety._cleaned_up = False
        with mock.patch.object(safety, "_zero_velocity",
                               side_effect=RuntimeError("Mock failure")):
            safety.cleanup()  # should complete without raising
            assert safety._cleaned_up is True


# ── Test: signal handlers ───────────────────────────────────────────────────

class TestSignalHandlers:
    """Verify signal handlers are installed correctly."""

    def test_install_signal_handlers_registers_atexit(self, safety):
        """atexit is registered after installing signal handlers."""
        with mock.patch("atexit.register") as mock_register:
            with mock.patch("signal.signal"):
                safety.install_signal_handlers()
                mock_register.assert_called_with(safety.cleanup)

    def test_install_signal_handlers_registers_sigint_sigterm(self, safety):
        """SIGINT and SIGTERM are both registered."""
        with mock.patch("atexit.register"):
            with mock.patch("signal.signal") as mock_signal:
                safety.install_signal_handlers()
                calls = [c[0][0] for c in mock_signal.call_args_list]
                assert signal.SIGINT in calls
                assert signal.SIGTERM in calls


# ── Test: PID registration ──────────────────────────────────────────────────

class TestPidRegistration:
    """Verify process PID tracking."""

    def test_register_child_pid_stores_pid(self, safety):
        """Valid PIDs are added to the internal list."""
        safety.register_child_pid(42)
        assert 42 in safety._launched_pids

    def test_register_child_pid_ignores_invalid(self, safety):
        """PID <= 0 is not stored."""
        safety.register_child_pid(0)
        safety.register_child_pid(-1)
        assert 0 not in safety._launched_pids
        assert -1 not in safety._launched_pids

    def test_multiple_pids_registered(self, safety):
        """Multiple PIDs can be registered and cleaned up."""
        for pid in range(100, 105):
            safety.register_child_pid(pid)
        assert len(safety._launched_pids) == 5


# ── Test: preflight orchestration ────────────────────────────────────────────

class TestRunPreflight:
    """Verify run_preflight() orchestrates checks correctly."""

    def test_run_preflight_returns_false_on_failure(self, safety, node_mock):
        """run_preflight returns False when a check raises PreflightError."""
        client_mock = mock.MagicMock()
        client_mock.wait_for_service.return_value = False
        node_mock.create_client.return_value = client_mock

        result = safety.run_preflight()
        assert result is False
        assert safety._preflight_done is False

    def test_run_preflight_skips_optional_checks(self, safety, node_mock):
        """run_preflight skips force/EMG/wrist when requested."""

        # Mock out the play and controller checks (they use services)
        import rclpy

        # Mock play success
        play_client = mock.MagicMock()
        play_client.wait_for_service.return_value = True
        play_future = mock.MagicMock()
        play_future.done.return_value = True
        play_result = mock.MagicMock()
        play_result.success = True
        play_result.message = "OK"
        play_future.result.return_value = play_result

        node_mock.create_client.return_value = play_client

        with mock.patch.object(rclpy, "spin_until_future_complete",
                               return_value=play_future):
            with mock.patch.object(safety, "_preflight_controllers"):
                with mock.patch.object(safety, "_preflight_force") as mock_force:
                    with mock.patch.object(safety, "_preflight_emg") as mock_emg:
                        result = safety.run_preflight(
                            require_force=False,
                            require_emg=False,
                            require_wrist=False,
                        )
                        mock_force.assert_not_called()
                        mock_emg.assert_not_called()
                        assert result is True
                        assert safety._preflight_done is True


# ── Test: note_command ──────────────────────────────────────────────────────

class TestNoteCommand:
    """Verify command rate tracking."""

    def test_note_command_increments_counter(self, safety):
        """Each call to note_command increments the counter."""
        safety._cmd_count = 0
        safety._last_cmd_time = time.monotonic()

        safety.note_command()
        assert safety._cmd_count == 1

        safety.note_command()
        assert safety._cmd_count == 2

    def test_note_command_resets_after_interval(self, safety):
        """Counter resets after 1 second interval."""
        safety._cmd_count = 5
        safety._last_cmd_time = time.monotonic() - 2.0  # > 1s ago

        safety.note_command()
        assert safety._cmd_count == 1  # reset


# ── Test: helper utilities ──────────────────────────────────────────────────

class TestHelpers:
    """Verify internal helper functions."""

    def test_safe_open_positions_default(self):
        """Default safe open positions are all zeros."""
        assert _SAFE_OPEN_POSITIONS == [0.0, 0.0, 0.0]

    def test_joint_names_are_three_fingers(self):
        """Joint names list contains three expected joints."""
        assert len(_JOINT_NAMES) == 3
        assert "j_thumb_fle" in _JOINT_NAMES
        assert "j_index_fle" in _JOINT_NAMES
        assert "j_mrl_fle" in _JOINT_NAMES

    def test_preflight_error_is_exception(self):
        """PreflightError is a standard Exception subclass."""
        assert issubclass(PreflightError, Exception)

    def test_safety_violation_is_exception(self):
        """SafetyViolation is a standard Exception subclass."""
        assert issubclass(SafetyViolation, Exception)


# ── Run ─────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    pytest.main([__file__, "-v"])
