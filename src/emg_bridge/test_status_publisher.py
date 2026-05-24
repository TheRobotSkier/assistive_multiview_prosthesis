"""Unit tests for EmgStatusPublisher — no ROS required.

Tests:
  - Mode change detection and logging
  - Gesture change logging
  - Force target change throttling
  - Fault add/clear tracking
  - Grasp phase transitions
  - Controller switches
  - Throttled_log timing behaviour
  - Status snapshot consistency
  - update_full bulk update
  - Unknown fault rejection
"""

import sys
import threading
import time
from io import StringIO
from unittest.mock import patch

import pytest

# We import without ROS — HAS_ROS will be False but the class still works
sys.path.insert(0, ".")
from emg_bridge.emg_status_publisher import (
    EmgStatusPublisher,
    EmgGraspStatus,
    EmgMode,
    GraspPhase,
    PowerRearm,
    FaultCode,
)


# ── Helpers ───────────────────────────────────────────────────────────────────

@pytest.fixture
def publisher():
    """Return a fresh EmgStatusPublisher with no ROS node."""
    return EmgStatusPublisher(ros_node=None)


@pytest.fixture
def status():
    """Return a bare EmgGraspStatus (for data-layer tests)."""
    return EmgGraspStatus()


def _captured_logs(pub: EmgStatusPublisher, fn, *args, **kwargs):
    """Call fn and return captured print output."""
    buf = StringIO()
    with patch("sys.stdout", buf):
        fn(*args, **kwargs)
    return buf.getvalue()


# ── EmgGraspStatus — basic properties ─────────────────────────────────────────

class TestStatusSnapshot:
    def test_defaults(self, status):
        assert status.mode == EmgMode.NOT_GRASPING
        assert status.gesture_name == "REST"
        assert status.confidence == 0.0
        assert status.grasp_phase == GraspPhase.IDLE
        assert status.faults == set()
        assert status.preflight_ok is False

    def test_thread_safety_basic(self, status):
        def _writer():
            for i in range(100):
                with status.lock:
                    status.mode = EmgMode.CONTROL_GRASP
                    status.confidence = float(i)

        def _reader():
            for _ in range(100):
                with status.lock:
                    _mode = status.mode
                    _conf = status.confidence

        threads = [
            threading.Thread(target=_writer),
            threading.Thread(target=_reader),
            threading.Thread(target=_reader),
        ]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        # No crash = pass


# ── Mode change logging ──────────────────────────────────────────────────────

class TestModeChanges:
    def test_initial_mode_is_logged_on_change(self, publisher):
        out = _captured_logs(publisher, publisher.set_mode, EmgMode.CONTROL_GRASP)
        assert "MODE:" in out
        assert "not-grasping" in out
        assert "control-grasp" in out

    def test_no_log_when_mode_unchanged(self, publisher):
        # First change — logged
        out1 = _captured_logs(publisher, publisher.set_mode, EmgMode.CONTROL_GRASP)
        assert "MODE:" in out1
        # Second call with same mode — not logged
        out2 = _captured_logs(publisher, publisher.set_mode, EmgMode.CONTROL_GRASP)
        assert "MODE:" not in out2

    def test_mode_round_trip(self, publisher):
        out = _captured_logs(publisher, publisher.set_mode, EmgMode.CONTROL_WRIST)
        assert "control-wrist" in out
        out = _captured_logs(publisher, publisher.set_mode, EmgMode.NOT_GRASPING)
        assert "not-grasping" in out


# ── Gesture change logging ───────────────────────────────────────────────────

class TestGestureChanges:
    def test_gesture_change_logged(self, publisher):
        out = _captured_logs(publisher, publisher.set_gesture, "POWER", 0.85, 0.6)
        assert "GESTURE:" in out
        assert "POWER" in out
        assert "0.85" in out
        assert "0.60" in out

    def test_gesture_unchanged_not_logged(self, publisher):
        publisher.set_gesture("POWER", 0.85)
        out = _captured_logs(publisher, publisher.set_gesture, "POWER", 0.90, 0.7)
        assert "GESTURE:" not in out  # name unchanged

    def test_gesture_reverts_to_rest(self, publisher):
        publisher.set_gesture("POWER", 0.85)
        out = _captured_logs(publisher, publisher.set_gesture, "REST", 0.0)
        assert "REST" in out


# ── Force target logging ─────────────────────────────────────────────────────

class TestForceTargetLogging:
    def test_force_change_logged_when_delta_exceeds_1(self, publisher):
        publisher.set_target_forces([50.0, 50.0, 50.0])
        # Elapse the force-log throttle so the next call goes through
        publisher._last_force_log = time.monotonic() - publisher.FORCE_LOG_INTERVAL - 0.1
        out = _captured_logs(publisher, publisher.set_target_forces, [55.0, 80.0, 120.0])
        assert "TARGET FORCE:" in out

    def test_force_change_not_logged_when_delta_is_1(self, publisher):
        publisher.set_target_forces([50.0, 50.0, 50.0])
        # Elapse throttle so the next call isn't blocked by timing
        publisher._last_force_log = time.monotonic() - publisher.FORCE_LOG_INTERVAL - 0.1
        out = _captured_logs(publisher, publisher.set_target_forces, [51.0, 50.0, 50.0])
        assert "TARGET FORCE:" not in out  # delta ≤ 1.0

    def test_force_unchanged_not_logged(self, publisher):
        publisher.set_target_forces([100.0, 100.0, 100.0])
        _captured_logs(publisher, publisher.set_target_forces, [100.0, 100.0, 100.0])
        out = _captured_logs(publisher, publisher.set_target_forces, [100.0, 100.0, 100.0])
        assert "TARGET FORCE:" not in out

    def test_force_throttled_by_interval(self, publisher):
        publisher._last_force_log = time.monotonic()  # fake recent log
        publisher._prev_target = [0, 0, 0]  # force delta > 1
        out = _captured_logs(publisher, publisher.set_target_forces, [200.0, 200.0, 200.0])
        assert "TARGET FORCE:" not in out  # throttled — within interval


# ── Grasp phase logging ──────────────────────────────────────────────────────

class TestGraspPhaseLogging:
    def test_phase_transition_logged(self, publisher):
        out = _captured_logs(publisher, publisher.set_grasp_phase, GraspPhase.CLOSING)
        assert "GRASP PHASE:" in out
        assert "idle" in out
        assert "closing" in out

    def test_phase_chain(self, publisher):
        publisher.set_grasp_phase(GraspPhase.CLOSING)
        publisher.set_grasp_phase(GraspPhase.FORCE_HOLD)
        out = _captured_logs(publisher, publisher.set_grasp_phase, GraspPhase.RELEASING)
        assert "force_hold" in out
        assert "releasing" in out


# ── Fault management ─────────────────────────────────────────────────────────

class TestFaultManagement:
    def test_add_fault_logs(self, publisher):
        out = _captured_logs(publisher, publisher.add_fault, FaultCode.STALE_EMG)
        assert "FAULT:" in out
        assert "STALE_EMG" in out

    def test_add_same_fault_twice_no_duplicate_log(self, publisher):
        publisher.add_fault(FaultCode.STALE_EMG)
        out = _captured_logs(publisher, publisher.add_fault, FaultCode.STALE_EMG)
        assert "FAULT:" not in out

    def test_clear_fault_logs(self, publisher):
        publisher.add_fault(FaultCode.OVERFORCE)
        out = _captured_logs(publisher, publisher.clear_fault, FaultCode.OVERFORCE)
        assert "FAULT CLEARED:" in out

    def test_clear_nonexistent_fault_no_log(self, publisher):
        out = _captured_logs(publisher, publisher.clear_fault, FaultCode.OVERFORCE)
        assert "FAULT" not in out

    def test_multiple_faults(self, publisher):
        publisher.add_fault(FaultCode.STALE_EMG)
        publisher.add_fault(FaultCode.CTRL_UNAVAILABLE)
        with publisher._state.lock:
            assert publisher._state.faults == {"STALE_EMG", "CTRL_UNAVAILABLE"}

    def test_unknown_fault_raises(self, publisher):
        with pytest.raises(ValueError, match="Unknown fault code"):
            publisher.add_fault("BOGUS_FAULT")

    def test_all_valid_fault_codes(self, publisher):
        for code in sorted(FaultCode.ALL):
            publisher.add_fault(code)
        assert publisher._state.faults == set(FaultCode.ALL)


# ── Controller switches ──────────────────────────────────────────────────────

class TestControllerSwitches:
    def test_controller_change_logged(self, publisher):
        out = _captured_logs(publisher, publisher.set_hand_controller, "velocity")
        assert "HAND CONTROLLER:" in out
        assert "none" in out
        assert "velocity" in out

    def test_controller_unchanged_not_logged(self, publisher):
        publisher.set_hand_controller("force")
        out = _captured_logs(publisher, publisher.set_hand_controller, "force")
        assert "HAND CONTROLLER:" not in out


# ── Throttle behaviour ───────────────────────────────────────────────────────

class TestThrottling:
    def test_routine_log_throttled(self, publisher):
        # First call — should log
        out1 = _captured_logs(publisher, publisher.throttled_log)
        assert "EMG Grasp @" in out1

        # Immediate second call — should NOT log (throttled)
        out2 = _captured_logs(publisher, publisher.throttled_log)
        assert "EMG Grasp @" not in out2

    def test_routine_log_after_interval(self, publisher):
        _captured_logs(publisher, publisher.throttled_log)
        # Fake elapsed time for both interval guards
        publisher._last_routine_log = time.monotonic() - publisher.ROUTINE_INTERVAL - 0.1
        publisher._last_any_log = time.monotonic() - publisher.MIN_LOG_INTERVAL - 0.1
        out = _captured_logs(publisher, publisher.throttled_log)
        assert "EMG Grasp @" in out

    def test_important_events_always_log(self, publisher):
        """Mode changes, faults etc. ignore throttling."""
        # Saturate throttle
        publisher._last_routine_log = time.monotonic()
        publisher._last_any_log = time.monotonic()

        out = _captured_logs(publisher, publisher.add_fault, FaultCode.STALE_FORCE)
        assert "FAULT:" in out  # Always logged regardless of throttle


# ── Threshold / sub-threshold ────────────────────────────────────────────────

class TestThresholds:
    """Tests that the publisher correctly tracks thresholds."""
    def test_confidence_below_threshold(self, publisher):
        publisher.set_gesture("REST", 0.3)
        with publisher._state.lock:
            assert publisher._state.confidence == 0.3

    def test_proportional_high(self, publisher):
        publisher.set_gesture("POWER", 0.9, 0.95)
        with publisher._state.lock:
            assert publisher._state.proportional == 0.95

    def test_wrist_values(self, publisher):
        publisher.set_wrist("rotating", 15.0)
        with publisher._state.lock:
            assert publisher._state.wrist_state == "rotating"
            assert publisher._state.wrist_cmd == 15.0


# ── update_full bulk ─────────────────────────────────────────────────────────

class TestUpdateFull:
    def test_update_full_mode_and_gesture(self, publisher):
        publisher.update_full(
            mode=EmgMode.CONTROL_GRASP,
            gesture_name="POWER",
            confidence=0.88,
        )
        with publisher._state.lock:
            s = publisher._state
            assert s.mode == EmgMode.CONTROL_GRASP
            assert s.gesture_name == "POWER"
            assert s.confidence == 0.88

    def test_update_full_partial(self, publisher):
        publisher.update_full(grasp_phase=GraspPhase.CLOSING)
        with publisher._state.lock:
            assert publisher._state.grasp_phase == GraspPhase.CLOSING
            assert publisher._state.mode == EmgMode.NOT_GRASPING  # unchanged

    def test_update_full_forces(self, publisher):
        publisher.update_full(
            target_forces=[100.0, 200.0, 150.0],
            measured_forces=[95.0, 190.0, 145.0],
            force_errors=[5.0, 10.0, 5.0],
        )
        with publisher._state.lock:
            assert publisher._state.target_forces == [100.0, 200.0, 150.0]
            assert publisher._state.measured_forces == [95.0, 190.0, 145.0]
            assert publisher._state.force_errors == [5.0, 10.0, 5.0]

    def test_update_full_none_skipped(self, publisher):
        """None arguments should not change state."""
        publisher.set_mode(EmgMode.CONTROL_GRASP)
        publisher.update_full(mode=None, gesture_name=None)
        with publisher._state.lock:
            assert publisher._state.mode == EmgMode.CONTROL_GRASP
            assert publisher._state.gesture_name == "REST"

    def test_update_full_confidence_only(self, publisher):
        publisher.update_full(confidence=0.75, proportional=0.5)
        with publisher._state.lock:
            assert publisher._state.confidence == 0.75
            assert publisher._state.proportional == 0.5
            assert publisher._state.gesture_name == "REST"  # unchanged


# ── Preflight ────────────────────────────────────────────────────────────────

class TestPreflight:
    def test_preflight_default_false(self, publisher):
        with publisher._state.lock:
            assert publisher._state.preflight_ok is False

    def test_preflight_set_true(self, publisher):
        publisher.set_preflight(True)
        with publisher._state.lock:
            assert publisher._state.preflight_ok is True


# ── ROS publish (offline — no crash) ─────────────────────────────────────────

class TestRosPublishOffline:
    def test_publish_ros_noop_without_node(self, publisher):
        """publish_ros should be a safe no-op when no ROS node was provided."""
        publisher.publish_ros()  # Must not raise

    def test_start_ros_node_returns_none_without_ros(self, status):
        """start_ros_node returns None when rclpy is not installed."""
        result = None
        # The function checks HAS_ROS — which is False in test env
        from emg_bridge.emg_status_publisher import start_ros_node
        result = start_ros_node(status)
        assert result is None


# ── Status text serialization ────────────────────────────────────────────────

class TestStatusText:
    def test_text_contains_all_keys(self):
        from emg_bridge.emg_status_publisher import _build_status_text
        s = EmgGraspStatus(
            mode=EmgMode.CONTROL_GRASP,
            gesture_name="POWER",
            confidence=0.92,
            proportional=0.65,
            grasp_phase=GraspPhase.CLOSING,
            faults={FaultCode.STALE_FORCE},
            preflight_ok=True,
        )
        text = _build_status_text(s, 42)
        assert "seq:42" in text
        assert "mode:control-grasp" in text
        assert "gesture:POWER" in text
        assert "conf:0.920" in text
        assert "prop:0.650" in text
        assert "grasp_phase:closing" in text
        assert "faults:STALE_FORCE" in text
        assert "preflight:ok" in text

    def test_text_no_faults(self):
        from emg_bridge.emg_status_publisher import _build_status_text
        s = EmgGraspStatus()
        text = _build_status_text(s, 1)
        assert "faults:none" in text

    def test_text_preflight_not_ready(self):
        from emg_bridge.emg_status_publisher import _build_status_text
        s = EmgGraspStatus(preflight_ok=False)
        text = _build_status_text(s, 1)
        assert "preflight:not_ready" in text


# ── Numeric serialization ────────────────────────────────────────────────────

class TestNumeric:
    def test_numeric_requires_ros(self):
        """_build_numeric raises RuntimeError when ROS is not installed."""
        from emg_bridge.emg_status_publisher import _build_numeric
        s = EmgGraspStatus(
            confidence=0.85,
            proportional=0.7,
            open_hold_s=3.5,
            target_forces=[100, 200, 150],
            measured_forces=[95, 190, 145],
            force_errors=[5, 10, 5],
            wrist_cmd=12.5,
            preflight_ok=True,
        )
        with pytest.raises(RuntimeError, match="ROS 2"):
            _build_numeric(s, 99)


# ── force_log always prints ──────────────────────────────────────────────────

class TestForceLog:
    def test_force_log_always_prints(self, publisher):
        publisher._last_any_log = time.monotonic()  # saturate
        out = _captured_logs(publisher, publisher.force_log, "test message")
        assert "test message" in out


# ── Edge cases ───────────────────────────────────────────────────────────────

class TestEdgeCases:
    def test_update_full_wrist_state_no_cmd(self, publisher):
        publisher.update_full(wrist_state="rotating", wrist_cmd=None)
        with publisher._state.lock:
            assert publisher._state.wrist_state == "rotating"
            assert publisher._state.wrist_cmd == 0.0  # defaulted

    def test_force_log_with_empty_faults(self, publisher):
        publisher.add_fault(FaultCode.CMD_CONFLICT)
        publisher.clear_fault(FaultCode.CMD_CONFLICT)
        # Trigger routine log that includes fault summary
        publisher._last_routine_log = time.monotonic() - publisher.ROUTINE_INTERVAL - 1
        publisher._last_any_log = 0.0
        out = _captured_logs(publisher, publisher.throttled_log)
        assert "Faults: none" in out

    def test_clear_only_one_of_multiple_faults(self, publisher):
        publisher.add_fault(FaultCode.STALE_EMG)
        publisher.add_fault(FaultCode.OVERFORCE)
        publisher.clear_fault(FaultCode.STALE_EMG)
        with publisher._state.lock:
            assert publisher._state.faults == {FaultCode.OVERFORCE}

    def test_set_grasp_phase_fault(self, publisher):
        out = _captured_logs(publisher, publisher.set_grasp_phase, GraspPhase.FAULT)
        assert "fault" in out.lower()
