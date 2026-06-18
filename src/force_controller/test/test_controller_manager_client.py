#!/usr/bin/env python3
"""Unit tests for ControllerManagerClient.

These tests verify the request construction, strictness fallback logic,
and error handling of the ControllerManagerClient without requiring a
running ROS 2 system.  Service clients are mocked to return controlled
responses.
"""

import time
import unittest
from unittest.mock import MagicMock, PropertyMock, patch, call

# We need to mock rclpy and controller_manager_msgs imports before importing
# the module under test, since those packages may not be available outside
# the ROS 2 environment.
import sys
import importlib

# Create mock modules for ROS 2 dependencies
mock_rclpy = MagicMock()
mock_rclpy_node = MagicMock()
mock_rclpy_executors = MagicMock()
mock_controller_manager_msgs = MagicMock()
mock_controller_manager_msgs_srv = MagicMock()
mock_controller_manager_msgs_msg = MagicMock()

# Setup mock service types with realistic behavior
mock_list_req = MagicMock()
mock_list_resp = MagicMock()
mock_load_req = MagicMock()
mock_load_resp = MagicMock()
mock_switch_req = MagicMock()
mock_switch_resp = MagicMock()

# SwitchController request constants
mock_switch_req.STRICT = 2
mock_switch_req.BEST_EFFORT = 1

mock_controller_manager_msgs_srv.ListControllers = MagicMock()
mock_controller_manager_msgs_srv.ListControllers_Request = mock_list_req
mock_controller_manager_msgs_srv.ListControllers_Response = mock_list_resp
mock_controller_manager_msgs_srv.LoadController = MagicMock()
mock_controller_manager_msgs_srv.LoadController_Request = mock_load_req
mock_controller_manager_msgs_srv.LoadController_Response = mock_load_resp
mock_controller_manager_msgs_srv.SwitchController = MagicMock()
mock_controller_manager_msgs_srv.SwitchController_Request = mock_switch_req
mock_controller_manager_msgs_srv.SwitchController_Response = mock_switch_resp

mock_controller_manager_msgs.srv = mock_controller_manager_msgs_srv

# ControllerState mock
mock_cs = MagicMock()
mock_controller_manager_msgs.msg = MagicMock(ControllerState=mock_cs)

# Register all mock modules BEFORE any import — this must cover every
# ROS 2 sub-module that any file in the force_controller package might
# import, because Python's import system will execute __init__.py which
# imports ForceControllerNode which has many ROS 2 dependencies.
sys.modules['rclpy'] = mock_rclpy
sys.modules['rclpy.node'] = mock_rclpy_node
sys.modules['rclpy.executors'] = mock_rclpy_executors
sys.modules['rclpy.qos'] = MagicMock()
sys.modules['controller_manager_msgs'] = mock_controller_manager_msgs
sys.modules['controller_manager_msgs.srv'] = mock_controller_manager_msgs_srv
sys.modules['controller_manager_msgs.msg'] = mock_controller_manager_msgs_msg
sys.modules['mia_hand_msgs'] = MagicMock()
sys.modules['mia_hand_msgs.msg'] = MagicMock()
sys.modules['sensor_msgs'] = MagicMock()
sys.modules['sensor_msgs.msg'] = MagicMock()
sys.modules['std_msgs'] = MagicMock()
sys.modules['std_msgs.msg'] = MagicMock()
sys.modules['std_srvs'] = MagicMock()
sys.modules['std_srvs.srv'] = MagicMock()

# Import the module directly (not through the package __init__.py)
# to avoid pulling in ForceControllerNode's full dependency chain.
_module = importlib.import_module('force_controller.controller_manager_client')
ControllerManagerClient = _module.ControllerManagerClient


def _make_controller_state(name: str, state: str) -> MagicMock:
    """Create a mock ControllerState with the given name and state."""
    entry = MagicMock()
    entry.name = name
    entry.state = state
    return entry


def _make_list_response(controllers: list[tuple[str, str]]) -> MagicMock:
    """Create a mock ListControllers_Response with given (name, state) pairs."""
    resp = MagicMock()
    resp.controller = [_make_controller_state(n, s) for n, s in controllers]
    return resp


def _make_load_response(ok: bool) -> MagicMock:
    resp = MagicMock()
    resp.ok = ok
    return resp


def _make_switch_response(ok: bool) -> MagicMock:
    resp = MagicMock()
    resp.ok = ok
    return resp


class TestControllerManagerClient(unittest.TestCase):
    """Tests for the ControllerManagerClient class."""

    def setUp(self):
        mock_rclpy.create_node.reset_mock(return_value=True, side_effect=True)
        mock_rclpy.spin_until_future_complete.reset_mock()
        mock_rclpy_executors.SingleThreadedExecutor.reset_mock(
            return_value=True, side_effect=True
        )

        self.mock_node = MagicMock()
        self.mock_logger = MagicMock()
        self.mock_node.get_logger.return_value = self.mock_logger
        self.mock_executor = MagicMock()
        self.mock_node.executor = self.mock_executor
        self.mock_node.context = object()

        # Mock service clients
        self.mock_list_cli = MagicMock()
        self.mock_load_cli = MagicMock()
        self.mock_switch_cli = MagicMock()
        self.mock_configure_cli = MagicMock()

        self.mock_node.create_client.side_effect = [
            self.mock_list_cli,
            self.mock_load_cli,
            self.mock_switch_cli,
            self.mock_configure_cli,
        ]

        self.client = ControllerManagerClient(self.mock_node)

    def test_private_executor_uses_helper_node(self):
        """Private executor mode keeps service clients off the owner node."""
        owner = MagicMock()
        owner.get_logger.return_value = self.mock_logger
        owner.get_name.return_value = "owner_node"
        owner.context = object()

        helper_node = MagicMock()
        helper_list_cli = MagicMock()
        helper_load_cli = MagicMock()
        helper_switch_cli = MagicMock()
        helper_configure_cli = MagicMock()
        helper_node.create_client.side_effect = [
            helper_list_cli,
            helper_load_cli,
            helper_switch_cli,
            helper_configure_cli,
        ]
        mock_rclpy.create_node.return_value = helper_node

        helper_executor = MagicMock()
        mock_rclpy_executors.SingleThreadedExecutor.return_value = helper_executor

        client = ControllerManagerClient(owner, use_private_executor=True)

        args, kwargs = mock_rclpy.create_node.call_args
        self.assertRegex(args[0], r"^owner_node_cm_client_[0-9a-f]+$")
        self.assertIs(kwargs["context"], owner.context)
        self.assertFalse(kwargs["use_global_arguments"])
        self.assertEqual(helper_node.create_client.call_count, 4)
        owner.create_client.assert_not_called()

        client.shutdown()

        helper_executor.shutdown.assert_called_once()
        helper_node.destroy_node.assert_called_once()

    def _setup_service_call(self, client_mock, response):
        """Configure a mock service client to return the given response."""
        future = MagicMock()
        future.done.return_value = True
        future.result.return_value = response
        future.exception.return_value = None
        client_mock.call_async.return_value = future
        client_mock.service_is_ready.return_value = True
        return future

    # ── wait_for_services ────────────────────────────────────────────────

    def test_wait_for_services_success(self):
        """All services become ready within timeout."""
        self.mock_list_cli.wait_for_service.return_value = True
        self.mock_load_cli.wait_for_service.return_value = True
        self.mock_switch_cli.wait_for_service.return_value = True

        result = self.client.wait_for_services(timeout_sec=5.0)
        self.assertTrue(result)

    def test_wait_for_services_timeout(self):
        """One service never becomes ready."""
        self.mock_list_cli.wait_for_service.return_value = False

        result = self.client.wait_for_services(timeout_sec=1.0)
        self.assertFalse(result)

    # ── list_controller_states ───────────────────────────────────────────

    def test_list_controller_states(self):
        """Returns correct name->state mapping from ListControllers response."""
        resp = _make_list_response([
            ("group_pos_ff_controller", "active"),
            ("group_vel_ff_controller", "inactive"),
        ])
        self._setup_service_call(self.mock_list_cli, resp)

        states = self.client.list_controller_states()
        self.assertEqual(states, {
            "group_pos_ff_controller": "active",
            "group_vel_ff_controller": "inactive",
        })

    def test_list_controller_states_empty(self):
        """Returns empty dict when no controllers are loaded."""
        resp = _make_list_response([])
        self._setup_service_call(self.mock_list_cli, resp)

        states = self.client.list_controller_states()
        self.assertEqual(states, {})

    # ── ensure_controller_loaded ─────────────────────────────────────────

    def test_ensure_loaded_already_present(self):
        """Does not call LoadController if controller already known."""
        resp = _make_list_response([("my_ctrl", "inactive")])
        self._setup_service_call(self.mock_list_cli, resp)

        self.client.ensure_controller_loaded("my_ctrl")

        # LoadController should NOT have been called
        self.mock_load_cli.call_async.assert_not_called()

    def test_ensure_loaded_needs_loading(self):
        """Calls LoadController when controller is not in the list."""
        list_resp = _make_list_response([])
        load_resp = _make_load_response(True)

        # First call: list_controllers returns empty
        # Second call: load_controller returns ok
        self._setup_service_call(self.mock_list_cli, list_resp)
        self._setup_service_call(self.mock_load_cli, load_resp)

        self.client.ensure_controller_loaded("new_ctrl")

        self.mock_load_cli.call_async.assert_called_once()

    def test_ensure_loaded_fails(self):
        """Raises RuntimeError when LoadController returns ok=False."""
        list_resp = _make_list_response([])
        load_resp = _make_load_response(False)

        self._setup_service_call(self.mock_list_cli, list_resp)
        self._setup_service_call(self.mock_load_cli, load_resp)

        with self.assertRaises(RuntimeError):
            self.client.ensure_controller_loaded("bad_ctrl")

    # ── switch_controllers ───────────────────────────────────────────────

    def test_switch_noop_when_all_in_desired_state(self):
        """Does nothing when all controllers are already in desired state."""
        resp = _make_list_response([
            ("group_vel_ff_controller", "active"),
            ("group_pos_ff_controller", "inactive"),
        ])
        self._setup_service_call(self.mock_list_cli, resp)

        # activate=vel (already active), deactivate=pos (already inactive)
        self.client.switch_controllers(
            activate=["group_vel_ff_controller"],
            deactivate=["group_pos_ff_controller"],
        )

        # SwitchController should NOT have been called
        self.mock_switch_cli.call_async.assert_not_called()

    def test_switch_strict_succeeds(self):
        """Switch succeeds on first try with STRICT strictness."""
        list_resp = _make_list_response([
            ("group_pos_ff_controller", "active"),
            ("group_vel_ff_controller", "inactive"),
        ])
        switch_resp = _make_switch_response(True)

        self._setup_service_call(self.mock_list_cli, list_resp)
        self._setup_service_call(self.mock_switch_cli, switch_resp)

        self.client.switch_controllers(
            activate=["group_vel_ff_controller"],
            deactivate=["group_pos_ff_controller"],
        )

        # Should have called switch once (STRICT succeeded)
        self.assertEqual(self.mock_switch_cli.call_async.call_count, 1)

        # Verify strictness was STRICT (2)
        call_args = self.mock_switch_cli.call_async.call_args
        req = call_args[0][0]
        self.assertEqual(req.strictness, 2)

    def test_switch_fallback_to_best_effort(self):
        """Falls back to BEST_EFFORT when STRICT fails."""
        list_resp = _make_list_response([
            ("group_pos_ff_controller", "active"),
        ])
        strict_resp = _make_switch_response(False)
        best_effort_resp = _make_switch_response(True)

        self._setup_service_call(self.mock_list_cli, list_resp)
        # First switch call (STRICT) fails, second (BEST_EFFORT) succeeds
        self.mock_switch_cli.call_async.side_effect = [
            self._setup_service_call(self.mock_switch_cli, strict_resp) or strict_resp,
            self._setup_service_call(self.mock_switch_cli, best_effort_resp) or best_effort_resp,
        ]
        # Re-configure the futures for each call
        future1 = MagicMock()
        future1.done.return_value = True
        future1.result.return_value = strict_resp
        future1.exception.return_value = None

        future2 = MagicMock()
        future2.done.return_value = True
        future2.result.return_value = best_effort_resp
        future2.exception.return_value = None

        self.mock_switch_cli.call_async.side_effect = [future1, future2]

        self.client.switch_controllers(
            activate=["group_vel_ff_controller"],
            deactivate=["group_pos_ff_controller"],
        )

        # Should have called switch twice (STRICT failed, BEST_EFFORT succeeded)
        self.assertEqual(self.mock_switch_cli.call_async.call_count, 2)

    def test_switch_both_fail_raises(self):
        """Raises RuntimeError when both STRICT and BEST_EFFORT fail."""
        list_resp = _make_list_response([
            ("group_pos_ff_controller", "active"),
        ])
        fail_resp = _make_switch_response(False)

        self._setup_service_call(self.mock_list_cli, list_resp)

        future1 = MagicMock()
        future1.done.return_value = True
        future1.result.return_value = fail_resp
        future1.exception.return_value = None

        future2 = MagicMock()
        future2.done.return_value = True
        future2.result.return_value = fail_resp
        future2.exception.return_value = None

        self.mock_switch_cli.call_async.side_effect = [future1, future2]

        with self.assertRaises(RuntimeError):
            self.client.switch_controllers(
                activate=["group_vel_ff_controller"],
                deactivate=["group_pos_ff_controller"],
            )

    def test_switch_loads_unloaded_controller(self):
        """Loads a controller before activating it if not yet known."""
        load_resp = _make_load_response(True)
        switch_resp = _make_switch_response(True)

        # switch_controllers flow:
        #   1. list_controller_states() -> empty (nothing loaded)
        #   2. ensure_controller_loaded() -> list_controller_states() -> empty
        #   3. ensure_controller_loaded() -> load_controller() -> ok
        #   4. _do_switch() -> switch_controller() -> ok
        empty_resp = _make_list_response([])
        future_list1 = MagicMock()
        future_list1.done.return_value = True
        future_list1.result.return_value = empty_resp
        future_list1.exception.return_value = None

        future_list2 = MagicMock()
        future_list2.done.return_value = True
        future_list2.result.return_value = empty_resp
        future_list2.exception.return_value = None

        future_load = MagicMock()
        future_load.done.return_value = True
        future_load.result.return_value = load_resp
        future_load.exception.return_value = None

        future_switch = MagicMock()
        future_switch.done.return_value = True
        future_switch.result.return_value = switch_resp
        future_switch.exception.return_value = None

        # Two list calls (switch_controllers + ensure_loaded), one load, one switch
        self.mock_list_cli.call_async.side_effect = [future_list1, future_list2]
        self.mock_load_cli.call_async.side_effect = [future_load]
        self.mock_switch_cli.call_async.side_effect = [future_switch]

        self.client.switch_controllers(
            activate=["group_vel_ff_controller"],
            deactivate=[],
        )

        # LoadController should have been called
        self.mock_load_cli.call_async.assert_called()

    # ── switch request field mapping ─────────────────────────────────────

    def test_switch_request_fields(self):
        """Verify the SwitchController request has correct field values."""
        list_resp = _make_list_response([
            ("group_pos_ff_controller", "active"),
        ])
        switch_resp = _make_switch_response(True)

        self._setup_service_call(self.mock_list_cli, list_resp)

        future = MagicMock()
        future.done.return_value = True
        future.result.return_value = switch_resp
        future.exception.return_value = None
        self.mock_switch_cli.call_async.return_value = future

        self.client.switch_controllers(
            activate=["group_vel_ff_controller"],
            deactivate=["group_pos_ff_controller"],
            timeout_sec=5.0,
        )

        # Inspect the request that was passed to call_async
        req = self.mock_switch_cli.call_async.call_args[0][0]
        self.assertEqual(req.activate_controllers, ["group_vel_ff_controller"])
        self.assertEqual(req.deactivate_controllers, ["group_pos_ff_controller"])
        self.assertEqual(req.strictness, 2)  # STRICT
        self.assertTrue(req.activate_asap)
        self.assertEqual(req.timeout.sec, 5)
        self.assertEqual(req.timeout.nanosec, 0)


    # ── services_ready ───────────────────────────────────────────────────

    def test_services_ready_all_true(self):
        """Returns True when all services are ready."""
        self.mock_list_cli.service_is_ready.return_value = True
        self.mock_load_cli.service_is_ready.return_value = True
        self.mock_switch_cli.service_is_ready.return_value = True

        self.assertTrue(self.client.services_ready())

    def test_services_ready_one_false(self):
        """Returns False when one service is not ready."""
        self.mock_list_cli.service_is_ready.return_value = True
        self.mock_load_cli.service_is_ready.return_value = False
        self.mock_switch_cli.service_is_ready.return_value = True

        self.assertFalse(self.client.services_ready())

    # ── service call failure handling ─────────────────────────────────────

    def test_service_call_timeout(self):
        """Raises TimeoutError when service call doesn't complete in time."""
        list_resp = _make_list_response([("ctrl", "active")])

        future = MagicMock()
        future.done.return_value = False  # Never completes
        self.mock_list_cli.call_async.return_value = future
        self.mock_list_cli.service_is_ready.return_value = True

        with self.assertRaises(TimeoutError):
            self.client.list_controller_states()

    def test_service_call_exception(self):
        """Raises RuntimeError when the service call itself fails."""
        future = MagicMock()
        future.done.return_value = True
        future.result.return_value = None
        future.exception.return_value = Exception("DDS connection lost")
        self.mock_list_cli.call_async.return_value = future
        self.mock_list_cli.service_is_ready.return_value = True

        with self.assertRaises(RuntimeError) as ctx:
            self.client.list_controller_states()

        self.assertIn("DDS connection lost", str(ctx.exception))

    def test_service_call_with_private_executor_uses_event_wait(self):
        """With private executor, uses threading.Event instead of spin_until_future_complete."""
        resp = _make_list_response([("group_pos_ff_controller", "active")])

        future = MagicMock()
        future.done.return_value = True
        future.result.return_value = resp
        future.exception.return_value = None
        future.add_done_callback.side_effect = lambda cb: cb(future)

        helper_list_cli = MagicMock()
        helper_list_cli.call_async.return_value = future
        helper_list_cli.service_is_ready.return_value = True
        mock_rclpy.create_node.return_value.create_client.return_value = helper_list_cli

        client = ControllerManagerClient(self.mock_node, use_private_executor=True)

        states = client.list_controller_states()

        self.assertEqual(states, {"group_pos_ff_controller": "active"})
        future.add_done_callback.assert_called_once()
        self.mock_executor.spin_until_future_complete.assert_not_called()

        client.shutdown()

    def test_service_call_with_no_attached_executor_uses_rclpy_spin(self):
        """Falls back to rclpy.spin_until_future_complete without an executor."""
        self.mock_node.executor = None
        resp = _make_list_response([("group_pos_ff_controller", "active")])

        future = MagicMock()
        future.done.return_value = True
        future.result.return_value = resp
        future.exception.return_value = None
        self.mock_list_cli.call_async.return_value = future
        self.mock_list_cli.service_is_ready.return_value = True

        states = self.client.list_controller_states()

        self.assertEqual(states, {"group_pos_ff_controller": "active"})
        mock_rclpy.spin_until_future_complete.assert_called_once_with(
            self.mock_node, future, timeout_sec=15.0
        )


if __name__ == "__main__":
    unittest.main()
