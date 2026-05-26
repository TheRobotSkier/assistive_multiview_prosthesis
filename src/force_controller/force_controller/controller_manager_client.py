"""Native ROS 2 service client for controller_manager interactions.

Replaces the fragile subprocess-based approach of calling ``ros2 service call``
via ``subprocess.run()``.  The native service client shares the node's existing
DDS participant, eliminating per-call discovery overhead (1-5 s per subprocess)
and timeout fragility under load.

Usage::

    from force_controller.controller_manager_client import ControllerManagerClient

    class MyNode(Node):
        def __init__(self):
            super().__init__("my_node")
            self._cm = ControllerManagerClient(self)

        def switch_to_velocity(self):
            self._cm.switch_controllers(
                activate=["group_vel_ff_controller"],
                deactivate=POSITION_CONTROLLERS,
            )
"""

from __future__ import annotations

import time
from typing import Optional

from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import (
    ListControllers,
    ListControllersRequest,
    ListControllersResponse,
    LoadController,
    LoadControllerRequest,
    SwitchController,
    SwitchControllerRequest,
)
from rclpy.node import Node


class ControllerManagerClient:
    """Manages ros2_control controller lifecycle via native ROS 2 service calls.

    All service calls are **synchronous** (blocking) because they are used
    during state transitions where the caller must know the outcome before
    proceeding.  The calls are fast (< 100 ms) because they reuse the node's
    existing DDS participant — no subprocess spawn or fresh discovery needed.
    """

    # Strictness constants matching controller_manager_msgs/SwitchController
    STRICT = SwitchControllerRequest.STRICT          # 2
    BEST_EFFORT = SwitchControllerRequest.BEST_EFFORT  # 1

    def __init__(self, node: Node, service_ns: str = "/controller_manager") -> None:
        self._node = node
        self._log = node.get_logger()
        self._ns = service_ns

        # Create service clients (lazy — discovery happens on first call)
        self._list_cli = node.create_client(ListControllers, f"{service_ns}/list_controllers")
        self._load_cli = node.create_client(LoadController, f"{service_ns}/load_controller")
        self._switch_cli = node.create_client(SwitchController, f"{service_ns}/switch_controller")

    # ── Service readiness ────────────────────────────────────────────────

    def wait_for_services(self, timeout_sec: float = 20.0) -> bool:
        """Block until all controller_manager services are available.

        Returns ``True`` if services became ready within *timeout_sec*,
        ``False`` otherwise.
        """
        deadline = time.monotonic() + timeout_sec
        clients = [self._list_cli, self._load_cli, self._switch_cli]
        for cli in clients:
            remaining = max(0.0, deadline - time.monotonic())
            if remaining <= 0:
                self._log.error("Timed out waiting for controller_manager services.")
                return False
            if not cli.wait_for_service(timeout_sec=remaining):
                self._log.error(
                    f"Service {cli.srv_name} not available after {timeout_sec:.1f}s."
                )
                return False
        self._log.debug("All controller_manager services are ready.")
        return True

    def services_ready(self) -> bool:
        """Return ``True`` if all services are currently discoverable."""
        return all(c.service_is_ready() for c in [self._list_cli, self._load_cli, self._switch_cli])

    # ── Controller introspection ─────────────────────────────────────────

    def list_controller_states(self) -> dict[str, str]:
        """Return a mapping of controller name -> state string.

        Example: ``{"group_pos_ff_controller": "active", ...}``
        """
        response = self._call_service(self._list_cli, ListControllersRequest())
        return {entry.name: entry.state for entry in response.controller}

    # ── Controller lifecycle ─────────────────────────────────────────────

    def ensure_controller_loaded(self, name: str) -> None:
        """Load a controller if it is not already known to the manager.

        Raises ``RuntimeError`` if loading fails.
        """
        states = self.list_controller_states()
        if name in states:
            return  # already loaded (any state)

        self._log.info(f"Loading controller '{name}'...")
        req = LoadControllerRequest()
        req.name = name
        response = self._call_service(self._load_cli, req)
        if not response.ok:
            raise RuntimeError(f"Failed to load controller '{name}'")

    def switch_controllers(
        self,
        activate: list[str],
        deactivate: list[str],
        timeout_sec: float = 10.0,
    ) -> None:
        """Activate and deactivate controllers with STRICT then BEST_EFFORT fallback.

        Skips controllers that are already in the desired state.  Loads any
        controller that is not yet known to the manager.

        Raises ``RuntimeError`` if the switch fails after both attempts.
        """
        # Filter to only those that actually need changing
        states = self.list_controller_states()
        to_activate = [c for c in activate if states.get(c) != "active"]
        to_deactivate = [c for c in deactivate if states.get(c) == "active"]

        if not to_activate and not to_deactivate:
            self._log.debug("All controllers already in desired state; nothing to switch.")
            return

        # Ensure controllers to be activated are loaded
        for ctrl in to_activate:
            self.ensure_controller_loaded(ctrl)

        # Try STRICT first (strictness=2)
        ok = self._do_switch(to_activate, to_deactivate, self.STRICT, timeout_sec)
        if ok:
            return

        # Fallback to BEST_EFFORT (strictness=1)
        self._log.warn("STRICT switch failed; retrying with BEST_EFFORT...")
        ok = self._do_switch(to_activate, to_deactivate, self.BEST_EFFORT, timeout_sec)
        if ok:
            return

        raise RuntimeError(
            f"Controller switch failed (activate={to_activate}, deactivate={to_deactivate})"
        )

    # ── Internal helpers ─────────────────────────────────────────────────

    def _do_switch(
        self,
        activate: list[str],
        deactivate: list[str],
        strictness: int,
        timeout_sec: float,
    ) -> bool:
        """Execute a single switch_controller service call."""
        req = SwitchControllerRequest()
        req.start_controllers = activate
        req.stop_controllers = deactivate
        req.strictness = strictness
        req.start_asap = True
        req.timeout = timeout_sec

        self._log.info(
            f"Switching controllers: start={activate}, stop={deactivate}, "
            f"strictness={strictness}, timeout={timeout_sec}s"
        )
        response = self._call_service(self._switch_cli, req)
        if not response.ok:
            self._log.warn(f"Switch returned ok=False (strictness={strictness})")
        return response.ok

    def _call_service(self, client, request, call_timeout_sec: float = 15.0):
        """Call a service synchronously with a timeout.

        Raises ``RuntimeError`` if the service is not ready or the call times
        out.
        """
        if not client.service_is_ready():
            # Try one more wait — service may be transiently unavailable
            if not client.wait_for_service(timeout_sec=5.0):
                raise RuntimeError(f"Service {client.srv_name} is not available")

        future = client.call_async(request)
        # Spin the node's executor until the future completes
        self._node.executor.spin_until_future_complete(
            future, timeout_sec=call_timeout_sec
        )
        if not future.done():
            raise TimeoutError(
                f"Service call to {client.srv_name} timed out after {call_timeout_sec}s"
            )
        result = future.result()
        if result is None:
            exc = future.exception()
            raise RuntimeError(
                f"Service call to {client.srv_name} failed: {exc}"
            )
        return result
