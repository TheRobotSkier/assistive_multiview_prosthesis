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

import threading
import time
from typing import Optional

from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import (
    ConfigureController,
    ConfigureController_Request,
    ListControllers,
    ListControllers_Request,
    LoadController,
    LoadController_Request,
    SwitchController,
    SwitchController_Request,
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
    STRICT = SwitchController_Request.STRICT          # 2
    BEST_EFFORT = SwitchController_Request.BEST_EFFORT  # 1

    def __init__(
        self,
        node: Node,
        service_ns: str = "/controller_manager",
        *,
        use_private_executor: bool = False,
    ) -> None:
        self._owner_node = node
        self._node = node
        self._log = node.get_logger()
        self._ns = service_ns
        self._helper_executor = None
        self._helper_thread: Optional[threading.Thread] = None

        if use_private_executor:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor

            owner_name = getattr(node, "get_name", lambda: "node")()
            helper_name = f"{owner_name}_cm_client_{id(self) & 0xffff:x}"
            self._node = rclpy.create_node(
                helper_name,
                context=getattr(node, "context", None),
                use_global_arguments=False,
            )
            self._helper_executor = SingleThreadedExecutor()
            self._helper_executor.add_node(self._node)
            self._helper_thread = threading.Thread(
                target=self._helper_executor.spin,
                name=f"{helper_name}_executor",
                daemon=True,
            )
            self._helper_thread.start()

        # Create service clients (lazy — discovery happens on first call)
        self._list_cli = self._node.create_client(
            ListControllers, f"{service_ns}/list_controllers"
        )
        self._load_cli = self._node.create_client(
            LoadController, f"{service_ns}/load_controller"
        )
        self._switch_cli = self._node.create_client(
            SwitchController, f"{service_ns}/switch_controller"
        )
        self._configure_cli = self._node.create_client(
            ConfigureController, f"{service_ns}/configure_controller"
        )

    def shutdown(self) -> None:
        """Stop any private helper executor owned by this client."""
        if self._helper_executor is None:
            return
        self._helper_executor.shutdown()
        if self._helper_thread is not None:
            self._helper_thread.join(timeout=1.0)
        self._node.destroy_node()
        self._helper_executor = None
        self._helper_thread = None

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
        response = self._call_service(self._list_cli, ListControllers_Request())
        return {entry.name: entry.state for entry in response.controller}
    def ensure_controller_loaded(self, name: str) -> None:
        """Load and configure a controller if it is not already known.

        Jazzy's ``load_controller`` does NOT auto-configure — the controller
        is left in ``unconfigured`` state.  This method also calls
        ``configure_controller`` so the controller is ``inactive`` and
        ready for ``switch_controllers`` to activate.

        Raises ``RuntimeError`` if loading or configuration fails.
        """
        states = self.list_controller_states()
        if name in states and states[name] != "unconfigured":
            return  # already loaded and configured

        if name not in states:
            self._log.info(f"Loading controller '{name}'...")
            req = LoadController_Request()
            req.name = name
            response = self._call_service(self._load_cli, req)
            if not response.ok:
                raise RuntimeError(f"Failed to load controller '{name}'")

        # Always configure — load_controller in Jazzy does not auto-configure.
        self._log.info(f"Configuring controller '{name}'...")
        self.configure_controller(name)

    def configure_controller(self, name: str) -> None:
        """Explicitly configure a loaded controller.

        ``load_controller`` in Jazzy does NOT auto-configure — the controller
        is left in ``unconfigured`` state.  ``switch_controller`` only activates
        controllers that are already ``inactive`` (i.e. configured).  Call this
        after ``ensure_controller_loaded`` and before ``switch_controllers``.

        Raises ``RuntimeError`` if configuration fails.
        """
        req = ConfigureController_Request()
        req.name = name
        response = self._call_service(self._configure_cli, req)
        if not response.ok:
            raise RuntimeError(f"Failed to configure controller '{name}'")

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
        req = SwitchController_Request()
        req.activate_controllers = activate
        req.deactivate_controllers = deactivate
        req.strictness = strictness
        req.activate_asap = True
        sec = int(timeout_sec)
        req.timeout.sec = sec
        req.timeout.nanosec = int(round((timeout_sec - sec) * 1_000_000_000))

        self._log.info(
            f"Switching controllers: activate={activate}, deactivate={deactivate}, "
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
        executor = getattr(self._node, "executor", None)
        if self._helper_executor is not None:
            done = threading.Event()
            future.add_done_callback(lambda _: done.set())
            if future.done():
                done.set()
            done.wait(timeout=call_timeout_sec)
        elif executor is not None:
            # Spin the node's executor until the future completes
            executor.spin_until_future_complete(
                future, timeout_sec=call_timeout_sec
            )
        else:
            import rclpy

            rclpy.spin_until_future_complete(
                self._node, future, timeout_sec=call_timeout_sec
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
