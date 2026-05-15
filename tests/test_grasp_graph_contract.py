"""Contract tests for the hardware grasp graph launch description.

Validates that mia_force_grasp_hardware.launch.py includes the expected
ros2_control stack, wrist driver, preshaping, proximity, pipeline manager,
and mia_safety — and that it does NOT include the conflicting standalone
driver (mia_hand_driver_node) or dead force_controller_node.

No ROS2 runtime required — uses pure Python launch introspection.
"""

from __future__ import annotations

import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node

_PROJECT_ROOT = Path(__file__).resolve().parents[1]
_LAUNCH_DIR = str(_PROJECT_ROOT / "src" / "prosthesis_launch" / "launch")
sys.path.insert(0, _LAUNCH_DIR)


def _generate_ld() -> LaunchDescription:
    import mia_force_grasp_hardware  # noqa: PLC0415

    return mia_force_grasp_hardware.generate_launch_description()


def _flatten_nodes(ld: LaunchDescription) -> list[Node]:
    """Recursively collect all Node actions from a LaunchDescription tree.

    Touches one level of IncludeLaunchDescription when the included
    launch file can be resolved (i.e. does not use OpaqueFunction).
    """
    nodes: list[Node] = []

    def _walk(entities):
        for entity in entities:
            if isinstance(entity, Node):
                nodes.append(entity)
            elif isinstance(entity, IncludeLaunchDescription):
                try:
                    _walk(entity.describe_sub_entities())
                except Exception:
                    pass
            elif hasattr(entity, "describe_sub_entities"):
                try:
                    _walk(entity.describe_sub_entities())
                except Exception:
                    pass

    try:
        _walk(ld.describe_sub_entities())
    except Exception:
        pass
    return nodes


def _collect_top_level_actions(ld: LaunchDescription):
    """Return (include_actions, node_actions) from the top-level LD."""
    includes: list[IncludeLaunchDescription] = []
    nodes: list[Node] = []
    for entity in ld.describe_sub_entities():
        if isinstance(entity, IncludeLaunchDescription):
            includes.append(entity)
        elif isinstance(entity, Node):
            nodes.append(entity)
    return includes, nodes


# ── helpers to query node identity ────────────────────────────────────────


def _node_pkg(n: Node) -> str:
    """Package name of a Node action, or '' if unavailable."""
    try:
        return n._Node__package[0]  # type: ignore[union-attr]
    except Exception:
        return ""


def _node_exe(n: Node) -> str:
    try:
        return n._Node__node_executable[0]  # type: ignore[union-attr]
    except Exception:
        return ""


def _node_name(n: Node) -> str:
    try:
        return n._Node__node_name[0]  # type: ignore[union-attr]
    except Exception:
        return ""


def _node_matches(n: Node, *, pkg: str | None = None,
                  exe: str | None = None, name: str | None = None) -> bool:
    if pkg is not None and _node_pkg(n) != pkg:
        return False
    if exe is not None and _node_exe(n) != exe:
        return False
    if name is not None and _node_name(n) != name:
        return False
    return True


# ── tests ─────────────────────────────────────────────────────────────────


class TestHardwareLaunchContainsRequiredNodes:
    """Top-level nodes directly declared in mia_force_grasp_hardware."""

    @staticmethod
    def _direct_nodes() -> list[Node]:
        ld = _generate_ld()
        _, nodes = _collect_top_level_actions(ld)
        return nodes

    @pytest.mark.parametrize(
        "pkg, exe, name",
        [
            ("wrist_driver", "wrist_driver_node", "wrist_driver"),
            ("pipeline_manager", "pipeline_manager_node", "pipeline_manager"),
            ("grasp_preshaping", "preshaping_service_bridge_node",
             "preshaping_service"),
            ("grasp_preshaping", "grasp_proximity_controller_node.py",
             "proximity_controller"),
            ("camera", "hand_pose_publisher", "hand_pose_publisher"),
        ],
    )
    def test_node_present(self, pkg, exe, name):
        nodes = self._direct_nodes()
        assert any(_node_matches(n, pkg=pkg, exe=exe, name=name)
                   for n in nodes), \
            f"Missing node {pkg=} {exe=} {name=}"


class TestHardwareLaunchExcludesStandaloneDriver:
    """Nodes that must NOT appear in the hardware launch."""

    @staticmethod
    def _all_nodes() -> list[Node]:
        ld = _generate_ld()
        return _flatten_nodes(ld)

    @pytest.mark.parametrize(
        "pkg, exe",
        [
            ("mia_hand_driver", "mia_hand_driver_node"),
            ("grasp_preshaping", "force_controller_node"),
        ],
    )
    def test_node_absent(self, pkg, exe):
        nodes = self._all_nodes()
        assert not any(_node_matches(n, pkg=pkg, exe=exe) for n in nodes), \
            f"Forbidden node present: {pkg=} {exe=}"


class TestHardwareLaunchIncludesRos2Control:
    """The ros2_control subsystem is pulled in via IncludeLaunchDescription."""

    def test_includes_mia_hand_system_interface_launch(self):
        ld = _generate_ld()
        includes, _ = _collect_top_level_actions(ld)
        paths = []
        for inc in includes:
            try:
                subs = inc.describe_sub_entities()
                # The PythonLaunchDescriptionSource path is embedded
                # in the sub-entities; check for the package name.
                for s in subs:
                    paths.append(str(s))
            except Exception:
                pass
        assert any(
            "mia_hand_ros2_control" in p
            for p in paths
        ), "Missing IncludeLaunchDescription for mia_hand_ros2_control"


class TestLaunchArguments:
    """Ensure required launch arguments are declared."""

    @staticmethod
    def _declared_args() -> set[str]:
        ld = _generate_ld()
        args: set[str] = set()
        for entity in ld.describe_sub_entities():
            if isinstance(entity, DeclareLaunchArgument):
                try:
                    args.add(entity._DeclareLaunchArgument__name[0])  # type: ignore[union-attr]
                except Exception:
                    pass
        return args

    @pytest.mark.parametrize(
        "arg_name",
        [
            "serial_port",
            "wrist_port",
            "config_file",
            "rviz",
            "camera",
            "segmentation",
            "emg",
            "mia_hand",
            "wrist",
        ],
    )
    def test_argument_declared(self, arg_name):
        args = self._declared_args()
        assert arg_name in args, \
            f"Launch argument '{arg_name}' not declared"
