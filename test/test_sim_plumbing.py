"""Pure-logic checks for the simulation task plumbing.

No ROS runtime is required: the execute script's top-level ROS imports are
replaced by minimal stub modules so its argparse contract can be exercised
directly, and the launch file is inspected through the AST.  The acceptance
commands (``ros2 launch ... --show-args`` and a live Gazebo session) remain
separate runtime checks.
"""

from __future__ import annotations

import ast
import importlib.util
import sys
import types
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
CHUNKS_SCRIPT = ROOT / "scripts" / "execute_assembly_chunks.py"
WORLD_SIM_CONFIG = ROOT / "config" / "tasks" / "world_sim.yaml"
SIM_CONTROLLERS_CONFIG = ROOT / "src" / "cs625_bringup" / "config" / "sim_controllers.yaml"
SIM_CELL_LAUNCH = ROOT / "src" / "cs625_bringup" / "launch" / "sim_cell.launch.py"

REAL_ACTION = "/arm_controller/follow_joint_trajectory"
SIM_CONTROLLER = "joint_trajectory_controller"
SIM_ACTION = f"/{SIM_CONTROLLER}/follow_joint_trajectory"

STUBBED_MODULES = (
    "rclpy",
    "rclpy.action",
    "rclpy.node",
    "builtin_interfaces",
    "builtin_interfaces.msg",
    "control_msgs",
    "control_msgs.action",
    "control_msgs.msg",
    "trajectory_msgs",
    "trajectory_msgs.msg",
)


def _stub(name: str, **attributes):
    module = types.ModuleType(name)
    module.__path__ = []  # allow ``from package.module import name``
    for key, value in attributes.items():
        setattr(module, key, value)
    sys.modules[name] = module
    return module


@pytest.fixture()
def chunks():
    """Import execute_assembly_chunks.py with ROS modules stubbed out."""
    saved = {name: sys.modules.get(name) for name in STUBBED_MODULES}
    _stub("rclpy", init=lambda: None, ok=lambda: True, shutdown=lambda: None)
    _stub("rclpy.action", ActionClient=type("ActionClient", (), {}))
    _stub("rclpy.node", Node=type("Node", (), {}))
    _stub("builtin_interfaces.msg", Duration=type("Duration", (), {}))
    _stub(
        "control_msgs.action",
        FollowJointTrajectory=type("FollowJointTrajectory", (), {}),
    )
    _stub("control_msgs.msg", JointTolerance=type("JointTolerance", (), {}))
    _stub("trajectory_msgs.msg", JointTrajectoryPoint=type("JointTrajectoryPoint", (), {}))

    spec = importlib.util.spec_from_file_location(
        "execute_assembly_chunks_under_test", CHUNKS_SCRIPT
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    try:
        yield module
    finally:
        for name, previous in saved.items():
            if previous is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = previous


def _callee_name(node: ast.AST) -> str | None:
    if isinstance(node, ast.Name):
        return node.id
    if isinstance(node, ast.Attribute):
        return node.attr
    return None


def _declared_arguments(source: str) -> dict[str, dict[str, ast.AST]]:
    tree = ast.parse(source)
    declared: dict[str, dict[str, ast.AST]] = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or _callee_name(node.func) != "DeclareLaunchArgument":
            continue
        if not node.args or not isinstance(node.args[0], ast.Constant):
            continue
        declared[str(node.args[0].value)] = {
            keyword.arg: keyword.value for keyword in node.keywords if keyword.arg
        }
    return declared


def _const(node: ast.AST | None):
    return node.value if isinstance(node, ast.Constant) else None


def test_action_defaults_to_real_arm(chunks):
    args = chunks.parse_args(["chunk.json"])
    assert args.action == REAL_ACTION
    assert chunks.DEFAULT_ACTION == REAL_ACTION


def test_action_accepts_simulation_controller(chunks):
    args = chunks.parse_args(["chunk.json", "--action", SIM_ACTION])
    assert args.action == SIM_ACTION


def test_help_exposes_action_and_default(chunks):
    help_text = chunks.build_parser().format_help()
    assert "--action" in help_text
    assert REAL_ACTION in help_text


def test_action_client_uses_parsed_action():
    tree = ast.parse(CHUNKS_SCRIPT.read_text(encoding="utf-8"))
    matches = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or _callee_name(node.func) != "ActionClient":
            continue
        matches.append(node)
    assert matches, "execute_assembly_chunks.py no longer creates an ActionClient"
    for call in matches:
        assert len(call.args) >= 3
        action_arg = call.args[2]
        assert isinstance(action_arg, ast.Attribute), "ActionClient must take --action"
        assert action_arg.attr == "action"
        assert isinstance(action_arg.value, ast.Name) and action_arg.value.id == "args"


def test_world_sim_config_is_minimal_and_matches_controller():
    document = yaml.safe_load(WORLD_SIM_CONFIG.read_text(encoding="utf-8"))
    assert document == {"action": SIM_ACTION, "world": "sim"}

    controllers = yaml.safe_load(SIM_CONTROLLERS_CONFIG.read_text(encoding="utf-8"))
    parameters = controllers["controller_manager"]["ros__parameters"]
    assert SIM_CONTROLLER in parameters
    assert (
        parameters[SIM_CONTROLLER]["type"]
        == "joint_trajectory_controller/JointTrajectoryController"
    )


def test_sim_cell_launch_is_parsable_and_declares_cli_arguments():
    source = SIM_CELL_LAUNCH.read_text(encoding="utf-8")
    ast.parse(source)  # syntax check without importing launch

    declarations = _declared_arguments(source)
    for name in ("scope", "mode", "headless", "launch_rviz"):
        assert name in declarations, f"sim_cell.launch.py does not declare {name}"

    assert _const(declarations["scope"].get("default_value")) == "load"
    assert _const(declarations["mode"].get("default_value")) == "safe"
    assert _const(declarations["headless"].get("default_value")) == "false"
    assert _const(declarations["launch_rviz"].get("default_value")) == "true"

    # The new real-cell world must remain the default, and the composition must
    # reuse the existing base + MoveIt launches.
    assert "cs625_real_cell.sdf" in source
    assert "sim_base.launch.py" in source
    assert "sim_moveit.launch.py" in source

    # Only sim_moveit may own RViz; sim_cell must not add a second rviz2 node.
    packages = []
    for node in ast.walk(ast.parse(source)):
        if not isinstance(node, ast.Call) or _callee_name(node.func) != "Node":
            continue
        keywords = {keyword.arg: keyword.value for keyword in node.keywords if keyword.arg}
        package = _const(keywords.get("package"))
        if package:
            packages.append(package)
    assert "rviz2" not in packages
