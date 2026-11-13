"""Guarded P7 arm-motion adapter with phase-level MoveIt evidence.

The adapter intentionally has no grasp logic.  It receives one explicitly
named phase (view, pregrasp, approach, or lift), asks MoveIt for one authoritative
collision-checked plan, executes exactly that returned trajectory through the
standard trajectory action, and emits a JSON receipt for the P7 orchestrator.
"""

from __future__ import annotations

import json
import math
import time
from typing import Any

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)
from moveit_msgs.srv import GetCartesianPath, GetMotionPlan, GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


VALID_PHASES = {"view", "pregrasp", "approach", "lift"}
EXPLICIT_MOVEIT_COLLISION_CODES = {
    MoveItErrorCodes.START_STATE_IN_COLLISION,
    MoveItErrorCodes.GOAL_IN_COLLISION,
}


def decode_motion_command(text: str) -> dict[str, Any]:
    """Validate the transport schema without accepting an implicit pose."""
    try:
        command = json.loads(text)
    except json.JSONDecodeError as error:
        raise ValueError("P7_ARM_COMMAND_INVALID_JSON") from error
    if not isinstance(command, dict):
        raise ValueError("P7_ARM_COMMAND_INVALID")
    command_id = str(command.get("command_id", "")).strip()
    phase = str(command.get("phase", "")).strip()
    pose = command.get("pose")
    if not command_id or phase not in VALID_PHASES or not isinstance(pose, dict):
        raise ValueError("P7_ARM_COMMAND_INVALID")
    frame_id = str(pose.get("frame_id", "")).strip()
    position = pose.get("position")
    orientation = pose.get("orientation")
    if not frame_id or not isinstance(position, dict) or not isinstance(orientation, dict):
        raise ValueError("P7_ARM_COMMAND_INVALID")
    try:
        parsed = {
            "x": float(position["x"]), "y": float(position["y"]), "z": float(position["z"]),
            "qx": float(orientation["x"]), "qy": float(orientation["y"]),
            "qz": float(orientation["z"]), "qw": float(orientation["w"]),
        }
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("P7_ARM_COMMAND_INVALID") from error
    if any(not math.isfinite(value) for value in parsed.values()):
        raise ValueError("P7_ARM_COMMAND_INVALID")
    quaternion_norm = math.sqrt(
        sum(value * value for key, value in parsed.items() if key.startswith("q"))
    )
    if quaternion_norm < math.sqrt(0.5):
        raise ValueError("P7_ARM_COMMAND_INVALID_ORIENTATION")
    # ROS callers often serialize a displayed quaternion with limited decimal
    # precision.  Normalize at the transport boundary so the subsequent FK
    # angular residual measures a frame mismatch, not serialization loss.
    for key in ("qx", "qy", "qz", "qw"):
        parsed[key] /= quaternion_norm
    result = {"command_id": command_id, "phase": phase, "frame_id": frame_id, **parsed}
    physical_tool_frame = str(command.get("physical_tool_frame", "")).strip()
    if physical_tool_frame:
        if phase != "view":
            raise ValueError("P7_ARM_COMMAND_INVALID")
        result["physical_tool_frame"] = physical_tool_frame
    raw_joint_goal = command.get("joint_goal_positions_rad")
    if raw_joint_goal is not None:
        if phase not in ("view", "pregrasp") or not isinstance(raw_joint_goal, dict) or not raw_joint_goal:
            raise ValueError("P7_ARM_COMMAND_INVALID_JOINT_GOAL")
        try:
            joint_goal = {
                str(name): float(value) for name, value in raw_joint_goal.items()
            }
        except (TypeError, ValueError) as error:
            raise ValueError("P7_ARM_COMMAND_INVALID_JOINT_GOAL") from error
        if any(not name or not math.isfinite(value) for name, value in joint_goal.items()):
            raise ValueError("P7_ARM_COMMAND_INVALID_JOINT_GOAL")
        result["joint_goal_positions_rad"] = joint_goal
    return result


def motion_status_payload(
    command_id: str,
    phase: str,
    success: bool,
    code: str,
    durations: dict[str, float],
    details: dict[str, Any] | None = None,
) -> dict[str, Any]:
    payload = {
        "command_id": command_id,
        "phase": phase,
        "success": success,
        "code": code,
        "ik_time_sec": round(max(0.0, durations.get("ik", 0.0)), 6),
        "planning_time_sec": round(
            max(0.0, durations.get("plan", 0.0) + durations.get("cartesian", 0.0)), 6
        ),
        "execution_time_sec": round(max(0.0, durations.get("execute", 0.0)), 6),
        "total_time_sec": round(max(0.0, sum(durations.values())), 6),
    }
    if details:
        payload.update(details)
    return payload


def record_phase_duration_once(
    durations: dict[str, float], phase: str, elapsed_sec: float
) -> float:
    """Record one phase exactly once and return the authoritative duration.

    ``_advance`` records a completed future before inspecting its result.  A
    terminal result then calls ``_finish`` in the same timer callback.  Using
    an additive update there counted the same interval twice; ``setdefault``
    preserves the completed measurement while still recording timeouts and
    other terminal paths whose future never completed.
    """

    return durations.setdefault(phase, max(0.0, elapsed_sec))


def moveit_collision_rejection(error_code: int | None) -> bool | str:
    """Return ``True`` only for an explicit MoveIt collision rejection.

    A generic planning failure does not prove collision, while a successful
    plan is not a Gazebo physical-contact measurement.  Both therefore remain
    ``unknown`` in this phase receipt.
    """

    if error_code in EXPLICIT_MOVEIT_COLLISION_CODES:
        return True
    return "unknown"


def joint_trajectory_path_length_rad(
    joint_names: list[str],
    trajectory_points: list[Any],
    start_joint_positions: dict[str, float],
) -> float:
    """Return cumulative Euclidean path length in joint configuration space."""

    if not joint_names:
        return 0.0
    try:
        previous = [float(start_joint_positions[name]) for name in joint_names]
    except KeyError as error:
        raise ValueError(f"missing start position for trajectory joint {error.args[0]}") from error
    path_length = 0.0
    for point in trajectory_points:
        current = [float(value) for value in point.positions]
        if len(current) != len(joint_names):
            raise ValueError("trajectory point dimension does not match joint_names")
        path_length += math.sqrt(
            sum((value - reference) ** 2 for value, reference in zip(current, previous))
        )
        previous = current
    return path_length


def _quaternion_multiply(left: tuple[float, float, float, float], right: tuple[float, float, float, float]):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lz * rw + lx * ry - ly * rx,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _rotate(vector: tuple[float, float, float], quaternion: tuple[float, float, float, float]):
    inverse = (-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3])
    rotated = _quaternion_multiply(_quaternion_multiply(quaternion, (*vector, 0.0)), inverse)
    return rotated[0], rotated[1], rotated[2]


def compose_pose(base_pose: Pose, relative_pose: Pose) -> Pose:
    """Return ``T_base_child = T_base_relative T_relative_child``."""
    base_q = (base_pose.orientation.x, base_pose.orientation.y, base_pose.orientation.z, base_pose.orientation.w)
    relative_q = (relative_pose.orientation.x, relative_pose.orientation.y, relative_pose.orientation.z, relative_pose.orientation.w)
    translated = _rotate((relative_pose.position.x, relative_pose.position.y, relative_pose.position.z), base_q)
    result = Pose()
    result.position.x = base_pose.position.x + translated[0]
    result.position.y = base_pose.position.y + translated[1]
    result.position.z = base_pose.position.z + translated[2]
    result.orientation.x, result.orientation.y, result.orientation.z, result.orientation.w = _quaternion_multiply(base_q, relative_q)
    return result


def transform_as_pose(transform) -> Pose:
    result = Pose()
    result.position.x = transform.translation.x
    result.position.y = transform.translation.y
    result.position.z = transform.translation.z
    result.orientation = transform.rotation
    return result


def pose_errors(expected: Pose, observed: Pose) -> tuple[float, float]:
    """Return translation metres and sign-invariant quaternion angular error."""
    position = math.sqrt(
        (expected.position.x - observed.position.x) ** 2
        + (expected.position.y - observed.position.y) ** 2
        + (expected.position.z - observed.position.z) ** 2
    )
    dot = abs(
        expected.orientation.x * observed.orientation.x
        + expected.orientation.y * observed.orientation.y
        + expected.orientation.z * observed.orientation.z
        + expected.orientation.w * observed.orientation.w
    )
    return position, 2.0 * math.acos(max(-1.0, min(1.0, dot)))


def nearest_wrapped_joint_position(value: float, reference: float) -> float:
    """Choose the +/- 2pi equivalent closest to the monitored joint state."""
    return value + round((reference - value) / (2.0 * math.pi)) * (2.0 * math.pi)


def pose_motion_plan_request(
    *,
    group_name: str,
    tip_pose: PoseStamped,
    planning_tip: str,
    joint_names: list[str],
    current_joint_positions: list[float],
    planning_attempts: int,
    allowed_planning_time: float,
    velocity_scale: float,
    acceleration_scale: float,
    position_tolerance_m: float,
    orientation_tolerance_rad: float,
) -> GetMotionPlan.Request:
    """Build one pose-goal request whose returned trajectory is authoritative.

    Keeping the current state and pose goal in the same MoveIt request lets the
    planner sample collision-free IK branches.  A separate IK request followed
    by a frozen joint goal can select a branch that is reachable in isolation
    but disconnected from the current state by a collision-free path.
    """

    if len(joint_names) != len(current_joint_positions) or not joint_names:
        raise ValueError("joint_names and current_joint_positions must have equal non-zero length")
    for name, value in (
        ("allowed_planning_time", allowed_planning_time),
        ("position_tolerance_m", position_tolerance_m),
        ("orientation_tolerance_rad", orientation_tolerance_rad),
    ):
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")

    request = GetMotionPlan.Request()
    motion_request = request.motion_plan_request
    motion_request.group_name = group_name
    motion_request.start_state.is_diff = True
    motion_request.start_state.joint_state.name = list(joint_names)
    motion_request.start_state.joint_state.position = list(current_joint_positions)
    motion_request.num_planning_attempts = planning_attempts
    motion_request.allowed_planning_time = allowed_planning_time
    motion_request.max_velocity_scaling_factor = velocity_scale
    motion_request.max_acceleration_scaling_factor = acceleration_scale

    position = PositionConstraint()
    position.header = tip_pose.header
    position.link_name = planning_tip
    tolerance_box = SolidPrimitive()
    tolerance_box.type = SolidPrimitive.BOX
    tolerance_box.dimensions = [2.0 * position_tolerance_m] * 3
    tolerance_pose = Pose()
    tolerance_pose.position = tip_pose.pose.position
    tolerance_pose.orientation.w = 1.0
    position.constraint_region.primitives = [tolerance_box]
    position.constraint_region.primitive_poses = [tolerance_pose]
    position.weight = 1.0

    orientation = OrientationConstraint()
    orientation.header = tip_pose.header
    orientation.link_name = planning_tip
    orientation.orientation = tip_pose.pose.orientation
    orientation.absolute_x_axis_tolerance = orientation_tolerance_rad
    orientation.absolute_y_axis_tolerance = orientation_tolerance_rad
    orientation.absolute_z_axis_tolerance = orientation_tolerance_rad
    orientation.weight = 1.0

    goal = Constraints()
    goal.position_constraints = [position]
    goal.orientation_constraints = [orientation]
    motion_request.goal_constraints = [goal]
    return request


def joint_motion_plan_request(
    *,
    group_name: str,
    joint_names: list[str],
    current_joint_positions: list[float],
    goal_joint_positions: list[float],
    planning_attempts: int,
    allowed_planning_time: float,
    velocity_scale: float,
    acceleration_scale: float,
    joint_tolerance_rad: float,
) -> GetMotionPlan.Request:
    """Build one collision-checked plan to a frozen candidate joint state."""

    if (
        not joint_names
        or len(joint_names) != len(current_joint_positions)
        or len(joint_names) != len(goal_joint_positions)
    ):
        raise ValueError("joint goal, start state and joint_names must have equal non-zero length")
    if not 1 <= planning_attempts <= 20:
        raise ValueError("planning_attempts must be within [1, 20]")
    for name, value in (
        ("allowed_planning_time", allowed_planning_time),
        ("joint_tolerance_rad", joint_tolerance_rad),
    ):
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
    if any(
        not math.isfinite(value)
        for value in current_joint_positions + goal_joint_positions
    ):
        raise ValueError("joint positions must be finite")

    request = GetMotionPlan.Request()
    motion_request = request.motion_plan_request
    motion_request.group_name = group_name
    motion_request.start_state.is_diff = True
    motion_request.start_state.joint_state.name = list(joint_names)
    motion_request.start_state.joint_state.position = list(current_joint_positions)
    motion_request.num_planning_attempts = planning_attempts
    motion_request.allowed_planning_time = allowed_planning_time
    motion_request.max_velocity_scaling_factor = velocity_scale
    motion_request.max_acceleration_scaling_factor = acceleration_scale
    goal = Constraints()
    for name, position in zip(joint_names, goal_joint_positions):
        constraint = JointConstraint()
        constraint.joint_name = name
        constraint.position = position
        constraint.tolerance_above = joint_tolerance_rad
        constraint.tolerance_below = joint_tolerance_rad
        constraint.weight = 1.0
        goal.joint_constraints.append(constraint)
    motion_request.goal_constraints = [goal]
    return request


class P7ArmMotionAdapter(Node):
    def __init__(self) -> None:
        super().__init__("cs625_p7_arm_motion_adapter")
        for name, value in (
            ("execute", False), ("require_confirmation", True),
            ("command_topic", "/p7/arm_motion_command"), ("status_topic", "/p7/arm_motion_status"),
            ("joint_states_topic", "/joint_states"), ("planning_group", "cs625_arm"),
            # IK must use the declared SRDF group tip.  The desired pose is
            # still expressed for the physical tool frame, then converted
            # through live TF before planning.
            ("planning_tip_frame", "my_end_effector_link"), ("physical_tool_frame", "tool0"),
            ("ik_service", "/compute_ik"), ("fk_service", "/compute_fk"),
            ("plan_service", "/plan_kinematic_path"),
            ("cartesian_path_service", "/compute_cartesian_path"),
            ("controller_action", "/joint_trajectory_controller/follow_joint_trajectory"),
            ("request_timeout_sec", 8.0), ("execution_timeout_sec", 90.0),
            ("planning_time_sec", 4.0), ("planning_attempts", 5),
            ("joint_goal_tolerance_rad", 0.001),
            ("goal_position_tolerance_m", 0.001),
            ("goal_orientation_tolerance_rad", 0.005),
            ("max_velocity_scale", 0.25), ("max_acceleration_scale", 0.25),
            ("fk_position_tolerance_m", 0.008), ("fk_orientation_tolerance_rad", 0.05),
            ("execution_joint_tolerance_rad", 0.01),
            ("approach_cartesian_max_step_m", 0.005),
            ("approach_min_fraction", 0.999),
            ("approach_revolute_jump_threshold_rad", 0.5),
            ("approach_prismatic_jump_threshold_m", 0.02),
            ("approach_max_cartesian_speed_mps", 0.05),
        ):
            self.declare_parameter(name, value)
        self.declare_parameter("joint_names", rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter("wrap_solution_joint_names", rclpy.Parameter.Type.STRING_ARRAY)
        self._joint_names = list(self.get_parameter("joint_names").value)
        if not self._joint_names:
            raise ValueError("joint_names must be explicitly configured")
        self._enabled = bool(self.get_parameter("execute").value) and not bool(self.get_parameter("require_confirmation").value)
        self._group = str(self.get_parameter("planning_group").value)
        self._planning_tip = str(self.get_parameter("planning_tip_frame").value)
        self._physical_tool = str(self.get_parameter("physical_tool_frame").value)
        self._request_timeout = float(self.get_parameter("request_timeout_sec").value)
        self._execution_timeout = float(self.get_parameter("execution_timeout_sec").value)
        self._planning_time = float(self.get_parameter("planning_time_sec").value)
        self._planning_attempts = int(self.get_parameter("planning_attempts").value)
        if not 1 <= self._planning_attempts <= 20:
            raise ValueError("planning_attempts must be within [1, 20]")
        self._goal_position_tolerance = float(
            self.get_parameter("goal_position_tolerance_m").value
        )
        self._goal_orientation_tolerance = float(
            self.get_parameter("goal_orientation_tolerance_rad").value
        )
        self._joint_goal_tolerance = float(
            self.get_parameter("joint_goal_tolerance_rad").value
        )
        self._velocity = float(self.get_parameter("max_velocity_scale").value)
        self._acceleration = float(self.get_parameter("max_acceleration_scale").value)
        self._fk_position_tolerance = float(self.get_parameter("fk_position_tolerance_m").value)
        self._fk_orientation_tolerance = float(self.get_parameter("fk_orientation_tolerance_rad").value)
        self._execution_joint_tolerance = float(self.get_parameter("execution_joint_tolerance_rad").value)
        self._approach_cartesian_max_step = float(self.get_parameter("approach_cartesian_max_step_m").value)
        self._approach_min_fraction = float(self.get_parameter("approach_min_fraction").value)
        self._approach_revolute_jump_threshold = float(
            self.get_parameter("approach_revolute_jump_threshold_rad").value
        )
        self._approach_prismatic_jump_threshold = float(
            self.get_parameter("approach_prismatic_jump_threshold_m").value
        )
        self._approach_max_cartesian_speed = float(
            self.get_parameter("approach_max_cartesian_speed_mps").value
        )
        self._wrap_solution_joint_names = set(self.get_parameter("wrap_solution_joint_names").value)
        self._ik = self.create_client(GetPositionIK, str(self.get_parameter("ik_service").value))
        self._fk = self.create_client(GetPositionFK, str(self.get_parameter("fk_service").value))
        self._plan = self.create_client(GetMotionPlan, str(self.get_parameter("plan_service").value))
        self._cartesian = self.create_client(
            GetCartesianPath, str(self.get_parameter("cartesian_path_service").value)
        )
        self._execute = ActionClient(self, FollowJointTrajectory, str(self.get_parameter("controller_action").value))
        self._status = self.create_publisher(String, str(self.get_parameter("status_topic").value), 10)
        self.create_subscription(String, str(self.get_parameter("command_topic").value), self._on_command, 10)
        self.create_subscription(JointState, str(self.get_parameter("joint_states_topic").value), self._on_joint_state, 10)
        self._joints: dict[str, float] = {}
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._active: dict[str, Any] | None = None
        self.create_timer(0.05, self._advance)

    def _on_joint_state(self, message: JointState) -> None:
        self._joints.update(zip(message.name, message.position))

    def _on_command(self, message: String) -> None:
        if self._active is not None:
            return
        try:
            command = decode_motion_command(message.data)
        except ValueError as error:
            self._publish("", "", False, str(error), {})
            return
        if not self._enabled:
            self._publish(command["command_id"], command["phase"], False, "EXECUTION_GATE_CLOSED", {})
            return
        if any(name not in self._joints for name in self._joint_names):
            self._publish(command["command_id"], command["phase"], False, "JOINT_STATES_UNAVAILABLE", {})
            return
        joint_goal = command.get("joint_goal_positions_rad")
        if joint_goal is not None and set(joint_goal) != set(self._joint_names):
            self._publish(
                command["command_id"], command["phase"], False,
                "P7_ARM_COMMAND_INVALID_JOINT_GOAL", {},
            )
            return
        if not (
            self._ik.service_is_ready()
            and self._fk.service_is_ready()
            and self._plan.service_is_ready()
            and self._cartesian.service_is_ready()
            and self._execute.server_is_ready()
        ):
            self._publish(command["command_id"], command["phase"], False, "MOVEIT_UNAVAILABLE", {})
            return
        physical_pose = PoseStamped()
        physical_pose.header.frame_id = command["frame_id"]
        physical_pose.pose.position.x, physical_pose.pose.position.y, physical_pose.pose.position.z = command["x"], command["y"], command["z"]
        physical_pose.pose.orientation.x, physical_pose.pose.orientation.y = command["qx"], command["qy"]
        physical_pose.pose.orientation.z, physical_pose.pose.orientation.w = command["qz"], command["qw"]
        physical_tool = command.get("physical_tool_frame", self._physical_tool)
        try:
            # lookup(tool, tip) yields T_tool_tip.  Therefore
            # T_frame_tip = T_frame_tool T_tool_tip.
            tool_to_tip = self._tf_buffer.lookup_transform(
                physical_tool, self._planning_tip, rclpy.time.Time()
            )
        except TransformException as error:
            self.get_logger().warning(f"P7 tool-to-tip TF unavailable: {error}")
            self._publish(command["command_id"], command["phase"], False, "TOOL_TIP_TF_UNAVAILABLE", {})
            return
        pose = PoseStamped()
        pose.header = physical_pose.header
        pose.pose = compose_pose(physical_pose.pose, transform_as_pose(tool_to_tip.transform))
        current_joint_positions = [self._joints[name] for name in self._joint_names]
        start_state_receipt = {
            name: round(position, 6)
            for name, position in zip(self._joint_names, current_joint_positions)
        }
        self._active = {
            "command": command, "physical_pose": physical_pose, "tip_pose": pose,
            "started": time.monotonic(), "phase_started": time.monotonic(), "durations": {},
            "start_state_joint_positions_rad": dict(zip(self._joint_names, current_joint_positions)),
            "verification": {
                "planning_tip_frame": self._planning_tip,
                "physical_tool_frame": physical_tool,
                "start_state_joint_positions_rad": start_state_receipt,
            },
        }
        if command["phase"] in ("view", "pregrasp"):
            if joint_goal is None:
                request = pose_motion_plan_request(
                    group_name=self._group,
                    tip_pose=pose,
                    planning_tip=self._planning_tip,
                    joint_names=self._joint_names,
                    current_joint_positions=current_joint_positions,
                    planning_attempts=self._planning_attempts,
                    allowed_planning_time=self._planning_time,
                    velocity_scale=self._velocity,
                    acceleration_scale=self._acceleration,
                    position_tolerance_m=self._goal_position_tolerance,
                    orientation_tolerance_rad=self._goal_orientation_tolerance,
                )
                planning_mode = "pose_goal_joint_space"
                goal_constraint_type = "pose"
            else:
                goal_joint_positions = [joint_goal[name] for name in self._joint_names]
                request = joint_motion_plan_request(
                    group_name=self._group,
                    joint_names=self._joint_names,
                    current_joint_positions=current_joint_positions,
                    goal_joint_positions=goal_joint_positions,
                    planning_attempts=self._planning_attempts,
                    allowed_planning_time=self._planning_time,
                    velocity_scale=self._velocity,
                    acceleration_scale=self._acceleration,
                    joint_tolerance_rad=self._joint_goal_tolerance,
                )
                planning_mode = "frozen_joint_goal"
                goal_constraint_type = "joint"
                self._active["joint_goal_positions_rad"] = dict(joint_goal)
            self._active["goal_constraint_type"] = goal_constraint_type
            self._active["verification"].update({
                "planning_mode": planning_mode,
                "goal_constraint_type": goal_constraint_type,
                "goal_position_tolerance_m": self._goal_position_tolerance,
                "goal_orientation_tolerance_rad": self._goal_orientation_tolerance,
                "planning_request_count": 1,
                "replanning_count": 0,
                "internal_planning_attempt_limit": self._planning_attempts,
                "moveit_raw_error_code": None,
                "moveit_collision_rejection": "unknown",
            })
            if joint_goal is not None:
                self._active["verification"].update({
                    "requested_joint_goal_positions_rad": {
                        name: round(joint_goal[name], 9) for name in self._joint_names
                    },
                    "joint_goal_tolerance_rad": self._joint_goal_tolerance,
                })
            self._active["phase"] = "plan"
            self._active["future"] = self._plan.call_async(request)
            return

        request = GetPositionIK.Request()
        request.ik_request.group_name = self._group
        request.ik_request.ik_link_name = self._planning_tip
        request.ik_request.pose_stamped = pose
        request.ik_request.robot_state.is_diff = True
        request.ik_request.robot_state.joint_state.name = self._joint_names
        request.ik_request.robot_state.joint_state.position = current_joint_positions
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout = Duration(sec=max(1, int(self._request_timeout)))
        self._active["verification"].update({
            "planning_mode": "cartesian_linear",
            "goal_constraint_type": "cartesian_waypoint",
        })
        self._active["phase"] = "ik"
        self._active["future"] = self._ik.call_async(request)

    def _advance(self) -> None:
        if self._active is None:
            return
        active = self._active
        phase = active["phase"]
        timeout = self._execution_timeout if phase == "execute" else self._request_timeout
        if not active["future"].done():
            if time.monotonic() - active["phase_started"] > timeout:
                self._finish(False, f"{phase.upper()}_TIMEOUT")
            return
        elapsed = time.monotonic() - active["phase_started"]
        record_phase_duration_once(active["durations"], phase, elapsed)
        try:
            response = active["future"].result()
        except Exception:
            self._finish(False, f"{phase.upper()}_SERVICE_ERROR")
            return
        if phase == "ik":
            if response.error_code.val != MoveItErrorCodes.SUCCESS:
                self._finish(False, "NO_IK")
                return
            self._normalize_wrapped_solution(response.solution.joint_state)
            if active["command"]["phase"] in ("approach", "lift"):
                request = GetCartesianPath.Request()
                request.header = active["tip_pose"].header
                request.start_state.is_diff = True
                request.start_state.joint_state.name = self._joint_names
                request.start_state.joint_state.position = [
                    self._joints[name] for name in self._joint_names
                ]
                request.group_name = self._group
                request.link_name = self._planning_tip
                request.waypoints = [active["tip_pose"].pose]
                request.max_step = self._approach_cartesian_max_step
                request.jump_threshold = 0.0
                request.revolute_jump_threshold = self._approach_revolute_jump_threshold
                request.prismatic_jump_threshold = self._approach_prismatic_jump_threshold
                request.avoid_collisions = True
                request.max_velocity_scaling_factor = self._velocity
                request.max_acceleration_scaling_factor = self._acceleration
                request.cartesian_speed_limited_link = self._planning_tip
                request.max_cartesian_speed = self._approach_max_cartesian_speed
                active["verification"].update({
                    "planning_request_count": 1,
                    "replanning_count": 0,
                    "internal_planning_attempt_limit": 1,
                    "moveit_raw_error_code": None,
                    "moveit_collision_rejection": "unknown",
                })
                self._next("cartesian", self._cartesian.call_async(request))
                return
            self._finish(False, "INTERNAL_STATE_ERROR")
            return
        if phase in ("plan", "cartesian"):
            if phase == "plan":
                raw_error_code = int(response.motion_plan_response.error_code.val)
                active["verification"].update({
                    "moveit_raw_error_code": raw_error_code,
                    "moveit_collision_rejection": moveit_collision_rejection(raw_error_code),
                })
                if raw_error_code != MoveItErrorCodes.SUCCESS:
                    self._finish(False, "PLANNING_FAILED")
                    return
                trajectory = response.motion_plan_response.trajectory.joint_trajectory
                active["planning_mode"] = active["verification"]["planning_mode"]
            else:
                raw_error_code = int(response.error_code.val)
                active["verification"].update({
                    "moveit_raw_error_code": raw_error_code,
                    "moveit_collision_rejection": moveit_collision_rejection(raw_error_code),
                })
                if raw_error_code != MoveItErrorCodes.SUCCESS:
                    self._finish(False, "CARTESIAN_PATH_FAILED")
                    return
                if response.fraction < self._approach_min_fraction:
                    active["verification"].update({
                        "planning_mode": "cartesian_linear",
                        "cartesian_path_fraction": round(response.fraction, 6),
                    })
                    self._finish(False, "CARTESIAN_PATH_INCOMPLETE")
                    return
                trajectory = response.solution.joint_trajectory
                active["planning_mode"] = "cartesian_linear"
                active["cartesian_path_fraction"] = response.fraction
            active["verification"]["planned_joint_path_length_rad"] = round(
                joint_trajectory_path_length_rad(
                    list(trajectory.joint_names),
                    list(trajectory.points),
                    active["start_state_joint_positions_rad"],
                ),
                6,
            )
            if not trajectory.points:
                self._finish(False, "PLANNING_EMPTY_TRAJECTORY")
                return
            fk_request = GetPositionFK.Request()
            fk_request.header.frame_id = active["physical_pose"].header.frame_id
            fk_request.fk_link_names = [active["verification"]["physical_tool_frame"]]
            fk_request.robot_state.is_diff = True
            fk_request.robot_state.joint_state.name = trajectory.joint_names
            fk_request.robot_state.joint_state.position = trajectory.points[-1].positions
            active["trajectory"] = trajectory
            self._next("fk", self._fk.call_async(fk_request))
            return
        if phase == "fk":
            if response.error_code.val != MoveItErrorCodes.SUCCESS or not response.pose_stamped:
                self._finish(False, "FK_VERIFICATION_FAILED")
                return
            position_error, orientation_error = pose_errors(active["physical_pose"].pose, response.pose_stamped[0].pose)
            active.setdefault("verification", {}).update({
                "fk_position_error_m": round(position_error, 6),
                "fk_orientation_error_rad": round(orientation_error, 6),
                "planning_tip_frame": self._planning_tip,
                "physical_tool_frame": active["verification"]["physical_tool_frame"],
                "planning_mode": active.get("planning_mode", "unknown"),
                "goal_constraint_type": active.get("goal_constraint_type", "cartesian_waypoint"),
                "start_state_joint_positions_rad": {
                    name: round(position, 6)
                    for name, position in active["start_state_joint_positions_rad"].items()
                },
                "planned_terminal_joint_positions_rad": {
                    name: round(position, 6)
                    for name, position in zip(active["trajectory"].joint_names, active["trajectory"].points[-1].positions)
                },
                "planned_trajectory_duration_sec": round(
                    active["trajectory"].points[-1].time_from_start.sec
                    + active["trajectory"].points[-1].time_from_start.nanosec / 1_000_000_000.0,
                    6,
                ),
            })
            if "cartesian_path_fraction" in active:
                active["verification"]["cartesian_path_fraction"] = round(
                    active["cartesian_path_fraction"], 6
                )
            if active.get("goal_constraint_type") == "pose":
                active["verification"].update({
                    "goal_position_tolerance_m": self._goal_position_tolerance,
                    "goal_orientation_tolerance_rad": self._goal_orientation_tolerance,
                })
            if position_error > self._fk_position_tolerance or orientation_error > self._fk_orientation_tolerance:
                self._finish(False, "FK_TARGET_MISMATCH")
                return
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = active["trajectory"]
            self._next("goal", self._execute.send_goal_async(goal))
            return
        if phase == "goal":
            if not response.accepted:
                self._finish(False, "EXECUTION_REJECTED")
                return
            self._next("execute", response.get_result_async())
            return
        active.setdefault("verification", {}).update({
            "controller_result_error_code": int(response.result.error_code),
            "controller_result_error_string": str(response.result.error_string),
        })
        if response.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self._finish(False, "EXECUTION_FAILED")
            return
        planned = dict(zip(active["trajectory"].joint_names, active["trajectory"].points[-1].positions))
        observed = {name: self._joints.get(name) for name in planned}
        joint_errors = {
            name: abs(observed[name] - position)
            for name, position in planned.items()
            if observed[name] is not None
        }
        max_joint_error = max(joint_errors.values(), default=float("inf"))
        verification = active.setdefault("verification", {})
        verification.update({
            "execution_terminal_joint_positions_rad": {
                name: round(position, 6) for name, position in observed.items() if position is not None
            },
            "execution_terminal_joint_max_error_rad": round(max_joint_error, 6),
            "execution_joint_tolerance_rad": self._execution_joint_tolerance,
        })
        if max_joint_error > self._execution_joint_tolerance:
            self._finish(False, "EXECUTION_TERMINAL_MISMATCH")
            return
        self._finish(True, "MOTION_SUCCEEDED")

    def _normalize_wrapped_solution(self, state: JointState) -> None:
        """Avoid needless multi-turn solutions in the Gazebo position loop."""
        for index, name in enumerate(state.name):
            if name in self._wrap_solution_joint_names and name in self._joints:
                state.position[index] = nearest_wrapped_joint_position(
                    state.position[index], self._joints[name]
                )

    def _next(self, phase: str, future) -> None:
        assert self._active is not None
        self._active["phase"] = phase
        self._active["phase_started"] = time.monotonic()
        self._active["future"] = future

    def _finish(self, success: bool, code: str) -> None:
        assert self._active is not None
        active = self._active
        record_phase_duration_once(
            active["durations"],
            active["phase"],
            time.monotonic() - active["phase_started"],
        )
        self._publish(
            active["command"]["command_id"], active["command"]["phase"], success, code,
            active["durations"], active.get("verification"),
        )
        self._active = None

    def _publish(
        self, command_id: str, phase: str, success: bool, code: str, durations: dict[str, float],
        details: dict[str, Any] | None = None,
    ) -> None:
        message = String()
        message.data = json.dumps(motion_status_payload(command_id, phase, success, code, durations, details), sort_keys=True)
        self._status.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P7ArmMotionAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
