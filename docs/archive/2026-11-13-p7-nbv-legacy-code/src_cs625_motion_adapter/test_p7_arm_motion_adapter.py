import math

import pytest
from geometry_msgs.msg import Pose, PoseStamped
from shape_msgs.msg import SolidPrimitive

from cs625_motion_adapter.p7_arm_motion_adapter import (
    compose_pose,
    decode_motion_command,
    joint_motion_plan_request,
    joint_trajectory_path_length_rad,
    motion_status_payload,
    moveit_collision_rejection,
    nearest_wrapped_joint_position,
    pose_motion_plan_request,
    pose_errors,
    record_phase_duration_once,
)


def test_motion_command_requires_explicit_phase_and_pose():
    command = decode_motion_command(
        '{"command_id":"e1","phase":"pregrasp","pose":{"frame_id":"base_link",'
        '"position":{"x":0.1,"y":0.2,"z":0.3},'
        '"orientation":{"x":0,"y":0,"z":0,"w":1}}}'
    )
    assert command["phase"] == "pregrasp"
    assert command["frame_id"] == "base_link"
    with pytest.raises(ValueError, match="P7_ARM_COMMAND_INVALID"):
        decode_motion_command('{"command_id":"e1","phase":"grasp"}')


def test_view_is_an_explicit_pose_command_phase():
    command = decode_motion_command(
        '{"command_id":"nbv-1","phase":"view","pose":{"frame_id":"base_link",'
        '"position":{"x":0.7,"y":0.1,"z":0.6},'
        '"orientation":{"x":0,"y":0,"z":0,"w":1}}}'
    )
    assert command["phase"] == "view"


def test_view_accepts_a_frozen_joint_goal_and_camera_tool_frame():
    command = decode_motion_command(
        '{"command_id":"nbv-fixed","phase":"view",'
        '"physical_tool_frame":"camera_depth_optical_frame",'
        '"joint_goal_positions_rad":{"joint_a":0.1,"joint_b":-0.2},'
        '"pose":{"frame_id":"base_link",'
        '"position":{"x":0.7,"y":0.1,"z":0.6},'
        '"orientation":{"x":0,"y":0,"z":0,"w":1}}}'
    )
    assert command["physical_tool_frame"] == "camera_depth_optical_frame"
    assert command["joint_goal_positions_rad"] == {"joint_a": 0.1, "joint_b": -0.2}


def test_motion_command_normalizes_a_serialized_quaternion():
    command = decode_motion_command(
        '{"command_id":"e1","phase":"pregrasp","pose":{"frame_id":"base_link",'
        '"position":{"x":0,"y":0,"z":0},'
        '"orientation":{"x":0.707,"y":0,"z":0,"w":0.707}}}'
    )
    assert math.sqrt(sum(command[key] ** 2 for key in ("qx", "qy", "qz", "qw"))) == pytest.approx(1.0)


def test_pregrasp_accepts_checked_joint_branch_but_cartesian_phases_do_not():
    text = ('{"command_id":"e1","phase":"pregrasp",'
            '"joint_goal_positions_rad":{"joint_a":0.1},'
            '"pose":{"frame_id":"base_link","position":{"x":0,"y":0,"z":0},'
            '"orientation":{"x":0,"y":0,"z":0,"w":1}}}')
    assert decode_motion_command(text)["joint_goal_positions_rad"] == {"joint_a": 0.1}
    with pytest.raises(ValueError, match="INVALID_JOINT_GOAL"):
        decode_motion_command(text.replace('"pregrasp"', '"approach"'))


def test_motion_status_keeps_phase_times_separate():
    status = motion_status_payload(
        "e1", "lift", True, "MOTION_SUCCEEDED", {"ik": 1.2, "plan": 2.3, "execute": 3.4},
        {"execution_terminal_joint_max_error_rad": 0.001},
    )
    assert status["success"] is True
    assert status["total_time_sec"] == 6.9
    assert status["execution_time_sec"] == 3.4
    assert status["execution_terminal_joint_max_error_rad"] == 0.001


def test_cartesian_approach_time_is_reported_as_planning_time():
    status = motion_status_payload(
        "e2", "approach", True, "MOTION_SUCCEEDED",
        {"ik": 0.1, "cartesian": 0.7, "fk": 0.2, "execute": 1.5},
    )
    assert status["planning_time_sec"] == 0.7
    assert status["total_time_sec"] == 2.5


def test_completed_phase_duration_is_not_counted_twice_at_finish():
    durations = {}
    assert record_phase_duration_once(durations, "execute", 3.4) == 3.4
    assert record_phase_duration_once(durations, "execute", 3.5) == 3.4
    assert durations == {"execute": 3.4}


def test_joint_trajectory_path_length_includes_start_state_and_all_segments():
    class Point:
        def __init__(self, positions):
            self.positions = positions

    length = joint_trajectory_path_length_rad(
        ["joint_a", "joint_b"],
        [Point([3.0, 4.0]), Point([3.0, 8.0])],
        {"joint_a": 0.0, "joint_b": 0.0},
    )
    assert length == pytest.approx(9.0)


def test_moveit_collision_semantics_do_not_infer_false_from_generic_codes():
    from moveit_msgs.msg import MoveItErrorCodes

    assert moveit_collision_rejection(MoveItErrorCodes.START_STATE_IN_COLLISION) is True
    assert moveit_collision_rejection(MoveItErrorCodes.GOAL_IN_COLLISION) is True
    assert moveit_collision_rejection(MoveItErrorCodes.PLANNING_FAILED) == "unknown"
    assert moveit_collision_rejection(MoveItErrorCodes.SUCCESS) == "unknown"
    assert moveit_collision_rejection(None) == "unknown"


def test_pose_capture_accepts_view_phase():
    from pathlib import Path

    capture = (
        Path(__file__).resolve().parents[3] / "test" / "p7_execute_pose_capture.py"
    ).read_text(encoding="utf-8")
    assert 'choices=("view", "pregrasp", "approach", "lift")' in capture


def test_tool_to_declared_tip_conversion_and_fk_error_are_explicit():
    physical_tool = Pose()
    physical_tool.position.x = 0.7
    physical_tool.orientation.w = 1.0
    tool_to_tip = Pose()
    tool_to_tip.position.x = -0.09
    tool_to_tip.orientation.w = 1.0
    planned_tip = compose_pose(physical_tool, tool_to_tip)
    assert planned_tip.position.x == pytest.approx(0.61)
    position, orientation = pose_errors(physical_tool, physical_tool)
    assert position == 0.0
    assert orientation == 0.0
    rotated = Pose()
    rotated.orientation.z = math.sin(math.pi / 4.0)
    rotated.orientation.w = math.cos(math.pi / 4.0)
    _, orientation = pose_errors(physical_tool, rotated)
    assert orientation == pytest.approx(math.pi / 2.0)


def test_pose_composition_keeps_a_relative_rotation_with_identity_base():
    base = Pose()
    base.orientation.w = 1.0
    relative = Pose()
    relative.orientation.z = math.sin(math.pi / 4.0)
    relative.orientation.w = math.cos(math.pi / 4.0)
    composed = compose_pose(base, relative)
    assert composed.orientation.z == pytest.approx(relative.orientation.z)
    assert composed.orientation.w == pytest.approx(relative.orientation.w)


def test_grasp_center_offset_is_explicit_in_the_application_description():
    from pathlib import Path

    description = (
        Path(__file__).resolve().parents[2]
        / "cs625_ap_description"
        / "urdf"
        / "cs625_active_perception.urdf.xacro"
    ).read_text(encoding="utf-8")
    assert 'p7_grasp_center_link' in description
    assert 'xyz="0.080 0 0"' in description


def test_p7_profile_uses_internal_planning_attempts_without_external_retries():
    from pathlib import Path

    profile = (
        Path(__file__).resolve().parents[2]
        / "cs625_bringup"
        / "config"
        / "p7_static_grasp_sim.yaml"
    ).read_text(encoding="utf-8")
    assert "planning_attempts: 5" in profile
    assert "goal_position_tolerance_m: 0.001" in profile
    assert "goal_orientation_tolerance_rad: 0.005" in profile

    runner = (
        Path(__file__).resolve().parents[3]
        / "test"
        / "run_p7_static_episode_capture.sh"
    ).read_text(encoding="utf-8")
    assert "p7_pregrasp_path_search.py" not in runner

    launch_runner = (
        Path(__file__).resolve().parents[3]
        / "test"
        / "run_p7_tipfix_sim.sh"
    ).read_text(encoding="utf-8")
    for adapter in ("arm", "gripper", "attachment"):
        assert f"test/run_p7_adapter.sh {adapter}" in launch_runner
    for node in (
        "/cs625_p7_arm_motion_adapter",
        "/cs625_p7_gripper_adapter",
        "/cs625_p7_attachment_adapter",
    ):
        assert node in launch_runner

    manifest = (
        Path(__file__).resolve().parents[3]
        / "test"
        / "p7_capture_manifest.py"
    ).read_text(encoding="utf-8")
    assert '"test/run_p7_adapter.sh"' in manifest


def test_pose_goal_plan_uses_explicit_start_state_and_no_frozen_joint_goal():
    pose = PoseStamped()
    pose.header.frame_id = "base_link"
    pose.pose.position.x = 0.61
    pose.pose.position.y = -0.34
    pose.pose.position.z = 0.30
    pose.pose.orientation.w = 1.0
    names = ["joint_a", "joint_b"]
    positions = [0.2, -0.4]

    request = pose_motion_plan_request(
        group_name="cs625_arm",
        tip_pose=pose,
        planning_tip="my_end_effector_link",
        joint_names=names,
        current_joint_positions=positions,
        planning_attempts=5,
        allowed_planning_time=4.0,
        velocity_scale=0.25,
        acceleration_scale=0.25,
        position_tolerance_m=0.001,
        orientation_tolerance_rad=0.005,
    )

    motion_request = request.motion_plan_request
    assert motion_request.group_name == "cs625_arm"
    assert motion_request.start_state.is_diff is True
    assert list(motion_request.start_state.joint_state.name) == names
    assert list(motion_request.start_state.joint_state.position) == positions
    assert motion_request.num_planning_attempts == 5
    assert len(motion_request.goal_constraints) == 1
    goal = motion_request.goal_constraints[0]
    assert not goal.joint_constraints
    assert len(goal.position_constraints) == 1
    assert len(goal.orientation_constraints) == 1

    position = goal.position_constraints[0]
    assert position.link_name == "my_end_effector_link"
    primitive = position.constraint_region.primitives[0]
    assert primitive.type == SolidPrimitive.BOX
    assert list(primitive.dimensions) == pytest.approx([0.002, 0.002, 0.002])
    region_pose = position.constraint_region.primitive_poses[0]
    assert region_pose.position.x == pytest.approx(0.61)
    assert region_pose.position.y == pytest.approx(-0.34)
    assert region_pose.position.z == pytest.approx(0.30)

    orientation = goal.orientation_constraints[0]
    assert orientation.link_name == "my_end_effector_link"
    assert orientation.absolute_x_axis_tolerance == pytest.approx(0.005)
    assert orientation.absolute_y_axis_tolerance == pytest.approx(0.005)
    assert orientation.absolute_z_axis_tolerance == pytest.approx(0.005)


def test_pose_goal_plan_rejects_invalid_tolerances():
    pose = PoseStamped()
    pose.header.frame_id = "base_link"
    pose.pose.orientation.w = 1.0
    with pytest.raises(ValueError, match="position_tolerance_m"):
        pose_motion_plan_request(
            group_name="cs625_arm",
            tip_pose=pose,
            planning_tip="my_end_effector_link",
            joint_names=["joint_a"],
            current_joint_positions=[0.0],
            planning_attempts=1,
            allowed_planning_time=4.0,
            velocity_scale=0.25,
            acceleration_scale=0.25,
            position_tolerance_m=0.0,
            orientation_tolerance_rad=0.005,
        )


def test_frozen_joint_goal_plan_uses_explicit_start_and_joint_constraints():
    request = joint_motion_plan_request(
        group_name="cs625_arm",
        joint_names=["joint_a", "joint_b"],
        current_joint_positions=[0.2, -0.4],
        goal_joint_positions=[0.8, 0.1],
        planning_attempts=5,
        allowed_planning_time=4.0,
        velocity_scale=0.25,
        acceleration_scale=0.25,
        joint_tolerance_rad=0.001,
    )
    motion_request = request.motion_plan_request
    assert motion_request.start_state.is_diff is True
    assert list(motion_request.start_state.joint_state.name) == ["joint_a", "joint_b"]
    assert list(motion_request.start_state.joint_state.position) == [0.2, -0.4]
    assert motion_request.num_planning_attempts == 5
    goal = motion_request.goal_constraints[0]
    assert not goal.position_constraints
    assert not goal.orientation_constraints
    assert [constraint.joint_name for constraint in goal.joint_constraints] == [
        "joint_a", "joint_b"
    ]
    assert [constraint.position for constraint in goal.joint_constraints] == pytest.approx(
        [0.8, 0.1]
    )
    assert all(
        constraint.tolerance_above == pytest.approx(0.001)
        and constraint.tolerance_below == pytest.approx(0.001)
        for constraint in goal.joint_constraints
    )


def test_view_trace_can_load_the_versioned_six_joint_goal():
    from pathlib import Path

    trace = (
        Path(__file__).resolve().parents[3] / "test" / "p7_execute_view_trace.py"
    ).read_text(encoding="utf-8")
    assert '"--joint-goal-file"' in trace
    assert '"joint_goal_positions_rad"' in trace
    assert '"physical_tool_frame": arguments.trace_frame' in trace


def test_wrapped_solution_is_nearest_to_monitored_state():
    assert nearest_wrapped_joint_position(-5.567312, 0.0) == pytest.approx(0.715873, abs=1e-5)
    assert nearest_wrapped_joint_position(5.18717, 0.0) == pytest.approx(-1.096015, abs=1e-5)
