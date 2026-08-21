#!/usr/bin/env python3
"""Host-side static checks for the Phase 0–6 repository contract."""

from __future__ import annotations

import ast
import os
import pathlib
import sys
import xml.etree.ElementTree as ET


ROOT = pathlib.Path(__file__).resolve().parents[1]
EXPECTED_PACKAGES = {
    "cs625_ap_interfaces",
    "cs625_ap_description",
    "cs625_sensor_adapter",
    "cs625_simulation",
    "cs625_bringup",
    "cs625_target_perception",
    "cs625_view_generation",
    "cs625_motion_adapter",
    "cs625_view_evaluation",
    "cs625_experiment_tools",
}
GENERATED_DIRS = {"build", "install", "log", ".colcon"}


def fail(message: str) -> None:
    raise AssertionError(message)


def main() -> int:
    src = ROOT / "src"
    package_dirs = {
        path.name
        for path in src.iterdir()
        if path.is_dir() and (path / "package.xml").exists()
    }
    if package_dirs != EXPECTED_PACKAGES:
        fail(f"Phase 0–1 package set mismatch: {sorted(package_dirs)}")

    required_files = [
        ROOT / "AGENTS.md",
        ROOT / "README.md",
        ROOT / ".repos" / "common.repos",
        ROOT / ".repos" / "sim.repos",
        ROOT / "docs" / "architecture.md",
        ROOT / "docs" / "dependencies.md",
        ROOT / "docs" / "interfaces.md",
        ROOT / "docs" / "frames_and_topics.md",
        ROOT / "docs" / "simulation.md",
        ROOT / "docs" / "migration_from_legacy.md",
        src / "cs625_bringup" / "launch" / "sensor_adapter.launch.py",
        src / "cs625_bringup" / "launch" / "sim_sensor_bridge.launch.py",
        src / "cs625_bringup" / "launch" / "sim_moveit.launch.py",
        src / "cs625_bringup" / "launch" / "sim_active_localization.launch.py",
        src / "cs625_bringup" / "launch" / "real_base.launch.py",
        src / "cs625_bringup" / "config" / "common.yaml",
        src / "cs625_bringup" / "config" / "sim.yaml",
        src / "cs625_bringup" / "config" / "real.yaml",
        src / "cs625_bringup" / "config" / "sim_gz_bridge.yaml",
        src / "cs625_bringup" / "config" / "moveit_sensors_3d.yaml",
        src / "cs625_bringup" / "config" / "sim_controllers.yaml",
        ROOT / "scripts" / "bootstrap_humble.sh",
        ROOT / "scripts" / "build.sh",
        ROOT / "scripts" / "test.sh",
        ROOT / "scripts" / "doctor_humble.sh",
        src / "cs625_sensor_adapter" / "tests" / "test_import.py",
        src / "cs625_target_perception" / "tests" / "test_import.py",
        src / "cs625_bringup" / "launch" / "sim_ground_truth.launch.py",
        src / "cs625_bringup" / "launch" / "sim_p2_pipeline.launch.py",
        src / "cs625_bringup" / "launch" / "sim_view_planning.launch.py",
        src / "cs625_bringup" / "launch" / "sim_baseline_strategy.launch.py",
        src / "cs625_bringup" / "config" / "view_planning_sim.yaml",
        src / "cs625_bringup" / "config" / "view_evaluation_sim.yaml",
        src / "cs625_bringup" / "config" / "p4_sim_loop.yaml",
        src / "cs625_bringup" / "scripts" / "prepare_gazebo_model.py",
        src / "cs625_ap_interfaces" / "msg" / "ViewCandidate.msg",
        src / "cs625_ap_interfaces" / "msg" / "ViewCandidateArray.msg",
        src / "cs625_view_generation" / "cs625_view_generation" / "candidate_generator.py",
        src / "cs625_motion_adapter" / "cs625_motion_adapter" / "reachability_filter.py",
        src / "cs625_view_evaluation" / "cs625_view_evaluation" / "baseline_selector.py",
        src / "cs625_motion_adapter" / "cs625_motion_adapter" / "sim_view_executor.py",
        src / "cs625_view_evaluation" / "cs625_view_evaluation" / "sim_episode_coordinator.py",
        src / "cs625_bringup" / "launch" / "sim_p4_loop.launch.py",
        src / "cs625_bringup" / "launch" / "sim_p5_joint_score.launch.py",
        src / "cs625_bringup" / "launch" / "real_preflight.launch.py",
        src / "cs625_bringup" / "launch" / "real_readiness.launch.py",
        src / "cs625_bringup" / "launch" / "real_execution.launch.py",
        src / "cs625_bringup" / "config" / "p5_joint_score_sim.yaml",
        src / "cs625_bringup" / "config" / "p6_real_safety.yaml",
        src / "cs625_bringup" / "config" / "p6_readiness.yaml",
        src / "cs625_bringup" / "config" / "p6_real_execution.yaml",
        src / "cs625_bringup" / "config" / "minimum_showcase_matrix.json",
        src / "cs625_view_evaluation" / "cs625_view_evaluation" / "joint_score.py",
        src / "cs625_view_evaluation" / "cs625_view_evaluation" / "policy.py",
        src / "cs625_motion_adapter" / "cs625_motion_adapter" / "real_preflight.py",
        src / "cs625_motion_adapter" / "cs625_motion_adapter" / "real_readiness_monitor.py",
        src / "cs625_motion_adapter" / "cs625_motion_adapter" / "real_view_executor.py",
        src / "cs625_experiment_tools" / "cs625_experiment_tools" / "paired_experiment.py",
        src / "cs625_experiment_tools" / "cs625_experiment_tools" / "p4_matrix_summary.py",
        ROOT / "docs" / "p5_experiment_protocol.md",
        ROOT / "docs" / "minimum_showcase_experiment.md",
        ROOT / "scripts" / "run_minimum_showcase_matrix.sh",
        ROOT / "docs" / "real_hardware_readiness.md",
        ROOT / "docs" / "p6_on_site_runbook.md",
        ROOT / "test" / "data" / "p5_candidate_snapshot.json",
        ROOT / "test" / "data" / "p5_paired_experiment.json",
    ]
    for path in required_files:
        if not path.exists():
            fail(f"Missing required file: {path.relative_to(ROOT)}")

    for package_dir in sorted(package_dirs):
        package_xml = src / package_dir / "package.xml"
        root = ET.parse(package_xml).getroot()
        name = root.findtext("name")
        if name != package_dir:
            fail(f"package.xml name mismatch in {package_dir}: {name}")
        export = root.find("export")
        build_type = export.findtext("build_type") if export is not None else None
        expected_build_type = (
            "ament_python" if package_dir in (
                "cs625_sensor_adapter",
                "cs625_target_perception",
                "cs625_view_generation",
                "cs625_motion_adapter",
                "cs625_view_evaluation",
                "cs625_experiment_tools",
            ) else "ament_cmake"
        )
        if build_type != expected_build_type:
            fail(
                f"build type mismatch in {package_dir}: "
                f"expected {expected_build_type}, got {build_type}"
            )

    bringup_root = ET.parse(src / "cs625_bringup" / "package.xml").getroot()
    bringup_exec_dependencies = {
        element.text for element in bringup_root.findall("exec_depend")
    }
    for required_dependency in (
        "moveit_ros_occupancy_map_monitor",
        "moveit_ros_perception",
    ):
        if required_dependency not in bringup_exec_dependencies:
            fail(
                "cs625_bringup must reuse the upstream MoveIt point-cloud stack: "
                f"missing {required_dependency}"
            )

    for xml_path in [
        src / "cs625_ap_description" / "urdf" / "cs625_camera_extension.xacro",
        src / "cs625_ap_description" / "urdf" / "cs625_active_perception.urdf.xacro",
        src / "cs625_simulation" / "worlds" / "minimal_occlusion.sdf",
    ]:
        ET.parse(xml_path)

    description_wrapper = (
        src / "cs625_ap_description" / "urdf" / "cs625_active_perception.urdf.xacro"
    ).read_text(encoding="utf-8")
    camera_extension = (
        src / "cs625_ap_description" / "urdf" / "cs625_camera_extension.xacro"
    ).read_text(encoding="utf-8")
    description_contract = description_wrapper + "\n" + camera_extension
    for required in (
        "$(find eli_cs_robot_description)/urdf/cs_macro.xacro",
        "$(find cs625_ap_description)/urdf/cs625_camera_extension.xacro",
        "my_end_effector_link",
        "tool0",
        "camera_depth_optical_frame",
        "camera_color_optical_frame",
        "tf_prefix",
        "sim_ignition",
        "camera_sensor_topic",
        'filename="gz_ros2_control-system"',
        "simulation_controllers",
        "robot_param_node",
        "robot_state_publisher",
        "controller_manager_name",
    ):
        if required not in description_contract:
            fail(f"description wrapper is missing required reuse/frame marker: {required}")
    if 'simulation_controllers="$(arg simulation_controllers)"' in description_wrapper:
        fail(
            "description wrapper passes simulation_controllers to cs_robot, but the "
            "reused senior macro does not accept that parameter"
        )
    if "package://eli_cs_robot_description/meshes/visual/tool0_" in description_wrapper:
        fail(
            "application end-effector meshes must use resolved file URIs for Gazebo"
        )

    sdf_text = (
        src / "cs625_simulation" / "worlds" / "minimal_occlusion.sdf"
    ).read_text(encoding="utf-8")
    fixture_sdf_text = (
        src / "cs625_simulation" / "worlds" / "rgbd_fixture.sdf"
    ).read_text(encoding="utf-8")
    if 'type="rgbd_camera"' not in fixture_sdf_text:
        fail("simulation fixture does not declare an RGB-D camera sensor")
    if "fixture_rgbd_camera" in sdf_text:
        fail("robot world must not contain a second fixed RGB-D camera")
    if "ignition-gazebo-sensors-system" not in sdf_text:
        fail("robot world must load the Gazebo Sensors system for its eye-in-hand camera")
    if "ignition-gazebo-user-commands-system" not in sdf_text:
        fail("robot world must load UserCommands before spawning the eye-in-hand sensor")
    minimal_world_root = ET.parse(
        src / "cs625_simulation" / "worlds" / "minimal_occlusion.sdf"
    ).getroot()
    if minimal_world_root.find(".//visual/material") is not None:
        fail("robot world must not use legacy fixed-function visual materials with Ogre2")
    if "<render_engine>ogre2</render_engine>" not in sdf_text + fixture_sdf_text:
        fail("Fortress headless RGB-D worlds must use the EGL-capable Ogre2 renderer")

    launch_text = (
        ROOT / "src" / "cs625_bringup" / "launch" / "sim_base.launch.py"
    ).read_text(encoding="utf-8")
    launch_tree = ast.parse(launch_text)
    world_defaults = {}
    ast_string_type = ast.Str if sys.version_info < (3, 8) else ast.Constant
    ast_string_value = "s" if sys.version_info < (3, 8) else "value"
    for assignment in ast.walk(launch_tree):
        if (
            not isinstance(assignment, ast.Assign)
            or len(assignment.targets) != 1
            or not isinstance(assignment.targets[0], ast.Name)
            or assignment.targets[0].id not in {"world_default", "fixture_world_default"}
        ):
            continue
        world_defaults[assignment.targets[0].id] = next(
            (
                getattr(node, ast_string_value, "")
                for node in ast.walk(assignment.value)
                if isinstance(node, ast_string_type)
                and isinstance(getattr(node, ast_string_value, ""), str)
                and getattr(node, ast_string_value, "").endswith(".sdf")
            ),
            "",
        )
    if world_defaults.get("world_default") != "minimal_occlusion.sdf":
        fail("sim_base world must default to the robot/occlusion world")
    if world_defaults.get("fixture_world_default") != "rgbd_fixture.sdf":
        fail("sim_base fixture_world must default to the RGB-D fixture")
    real_launch_path = ROOT / "src" / "cs625_bringup" / "launch" / "real_base.launch.py"
    real_launch_text = real_launch_path.read_text(encoding="utf-8")
    for launch_path, launch_source in (
        (ROOT / "src" / "cs625_bringup" / "launch" / "sim_base.launch.py", launch_text),
        (real_launch_path, real_launch_text),
    ):
        if "LogWarn" in launch_source:
            fail(
                f"{launch_path.relative_to(ROOT)} uses LogWarn, which is not available "
                "in the Humble launch.actions API"
            )
    if 'DeclareLaunchArgument("execute", default_value="false"' not in launch_text:
        fail("sim_base.launch.py does not keep execute=false by default")
    if 'DeclareLaunchArgument("require_confirmation", default_value="true"' not in launch_text:
        fail("sim_base.launch.py does not require confirmation by default")
    for plugin_path_marker in (
        'get_package_prefix("gz_ros2_control")',
        '"IGN_GAZEBO_SYSTEM_PLUGIN_PATH"',
        '"GZ_SIM_SYSTEM_PLUGIN_PATH"',
        '"libgz_ros2_control-system.so"',
    ):
        if plugin_path_marker not in launch_text:
            fail(f"Gazebo control plugin path setup is missing: {plugin_path_marker}")

    sim_control_text = (
        ROOT / "src" / "cs625_bringup" / "launch" / "sim_control.launch.py"
    ).read_text(encoding="utf-8")
    for spawn_marker in (
        '"-file",\n            gazebo_model_path',
        'name="cs625_spawn_robot"',
        "RegisterEventHandler",
        "OnProcessExit",
        "prepare_gazebo_model",
        "prepare_gazebo_model.py",
        "target_action=spawn_robot",
        "target_action=joint_state_spawner",
        "--headless-rendering",
        "--controller-manager-timeout",
        "--switch-timeout",
        "--service-call-timeout",
        "camera_image_width",
        "camera_image_height",
        "camera_update_rate",
        "camera_enabled",
        "gazebo_model_file",
    ):
        if spawn_marker not in sim_control_text:
            fail(f"deterministic Gazebo control launch is missing: {spawn_marker}")
    if (
        '"-topic"' in sim_control_text
        or '"-string"' in sim_control_text
        or '"-allow_renaming"' in sim_control_text
        or "cs625_spawn_retry" in launch_text
    ):
        fail("simulation control must use one file-based spawn without a retry entity")

    model_prepare_text = (
        ROOT / "src" / "cs625_bringup" / "scripts" / "prepare_gazebo_model.py"
    ).read_text(encoding="utf-8")
    for model_prepare_marker in (
        "subprocess.run",
        "normalize_xacro_arguments",
        "visual.find(\"./geometry/mesh\")",
        "link.remove(visual)",
        "temporary_path.replace(arguments.output)",
    ):
        if model_prepare_marker not in model_prepare_text:
            fail(f"Gazebo visual-mesh preparation is missing: {model_prepare_marker}")

    active_launch_text = (
        ROOT / "src" / "cs625_bringup" / "launch" / "sim_active_localization.launch.py"
    ).read_text(encoding="utf-8")
    for active_marker in (
        'DeclareLaunchArgument("strategy", default_value="disabled"',
        'default_value="sim_control.launch.py"',
        'default_value="cs625_bringup"',
        'default_value="sim_controllers.yaml"',
        "no target-perception/NBV algorithm is started",
    ):
        if active_marker not in active_launch_text:
            fail(f"sim active-localization entry is missing: {active_marker}")

    adapter_launch_text = (
        ROOT / "src" / "cs625_bringup" / "launch" / "sensor_adapter.launch.py"
    ).read_text(encoding="utf-8")
    for required in (
        'package="cs625_sensor_adapter"',
        'executable="rgbd_sensor_adapter"',
        'ParameterFile(LaunchConfiguration("common_config")',
        'ParameterFile(LaunchConfiguration("profile_config")',
    ):
        if required not in adapter_launch_text:
            fail(f"common sensor adapter launch is missing: {required}")
    if "if value:" not in adapter_launch_text:
        fail("sensor adapter launch must not override profile topics with empty arguments")

    bridge_launch_text = (
        ROOT / "src" / "cs625_bringup" / "launch" / "sim_sensor_bridge.launch.py"
    ).read_text(encoding="utf-8")
    if 'package="ros_gz_bridge"' not in bridge_launch_text:
        fail("simulation sensor bridge must reuse ros_gz_bridge")
    bridge_config_text = (
        src / "cs625_bringup" / "config" / "sim_gz_bridge.yaml"
    ).read_text(encoding="utf-8")
    for bridge_marker in (
        'gz_topic_name: "/camera/image"',
        'gz_topic_name: "/camera/depth_image"',
        'gz_topic_name: "/camera/camera_info"',
        'gz_topic_name: "/camera/points"',
        'ros_topic_name: "/sim/camera/image"',
        'ros_topic_name: "/sim/camera/depth_image"',
        'ros_topic_name: "/sim/camera/camera_info"',
        'ros_topic_name: "/sim/camera/points"',
    ):
        if bridge_marker not in bridge_config_text:
            fail(f"simulation bridge contract is missing: {bridge_marker}")

    moveit_launch_text = (
        ROOT / "src" / "cs625_bringup" / "launch" / "sim_moveit.launch.py"
    ).read_text(encoding="utf-8")
    for moveit_marker in (
        "MoveItConfigsBuilder",
        ".robot_description(file_path=description_path",
        ".robot_description_semantic(file_path=srdf_path",
        ".sensors_3d(file_path=sensors_file)",
        'package="moveit_ros_move_group"',
        '"use_sim_time": True',
        '"octomap_frame": "base_link"',
        '"octomap_resolution": 0.02',
    ):
        if moveit_marker not in moveit_launch_text:
            fail(f"application MoveIt composition is missing reuse marker: {moveit_marker}")
    moveit_sensor_config = (
        src / "cs625_bringup" / "config" / "moveit_sensors_3d.yaml"
    ).read_text(encoding="utf-8")
    if "/sensors/camera/points" not in moveit_sensor_config:
        fail("MoveIt sensor configuration must consume the normalized point cloud topic")

    view_planning_config = (
        src / "cs625_bringup" / "config" / "view_planning_sim.yaml"
    ).read_text(encoding="utf-8")
    for marker in (
        "candidate_count: 24",
        "planning_frame: base_link",
        "camera_frame: camera_depth_optical_frame",
        "tool_frame: tool0",
        "workspace_min:",
        "workspace_max:",
    ):
        if marker not in view_planning_config:
            fail(f"P3 simulation view-planning profile is missing: {marker}")

    candidate_source = (
        src / "cs625_view_generation" / "cs625_view_generation" / "candidate_generator.py"
    ).read_text(encoding="utf-8")
    for marker in (
        "candidate_count must be between 20 and 50",
        "look_at_camera_pose",
        "ViewCandidateArray",
        "raw_candidates_topic",
    ):
        if marker not in candidate_source:
            fail(f"P3 candidate generator is missing: {marker}")
    motion_source = (
        src / "cs625_motion_adapter" / "cs625_motion_adapter" / "reachability_filter.py"
    ).read_text(encoding="utf-8")
    for marker in (
        "GetPositionIK",
        "GetStateValidity",
        "GetMotionPlan",
        "NO_IK",
        "PLANNING_FAILED",
        "TF_UNAVAILABLE",
        "planning-only",
        "return float(sum(",
        "_publish_reachable_with_replay",
        "reachable_publish_replay_count",
    ):
        if marker not in motion_source:
            fail(f"P3 reachability filter is missing: {marker}")
    if "FollowJointTrajectory" in motion_source or "ActionClient" in motion_source:
        fail("P3 reachability filter must not contain trajectory execution code")

    executor_source = (
        src / "cs625_motion_adapter" / "cs625_motion_adapter" / "sim_view_executor.py"
    ).read_text(encoding="utf-8")
    for marker in (
        'profile == "sim"',
        "EXECUTION_GATE_CLOSED",
        "FollowJointTrajectory",
        "SENSOR_SETTLED",
        "_publish_status_with_replay",
        "ClockType.STEADY_TIME",
    ):
        if marker not in executor_source:
            fail(f"P4 simulation executor safety/closure marker missing: {marker}")
    coordinator_source = (
        src / "cs625_view_evaluation" / "cs625_view_evaluation" / "sim_episode_coordinator.py"
    ).read_text(encoding="utf-8")
    for marker in (
        "max_views must be between 1 and 3",
        "max_failed_attempts must be between 1 and 10",
        "NO_REACHABLE_AFTER_FAILURE",
        "view_records",
        "_publish_selection_with_replay",
    ):
        if marker not in coordinator_source:
            fail(f"P4 episode coordinator marker missing: {marker}")
    p4_config_text = (src / "cs625_bringup" / "config" / "p4_sim_loop.yaml").read_text(
        encoding="utf-8"
    )
    if "use_sim_time: true" not in p4_config_text:
        fail("P4 executor must compare post-motion sensor data in Gazebo simulation time")
    p4_matrix_source = (
        src / "cs625_experiment_tools" / "cs625_experiment_tools" / "p4_matrix_summary.py"
    ).read_text(encoding="utf-8")
    matrix_manifest = __import__("json").loads(
        (src / "cs625_bringup" / "config" / "minimum_showcase_matrix.json").read_text(
            encoding="utf-8"
        )
    )
    if (
        len(matrix_manifest["scene_ids"]) != 3
        or len(matrix_manifest["seeds"]) != 5
        or len(matrix_manifest["strategies"]) < 3
    ):
        fail("minimum showcase must register 3 scenes × 5 seeds × at least 3 baselines")
    for marker in ("reachability_rate", "planning_time_sec_total", "motion_cost_total", "successful_observation_rate", "failure_codes"):
        if marker not in p4_matrix_source:
            fail(f"P4 matrix export is missing metric: {marker}")
    for scene in matrix_manifest["scene_ids"]:
        scene_path = src / "cs625_simulation" / "worlds" / f"{scene}.sdf"
        if not scene_path.exists():
            fail(f"registered showcase scene is missing: {scene_path.relative_to(ROOT)}")
        ET.parse(scene_path)

    joint_score_source = (
        src / "cs625_view_evaluation" / "cs625_view_evaluation" / "joint_score.py"
    ).read_text(encoding="utf-8")
    policy_source = (
        src / "cs625_view_evaluation" / "cs625_view_evaluation" / "policy.py"
    ).read_text(encoding="utf-8")
    p5_config_text = (src / "cs625_bringup" / "config" / "p5_joint_score_sim.yaml").read_text(
        encoding="utf-8"
    )
    experiment_source = (
        src / "cs625_experiment_tools" / "cs625_experiment_tools" / "paired_experiment.py"
    ).read_text(encoding="utf-8")
    for marker in ("localization_gain_proxy", "proposed_joint_score", "JointScoreConfig"):
        if marker not in joint_score_source:
            fail(f"P5 joint score is missing: {marker}")
    for marker in ("reachability_only", "proposed_joint_score", "SCORE_CONFIG_REQUIRED"):
        if marker not in policy_source:
            fail(f"P5 selection policy is missing: {marker}")
    for marker in (
        "strategy: proposed_joint_score",
        "localization_gain_weight:",
        "reachability_quality_weight:",
        "motion_cost_weight:",
        "planning_time_weight:",
    ):
        if marker not in p5_config_text:
            fail(f"P5 score profile is missing: {marker}")
    for marker in ("candidate_scores.csv", "paired_summary.csv", "summary_plot.png", "Research use allowed"):
        if marker not in experiment_source:
            fail(f"P5 experiment artifact is missing: {marker}")

    real_preflight_source = (
        src / "cs625_motion_adapter" / "cs625_motion_adapter" / "real_preflight.py"
    ).read_text(encoding="utf-8")
    real_preflight_launch = (
        src / "cs625_bringup" / "launch" / "real_preflight.launch.py"
    ).read_text(encoding="utf-8")
    p6_config_text = (src / "cs625_bringup" / "config" / "p6_real_safety.yaml").read_text(
        encoding="utf-8"
    )
    for marker in ("REAL_EXECUTION_REFUSED", "real_motion_occurred", "R0_PROFILE_SAFE"):
        if marker not in real_preflight_source:
            fail(f"P6 preflight safety marker is missing: {marker}")
    if "ActionClient" in real_preflight_source or "FollowJointTrajectory" in real_preflight_source:
        fail("P6 real preflight must remain status-only")
    for marker in ('executable="real_preflight"', 'default_value="false"', 'default_value="true"'):
        if marker not in real_preflight_launch:
            fail(f"P6 preflight launch safety marker is missing: {marker}")
    for marker in ("execute: false", "require_confirmation: true", "max_velocity_scale: 0.10"):
        if marker not in p6_config_text:
            fail(f"P6 real profile is missing safe default: {marker}")
    for marker in ("controller_activation_allowed", "requested real controller activation was blocked"):
        if marker not in real_launch_text:
            fail(f"P6 controller activation gate is missing: {marker}")

    readiness_source = (
        src / "cs625_motion_adapter" / "cs625_motion_adapter" / "real_readiness_monitor.py"
    ).read_text(encoding="utf-8")
    readiness_launch = (
        src / "cs625_bringup" / "launch" / "real_readiness.launch.py"
    ).read_text(encoding="utf-8")
    for marker in ("r1_joint_state", "r2_hand_eye_manifest", "r3_moveit_services", "real_motion_occurred"):
        if marker not in readiness_source:
            fail(f"P6 readiness evidence marker is missing: {marker}")
    if "ActionClient" in readiness_source or "FollowJointTrajectory" in readiness_source:
        fail("P6 readiness monitor must remain read-only")
    if 'executable="real_readiness_monitor"' not in readiness_launch:
        fail("P6 readiness launch does not use the read-only monitor")

    real_executor_source = (
        src / "cs625_motion_adapter" / "cs625_motion_adapter" / "real_view_executor.py"
    ).read_text(encoding="utf-8")
    real_executor_launch = (
        src / "cs625_bringup" / "launch" / "real_execution.launch.py"
    ).read_text(encoding="utf-8")
    real_executor_config = (src / "cs625_bringup" / "config" / "p6_real_execution.yaml").read_text(
        encoding="utf-8"
    )
    for marker in ("r4_authorized", "operator_id", "approval_id", "max_velocity_scale", "R4_AUTHORIZATION_REQUIRED"):
        if marker not in real_executor_source:
            fail(f"P6 R4 authorization marker is missing: {marker}")
    for marker in ('DeclareLaunchArgument("start_executor", default_value="false")', 'executable="real_view_executor"'):
        if marker not in real_executor_launch:
            fail(f"P6 R4 launch gate is missing: {marker}")
    for marker in ("execute: false", "require_confirmation: true", "r4_authorized: false", "max_velocity_scale: 0.10"):
        if marker not in real_executor_config:
            fail(f"P6 R4 safe profile default is missing: {marker}")

    p3_launch_text = (
        src / "cs625_bringup" / "launch" / "sim_view_planning.launch.py"
    ).read_text(encoding="utf-8")
    for marker in (
        'DeclareLaunchArgument("execute", default_value="false")',
        'DeclareLaunchArgument("require_confirmation", default_value="true")',
        'package="cs625_view_generation"',
        'package="cs625_motion_adapter"',
    ):
        if marker not in p3_launch_text:
            fail(f"P3 launch safety/composition marker missing: {marker}")
    sim_controllers_text = (
        src / "cs625_bringup" / "config" / "sim_controllers.yaml"
    ).read_text(encoding="utf-8")
    for controller_marker in (
        "joint_state_broadcaster/JointStateBroadcaster",
        "joint_trajectory_controller/JointTrajectoryController",
        "shoulder_pan_joint",
        "wrist_3_joint",
    ):
        if controller_marker not in sim_controllers_text:
            fail(f"simulation controller profile is missing: {controller_marker}")

    common_config_text = (
        src / "cs625_bringup" / "config" / "common.yaml"
    ).read_text(encoding="utf-8")
    for normalized_topic in (
        "/sensors/camera/color/image",
        "/sensors/camera/depth/image",
        "/sensors/camera/depth/camera_info",
        "/sensors/camera/points",
        "/sensors/camera/status",
    ):
        if normalized_topic not in common_config_text:
            fail(f"normalized sensor topic missing from common config: {normalized_topic}")

    for required in (
        'DeclareLaunchArgument("execute", default_value="false"',
        'DeclareLaunchArgument("require_confirmation", default_value="true"',
        '"activate_joint_controller",\n            default_value="false"',
        '"robot_ip",\n            default_value=""',
        'default_value="eli_cs_robot_driver"',
        'default_value="elite_control.launch.py"',
        'default_value="eli_cs_robot_description"',
        'application_description_default',
        '"sensor_adapter.launch.py"',
    ):
        if required not in real_launch_text:
            fail(f"real profile safety/reuse contract is missing: {required}")

    adapter_setup = (
        src / "cs625_sensor_adapter" / "setup.py"
    ).read_text(encoding="utf-8")
    if "rgbd_sensor_adapter" not in adapter_setup:
        fail("sensor adapter console entry point is missing")
    adapter_source = (
        src / "cs625_sensor_adapter" / "cs625_sensor_adapter" / "point_cloud_relay.py"
    ).read_text(encoding="utf-8")
    if "self._publishers = {}" in adapter_source:
        fail("sensor adapter must not shadow rclpy.node.Node._publishers")
    if "self._stream_publishers" not in adapter_source:
        fail("sensor adapter stream publisher registry is missing")
    if "qos_profile_sensor_data" not in adapter_source:
        fail("sensor adapter must use sensor-data QoS for RGB-D streams")

    for script_name, verb in (("build.sh", "build"), ("test.sh", "test")):
        script_text = (ROOT / "scripts" / script_name).read_text(encoding="utf-8")
        expected_order = (
            'colcon \\\n'
            '  --log-base "${colcon_root}/log" \\\n'
            f"  {verb} \\\n"
        )
        if expected_order not in script_text:
            fail(
                f"{script_name} must place the global --log-base option before "
                f"the colcon {verb} subcommand"
            )

    forbidden_literals = ["/home/", "/root/", "192.168.", "10.0.0."]
    checker_path = pathlib.Path(__file__).resolve()
    for current_root, directories, filenames in os.walk(
        ROOT, topdown=True, onerror=lambda _error: None
    ):
        directories[:] = [
            directory
            for directory in directories
            if directory not in GENERATED_DIRS and directory != ".git"
        ]
        for filename in filenames:
            path = pathlib.Path(current_root) / filename
            # The checker necessarily contains the literals it is validating.
            if path.resolve() == checker_path:
                continue
            try:
                text = path.read_text(encoding="utf-8")
            except UnicodeDecodeError:
                continue
            for literal in forbidden_literals:
                if literal in text:
                    fail(
                        f"forbidden machine-specific literal {literal!r} in "
                        f"{path.relative_to(ROOT)}"
                    )

    for current_root, directories, filenames in os.walk(
        ROOT, topdown=True, onerror=lambda _error: None
    ):
        directories[:] = [
            directory
            for directory in directories
            if directory not in GENERATED_DIRS and directory != ".git"
        ]
        for filename in filenames:
            if not filename.endswith(".py"):
                continue
            path = pathlib.Path(current_root) / filename
            ast.parse(path.read_text(encoding="utf-8"), filename=str(path))

    print("Phase 0–6 static contract checks: PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
