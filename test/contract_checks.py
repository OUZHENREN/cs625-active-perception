#!/usr/bin/env python3
"""Host-side static checks for the Phase 0–1 repository contract."""

from __future__ import annotations

import ast
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
            "ament_python" if package_dir in ("cs625_sensor_adapter", "cs625_target_perception") else "ament_cmake"
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
    if "<render_engine>ogre2</render_engine>" not in sdf_text + fixture_sdf_text:
        fail("Fortress headless RGB-D worlds must use the EGL-capable Ogre2 renderer")

    launch_text = (
        ROOT / "src" / "cs625_bringup" / "launch" / "sim_base.launch.py"
    ).read_text(encoding="utf-8")
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
        '"-topic",\n            "robot_description"',
        'name="cs625_spawn_robot"',
        "RegisterEventHandler",
        "OnProcessExit",
        "target_action=spawn_robot",
        "target_action=joint_state_spawner",
        "--headless-rendering",
    ):
        if spawn_marker not in sim_control_text:
            fail(f"deterministic Gazebo control launch is missing: {spawn_marker}")
    if (
        '"-string"' in sim_control_text
        or '"-allow_renaming"' in sim_control_text
        or "cs625_spawn_retry" in launch_text
    ):
        fail("simulation control must use one topic-based spawn without a retry entity")

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
    for path in ROOT.rglob("*"):
        if (
            not path.is_file()
            or ".git" in path.parts
            or GENERATED_DIRS.intersection(path.parts)
        ):
            continue
        # The checker necessarily contains the literals it is validating.
        if path.resolve() == checker_path:
            continue
        try:
            text = path.read_text(encoding="utf-8")
        except UnicodeDecodeError:
            continue
        for literal in forbidden_literals:
            if literal in text:
                fail(f"forbidden machine-specific literal {literal!r} in {path.relative_to(ROOT)}")

    for path in ROOT.rglob("*.py"):
        if GENERATED_DIRS.intersection(path.parts):
            continue
        ast.parse(path.read_text(encoding="utf-8"), filename=str(path))

    print("Phase 0–2 static contract checks: PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
