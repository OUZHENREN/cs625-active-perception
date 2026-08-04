"""CS625 Gazebo control chain with deterministic model spawn ordering.

The application xacro continues to wrap the senior CS625 description.  This
launch owns only the simulation process ordering that the application needs:
Gazebo and robot_state_publisher start first, the robot is created once from
the transient-local robot_description topic, and controllers are spawned only
after entity creation succeeds.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _compose(context):
    description_path = PathJoinSubstitution(
        [
            FindPackageShare(LaunchConfiguration("description_package")),
            "urdf",
            LaunchConfiguration("description_file"),
        ]
    )
    controllers_path = PathJoinSubstitution(
        [
            FindPackageShare(LaunchConfiguration("runtime_config_package")),
            "config",
            LaunchConfiguration("controllers_file"),
        ]
    )
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            description_path,
            " ",
            "name:=cs",
            " ",
            "cs_type:=",
            LaunchConfiguration("cs_type"),
            " ",
            "prefix:=",
            LaunchConfiguration("prefix"),
            " ",
            "safety_limits:=",
            LaunchConfiguration("safety_limits"),
            " ",
            "safety_pos_margin:=",
            LaunchConfiguration("safety_pos_margin"),
            " ",
            "safety_k_position:=",
            LaunchConfiguration("safety_k_position"),
            " ",
            "sim_ignition:=true",
            " ",
            "simulation_controllers:=",
            controllers_path,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    headless = LaunchConfiguration("headless").perform(context).lower()
    server_only = headless in ("1", "true", "yes", "on")
    # Standard Fortress EGL headless rendering with Ogre2.  On GPU-less VMs the
    # Ogre-Next 2.2.5 EGL PBuffer path may select /dev/dri/card0 and stall; the
    # VMware workaround (Xvfb + ogre1) is documented in WORKLOG_2026-08-04_RGBD_RENDER_SOLUTION.md.
    gz_flags = (
        " -s -r -v 4 --headless-rendering "
        if server_only
        else " -r -v 4 "
    )
    actions = []
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={
            "gz_args": [gz_flags, LaunchConfiguration("world_file")]
        }.items(),
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="cs625_spawn_robot",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "cs",
        ],
    )
    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_joint_state_broadcaster",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )
    trajectory_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_joint_trajectory_controller",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
        output="screen",
    )

    start_joint_state_after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot, on_exit=[joint_state_spawner])
    )
    start_trajectory_after_joint_state = RegisterEventHandler(
        OnProcessExit(target_action=joint_state_spawner, on_exit=[trajectory_spawner])
    )

    # Register handlers before their target processes can exit.
    return actions + [
        robot_state_publisher,
        clock_bridge,
        gazebo,
        start_joint_state_after_spawn,
        start_trajectory_after_joint_state,
        spawn_robot,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("cs_type", default_value="cs625"),
            DeclareLaunchArgument("prefix", default_value=""),
            DeclareLaunchArgument("safety_limits", default_value="true"),
            DeclareLaunchArgument("safety_pos_margin", default_value="0.15"),
            DeclareLaunchArgument("safety_k_position", default_value="20"),
            DeclareLaunchArgument("runtime_config_package", default_value="cs625_bringup"),
            DeclareLaunchArgument("controllers_file", default_value="sim_controllers.yaml"),
            DeclareLaunchArgument("description_package", default_value="cs625_ap_description"),
            DeclareLaunchArgument("description_file", default_value="cs625_active_perception.urdf.xacro"),
            DeclareLaunchArgument("world_file", default_value="empty.sdf"),
            DeclareLaunchArgument("headless", default_value="true"),
            OpaqueFunction(function=_compose),
        ]
    )
