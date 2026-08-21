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
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
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
            " ",
            "camera_image_width:=",
            LaunchConfiguration("camera_image_width"),
            " ",
            "camera_image_height:=",
            LaunchConfiguration("camera_image_height"),
            " ",
            "camera_update_rate:=",
            LaunchConfiguration("camera_update_rate"),
            " ",
            "camera_enabled:=",
            LaunchConfiguration("camera_enabled"),
        ]
    )
    gazebo_model_path = LaunchConfiguration("gazebo_model_file")
    # gz_ros2_control is configured by the robot model to read
    # ``robot_description`` from this node.  Publish the exact renderer-safe
    # Gazebo model prepared below, not the separate full MoveIt-oriented
    # expansion: that latter expansion can select mock_components hardware and
    # makes controller initialization order-dependent.
    gazebo_robot_description = {
        "robot_description": Command(
            [FindExecutable(name="cat"), " ", gazebo_model_path]
        )
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}, gazebo_robot_description],
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
            "-file",
            gazebo_model_path,
            "-name",
            "cs",
        ],
    )
    # Harmonic/Ogre2 on the target WSL renderer stalls while loading the
    # upstream DAE visual meshes into a camera render scene.  Gazebo receives
    # a visual-mesh-free but kinematically identical URDF; RSP and MoveIt keep
    # the complete official description above.  The generated file retains
    # collision, joints, ros2_control and the eye-in-hand RGB-D sensor.
    prepare_gazebo_model = ExecuteProcess(
        cmd=[
            FindExecutable(name="python3"),
            PathJoinSubstitution(
                [
                    FindPackageShare("cs625_bringup"),
                    "scripts",
                    "prepare_gazebo_model.py",
                ]
            ),
            "--output",
            gazebo_model_path,
            "--xacro",
            FindExecutable(name="xacro"),
            description_path,
            "name:=cs",
            "cs_type:=",
            LaunchConfiguration("cs_type"),
            "prefix:=",
            LaunchConfiguration("prefix"),
            "safety_limits:=",
            LaunchConfiguration("safety_limits"),
            "safety_pos_margin:=",
            LaunchConfiguration("safety_pos_margin"),
            "safety_k_position:=",
            LaunchConfiguration("safety_k_position"),
            "sim_ignition:=true",
            "simulation_controllers:=",
            controllers_path,
            "camera_image_width:=",
            LaunchConfiguration("camera_image_width"),
            "camera_image_height:=",
            LaunchConfiguration("camera_image_height"),
            "camera_update_rate:=",
            LaunchConfiguration("camera_update_rate"),
            "camera_enabled:=",
            LaunchConfiguration("camera_enabled"),
        ],
        output="screen",
    )
    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_joint_state_broadcaster",
        arguments=[
            "joint_state_broadcaster",
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            LaunchConfiguration("controller_manager_timeout"),
            "--switch-timeout",
            LaunchConfiguration("controller_switch_timeout"),
            "--service-call-timeout",
            LaunchConfiguration("controller_service_call_timeout"),
        ],
        output="screen",
    )
    trajectory_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_joint_trajectory_controller",
        arguments=[
            "joint_trajectory_controller",
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            LaunchConfiguration("controller_manager_timeout"),
            "--switch-timeout",
            LaunchConfiguration("controller_switch_timeout"),
            "--service-call-timeout",
            LaunchConfiguration("controller_service_call_timeout"),
        ],
        output="screen",
    )

    start_spawn_after_model_prepared = RegisterEventHandler(
        OnProcessExit(target_action=prepare_gazebo_model, on_exit=[spawn_robot])
    )
    # ``ros_gz_sim create`` returns as soon as the entity request is accepted.
    # In Harmonic on WSL, gz_ros2_control exposes the model only a few render
    # iterations later; spawning controllers immediately races that setup and
    # yields "No state interfaces found".  Give the control plugin a bounded
    # settling interval before querying its interfaces.
    start_joint_state_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[TimerAction(period=5.0, actions=[joint_state_spawner])],
        )
    )
    # Gazebo's control plugin can also subscribe to /robot_description.  The
    # full RSP description is deliberately suitable for MoveIt and may contain
    # a non-Gazebo mock hardware stanza, while the generated model passed to
    # ``ros_gz_sim create`` contains the required GazeboSimSystem stanza.  If
    # RSP publishes first, controller_manager nondeterministically initializes
    # the wrong hardware.  Start RSP only after the spawned model has had time
    # to initialize its own control plugin; later topic data is then ignored
    # by the already initialized resource manager.
    start_robot_state_publisher_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[TimerAction(period=7.0, actions=[robot_state_publisher])],
        )
    )
    start_trajectory_after_joint_state = RegisterEventHandler(
        OnProcessExit(target_action=joint_state_spawner, on_exit=[trajectory_spawner])
    )
    # Register handlers before their target processes can exit.
    return actions + [
        clock_bridge,
        gazebo,
        start_spawn_after_model_prepared,
        start_joint_state_after_spawn,
        start_robot_state_publisher_after_spawn,
        start_trajectory_after_joint_state,
        prepare_gazebo_model,
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
            DeclareLaunchArgument(
                "camera_image_width",
                default_value="320",
                description="Eye-in-hand RGB-D image width for the sim profile.",
            ),
            DeclareLaunchArgument(
                "camera_image_height",
                default_value="240",
                description="Eye-in-hand RGB-D image height for the sim profile.",
            ),
            DeclareLaunchArgument(
                "camera_update_rate",
                default_value="10.0",
                description="Eye-in-hand RGB-D update rate in Hz for the sim profile.",
            ),
            DeclareLaunchArgument(
                "camera_enabled",
                default_value="true",
                description="Instantiate the eye-in-hand RGB-D sensor in Gazebo.",
            ),
            DeclareLaunchArgument(
                "gazebo_model_file",
                default_value="/tmp/cs625_active_perception_gazebo_model.urdf",
                description="Generated visual-mesh-free URDF used only by Gazebo.",
            ),
            DeclareLaunchArgument(
                "controller_manager_timeout",
                default_value="60",
                description="Seconds to wait for the controller-manager services.",
            ),
            DeclareLaunchArgument(
                "controller_switch_timeout",
                default_value="60",
                description="Seconds to wait while Gazebo initializes the render thread.",
            ),
            DeclareLaunchArgument(
                "controller_service_call_timeout",
                default_value="60",
                description="Seconds to wait for a controller-manager service response.",
            ),
            OpaqueFunction(function=_compose),
        ]
    )
