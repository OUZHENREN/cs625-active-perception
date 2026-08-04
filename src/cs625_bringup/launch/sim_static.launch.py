"""Static CS625 world launch — the robot model lives in the SDF.

No dynamic spawning.  Gazebo loads everything at world-init time, avoiding
the render-thread vs controller-manager race present in sim_control.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world_default = PathJoinSubstitution(
        [FindPackageShare("cs625_simulation"), "worlds", "cs625_static.sdf"]
    )

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=world_default,
                              description="Static SDF world with embedded robot model."),
        DeclareLaunchArgument("headless", default_value="false",
                              description="Run Gazebo without GUI"),
        DeclareLaunchArgument("launch_rviz", default_value="false"),
        DeclareLaunchArgument("launch_sensor_adapter", default_value="true"),
        DeclareLaunchArgument("camera_enabled", default_value="true"),
        DeclareLaunchArgument("launch_moveit", default_value="true"),
        DeclareLaunchArgument("cs_type", default_value="cs625"),
        DeclareLaunchArgument("description_package", default_value="cs625_ap_description"),
        DeclareLaunchArgument("description_file",
                              default_value="cs625_active_perception.urdf.xacro"),

        # Gazebo server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            ),
            launch_arguments={
                "gz_args": [" -r -v 4 ", LaunchConfiguration("world")],
            }.items(),
        ),

        # Clock bridge
        Node(
            package="ros_gz_bridge", executable="parameter_bridge",
            name="clock_bridge", output="screen",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        ),

        # robot_state_publisher for TF
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            name="robot_state_publisher", output="screen",
            parameters=[{"use_sim_time": True, "robot_description": ""}],
        ),

        # Controller spawners — start after Gazebo
        RegisterEventHandler(
            OnProcessExit(
                target_action=None,  # run immediately
                on_exit=[
                    Node(
                        package="controller_manager", executable="spawner",
                        name="spawner_joint_state_broadcaster",
                        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
                        output="screen",
                    ),
                    Node(
                        package="controller_manager", executable="spawner",
                        name="spawner_joint_trajectory_controller",
                        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
                        output="screen",
                    ),
                ],
            )
        ),

        # Sensor bridge
        Node(
            package="ros_gz_bridge", executable="parameter_bridge",
            name="cs625_sim_sensor_bridge", output="screen",
            arguments=[
                "/camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
                "/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
                "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                "/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            ],
        ),

        # Sensor adapter
        Node(
            package="cs625_sensor_adapter", executable="point_cloud_relay",
            name="cs625_sensor_adapter", output="screen",
            parameters=[{
                "input_color_topic": "/camera/image",
                "input_depth_topic": "/camera/depth_image",
                "input_camera_info_topic": "/camera/camera_info",
                "input_points_topic": "/camera/points",
                "output_color_topic": "/sensors/camera/color/image",
                "output_depth_topic": "/sensors/camera/depth/image",
                "output_camera_info_topic": "/sensors/camera/depth/camera_info",
                "output_points_topic": "/sensors/camera/points",
                "output_status_topic": "/sensors/camera/status",
                "color_enabled": True,
                "depth_enabled": True,
                "camera_info_enabled": True,
                "points_enabled": True,
            }],
        ),
    ])
