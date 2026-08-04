"""Bridge the senior Gazebo RGB-D topics into the application namespace."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_default = PathJoinSubstitution(
        [FindPackageShare("cs625_bringup"), "config", "sim_gz_bridge.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bridge_config",
                default_value=config_default,
                description="Gazebo RGB-D bridge YAML.",
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="cs625_sim_sensor_bridge",
                output="screen",
                parameters=[
                    {"config_file": LaunchConfiguration("bridge_config")}
                ],
            ),
        ]
    )
