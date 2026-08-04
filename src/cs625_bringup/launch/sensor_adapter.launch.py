"""Common normalized RGB-D adapter composition for sim and real profiles."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def _compose(context):
    parameters = [
        ParameterFile(LaunchConfiguration("common_config").perform(context), allow_substs=True),
        ParameterFile(LaunchConfiguration("profile_config").perform(context), allow_substs=True),
    ]
    topic_parameters = {}
    for name in (
        "input_color_topic",
        "input_depth_topic",
        "input_camera_info_topic",
        "input_points_topic",
    ):
        value = LaunchConfiguration(name).perform(context).strip()
        if value:
            topic_parameters[name] = value
    if topic_parameters:
        parameters.append(topic_parameters)

    return [
        Node(
            package="cs625_sensor_adapter",
            executable="rgbd_sensor_adapter",
            name="cs625_sensor_adapter",
            output="screen",
            parameters=parameters,
        )
    ]


def generate_launch_description():
    common_config_default = PathJoinSubstitution(
        [FindPackageShare("cs625_bringup"), "config", "common.yaml"]
    )

    arguments = [
        DeclareLaunchArgument(
            "common_config",
            default_value=common_config_default,
            description="Shared normalized sensor contract.",
        ),
        DeclareLaunchArgument(
            "profile_config",
            description="Sim or real sensor profile configuration.",
        ),
        DeclareLaunchArgument("input_color_topic", default_value=""),
        DeclareLaunchArgument("input_depth_topic", default_value=""),
        DeclareLaunchArgument("input_camera_info_topic", default_value=""),
        DeclareLaunchArgument("input_points_topic", default_value=""),
    ]

    return LaunchDescription(arguments + [OpaqueFunction(function=_compose)])
