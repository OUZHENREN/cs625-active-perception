"""Bridge Gazebo dynamic poses for P2 target perception."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world_name", default_value="rgbd_fixture",
                              description="Gazebo world name for pose bridging."),
        DeclareLaunchArgument("target_model", default_value="target_object",
                              description="Model name whose pose to track."),
        DeclareLaunchArgument("output_topic", default_value="/sim/target_pose_raw",
                              description="Raw target pose topic from Gazebo."),
        Node(
            package="ros_gz_bridge", executable="parameter_bridge",
            name="target_pose_bridge", output="screen",
            arguments=[
                "/world/rgbd_fixture/dynamic_pose/info@geometry_msgs/msg/PoseStamped[gz.msgs.Pose_V",
            ],
            remappings=[
                ("/world/rgbd_fixture/dynamic_pose/info", "/sim/target_pose_raw"),
            ],
        ),
    ])
