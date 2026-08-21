"""P2 pipeline: camera + ground-truth + target perception + optional rosbag."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    record_bag = LaunchConfiguration("record_bag")
    return LaunchDescription([
        DeclareLaunchArgument("record_bag", default_value="false",
                              description="Record rosbag of P2 topics."),
        DeclareLaunchArgument("bag_path", default_value="/tmp/p2_bag",
                              description="rosbag output directory."),
        DeclareLaunchArgument("launch_moveit", default_value="false",
                              description="Launch MoveIt alongside the pipeline."),
        # Camera + adapter chain (from sim_base)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare("cs625_bringup"), "launch", "sim_base.launch.py"]
            ),
            launch_arguments={
                "launch_fixture_world": "true",
                "launch_official_sim": "false",
                "headless": "false",
                "launch_sensor_adapter": "true",
                "camera_enabled": "true",
                "launch_moveit": LaunchConfiguration("launch_moveit"),
            }.items(),
        ),
        # Ground-truth pose bridge
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare("cs625_bringup"), "launch",
                 "sim_ground_truth.launch.py"]
            ),
        ),
        # Target perception relay
        Node(
            package="cs625_target_perception",
            executable="target_pose_relay",
            name="cs625_target_perception",
            output="screen",
            parameters=[{
                "input_target_pose_topic": "/sim/target_pose_raw",
                "output_target_pose_topic": "/perception/target_pose",
                # Preserve the Gazebo/world frame of the source pose.  A
                # target_frame TF is published only by a real estimator that
                # owns that coordinate transform.
                "target_frame_id": "",
            }],
        ),
    ])
