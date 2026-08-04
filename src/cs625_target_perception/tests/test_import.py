"""Verify cs625_target_perception imports and node creation."""
import rclpy


def test_import_and_create():
    rclpy.init(args=[])
    from cs625_target_perception.target_pose_relay import TargetPoseRelay
    assert TargetPoseRelay is not None
    node = TargetPoseRelay()
    assert node.has_parameter("input_target_pose_topic")
    assert node.has_parameter("output_target_pose_topic")
    node.destroy_node()
    rclpy.shutdown()
