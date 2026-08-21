"""Parameter-driven target pose relay following the sensor adapter pattern."""
from __future__ import annotations
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class TargetPoseRelay(Node):
    """Relay a target pose from input topic to normalized output topic."""

    def __init__(self) -> None:
        super().__init__("target_pose_relay")
        self.declare_parameter("input_target_pose_topic", "")
        self.declare_parameter("output_target_pose_topic", "/perception/target_pose")
        self.declare_parameter("target_frame_id", "")
        self.declare_parameter("drop_invalid_messages", True)

        in_topic = str(self.get_parameter("input_target_pose_topic").value)
        out_topic = str(self.get_parameter("output_target_pose_topic").value)
        self._target_frame_id = str(self.get_parameter("target_frame_id").value)
        self._drop_invalid = bool(self.get_parameter("drop_invalid_messages").value)

        if not in_topic:
            self.get_logger().error("input_target_pose_topic is required")
            return

        self._pub = self.create_publisher(PoseStamped, out_topic, 10)
        self._sub = self.create_subscription(
            PoseStamped, in_topic, self._relay, 10
        )
        self._received_count = 0
        self.get_logger().info(f"Relaying {in_topic} -> {out_topic}")

    def _relay(self, msg: PoseStamped) -> None:
        if self._drop_invalid and (
            msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0
        ):
            self.get_logger().warning("Dropping target pose with zero timestamp")
            return
        # A PoseStamped must retain the frame in which its position is
        # expressed.  ``target_frame`` is an object-attached TF frame, not a
        # coordinate frame for world/Gazebo pose coordinates.  Profiles may
        # override this only when they also provide a corresponding transform.
        if self._target_frame_id:
            msg.header.frame_id = self._target_frame_id
        self._pub.publish(msg)
        self._received_count += 1


def main(args=None):
    rclpy.init(args=args)
    node = TargetPoseRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
