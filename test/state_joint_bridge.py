#!/usr/bin/env python3
"""把真机 CS625State 的关节角转成 /joint_states，供 MoveIt 只规划使用。

只读：订阅 /cs625/raw_state，发布 /joint_states；不向机器人发送任何指令。
"""

from __future__ import annotations

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from cs625_state_monitor.msg import CS625State

ARM_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
GRIPPER_JOINTS = ["gripper_left_finger_joint", "gripper_right_finger_joint"]


class StateJointBridge(Node):
    def __init__(self) -> None:
        super().__init__("state_joint_bridge")
        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(CS625State, "/cs625/raw_state", self.on_state, 10)
        self.last_msg = None
        self.create_timer(0.05, self.publish)
        self.get_logger().info("state_joint_bridge: /cs625/raw_state -> /joint_states (read-only)")

    def on_state(self, msg: CS625State) -> None:
        self.last_msg = msg

    def publish(self) -> None:
        if self.last_msg is None:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        positions = [math.radians(v) for v in self.last_msg.actual_joint_deg] + [0.0, 0.0]
        msg.name = ARM_JOINTS + GRIPPER_JOINTS
        msg.position = positions
        self.publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = StateJointBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
