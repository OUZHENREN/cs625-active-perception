#!/usr/bin/env python3
"""到位复核：当前关节状态是否无接触。退出码 0=安全，4=有接触。"""
import sys

import rclpy
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity
from rclpy.node import Node
from sensor_msgs.msg import JointState


def main() -> int:
    rclpy.init()
    node = rclpy.create_node("check_state_ok")
    state: dict = {}
    node.create_subscription(JointState, "/joint_states",
                             lambda msg: state.setdefault("j", msg), 10)
    import time
    deadline = time.time() + 5
    while time.time() < deadline and "j" not in state:
        rclpy.spin_once(node, timeout_sec=0.2)
    if "j" not in state:
        print("读不到关节状态")
        return 4
    client = node.create_client(GetStateValidity, "/check_state_validity")
    if not client.wait_for_service(timeout_sec=5):
        print("碰撞服务不可用")
        return 4
    request = GetStateValidity.Request()
    robot_state = RobotState()
    robot_state.joint_state.name = list(state["j"].name)
    robot_state.joint_state.position = list(state["j"].position)
    request.robot_state = robot_state
    request.group_name = "cs625_arm"
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10)
    result = future.result()
    if result is None:
        print("判定失败")
        return 4
    if result.valid:
        print("到位复核：无接触 ✅")
        return 0
    pairs = [(c.contact_body_1, c.contact_body_2) for c in result.contacts[:3]]
    print(f"到位复核：存在接触 {pairs} ❌")
    return 4


if __name__ == "__main__":
    raise SystemExit(main())
