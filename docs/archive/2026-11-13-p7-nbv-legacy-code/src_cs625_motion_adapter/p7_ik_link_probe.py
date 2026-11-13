#!/usr/bin/env python3
"""Probe whether each declared CS625 end-effector link is accepted by MoveIt.

This is a non-motion diagnostic.  It looks up the live transform of a link and
submits that exact pose to ``/compute_ik``.  A failure therefore identifies a
planning-model or link-name mismatch rather than an unreachable target.
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionFK, GetPositionIK, GetStateValidity
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener


class IkLinkProbe(Node):
    def __init__(self) -> None:
        super().__init__("p7_ik_link_probe")
        self._tf = Buffer()
        self._listener = TransformListener(self._tf, self)
        self._client = self.create_client(GetPositionIK, "/compute_ik")
        self._fk_client = self.create_client(GetPositionFK, "/compute_fk")
        self._validity_client = self.create_client(GetStateValidity, "/check_state_validity")
        self._joints: dict[str, float] = {}
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)

    def _on_joint_state(self, message: JointState) -> None:
        self._joints.update(zip(message.name, message.position))

    def _state_names(self) -> list[str]:
        return [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
        ]

    def _apply_live_state(self, robot_state) -> None:
        robot_state.is_diff = True
        robot_state.joint_state.name = self._state_names()
        robot_state.joint_state.position = [self._joints[name] for name in self._state_names()]

    def current_state_validity(self) -> tuple[bool, list[str]]:
        if not self._validity_client.wait_for_service(timeout_sec=10.0):
            return (False, ["VALIDITY_SERVICE_UNAVAILABLE"])
        request = GetStateValidity.Request()
        request.group_name = "cs625_arm"
        self._apply_live_state(request.robot_state)
        future = self._validity_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done() or future.result() is None:
            return (False, ["VALIDITY_CALL_TIMEOUT"])
        response = future.result()
        contacts = [f"{entry.contact_body_1}<->{entry.contact_body_2}" for entry in response.contacts]
        return (response.valid, contacts)

    def probe(self, link: str, avoid_collisions: bool) -> tuple[str, int]:
        if not self._client.wait_for_service(timeout_sec=10.0) or not self._fk_client.wait_for_service(timeout_sec=10.0):
            return ("IK_SERVICE_UNAVAILABLE", MoveItErrorCodes.FAILURE)
        # Static transforms are delivered through subscriptions; let the node
        # executor receive them before a one-shot lookup.
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self._joints:
            return ("JOINT_STATES_UNAVAILABLE", MoveItErrorCodes.ROBOT_STATE_STALE)
        fk_request = GetPositionFK.Request()
        fk_request.header.frame_id = "world"
        fk_request.fk_link_names = [link]
        self._apply_live_state(fk_request.robot_state)
        fk_future = self._fk_client.call_async(fk_request)
        rclpy.spin_until_future_complete(self, fk_future, timeout_sec=10.0)
        if not fk_future.done() or fk_future.result() is None:
            return ("FK_CALL_TIMEOUT", MoveItErrorCodes.TIMED_OUT)
        fk_response = fk_future.result()
        if fk_response.error_code.val != MoveItErrorCodes.SUCCESS or not fk_response.pose_stamped:
            return ("FK_FAILED", fk_response.error_code.val)
        pose = fk_response.pose_stamped[0]
        request = GetPositionIK.Request()
        request.ik_request.group_name = "cs625_arm"
        request.ik_request.ik_link_name = link
        request.ik_request.pose_stamped = pose
        self._apply_live_state(request.ik_request.robot_state)
        request.ik_request.avoid_collisions = avoid_collisions
        request.ik_request.timeout = Duration(seconds=5.0).to_msg()
        future = self._client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done() or future.result() is None:
            return ("IK_CALL_TIMEOUT", MoveItErrorCodes.TIMED_OUT)
        return ("MOVEIT_RESPONSE", future.result().error_code.val)


def main() -> None:
    rclpy.init()
    node = IkLinkProbe()
    try:
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.1)
        valid, contacts = node.current_state_validity()
        print(f"IK_LINK_PROBE current_state_valid={valid} contacts={contacts}", flush=True)
        for link in ("my_end_effector_link", "tool0"):
            for avoid_collisions in (True, False):
                stage, code = node.probe(link, avoid_collisions)
                print(
                    f"IK_LINK_PROBE link={link} avoid_collisions={avoid_collisions} "
                    f"stage={stage} moveit_code={code}",
                    flush=True,
                )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
