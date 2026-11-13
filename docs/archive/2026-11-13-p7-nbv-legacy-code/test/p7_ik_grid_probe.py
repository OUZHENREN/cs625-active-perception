#!/usr/bin/env python3
"""Read-only MoveIt IK grid probe for selecting a reachable P7 fixture pose."""

from __future__ import annotations

import rclpy
from builtin_interfaces.msg import Duration
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK


def main() -> None:
    rclpy.init()
    node = rclpy.create_node("p7_ik_grid_probe")
    client = node.create_client(GetPositionIK, "/compute_ik")
    if not client.wait_for_service(timeout_sec=12.0):
        raise RuntimeError("/compute_ik unavailable")
    # The initial tool orientation published by the running model is used for
    # the first geometric reachability pass; no trajectory is sent.
    # Include the currently reported tool neighbourhood first; each request is
    # bounded so this remains a short, non-motion diagnostic.
    candidates = (
        (0.55, -0.35, 0.25), (0.70, -0.35, 0.30),
        (0.85, -0.30, 0.35), (1.00, -0.20, 0.40),
        (0.55, 0.00, 0.30), (0.70, 0.00, 0.40),
        (0.85, 0.20, 0.45), (1.10, 0.00, 0.55),
    )
    for x, y, z in candidates:
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name = "cs625_arm"
        ik.ik_link_name = "tool0"
        # P3/P4 candidate generation and the MoveIt profile use base_link as
        # the planning frame, even though Gazebo emits a separate world model.
        ik.pose_stamped.header.frame_id = "base_link"
        ik.pose_stamped.pose.position.x = x
        ik.pose_stamped.pose.position.y = y
        ik.pose_stamped.pose.position.z = z
        ik.pose_stamped.pose.orientation.x = 0.70710678
        ik.pose_stamped.pose.orientation.w = 0.70710678
        ik.avoid_collisions = True
        ik.timeout = Duration(nanosec=500_000_000)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
        response = future.result()
        code = None if response is None else response.error_code.val
        ok = code == MoveItErrorCodes.SUCCESS
        print(f"x={x:+.2f} y={y:+.2f} z={z:+.2f} {'IK_OK' if ok else f'NO_IK(code={code})'}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
