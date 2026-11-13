#!/usr/bin/env python3
"""P2 smoke test for cs625_compliant_placement without a robot.

Publishes CS625State on /cs625/raw_state, drives the placement node through:
  start -> target_reached
  start(far) -> SOFT_APPROACH -> contact (CONTACT_HOLD) -> safety (ABORTED)
  start(far) -> stop(abort=True)
  get_state
No motion is commanded; the node only publishes state and tries to set a speed
slider service that does not exist here.
"""

import sys
import time

import rclpy
from rclpy.node import Node
from cs625_state_monitor.msg import CS625State
from cs625_compliant_placement.msg import CompliantPlacementState
from cs625_compliant_placement.srv import (
    StartCompliantPlacement,
    StopCompliantPlacement,
    GetCompliantPlacementState,
)


class Smoke(Node):
    def __init__(self):
        super().__init__("compliant_smoke_test")
        self.pub = self.create_publisher(CS625State, "/cs625/raw_state", 10)
        self.sub = self.create_subscription(
            CompliantPlacementState, "/cs625/compliant_placement/state", self.on_state, 10
        )
        self.latest = None
        self.tau = 0.0
        self.tcp = (0.0, 0.0, 0.0)
        self.msg = CS625State()
        self.msg.header.frame_id = "base_link"
        self.msg.parser_ok = True
        self.msg.has_joint_data = True
        self.msg.has_actual_tool_data = True
        self.timer = self.create_timer(0.02, self.publish)
        self.start_cli = self.create_client(StartCompliantPlacement, "start_compliant_placement")
        self.stop_cli = self.create_client(StopCompliantPlacement, "stop_compliant_placement")
        self.get_cli = self.create_client(
            GetCompliantPlacementState, "get_compliant_placement_state"
        )

    def publish(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.joint_torques = [self.tau] * 6
        self.msg.actual_tcp_x_mm = self.tcp[0]
        self.msg.actual_tcp_y_mm = self.tcp[1]
        self.msg.actual_tcp_z_mm = self.tcp[2]
        self.pub.publish(self.msg)

    def on_state(self, msg):
        self.latest = msg


def spin_for(node, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.01)


def wait_for(node, predicate, timeout, label):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.01)
        if predicate():
            return True
    print(f"FAIL {label}: timeout")
    return False


def call(node, client, request, timeout=5.0):
    if not client.wait_for_service(timeout_sec=timeout):
        raise RuntimeError(f"service {client.srv_name} not available")
    future = client.call_async(request)
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.01)
        if future.done():
            return future.result()
    raise RuntimeError(f"service {client.srv_name} timed out")


def main():
    rclpy.init()
    node = Smoke()
    checks = []

    if not wait_for(node, lambda: node.latest is not None, 5.0, "initial state"):
        checks.append(("initial_state", False))
    else:
        checks.append(("initial_state", True))

    req = StartCompliantPlacement.Request()
    req.target_tcp_x_mm = 0.0
    req.target_tcp_y_mm = 0.0
    req.target_tcp_z_mm = 0.0
    req.target_tolerance_mm = 3.0
    req.use_force_contact = True
    req.use_jam_detection = True
    resp = call(node, node.start_cli, req)
    checks.append(("start_accepted", bool(resp.success)))

    ok = wait_for(
        node,
        lambda: node.latest is not None and not node.latest.active
        and node.latest.stop_reason == "target_reached",
        3.0,
        "target_reached",
    )
    checks.append(("target_reached", ok))

    req.target_tcp_x_mm = 50.0
    resp = call(node, node.start_cli, req)
    ok = resp.success and wait_for(
        node,
        lambda: node.latest is not None and node.latest.active
        and node.latest.state == "SOFT_APPROACH",
        3.0,
        "soft_approach",
    )
    checks.append(("soft_approach", ok))

    node.tau = 0.33  # sum~2.0 -> residual window 0.8..2.0 (contact, not safety)
    ok = wait_for(
        node,
        lambda: node.latest is not None and node.latest.contact_detected
        and node.latest.state == "CONTACT_HOLD",
        3.0,
        "contact_hold",
    )
    checks.append(("contact_hold", ok))

    node.tau = 100.0
    ok = wait_for(
        node,
        lambda: node.latest is not None and not node.latest.active
        and node.latest.stop_reason == "safety_triggered",
        3.0,
        "safety_abort",
    )
    checks.append(("safety_abort", ok))

    node.tau = 0.0
    req.target_tcp_x_mm = 100.0
    call(node, node.start_cli, req)
    stop = StopCompliantPlacement.Request()
    stop.abort = True
    resp = call(node, node.stop_cli, stop)
    ok = resp.success and wait_for(
        node,
        lambda: node.latest is not None and not node.latest.active
        and node.latest.stop_reason == "aborted_by_request",
        3.0,
        "stop_abort",
    )
    checks.append(("stop_abort", ok))

    resp = call(node, node.get_cli, GetCompliantPlacementState.Request())
    checks.append(("get_state", bool(resp.success) and resp.state.state == "IDLE"))

    print("\n=== P2 smoke result ===")
    failed = 0
    for name, ok in checks:
        print(f"{'PASS' if ok else 'FAIL'}  {name}")
        failed += 0 if ok else 1
    print(f"total={len(checks)} failed={failed}")
    rclpy.shutdown()
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
