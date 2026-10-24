#!/usr/bin/env python3
"""ROS 2 service wrapper for the HITBOT EFG80 gripper.

The low-level Windows bridge owns COM9 and exposes a line-oriented TCP API
(``status`` / ``open`` / ``close``).  This node only forwards those commands as
ROS services and publishes the latest status as JSON.  It never commands the
CS625 arm.

Services:
  /cs625/gripper/status  std_srvs/Trigger  -> message contains JSON status
  /cs625/gripper/open    std_srvs/Trigger  -> open to open_mm
  /cs625/gripper/close   std_srvs/Trigger  -> close to close_mm
Topic:
  /cs625/gripper/status  std_msgs/String   -> JSON, refreshed periodically

Run:
  source scripts/source_dev_env.sh --full
  ros2 run cs625_motion_adapter gripper_bridge
"""

from __future__ import annotations

import json
import socket

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class GripperBridge(Node):
    def __init__(self) -> None:
        super().__init__("cs625_gripper_bridge")
        self.declare_parameter("bridge_host", "127.0.0.1")
        self.declare_parameter("bridge_port", 7500)
        self.declare_parameter("open_mm", 50.0)
        self.declare_parameter("close_mm", 0.0)
        self.declare_parameter("poll_period_sec", 1.0)

        self._host = str(self.get_parameter("bridge_host").value)
        self._port = int(self.get_parameter("bridge_port").value)
        self._open_mm = float(self.get_parameter("open_mm").value)
        self._close_mm = float(self.get_parameter("close_mm").value)
        self._period = max(0.2, float(self.get_parameter("poll_period_sec").value))

        self._publisher = self.create_publisher(String, "/cs625/gripper/status", 10)
        self._status_srv = self.create_service(
            Trigger, "/cs625/gripper/status", self._handle_status
        )
        self._open_srv = self.create_service(
            Trigger, "/cs625/gripper/open", self._handle_open
        )
        self._close_srv = self.create_service(
            Trigger, "/cs625/gripper/close", self._handle_close
        )
        self._timer = self.create_timer(self._period, self._poll)
        self.get_logger().info(
            "gripper bridge services ready: /cs625/gripper/{status,open,close}; "
            f"transport={self._host}:{self._port}"
        )

    # ------------------------------------------------------------------ bridge
    def _call_bridge(self, command: str) -> dict:
        with socket.create_connection((self._host, self._port), timeout=1.0) as sock:
            sock.sendall((command + "\n").encode("ascii"))
            raw = sock.recv(4096).decode("utf-8", "replace").strip()
        if not raw:
            raise RuntimeError("empty bridge response")
        return json.loads(raw)

    def _command(self, command: str) -> tuple[bool, str]:
        try:
            payload = self._call_bridge(command)
            ok = bool(payload.get("ok"))
            return ok, json.dumps(payload, ensure_ascii=False)
        except Exception as exc:  # noqa: BLE001 - surfaced through the ROS service
            return False, f"{type(exc).__name__}: {exc}"

    # ----------------------------------------------------------------- services
    def _handle_status(self, _request, response: Trigger.Response) -> Trigger.Response:
        response.success, response.message = self._command("status")
        return response

    def _handle_open(self, _request, response: Trigger.Response) -> Trigger.Response:
        response.success, response.message = self._command("open")
        if response.success:
            response.message = (
                f"open request accepted (target {self._open_mm:g} mm): {response.message}"
            )
        return response

    def _handle_close(self, _request, response: Trigger.Response) -> Trigger.Response:
        response.success, response.message = self._command("close")
        if response.success:
            response.message = (
                f"close request accepted (target {self._close_mm:g} mm): {response.message}"
            )
        return response

    # -------------------------------------------------------------------- timer
    def _poll(self) -> None:
        ok, message = self._command("status")
        payload = message
        if not ok:
            self.get_logger().warning(f"gripper status unavailable: {message}")
        self._publisher.publish(String(data=payload))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GripperBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
