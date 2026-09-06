"""Publish one deterministic attachment command for a live Jazzy smoke run."""

import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main() -> None:
    action = sys.argv[1] if len(sys.argv) > 1 else "detach"
    if action not in {"attach", "detach"}:
        raise SystemExit("usage: p7_attachment_smoke_publisher.py [attach|detach]")
    rclpy.init()
    node = Node("cs625_p7_attachment_smoke_publisher")
    publisher = node.create_publisher(String, "/p7/attachment_command", 10)
    message = String()
    message.data = json.dumps({
        "command_id": f"p7-attachment-{action}-smoke",
        "action": action,
    })
    # Wait for discovery before publishing exactly one state transition;
    # repeated detach commands would not yield a new Gazebo state event.
    time.sleep(0.8)
    publisher.publish(message)
    rclpy.spin_once(node, timeout_sec=0.25)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
