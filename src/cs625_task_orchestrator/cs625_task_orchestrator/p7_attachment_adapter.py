"""Bridge auditable P7 attachment commands to Gazebo DetachableJoint.

This is intentionally an *attachment-assisted* mechanism. A successful
status proves Gazebo's fixed detachable joint changed state; it does not prove
finger contact, frictional force closure, or a mesh-level collision-free grasp.
"""

import json
import os
import subprocess
import time
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


def attachment_status_payload(command_id: str, action: str, attached: bool | None, elapsed_sec: float, detail: str = "") -> dict[str, Any]:
    """Create a schema-stable result without inferring unavailable evidence."""
    expected = action == "attach"
    success = attached is expected
    if attached is None:
        code = "ATTACHMENT_STATE_UNOBSERVED"
    elif success:
        code = "ATTACHMENT_ATTACHED" if expected else "ATTACHMENT_DETACHED"
    else:
        code = "ATTACHMENT_STATE_MISMATCH"
    return {
        "command_id": command_id,
        "action": action,
        "attached": attached,
        "success": success,
        "code": code,
        "elapsed_sec": round(elapsed_sec, 6),
        "mechanism": "gazebo_detachable_joint_attachment_assisted",
        "detail": detail,
    }


def parse_gz_attachment_state(text: str) -> bool | None:
    """Parse the official DetachableJoint ``gz.msgs.StringMsg`` state."""
    lowered = text.lower()
    if 'data: "attached"' in lowered:
        return True
    if 'data: "detached"' in lowered:
        return False
    return None


class P7AttachmentAdapter(Node):
    def __init__(self) -> None:
        super().__init__("cs625_p7_attachment_adapter")
        for name, value in (
            ("command_topic", "/p7/attachment_command"),
            ("status_topic", "/p7/attachment_status"),
            ("attach_topic", "/p7/attachment/attach"),
            ("detach_topic", "/p7/attachment/detach"),
            ("state_topic", "/p7/attachment/state"),
            ("monitor_timeout_sec", 4.0),
        ):
            self.declare_parameter(name, value)
        partition = os.environ.get("CS625_GZ_PARTITION", "")
        if partition and not os.environ.get("GZ_PARTITION"):
            os.environ["GZ_PARTITION"] = partition
            os.environ.setdefault("IGN_PARTITION", partition)
        self._attach_topic = str(self.get_parameter("attach_topic").value)
        self._detach_topic = str(self.get_parameter("detach_topic").value)
        self._state_topic = str(self.get_parameter("state_topic").value)
        self._timeout = float(self.get_parameter("monitor_timeout_sec").value)
        self._status_pub = self.create_publisher(String, str(self.get_parameter("status_topic").value), 10)
        self._command_sub = self.create_subscription(String, str(self.get_parameter("command_topic").value), self._on_command, 10)

    def _publish(self, payload: dict[str, Any]) -> None:
        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        self._status_pub.publish(message)

    def _on_command(self, message: String) -> None:
        started = time.monotonic()
        try:
            command = json.loads(message.data)
            command_id = str(command["command_id"])
            action = str(command["action"])
            if action not in {"attach", "detach"}:
                raise ValueError("action must be attach or detach")
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
            self._publish({"success": False, "code": "ATTACHMENT_COMMAND_INVALID", "detail": str(exc), "mechanism": "gazebo_detachable_joint_attachment_assisted"})
            return

        monitor = None
        try:
            monitor = subprocess.Popen(["gz", "topic", "-e", "-t", self._state_topic, "-n", "1"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            # WSL's first Gazebo-transport subscriber handshake can take
            # hundreds of milliseconds.  Wait long enough to avoid treating
            # a genuine state transition as unobserved evidence.
            time.sleep(1.0)
            topic = self._attach_topic if action == "attach" else self._detach_topic
            subprocess.run(["gz", "topic", "-t", topic, "-m", "gz.msgs.Empty", "-p", "unused: true"], check=True, capture_output=True, text=True, timeout=self._timeout)
            stdout, stderr = monitor.communicate(timeout=self._timeout)
            payload = attachment_status_payload(command_id, action, parse_gz_attachment_state(stdout), time.monotonic() - started, stderr.strip())
        except FileNotFoundError:
            payload = attachment_status_payload(command_id, action, None, time.monotonic() - started, "gz executable unavailable")
            payload["code"] = "ATTACHMENT_GZ_UNAVAILABLE"
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as exc:
            if monitor is not None:
                monitor.kill()
            payload = attachment_status_payload(command_id, action, None, time.monotonic() - started, str(exc))
        finally:
            if monitor is not None and monitor.poll() is None:
                monitor.kill()
        self._publish(payload)


def main() -> None:
    rclpy.init()
    node = P7AttachmentAdapter()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
