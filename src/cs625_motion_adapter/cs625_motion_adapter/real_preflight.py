"""P6 real-profile readiness gate with no trajectory client or command path."""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class RealPreflight(Node):
    """Publish an auditable R0 safety result and refuse real execution.

    R1--R4 require an actual verified robot, camera and human safety review;
    this node never creates an action client and cannot command hardware.
    """

    def __init__(self) -> None:
        super().__init__("cs625_real_preflight")
        for name, value in (
            ("profile", "real"), ("execute", False), ("require_confirmation", True),
            ("robot_ip", ""), ("camera_enabled", True),
            ("max_velocity_scale", 0.0), ("max_acceleration_scale", 0.0),
            ("workspace_min", rclpy.Parameter.Type.DOUBLE_ARRAY),
            ("workspace_max", rclpy.Parameter.Type.DOUBLE_ARRAY),
            ("status_topic", "/motion/status"),
            ("vision_target_topic", ""),
            ("grasp_task_state_topic", ""),
            ("grasp_handoff_enabled", False),
            ("exit_after_publish", False),
        ):
            self.declare_parameter(name, value)
        self._status = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._publish_once()
        if bool(self.get_parameter("exit_after_publish").value):
            self._exit_timer = self.create_timer(0.25, self._finish)

    def _finish(self) -> None:
        self._exit_timer.cancel()
        self.get_logger().info("P6 preflight completed without a hardware command")
        rclpy.shutdown()

    def _publish_once(self) -> None:
        profile = str(self.get_parameter("profile").value).strip()
        execute = bool(self.get_parameter("execute").value)
        confirmation = bool(self.get_parameter("require_confirmation").value)
        velocity = float(self.get_parameter("max_velocity_scale").value)
        acceleration = float(self.get_parameter("max_acceleration_scale").value)
        workspace_min = list(self.get_parameter("workspace_min").value)
        workspace_max = list(self.get_parameter("workspace_max").value)
        vision_target_topic = str(self.get_parameter("vision_target_topic").value).strip()
        grasp_task_state_topic = str(self.get_parameter("grasp_task_state_topic").value).strip()
        grasp_handoff_enabled = bool(self.get_parameter("grasp_handoff_enabled").value)
        valid_workspace = (
            len(workspace_min) == 3
            and len(workspace_max) == 3
            and all(low < high for low, high in zip(workspace_min, workspace_max))
        )
        result = {
            "profile": profile,
            "real_motion_occurred": False,
            "r0_profile_safe": profile == "real" and not execute and confirmation and valid_workspace,
            "r1_driver_connection": "NOT_VERIFIED",
            "r2_camera_tf": "NOT_VERIFIED",
            "r3_moveit_planning_only": "NOT_VERIFIED",
            "r4_low_speed_execution": "BLOCKED_BY_EXTERNAL_HARDWARE_AND_HUMAN_REVIEW",
            "velocity_scale": velocity,
            "acceleration_scale": acceleration,
            "vision_target_topic": vision_target_topic,
            "grasp_task_state_topic": grasp_task_state_topic,
            "grasp_handoff_enabled": grasp_handoff_enabled,
        }
        if profile != "real":
            result["code"] = "PROFILE_MISMATCH"
        elif execute:
            result["code"] = "REAL_EXECUTION_REFUSED"
        elif not confirmation:
            result["code"] = "REAL_CONFIRMATION_GATE_INVALID"
        elif not valid_workspace:
            result["code"] = "WORKSPACE_PROFILE_INVALID"
        elif velocity <= 0.0 or velocity > 1.0 or acceleration <= 0.0 or acceleration > 1.0:
            result["code"] = "SCALING_PROFILE_INVALID"
        elif grasp_handoff_enabled and (not vision_target_topic or not grasp_task_state_topic):
            result["code"] = "GRASP_HANDOFF_PROFILE_INVALID"
        else:
            result["code"] = "R0_PROFILE_SAFE"
        message = String()
        message.data = json.dumps(result, sort_keys=True)
        self._status.publish(message)
        self.get_logger().info(message.data)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RealPreflight()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
