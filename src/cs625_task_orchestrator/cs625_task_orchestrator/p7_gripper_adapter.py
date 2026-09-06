"""Simulation gripper command adapter with explicit joint-state acknowledgement."""

from __future__ import annotations

import json
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String


def gripper_status_payload(
    command_id: str,
    success: bool,
    code: str,
    target_position: float,
    elapsed_sec: float,
    observed_positions: dict[str, float] | None = None,
    tolerance_m: float | None = None,
) -> dict:
    payload = {
        "command_id": command_id,
        "success": success,
        "code": code,
        "target_position": target_position,
        "elapsed_sec": max(0.0, elapsed_sec),
    }
    if observed_positions:
        errors = {
            name: abs(position - target_position)
            for name, position in observed_positions.items()
        }
        payload.update({
            "observed_positions_m": observed_positions,
            "terminal_errors_m": errors,
            "max_terminal_error_m": max(errors.values()),
        })
        if tolerance_m is not None:
            payload["target_tolerance_m"] = tolerance_m
    return payload


class P7GripperAdapter(Node):
    """Forward a guarded P7 command and wait for the simulated joint state."""

    def __init__(self) -> None:
        super().__init__("cs625_p7_gripper_adapter")
        for name, value in (("command_topic", "/p7/gripper_command"), ("status_topic", "/p7/gripper_status"), ("controller_command_topic", "/gripper_controller/commands"), ("joint_states_topic", "/joint_states"), ("joint_name", "gripper_right_finger_joint"), ("peer_joint_name", "gripper_left_finger_joint"), ("target_tolerance_m", 0.002), ("timeout_sec", 5.0), ("minimum_position_m", 0.0), ("maximum_position_m", 0.040)):
            self.declare_parameter(name, value)
        self._joint_name = str(self.get_parameter("joint_name").value)
        self._peer_joint_name = str(self.get_parameter("peer_joint_name").value)
        self._joint_names = (self._joint_name, self._peer_joint_name)
        self._tolerance = float(self.get_parameter("target_tolerance_m").value)
        self._timeout = float(self.get_parameter("timeout_sec").value)
        self._minimum = float(self.get_parameter("minimum_position_m").value)
        self._maximum = float(self.get_parameter("maximum_position_m").value)
        if not (0.0 < self._tolerance <= 0.01 and 0.0 < self._timeout <= 30.0):
            raise ValueError("invalid gripper tolerance or timeout")
        self._controller_pub = self.create_publisher(Float64MultiArray, str(self.get_parameter("controller_command_topic").value), 10)
        self._status_pub = self.create_publisher(String, str(self.get_parameter("status_topic").value), 10)
        self.create_subscription(String, str(self.get_parameter("command_topic").value), self._on_command, 10)
        self.create_subscription(JointState, str(self.get_parameter("joint_states_topic").value), self._on_joint_state, 10)
        self._active: dict | None = None
        self._latest_positions: dict[str, float] = {}
        self.create_timer(0.05, self._on_timer)

    def _on_command(self, message: String) -> None:
        if self._active is not None:
            return
        try:
            command = json.loads(message.data)
        except json.JSONDecodeError:
            self._publish("", False, "GRIPPER_COMMAND_INVALID", 0.0, 0.0)
            return
        command_id, target = str(command.get("command_id", "")).strip(), command.get("target_position_m")
        if not command_id or isinstance(target, bool) or not isinstance(target, (int, float)):
            self._publish(command_id, False, "GRIPPER_COMMAND_INVALID", 0.0, 0.0)
            return
        target = float(target)
        if not math.isfinite(target) or target < self._minimum or target > self._maximum:
            self._publish(command_id, False, "GRIPPER_COMMAND_OUT_OF_RANGE", target, 0.0)
            return
        outgoing = Float64MultiArray()
        outgoing.data = [target]
        self._controller_pub.publish(outgoing)
        self._active = {"command_id": command_id, "target": target, "started": time.monotonic()}

    def _on_joint_state(self, message: JointState) -> None:
        positions = dict(zip(message.name, message.position))
        for name in self._joint_names:
            if name in positions:
                self._latest_positions[name] = float(positions[name])
        if self._active is None or any(
            name not in self._latest_positions for name in self._joint_names
        ):
            return
        if max(
            abs(self._latest_positions[name] - self._active["target"])
            for name in self._joint_names
        ) <= self._tolerance:
            self._finish(True, "GRIPPER_REACHED")

    def _on_timer(self) -> None:
        if self._active is not None and time.monotonic() - self._active["started"] > self._timeout:
            self._finish(False, "GRIPPER_TIMEOUT")

    def _finish(self, success: bool, code: str) -> None:
        assert self._active is not None
        self._publish(
            self._active["command_id"], success, code, self._active["target"],
            time.monotonic() - self._active["started"],
        )
        self._active = None

    def _publish(self, command_id: str, success: bool, code: str, target: float, elapsed: float) -> None:
        message = String()
        observed = {
            name: self._latest_positions[name]
            for name in self._joint_names
            if name in self._latest_positions
        }
        message.data = json.dumps(
            gripper_status_payload(
                command_id, success, code, target, elapsed, observed, self._tolerance
            ),
            sort_keys=True,
        )
        self._status_pub.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P7GripperAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
