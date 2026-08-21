"""Read-only P6 R1--R3 evidence collector; it never commands hardware."""

from __future__ import annotations

import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from cs625_ap_interfaces.msg import SensorStatus


class RealReadinessMonitor(Node):
    """Collect timestamped, non-motion R1--R3 readiness evidence."""

    def __init__(self) -> None:
        super().__init__("cs625_real_readiness_monitor")
        for name, value in (
            ("profile", "real"), ("joint_states_topic", "/joint_states"),
            ("sensor_status_topic", "/sensors/camera/status"),
            ("status_topic", "/motion/status"), ("planning_frame", ""),
            ("camera_frame", ""), ("expected_joint_names", rclpy.Parameter.Type.STRING_ARRAY),
            ("required_moveit_services", rclpy.Parameter.Type.STRING_ARRAY),
            ("hand_eye_manifest", ""), ("freshness_timeout_sec", 1.0),
            ("assessment_timeout_sec", 15.0), ("exit_after_assessment", False),
        ):
            self.declare_parameter(name, value)
        self._started_ns = self.get_clock().now().nanoseconds
        self._joint_stamp_ns = None
        self._joint_names: set[str] = set()
        self._sensor_status = None
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._joint_sub = self.create_subscription(
            JointState, str(self.get_parameter("joint_states_topic").value), self._on_joint, 10
        )
        retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._sensor_sub = self.create_subscription(
            SensorStatus, str(self.get_parameter("sensor_status_topic").value), self._on_sensor, retained
        )
        self._status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), retained
        )
        self._timer = self.create_timer(0.5, self._assess)

    def _on_joint(self, message: JointState) -> None:
        self._joint_stamp_ns = self.get_clock().now().nanoseconds
        self._joint_names = set(message.name)

    def _on_sensor(self, message: SensorStatus) -> None:
        self._sensor_status = message

    def _manifest_valid(self) -> tuple[bool, str]:
        path_text = str(self.get_parameter("hand_eye_manifest").value).strip()
        if not path_text:
            return False, "HAND_EYE_MANIFEST_UNSET"
        path = Path(path_text)
        if not path.is_file():
            return False, "HAND_EYE_MANIFEST_MISSING"
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return False, "HAND_EYE_MANIFEST_INVALID"
        required = {"base_frame", "camera_frame", "method", "operator", "timestamp_utc", "transform"}
        return (required <= set(payload), "HAND_EYE_MANIFEST_VALID" if required <= set(payload) else "HAND_EYE_MANIFEST_INCOMPLETE")

    def _assess(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        freshness_ns = int(float(self.get_parameter("freshness_timeout_sec").value) * 1_000_000_000)
        expected_joints = set(self.get_parameter("expected_joint_names").value)
        joints_fresh = self._joint_stamp_ns is not None and now_ns - self._joint_stamp_ns <= freshness_ns
        joints_complete = bool(expected_joints) and expected_joints <= self._joint_names
        sensor_fresh = self._sensor_status is not None and self._sensor_status.connected and self._sensor_status.fresh
        planning_frame = str(self.get_parameter("planning_frame").value).strip()
        camera_frame = str(self.get_parameter("camera_frame").value).strip()
        tf_ready = False
        if planning_frame and camera_frame:
            try:
                tf_ready = self._buffer.can_transform(planning_frame, camera_frame, rclpy.time.Time())
            except TransformException:
                tf_ready = False
        services = list(self.get_parameter("required_moveit_services").value)
        # Service discovery is deliberately graph-only: no MoveIt request is sent.
        service_names = {name for name, _types in self.get_service_names_and_types()}
        services_ready = bool(services) and all(name in service_names for name in services)
        manifest_ready, manifest_code = self._manifest_valid()
        result = {
            "profile": str(self.get_parameter("profile").value),
            "real_motion_occurred": False,
            "r1_joint_state": "PASS" if joints_fresh and joints_complete else "PENDING",
            "r2_sensor_status": "PASS" if sensor_fresh else "PENDING",
            "r2_tf": "PASS" if tf_ready else "PENDING",
            "r2_hand_eye_manifest": manifest_code,
            "r3_moveit_services": "PASS" if services_ready else "PENDING",
            "expected_joint_count": len(expected_joints),
            "observed_joint_count": len(self._joint_names),
            "code": "R1_R3_READY" if joints_fresh and joints_complete and sensor_fresh and tf_ready and manifest_ready and services_ready else "R1_R3_EVIDENCE_PENDING",
        }
        message = String()
        message.data = json.dumps(result, sort_keys=True)
        self._status_pub.publish(message)
        self.get_logger().info(message.data)
        elapsed_ns = now_ns - self._started_ns
        if bool(self.get_parameter("exit_after_assessment").value) and elapsed_ns >= int(float(self.get_parameter("assessment_timeout_sec").value) * 1_000_000_000):
            self._timer.cancel()
            rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RealReadinessMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
