"""Simulation-only execution of a previously hard-filtered selected view."""

from __future__ import annotations

import json
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory  # documents the controller contract
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetMotionPlan, GetPositionIK
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from cs625_ap_interfaces.msg import ViewSelection


def execution_status_payload(
    selection: ViewSelection,
    success: bool,
    code: str,
    profile: str,
    selection_started: float,
    finished_at: float,
    phase_durations: dict[str, float],
) -> dict:
    """Build the immutable, per-attempt runtime record for a P4 episode."""
    return {
        "cycle_index": selection.cycle_index,
        "candidate_id": selection.candidate.candidate_id,
        "success": success,
        "code": code,
        "profile": profile,
        "motion_cost": selection.candidate.motion_cost,
        "planning_time_sec": selection.candidate.planning_time_sec,
        "view_wall_time_sec": max(0.0, finished_at - selection_started),
        "execution_ik_time_sec": phase_durations.get("ik_wall_time_sec", 0.0),
        "execution_motion_planning_time_sec": phase_durations.get("plan_wall_time_sec", 0.0),
        "trajectory_execution_time_sec": phase_durations.get("result_wall_time_sec", 0.0),
        "sensor_settle_wait_time_sec": phase_durations.get("waiting_sensor_wall_time_sec", 0.0),
    }


class SimViewExecutor(Node):
    """Plan, execute in Gazebo, then require a post-motion RGB-D frame."""

    def __init__(self) -> None:
        super().__init__("cs625_sim_view_executor")
        for name, value in (
            ("profile", ""), ("execute", False), ("require_confirmation", True),
            ("selected_view_topic", "/view_planner/selected_view"),
            ("execution_status_topic", "/motion/execution_status"),
            ("points_topic", "/sensors/camera/points"), ("planning_group", ""),
            ("tool_frame", ""), ("ik_service", "/compute_ik"),
            ("plan_service", "/plan_kinematic_path"),
            ("controller_action", "/joint_trajectory_controller/follow_joint_trajectory"),
            ("planning_time_sec", 2.0), ("goal_tolerance_rad", 0.01),
            ("max_velocity_scale", 1.0), ("max_acceleration_scale", 1.0),
            ("request_timeout_sec", 10.0), ("execution_timeout_sec", 90.0), ("sensor_settle_timeout_sec", 15.0),
            ("execution_status_publish_replay_count", 5),
            ("execution_status_publish_replay_period_sec", 0.5),
        ):
            self.declare_parameter(name, value)
        self._profile = str(self.get_parameter("profile").value).strip()
        self._enabled = bool(self.get_parameter("execute").value) and not bool(
            self.get_parameter("require_confirmation").value
        ) and self._profile == "sim"
        self._group = str(self.get_parameter("planning_group").value).strip()
        self._tool = str(self.get_parameter("tool_frame").value).strip()
        if not self._group or not self._tool:
            raise ValueError("planning_group and tool_frame must be profile-configured")
        self._planning_time = float(self.get_parameter("planning_time_sec").value)
        self._tolerance = float(self.get_parameter("goal_tolerance_rad").value)
        self._max_velocity_scale = float(self.get_parameter("max_velocity_scale").value)
        self._max_acceleration_scale = float(self.get_parameter("max_acceleration_scale").value)
        self._request_timeout = float(self.get_parameter("request_timeout_sec").value)
        self._execution_timeout = float(self.get_parameter("execution_timeout_sec").value)
        self._sensor_timeout = float(self.get_parameter("sensor_settle_timeout_sec").value)
        self._status_replay_count = int(self.get_parameter("execution_status_publish_replay_count").value)
        self._status_replay_period = float(self.get_parameter("execution_status_publish_replay_period_sec").value)
        if not 1 <= self._status_replay_count <= 10:
            raise ValueError("execution_status_publish_replay_count must be between 1 and 10")
        if not 0.1 <= self._status_replay_period <= 2.0:
            raise ValueError("execution_status_publish_replay_period_sec must be between 0.1 and 2.0")
        self._ik = self.create_client(GetPositionIK, str(self.get_parameter("ik_service").value))
        self._plan = self.create_client(GetMotionPlan, str(self.get_parameter("plan_service").value))
        self._execute = ActionClient(
            self,
            FollowJointTrajectory,
            str(self.get_parameter("controller_action").value),
        )
        self._status = self.create_publisher(String, str(self.get_parameter("execution_status_topic").value), 10)
        self.create_subscription(
            ViewSelection,
            str(self.get_parameter("selected_view_topic").value),
            self._on_selection,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.create_subscription(PointCloud2, str(self.get_parameter("points_topic").value), self._on_cloud, qos_profile_sensor_data)
        self._selection = None
        self._future = None
        self._phase = "idle"
        self._motion_stamp = None
        self._phase_started = 0.0
        self._selection_started = 0.0
        self._phase_durations = {}
        # Progress and timeout monitoring must remain live even when Gazebo
        # has not started publishing /clock yet.  Sensor-success validation
        # below still compares Gazebo timestamps.
        self._wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._status_replay_message = None
        self._status_replay_remaining = 0
        self._status_replay_timer = None
        self.create_timer(0.05, self._advance, clock=self._wall_clock)

    def _on_selection(self, selection: ViewSelection) -> None:
        if self._phase != "idle":
            return
        self.get_logger().info(
            f"P4 executor received selection {selection.candidate.candidate_id} "
            f"for cycle {selection.cycle_index}"
        )
        self._selection = selection
        self._selection_started = time.monotonic()
        self._phase_durations = {}
        if not self._enabled:
            self._finish(False, "EXECUTION_GATE_CLOSED")
            return
        self._set_phase("waiting_moveit")

    def _set_phase(self, phase: str) -> None:
        """Enter an executor phase and retain its steady-clock duration.

        These durations are runtime evidence for the simulation execution
        chain.  They are deliberately separate from the P3 candidate planning
        duration carried in ``planning_time_sec``.
        """
        now = time.monotonic()
        if self._phase != "idle":
            field = f"{self._phase}_wall_time_sec"
            self._phase_durations[field] = self._phase_durations.get(field, 0.0) + max(
                0.0, now - self._phase_started
            )
        self._phase = phase
        self._phase_started = now

    def _request_ik(self) -> None:
        assert self._selection is not None
        request = GetPositionIK.Request()
        request.ik_request.group_name = self._group
        request.ik_request.ik_link_name = self._tool
        request.ik_request.pose_stamped = self._selection.candidate.tool_pose
        # Seed from MoveIt's monitored current state; do not send an empty
        # RobotState as a complete state (which is rejected by MoveIt).
        request.ik_request.robot_state.is_diff = True
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout = Duration(sec=5)
        self._future = self._ik.call_async(request)
        self._set_phase("ik")

    def _advance(self) -> None:
        if self._phase == "waiting_moveit":
            if self._ik.service_is_ready() and self._plan.service_is_ready() and self._execute.server_is_ready():
                self._request_ik()
            elif time.monotonic() - self._phase_started > self._request_timeout:
                self._finish(False, "MOVEIT_UNAVAILABLE")
            return
        if self._phase == "waiting_sensor":
            if time.monotonic() - self._phase_started > self._sensor_timeout:
                self._finish(False, "SENSOR_SETTLE_TIMEOUT")
            return
        if self._phase not in {"ik", "plan", "goal", "result"}:
            return
        if not self._future.done():
            timeout = self._execution_timeout if self._phase == "result" else self._request_timeout
            if time.monotonic() - self._phase_started > timeout:
                self._finish(False, f"{self._phase.upper()}_TIMEOUT")
            return
        try:
            response = self._future.result()
        except Exception:
            self._finish(False, "SERVICE_ERROR")
            return
        if self._phase == "ik":
            if response.error_code.val != MoveItErrorCodes.SUCCESS:
                self._finish(False, "NO_IK")
                return
            request = GetMotionPlan.Request()
            request.motion_plan_request.group_name = self._group
            request.motion_plan_request.num_planning_attempts = 1
            request.motion_plan_request.allowed_planning_time = self._planning_time
            request.motion_plan_request.max_velocity_scaling_factor = self._max_velocity_scale
            request.motion_plan_request.max_acceleration_scaling_factor = self._max_acceleration_scale
            request.motion_plan_request.goal_constraints = [self._constraints(response.solution.joint_state)]
            self._future = self._plan.call_async(request)
            self._set_phase("plan")
            return
        if self._phase == "plan":
            if response.motion_plan_response.error_code.val != MoveItErrorCodes.SUCCESS:
                self._finish(False, "PLANNING_FAILED")
                return
            # The trajectory is generated immediately by MoveIt above, so the
            # standard ros2_control action does not bypass collision checking.
            # It is used only for the explicit ``profile=sim`` gate.
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = response.motion_plan_response.trajectory.joint_trajectory
            self._future = self._execute.send_goal_async(goal)
            self._set_phase("goal")
            return
        if self._phase == "goal":
            if not response.accepted:
                self._finish(False, "EXECUTION_REJECTED")
                return
            self._future = response.get_result_async()
            self._set_phase("result")
            return
        if response.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self._finish(False, "EXECUTION_FAILED")
            return
        now = self.get_clock().now().to_msg()
        self._motion_stamp = (now.sec, now.nanosec)
        self._set_phase("waiting_sensor")

    def _on_cloud(self, cloud: PointCloud2) -> None:
        if self._phase != "waiting_sensor" or self._motion_stamp is None:
            return
        stamp = (cloud.header.stamp.sec, cloud.header.stamp.nanosec)
        if stamp > self._motion_stamp:
            self._finish(True, "SENSOR_SETTLED")

    def _constraints(self, joints):
        result = Constraints()
        for name, position in zip(joints.name, joints.position):
            item = JointConstraint(joint_name=name, position=position, weight=1.0)
            item.tolerance_above = self._tolerance
            item.tolerance_below = self._tolerance
            result.joint_constraints.append(item)
        return result

    def _finish(self, success: bool, code: str) -> None:
        if self._selection is None:
            return
        now = time.monotonic()
        if self._phase != "idle":
            field = f"{self._phase}_wall_time_sec"
            self._phase_durations[field] = self._phase_durations.get(field, 0.0) + max(
                0.0, now - self._phase_started
            )
        message = String()
        message.data = json.dumps(
            execution_status_payload(
                self._selection,
                success,
                code,
                self._profile,
                self._selection_started,
                now,
                self._phase_durations,
            ),
            sort_keys=True,
        )
        self._publish_status_with_replay(message)
        self.get_logger().info(
            f"P4 execution completed: {code}; "
            f"coordinator subscribers={self._status.get_subscription_count()}"
        )
        self._selection = None
        self._future = None
        self._phase = "idle"
        self._selection_started = 0.0
        self._phase_durations = {}

    def _publish_status_with_replay(self, message: String) -> None:
        """Publish a bounded duplicate status burst for WSL DDS discovery."""
        self._stop_status_replay()
        self._status.publish(message)
        self._status_replay_message = message
        self._status_replay_remaining = self._status_replay_count - 1
        if self._status_replay_remaining:
            self._status_replay_timer = self.create_timer(
                self._status_replay_period, self._replay_status, clock=self._wall_clock
            )

    def _replay_status(self) -> None:
        if self._status_replay_message is None or self._status_replay_remaining <= 0:
            self._stop_status_replay()
            return
        self._status.publish(self._status_replay_message)
        self._status_replay_remaining -= 1

    def _stop_status_replay(self) -> None:
        if self._status_replay_timer is not None:
            self._status_replay_timer.cancel()
            self.destroy_timer(self._status_replay_timer)
            self._status_replay_timer = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimViewExecutor()
    try:
        rclpy.spin(node)
    # P4 finishes by requesting launch shutdown after its JSON is committed.
    # This is an expected lifecycle transition, not an execution failure.
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node._stop_status_replay()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
