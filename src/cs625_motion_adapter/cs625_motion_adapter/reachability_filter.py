"""Filter camera candidates through TF, IK, collision and planning checks.

The node is deliberately planning-only.  It never creates a trajectory action
client and never sends a trajectory to a controller; that boundary keeps P3
evaluation valid with ``execute:=false``.
"""

from __future__ import annotations

import json
import math
import time
from typing import Dict, Iterable, Tuple

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetMotionPlan, GetPositionIK, GetStateValidity
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from cs625_ap_interfaces.msg import ActiveLocalizationState, ViewCandidate, ViewCandidateArray


def _quaternion_multiply(left: Tuple[float, float, float, float], right: Tuple[float, float, float, float]):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _rotate(vector: Tuple[float, float, float], quaternion: Tuple[float, float, float, float]):
    inverse = (-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3])
    rotated = _quaternion_multiply(
        _quaternion_multiply(quaternion, (vector[0], vector[1], vector[2], 0.0)), inverse
    )
    return rotated[0], rotated[1], rotated[2]


def compose_pose(base_pose: Pose, relative_pose: Pose) -> Pose:
    """Compose ``T_base_relative`` with ``T_relative_child``."""
    base_q = (
        base_pose.orientation.x,
        base_pose.orientation.y,
        base_pose.orientation.z,
        base_pose.orientation.w,
    )
    relative_q = (
        relative_pose.orientation.x,
        relative_pose.orientation.y,
        relative_pose.orientation.z,
        relative_pose.orientation.w,
    )
    translated = _rotate(
        (relative_pose.position.x, relative_pose.position.y, relative_pose.position.z), base_q
    )
    result = Pose()
    result.position.x = base_pose.position.x + translated[0]
    result.position.y = base_pose.position.y + translated[1]
    result.position.z = base_pose.position.z + translated[2]
    result.orientation.x, result.orientation.y, result.orientation.z, result.orientation.w = (
        _quaternion_multiply(base_q, relative_q)
    )
    return result


def transform_as_pose(transform) -> Pose:
    """Adapt a geometry_msgs/Transform into the pose composition primitive."""
    pose = Pose()
    pose.position.x = transform.translation.x
    pose.position.y = transform.translation.y
    pose.position.z = transform.translation.z
    pose.orientation = transform.rotation
    return pose


class ReachabilityFilter(Node):
    """Evaluate one candidate at a time so every rejection retains a reason."""

    def __init__(self) -> None:
        super().__init__("cs625_reachability_filter")
        self.declare_parameter("raw_candidates_topic", "/view_planner/raw_candidates")
        self.declare_parameter("reachable_candidates_topic", "/view_planner/reachable_candidates")
        self.declare_parameter("motion_status_topic", "/motion/status")
        self.declare_parameter("state_topic", "/active_localization/state")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("planning_frame", "")
        self.declare_parameter("camera_frame", "")
        self.declare_parameter("tool_frame", "")
        self.declare_parameter("planning_group", "")
        # An untyped empty list becomes BYTE_ARRAY in rclpy, which rejects the
        # profile's floating-point workspace bounds.  Declare the intended
        # type explicitly while still requiring the profile to provide values.
        self.declare_parameter("workspace_min", rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("workspace_max", rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("joint_limit_names", rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter("joint_limit_min", rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("joint_limit_max", rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("ik_service", "/compute_ik")
        self.declare_parameter("state_validity_service", "/check_state_validity")
        self.declare_parameter("plan_service", "/plan_kinematic_path")
        self.declare_parameter("service_wait_sec", 10.0)
        self.declare_parameter("request_timeout_sec", 5.0)
        self.declare_parameter("planning_time_sec", 2.0)
        self.declare_parameter("goal_tolerance_rad", 0.01)
        self.declare_parameter("reachable_publish_replay_count", 5)
        self.declare_parameter("reachable_publish_replay_period_sec", 0.5)

        self._planning_frame = self._required_string("planning_frame")
        self._camera_frame = self._required_string("camera_frame")
        self._tool_frame = self._required_string("tool_frame")
        self._planning_group = self._required_string("planning_group")
        self._workspace_min = tuple(float(value) for value in self.get_parameter("workspace_min").value)
        self._workspace_max = tuple(float(value) for value in self.get_parameter("workspace_max").value)
        if len(self._workspace_min) != 3 or len(self._workspace_max) != 3:
            raise ValueError("workspace_min and workspace_max must each have exactly three configured values")
        if any(low >= high for low, high in zip(self._workspace_min, self._workspace_max)):
            raise ValueError("workspace bounds are invalid")
        limit_names = list(self.get_parameter("joint_limit_names").value)
        limit_min = list(self.get_parameter("joint_limit_min").value)
        limit_max = list(self.get_parameter("joint_limit_max").value)
        if not limit_names or not (len(limit_names) == len(limit_min) == len(limit_max)):
            raise ValueError("joint-limit names and bounds must be profile-configured arrays")
        if any(low >= high for low, high in zip(limit_min, limit_max)):
            raise ValueError("joint-limit bounds are invalid")
        self._joint_limits = {
            name: (float(low), float(high))
            for name, low, high in zip(limit_names, limit_min, limit_max)
        }
        self._request_timeout = float(self.get_parameter("request_timeout_sec").value)
        self._planning_time = float(self.get_parameter("planning_time_sec").value)
        self._goal_tolerance = float(self.get_parameter("goal_tolerance_rad").value)
        self._service_wait_sec = float(self.get_parameter("service_wait_sec").value)
        self._reachable_replay_count = int(
            self.get_parameter("reachable_publish_replay_count").value
        )
        self._reachable_replay_period = float(
            self.get_parameter("reachable_publish_replay_period_sec").value
        )
        if not 1 <= self._reachable_replay_count <= 10:
            raise ValueError("reachable_publish_replay_count must be between 1 and 10")
        if not 0.1 <= self._reachable_replay_period <= 2.0:
            raise ValueError("reachable_publish_replay_period_sec must be between 0.1 and 2.0")

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._ik_client = self.create_client(GetPositionIK, str(self.get_parameter("ik_service").value))
        self._state_client = self.create_client(
            GetStateValidity, str(self.get_parameter("state_validity_service").value)
        )
        self._plan_client = self.create_client(
            GetMotionPlan, str(self.get_parameter("plan_service").value)
        )
        self._reachable_publisher = self.create_publisher(
            ViewCandidateArray,
            str(self.get_parameter("reachable_candidates_topic").value),
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._status_publisher = self.create_publisher(
            String,
            str(self.get_parameter("motion_status_topic").value),
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._state_publisher = self.create_publisher(
            ActiveLocalizationState, str(self.get_parameter("state_topic").value), 10
        )
        self._subscription = self.create_subscription(
            ViewCandidateArray,
            str(self.get_parameter("raw_candidates_topic").value),
            self._on_candidates,
            10,
        )
        self._joint_subscription = self.create_subscription(
            JointState,
            str(self.get_parameter("joint_states_topic").value),
            self._on_joint_state,
            10,
        )
        self._current_joints: Dict[str, float] = {}
        self._pending: ViewCandidateArray | None = None
        self._index = 0
        self._future = None
        self._future_started = 0.0
        self._phase = "idle"
        self._service_deadline = 0.0
        self._output: ViewCandidateArray | None = None
        self._reachable_replay_message: ViewCandidateArray | None = None
        self._reachable_replay_remaining = 0
        self._reachable_replay_timer = None
        self.create_timer(0.05, self._advance)

    def _required_string(self, name: str) -> str:
        value = str(self.get_parameter(name).value).strip()
        if not value:
            raise ValueError(f"{name} must be supplied by the profile configuration")
        return value

    def _on_joint_state(self, message: JointState) -> None:
        self._current_joints.update(zip(message.name, message.position))

    def _on_candidates(self, message: ViewCandidateArray) -> None:
        if self._pending is not None:
            self.get_logger().warning("Ignoring a candidate update while the current P3 evaluation is running")
            return
        if not message.candidates:
            self.get_logger().warning("Received an empty candidate set")
            return
        self._pending = message
        self._output = ViewCandidateArray()
        self._output.header = message.header
        self._output.source_candidate_count = message.source_candidate_count or len(message.candidates)
        self._index = 0
        self._phase = "waiting_services"
        self._service_deadline = time.monotonic() + self._service_wait_sec
        self._publish_state(ActiveLocalizationState.FILTER_REACHABILITY, "waiting for MoveIt services")

    def _advance(self) -> None:
        if self._pending is None or self._output is None:
            return
        if self._phase == "waiting_services":
            if self._services_ready():
                self._phase = "next"
            elif time.monotonic() > self._service_deadline:
                self._finish_unavailable()
            return
        if self._phase in {"ik", "validity", "plan"}:
            if not self._future.done():
                if time.monotonic() - self._future_started > self._request_timeout:
                    self._finish_current("SERVICE_TIMEOUT")
                return
            try:
                response = self._future.result()
            except Exception as error:  # service transport failure
                self.get_logger().warning(f"{self._phase} service failed: {error}")
                self._finish_current("SERVICE_ERROR")
                return
            self._consume_response(response)
            return
        if self._phase == "next":
            if self._index >= len(self._pending.candidates):
                self._publish_result()
                return
            self._start_current()

    def _services_ready(self) -> bool:
        return all(
            client.service_is_ready()
            for client in (self._ik_client, self._state_client, self._plan_client)
        )

    def _current(self) -> ViewCandidate:
        assert self._pending is not None
        return self._pending.candidates[self._index]

    def _start_current(self) -> None:
        candidate = self._current()
        candidate.reachable = False
        candidate.failure_code = ""
        if not self._inside_workspace(candidate.camera_pose.pose):
            self._finish_current("WORKSPACE_BOUNDS")
            return
        try:
            camera_to_tool = self._tf_buffer.lookup_transform(
                self._camera_frame,
                self._tool_frame,
                rclpy.time.Time(),
            )
        except TransformException as error:
            self.get_logger().warning(f"Camera-to-tool TF unavailable: {error}")
            self._finish_current("TF_UNAVAILABLE")
            return
        candidate.tool_pose.header = candidate.camera_pose.header
        candidate.tool_pose.pose = compose_pose(
            candidate.camera_pose.pose, transform_as_pose(camera_to_tool.transform)
        )
        request = GetPositionIK.Request()
        request.ik_request.group_name = self._planning_group
        request.ik_request.ik_link_name = self._tool_frame
        request.ik_request.pose_stamped = candidate.tool_pose
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout = Duration(sec=max(1, math.ceil(self._request_timeout)))
        self._future = self._ik_client.call_async(request)
        self._future_started = time.monotonic()
        self._phase = "ik"

    def _consume_response(self, response) -> None:
        if self._phase == "ik":
            if response.error_code.val != MoveItErrorCodes.SUCCESS:
                self._finish_current("NO_IK")
                return
            self._solution = response.solution
            request = GetStateValidity.Request()
            request.group_name = self._planning_group
            request.robot_state = self._solution
            self._future = self._state_client.call_async(request)
            self._future_started = time.monotonic()
            self._phase = "validity"
            return
        if self._phase == "validity":
            if not response.valid:
                self._finish_current("COLLISION")
                return
            request = GetMotionPlan.Request()
            request.motion_plan_request.group_name = self._planning_group
            request.motion_plan_request.num_planning_attempts = 1
            request.motion_plan_request.allowed_planning_time = self._planning_time
            request.motion_plan_request.goal_constraints = [self._goal_constraints(self._solution.joint_state)]
            self._future = self._plan_client.call_async(request)
            self._future_started = time.monotonic()
            self._phase = "plan"
            self._publish_state(ActiveLocalizationState.PLAN_TO_VIEW, "planning candidate motion (planning-only)")
            return
        if response.motion_plan_response.error_code.val != MoveItErrorCodes.SUCCESS:
            self._finish_current("PLANNING_FAILED")
            return
        candidate = self._current()
        candidate.reachable = True
        candidate.motion_cost = self._joint_motion_cost(self._solution.joint_state)
        candidate.joint_margin = self._joint_margin(self._solution.joint_state)
        candidate.planning_time_sec = time.monotonic() - self._future_started
        self._finish_current("")

    def _goal_constraints(self, joint_state: JointState) -> Constraints:
        constraints = Constraints()
        for name, position in zip(joint_state.name, joint_state.position):
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = position
            constraint.tolerance_above = self._goal_tolerance
            constraint.tolerance_below = self._goal_tolerance
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)
        return constraints

    def _joint_motion_cost(self, joint_state: JointState) -> float:
        # ``sum`` returns integer 0 for an empty iterator.  ROS generated
        # float64 converters require a Python float even in that valid
        # startup case, before the first joint-state message arrives.
        return float(sum(
            abs(position - self._current_joints[name])
            for name, position in zip(joint_state.name, joint_state.position)
            if name in self._current_joints
        ))

    def _joint_margin(self, joint_state: JointState) -> float:
        """Return the worst normalized distance from configured joint limits."""
        margins = []
        for name, position in zip(joint_state.name, joint_state.position):
            limits = self._joint_limits.get(name)
            if limits is None:
                continue
            lower, upper = limits
            half_range = (upper - lower) / 2.0
            margins.append(max(0.0, min(1.0, min(position - lower, upper - position) / half_range)))
        return min(margins) if margins else 0.0

    def _inside_workspace(self, pose: Pose) -> bool:
        return all(
            lower <= value <= upper
            for value, lower, upper in zip(
                (pose.position.x, pose.position.y, pose.position.z),
                self._workspace_min,
                self._workspace_max,
            )
        )

    def _finish_current(self, failure_code: str) -> None:
        candidate = self._current()
        if failure_code:
            candidate.reachable = False
            candidate.failure_code = failure_code
        self._output.candidates.append(candidate)
        self._index += 1
        self._phase = "next"

    def _finish_unavailable(self) -> None:
        while self._index < len(self._pending.candidates):
            self._finish_current("MOVEIT_UNAVAILABLE")
        self._publish_result()

    def _publish_result(self) -> None:
        assert self._output is not None
        reachable = [candidate for candidate in self._output.candidates if candidate.reachable]
        # The topic contract reserves this stream for hard-filtered candidates.
        filtered = ViewCandidateArray()
        filtered.header = self._output.header
        filtered.source_candidate_count = self._output.source_candidate_count
        filtered.candidates = reachable
        self._publish_reachable_with_replay(filtered)
        failures = {}
        for candidate in self._output.candidates:
            if candidate.failure_code:
                failures[candidate.failure_code] = failures.get(candidate.failure_code, 0) + 1
        self._publish_status({
            "candidate_count": len(self._output.candidates),
            "reachable_count": len(reachable),
            "failure_counts": failures,
            "planning_only": True,
        })
        self._publish_state(
            ActiveLocalizationState.EVALUATE_VIEWS,
            f"P3 complete: {len(reachable)}/{len(self._output.candidates)} candidates reachable",
        )
        self.get_logger().info(
            f"P3 reachability complete: {len(reachable)}/{len(self._output.candidates)} reachable; "
            f"failures={failures}"
        )
        self._pending = None
        self._output = None
        self._phase = "idle"

    def _publish_reachable_with_replay(self, filtered: ViewCandidateArray) -> None:
        """Publish a bounded replay burst after a completed P3 evaluation.

        In WSL/Fast DDS, a late P4 process can miss an otherwise compatible
        one-shot transient-local sample during endpoint discovery.  Replays
        carry the same immutable filtered result; the coordinator accepts only
        the first one, so this does not create additional episode selections.
        """
        self._stop_reachable_replay()
        self._reachable_publisher.publish(filtered)
        self.get_logger().info(
            "Published reachable candidates for P4: "
            f"{len(filtered.candidates)} candidates, "
            f"subscribers={self._reachable_publisher.get_subscription_count()}"
        )
        self._reachable_replay_message = filtered
        self._reachable_replay_remaining = self._reachable_replay_count - 1
        if self._reachable_replay_remaining:
            self._reachable_replay_timer = self.create_timer(
                self._reachable_replay_period, self._replay_reachable
            )

    def _replay_reachable(self) -> None:
        if self._reachable_replay_message is None or self._reachable_replay_remaining <= 0:
            self._stop_reachable_replay()
            return
        self._reachable_publisher.publish(self._reachable_replay_message)
        self.get_logger().info(
            "Replayed reachable candidates for P4: "
            f"remaining={self._reachable_replay_remaining - 1}, "
            f"subscribers={self._reachable_publisher.get_subscription_count()}"
        )
        self._reachable_replay_remaining -= 1

    def _stop_reachable_replay(self) -> None:
        if self._reachable_replay_timer is not None:
            self._reachable_replay_timer.cancel()
            self.destroy_timer(self._reachable_replay_timer)
            self._reachable_replay_timer = None

    def _publish_status(self, detail: Dict) -> None:
        message = String()
        message.data = json.dumps(detail, sort_keys=True)
        self._status_publisher.publish(message)

    def _publish_state(self, state: int, detail: str) -> None:
        message = ActiveLocalizationState()
        message.stamp = self.get_clock().now().to_msg()
        message.state = state
        message.detail = detail
        message.execute_enabled = False
        self._state_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ReachabilityFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_reachable_replay()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
