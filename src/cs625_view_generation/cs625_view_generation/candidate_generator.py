"""Generate reproducible camera poses around a target without touching MoveIt."""

from __future__ import annotations

import math
from typing import Iterable, Tuple

import rclpy
import tf2_geometry_msgs  # Registers PoseStamped conversions with tf2_ros.Buffer.
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener

from cs625_ap_interfaces.msg import ViewCandidate, ViewCandidateArray


def _normalize(vector: Tuple[float, float, float]) -> Tuple[float, float, float]:
    length = math.sqrt(sum(component * component for component in vector))
    if length < 1e-9:
        raise ValueError("zero-length view direction")
    return tuple(component / length for component in vector)


def _cross(left: Tuple[float, float, float], right: Tuple[float, float, float]):
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _quaternion_from_matrix(matrix: Iterable[Iterable[float]]):
    """Convert a proper 3x3 rotation matrix to an xyzw ROS quaternion."""
    rows = [tuple(row) for row in matrix]
    trace = rows[0][0] + rows[1][1] + rows[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        return (
            (rows[2][1] - rows[1][2]) / scale,
            (rows[0][2] - rows[2][0]) / scale,
            (rows[1][0] - rows[0][1]) / scale,
            0.25 * scale,
        )
    diagonal = [rows[0][0], rows[1][1], rows[2][2]]
    index = diagonal.index(max(diagonal))
    if index == 0:
        scale = math.sqrt(1.0 + rows[0][0] - rows[1][1] - rows[2][2]) * 2.0
        return (
            0.25 * scale,
            (rows[0][1] + rows[1][0]) / scale,
            (rows[0][2] + rows[2][0]) / scale,
            (rows[2][1] - rows[1][2]) / scale,
        )
    if index == 1:
        scale = math.sqrt(1.0 + rows[1][1] - rows[0][0] - rows[2][2]) * 2.0
        return (
            (rows[0][1] + rows[1][0]) / scale,
            0.25 * scale,
            (rows[1][2] + rows[2][1]) / scale,
            (rows[0][2] - rows[2][0]) / scale,
        )
    scale = math.sqrt(1.0 + rows[2][2] - rows[0][0] - rows[1][1]) * 2.0
    return (
        (rows[0][2] + rows[2][0]) / scale,
        (rows[1][2] + rows[2][1]) / scale,
        0.25 * scale,
        (rows[1][0] - rows[0][1]) / scale,
    )


def look_at_camera_pose(position: Tuple[float, float, float], target: Tuple[float, float, float]) -> Pose:
    """Return a ROS optical-frame pose whose +Z axis points at ``target``."""
    forward = _normalize(tuple(target[index] - position[index] for index in range(3)))
    world_up = (0.0, 0.0, 1.0)
    if abs(sum(forward[index] * world_up[index] for index in range(3))) > 0.98:
        world_up = (0.0, 1.0, 0.0)
    right = _normalize(_cross(forward, world_up))
    down = _cross(forward, right)
    quaternion = _quaternion_from_matrix(
        (
            (right[0], down[0], forward[0]),
            (right[1], down[1], forward[1]),
            (right[2], down[2], forward[2]),
        )
    )
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = position
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion
    return pose


class CandidateGenerator(Node):
    """Create an evenly distributed, target-facing camera candidate set."""

    def __init__(self) -> None:
        super().__init__("cs625_candidate_generator")
        self.declare_parameter("input_target_pose_topic", "/perception/target_pose")
        self.declare_parameter("raw_candidates_topic", "/view_planner/raw_candidates")
        self.declare_parameter("planning_frame", "")
        self.declare_parameter("candidate_count", 24)
        self.declare_parameter("view_radius_m", 0.70)
        self.declare_parameter("min_elevation_rad", 0.20)
        self.declare_parameter("max_elevation_rad", 1.05)
        self.declare_parameter("regenerate_on_target_update", False)

        self._planning_frame = str(self.get_parameter("planning_frame").value)
        self._candidate_count = int(self.get_parameter("candidate_count").value)
        self._radius = float(self.get_parameter("view_radius_m").value)
        self._min_elevation = float(self.get_parameter("min_elevation_rad").value)
        self._max_elevation = float(self.get_parameter("max_elevation_rad").value)
        self._repeat = bool(self.get_parameter("regenerate_on_target_update").value)
        if not self._planning_frame:
            raise ValueError("planning_frame must be supplied by the profile configuration")
        if not 20 <= self._candidate_count <= 50:
            raise ValueError("candidate_count must be between 20 and 50 for the P3 gate")
        if self._radius <= 0.0 or self._min_elevation > self._max_elevation:
            raise ValueError("invalid candidate geometry parameters")

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._published = False
        self._publisher = self.create_publisher(
            ViewCandidateArray,
            str(self.get_parameter("raw_candidates_topic").value),
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._subscription = self.create_subscription(
            PoseStamped,
            str(self.get_parameter("input_target_pose_topic").value),
            self._on_target_pose,
            10,
        )

    def _on_target_pose(self, target_pose: PoseStamped) -> None:
        if self._published and not self._repeat:
            return
        if not target_pose.header.frame_id:
            self.get_logger().error("Target pose has no frame_id; candidate generation deferred")
            return
        try:
            target_in_base = self._tf_buffer.transform(
                target_pose,
                self._planning_frame,
                timeout=Duration(seconds=0.2),
            )
        except TransformException as error:
            self.get_logger().warning(
                f"Target pose transform to {self._planning_frame} unavailable: {error}"
            )
            return
        self._publish_candidates(target_in_base)

    def _publish_candidates(self, target_pose: PoseStamped) -> None:
        target = (
            target_pose.pose.position.x,
            target_pose.pose.position.y,
            target_pose.pose.position.z,
        )
        output = ViewCandidateArray()
        output.header = target_pose.header
        output.source_candidate_count = self._candidate_count
        for index in range(self._candidate_count):
            fraction = (index + 0.5) / self._candidate_count
            elevation = self._min_elevation + fraction * (
                self._max_elevation - self._min_elevation
            )
            azimuth = index * math.pi * (3.0 - math.sqrt(5.0))
            position = (
                target[0] + self._radius * math.cos(elevation) * math.cos(azimuth),
                target[1] + self._radius * math.cos(elevation) * math.sin(azimuth),
                target[2] + self._radius * math.sin(elevation),
            )
            candidate = ViewCandidate()
            candidate.candidate_id = f"fibonacci_{index:02d}"
            candidate.camera_pose.header = output.header
            candidate.camera_pose.pose = look_at_camera_pose(position, target)
            # A geometry-only proxy is recorded for P4 baseline selection. It
            # is not a claim of measured target-surface coverage.
            candidate.coverage_proxy = math.sin(elevation)
            output.candidates.append(candidate)
        self._publisher.publish(output)
        self._published = True
        self.get_logger().info(
            f"Published {len(output.candidates)} reproducible camera candidates in "
            f"{self._planning_frame}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CandidateGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
