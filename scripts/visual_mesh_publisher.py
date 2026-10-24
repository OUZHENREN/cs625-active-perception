#!/usr/bin/env python3
"""Publish the full visual meshes of the box and slot at their current poses.

The MoveIt collision scene uses simplified collision meshes (no handles).  This
node publishes the visual GLB meshes on /visualization_marker_array so RViz can
show the complete objects while planning still uses the collision approximation.

Run:
  source scripts/source_dev_env.sh --full
  ros2 run cs625_motion_adapter visual_mesh_publisher
"""

from __future__ import annotations

import os
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


# Mesh locations come from the environment so no machine-specific path is baked in.
_MESH_DIR = os.environ.get("CS625_MESH_DIR", "meshes")
BOX_MESH = Path(_MESH_DIR, "box.glb").resolve().as_uri()
SLOT_MESH = Path(_MESH_DIR, "slot_pointcloud.glb").resolve().as_uri()


class VisualMeshPublisher(Node):
    def __init__(self) -> None:
        super().__init__("cs625_visual_mesh_publisher")
        self._poses: dict[str, PoseStamped] = {}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            PoseStamped, "/box_target_pose",
            lambda msg: self._poses.__setitem__("box", msg), latched,
        )
        self.create_subscription(
            PoseStamped, "/slot_target_pose",
            lambda msg: self._poses.__setitem__("slot", msg), 10,
        )
        self._box_visible = True
        self.create_subscription(
            String, "/cs625/grasp_state_event", self._on_grasp_event, 10
        )
        self._publisher = self.create_publisher(MarkerArray, "/visualization_marker_array", 10)
        self.create_timer(0.5, self._publish)
        self.get_logger().info("visual mesh publisher ready (box.glb / slot_pointcloud.glb)")

    def _on_grasp_event(self, msg: String) -> None:
        event = (msg.data or "").strip().lower()
        if event == "grasp_confirmed":
            self._box_visible = False
            self.get_logger().info("box visual hidden: grasp_confirmed")
        elif event in ("release_confirmed", "reset"):
            self._box_visible = True
            self.get_logger().info(f"box visual restored: {event}")

    def _delete_marker(self, marker_id: int, namespace: str) -> Marker:
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.action = Marker.DELETE
        return marker

    def _publish(self) -> None:
        markers = MarkerArray()
        for marker_id, (key, mesh, label) in enumerate(
            (("box", BOX_MESH, "box_visual"), ("slot", SLOT_MESH, "slot_visual"))
        ):
            pose = self._poses.get(key)
            if pose is None:
                continue
            if key == "box" and not self._box_visible:
                markers.markers.append(self._delete_marker(marker_id, label))
                continue
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = label
            marker.id = marker_id
            marker.type = Marker.MESH_RESOURCE
            marker.action = Marker.ADD
            marker.pose = pose.pose
            marker.scale.x = marker.scale.y = marker.scale.z = 1.0
            marker.mesh_resource = mesh
            marker.mesh_use_embedded_materials = True
            markers.markers.append(marker)
        if markers.markers:
            self._publisher.publish(markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisualMeshPublisher()
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
