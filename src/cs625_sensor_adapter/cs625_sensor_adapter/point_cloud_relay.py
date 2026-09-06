"""Parameter-driven RGB-D relay with timestamp/frame freshness status."""

from __future__ import annotations

from copy import deepcopy
from typing import Dict, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

from cs625_ap_interfaces.msg import SensorStatus


class RgbdSensorAdapter(Node):
    """Relay four RGB-D streams only after explicit profile configuration.

    The node deliberately has no camera-brand defaults. Profile launch files
    provide all source and normalized topic names, keeping sim/real differences
    below this adapter boundary.
    """

    def __init__(self) -> None:
        super().__init__("point_cloud_relay")
        for stream in ("color", "depth", "camera_info", "points"):
            self.declare_parameter(f"input_{stream}_topic", "")
            self.declare_parameter(f"output_{stream}_topic", "")
        self.declare_parameter("status_topic", "")
        self.declare_parameter("freshness_timeout_sec", 1.0)
        self.declare_parameter("drop_invalid_messages", True)
        self.declare_parameter("validate_camera_info", True)
        self.declare_parameter("motion_stale_guard", False)
        self.declare_parameter("motion_complete_time_sec", 0.0)
        self.declare_parameter("motion_status_topic", "/motion/status")
        self.declare_parameter("point_cloud_axis_convention", "ros_optical")

        input_topics = {
            stream: str(self.get_parameter(f"input_{stream}_topic").value)
            for stream in ("color", "depth", "camera_info", "points")
        }
        output_topics = {
            stream: str(self.get_parameter(f"output_{stream}_topic").value)
            for stream in ("color", "depth", "camera_info", "points")
        }
        status_topic = str(self.get_parameter("status_topic").value)

        self._last_receive_ns: Dict[str, Optional[int]] = {
            stream: None for stream in input_topics
        }
        self._received_count_by_stream = {stream: 0 for stream in input_topics}
        self._last_frame_id = ""
        self._freshness_timeout_sec = float(
            self.get_parameter("freshness_timeout_sec").value
        )
        self._drop_invalid_messages = bool(
            self.get_parameter("drop_invalid_messages").value
        )
        self._point_cloud_axis_convention = str(
            self.get_parameter("point_cloud_axis_convention").value
        )
        if self._point_cloud_axis_convention not in (
            "ros_optical",
            "gazebo_camera_x_forward",
        ):
            raise ValueError(
                "point_cloud_axis_convention must be ros_optical or "
                "gazebo_camera_x_forward"
            )
        # Do not shadow rclpy.node.Node._publishers, which is the internal
        # list used by create_publisher().
        self._stream_publishers = {}
        self._status_publisher = None
        self._subscriptions = []

        missing = [
            name
            for name, value in list(input_topics.items()) + list(output_topics.items())
            if not value
        ]
        if not status_topic:
            missing.append("status_topic")
        if missing:
            self.get_logger().error(
                "Required topic parameters are unset: "
                + ", ".join(sorted(set(missing)))
            )
            return

        message_types = {
            "color": Image,
            "depth": Image,
            "camera_info": CameraInfo,
            "points": PointCloud2,
        }
        for stream, message_type in message_types.items():
            self._stream_publishers[stream] = self.create_publisher(
                message_type, output_topics[stream], qos_profile_sensor_data
            )
        self._status_publisher = self.create_publisher(SensorStatus, status_topic, 10)
        self._subscriptions = [
            self.create_subscription(
                Image,
                input_topics["color"],
                lambda message: self._relay("color", message),
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                Image,
                input_topics["depth"],
                lambda message: self._relay("depth", message),
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                CameraInfo,
                input_topics["camera_info"],
                lambda message: self._relay("camera_info", message),
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                PointCloud2,
                input_topics["points"],
                lambda message: self._relay("points", message),
                qos_profile_sensor_data,
            ),
        ]
        self._status_timer = self.create_timer(1.0, self._publish_status)
        self.get_logger().info(
            "RGB-D adapter configured for streams: " + ", ".join(input_topics)
        )

    @staticmethod
    def _valid_header(message) -> bool:
        return bool(message.header.frame_id) and (
            message.header.stamp.sec != 0 or message.header.stamp.nanosec != 0
        )

    @staticmethod
    def _gazebo_points_to_ros_optical(message: PointCloud2) -> PointCloud2:
        """Convert Gazebo camera (X forward) XYZ fields to ROS optical axes.

        The byte layout, organization, RGB field, timestamps and all unknown
        fields are retained.  Only float32 x/y/z values are rewritten:
        ``(x_o, y_o, z_o) = (-y_g, -z_g, x_g)``.
        """

        fields = {field.name: field for field in message.fields}
        required = ("x", "y", "z")
        if any(name not in fields for name in required):
            raise ValueError("PointCloud2 is missing x/y/z fields")
        if any(fields[name].datatype != 7 or fields[name].count != 1 for name in required):
            raise ValueError("PointCloud2 x/y/z fields must be scalar FLOAT32")
        converted = deepcopy(message)
        mutable = bytearray(converted.data)
        endian = ">" if converted.is_bigendian else "<"
        dtype = np.dtype(
            {
                "names": list(required),
                "formats": [f"{endian}f4"] * 3,
                "offsets": [fields[name].offset for name in required],
                "itemsize": converted.point_step,
            }
        )
        points = np.ndarray(
            shape=(converted.height, converted.width),
            dtype=dtype,
            buffer=mutable,
            strides=(converted.row_step, converted.point_step),
        )
        x_g = points["x"].copy()
        y_g = points["y"].copy()
        z_g = points["z"].copy()
        points["x"] = -y_g
        points["y"] = -z_g
        points["z"] = x_g
        converted.data = bytes(mutable)
        return converted

    def _relay(self, stream: str, message) -> None:
        if not self._valid_header(message) and self._drop_invalid_messages:
            self.get_logger().warning(
                f"Dropping {stream} message without a valid frame_id and timestamp"
            )
            self._publish_status(detail=f"invalid {stream} header")
            return

        # CameraInfo validation (P2)
        if stream == "camera_info" and self.get_parameter("validate_camera_info").value:
            k = message.k
            if k[0] <= 0.0 or k[4] <= 0.0:
                self.get_logger().warning(
                    "CameraInfo has invalid K matrix (zero focal length)"
                )
                if self._drop_invalid_messages:
                    self._publish_status(detail="invalid camera_info K")
                    return
            if message.width == 0 or message.height == 0:
                self.get_logger().warning("CameraInfo has zero dimensions")
                if self._drop_invalid_messages:
                    self._publish_status(detail="invalid camera_info dimensions")
                    return

        # Stale-frame guard (P2): drop frames older than last motion completion
        if self.get_parameter("motion_stale_guard").value:
            motion_cutoff_sec = float(
                self.get_parameter("motion_complete_time_sec").value
            )
            if motion_cutoff_sec > 0.0:
                msg_sec = (
                    message.header.stamp.sec
                    + message.header.stamp.nanosec * 1e-9
                )
                if msg_sec < motion_cutoff_sec:
                    self.get_logger().debug(
                        f"Dropping stale {stream} frame: "
                        f"stamp {msg_sec:.3f} < cutoff {motion_cutoff_sec:.3f}"
                    )
                    self._publish_status(detail=f"stale_{stream}_dropped")
                    return

        if (
            stream == "points"
            and self._point_cloud_axis_convention == "gazebo_camera_x_forward"
        ):
            try:
                message = self._gazebo_points_to_ros_optical(message)
            except ValueError as error:
                self.get_logger().warning(f"Dropping invalid point cloud: {error}")
                self._publish_status(detail="invalid point cloud layout")
                return

        publisher = self._stream_publishers.get(stream)
        if publisher is not None:
            publisher.publish(message)
        self._last_receive_ns[stream] = self.get_clock().now().nanoseconds
        self._last_frame_id = message.header.frame_id
        self._received_count_by_stream[stream] += 1
        self._publish_status(detail=f"received {stream}")

    def _publish_status(self, detail: str = "waiting") -> None:
        if self._status_publisher is None:
            return
        now = self.get_clock().now()
        connected = any(value > 0 for value in self._received_count_by_stream.values())
        fresh = True
        for stream, last_receive_ns in self._last_receive_ns.items():
            if last_receive_ns is None:
                fresh = False
                break
            age_sec = (now.nanoseconds - last_receive_ns) / 1e9
            if not 0.0 <= age_sec <= self._freshness_timeout_sec:
                fresh = False
                break
        status = SensorStatus()
        status.stamp = now.to_msg()
        status.frame_id = self._last_frame_id
        status.connected = connected
        status.fresh = fresh
        status.received_count = sum(self._received_count_by_stream.values())
        status.detail = detail if detail != "waiting" else ("fresh" if fresh else "waiting")
        self._status_publisher.publish(status)


# Backward-compatible symbol for the first skeleton name.
PointCloudRelay = RgbdSensorAdapter


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RgbdSensorAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
