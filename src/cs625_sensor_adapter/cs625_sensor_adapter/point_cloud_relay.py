"""Parameter-driven RGB-D relay with timestamp/frame freshness status."""

from __future__ import annotations

from typing import Dict, Optional

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

    def _relay(self, stream: str, message) -> None:
        if not self._valid_header(message) and self._drop_invalid_messages:
            self.get_logger().warning(
                f"Dropping {stream} message without a valid frame_id and timestamp"
            )
            self._publish_status(detail=f"invalid {stream} header")
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
