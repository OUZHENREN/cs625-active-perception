import pytest
import struct


pytest.importorskip("rclpy")


def test_relay_module_imports():
    from cs625_sensor_adapter.point_cloud_relay import (
        PointCloudRelay,
        RgbdSensorAdapter,
    )

    assert PointCloudRelay is RgbdSensorAdapter


def test_gazebo_x_forward_cloud_is_normalized_without_losing_rgb_or_layout():
    from sensor_msgs.msg import PointCloud2, PointField
    from cs625_sensor_adapter.point_cloud_relay import RgbdSensorAdapter

    message = PointCloud2()
    message.height = 1
    message.width = 2
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=16, datatype=PointField.FLOAT32, count=1),
    ]
    message.is_bigendian = False
    message.point_step = 24
    message.row_step = 48
    first = struct.pack("<fffIIf", 2.0, 0.5, -0.25, 0, 0x11223344, 0.0)
    second = struct.pack("<fffIIf", 3.0, -1.0, 0.75, 0, 0x55667788, 0.0)
    message.data = first + second

    converted = RgbdSensorAdapter._gazebo_points_to_ros_optical(message)

    assert converted.height == 1
    assert converted.width == 2
    assert converted.point_step == 24
    assert converted.row_step == 48
    assert struct.unpack_from("<fff", converted.data, 0) == pytest.approx(
        (-0.5, 0.25, 2.0)
    )
    assert struct.unpack_from("<fff", converted.data, 24) == pytest.approx(
        (1.0, -0.75, 3.0)
    )
    assert converted.data[12:24] == message.data[12:24]
    assert converted.data[36:48] == message.data[36:48]
    assert bytes(message.data) == first + second
