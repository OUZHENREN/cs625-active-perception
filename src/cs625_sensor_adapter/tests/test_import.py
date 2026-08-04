import pytest


pytest.importorskip("rclpy")


def test_relay_module_imports():
    from cs625_sensor_adapter.point_cloud_relay import (
        PointCloudRelay,
        RgbdSensorAdapter,
    )

    assert PointCloudRelay is RgbdSensorAdapter
