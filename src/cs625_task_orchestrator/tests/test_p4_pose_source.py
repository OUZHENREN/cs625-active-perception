"""P4 实时 TCP 位姿转换的单元测试（纯数学，不需要真机/ROS）。"""

from __future__ import annotations

import math
import pathlib
import sys
from types import SimpleNamespace

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from cs625_task_orchestrator.p4_pose_source import (  # noqa: E402
    format_values,
    parse_static_values,
    pose_to_values,
    quaternion_to_rpy_deg,
    state_to_flange_values,
    state_to_values,
)


def _pose(x=0.0, y=0.0, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    return SimpleNamespace(
        position=SimpleNamespace(x=x, y=y, z=z),
        orientation=SimpleNamespace(x=qx, y=qy, z=qz, w=qw),
    )


def test_identity_quaternion_is_zero_rpy():
    roll, pitch, yaw = quaternion_to_rpy_deg(0.0, 0.0, 0.0, 1.0)
    assert abs(roll) < 1e-9
    assert abs(pitch) < 1e-9
    assert abs(yaw) < 1e-9


def test_yaw_90_degrees():
    half = math.pi / 4.0
    roll, pitch, yaw = quaternion_to_rpy_deg(0.0, 0.0, math.sin(half), math.cos(half))
    assert abs(roll) < 1e-9
    assert abs(pitch) < 1e-9
    assert abs(yaw - 90.0) < 1e-6


def test_pose_meters_to_millimeters_and_roll_180():
    values = pose_to_values(_pose(x=0.4, y=0.1, z=0.7, qx=1.0, qy=0.0, qz=0.0, qw=0.0))
    assert values[:3] == [400.0, 100.0, 700.0]
    assert abs(abs(values[3]) - 180.0) < 1e-6


def test_state_message_fields_are_already_mm_deg():
    msg = SimpleNamespace(
        has_actual_tool_data=True,
        actual_tcp_x_mm=419.966,
        actual_tcp_y_mm=353.329,
        actual_tcp_z_mm=732.167,
        actual_rot_x_deg=-166.891,
        actual_rot_y_deg=55.656,
        actual_rot_z_deg=-122.934,
    )
    assert format_values(state_to_values(msg)) == (
        "419.966,353.329,732.167,-166.891,55.656,-122.934"
    )


def test_state_without_tool_data_is_rejected():
    msg = SimpleNamespace(has_actual_tool_data=False)
    try:
        state_to_values(msg)
    except ValueError as exc:
        assert "has_actual_tool_data" in str(exc)
    else:
        raise AssertionError("expected ValueError")


def test_static_pose_parsing_and_formatting():
    values = parse_static_values("1.0,2.0,3.0,4.0,5.0,6.0")
    assert format_values(values) == "1.000,2.000,3.000,4.000,5.000,6.000"


def test_static_pose_rejects_wrong_length():
    try:
        parse_static_values("1,2,3")
    except ValueError as exc:
        assert "6" in str(exc)
    else:
        raise AssertionError("expected ValueError")


def test_state_tcp_to_flange_matches_senior_offset():
    msg = SimpleNamespace(
        has_actual_tool_data=True,
        actual_tcp_x_mm=717.456,
        actual_tcp_y_mm=575.402,
        actual_tcp_z_mm=577.134,
        actual_rot_x_deg=-166.996,
        actual_rot_y_deg=55.545,
        actual_rot_z_deg=-116.286,
        actual_tcp_offset_x_mm=-46.0,
        actual_tcp_offset_y_mm=0.0,
        actual_tcp_offset_z_mm=353.0,
        actual_tcp_offset_rx_deg=0.0,
        actual_tcp_offset_ry_deg=0.0,
        actual_tcp_offset_rz_deg=0.0,
    )
    values = state_to_flange_values(msg)
    assert abs(values[0] - 509.116) < 0.5
    assert abs(values[1] - 332.963) < 0.5
    assert abs(values[2] - 733.795) < 0.5
