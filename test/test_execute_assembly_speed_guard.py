"""离线测试：执行器的速度治理与行程硬闸（2026-11-18 事故后新增）。

事故：一份关节行程 47.6° 的计划被直接下发，机械臂最终停在"2 倍指令位移"的越界姿态
（J1 128.7°、J6 156.8°），随后驱动失联。所有者要求：速度与行程必须由代码强制，
不允许任何逾越。

这些测试只测纯函数（不连 ROS、不动机械臂）。
"""

from __future__ import annotations

import importlib.util
import math
import pathlib

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "execute_assembly_chunks", ROOT / "scripts" / "execute_assembly_chunks.py"
)
executor = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(executor)

NAMES = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
         "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
         "gripper_left_finger_joint", "gripper_right_finger_joint"]


def plan(points_deg: list[list[float]], times: list[float]) -> dict:
    """points_deg: 每个点 6 个手臂关节角（度）；夹爪补 0。"""
    points = []
    for angles, stamp in zip(points_deg, times):
        q = [math.radians(a) for a in angles] + [0.0, 0.0]
        points.append({"q": q, "t": stamp})
    return {"joint_names": NAMES, "points": points}


def test_governor_never_plays_faster_than_the_plan():
    data = plan([[0, 0, 0, 0, 0, 0], [10, 0, 0, 0, 0, 0]], [0.0, 10.0])
    times, info = executor.govern_timestamps(data, 1.0)
    assert times[-1] == pytest.approx(10.0)          # 计划本来就慢，不改
    assert info["max_speed"] <= executor.MAX_JOINT_SPEED_RAD_S + 1e-9


def test_governor_slows_down_a_fast_plan():
    # 10° 在 0.1 s 内完成 → 100 °/s，必须被压到 2.9 °/s 量级
    data = plan([[0, 0, 0, 0, 0, 0], [10, 0, 0, 0, 0, 0]], [0.0, 0.1])
    times, info = executor.govern_timestamps(data, 1.0)
    assert times[-1] >= math.radians(10) / executor.MAX_JOINT_SPEED_RAD_S - 1e-9
    assert info["max_speed"] <= executor.MAX_JOINT_SPEED_RAD_S + 1e-9


def test_governor_keeps_the_worst_joint_under_the_cap():
    # 只有 wrist_2 动 20°，其余不动
    data = plan([[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 20, 0]], [0.0, 0.2])
    times, info = executor.govern_timestamps(data, 3.0)
    assert info["max_speed"] <= executor.MAX_JOINT_SPEED_RAD_S + 1e-9
    assert times[-1] > 3.0          # time_scale 只会更慢


def test_time_scale_below_one_cannot_speed_up_the_governor():
    data = plan([[0, 0, 0, 0, 0, 0], [5, 0, 0, 0, 0, 0]], [0.0, 1.0])
    slow, _ = executor.govern_timestamps(data, 1.0)
    forced, _ = executor.govern_timestamps(data, 0.1)   # 即使传 <1 也不会更快
    assert forced[-1] == pytest.approx(slow[-1])


def test_joint_deltas_ignores_fingers_and_reports_the_worst():
    data = plan([[0, 0, 0, 0, 0, 0], [30, 0, 0, 0, -47.6, 0]], [0.0, 6.0])
    deltas = executor.joint_deltas(data)
    assert "gripper_left_finger_joint" not in deltas
    assert deltas["wrist_2_joint"] == pytest.approx(math.radians(47.6), rel=1e-6)


def test_large_delta_is_above_the_hard_limit():
    """2026-11-18 那份计划（wrist_2 47.6°）必须被判超限。"""
    data = plan([[45.1, -59.6, -151.9, 0.5, 96.8, 88.5],
                 [86.0, -55.0, -151.4, -16.1, 49.2, 121.9]], [0.0, 6.0])
    deltas = executor.joint_deltas(data)
    assert max(deltas.values()) > executor.MAX_JOINT_DELTA_RAD


def test_the_governor_is_the_only_speed_authority():
    """2026-11-20：time_scale 不再叠加，否则实际速度变成上限的 1/3。

    事故：调用方习惯性传 --time-scale 3.0，J4 转 94° 花了 98 s（0.95°/s），
    所有者反馈"太慢"。现在治理器是唯一权威。
    """
    data = plan([[0, 0, 0, 0, 0, 0], [30, 0, 0, 0, 0, 0]], [0.0, 0.1])
    times_default, info_default = executor.govern_timestamps(data, 1.0)
    times_scaled, info_scaled = executor.govern_timestamps(data, 3.0)
    assert times_default[-1] == pytest.approx(times_scaled[-1])
    assert info_default["max_speed"] <= executor.MAX_JOINT_SPEED_RAD_S + 1e-9
    assert info_scaled["max_speed"] <= executor.MAX_JOINT_SPEED_RAD_S + 1e-9


def test_a_thirty_degree_move_takes_the_expected_time():
    """30° 在 0.05 rad/s 下应当约 10.5 s（不是 31 s）。"""
    data = plan([[0, 0, 0, 0, 0, 0], [30, 0, 0, 0, 0, 0]], [0.0, 6.0])
    _times, info = executor.govern_timestamps(data, 1.0)
    assert info["exec_duration"] == pytest.approx(10.47, abs=0.3)
