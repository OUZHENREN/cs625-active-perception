"""离线测试：超行程计划自动分段（所有者 2026-11-19）。

要求：计划单关节行程超过 25° 硬闸、但终点仍在绝对角度区间内时，执行器应自己分段，
不该让人工手工拆。分段必须：
  · 每段任一关节累计行程 ≤ 25°；
  · 相邻段共享边界点（拼起来与原轨迹重合）；
  · 每段时间戳从 0 开始单调递增；
  · 终点与原始计划终点一致。
"""

from __future__ import annotations

import importlib.util
import math
import pathlib

SPEC = importlib.util.spec_from_file_location(
    "exec_split", pathlib.Path(__file__).resolve().parents[1]
    / "scripts" / "execute_assembly_chunks.py")
executor = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(executor)

NAMES = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
         "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
         "gripper_left_finger_joint", "gripper_right_finger_joint"]


def make_plan(start_deg, end_deg, points=21, duration=6.0) -> dict:
    q0 = [math.radians(v) for v in start_deg] + [0.0, 0.0]
    q1 = [math.radians(v) for v in end_deg] + [0.0, 0.0]
    out = []
    for i in range(points):
        a = i / (points - 1)
        out.append({"q": [q0[k] + (q1[k] - q0[k]) * a for k in range(len(q0))],
                    "t": duration * a})
    return {"joint_names": NAMES, "points": out}


def test_a_large_move_is_split_into_legal_chunks():
    # 肩转 40°，腕2 −36° → 都超过 25°
    plan = make_plan([0, -60, -150, 0, 100, 80], [40, -60, -150, 0, 64, 80])
    chunks = executor.split_plan(plan, executor.MAX_JOINT_DELTA_RAD)
    assert len(chunks) >= 2
    for chunk in chunks:
        deltas = executor.joint_deltas(chunk)
        assert max(deltas.values()) <= executor.MAX_JOINT_DELTA_RAD + 1e-9


def test_chunks_are_contiguous_and_keep_the_endpoint():
    plan = make_plan([0, -60, -150, 0, 100, 80], [40, -60, -150, 0, 64, 80])
    chunks = executor.split_plan(plan, executor.MAX_JOINT_DELTA_RAD)
    for previous, following in zip(chunks, chunks[1:]):
        assert previous["points"][-1]["q"] == following["points"][0]["q"]
    assert chunks[-1]["points"][-1]["q"] == plan["points"][-1]["q"]


def test_each_chunk_starts_at_time_zero_and_increases():
    plan = make_plan([0, -60, -150, 0, 100, 80], [40, -60, -150, 0, 64, 80])
    for chunk in executor.split_plan(plan, executor.MAX_JOINT_DELTA_RAD):
        times = [point["t"] for point in chunk["points"]]
        assert times[0] == 0.0
        assert all(b > a for a, b in zip(times, times[1:]))


def test_a_single_coarse_segment_is_densified_and_split():
    """只有两个点、跨度 60°：必须先加密再分段，不能一段带走 60°。"""
    plan = make_plan([0, -60, -150, 0, 100, 80], [60, -60, -150, 0, 100, 80], points=2)
    chunks = executor.split_plan(plan, executor.MAX_JOINT_DELTA_RAD)
    assert len(chunks) >= 3
    for chunk in chunks:
        assert max(executor.joint_deltas(chunk).values()) \
            <= executor.MAX_JOINT_DELTA_RAD + 1e-9


def test_a_small_move_stays_one_chunk():
    plan = make_plan([0, -60, -150, 0, 100, 80], [10, -60, -150, 0, 100, 80])
    chunks = executor.split_plan(plan, executor.MAX_JOINT_DELTA_RAD)
    assert len(chunks) == 1


def test_plan_jobs_reads_glob_strings_and_splits(tmp_path):
    """glob.glob 返回字符串；plan_jobs 必须能读，并自动分段。

    2026-11-19 真机踩过：分段改动把 read_text 用在了字符串上，执行器第一步就崩，
    机械臂一步都没动（好在这类失败不会发出任何轨迹）。
    """
    plan = make_plan([0, -60, -150, 0, 100, 80], [40, -60, -150, 0, 64, 80])
    path = tmp_path / "plan.json"
    path.write_text(__import__("json").dumps(plan), encoding="utf-8")

    jobs, refused = executor.plan_jobs([str(path)])        # 故意传字符串，模拟 glob
    assert refused is None
    assert len(jobs) >= 2
    assert jobs[0][0].startswith("plan.json#1/")


def test_plan_jobs_refuses_when_auto_split_is_disabled(tmp_path):
    plan = make_plan([0, -60, -150, 0, 100, 80], [40, -60, -150, 0, 64, 80])
    path = tmp_path / "plan.json"
    path.write_text(__import__("json").dumps(plan), encoding="utf-8")
    jobs, refused = executor.plan_jobs([str(path)], no_auto_split=True)
    assert refused == 13
    assert jobs == []


def test_plan_jobs_keeps_a_small_plan_as_one_job(tmp_path):
    plan = make_plan([0, -60, -150, 0, 100, 80], [10, -60, -150, 0, 100, 80])
    path = tmp_path / "plan.json"
    path.write_text(__import__("json").dumps(plan), encoding="utf-8")
    jobs, refused = executor.plan_jobs([str(path)])
    assert refused is None and len(jobs) == 1
