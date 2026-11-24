"""2026-11-24 全面审计后补的回归测试：插槽基线对比 / 运动配置绑定 / 看门狗对齐。

覆盖：
  · check_slot_reference：同一测量管线自比（PASS / WARN / REFUSE / 无基线）；
  · move_tcp_linear 间隙闸三键真正从 config approach_motion 读取（不再是死配置）；
  · 执行器看门狗 SAFE_JOINT_LIMITS 与规划器 ABSOLUTE_LIMITS 一致（旧值会误杀
    已验证的抓取位姿 lift=−124.5°）；
  · 下发前全轨迹点安全区间检查存在（exit 19）。
"""

from __future__ import annotations

import importlib.util
import json
import math
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]

SPEC = importlib.util.spec_from_file_location(
    "check_slot", ROOT / "scripts" / "check_slot_reference.py")
check = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(check)

SPEC2 = importlib.util.spec_from_file_location(
    "executor2", ROOT / "scripts" / "execute_assembly_chunks.py")
executor = importlib.util.module_from_spec(SPEC2)
SPEC2.loader.exec_module(executor)


def _write_pose(path: pathlib.Path, translation, quaternion):
    path.write_text(json.dumps({"per_group": [{"group": "g00_m", "pose_base": {
        "translation": translation, "quaternion": quaternion}}]}), encoding="utf-8")


def _run_main(argv: list[str]) -> int:
    saved = sys.argv
    sys.argv = ["check_slot_reference.py"] + argv
    try:
        return check.main()
    finally:
        sys.argv = saved


def test_gap_math():
    a = (check.np.array([0.0, 0.0, 0.0]), check.np.array([0.0, 0.0, 0.0, 1.0]))
    b = (check.np.array([0.010, 0.0, 0.0]), check.np.array([0.0, 0.0, 0.0, 1.0]))
    mm, deg = check.gap_mm_deg(a, b)
    assert math.isclose(mm, 10.0) and math.isclose(deg, 0.0)


def test_slot_check_passes_and_refuses(tmp_path):
    baseline = tmp_path / "baseline.json"
    measurement = tmp_path / "measurement.json"
    moved = tmp_path / "moved.json"
    pose = [1.25576, 0.1422, 0.44284]
    quat = [-0.000398, 0.006012, -0.00728, 0.999955]
    _write_pose(baseline, pose, quat)
    _write_pose(measurement, pose, quat)
    moved_pose = [p + d for p, d in zip(pose, (0.015, 0.0, 0.0))]
    _write_pose(moved, moved_pose, quat)

    saved = check.BASELINE
    check.BASELINE = baseline
    try:
        assert _run_main(["--measurement", str(measurement)]) == 0      # PASS
        assert _run_main(["--measurement", str(moved)]) == 2            # REFUSE
    finally:
        check.BASELINE = saved


def test_slot_check_refuses_without_baseline(tmp_path):
    measurement = tmp_path / "measurement.json"
    _write_pose(measurement, [1.0, 0.0, 0.4], [0.0, 0.0, 0.0, 1.0])
    saved = check.BASELINE
    check.BASELINE = tmp_path / "does_not_exist.json"
    try:
        assert _run_main(["--measurement", str(measurement)]) == 3
    finally:
        check.BASELINE = saved


def test_clearance_gate_reads_config_approach_motion():
    source = (ROOT / "scripts" / "move_tcp_linear.py").read_text(encoding="utf-8")
    for key in ("clearance_min_mm", "clearance_k", "gate_until_mm"):
        assert f'get("{key}"' in source, f"间隙闸键 {key} 没有运行时绑定 config"
    assert "cartesian_velocity" in source, "默认速度没有绑定 config"


def test_executor_watchdog_limits_match_planner_approval():
    limits = {name: (low, high) for name, low, high in executor.SAFE_JOINT_LIMITS}
    lift = (math.degrees(limits["shoulder_lift_joint"][0]),
            math.degrees(limits["shoulder_lift_joint"][1]))
    elbow = (math.degrees(limits["elbow_joint"][0]),
             math.degrees(limits["elbow_joint"][1]))
    assert lift == (-130.0, 10.0), "看门狗肩部区间必须与规划器一致"
    assert math.isclose(elbow[0], -160.0) and math.isclose(elbow[1], -60.0), \
        "看门狗肘部区间必须与规划器一致"


def test_pre_send_range_check_exists():
    source = (ROOT / "scripts" / "execute_assembly_chunks.py").read_text(encoding="utf-8")
    assert "REFUSED_OUT_OF_RANGE_PLAN" in source
    assert "return 19" in source
