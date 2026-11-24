"""关节速度上限：默认 0.10，只有白名单里的"安全换位段"可到 0.20（2026-11-24）。

所有者当天批准：
  · "绝对安全的换位段可以加速" → 0.20 rad/s（11.5°/s）；
  · 但速度是安全红线，不允许调用方随手把上限抬高。

所以执行器自己守两道：
  ① 白名单 `SAFE_TRANSIT_PHASES`：只有这些阶段能提速；
  ② 绝对上限 `HARD_MAX_JOINT_SPEED_RAD_S`：请求超过它就被夹住，不会放行。
"""

from __future__ import annotations

import importlib.util
import math
import pathlib

import yaml

ROOT = pathlib.Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "exec_speed", ROOT / "scripts" / "execute_assembly_chunks.py")
executor = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(executor)

NAMES = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
         "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


def test_default_cap_is_the_original_contract():
    cap, _ = executor.resolve_speed_cap(executor.MAX_JOINT_SPEED_RAD_S, "grasp")
    assert cap == executor.MAX_JOINT_SPEED_RAD_S


def test_whitelisted_transit_phase_may_go_to_the_approved_speed():
    cap, note = executor.resolve_speed_cap(0.20, "pre_grasp")
    assert math.isclose(cap, 0.20)
    assert "安全换位段" in note


def test_contact_phase_is_never_accelerated_even_if_it_asks():
    for phase in ("grasp", "insert", "lift", "transit", "staging", ""):
        cap, note = executor.resolve_speed_cap(0.20, phase)
        assert cap == executor.MAX_JOINT_SPEED_RAD_S, phase
        assert "白名单" in note or "默认" in note


def test_requests_above_the_hard_ceiling_are_clamped_not_honoured():
    cap, note = executor.resolve_speed_cap(0.60, "observe:slot_coarse")
    assert math.isclose(cap, executor.HARD_MAX_JOINT_SPEED_RAD_S)
    assert "硬上限" in note


def test_governor_halves_the_duration_when_the_cap_doubles():
    def plan():
        q0 = [0.0] * 6
        q1 = [math.radians(20.0)] + [0.0] * 5
        # 4 段，每段 5°：高于 MIN_SEGMENT_SEC（0.25 s）地板，速度上限才起作用
        pts = [{"q": [a + (b - a) * i / 4 for a, b in zip(q0, q1)],
                "t": 1.0 * i / 4} for i in range(5)]
        return {"joint_names": NAMES, "points": pts}

    slow, _ = executor.govern_timestamps(plan(), 1.0, executor.MAX_JOINT_SPEED_RAD_S)
    fast, _ = executor.govern_timestamps(
        plan(), 1.0, executor.HARD_MAX_JOINT_SPEED_RAD_S)
    assert fast[-1] < slow[-1] * 0.6, (slow[-1], fast[-1])


def test_the_two_efficiency_flows_ask_for_the_approved_speed_only_on_transit():
    for name in ("load_efficient.yaml", "full_efficient.yaml"):
        cfg = yaml.safe_load((ROOT / "config" / "tasks" / name).read_text(encoding="utf-8"))
        speeds = cfg["phase_speeds"]
        accelerated = {k for k, v in speeds.items() if float(v.get("joint_speed", 0)) > 0}
        assert accelerated, f"{name}: 没有配置任何提速段"
        for key in accelerated:
            # 配置里的键是"观测位名字"，执行器收到的是带前缀的阶段名；
            # pre_grasp / baseline 是关节阶段，名字不变
            phase = key if key in ("pre_grasp", "baseline") else f"observe:{key}"
            assert phase in executor.SAFE_TRANSIT_PHASES, (
                f"{name}: {phase} 不在执行器白名单里，写了也不会生效")
        assert "grasp" not in accelerated and "insert" not in accelerated


def test_a_slower_request_is_honoured_not_raised():
    # 2026-11-24 audit：旧实现把任何 ≤0.10 的请求静默抬到 0.10
    cap, note = executor.resolve_speed_cap(0.05, "grasp")
    assert math.isclose(cap, 0.05)
    assert "更慢" in note


def _write_plan(tmp_path, degs, points=21):
    plan = {"joint_names": NAMES,
            "points": [{"q": [math.radians(v) * i / (points - 1) for v in degs],
                        "t": 6.0 * i / (points - 1)}
                       for i in range(points)]}
    f = tmp_path / "plan.json"
    f.write_text(__import__("json").dumps(plan))
    return str(f)


def test_whitelisted_transit_phase_runs_single_shot(tmp_path):
    path = _write_plan(tmp_path, [45.0] + [0.0] * 5)   # 45° 超过 25° 闸
    jobs, refused = executor.plan_jobs([path], single_shot_phase="observe:tables")
    assert refused is None
    assert len(jobs) == 1, "白名单换位段应单段执行"


def test_contact_phase_still_auto_splits(tmp_path):
    path = _write_plan(tmp_path, [45.0] + [0.0] * 5)
    jobs, refused = executor.plan_jobs([path], single_shot_phase="grasp")
    assert refused is None
    assert len(jobs) > 1, "接触/装配段必须保留 25° 自动分段"


def test_joint_deltas_use_cumulative_travel_not_net():
    pts = [{"q": [0.0] * 6, "t": 0.0},
           {"q": [math.radians(20.0)] + [0.0] * 5, "t": 1.0},
           {"q": [math.radians(-20.0)] + [0.0] * 5, "t": 2.0}]
    deltas = executor.joint_deltas({"joint_names": NAMES, "points": pts})
    # 净变化只有 20°，累计行程是 60°：摆出去再摆回来不能绕过 25° 闸
    assert math.isclose(deltas["shoulder_pan_joint"], math.radians(60.0), abs_tol=1e-6)
