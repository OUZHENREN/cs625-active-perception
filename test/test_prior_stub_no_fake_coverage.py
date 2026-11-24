"""先验顶替实拍时必须留下可识别的证据缺口（2026-11-24，所有者批准 B）。

背景：粗观测阶段的命令带 ``--prior auto --prior-max-age-min 30``，先验文件
（上一次成功测量的工件位姿）在 30 分钟内时代码直接 return 0，**相机一帧都不采**，
再写一个占位 JSON 交差。旧占位 JSON 里写的是
``coverage_median = float(payload.get("coverage", 0.97))``——先验文件里根本没有
coverage 字段，于是永远是 0.97。下游（面板、登记工具）看到"覆盖率 0.97"，
实际上一张照片都没有。

现在：先验只作**配准引导**（``--prior-max-age-min 0``，见 config/tasks/*_efficient.yaml），
万一再走捷径，占位记录必须显式写 ``coverage_median = None`` + ``measured = False``，
并且不能被登记成示教位（``add_station_from_measurement`` 遇到 None 会拒绝）。
"""

from __future__ import annotations

import importlib.util
import pathlib

ROOT = pathlib.Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "measure_object", ROOT / "scripts" / "measure_object.py")
measure = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(measure)

POSE = {"translation": [0.65266, 0.47846, 0.11803],
        "quaternion": [-0.074199, 0.997236, 0.003477, 0.001499]}


def test_stub_does_not_fabricate_coverage():
    payload = measure.prior_stub_payload(POSE, 4.4)
    assert payload["coverage_median"] is None, "先验顶替实拍时不得编造覆盖率"
    assert payload["measured"] is False
    assert payload["source"] == "prior"
    assert payload["prior_age_min"] == 4.4
    assert payload["per_group"][0]["pose_fitness"] is None


def test_stub_keeps_the_pose_so_downstream_can_still_use_it():
    payload = measure.prior_stub_payload(POSE, 4.4)
    assert payload["per_group"][0]["pose_base"] == POSE
    assert payload["verdict"] == "OK"


def test_efficiency_flows_never_substitute_a_prior_for_a_capture():
    for name in ("load_efficient.yaml", "full_efficient.yaml"):
        text = (ROOT / "config" / "tasks" / name).read_text(encoding="utf-8")
        assert "--prior-max-age-min 30" not in text, (
            f"{name} 仍会把 30 分钟内的先验顶替成实拍")
        assert text.count("--prior-max-age-min 0") == 2, name


def test_coarse_is_two_groups_of_two_frames_and_fine_is_unchanged():
    """所有者 2026-11-24：粗观测 2 组×2 帧提速，精观测保持 3 组×3 帧。"""
    import yaml

    for name in ("load_efficient.yaml", "full_efficient.yaml"):
        cfg = yaml.safe_load((ROOT / "config" / "tasks" / name).read_text(encoding="utf-8"))
        checks = cfg["observe_checks"]
        coarse = [v for k, v in checks.items() if k.startswith("workpiece_coarse")]
        assert coarse, name
        for command in coarse:
            assert "--stage coarse" in command, (name, command)
            assert "--groups 2" in command and "--frames 2" in command, (name, command)
        fine = checks["workpiece_fine"]
        assert "--stage fine" in fine and "--groups 3" in fine, (name, fine)
        assert "--frames" not in fine, (name, fine)
