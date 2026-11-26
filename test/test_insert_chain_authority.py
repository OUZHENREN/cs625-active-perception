"""插入链只有一个权威：11-04 手动验证的插槽参考位姿（所有者 2026-11-25）。

背景：效率模式原来直接用 config 常量 insert / pre_insert，而示教链是从
`runtime/slot_reference_verified.json` 推导。两者实测相差
  pre_insert 110.3 mm（缺了 +100 mm 抬高）
  insert      43.2 mm（Δy −41.6）
槽内每侧余量只有约 1.5°，在真机上直接跑效率模式插入必撞。

现在效率模式调 `reference_insert_chain()`，与示教链共用 pose_math.insert_from_slot，
本测试锁住：
  · 三个位姿确实由参考位姿推导（改参考 → 输出跟着变）；
  · 与示教链的推导逐位一致（同一口径）；
  · config 里的常量与推导结果一致（避免两处数字打架）。
"""

from __future__ import annotations

import importlib.util
import json
import pathlib
import sys

import numpy as np
import yaml

ROOT = pathlib.Path(__file__).resolve().parents[1]

SPEC = importlib.util.spec_from_file_location(
    "rpp_insert", ROOT / "scripts" / "run_pick_and_place.py")
rpp = importlib.util.module_from_spec(SPEC)
sys.modules["rpp_insert"] = rpp        # dataclass + __future__ annotations 需要
SPEC.loader.exec_module(rpp)

sys.path.insert(0, str(ROOT / "scripts" / "teaching"))
import pose_math as pm                 # noqa: E402


def _manual_chain(slot_xyz, slot_quat):
    """按示教链的口径手算一遍，作为独立对照。"""
    from scipy.spatial.transform import Rotation as R
    flat, tilted, pre = pm.insert_from_slot(slot_xyz, slot_quat,
                                            tilt_deg=-10.0, extra_height_m=0.100)

    def retract(pose, d=0.010):
        back = R.from_quat(pose[1]).as_matrix() @ np.array([d, 0.0, 0.0])
        return np.asarray(pose[0], float) - back

    return retract(pre), retract(tilted), retract(flat)


def test_derived_chain_matches_the_teaching_chain_exactly():
    """2026-11-26 起单一权威 = config slot_origin（不再是 runtime 参考文件）。"""
    cfg = yaml.safe_load((ROOT / "config" / "tasks" / "common.yaml").read_text())
    pre, tilted, flat = _manual_chain(cfg["slot_origin"], cfg["slot_origin_q"])
    chain = rpp.reference_insert_chain()
    assert np.allclose(np.asarray(chain["pre_insert"][0], float), pre, atol=1e-9)
    assert np.allclose(np.asarray(chain["insert"][0], float), tilted, atol=1e-9)
    assert np.allclose(np.asarray(chain["straighten"][0], float), flat, atol=1e-9)


def test_chain_follows_the_slot_origin_not_a_hardcoded_value(tmp_path, monkeypatch):
    """把 slot_origin 整体平移 20 mm，推导结果必须跟着动 20 mm（说明没有写死）。"""
    base = rpp.reference_insert_chain()
    cfg = yaml.safe_load((ROOT / "config" / "tasks" / "common.yaml").read_text())
    moved = dict(cfg)
    moved["slot_origin"] = [float(cfg["slot_origin"][0]) + 0.020] + list(cfg["slot_origin"][1:])
    fake_cfg = tmp_path / "common.yaml"
    fake_cfg.write_text(yaml.safe_dump(moved), encoding="utf-8")
    monkeypatch.setattr(rpp, "COMMON_CONFIG", fake_cfg, raising=False)
    before = rpp.reference_insert_chain()
    delta = np.asarray(before["insert"][0], float) - np.asarray(base["insert"][0], float)
    assert np.allclose(delta, [0.020, 0.0, 0.0], atol=1e-6), delta


def test_config_constants_agree_with_the_derived_chain():
    cfg = yaml.safe_load((ROOT / "config" / "tasks" / "common.yaml").read_text())
    chain = rpp.reference_insert_chain()
    assert np.allclose(np.asarray(cfg["pre_insert"], float),
                       np.asarray(chain["pre_insert"][0], float), atol=1e-4)
    assert np.allclose(np.asarray(cfg["insert"], float),
                       np.asarray(chain["insert"][0], float), atol=1e-4)
    # 倾斜进槽姿态（w≈0.088）不能退化成平放（w≈0.0）
    assert abs(float(cfg["pre_insert_q"][3])) > 0.05
