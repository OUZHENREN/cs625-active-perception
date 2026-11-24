"""接近工件必须"远处关节空间 + 近处笛卡尔直线"（2026-11-24 一天三次碰撞后）。

当天三次撞到工件的根因是：从远处用关节空间插值接近工件，终点虽然对，
路径上 TCP 会走出弧线（实测 target z=0.0895，机械臂却停在 z=0.2767、横摆 49 mm）。
手工流程已改成 `scripts/move_tcp_linear.py`（笛卡尔直线 + 点云间隙闸），
但 `run_pick_and_place.py` 的 pre_grasp / grasp 两个阶段仍在用 `node.plan_joint(...)`，
也就是说"效率模式"一旦跑到抓取位就会重演那次碰撞。

本测试锁住修复后的口径：
  · pre_grasp：关节空间只走到"预抓取位正上方 cartesian_radius_m"，最后一段笛卡尔；
  · grasp：完全笛卡尔下降（不再有关节空间规划）；
  · 半径来自 config/tasks/common.yaml 的 approach_motion，不写死。
"""

from __future__ import annotations

import pathlib
import re

import yaml

ROOT = pathlib.Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "scripts" / "run_pick_and_place.py").read_text(encoding="utf-8")
CONFIG = yaml.safe_load((ROOT / "config" / "tasks" / "common.yaml").read_text(encoding="utf-8"))


def block(header: str, stop: str) -> str:
    """截取一个阶段分支的代码，并把注释行去掉（注释里会提到旧写法）。"""
    start = SOURCE.index(header)
    end = SOURCE.index(stop, start)
    lines = [line for line in SOURCE[start:end].splitlines()
             if not line.strip().startswith("#")]
    return "\n".join(lines)


def test_grasp_phase_never_plans_in_joint_space():
    grasp = block('elif name == "grasp":', 'elif kind == "measure":')
    assert "cartesian_move(" in grasp, "抓取下降必须走笛卡尔直线"
    assert "plan_joint(" not in grasp, (
        "grasp 阶段又用关节空间规划了：150 mm 的垂直下降在关节空间里是弧线，"
        "会扫到工件（手册第 48 条）")


def test_pre_grasp_stages_above_the_part_and_then_goes_cartesian():
    pre = block('elif name == "pre_grasp":', 'elif name == "grasp":')
    assert "approach_staging_point(" in pre, "关节空间终点必须先抬到工件上方"
    assert "cartesian_move(pre" in pre, "最后一段必须是笛卡尔直线"
    # 关节空间的终点不能直接是预抓取点
    assert "node.ik(pre," not in pre, "关节空间不能直接奔向预抓取点"


def test_staging_height_comes_from_config():
    motion = CONFIG["approach_motion"]
    assert float(motion["cartesian_radius_m"]) > 0.0
    assert "cartesian_radius_m" in SOURCE, "半径必须从 approach_motion 读，不能写死"


def test_staging_point_is_radius_above_the_pre_grasp_point():
    helper = block("def approach_staging_point(", "def cartesian_move(")
    assert re.search(r"point\[2\] \+= float\(radius_m\)", helper), (
        "staging 点必须是预抓取点正上方 radius_m")
    assert "np.asarray(pre_xyz, dtype=float).copy()" in helper, "不能原地修改预抓取点"
