#!/usr/bin/env python3
"""NBV（平移式）：保持工具姿态不变，把相机整体左右平移，用侧面补足偏航约束。

为什么用平移而不是转腕（2026-11-12 所有者指出的关键）
----------------------------------------------------
* 大角度转腕会让末端法兰和机械臂本体自碰撞（已经发生一次）；
* J6 相对当前角度只能 ±45°，否则电机与相机线缆缠绕；
* 但前三个轴可以带着整条手臂平移：工具姿态不变，相机就整体横移，
  侧面的可见性随之改变——这正是"把相机往左移"的物理含义。

每次移动后测 1 组做验收（扶手 >=0.70 且覆盖率 >=0.90 即通过），
全部不合格则回到起始位置并报告。
"""

from __future__ import annotations

import argparse
import json
import pathlib
import subprocess
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

REPO = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))
import nbv_score as nbv  # noqa: E402

PROBE_OUT = pathlib.Path("/tmp/nbv_lateral_probe.json")
ARM = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


def current_joints(node) -> dict:
    got: dict = {}
    node.create_subscription(JointState, "/joint_states",
                             lambda msg: got.setdefault("j", msg), 10)
    import time
    deadline = time.time() + 5
    while time.time() < deadline and "j" not in got:
        rclpy.spin_once(node, timeout_sec=0.2)
    msg = got["j"]
    names = list(msg.name)
    return {name: float(msg.position[names.index(name)]) for name in ARM}


def plan_and_move(tcp_now, target_xyz, max_wrist_deg: float) -> bool:
    """平移 + 小幅转腕瞄准工件中心；规划器会拒绝腕部超限/碰撞的方案。

    所有者 2026-11-12：纯平移不够灵活，允许后三轴"适当转动"，但绝不允许大幅转动。
    所以这里把腕部变化上限收到 max_wrist_deg（默认 25°），并依赖 plan_pose_once 的
    腕部限位 + /check_state_validity 抽样复核这两道闸。
    """
    import numpy as _np
    from scipy.spatial.transform import Rotation as _R
    handeye = json.loads((REPO / "config/handeye/tcp_to_camera.json").read_text())
    T_tcp_cam = _np.array(handeye["T_tcp_cam"])
    camera_now = tcp_now @ T_tcp_cam
    # 目标相机位置 = 目标 TCP 平移同样的世界位移（姿态先沿用当前）
    delta = _np.asarray(target_xyz, float) - tcp_now[:3, 3]
    camera_target = camera_now[:3, 3] + delta
    centre = _part_centre()
    axis = centre - camera_target
    axis /= max(_np.linalg.norm(axis), 1e-9)
    current_axis = camera_now[:3, :3][:, 2]
    cross = _np.cross(current_axis, axis)
    sine = float(_np.linalg.norm(cross))
    cosine = float(_np.dot(current_axis, axis))
    swing = (_np.eye(3) if sine < 1e-9
             else _R.from_rotvec(cross / sine * _np.arctan2(sine, cosine)).as_matrix())
    rotation = swing @ tcp_now[:3, :3]
    tcp_target = target_xyz - rotation @ T_tcp_cam[:3, 3] + (camera_target - camera_target)
    # 注意：TCP = 相机位置 − R·t_tcp_cam
    tcp_target = camera_target - rotation @ T_tcp_cam[:3, 3]
    quat = _R.from_matrix(rotation).as_quat()
    command = [sys.executable, str(REPO / "scripts" / "plan_pose_once.py"),
               *[f"{v:.4f}" for v in tcp_target], *[f"{v:.5f}" for v in quat],
               "--max-wrist-delta", f"{max_wrist_deg:.1f}",
               "--output", "/tmp/nbv_lateral_plan.json"]
    completed = subprocess.run(command, capture_output=True, text=True, timeout=300)
    lines = (completed.stdout or "").strip().splitlines()
    last = lines[-1] if lines else ""
    print("   规划:", last)
    if "code 1 " not in (completed.stdout or ""):
        return False
    moved = subprocess.run(
        [sys.executable, str(REPO / "scripts" / "execute_assembly_chunks.py"),
         "/tmp/nbv_lateral_plan.json", "--time-scale", "2.0", "--dwell", "0.5",
         "--goal-tolerance", "0.005"], capture_output=True, text=True, timeout=900)
    lines = (moved.stdout or "").strip().splitlines()
    print("   执行:", lines[-1] if lines else "")
    return moved.returncode == 0


def move_to(xyz, retries: int = 2) -> bool:
    """直线平移到目标位置；遇到 'resend succeeded' 时重试（外部控制被顶掉后已自动恢复）。"""
    command = [sys.executable, str(REPO / "scripts" / "move_tcp_linear.py"),
               f"{xyz[0]:.4f}", f"{xyz[1]:.4f}", f"{xyz[2]:.4f}", "70", "0.05", "2.0"]
    for attempt in range(retries + 1):
        completed = subprocess.run(command, capture_output=True, text=True, timeout=600)
        lines = (completed.stdout or "").strip().splitlines()
        last = lines[-1] if lines else "(no output)"
        print(f"   移动({attempt + 1}/{retries + 1}):", last)
        if completed.returncode == 0:
            return True
        if "resend succeeded" not in last:
            return False
    return False


def _part_centre():
    payload = pathlib.Path(REPO / "runtime/measurements/workpiece_coarse.json")
    if payload.exists():
        data = json.loads(payload.read_text())
        return np.array(data["pose_base_centre_mm"]) / 1000.0
    return nbv.read_tcp()[:3, 3] + np.array([0.0, 0.25, -0.4])


def probe():
    command = [sys.executable, str(REPO / "scripts" / "measure_object.py"),
               "--preset", "workpiece", "--stage", "coarse", "--groups", "1",
               "--out", str(PROBE_OUT)]
    subprocess.run(command, capture_output=True, text=True, timeout=600)
    try:
        data = json.loads(PROBE_OUT.read_text())
    except (OSError, ValueError):
        return False, 0.0, 0.0
    handles = [(group.get("symmetry_check") or {}).get("handle_score") or 0.0
               for group in data.get("per_group", [])]
    coverage = float(data.get("coverage_median") or 0.0)
    best = max(handles) if handles else 0.0
    return (best >= 0.70 and coverage >= 0.90), best, coverage


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--offsets-mm", default="120,200,280,-120,-200,-280",
                        help="横向偏移（mm）；正号=沿 left（画面左侧）移动。"
                             "默认先试正侧——所有者 2026-11-12：先向画面左侧移动才拍得到工件左侧面")
    parser.add_argument("--max-wrist-deg", type=float, default=25.0,
                        help="后三轴允许的最大变化（度）；超过由规划器直接拒绝")
    parser.add_argument("--time-budget-sec", type=float, default=120.0,
                        help="整个 NBV 的时间预算（秒）；超时立即停止并报告，不再无意义等待")
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("nbv_lateral")
    state = current_joints(node)
    print("当前 J6 = %+.1f deg" % np.degrees(state["wrist_3_joint"]))

    handeye = json.loads((REPO / "config/handeye/tcp_to_camera.json").read_text())
    T_tcp_cam = np.array(handeye["T_tcp_cam"])
    tcp_now = nbv.read_tcp()
    if tcp_now is None:
        print("读不到 TCP", file=sys.stderr)
        return 2
    camera = tcp_now @ T_tcp_cam
    start = tcp_now[:3, 3].copy()

    payload = pathlib.Path("runtime/measurements/workpiece_coarse.json")
    if payload.exists():
        data = json.loads(payload.read_text())
        centre = np.array(data["pose_base_centre_mm"]) / 1000.0
    else:
        centre = start + np.array([0.0, 0.25, -0.4])
    view = centre - camera[:3, 3]
    view[2] = 0.0
    view /= max(np.linalg.norm(view), 1e-9)
    # 所有者 2026-11-12：这里的"左"是【当前相机画面里工件的左侧】。
    # 相机朝 view 看时，画面左方 = view × 世界 z（右手系），不是 z × view。
    left = np.cross(view, np.array([0.0, 0.0, 1.0]))
    # 所有者 2026-11-12：按工件的偏转角决定优先方向——
    # 转角为负（例如 −30°）优先往画面左侧找；转角为正优先往画面右侧找。
    from scipy.spatial.transform import Rotation as _R
    import yaml as _yaml
    yaw_deg = None
    try:
        cfg = _yaml.safe_load((REPO / "config/tasks/common.yaml").read_text())
        # 0° 基准 = 工件摆正时实测的那一帧（所有者 2026-11-12 指定）；没有则退回示教参考
        ref_q = np.array(cfg.get("workpiece_zero_quat")
                         or cfg["workpiece_reference"]["quat"], dtype=float)
        meas_q = np.array(json.loads((REPO / "runtime/measurements/workpiece_coarse.json")
                                     .read_text())["per_group"][0]["pose_base"]["quaternion"])
        delta = _R.from_quat(meas_q) * _R.from_quat(ref_q).inv()
        yaw_deg = float(np.degrees(delta.as_euler("zyx")[0]))
    except Exception:  # noqa: BLE001 - 读不到就按当前顺序
        yaw_deg = None
    if yaw_deg is not None and 0.0 < yaw_deg <= 45.0:
        print("工件偏转 %+.1f°（正角度且在 45° 内）→ 当前粗观测位通常三面齐全，跳过 NBV"
              % yaw_deg)
        return 0
    if yaw_deg is not None:
        prefer_left = yaw_deg >= 0
        order = args.offsets_mm.split(",")
        pos = [v for v in order if not v.strip().startswith("-")]
        neg = [v for v in order if v.strip().startswith("-")]
        args.offsets_mm = ",".join(pos + neg if prefer_left else neg + pos)
        print("工件偏转 %.1f° → 优先试%s侧（正=画面左，负=画面右）"
              % (yaw_deg, "左" if prefer_left else "右"))
    print("视线(水平) =", np.round(view, 3), " 左方向 =", np.round(left, 3))
    print("提示：正偏移 = 沿画面左侧；负偏移 = 沿画面右侧。默认先试正侧。")

    # 出发前先确认"当前视角里真的有工件"：否则横移只是在盲走。
    # 2026-11-12 的教训：机械臂停在插槽一侧时直接跑 NBV，探针覆盖只有 0.31，
    # 继续横移还撞出了 PROTECTIVE_STOP。
    if args.execute:
        ok_now, handle_now, coverage_now = probe()
        print("出发前探针：扶手 %.2f 覆盖 %.3f" % (handle_now, coverage_now))
        if coverage_now < 0.60:
            print("当前视角看不到工件（覆盖率 %.3f），拒绝移动；"
                  "请先把机械臂移到工件观测位" % coverage_now)
            return 5

    import time as _time
    started = _time.time()
    tried = False
    for offset_mm in [int(v) for v in args.offsets_mm.split(",") if v.strip()]:
        target = start + left * (offset_mm / 1000.0)
        target[2] = start[2]
        print("尝试横向 %+d mm -> TCP %s" % (offset_mm, np.round(target, 3)))
        if not args.execute:
            continue
        spent = _time.time() - started
        if spent > args.time_budget_sec:
            print("已用 %.0f s，超过预算 %.0f s，停止（算法该改，不该继续等）"
                  % (spent, args.time_budget_sec))
            return 6
        tried = True
        moved = plan_and_move(tcp_now, target, args.max_wrist_deg)
        if not moved:
            print("   平移+微调被拒/失败，退回纯平移")
            moved = move_to(target)
        if not moved:
            print("   移动失败，试下一个")
            continue
        good, handle, coverage = probe()
        print("   探针：扶手 %.2f 覆盖 %.3f -> %s" % (handle, coverage,
                                                     "可用" if good else "不可用"))
        if good:
            print("选定横向 %+d mm" % offset_mm)
            return 0
    if args.execute:
        if tried:
            print("都不合格，回到起始位置")
            move_to(start)
        return 3
    print("（未执行）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
