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


def move_to(xyz) -> bool:
    command = [sys.executable, str(REPO / "scripts" / "move_tcp_linear.py"),
               f"{xyz[0]:.4f}", f"{xyz[1]:.4f}", f"{xyz[2]:.4f}", "70", "0.05", "2.0"]
    completed = subprocess.run(command, capture_output=True, text=True, timeout=600)
    lines = (completed.stdout or "").strip().splitlines()
    print("   移动:", lines[-1] if lines else "(no output)")
    return completed.returncode == 0


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
    parser.add_argument("--offsets-mm", default="-280,-200,-120,120,200,280")
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
    left = np.array([-view[1], view[0], 0.0])
    print("视线(水平) =", np.round(view, 3), " 左方向 =", np.round(left, 3))

    tried = False
    for offset_mm in [int(v) for v in args.offsets_mm.split(",") if v.strip()]:
        target = start + left * (offset_mm / 1000.0)
        target[2] = start[2]
        print("尝试横向 %+d mm -> TCP %s" % (offset_mm, np.round(target, 3)))
        if not args.execute:
            continue
        tried = True
        if not move_to(target):
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
