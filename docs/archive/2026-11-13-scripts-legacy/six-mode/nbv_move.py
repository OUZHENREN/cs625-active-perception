#!/usr/bin/env python3
"""NBV：把机械臂移到"能看到手柄面 + 一个侧面"的精观测位，并闭环验收。

为什么这么写（2026-11-12 的教训）
---------------------------------
* 只按"可见面积"打分选视角，在真机上会翻车：选出来的视角有时 1 组都测不出。
  所以必须【到位 → 测 1 组 → 不合格就换下一个候选】。
* 代价函数里只要有"少动"的奖励，优化器就会选"完全不动"的候选（实测 Δ方位 = 0°），
  于是机械臂只是上下动。所以"最小横向位移"要当硬约束（|Δ方位| ≥ 25°）。
* J6 相对当前角度变化必须 ≤ ±45°，否则电机与相机的线会缠绕；
  前三轴可以大范围移动（真正的换位主力），后三轴只做微调。
* 为了让画面别横着/倒着，姿态用"从当前姿态到目标光轴的最短弧摆动"（保滚转），
  再叠几种滚转（0/±45/±90/180）逐个试。

用法
----
    python3 scripts/nbv_move.py --measurements runtime/measurements/workpiece_coarse.json \
        --group g00_m --execute
不带 --execute 时只评分 + 规划，不移动。
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import subprocess
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState

REPO = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))
import coarse_pose as cp  # noqa: E402
import nbv_score as nbv  # noqa: E402

ARM = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
ROLLS = (0, -45, 45, -90, 90, 180)
ELEVATIONS = (10.0, 20.0, 30.0)
MIN_AZIMUTH_STEP = 25.0
PLAN_OUT = pathlib.Path("/tmp/nbv_plan.json")
PROBE_OUT = pathlib.Path("/tmp/nbv_probe.json")


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


def plan_pose(tcp: np.ndarray, quat: np.ndarray) -> tuple[bool, str]:
    """规划到目标 TCP 位姿；成功返回 (True, 末点关节角列表)。"""
    command = [sys.executable, str(REPO / "scripts" / "plan_pose_once.py"),
               *[f"{v:.4f}" for v in tcp], *[f"{v:.5f}" for v in quat],
               "--output", str(PLAN_OUT)]
    completed = subprocess.run(command, capture_output=True, text=True, timeout=240)
    text = (completed.stdout or "").strip().splitlines()
    last = text[-1] if text else ""
    if "code 1 " not in (completed.stdout or ""):
        return False, last
    plan = json.loads(PLAN_OUT.read_text())
    return True, plan["points"][-1]["q"]


def execute_plan() -> tuple[bool, str]:
    completed = subprocess.run(
        [sys.executable, str(REPO / "scripts" / "execute_assembly_chunks.py"),
         str(PLAN_OUT), "--time-scale", "2.0", "--dwell", "0.5",
         "--goal-tolerance", "0.005"], capture_output=True, text=True, timeout=900)
    lines = (completed.stdout or "").strip().splitlines()
    return completed.returncode == 0, lines[-1] if lines else ""


def probe_view(preset: str = "workpiece") -> tuple[bool, float, float]:
    """到位后测 1 组，返回 (是否可用, 扶手分, 覆盖率)。"""
    command = [sys.executable, str(REPO / "scripts" / "measure_object.py"),
               "--preset", preset, "--stage", "coarse", "--groups", "1",
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
    parser.add_argument("--measurements", required=True)
    parser.add_argument("--group", default="g00_m")
    parser.add_argument("--distance", type=float, default=0.75)
    parser.add_argument("--j6-limit", type=float, default=45.0)
    parser.add_argument("--max-moves", type=int, default=3,
                        help="最多真正移动几次（每次移动后都做 1 组探针验收）")
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("nbv_move")
    state = current_joints(node)
    seed = [state[name] for name in ARM]
    j6_now = state["wrist_3_joint"]
    print(f"当前 J6 = {math.degrees(j6_now):+.1f}°")

    handeye = json.loads((REPO / "config/handeye/tcp_to_camera.json").read_text())
    T_tcp_cam = np.array(handeye["T_tcp_cam"])
    tcp_now = nbv.read_tcp()
    if tcp_now is None:
        print("读不到 TCP", file=sys.stderr)
        return 2
    T_base_cam = tcp_now @ T_tcp_cam

    payload = json.loads(pathlib.Path(args.measurements).read_text(encoding="utf-8"))
    rows = payload.get("per_group") if isinstance(payload, dict) else payload
    row = [item for item in rows if item.get("group") == args.group][0]
    base = np.eye(4)
    base[:3, :3] = R.from_quat(row["pose_base"]["quaternion"]).as_matrix()
    base[:3, 3] = row["pose_base"]["translation"]
    bbox = cp.load_template_bbox(nbv.TEMPLATE)
    size = bbox[1] - bbox[0]
    narrow_area = float(max(min(size[1] * size[2], size[0] * size[1]), 1e-6))
    centre = base[:3, :3] @ ((bbox[0] + bbox[1]) / 2) + base[:3, 3]
    faces = nbv.box_faces(bbox[0], bbox[1])

    to_part = centre - T_base_cam[:3, 3]
    azimuth0 = math.degrees(math.atan2(-to_part[1], -to_part[0]))
    candidates = []
    for offset in range(-180, 180, 20):
        if abs(offset) < MIN_AZIMUTH_STEP:
            continue
        for elevation in ELEVATIONS:
            az, el = math.radians(azimuth0 + offset), math.radians(elevation)
            direction = np.array([math.cos(el) * math.cos(az),
                                  math.cos(el) * math.sin(az), math.sin(el)])
            if not nbv.handle_face_visible(direction, base, bbox[0], bbox[1],
                                           min_cosine=0.5):
                continue
            score = nbv.score_direction(direction, base, faces, narrow_area)
            candidates.append((score, offset, elevation, direction))
    candidates.sort(key=lambda item: -item[0])
    print(f"候选 {len(candidates)} 个（要求 |Δ方位| ≥ {MIN_AZIMUTH_STEP:.0f}°，"
          f"手柄面 cos ≥ 0.5）")

    moves = 0
    for score, offset, elevation, direction in candidates:
        camera = centre + direction * args.distance
        axis = (centre - camera) / np.linalg.norm(centre - camera)
        current_axis = T_base_cam[:3, :3][:, 2]
        cross = np.cross(current_axis, axis)
        sine = float(np.linalg.norm(cross))
        cosine = float(np.dot(current_axis, axis))
        swing = (np.eye(3) if sine < 1e-9
                 else R.from_rotvec(cross / sine * math.atan2(sine, cosine)).as_matrix())
        for roll_deg in ROLLS:
            roll = R.from_rotvec(np.array([0, 0, 1.0]) * math.radians(roll_deg)).as_matrix()
            rotation = swing @ T_base_cam[:3, :3] @ roll
            tcp = camera - rotation @ T_tcp_cam[:3, 3]
            quat = R.from_matrix(rotation).as_quat()
            reachable, detail = plan_pose(tcp, quat)
            if not reachable:
                continue
            j6_new = detail[ARM.index("wrist_3_joint")]
            delta = math.degrees((j6_new - j6_now + math.pi) % (2 * math.pi) - math.pi)
            if abs(delta) > args.j6_limit:
                continue
            lateral = float(np.linalg.norm((camera - T_base_cam[:3, 3])[:2]) * 1000)
            print(f"候选：Δ方位 {offset:+d}° 俯角 {elevation:.0f}° 滚转 {roll_deg:+d}° "
                  f"得分 {score:.4f} ΔJ6 {delta:+.1f}° 横向 {lateral:.0f} mm")
            if not args.execute:
                json.dump({"tcp": list(map(float, tcp)), "quat": list(map(float, quat))},
                          open("/tmp/nbv_move.json", "w"))
                print("（未执行）")
                return 0
            ok, message = execute_plan()
            print(f"   执行：{message}")
            if not ok:
                continue
            moves += 1
            good, handle, coverage = probe_view()
            print(f"   探针：扶手 {handle:.2f} 覆盖 {coverage:.3f} → "
                  f"{'可用' if good else '不可用'}")
            if good:
                print("选定此视角")
                return 0
            if moves >= args.max_moves:
                print(f"已移动 {moves} 次仍不可用，停止（避免无休止换位）")
                return 3
    print("没有可用视角")
    return 3


if __name__ == "__main__":
    raise SystemExit(main())
