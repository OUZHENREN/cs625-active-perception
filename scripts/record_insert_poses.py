#!/usr/bin/env python3
"""记录插入链的已验证位姿，并推导 insert_in_slot_frame（所有者 2026-11-25 方案）。

为什么这样做
------------
插槽是以大平面为主的夹具，ICP 在面内没有约束（实测同一片点云两条配准路径差 66 mm），
所以"测量出来的插槽绝对位姿"本身不是良定义的量（手册第 68 条）。
所有者 2026-11-25 定：由他手动摆出**真的能插进去**的位姿，我们直接记录，
不再靠推导/经验平移去猜。

记录哪些
--------
    insert      工件插到底、座好时的 TCP 位姿（最关键）
    straighten  夹爪带件转平后的位姿（转正终点）
    pre_insert  退出到预插入高度时的位姿（可选，也能由 insert 派生）
    withdraw    完全脱离槽口后的位姿（可选）

每个位姿记录：TCP xyz + quat、六个关节角、记录时间、备注。
**只在机械臂静止时记录**（连续 1.2 s 关节速度≈0 且位置稳定在 0.2° 内），
因为手拖动刚松手时读数会漂。

用法
----
    source scripts/source_dev_env.sh --full

    # ① 所有者手动摆好 → 记录（每个位姿一条命令）
    python3 scripts/record_insert_poses.py --name insert --note "插到底、座好"
    python3 scripts/record_insert_poses.py --name straighten --note "转平后"
    python3 scripts/record_insert_poses.py --name pre_insert

    # ② 看已记录的
    python3 scripts/record_insert_poses.py --show

    # ③ 用"记录时的插槽测量"推导相对变换（与 grasp_in_workpiece_frame 同构）
    python3 scripts/record_insert_poses.py --derive --name insert \\
        --slot-measurement runtime/measurements/slot_latest.json

    # ④ 所有者确认后，把结果写进 config/tasks/common.yaml 的 insert_calibration 块
    python3 scripts/record_insert_poses.py --promote

安全：只读机械臂状态、只写本地文件；不发任何运动指令。
"""

from __future__ import annotations

import argparse
import datetime
import json
import math
import pathlib
import sys
import time

import numpy as np

sys.stdout.reconfigure(line_buffering=True)

REPO = pathlib.Path(__file__).resolve().parents[1]
STORE = REPO / "runtime" / "insert_calibration.json"
COMMON = REPO / "config" / "tasks" / "common.yaml"
JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
# 与 plan_pose_ik / goto_park_pose 一致的绝对安全区间（2026-11-24 所有者批准）
LIMITS_DEG = {
    "shoulder_pan_joint": (-100.0, 100.0),
    "shoulder_lift_joint": (-130.0, 10.0),
    "elbow_joint": (-160.0, -60.0),
    "wrist_1_joint": (-150.0, 150.0),
    "wrist_2_joint": (-150.0, 150.0),
    "wrist_3_joint": (-150.0, 150.0),
}
PHASES = ("insert", "straighten", "pre_insert", "withdraw")
STILL_SPEED_RAD_S = 0.01
STILL_SPREAD_DEG = 0.2
STILL_WINDOW_S = 1.2


def read_still_pose(window: float = STILL_WINDOW_S):
    """等机械臂静止后读一次位姿；返回 dict 或 (None, 原因)。"""
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    if not rclpy.ok():
        rclpy.init(args=None)
    node = rclpy.create_node("record_insert_poses")
    latest = {"j": [], "tcp": None}

    node.create_subscription(
        JointState, "/joint_states",
        lambda m: latest["j"].append((time.time(), dict(zip(m.name, m.position)),
                                      dict(zip(m.name, m.velocity)))), 10)
    node.create_subscription(
        PoseStamped, "/tcp_pose_broadcaster/pose",
        lambda m: latest.__setitem__("tcp", m), 10)

    deadline = time.time() + 25.0
    while time.time() < deadline and not latest["j"]:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not latest["j"]:
        node.destroy_node()
        return None, "读不到 /joint_states（驱动栈没起？）"

    end = time.time() + window
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    samples = [s for s in latest["j"] if time.time() - s[0] <= window]
    node.destroy_node()

    if len(samples) < 5:
        return None, f"静止窗口内样本太少（{len(samples)}）"
    speeds = [max(abs(v) for v in s[2].values()) for s in samples if s[2]]
    if speeds and max(speeds) > STILL_SPEED_RAD_S:
        return None, f"还在动（最大关节速度 {max(speeds):.3f} rad/s > {STILL_SPEED_RAD_S}），请松手等停稳"
    spread = 0.0
    for name in JOINTS:
        values = [math.degrees(s[1][name]) for s in samples if name in s[1]]
        if values:
            spread = max(spread, max(values) - min(values))
    if spread > STILL_SPREAD_DEG:
        return None, f"位置还在漂（窗口内最大变化 {spread:.2f}° > {STILL_SPREAD_DEG}°）"

    positions = samples[-1][1]
    if len([n for n in JOINTS if n in positions]) < 6:
        return None, "关节数据不完整"
    deg = {n: math.degrees(positions[n]) for n in JOINTS}
    for name, (low, high) in LIMITS_DEG.items():
        if not (low - 1e-6 <= deg[name] <= high + 1e-6):
            return None, f"拒绝记录：{name} = {deg[name]:.1f}° 不在安全区间 [{low}, {high}]"
    if latest["tcp"] is None:
        return None, "读不到 /tcp_pose_broadcaster/pose"
    p = latest["tcp"].pose.position
    q = latest["tcp"].pose.orientation
    return {"tcp": [p.x, p.y, p.z], "quat": [q.x, q.y, q.z, q.w],
            "joints_deg": [round(deg[n], 2) for n in JOINTS],
            "still": {"max_speed_rad_s": round(max(speeds) if speeds else 0.0, 4),
                      "max_spread_deg": round(spread, 3)}}, None


def _store_label() -> str:
    """存储路径的显示名（测试里 STORE 可能在仓库外，不能用 relative_to）。"""
    try:
        return str(STORE.relative_to(REPO))
    except ValueError:
        return str(STORE)


def load_store() -> dict:
    if STORE.exists():
        try:
            return json.loads(STORE.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            pass
    return {"recorded_at": None, "poses": {}}


def save_store(payload: dict) -> None:
    STORE.parent.mkdir(parents=True, exist_ok=True)
    STORE.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def cmd_record(args) -> int:
    payload = load_store()
    pose, reason = read_still_pose()
    if pose is None:
        print(f"未记录：{reason}")
        return 2
    pose["note"] = args.note
    pose["recorded_at"] = datetime.datetime.now().isoformat(timespec="seconds")
    payload["poses"][args.name] = pose
    payload["recorded_at"] = pose["recorded_at"]
    save_store(payload)
    print(f"已记录 {args.name}：")
    print(f"  TCP   {np.round(pose['tcp'], 4).tolist()}   "
          f"quat {np.round(pose['quat'], 4).tolist()}")
    print(f"  关节  {pose['joints_deg']}")
    print(f"  静止确认 最大速度 {pose['still']['max_speed_rad_s']} rad/s, "
          f"窗口内偏移 {pose['still']['max_spread_deg']}°")
    if args.note:
        print(f"  备注  {args.note}")
    print(f"  存储  {_store_label()}")
    return 0


def cmd_show(args) -> int:
    payload = load_store()
    poses = payload.get("poses") or {}
    if not poses:
        print(f"还没有记录（{_store_label()} 不存在或为空）")
        return 0
    print(f"已记录的插入链位姿（{payload.get('recorded_at')}）：")
    for name in PHASES + tuple(n for n in poses if n not in PHASES):
        pose = poses.get(name)
        if not pose:
            continue
        print(f"\n  [{name}]  {pose.get('recorded_at', '')}  {pose.get('note', '')}")
        print(f"    TCP   {np.round(pose['tcp'], 4).tolist()}")
        print(f"    quat  {np.round(pose['quat'], 4).tolist()}")
        print(f"    关节  {pose['joints_deg']}")
    missing = [p for p in PHASES if p not in poses]
    if missing:
        print(f"\n  未记录：{missing}")
    return 0


def cmd_derive(args) -> int:
    """insert_in_slot_frame = inv(T_base_slot) @ T_base_tcp（与抓取同构）。"""
    from scipy.spatial.transform import Rotation as R

    payload = load_store()
    pose = (payload.get("poses") or {}).get(args.name)
    if pose is None:
        print(f"没有记录 {args.name}，先 --record 它")
        return 3
    path = pathlib.Path(args.slot_measurement)
    if not path.exists():
        print(f"插槽测量不存在：{path}")
        return 3
    slot = json.loads(path.read_text(encoding="utf-8"))["per_group"][0]["pose_base"]
    T_slot = np.eye(4)
    T_slot[:3, :3] = R.from_quat(slot["quaternion"]).as_matrix()
    T_slot[:3, 3] = slot["translation"]
    T_tcp = np.eye(4)
    T_tcp[:3, :3] = R.from_quat(pose["quat"]).as_matrix()
    T_tcp[:3, 3] = pose["tcp"]
    T_rel = np.linalg.inv(T_slot) @ T_tcp
    # 往返校验：用同一个 T_slot 复原记录的 TCP
    back = T_slot @ T_rel
    err_mm = float(np.linalg.norm(back[:3, 3] - T_tcp[:3, 3])) * 1000.0
    ang = float(np.abs((R.from_matrix(back[:3, :3]) * R.from_matrix(T_tcp[:3, :3]).inv()).magnitude()))
    print(f"用 {path.name} 的插槽位姿 + 记录的 {args.name} 推导相对变换：")
    print(f"  插槽 (T_base_slot)  {np.round(T_slot[:3, 3], 4).tolist()}")
    print(f"  {args.name:10s}(T_base_tcp) {np.round(T_tcp[:3, 3], 4).tolist()}")
    print(f"  相对变换 translation {np.round(T_rel[:3, 3], 6).tolist()}")
    print(f"          quaternion  {np.round(R.from_matrix(T_rel[:3, :3]).as_quat(), 6).tolist()}")
    print(f"  往返校验 {err_mm:.4f} mm / {math.degrees(ang):.5f}°（应为 0）")
    payload.setdefault("derived", {})[args.name] = {
        "translation": [round(float(v), 6) for v in T_rel[:3, 3]],
        "quaternion": [round(float(v), 6) for v in R.from_matrix(T_rel[:3, :3]).as_quat()],
        "slot_measurement": str(path),
        "round_trip_mm": round(err_mm, 6),
        "round_trip_deg": round(math.degrees(ang), 6),
    }
    save_store(payload)
    print(f"  已写入 {_store_label()} 的 derived.{args.name}")
    return 0


def cmd_promote(args) -> int:
    """把记录/推导结果写成 config 的 insert_calibration 块（所有者确认后执行）。"""
    import yaml
    payload = load_store()
    poses = payload.get("poses") or {}
    if not poses:
        print("没有可 promote 的记录")
        return 3
    block = {"recorded_at": payload.get("recorded_at"),
             "source": "owner hand-posed verified poses (scripts/record_insert_poses.py)",
             "poses": {name: {"tcp": [round(v, 5) for v in pose["tcp"]],
                              "quat": [round(v, 6) for v in pose["quat"]],
                              "joints_deg": pose["joints_deg"],
                              "note": pose.get("note", "")}
                       for name, pose in poses.items()}}
    if payload.get("derived"):
        block["derived_in_slot_frame"] = payload["derived"]
    text = COMMON.read_text(encoding="utf-8")
    rendered = "insert_calibration:\n" + yaml.safe_dump(
        block, allow_unicode=True, sort_keys=False, default_flow_style=False)
    rendered = "".join("  " + line if line.strip() else line
                       for line in rendered.splitlines(keepends=True))
    if "insert_calibration:" in text:
        start = text.index("insert_calibration:")
        end = len(text)
        for index in range(start + len("insert_calibration:"), len(text)):
            if text[index] == "\n" and index + 1 < len(text) and text[index + 1] not in " \n\t#":
                end = index + 1
                break
        text = text[:start] + rendered + "\n" + text[end:]
    else:
        text = text.rstrip("\n") + "\n\n" + rendered + "\n"
    COMMON.write_text(text, encoding="utf-8")
    check = (yaml.safe_load(COMMON.read_text(encoding="utf-8")) or {}).get("insert_calibration") or {}
    if not check.get("poses"):
        print("写入后回读校验失败")
        return 5
    print(f"已写入 config/tasks/common.yaml 的 insert_calibration（{len(check['poses'])} 个位姿）")
    print("注意：效率模式目前仍从 runtime/slot_reference_verified.json 推导插入链；")
    print("      要改用这份标定，需要再改 reference_insert_chain()——请所有者确认后处理。")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--name", choices=PHASES,
                        help="要记录/推导的位姿名")
    parser.add_argument("--note", default="", help="备注，例如：插到底座好")
    parser.add_argument("--slot-measurement",
                        default="runtime/measurements/slot_latest.json",
                        help="推导相对变换时使用的插槽测量文件")
    parser.add_argument("--show", action="store_true", help="查看已记录的位姿")
    parser.add_argument("--derive", action="store_true",
                        help="推导 insert_in_slot_frame（用记录的位姿 + 插槽测量）")
    parser.add_argument("--promote", action="store_true",
                        help="把记录写进 config/tasks/common.yaml 的 insert_calibration 块")
    args = parser.parse_args()

    if args.show:
        return cmd_show(args)
    if args.promote:
        return cmd_promote(args)
    if args.derive:
        if not args.name:
            print("--derive 需要 --name")
            return 3
        return cmd_derive(args)
    if not args.name:
        parser.print_help()
        return 3
    return cmd_record(args)


if __name__ == "__main__":
    raise SystemExit(main())
