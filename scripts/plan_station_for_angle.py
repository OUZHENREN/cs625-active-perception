#!/usr/bin/env python3
"""给一个工件偏角，告诉操作者"该站到哪、能不能到"（2026-11-19 所有者定）。

背景
----
精拍示教位姿时，每个角度都要先回答两个问题：
  1. 保持"相对视角"不变的话，相机理论位置在哪？
  2. 那个位置在当前臂姿下可达吗？要不要分段？有没有更近的已验证站位可用？

以前这两步要手工算：先跑 nbv_from_reference 拿推导位姿，再复制数字去跑
plan_pose_ik。这个工具把它们合成一条命令，并且**只规划、不动臂**。

用法
----
    # 给一个角度（相对 0° 基准，度）
    python3 scripts/plan_station_for_angle.py --yaw -20

    # 指定参考测量（默认用 0° 的参考测量）
    python3 scripts/plan_station_for_angle.py --yaw -20 \
        --reference-measurement runtime/measurements/workpiece_zero.json

    # 只查已验证站位怎么选，不算推导位姿
    python3 scripts/plan_station_for_angle.py --yaw -20 --skip-derived

输出
----
  · 查表选站结果（tier / 实测覆盖率 / 候选排序）
  · 推导位姿（相对视角保持不变的相机位置）
  · 对推导位姿做 plan_pose_ik **只规划**：可达 / 不可达 + 原因
  · 结论：建议站到哪（推导位姿 or 已验证站位）

注意：本脚本**不会移动机械臂**。要动臂，把打印出来的命令自己跑一遍（或加 --execute
由你确认后执行，见下）。
"""

from __future__ import annotations

import argparse
import json
import pathlib
import subprocess
import sys

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

REPO = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))

import nbv_views  # noqa: E402

HAND_EYE = REPO / "config" / "handeye" / "tcp_to_camera.json"
COMMON = REPO / "config" / "tasks" / "common.yaml"


def derived_pose_from_pose(reference_measurement: pathlib.Path,
                           current_pose: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """用**实测的当前工件位姿**推导相机位姿（推荐）。

    工件平移过之后，按角度合成出来的位置是错的；直接拿实测位姿最准。
    """
    views = yaml.safe_load(COMMON.read_text(encoding="utf-8"))["workpiece_views"]
    anchor = views["0"]
    T_tcp_cam = np.array(json.loads(HAND_EYE.read_text(encoding="utf-8"))["T_tcp_cam"])

    T_cam0 = np.eye(4)
    T_cam0[:3, :3] = R.from_quat(anchor["quat"]).as_matrix()
    T_cam0[:3, 3] = anchor["tcp"]

    T_base_obj_ref = nbv_views.load_pose(reference_measurement)
    T_obj_cam = np.linalg.inv(T_base_obj_ref) @ (T_cam0 @ T_tcp_cam)
    T_base_cam = current_pose @ T_obj_cam
    T_base_tcp = T_base_cam @ np.linalg.inv(T_tcp_cam)
    return T_base_tcp[:3, 3], R.from_matrix(T_base_tcp[:3, :3]).as_quat()


def derived_pose_for_yaw(reference_measurement: pathlib.Path,
                         workpiece_yaw_deg: float) -> tuple[np.ndarray, np.ndarray]:
    """按"相对视角不变"推导该偏角下的相机位姿。

    原理：相机相对工件的位姿在 0° 标定一次，之后工件绕竖直轴转多少，相机就跟着转多少。
        T_obj_cam = inv(T_base_obj_0°) @ (T_base_cam_0° @ T_tcp_cam)
        T_base_cam = T_base_obj(yaw) @ T_obj_cam
    """
    views = yaml.safe_load(COMMON.read_text(encoding="utf-8"))["workpiece_views"]
    anchor = views["0"]
    T_tcp_cam = np.array(json.loads(HAND_EYE.read_text(encoding="utf-8"))["T_tcp_cam"])

    T_cam0 = np.eye(4)
    T_cam0[:3, :3] = R.from_quat(anchor["quat"]).as_matrix()
    T_cam0[:3, 3] = anchor["tcp"]

    T_base_obj_ref = nbv_views.load_pose(reference_measurement)
    T_obj_cam = np.linalg.inv(T_base_obj_ref) @ (T_cam0 @ T_tcp_cam)

    T_base_obj_now = T_base_obj_ref.copy()
    T_base_obj_now[:3, :3] = (R.from_euler("z", workpiece_yaw_deg, degrees=True).as_matrix()
                              @ T_base_obj_ref[:3, :3])
    T_base_cam = T_base_obj_now @ T_obj_cam
    T_base_tcp = T_base_cam @ np.linalg.inv(T_tcp_cam)
    return T_base_tcp[:3, 3], R.from_matrix(T_base_tcp[:3, :3]).as_quat()


def current_tcp(timeout: float = 12.0):
    """读当前 TCP 位姿（位置 + 四元数）；读不到返回 None。"""
    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.node import Node
    except Exception:  # noqa: BLE001
        return None
    import time as _time
    try:
        rclpy.init()
    except Exception:  # noqa: BLE001 - 已初始化
        pass
    node = Node("plan_station_for_angle_tcp")
    box: dict = {}
    node.create_subscription(PoseStamped, "/tcp_pose_broadcaster/pose",
                             lambda m: box.setdefault("m", m), 10)
    end = _time.time() + timeout
    while "m" not in box and _time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:  # noqa: BLE001
        pass
    message = box.get("m")
    if message is None:
        return None
    p = message.pose.position
    q = message.pose.orientation
    return np.array([p.x, p.y, p.z]), np.array([q.x, q.y, q.z, q.w])


def already_there(tcp: np.ndarray, quat: np.ndarray,
                  position_tol_mm: float = 5.0, angle_tol_deg: float = 2.0) -> bool:
    """当前臂位是否已经基本等于目标位姿（避免"在原地规划一次大改构型"）。"""
    current = current_tcp()
    if current is None:
        return False
    current_tcp_position, current_quat = current
    distance_mm = float(np.linalg.norm(tcp - current_tcp_position) * 1000.0)
    # 四元数夹角（q 与 -q 等价）
    dot = abs(float(np.dot(quat, current_quat)))
    angle_deg = float(np.degrees(2 * np.arccos(min(1.0, dot))))
    return distance_mm <= position_tol_mm and angle_deg <= angle_tol_deg


def plan_only(tcp: np.ndarray, quat: np.ndarray) -> tuple[int, str]:
    """对给定位姿做 plan_pose_ik（只规划，不动臂）。返回 (退出码, 最后一行)。"""
    command = [sys.executable, str(REPO / "scripts" / "plan_pose_ik.py"),
               *(f"{v:.5f}" for v in tcp), *(f"{v:.6f}" for v in quat),
               "--output", "/tmp/plan_station_for_angle.json"]
    completed = subprocess.run(command, capture_output=True, text=True, timeout=900)
    lines = (completed.stdout or "").strip().splitlines()
    tail = lines[-1] if lines else f"(exit {completed.returncode})"
    return completed.returncode, tail, lines


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--yaw", type=float, required=True,
                        help="工件偏角（度，相对 0° 基准；逆时针为正，与 measure_object 一致）")
    parser.add_argument("--reference-measurement", default=str(
        REPO / "runtime" / "measurements" / "workpiece_zero.json"),
        help="0° 的参考测量（推导相对视角用）")
    parser.add_argument("--current-measurement", default="",
                        help="当前工件测量（给选站用；不给则只做推导+规划）")
    parser.add_argument("--skip-derived", action="store_true",
                        help="不推导/不规划，只查表选站")
    parser.add_argument("--execute", action="store_true",
                        help="规划成功后直接执行（默认只规划；执行前请确认周围安全）")
    args = parser.parse_args()

    views = yaml.safe_load(COMMON.read_text(encoding="utf-8"))["workpiece_views"]

    # ---------- 1) 查表选站 ----------
    station = None
    if args.current_measurement:
        ref_pose = nbv_views.load_pose(pathlib.Path(args.reference_measurement))
        cur_pose = nbv_views.load_pose(pathlib.Path(args.current_measurement))
        T_tcp_cam = np.array(json.loads(HAND_EYE.read_text(encoding="utf-8"))["T_tcp_cam"])
        try:
            plan = nbv_views.station_plan(views, ref_pose, cur_pose, T_tcp_cam)
        except ValueError as error:
            print(f"选站失败：{error}")
            plan = None
        if plan and plan.get("station"):
            error, key, entry = plan["station"]
            coverage = entry.get("empirical_coverage")
            station = {
                "key": key,
                "tcp": plan["stored_tcp"],
                "quat": plan["stored_quat"],
                "tier": int(entry.get("tier", 2)),
                "coverage": coverage,
                "error_deg": error,
                "verdict": plan["verdict"],
            }
            coverage_text = f"{float(coverage):.3f}" if coverage is not None else "未实测"
            print(f"[选站] 选中 {key}（tier {station['tier']}，差 {error:.1f}°，"
                  f"实测覆盖率 {coverage_text}，verdict={plan['verdict']}）")
            ranked = plan.get("ranked") or []
            if ranked:
                print("[选站] 候选排序：" + " > ".join(
                    f"{item['key']}(t{item['tier']},"
                    f"cov {item['coverage'] if item['coverage'] is not None else '—'},"
                    f"{item['error_deg']:.1f}°)" for item in ranked))
        else:
            print("[选站] 没有覆盖该偏角的已验证站位")
    else:
        print("[选站] 未提供 --current-measurement，跳过查表选站")

    # ---------- 2) 推导位姿 + 只规划 ----------
    if args.skip_derived:
        return 0

    # 有实测位姿就用实测（工件平移过之后，按角度合成的位置是错的）
    if args.current_measurement:
        current_pose = nbv_views.load_pose(pathlib.Path(args.current_measurement))
        tcp, quat = derived_pose_from_pose(pathlib.Path(args.reference_measurement),
                                           current_pose)
        source = f"实测工件位姿（{pathlib.Path(args.current_measurement).name}）"
    else:
        tcp, quat = derived_pose_for_yaw(pathlib.Path(args.reference_measurement), args.yaw)
        source = "按角度合成（未给 --current-measurement）"
    print(f"\n[推导] 保持相对视角不变，{args.yaw:+.1f}° 的相机理论位置（{source}）：")
    print("        tcp  = " + " ".join(f"{v:.5f}" for v in tcp))
    print("        quat = " + " ".join(f"{v:.6f}" for v in quat))

    if already_there(tcp, quat):
        print("[规划] 当前臂位已经在这个位姿附近（≤5 mm / ≤2°），无需移动。")
        print("        直接拍摄即可；要微调请手动 jog 或给别的角度。")
        return 0

    code, tail, lines = plan_only(tcp, quat)
    for line in lines:
        if line.strip():
            print("        " + line.strip())
    reachable = code == 0
    if reachable:
        print("[规划] 推导位姿**可达**（已写出计划，未执行）")
    else:
        print(f"[规划] 推导位姿**不可达**（exit {code}）：{tail}")
        print("        含义：这个偏角下无法严格保持相对视角，需要改用已验证站位，")
        print("        或接受较大的视角偏差（拍完看覆盖率决定是否可用）。")

    # ---------- 3) 结论 ----------
    print("\n[结论]")
    if reachable:
        print(f"  优先用推导位姿（tcp {' '.join(f'{v:.4f}' for v in tcp)}）拍摄。")
        if station:
            print(f"  若推导位姿拍出来不好，回退到已验证站位 {station['key']}。")
        if args.execute:
            print("  --execute 已给出：执行推导位姿的移动。")
            plan_file = "/tmp/plan_station_for_angle.json"
            run = subprocess.run(
                [sys.executable, str(REPO / "scripts" / "execute_assembly_chunks.py"),
                 plan_file, "--time-scale", "3.0", "--goal-tolerance", "0.001",
                 "--verify-final", "0.004", "--fault-poll", "0.1"],
                capture_output=True, text=True, timeout=900)
            print("  " + (run.stdout or "").strip().splitlines()[-1])
            return 0 if "ALL_ASSEMBLY_CHUNKS_OK" in (run.stdout or "") else 4
    else:
        if station:
            print(f"  用已验证站位 {station['key']}"
                  f"（tcp {' '.join(f'{v:.4f}' for v in station['tcp'])}）。")
        else:
            print("  推导位姿不可达且没有可用站位：这个角度需要单独设计视角，")
            print("  或换一个工件摆放位置再拍。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
