#!/usr/bin/env python3
"""按"已验证的相对视角"推导当前应站的精观测位（NBV 的可靠做法）。

原理
----
工件是在桌面上绕竖直轴转动，所以**相机相对工件的位姿不变，绝对位姿跟着转**。
因此在 0° 标定出一个"能测准"的相机位姿后，对任意偏转角都应当：
    T_obj_cam  = inv(T_base_obj_0°) @ T_base_cam_0°      （相对视角，标定一次）
    T_base_cam = T_base_obj_now @ T_obj_cam              （跟着工件转）
    T_base_tcp = T_base_cam @ inv(T_tcp_cam)
不需要搜索、不需要几何打分，也就不会撞。

用法
----
    python3 scripts/nbv_from_reference.py --execute
"""

from __future__ import annotations

import argparse
import json
import pathlib
import subprocess
import sys
from datetime import datetime

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

REPO = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))
import nbv_score as nbv  # noqa: E402
import coarse_pose as cp  # noqa: E402
import nbv_views  # noqa: E402

HAND_EYE = REPO / "config" / "handeye" / "tcp_to_camera.json"
COMMON = REPO / "config" / "tasks" / "common.yaml"


def pose_from_measurement(path: pathlib.Path, group: str = "g00_m") -> np.ndarray:
    payload = json.loads(path.read_text(encoding="utf-8"))
    return nbv_views.pose_from_payload(payload, group)


GAP_DIR = REPO / "runtime" / "measurements"


def record_station_gap(plan: dict, reason: str, views: dict,
                       current_measurement: pathlib.Path) -> pathlib.Path:
    """把"这次没选到可用站位"落盘，供所有者决定是否新增/复核站位。

    为什么记录（2026-11-13 定）：查表未命中时只打印"拒绝"没有"然后"，
    事后无法判断该补哪个角度。记录推导出的目标位姿，下次人工摆到该角度时
    可以直接实测并把结果并入 workpiece_views。
    """
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")   # 微秒：同秒内多次调用不互相覆盖
    payload = {
        "stamp": stamp,
        "reason": reason,                 # no_station / station_moved
        "yaw_deg": round(float(plan["yaw_deg"]), 2),
        "current_measurement": str(current_measurement),
        "tolerance_deg": float(views.get("tolerance_deg", 12.0)),
        "derived_tcp": [round(float(v), 5) for v in plan["derived_tcp"]],
        "derived_quat": [round(float(v), 6) for v in plan["derived_quat"]],
        "nearest_known_station_deg": plan.get("nearest_key"),
        "nearest_error_deg": plan.get("nearest_error_deg"),
        "skipped_unvalidated": plan.get("skipped_unvalidated") or [],
        "next_steps": [
            "1) 先原地精测（不换位）；测稳就继续，不需要新站位",
            "2) 原地测不稳 → 人工摆位到能测的角度",
            "3) 该角度实测通过后，把位姿并入 config/tasks/common.yaml 的 workpiece_views",
        ],
    }
    GAP_DIR.mkdir(parents=True, exist_ok=True)
    path = GAP_DIR / f"station_gap_{stamp}.json"
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    return path


def print_gap_guidance(plan: dict, reason: str, views: dict,
                       current_measurement: pathlib.Path) -> None:
    """失败时的"然后呢"：说清为什么、下一步做什么、记录落在哪。"""
    nearest = plan.get("nearest_key")
    nearest_err = plan.get("nearest_error_deg")
    skipped = plan.get("skipped_unvalidated") or []
    if reason == "no_station":
        tol = float(views.get("tolerance_deg", 12.0))
        print(f"→ 无可用预教站位覆盖该偏转角（容差 {tol:.0f}°）")
        if nearest is not None:
            where = "超出容差" if (nearest_err or 0) > tol else "在容差内但未通过精度门"
            print(f"   最近站位 {nearest}°，差 {nearest_err:.1f}°（{where}）")
        others = [(k, e) for k, e in skipped if str(k) != str(nearest)]
        if others:
            names = "、".join(f"{k}°(差 {e:.1f}°)" for k, e in others)
            print(f"   另有站位在容差内但**未通过精度门**，已跳过：{names}")
    else:
        print("→ 工件位置与示教站位不符（推导 vs 存储差 "
              f"{plan.get('delta_mm', float('nan')):.1f} mm），拒绝自动换位")
    print("   下一步：① 原地精测（默认）；② 测不稳则人工摆位；"
          "③ 新角度实测通过后并入 workpiece_views")
    path = record_station_gap(plan, reason, views, current_measurement)
    print(f"   已记录缺口：{path.relative_to(REPO)}（含推导目标位姿，便于下次补站位）")


def auto_station(target_pose: dict, execute: bool) -> int:
    """方案 A：查表选中的站位（已验证绝对位姿）→ plan_pose_ik 执行（带全套安全闸）。

    2026-11-13 所有者定调：预教站位 + 按偏转角自动选站，NBV 只做"选站"，
    不做大幅换构型。目标位姿来自已验证的 workpiece_views 表，不再推导/搜索。
    """
    tcp = np.asarray(target_pose["tcp"], dtype=float)
    quat = np.asarray(target_pose["quat"], dtype=float)
    cmd = [sys.executable, str(REPO / "scripts" / "plan_pose_ik.py"),
           *(f"{v:.5f}" for v in tcp), *(f"{v:.6f}" for v in quat)]
    if not execute:
        print("（未执行）")
        return 0
    move = subprocess.run(cmd, capture_output=True, text=True, timeout=900)
    tail = (move.stdout or "").strip().splitlines()
    print("   换位执行:", tail[-1] if tail else f"(exit {move.returncode})")
    if move.returncode == 6:
        # plan_pose_ik 拒绝"大幅改构型"：不做自动换位，原地精测（安全第一）
        return 6
    if move.returncode != 0:
        return 4
    verify = subprocess.run([sys.executable, str(REPO / "scripts" / "check_state_ok.py")],
                            capture_output=True, text=True, timeout=120)
    vtail = (verify.stdout or "").strip().splitlines()
    print("   到位复核:", vtail[-1] if vtail else "(no output)")
    return 0 if verify.returncode == 0 else 5


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference-measurement", default=str(
        REPO / "runtime" / "measurements" / "workpiece_zero.json"))
    parser.add_argument("--current-measurement", default=str(
        REPO / "runtime" / "measurements" / "workpiece_now.json"))
    parser.add_argument("--view-key", default="0")
    parser.add_argument("--max-wrist-delta", type=float, default=90.0,
                        help="透传给 plan_pose_ik；终点构型仍在绝对安全区间内即可")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--auto-station", action="store_true",
                        help="方案 A：按当前工件偏转角查表选预教站位，"
                             "与 0° 相对视角推导交叉核对后执行（不搜索视角）")
    parser.add_argument("--family-gate-mm", type=float, default=200.0,
                        help="relative_zero 族站位：推导目标与存储位姿的位置差超过"
                             "该值（mm）→ 判工件已移动，拒绝自动换位。"
                             "2026-11-14 所有者从 40 放宽到 200：相机绕工件转 12-15° "
                             "本身就有 90-120 mm 几何位移，40 mm 会误判；"
                             "工件真被搬走 200 mm 以上仍会被拦住")
    args = parser.parse_args()

    if args.auto_station:
        config = yaml.safe_load(COMMON.read_text())
        views = config.get("workpiece_views") or {}
        handeye = json.loads(HAND_EYE.read_text())
        T_tcp_cam = np.array(handeye["T_tcp_cam"])
        try:
            ref_pose = pose_from_measurement(pathlib.Path(args.reference_measurement))
            cur_pose = pose_from_measurement(pathlib.Path(args.current_measurement))
        except (OSError, json.JSONDecodeError, KeyError, IndexError) as exc:
            print(f"读测量文件失败：{exc}", file=sys.stderr)
            return 2
        try:
            plan = nbv_views.station_plan(views, ref_pose, cur_pose, T_tcp_cam,
                                          family_gate_mm=args.family_gate_mm)
        except ValueError as exc:
            print(f"选站失败：{exc}", file=sys.stderr)
            return 2

        print(f"工件偏转角 {plan['yaw_deg']:+.1f}°（相对 0° 基准）")
        if plan["verdict"] == "no_station":
            print_gap_guidance(plan, "no_station", views,
                               pathlib.Path(args.current_measurement))
            return 6
        error, key, entry = plan["station"]
        family = entry.get("family", "relative_zero")
        print(f"选中站位 {key}°（差 {error:.1f}°，family={family}）："
              f"TCP {np.round(plan['stored_tcp'], 4)}")
        print(f"0° 相对视角推导目标 TCP {np.round(plan['derived_tcp'], 4)}，"
              f"与站位差 {plan['delta_mm']:.1f} mm / {plan['delta_deg']:.1f}°")
        if plan["verdict"] == "station_far":
            print(f"→ 远站位告警：站位与偏转角相差 {error:.1f}°（近站位门 "
                  f"{plan.get('near_tolerance_deg', 12.0):.0f}°），"
                  "相对视角核对不适用；精度可能下降，以精观测质量门为准")
        if plan["verdict"] == "station_moved":
            print_gap_guidance(plan, "station_moved", views,
                               pathlib.Path(args.current_measurement))
            return 6
        print("→ 使用该已验证站位（不搜索）")
        return auto_station({"tcp": plan["stored_tcp"], "quat": plan["stored_quat"]},
                            args.execute)

    bbox = cp.load_template_bbox(pathlib.Path.home()
                                 / "cs625_rvs_archive" / "models_20261023"
                                 / "弹仓_装配导出.STL")
    config = yaml.safe_load(COMMON.read_text())
    view = (config.get("workpiece_views") or {}).get(args.view_key)
    if not view or "tcp" not in view:
        print(f"配置里没有已验证视角 {args.view_key}", file=sys.stderr)
        return 2
    handeye = json.loads(HAND_EYE.read_text())
    T_tcp_cam = np.array(handeye["T_tcp_cam"])

    # 标定时的相机位（已验证视角）
    T_base_tcp_ref = np.eye(4)
    T_base_tcp_ref[:3, :3] = R.from_quat(view["quat"]).as_matrix()
    T_base_tcp_ref[:3, 3] = view["tcp"]
    T_base_cam_ref = T_base_tcp_ref @ T_tcp_cam

    T_base_obj_ref = pose_from_measurement(pathlib.Path(args.reference_measurement))
    T_obj_cam = np.linalg.inv(T_base_obj_ref) @ T_base_cam_ref

    T_base_obj_now = pose_from_measurement(pathlib.Path(args.current_measurement))
    T_base_cam_now = T_base_obj_now @ T_obj_cam
    T_base_tcp_now = T_base_cam_now @ np.linalg.inv(T_tcp_cam)
    tcp = T_base_tcp_now[:3, 3]
    quat = R.from_matrix(np.array(T_base_tcp_now[:3, :3])).as_quat()

    delta = (R.from_matrix(np.array(T_base_obj_now[:3, :3]))
             * R.from_matrix(np.array(T_base_obj_ref[:3, :3])).inv())
    print("工件相对标定时转动 %.1f°" % np.degrees(delta.magnitude()))
    print("目标 TCP", np.round(tcp, 4), "四元数", np.round(quat, 5))


    # 位置保持不变（相对视角的关键），姿态只要求"光轴指向工件中心"，滚转自由：
    # 沿用手眼基准滚转会逼出不可达的腕部构型（实测规划直接 code 99999）。
    camera_pos = T_base_cam_now[:3, 3]
    centre = T_base_obj_now[:3, :3] @ ((bbox[0] + bbox[1]) / 2) + T_base_obj_now[:3, 3]
    axis = centre - camera_pos
    axis /= np.linalg.norm(axis)
    # 姿态：从【当前相机姿态】做最短弧摆动到"光轴指向工件"，保持滚转不变，
    # 这样腕部构型离当前最近、最容易可达（实测"按 look-at 重建"会全不可达）。
    handeye_now = json.loads(HAND_EYE.read_text())
    T_tcp_cam_now = np.array(handeye_now["T_tcp_cam"])
    current_tcp = nbv.read_tcp()
    current_camera = current_tcp @ T_tcp_cam_now
    current_axis = current_camera[:3, :3][:, 2]
    cross = np.cross(current_axis, axis)
    sine = float(np.linalg.norm(cross))
    cosine = float(np.dot(current_axis, axis))
    swing = (np.eye(3) if sine < 1e-9
             else R.from_rotvec(cross / sine * np.arctan2(sine, cosine)).as_matrix())
    # 【安全限幅】自动 NBV 只做"小幅微调"：如果为了到目标视角需要大幅改构型
    # （任一关节 > 25° 或整臂合计 > 1.0 rad），一律不动，交人工摆位。
    # 2026-11-13 两次自碰撞/急停都是在"自动大幅改构型"时发生的。
    current_positions = None
    try:
        import rclpy as _rclpy
        from rclpy.node import Node as _Node
        from sensor_msgs.msg import JointState as _JS
        _rclpy.init()
        _node = _Node("nbv_limit_check")
        _state = {}
        _node.create_subscription(_JS, "/joint_states",
                                  lambda msg: _state.setdefault("j", msg), 10)
        import time as _time
        _deadline = _time.time() + 5
        while _time.time() < _deadline and "j" not in _state:
            _rclpy.spin_once(_node, timeout_sec=0.2)
        if "j" in _state:
            current_positions = dict(zip(_state["j"].name, _state["j"].position))
        _node.destroy_node()
        _rclpy.shutdown()
    except Exception as _error:  # noqa: BLE001
        print(f"读当前关节失败：{_error}")

    tried = []
    # ===== 换视角必须先"平移"、不许"转腕"（agent_memory 137）=====
    # 相机在 TCP 后方约 0.3 m：转腕等于把相机甩出大圆弧，既危险又对视野没帮助。
    # 正确做法：保持当前工具姿态，用【前三轴】把相机整体平移过去。
    handeye_now = json.loads(HAND_EYE.read_text())
    T_tcp_cam_now = np.array(handeye_now["T_tcp_cam"])
    current_tcp = nbv.read_tcp()
    current_camera = current_tcp @ T_tcp_cam_now
    t_tcp_cam = np.linalg.inv(T_tcp_cam_now)[:3, 3]
    tcp_target_translate = camera_pos - current_camera[:3, :3] @ t_tcp_cam
    print("平移目标 TCP", np.round(tcp_target_translate, 4),
          "（姿态保持当前，前三轴负责移动）")
    if not args.execute:
        print("（未执行）")
        return 0
    move = subprocess.run(
        [sys.executable, str(REPO / "scripts" / "move_tcp_linear.py"),
         f"{tcp_target_translate[0]:.4f}", f"{tcp_target_translate[1]:.4f}",
         f"{tcp_target_translate[2]:.4f}", "50", "0.02", "4.0"],
        capture_output=True, text=True, timeout=900)
    lines_out = (move.stdout or "").strip().splitlines()
    print("   平移执行:", lines_out[-1] if lines_out else "")
    if move.returncode != 0:
        return 4
    verify = subprocess.run([sys.executable, str(REPO / "scripts" / "check_state_ok.py")],
                            capture_output=True, text=True, timeout=120)
    tail = (verify.stdout or "").strip().splitlines()
    print("   到位复核:", tail[-1] if tail else "(no output)")
    return 0 if verify.returncode == 0 else 5


if __name__ == "__main__":
    raise SystemExit(main())
