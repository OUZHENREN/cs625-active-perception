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

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

REPO = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))
import nbv_score as nbv  # noqa: E402
import coarse_pose as cp  # noqa: E402

HAND_EYE = REPO / "config" / "handeye" / "tcp_to_camera.json"
COMMON = REPO / "config" / "tasks" / "common.yaml"


def pose_from_measurement(path: pathlib.Path, group: str = "g00_m") -> np.ndarray:
    payload = json.loads(path.read_text(encoding="utf-8"))
    rows = payload.get("per_group") if isinstance(payload, dict) else payload
    row = [item for item in rows if item.get("group") == group][0]
    pose = np.eye(4)
    pose[:3, :3] = R.from_quat(row["pose_base"]["quaternion"]).as_matrix()
    pose[:3, 3] = row["pose_base"]["translation"]
    return pose


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference-measurement", default=str(
        REPO / "runtime" / "measurements" / "workpiece_zero.json"))
    parser.add_argument("--current-measurement", default=str(
        REPO / "runtime" / "measurements" / "workpiece_now.json"))
    parser.add_argument("--view-key", default="0")
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

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

    if not args.execute:
        print("（未执行）")
        return 0

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
    for roll_deg in (0, -45, 45, -90, 90, 180):
        roll = R.from_rotvec(np.array([0.0, 0.0, 1.0])
                             * np.radians(roll_deg)).as_matrix()
        rotation = swing @ current_camera[:3, :3] @ roll
        # TCP = 相机位置 − R_cam · t_tcp_cam（t_tcp_cam 是相机在 TCP 系里的平移）
        t_tcp_cam = np.linalg.inv(T_tcp_cam)[:3, 3]
        tcp_target = camera_pos - rotation @ t_tcp_cam
        quat_target = R.from_matrix(rotation).as_quat()
        plan = subprocess.run([sys.executable, str(REPO / "scripts" / "plan_pose_once.py"),
                               *[f"{v:.4f}" for v in tcp_target],
                               *[f"{v:.5f}" for v in quat_target],
                               "--max-wrist-delta", "45",
                               "--output", "/tmp/nbv_ref_plan.json"],
                              capture_output=True, text=True, timeout=300)
        text = ((plan.stdout or "") + (plan.stderr or "")).strip().splitlines()
        print(f"滚转 {roll_deg:+4d}° 规划:", text[-1] if text else "(none)")
        if "code 1 " in (plan.stdout or ""):
            moved = subprocess.run(
                [sys.executable, str(REPO / "scripts" / "execute_assembly_chunks.py"),
                 "/tmp/nbv_ref_plan.json", "--time-scale", "2.0", "--dwell", "0.5",
                 "--goal-tolerance", "0.005"], capture_output=True, text=True, timeout=900)
            lines = (moved.stdout or "").strip().splitlines()
            print("执行:", lines[-1] if lines else "")
            return 0 if moved.returncode == 0 else 4
    print("所有滚转都不可达")
    return 3


if __name__ == "__main__":
    raise SystemExit(main())
