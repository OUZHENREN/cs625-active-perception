#!/usr/bin/env python3
"""从粗测位姿算"精观测位"，规划（可选执行）——滚转沿用当前值，避免无谓的腕部大转。

两条关键几何事实（2026-11-10 实测）
------------------------------------
1. 相机光轴与工具 z 轴平行；相机在 TCP 系里位于 (0.143, 0.042, −0.255)，距 TCP 0.296 m。
   ⇒ "让相机对准工件" 等价于 "让工具 z 轴指向工件"；
   ⇒ 绕工具轴的滚转对相机**没有影响**（只转图像），是自由量，应该沿用当前值。
2. 工件竖立在桌上：平视只能看到 2 个正交面，三平面法必须斜看（俯角默认 30°）。

因此目标姿态用"最短弧把当前工具 z 轴转到期望光轴方向"，而不是照抄对象系姿态——
后者在对象系被修正（例如 CAD 坐标翻转）时会凭空产生 180° 的 J6 转角。

用法
----
    # 只算 + 规划
    python3 scripts/plan_fine_view.py --object-cloud runtime/vision_dataset/live_fv2 \
        --measurements /tmp/fv2.json --group g00_fv2 --plan-only

    # 规划并执行
    python3 scripts/plan_fine_view.py ... --execute
"""

from __future__ import annotations

import argparse
import json
import pathlib
import subprocess
import sys
import time

import numpy as np

REPO = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))
import coarse_pose as cp                                     # noqa: E402

HANDEYE = REPO / "config" / "handeye" / "tcp_to_camera.json"
TEMPLATE = pathlib.Path.home() / "cs625_rvs_archive/models_20261023/弹仓_装配导出.STL"
CAD_FLIP_X = True        # CAD 坐标上下反了（所有者 2026-11-10 确认）：对象系绕 x 转 180°


def read_tcp() -> np.ndarray | None:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from scipy.spatial.transform import Rotation as R

    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node("plan_fine_view_tcp")
    holder: dict = {}
    node.create_subscription(PoseStamped, "/tcp_pose_broadcaster/pose",
                             lambda msg: holder.__setitem__("tcp", msg), 10)
    deadline = time.time() + 8.0
    while time.time() < deadline and "tcp" not in holder:
        rclpy.spin_once(node, timeout_sec=0.2)
    if "tcp" not in holder:
        return None
    pose = holder["tcp"].pose
    matrix = np.eye(4)
    matrix[:3, :3] = R.from_quat([pose.orientation.x, pose.orientation.y,
                                  pose.orientation.z, pose.orientation.w]).as_matrix()
    matrix[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return matrix


def object_frame_correction(bbox) -> np.ndarray:
    """CAD 坐标反了：对象系绕 bbox 中心转 180°（x 轴），使 +z 对应实物"向上"。"""
    from scipy.spatial.transform import Rotation as R

    centre = (bbox[0] + bbox[1]) / 2
    flip = np.eye(4)
    flip[:3, :3] = R.from_euler("x", 180, degrees=True).as_matrix()
    move = np.eye(4)
    move[:3, 3] = centre
    back = np.eye(4)
    back[:3, 3] = -centre
    return move @ flip @ back


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--measurements", required=True, help="coarse_pose 输出的 JSON")
    parser.add_argument("--group", required=True, help="用哪一组的位姿（应与当前 TCP 同步）")
    parser.add_argument("--distance", type=float, default=0.45)
    parser.add_argument("--elevation", type=float, default=30.0)
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    T_base_tcp = read_tcp()
    if T_base_tcp is None:
        print("读不到 TCP", file=sys.stderr)
        return 2
    handeye = json.loads(HANDEYE.read_text())
    T_tcp_cam = np.array(handeye["T_tcp_cam"])
    T_base_cam = T_base_tcp @ T_tcp_cam

    rows = json.loads(pathlib.Path(args.measurements).read_text())
    row = [item for item in rows if item["group"] == args.group][0]
    T_cam_obj = np.eye(4)
    T_cam_obj[:3, :3] = np.array(row["pose"]["rotation"])
    T_cam_obj[:3, 3] = np.array(row["pose"]["translation"])

    bbox = cp.load_template_bbox(TEMPLATE)
    correction = object_frame_correction(bbox) if CAD_FLIP_X else np.eye(4)
    centre_obj = (bbox[0] + bbox[1]) / 2
    T_base_obj = T_base_cam @ T_cam_obj @ correction
    centre = T_base_obj[:3, :3] @ centre_obj + T_base_obj[:3, 3]
    print(f"工件中心(base): {np.round(centre, 3)}")

    azimuth = np.arctan2(centre[1] - T_base_cam[1, 3], centre[0] - T_base_cam[0, 3])
    elevation = np.radians(args.elevation)
    camera_target = centre - args.distance * np.array(
        [np.cos(elevation) * np.cos(azimuth), np.cos(elevation) * np.sin(azimuth),
         -np.sin(elevation)])
    desired_axis = (centre - camera_target) / np.linalg.norm(centre - camera_target)

    from scipy.spatial.transform import Rotation as R
    current_axis = T_base_tcp[:3, :3][:, 2]
    cross = np.cross(current_axis, desired_axis)
    sine = float(np.linalg.norm(cross))
    cosine = float(np.dot(current_axis, desired_axis))
    swing = (np.eye(3) if sine < 1e-9
             else R.from_rotvec(cross / sine * np.arctan2(sine, cosine)).as_matrix())
    rotation = swing @ T_base_tcp[:3, :3]
    tcp_target = camera_target - rotation @ T_tcp_cam[:3, 3]
    quat = R.from_matrix(rotation).as_quat()
    print(f"精相机(base): {np.round(camera_target, 3)}  距离 {args.distance:.2f} m  "
          f"俯角 {args.elevation:.0f}°")
    print(f"目标 TCP   : {np.round(tcp_target, 4)}  quat {np.round(quat, 5)}")
    print(f"工具 z 轴转动 {np.degrees(np.arccos(np.clip(cosine, -1, 1))):.1f}°（沿用当前滚转）")

    command = [sys.executable, "-u", str(REPO / "scripts" / "plan_pose_once.py"),
               *[f"{v:.6f}" for v in tcp_target], *[f"{v:.6f}" for v in quat],
               "--output", "/tmp/fine_minroll.json"]
    run = subprocess.run(command, capture_output=True, text=True, timeout=300)
    summary = (run.stdout or "").strip().splitlines()
    print("规划:", summary[-1] if summary else "(no output)")
    ok = "code 1 " in (run.stdout or "")
    if args.execute and ok:
        run = subprocess.run([sys.executable, "-u", str(REPO / "scripts" / "execute_assembly_chunks.py"),
                              "/tmp/fine_minroll.json", "--time-scale", "2.5", "--dwell", "0.5",
                              "--goal-tolerance", "0.003"], capture_output=True, text=True, timeout=400)
        print("执行:", "OK" if "ALL_ASSEMBLY_CHUNKS_OK" in (run.stdout or "") else "失败")
    return 0 if ok else 3


if __name__ == "__main__":
    raise SystemExit(main())
