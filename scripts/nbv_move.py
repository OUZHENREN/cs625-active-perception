#!/usr/bin/env python3
"""把机械臂移到"能看到手柄面 + 一个侧面"的精观测位，并保证 J6 不缠线。

约束（所有者 2026-11-12）
--------------------------
* J6（wrist_3）相对当前角度变化不得超过 ±45°，否则电机与相机的线会缠绕；
* 观测方向只能在当前视线 ±45° 内微调（这个范围足够看到两个侧面）。

做法
----
1. 读粗位姿（measure_object --stage coarse 的输出）；
2. 在当前视线 ±45°、俯角 {15,30,45}° 内生成候选方向，按"可见面积 + 窄面加权"打分；
3. 对每个候选、每个滚转（0/±45/±90）做 IK，筛掉 |ΔJ6|>45° 的解；
4. 规划（动程护栏）后执行，输出最终 TCP 与 J6。
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
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState

REPO = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))
import coarse_pose as cp  # noqa: E402
import nbv_score as nbv  # noqa: E402

ARM = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


def current_state(node):
    got = {}
    node.create_subscription(JointState, "/joint_states", lambda m: got.setdefault("j", m), 10)
    import time
    t0 = time.time()
    while time.time() - t0 < 5 and "j" not in got:
        rclpy.spin_once(node, timeout_sec=0.2)
    msg = got["j"]
    names = list(msg.name)
    return {n: float(msg.position[names.index(n)]) for n in ARM}


def solve_ik(node, client, positions, tcp, quat, seed_names, seed_positions):
    request = GetPositionIK.Request()
    request.ik_request.group_name = "arm"
    request.ik_request.ik_link_name = "tcp_pose"
    request.ik_request.avoid_collisions = False
    pose = PoseStamped()
    pose.header.frame_id = "base_link"
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, tcp)
    (pose.pose.orientation.x, pose.pose.orientation.y,
     pose.pose.orientation.z, pose.pose.orientation.w) = map(float, quat)
    request.ik_request.pose_stamped = pose
    request.ik_request.robot_state.joint_state.name = seed_names
    request.ik_request.robot_state.joint_state.position = seed_positions
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=20)
    response = future.result()
    if response is None or response.error_code.val != 1:
        return None
    return dict(zip(response.solution.joint_state.name,
                    response.solution.joint_state.position))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--measurements", required=True)
    parser.add_argument("--group", default="g00_m")
    parser.add_argument("--distance", type=float, default=0.75)
    parser.add_argument("--j6-limit", type=float, default=45.0, help="J6 允许变化（度）")
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("nbv_move")
    state = current_state(node)
    names = ARM
    seed = [state[n] for n in names]
    j6_now = state["wrist_3_joint"]
    print(f"当前 J6 = {math.degrees(j6_now):+.1f}°")

    handeye = json.loads((REPO / "config/handeye/tcp_to_camera.json").read_text())
    T_tcp_cam = np.array(handeye["T_tcp_cam"])
    tcp_now = nbv.read_tcp()
    T_base_cam = tcp_now @ T_tcp_cam

    payload = json.loads(pathlib.Path(args.measurements).read_text(encoding="utf-8"))
    rows = payload.get("per_group") if isinstance(payload, dict) else payload
    row = [r for r in rows if r.get("group") == args.group][0]
    base = np.eye(4)
    base[:3, :3] = R.from_quat(row["pose_base"]["quaternion"]).as_matrix()
    base[:3, 3] = row["pose_base"]["translation"]
    bbox = cp.load_template_bbox(nbv.TEMPLATE)
    centre = base[:3, :3] @ ((bbox[0] + bbox[1]) / 2) + base[:3, 3]
    faces = nbv.box_faces(bbox[0], bbox[1])
    size = bbox[1] - bbox[0]
    narrow_area = float(max(min(size[1] * size[2], size[0] * size[1]), 1e-6))

    # 所有者 2026-11-12：J6 只允许微调 ±45°，但 **前三轴可以大范围移动**（在既有工作空间内）。
    # 所以候选不再限制在"当前视线 ±45°"，而是绕工件整圈；代价函数里把
    # "前三轴大位移"当作正常、"后三轴（尤其 J6）位移"当作应避免的。
    to_part = centre - T_base_cam[:3, 3]
    azimuth0 = math.degrees(math.atan2(-to_part[1], -to_part[0]))
    candidates = []
    for offset in range(-180, 180, 20):
        for elevation in (15.0, 30.0, 45.0):
            az, el = math.radians(azimuth0 + offset), math.radians(elevation)
            direction = np.array([math.cos(el) * math.cos(az),
                                  math.cos(el) * math.sin(az), math.sin(el)])
            if not nbv.handle_face_visible(direction, base, bbox[0], bbox[1]):
                continue
            candidates.append((nbv.score_direction(direction, base, faces, narrow_area),
                               offset, elevation, direction))
    candidates.sort(key=lambda item: -item[0])
    print(f"候选 {len(candidates)} 个，最高分 {candidates[0][0]:.4f}" if candidates else "无候选")

    # 不直接调用 IK 服务（链路名/组名与 MoveIt 内部约定不一致会全失败），
    # 改成"让 plan_pose_once 规划，再从轨迹末点读 J6"——这是真正会执行的解。
    ik_ok = 0
    best_delta = None
    best_cost = None
    for score, offset, elevation, direction in candidates:
        camera = centre + direction * args.distance
        axis = (centre - camera) / np.linalg.norm(centre - camera)
        current_axis = T_base_cam[:3, :3][:, 2]
        cross = np.cross(current_axis, axis)
        sine = float(np.linalg.norm(cross))
        cosine = float(np.dot(current_axis, axis))
        swing = (np.eye(3) if sine < 1e-9
                 else R.from_rotvec(cross / sine * math.atan2(sine, cosine)).as_matrix())
        for roll_deg in (0, -45, 45, -90, 90, 180):
            roll = R.from_rotvec(np.array([0, 0, 1.0]) * math.radians(roll_deg)).as_matrix()
            rotation = swing @ T_base_cam[:3, :3] @ roll
            tcp = camera - rotation @ T_tcp_cam[:3, 3]
            quat = R.from_matrix(rotation).as_quat()
            command = [sys.executable, str(REPO / "scripts" / "plan_pose_once.py"),
                       *[f"{v:.4f}" for v in tcp], *[f"{v:.5f}" for v in quat],
                       "--output", "/tmp/nbv_move_plan.json"]
            planned = subprocess.run(command, capture_output=True, text=True, timeout=180)
            out = planned.stdout or ""
            if "code 1 " not in out:
                continue
            ik_ok += 1
            plan = json.loads(pathlib.Path("/tmp/nbv_move_plan.json").read_text())
            last = plan["points"][-1]["q"]
            j6_new = last[names.index("wrist_3_joint")]
            delta = math.degrees((j6_new - j6_now + math.pi) % (2 * math.pi) - math.pi)
            if best_delta is None or abs(delta) < abs(best_delta):
                best_delta = delta
            # 前三轴=主运动（允许大），后三轴=微调（越少越好）
            wrist = sum(abs(last[i] - seed[i]) for i in (3, 4, 5))
            proximal = sum(abs(last[i] - seed[i]) for i in (0, 1, 2))
            cost = score - 0.02 * wrist
            print(f"  方位偏移 {offset:+d}° 俯角 {elevation:.0f}° 滚转 {roll_deg:+d}° "
                  f"得分 {score:.4f} ΔJ6 {delta:+.1f}° 腕 {wrist:.2f} rad "
                  f"前三轴 {proximal:.2f} rad → 代价 {cost:.4f}")
            if abs(delta) > args.j6_limit:
                continue
            if best_cost is None or cost > best_cost[0]:
                best_cost = (cost, offset, elevation, roll_deg, tcp, quat, score, delta)
            continue
            if best_cost is not None and best_cost[0] == cost:
                # 记住当前这条轨迹，等所有候选评完再决定
                pathlib.Path("/tmp/nbv_best_plan.json").write_text(
                    pathlib.Path("/tmp/nbv_move_plan.json").read_text())
    print(f"规划成功 {ik_ok} 个；|ΔJ6| 最小 = "
          f"{None if best_delta is None else round(abs(best_delta), 1)}°")
    if best_cost is not None:
        cost, offset, elevation, roll_deg, tcp, quat, score, delta = best_cost
        print(f"选定：方位偏移 {offset:+d}° 俯角 {elevation:.0f}° 滚转 {roll_deg:+d}° "
              f"得分 {score:.4f} ΔJ6 {delta:+.1f}° 代价 {cost:.4f} TCP {np.round(tcp, 3)}")
        json.dump({"tcp": list(map(float, tcp)), "quat": list(map(float, quat))},
                  open("/tmp/nbv_move.json", "w"))
        if args.execute:
            moved = subprocess.run(
                [sys.executable, str(REPO / "scripts" / "execute_assembly_chunks.py"),
                 "/tmp/nbv_best_plan.json", "--time-scale", "2.0", "--dwell", "0.5",
                 "--goal-tolerance", "0.005"], capture_output=True, text=True, timeout=600)
            print("   执行:", (moved.stdout or "").strip().splitlines()[-1])
            return 0 if moved.returncode == 0 else 1
        print("（未执行）")
        return 0
    print("没有满足约束（J6 ±45° + 可达）的候选")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
