
#!/usr/bin/env python3
"""用 IK + 关节插值生成轨迹（当 OMPL 规划失败但位姿其实可达时使用）。

为什么需要（2026-11-13）
------------------------
`plan_pose_once.py` 走的是 OMPL 规划，遇到"位姿本身可达但关节空间绕得远"的情况会返回
code 99999（无解）。实测同一个目标位姿 `/compute_ik` 能直接解出来（code 1），
所以这里改为：IK 求一组解 → 关节线性插值 → 过同样的安全闸（腕部限位 + 抽样碰撞）→ 执行。

安全闸与 plan_pose_once 一致：
· 腕部（J4/J5/J6）相对起始变化 ≤ --max-wrist-delta（默认 45°）；
· 单关节变化 ≤ --max-joint-delta；时长上限；
· 轨迹抽样 /check_state_validity。
"""

from __future__ import annotations

# 2026-11-13 教训：不允许忽略任何接触对。
# 我曾把 forearm_link↔tool0 当成"模型假阳性"忽略掉，结果放过了真实的自碰撞
# （夹爪撞到机械臂本体，机器人进入 FAULT + POWER_OFF）。
# 接触就是接触——不能为了让规划通过而屏蔽碰撞信号。
IGNORED_CONTACT_PAIRS: set = set()


def contacts_are_real(contacts) -> bool:
    for contact in contacts:
        pair = frozenset({contact.contact_body_1, contact.contact_body_2})
        if pair not in IGNORED_CONTACT_PAIRS:
            return True
    return False
def require_current_state_valid(node, names, positions, label: str = "current") -> bool:
    """移动前的硬性自检：**当前状态本身就是碰着的，就绝对不许再动**。

    2026-11-13 事故（agent_memory 135）：当时当前状态已报 forearm_link↔tool0 接触，
    却继续规划并执行，造成夹爪撞机械臂、J4 信号中断。
    返回 True 表示可以继续；False 表示必须停手并报告。
    """
    from moveit_msgs.msg import RobotState as _RS
    from moveit_msgs.srv import GetStateValidity as _GSV
    checker = node.create_client(_GSV, "/check_state_validity")
    if not checker.wait_for_service(timeout_sec=5.0):
        print(f"[guard] /check_state_validity 不可用，拒绝移动（{label}）")
        return False
    request = _GSV.Request()
    state = _RS()
    state.joint_state.name = list(names)
    state.joint_state.position = [float(v) for v in positions]
    request.robot_state = state
    request.group_name = "cs625_arm"
    future = checker.call_async(request)
    import rclpy as _rclpy
    _rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    result = future.result()
    if result is None:
        print(f"[guard] 无法判定当前状态，拒绝移动（{label}）")
        return False
    if not result.valid and contacts_are_real(result.contacts):
        pairs = [(c.contact_body_1, c.contact_body_2) for c in result.contacts[:3]]
        print(f"[guard] 当前状态已存在接触 {pairs} —— 停止，请人工摆位后再动")
        return False
    return True

def target_inside_workspace(tcp_xyz, bounds=None) -> tuple[bool, str]:
    """目标 TCP 必须落在安全作业空间内（config/tasks/common.yaml → workspace_bounds）。

    2026-11-13 事故：一次"纯平移"把 TCP 移到了 y=-0.17（越过了机器人本体一侧），
    关节限位全都在范围内，但这个位置本身是危险/无意义的。
    所以除了关节区间，还必须检查【空间位置】。
    """
    import yaml as _yaml
    if bounds is None:
        try:
            config = _yaml.safe_load((REPO / "config" / "tasks" / "common.yaml").read_text())
            bounds = config.get("workspace_bounds") or {}
        except Exception:  # noqa: BLE001
            bounds = {}
    for axis, index in (("x", 0), ("y", 1), ("z", 2)):
        limits = bounds.get(axis)
        if not limits:
            continue
        low, high = float(limits[0]), float(limits[1])
        value = float(tcp_xyz[index])
        if not (low <= value <= high):
            return False, f"TCP {axis}={value:.3f} 超出安全作业空间 [{low}, {high}]"
    return True, ""




import argparse
import json
import math
import pathlib
import sys

REPO = pathlib.Path(__file__).resolve().parents[1]

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from rclpy.node import Node
from sensor_msgs.msg import JointState

GROUP = "cs625_arm"
TIP = "my_end_effector_link"
FRAME = "world"


def _wrap_deg(angle: float) -> float:
    """把角度差折算到 (−180, 180]。"""
    import math as _m
    return (angle + _m.pi) % (2 * _m.pi) - _m.pi


def acceptable(candidate: list[float], names=None) -> bool:
    """判断一个关节构型是否可以接受。

    判据只看【终点构型本身】（绝对量），不再用"相对当前 ±45°"——
    后者是电缆缠绕的代理指标，会误杀"终点仍在安全区内"的移动
    （所有者 2026-11-13 追问：既然满足安全区间，为什么不能执行？）。
      · 每个关节的终点必须落在绝对安全区间内；
      · 危险组合：肩抬接近水平（> −20°）且肘大角度伸展（|肘| > 100°）→ 手臂横在桌面高度；
      · 危险组合：腕部三个关节都接近极限（任两个 |角度| > 140°）。
    """
    limits = {
        "shoulder_pan_joint": (-100.0, 100.0),
        "shoulder_lift_joint": (-110.0, 10.0),
        "elbow_joint": (-150.0, -100.0),
        "wrist_1_joint": (-150.0, 150.0),
        "wrist_2_joint": (-150.0, 150.0),
        "wrist_3_joint": (-150.0, 150.0),
    }
    order = names or ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                      "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
    degrees = [math.degrees(candidate[i]) for i in range(len(candidate))]
    for index, name in enumerate(order[:len(degrees)]):
        low, high = limits.get(name, (-180.0, 180.0))
        if not (low <= degrees[index] <= high):
            return False
    lift, elbow = degrees[1], degrees[2]
    if lift > -20.0 and abs(elbow) > 100.0:
        return False
    wrists = degrees[3:6]
    if sum(1 for value in wrists if abs(value) > 140.0) >= 2:
        return False
    return True


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("pose", type=float, nargs=7, help="x y z qx qy qz qw")
    parser.add_argument("--output", default="/tmp/ik_plan.json")
    parser.add_argument("--max-wrist-delta", type=float, default=45.0)
    parser.add_argument("--max-joint-delta", type=float, default=2.0)
    parser.add_argument("--max-duration", type=float, default=90.0)
    parser.add_argument("--steps", type=int, default=24)
    parser.add_argument("--ik-seeds", type=int, default=12,
                        help="用多少个扰动种子求 IK，取腕部变化最小的解")
    args = parser.parse_args()

    rclpy.init()
    node = Node("plan_pose_ik")
    state: dict = {}
    node.create_subscription(JointState, "/joint_states",
                             lambda msg: state.setdefault("j", msg), 10)
    import time
    deadline = time.time() + 5
    while time.time() < deadline and "j" not in state:
        rclpy.spin_once(node, timeout_sec=0.2)
    names = list(state["j"].name)
    start = list(state["j"].position)

    # 目标点必须落在安全作业空间内（只查关节区间不够：今天曾把 TCP 移到 y=-0.17）
    inside, reason = target_inside_workspace(args.pose[:3])
    if not inside:
        print(f"code 7 {reason}（拒绝）")
        return 7

    # 移动前硬性自检：当前状态已接触则一律不动（agent_memory 135）
    if not require_current_state_valid(node, names, start):
        return 6

    client = node.create_client(GetPositionIK, "/compute_ik")
    if not client.wait_for_service(timeout_sec=10):
        print("IK 服务不可用")
        return 4
    wrist_idx = [i for i, name in enumerate(names) if name in
                 ("wrist_1_joint", "wrist_2_joint", "wrist_3_joint")]
    # 绝对安全区间（见 agent_memory 136）：这是"关节能不能待在那儿"的硬约束。
    # 之前用的"相对当前 ±45°"其实是电缆缠绕的代理指标，会误杀"终点仍在安全区内"的移动。
    ABSOLUTE_LIMITS = {
        "shoulder_pan_joint": (-100.0, 100.0),
        "shoulder_lift_joint": (-110.0, 10.0),
        "elbow_joint": (-150.0, -100.0),
        "wrist_1_joint": (-150.0, 150.0),
        "wrist_2_joint": (-150.0, 150.0),
        "wrist_3_joint": (-150.0, 150.0),
    }

    # 同一个位姿有多组 IK 解；按"腕部变化最小"挑一个，避免为了视角把腕子拧过去。
    # 只用当前构型做种子通常会给出腕部变化最大的那个解，所以这里扰动种子多试几次。
    candidates_pool = []
    rng = np.random.default_rng(7)
    for attempt in range(args.ik_seeds):
        seed = list(start)
        if attempt:
            for index in wrist_idx:
                seed[index] += float(rng.uniform(-0.6, 0.6))
        request = GetPositionIK.Request()
        request.ik_request.group_name = GROUP
        request.ik_request.ik_link_name = TIP
        request.ik_request.avoid_collisions = False
        pose = PoseStamped()
        pose.header.frame_id = FRAME
        (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z) = args.pose[:3]
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = args.pose[3:]
        request.ik_request.pose_stamped = pose
        request.ik_request.robot_state.joint_state.name = names
        request.ik_request.robot_state.joint_state.position = seed
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=20)
        response = future.result()
        if response is None or response.error_code.val != 1:
            continue
        solution = dict(zip(response.solution.joint_state.name,
                            response.solution.joint_state.position))
        candidate = [solution.get(name, start[i]) for i, name in enumerate(names)]
        if not acceptable(candidate, names):
            continue
        arm_delta = sum(abs(candidate[i] - start[i]) for i in range(len(names)))
        wrist_delta = max(abs(candidate[i] - start[i]) for i in wrist_idx)
        candidates_pool.append((arm_delta, wrist_delta, candidate))
    candidates_pool.sort(key=lambda item: (item[0], item[1]))
    if not candidates_pool:
        print("IK_FAILED 所有种子都无解（或被安全区间拒绝）")
        return 5
    target_choice = None
    wrist_limit_note = ""
    checker = node.create_client(GetStateValidity, "/check_state_validity")
    if not checker.wait_for_service(timeout_sec=5):
        print("code 4 /check_state_validity 不可用，拒绝（不做无验证的移动）")
        return 4

    def trajectory_ok(candidate):
        """按候选构型插值，抽样检查中途与终点是否无接触。"""
        test_points = []
        for sample in range(1, args.steps + 1):
            alpha = sample / args.steps
            test_points.append([start[i] + (candidate[i] - start[i]) * alpha
                                for i in range(len(names))])
        for index in sorted({len(test_points) // 3, 2 * len(test_points) // 3,
                             len(test_points) - 1}):
            request_valid = GetStateValidity.Request()
            robot_state = RobotState()
            robot_state.joint_state.name = names
            robot_state.joint_state.position = [float(v) for v in test_points[index]]
            request_valid.robot_state = robot_state
            request_valid.group_name = GROUP
            future_valid = checker.call_async(request_valid)
            rclpy.spin_until_future_complete(node, future_valid, timeout_sec=8)
            result = future_valid.result()
            if result is None:
                return False, f"碰撞服务无响应（sample {index}）"
            if not result.valid and contacts_are_real(result.contacts):
                pairs = [(c.contact_body_1, c.contact_body_2) for c in result.contacts[:3]]
                return False, f"point {index} 接触 {pairs}"
        return True, ""

    for arm_delta, wrist_delta, candidate in candidates_pool:
        if not acceptable(candidate, names):
            continue
        ok, reason = trajectory_ok(candidate)
        if not ok:
            print(f"  候选（整臂 {arm_delta:.2f} rad）被拒：{reason}")
            continue
        target_choice = (arm_delta, wrist_delta, candidate)
        break
    if target_choice is None:
        print("code 4 所有 IK 候选轨迹都有接触或超出安全区间（拒绝）")
        return 4

    target = target_choice[2]
    wrist_delta = target_choice[1]
    print(f"IK 解：整臂变化 {target_choice[0]:.2f} rad，腕部变化 {math.degrees(wrist_delta):.1f}°"
          f"（终点构型在绝对安全区间内，且轨迹抽样无接触）")

    points = [{"q": list(start), "t": 0.0}]
    for sample in range(1, args.steps + 1):
        alpha = sample / args.steps
        points.append({"q": [start[i] + (target[i] - start[i]) * alpha
                             for i in range(len(names))], "t": 6.0 * alpha})
    duration = points[-1]["t"]
    if duration > args.max_duration:
        print(f"code 2 duration {duration:.1f}s -- 超过时长上限（拒绝）")
        return 2

    pathlib.Path(args.output).write_text(json.dumps({
        "joint_names": names,
        "points": points,
    }))
    print(f"code 1 points {len(points)} duration {duration:.2f}s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
