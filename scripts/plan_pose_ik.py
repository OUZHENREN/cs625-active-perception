
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



import argparse
import json
import math
import pathlib
import sys

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


def acceptable(candidate: list[float]) -> bool:
    """拒绝危险构型（2026-11-13 所有者警告：J2 过低会撞桌面；J6 超 ±180° 会缠线）。

    判据（按 JOINT_ORDER 的索引：0 肩转、1 肩抬、2 肘、3/4/5 腕）：
      · wrist_3 绝对值 > 170°          → 拒绝
      · 肩抬 < 5° 且 肘伸展 > 100°（手臂横伸到桌面高度）→ 拒绝
      · 肩转与当前相差 > 90°（换到了另一种翻转构型）→ 拒绝
    """
    import math as _m
    pan, lift, elbow = candidate[0], candidate[1], candidate[2]
    wrist_3 = candidate[5]
    if abs(_m.degrees(wrist_3)) > 170.0:
        return False
    if _m.degrees(lift) < 5.0 and abs(_m.degrees(elbow)) > 100.0:
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

    # 移动前硬性自检：当前状态已接触则一律不动（agent_memory 135）
    if not require_current_state_valid(node, names, start):
        return 6

    client = node.create_client(GetPositionIK, "/compute_ik")
    if not client.wait_for_service(timeout_sec=10):
        print("IK 服务不可用")
        return 4
    wrist_idx = [i for i, name in enumerate(names) if name in
                 ("wrist_1_joint", "wrist_2_joint", "wrist_3_joint")]

    # 同一个位姿有多组 IK 解；按"腕部变化最小"挑一个，避免为了视角把腕子拧过去。
    # 只用当前构型做种子通常会给出腕部变化最大的那个解，所以这里扰动种子多试几次。
    best = None
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
        if not acceptable(candidate):
            continue
        # 换构型（肩转相对当前 >90°）一律不要：实测那正是"横伸到桌面"的那一支
        if abs(_wrap_deg(candidate[0] - start[0])) > 90.0:
            continue
        # 选解标准：整条手臂的关节变化量（前三轴也算），避免换到另一种"翻转构型"
        arm_delta = sum(abs(candidate[i] - start[i]) for i in range(len(names)))
        wrist_delta = max(abs(candidate[i] - start[i]) for i in wrist_idx)
        if best is None or (arm_delta, wrist_delta) < (best[0], best[1]):
            best = (arm_delta, wrist_delta, candidate)
    if best is None:
        print("IK_FAILED 所有种子都无解")
        return 5
    target = best[2]
    print(f"IK 解：整臂变化 {best[0]:.2f} rad，腕部最大变化 {math.degrees(best[1]):.1f}°")
    wrist_delta = max(abs(target[i] - start[i]) for i in wrist_idx)
    max_delta = max(abs(target[i] - start[i]) for i in range(len(names)))
    if wrist_delta > math.radians(args.max_wrist_delta):
        print(f"code 3 wrist_delta {wrist_delta:.2f} rad -- 腕部转动超过 "
              f"{args.max_wrist_delta:.0f}°（拒绝）")
        return 3
    if max_delta > args.max_joint_delta:
        print(f"code 2 max_joint_delta {max_delta:.2f} rad -- 超过单关节上限（拒绝）")
        return 2

    points = []
    # 第一个点必须是【当前关节角】：执行器会用 FIRST_POINT_FAR 拒绝"起点不符"的轨迹。
    points.append({"q": list(start), "t": 0.0})
    for step in range(1, args.steps + 1):
        alpha = step / args.steps
        q = [start[i] + (target[i] - start[i]) * alpha for i in range(len(names))]
        points.append({"q": q, "t": 6.0 * alpha})
    duration = points[-1]["t"]
    if duration > args.max_duration:
        print(f"code 2 duration {duration:.1f}s -- 超过时长上限（拒绝）")
        return 2

    checker = node.create_client(GetStateValidity, "/check_state_validity")
    if checker.wait_for_service(timeout_sec=5):
        # 不检查第 0 点：它必然是"当前状态"，而当前模型对 forearm_link↔tool0 这类
        # 相邻连杆报的是常态接触（实测当前位置就被判 invalid），拿它当"碰撞"没意义。
        for index in sorted({len(points) // 3, 2 * len(points) // 3, len(points) - 1}):
            request_valid = GetStateValidity.Request()
            robot_state = RobotState()
            robot_state.joint_state.name = names
            robot_state.joint_state.position = points[index]["q"]
            request_valid.robot_state = robot_state
            request_valid.group_name = GROUP
            future_valid = checker.call_async(request_valid)
            rclpy.spin_until_future_complete(node, future_valid, timeout_sec=8)
            result = future_valid.result()
            if result is not None and not result.valid and contacts_are_real(result.contacts):
                pairs = [(c.contact_body_1, c.contact_body_2) for c in result.contacts[:3]]
                print(f"code 4 collision/invalid at point {index} {pairs}（拒绝）")
                return 4
    else:
        print("warn: /check_state_validity 不可用，跳过碰撞抽样")

    pathlib.Path(args.output).write_text(json.dumps({
        "joint_names": names,
        "points": points,
    }))
    print(f"code 1 points {len(points)} duration {duration:.2f}s wrist_delta {wrist_delta:.2f} rad")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
