
#!/usr/bin/env python3
"""Plan a single TCP pose goal and dump the joint trajectory (plan only).

Why this is a tracked script
----------------------------
The coarse->fine pipeline used to shell out to ``/tmp/plan_pose.py``.  A pipeline
that depends on an untracked file in ``/tmp`` breaks silently after a reboot, and
that script also hard-coded assembly-grade tolerances (0.5 mm position sphere,
0.001 rad orientation).  Those tolerances are right for insertion, where a
"successful" 2.9 deg misalignment is a false success, but they make an ordinary
transit to an observation pose fail: the planner has to hit one exact point in
Cartesian space and OMPL usually gives up (measured: code 99999 on a pose whose
IK exists and whose path is collision-free).

An observation goal only has to put the camera in the right place - the next
stage re-derives the target from the new frame - so 5 mm / 0.02 rad is enough.

Plan only: nothing is executed here.

Usage:
  python3 scripts/plan_pose_once.py X Y Z QX QY QZ QW \
      [--velocity 0.10] [--position-tolerance 0.005] \
      [--orientation-tolerance 0.02] [--output /tmp/jm.json]

Prints ``code <MoveItErrorCodes> points <n> duration <s>`` and exits 0 on
success.  The trajectory JSON is written for scripts/execute_assembly_chunks.py.
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
import pathlib
import sys
import time

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (BoundingVolume, Constraints,
                             OrientationConstraint, PositionConstraint)
from moveit_msgs.srv import GetMotionPlan as GetMotionPlanSrv
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

sys.stdout.reconfigure(line_buffering=True)

FRAME = "world"          # the MoveIt model frame; identity with base on this cell
GROUP = "cs625_arm"
TIP = "my_end_effector_link"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("pose", nargs=7, type=float, metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"))
    parser.add_argument("--velocity", type=float, default=0.05,
                        help="velocity AND acceleration scaling for the plan.  On this "
                             "cell 0.05 IS the normal speed (the owner's words), so do "
                             "not treat it as a slow-down; assembly segments drop to "
                             "0.005-0.01")
    parser.add_argument("--position-tolerance", type=float, default=0.005,
                        help="position sphere radius in metres")
    parser.add_argument("--orientation-tolerance", type=float, default=0.02,
                        help="per-axis orientation tolerance in radians")
    parser.add_argument("--attempts", type=int, default=10)
    parser.add_argument("--budget", type=float, default=15.0, help="planning seconds")
    parser.add_argument("--output", default="/tmp/jm.json")
    parser.add_argument("--frame", default=FRAME)
    parser.add_argument("--max-joint-delta", type=float, default=2.0,
                        help="允许的单关节最大变化（rad）；超过就拒绝该规划（默认 2.0 rad ≈ 115°）")
    parser.add_argument("--max-wrist-delta", type=float, default=45.0,
                        help="腕部三个关节（J4/J5/J6）相对起始姿态的最大变化（度）；"
                             "超过就拒绝——大角度转腕会让法兰与机械臂自碰撞")
    parser.add_argument("--check-collision", action="store_true", default=True,
                        help="用 /check_state_validity 抽样验证规划轨迹（默认开）")
    parser.add_argument("--max-duration", type=float, default=90.0,
                        help="允许的轨迹最长时长（s）；超过就拒绝")
    arguments = parser.parse_args()

    xyz = arguments.pose[:3]
    quat = arguments.pose[3:]

    rclpy.init()
    node = Node("plan_pose_once")
    client = node.create_client(GetMotionPlanSrv, "/plan_kinematic_path")
    if not client.wait_for_service(timeout_sec=10.0):
        print("plan service unavailable")
        return 4

    # 移动前硬性自检：当前状态已接触则一律不动（agent_memory 135）
    guard_state: dict = {}
    node.create_subscription(JointState, "/joint_states",
                             lambda msg: guard_state.setdefault("j", msg), 10)
    deadline = time.time() + 5.0
    while time.time() < deadline and "j" not in guard_state:
        rclpy.spin_once(node, timeout_sec=0.2)
    if "j" in guard_state:
        if not require_current_state_valid(node, guard_state["j"].name,
                                           guard_state["j"].position):
            return 6
    else:
        print("[guard] 读不到当前关节状态，拒绝规划")
        return 6

    constraints = Constraints()
    position = PositionConstraint()
    position.header.frame_id = arguments.frame
    position.link_name = TIP
    volume = BoundingVolume()
    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [arguments.position_tolerance]
    volume.primitives.append(sphere)
    centre = Pose()
    centre.position.x, centre.position.y, centre.position.z = xyz
    centre.orientation.w = 1.0
    volume.primitive_poses.append(centre)
    position.constraint_region = volume
    position.weight = 1.0
    constraints.position_constraints.append(position)

    orientation = OrientationConstraint()
    orientation.header.frame_id = arguments.frame
    orientation.link_name = TIP
    (orientation.orientation.x, orientation.orientation.y,
     orientation.orientation.z, orientation.orientation.w) = quat
    orientation.absolute_x_axis_tolerance = arguments.orientation_tolerance
    orientation.absolute_y_axis_tolerance = arguments.orientation_tolerance
    orientation.absolute_z_axis_tolerance = arguments.orientation_tolerance
    orientation.weight = 1.0
    constraints.orientation_constraints.append(orientation)

    request = GetMotionPlanSrv.Request()
    motion = request.motion_plan_request
    motion.group_name = GROUP
    motion.goal_constraints.append(constraints)
    motion.num_planning_attempts = arguments.attempts
    motion.allowed_planning_time = arguments.budget
    motion.max_velocity_scaling_factor = arguments.velocity
    motion.max_acceleration_scaling_factor = arguments.velocity

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=arguments.budget + 25.0)
    response = future.result()
    if response is None:
        print("plan timed out")
        return 5

    code = response.motion_plan_response.error_code.val
    trajectory = response.motion_plan_response.trajectory.joint_trajectory
    points = trajectory.points
    if code != 1 or not points:
        print(f"code {code} points {len(points)}")
        return 6

    duration = points[-1].time_from_start.sec + points[-1].time_from_start.nanosec * 1e-9

    # 动程限制（2026-11-12 所有者反馈"又在无限制地移动"）：
    # 关节空间规划对远处目标可能给出上百秒、上千点的大幅摆动，看起来像"乱动"。
    # 这里在写轨迹之前就量一下"每个关节的最大变化"和"总时长"，超限直接拒绝，
    # 要求调用方改成分段的笛卡尔移动或更近的中继点。
    import numpy as _np
    max_delta = max(float(_np.max(_np.abs(_np.asarray(p.positions) - _np.asarray(points[0].positions))))
                    for p in points)
    # 腕部限位（2026-11-12：因为大角度转腕导致法兰与机械臂自碰撞，所有者明确指出）
    wrist_indices = [i for i, name in enumerate(trajectory.joint_names)
                     if name in ("wrist_1_joint", "wrist_2_joint", "wrist_3_joint")]
    base_positions = list(points[0].positions)
    wrist_delta = max(
        (abs(point.positions[index] - base_positions[index]) for point in points
         for index in wrist_indices),
        default=0.0)
    wrist_limit = float(_np.radians(arguments.max_wrist_delta))
    if wrist_delta > wrist_limit:
        print(f"code 3 points {len(points)} duration {duration:.2f}s "
              f"wrist_delta {wrist_delta:.2f} rad -- REFUSED: 腕部转动超过 "
              f"{arguments.max_wrist_delta:.0f}°（会自碰撞）")
        return 3

    if max_delta > arguments.max_joint_delta or duration > arguments.max_duration:
        print(f"code 2 points {len(points)} duration {duration:.2f}s "
              f"max_joint_delta {max_delta:.2f} rad -- REFUSED by motion bounds "
              f"(limit {arguments.max_joint_delta:.2f} rad / {arguments.max_duration:.1f} s); "
              f"split into shorter segments")
        return 2
    # 自碰撞抽样验证：MoveIt 的规划器会检查碰撞，但"关节空间远距离规划"仍可能给出
    # 擦着自身走的路径；这里用 /check_state_validity 对轨迹抽样复核（含自碰撞）。
    # 2026-11-12 就是因为大角度转腕导致法兰撞到机械臂本体，所有者当场指出。
    if arguments.check_collision and wrist_indices:
        from moveit_msgs.msg import RobotState as _RobotState
        from moveit_msgs.srv import GetStateValidity as _GetStateValidity
        checker = node.create_client(_GetStateValidity, "/check_state_validity")
        if checker.wait_for_service(timeout_sec=5.0):
            import numpy as _np2
            sample_indices = sorted(set(
                _np2.linspace(0, len(points) - 1, min(8, len(points))).astype(int).tolist()))
            bad = None
            for index in sample_indices:
                request = _GetStateValidity.Request()
                state = _RobotState()
                state.joint_state.name = list(trajectory.joint_names)
                state.joint_state.position = [float(v) for v in points[index].positions]
                request.robot_state = state
                request.group_name = GROUP
                future_valid = checker.call_async(request)
                rclpy.spin_until_future_complete(node, future_valid, timeout_sec=8.0)
                result = future_valid.result()
                if result is not None and not result.valid and contacts_are_real(result.contacts):
                    bad = index
                    break
            if bad is not None:
                print(f"code 4 points {len(points)} duration {duration:.2f}s "
                      f"collision_at_point {bad} -- REFUSED: 轨迹抽样处于自碰撞/非法状态")
                return 4
        else:
            print("warn: /check_state_validity 不可用，跳过碰撞抽样")

    pathlib.Path(arguments.output).write_text(json.dumps({
        "joint_names": list(trajectory.joint_names),
        "points": [{"q": list(p.positions), "t": p.time_from_start.sec
                    + p.time_from_start.nanosec * 1e-9} for p in points],
    }))
    print(f"code {code} points {len(points)} duration {duration:.2f}s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
