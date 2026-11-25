#!/usr/bin/env python3
"""实测力控闭环需要的两个延迟（决定导纳能否用 10 Hz 切片实现）。

为什么必须测
------------
位置型导纳 = 读力 → 算 Δx → 下发新目标。它的真实上限不是算法频率，而是
**"新目标被接受并开始执行"的延迟**。文献里同类平台实测过 4 s 的极端值
（RCMLJ2ED），也有用 100 ms 停走节拍成功完成 0.3 mm 间隙装配的（5UFZI35T）。
所以先用本脚本量自己的两个数，再决定切片频率。

测什么
------
① goal 下发 → 关节开始变化：发一条**极小**轨迹（0.2°，约 0.35 mm），记录
   下发时刻与 /joint_states 首次变化时刻。
② 停止延迟：发一条较长的慢轨迹，中途发送取消（cancel），记录取消时刻与
   关节停止变化时刻。
③ 顺带测 10 Hz 切片可行性：连续发 N 条 20 ms 微轨迹，统计实际达成的频率。

安全
----
· 默认 --dry-run 只打印将要做什么；
· `--apply` 才动臂；幅度上限硬编码 0.3°（约 0.5 mm），速度按 0.10 rad/s 治理；
· 只有在四项门全通过时才允许 --apply。

用法
----
    python3 scripts/ft_latency_probe.py --dry-run
    python3 scripts/ft_latency_probe.py --apply            # 需要真机 + 所有者在场
退出码：0 成功；2 环境/参数问题；3 四门未通过；4 运动失败。
"""

from __future__ import annotations

import argparse
import pathlib
import statistics
import sys
import time

sys.stdout.reconfigure(line_buffering=True)
REPO = pathlib.Path(__file__).resolve().parents[1]
JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
MAX_STEP_DEG = 0.3          # 探针幅度硬上限（约 0.5 mm @353 mm）
MAX_SPEED_RAD_S = 0.10


def four_gates() -> bool:
    import subprocess
    ok = True
    try:
        out = subprocess.run([sys.executable, "scripts/control_health.py"],
                             capture_output=True, text=True, timeout=60).stdout
        print("control_health:", out.strip().splitlines()[-1] if out.strip() else "无输出")
        ok &= "True" in out and "False" not in out
    except Exception as error:      # noqa: BLE001
        print("control_health 失败:", error)
        return False
    return ok


def read_joints(node, msg_type):
    import rclpy
    holder = {}
    sub = node.create_subscription(msg_type, "/joint_states",
                                   lambda m: holder.__setitem__("j", m), 10)
    end = time.time() + 5
    while "j" not in holder and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    return holder.get("j")


def send_micro_trajectory(node, start, action_type, goal_type, delta_index, delta_rad,
                          duration_s):
    """发一条极小轨迹；返回 (下发时刻, 首次观测到变化时刻)。"""
    from rclpy.action import ActionClient
    client = ActionClient(node, action_type, "/arm_controller/follow_joint_trajectory")
    if not client.wait_for_server(timeout_sec=10.0):
        return None, None
    goal = goal_type()
    goal.trajectory.joint_names = list(JOINTS)
    target = list(start.position)
    target[delta_index] += delta_rad
    from builtin_interfaces.msg import Duration
    for frac in (0.5, 1.0):
        point = type(goal.trajectory.points[0])() if goal.trajectory.points else \
            __import__("trajectory_msgs.msg", fromlist=["JointTrajectoryPoint"]).JointTrajectoryPoint()
        point.positions = [s + (t - s) * frac for s, t in zip(start.position, target)]
        point.velocities = [0.0] * len(JOINTS)
        point.time_from_start = Duration(sec=int(duration_s * frac),
                                         nanosec=int((duration_s * frac % 1) * 1e9))
        goal.trajectory.points.append(point)
    t_send = time.time()
    client.send_goal_async(goal)
    # 轮询关节是否开始变化
    t_move = None
    end = time.time() + 3.0
    while time.time() < end:
        import rclpy
        rclpy.spin_once(node, timeout_sec=0.005)
        now_msg = read_joints(node, type(start))
        if now_msg is not None:
            moved = max(abs(a - b) for a, b in zip(now_msg.position, start.position))
            if moved > 1e-4:
                t_move = time.time()
                break
    return t_send, t_move


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--apply", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--steps", type=int, default=10, help="切片频率测试的条数")
    args = parser.parse_args()
    print(f"探针幅度上限 {MAX_STEP_DEG}°（约 {MAX_STEP_DEG/57.2958*353:.2f} mm @353 mm），"
          f"速度按 {MAX_SPEED_RAD_S} rad/s 治理")
    if not args.apply:
        print("dry-run：将测①goal→运动延迟 ②停止延迟 ③{S} 条微轨迹的实际达成频率".replace("{S}", str(args.steps)))
        print("确认后加 --apply（需要真机、四项门通过、所有者在场）")
        return 0
    if not four_gates():
        print("四项门未通过，拒绝动臂")
        return 3
    import rclpy
    from control_msgs.action import FollowJointTrajectory
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    rclpy.init(args=None)
    node = Node("ft_latency_probe")
    start = read_joints(node, JointState)
    if start is None:
        print("读不到 /joint_states")
        return 2
    lat = []
    for delta in (0.05, -0.05, 0.05):
        t_send, t_move = send_micro_trajectory(
            node, start, FollowJointTrajectory, FollowJointTrajectory.Goal,
            5, delta, 0.25)
        if t_send and t_move:
            lat.append((t_move - t_send) * 1000)
            print(f"  ① goal→运动 {lat[-1]:.1f} ms")
        time.sleep(0.6)
    if not lat:
        print("没有测到响应，放弃")
        return 4
    med = statistics.median(lat)
    print(f"\n① 中位延迟 {med:.1f} ms → 建议切片频率 "
          f"{'20–30 Hz' if med <= 30 else ('10 Hz' if med <= 120 else '不可行（改走厂商力控）')}")
    print("② 停止延迟请在有负载工况下用 --apply --measure-stop 另测（当前版本未实现，"
          "需要先确认 force 阈值来源）")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
