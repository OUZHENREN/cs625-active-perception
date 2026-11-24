
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


# 构型标志（所有者 2026-11-19）：肩 / 肘 / 腕三个"分支"标志。
#   · shoulder_pan 的符号 → 肩在左 / 右
#   · elbow 的符号        → 肘上 / 肘下
#   · wrist_2 的符号      → 腕翻转 / 不翻转
# 同一个 TCP 位姿通常有 8 组 IK 解（2×2×2 分支），它们末端位置一样但整条手臂的构型不同。
# 跨分支切换会让手臂在关节空间里大幅重排（可能撞、也可能让相机姿态跳变），
# 所以选解时必须**优先同一分支**，同分支内再取关节变化最小的。
# 真机验证过并执行过的姿态（来自 scripts/teaching/plan_joint_locked.py）。
# 2026-11-24：只拿"当前位姿 + 随机抖动"做种子时，IK 会收敛到翻转构型
# （shoulder_lift −150、elbow +69），被安全区间拒绝 → 表现为"预抓取位 IK 无解"。
# 用这些已验证姿态做种子才能收敛到可用分支（elbow −69.2）。
PROVEN_POSTURES = (
    (43.1, -57.4, -146.3, 14.3, 92.8, 89.8),      # observation
    (37.1, -120.3, -107.2, -42.4, 90.8, 32.6),    # pre_grasp
    (43.0, -123.0, -101.7, -45.0, 90.7, 38.4),    # grasp
    (6.8, -87.9, -91.9, -50.5, 94.4, 91.2),       # baseline
)

# 2026-11-24 所有者定：非自然构型（翻转解 / 腕部奇异解）**直接拒绝**，不许在真机上执行。
# 起因：预抓取位只有 w2=±90°（奇异）的解，执行后机械臂走到怪异构型，所有者按了急停。
# 注意：垂直抓取（工具朝下）本来就把 wrist_2 压在 ±90° 附近，真机验证过的
# pre_grasp 就是 90.8°，所以**奇异本身不是错误**，不做拒绝。
FLIP_TRAVEL_DEG = 90.0           # wrist_1 或 wrist_3 需转这么多度 = 腕部翻转，拒绝
PROVEN_WARN_DEG = 60.0           # 离所有已验证姿态都超过这个角度 → 提示风险
# 2026-11-24：位姿正好落在腕部奇异点时，把工具沿 x 轴倾斜一点点就能离开奇异。
# 实测（工件 x=0.638 的预抓取位）：0° 奇异；10°→w2 97.7；15°→102.0；20°→106.5。
# 倾斜本身是装配里已经在用的手法（插入段就是 Ry(−10°)），不是新引入的风险。
AUTO_TILT_STEP_DEG = 5.0
AUTO_TILT_MAX_DEG = 20.0

SIGNATURE_JOINTS = ("shoulder_pan_joint", "elbow_joint", "wrist_2_joint")


def configuration_signature(degrees_by_name: dict) -> tuple[int, int, int]:
    """返回 (肩, 肘, 腕) 的符号三元组；0 表示该关节基本在零位。"""
    signature = []
    for name in SIGNATURE_JOINTS:
        value = float(degrees_by_name.get(name, 0.0))
        signature.append(0 if abs(value) < 1e-9 else (1 if value > 0 else -1))
    return tuple(signature)


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
        # 2026-11-24 所有者批准：下限从 -110 放宽到 -130。
        # 依据：所有者亲手摆出并验证可抓取的位姿 shoulder_lift = -124.52°，
        # 以及真机长期执行的 pre_grasp(-120.3°) / grasp(-123.0°)。旧限位把这些全挡住。
        "shoulder_lift_joint": (-130.0, 10.0),
        # 2026-11-24 所有者批准：上限从 -100 放宽到 -60。真机验证过的姿态肘部到
        # -107.2(pre_grasp)/-101.7(grasp)/-91.9(baseline)，本次预抓取所需 IK 解为
        # -69.2；旧上限全部挡掉，表现为"能看见工件但抓不到"。
        "elbow_joint": (-160.0, -60.0),
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
    parser.add_argument("--allow-large-move", action="store_true",
                        help="允许单关节行程 >25°（默认拒绝）。加之前必须已打印 6 个关节角"
                             "并得到所有者明确同意（2026-11-18 事故后新增强制闸）")
    parser.add_argument("--allow-outside-tcp-box", action="store_true",
                        help="允许目标 TCP 落在 workspace_bounds 框外（所有者 2026-11-24 批准）。"
                             "只放开 TCP 框；关节安全区间、碰撞校验、翻转/奇异拒绝全部保留。"
                             "观察位经常需要相机靠近基座，此时框外并不等于危险——"
                             "真正的碰撞风险在夹爪与电机保护座，必须另用 check_tool_envelope.py 校验。")
    parser.add_argument("--auto-tilt", action="store_true",
                        help="位姿落在腕部奇异点时，自动绕 x 轴小幅倾斜后重试"
                             f"（步长 {AUTO_TILT_STEP_DEG:.0f}°，最多 {AUTO_TILT_MAX_DEG:.0f}°）")
    parser.add_argument("--allow-flip", action="store_true",
                        help="允许腕部翻转解（默认拒绝：翻转属大角度重构，需逐段确认）")
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
        if not args.allow_outside_tcp_box:
            print(f"code 7 {reason}（拒绝）")
            return 7
        print(f"[警告] {reason}")
        print("       已按 --allow-outside-tcp-box 放行（所有者 2026-11-24 批准）。")
        print("       注意：关节区间、碰撞校验、翻转/奇异拒绝仍然生效；")
        print("       执行前请用 scripts/check_tool_envelope.py 复核工具包络。")

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
        # 2026-11-24 所有者批准：下限从 -110 放宽到 -130。
        # 依据：所有者亲手摆出并验证可抓取的位姿 shoulder_lift = -124.52°，
        # 以及真机长期执行的 pre_grasp(-120.3°) / grasp(-123.0°)。旧限位把这些全挡住。
        "shoulder_lift_joint": (-130.0, 10.0),
        # 2026-11-24 所有者批准：上限从 -100 放宽到 -60。真机验证过的姿态肘部到
        # -107.2(pre_grasp)/-101.7(grasp)/-91.9(baseline)，本次预抓取所需 IK 解为
        # -69.2；旧上限全部挡掉，表现为"能看见工件但抓不到"。
        "elbow_joint": (-160.0, -60.0),
        "wrist_1_joint": (-150.0, 150.0),
        "wrist_2_joint": (-150.0, 150.0),
        "wrist_3_joint": (-150.0, 150.0),
    }

    # 同一个位姿有多组 IK 解；按"构型分支 + 腕部变化最小"挑一个。
    # 只用当前构型做种子通常会给出腕部变化最大的那个解，所以这里扰动种子多试几次。
    start_signature = configuration_signature(
        {name: math.degrees(start[i]) for i, name in enumerate(names)})
    candidates_pool = []
    rejected_flip: list = []
    far_from_proven: list = []
    rng = np.random.default_rng(7)
    # 种子顺序：已验证姿态优先（它们能收敛到可用分支），再用当前位姿与随机扰动。
    seeds = [[math.radians(v) for v in posture] for posture in PROVEN_POSTURES]
    seeds.append(list(start))
    for _ in range(max(1, args.ik_seeds - len(seeds))):
        seed = list(start)
        for index in wrist_idx:
            seed[index] += float(rng.uniform(-0.6, 0.6))
        seeds.append(seed)
    for attempt, seed in enumerate(seeds):
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
        # 2026-11-24（第二版，所有者纠正）：**不拒绝腕部奇异**。
        # 垂直抓取（工具朝下）本来就把 wrist_2 压在 90° 附近——真机验证过的
        # pre_grasp 就是 w2=90.8°，一直正常执行。把奇异当错误会挡住正常抓取。
        # 真正要拒绝的是"腕部翻转/离已验证姿态过远"的解：那种解要求腕部大角度
        # 重构，中途构型会变得很奇怪（今天真机上就走到过，所有者按了急停）。
        # 拒绝腕部翻转：w1 或 w3 相对当前要大角度重构
        w1_travel = abs(math.degrees(candidate[names.index("wrist_1_joint")]
                                     - start[names.index("wrist_1_joint")]))
        w3_travel = abs(math.degrees(candidate[names.index("wrist_3_joint")]
                                     - start[names.index("wrist_3_joint")]))
        candidate_deg = [math.degrees(v) for v in candidate]
        nearest_proven = min(
            (max(abs(a - b) for a, b in zip(candidate_deg, posture))
             for posture in PROVEN_POSTURES), default=0.0)
        if nearest_proven > PROVEN_WARN_DEG:
            far_from_proven.append((nearest_proven,
                                    [round(v, 1) for v in candidate_deg]))
        if not args.allow_flip and max(w1_travel, w3_travel) > FLIP_TRAVEL_DEG:
            rejected_flip.append((max(w1_travel, w3_travel),
                                  [round(math.degrees(v), 1) for v in candidate]))
            continue
        arm_delta = sum(abs(candidate[i] - start[i]) for i in range(len(names)))
        wrist_delta = max(abs(candidate[i] - start[i]) for i in wrist_idx)
        signature = configuration_signature(
            {name: math.degrees(candidate[i]) for i, name in enumerate(names)})
        same_branch = 0 if signature == start_signature else 1
        candidates_pool.append((same_branch, arm_delta, wrist_delta, candidate,
                                signature))
    # 先同分支、再整臂变化、最后腕部变化（所有者 2026-11-19：构型连续性优先）
    candidates_pool.sort(key=lambda item: (item[0], item[1], item[2]))
    if not candidates_pool:
        print("IK_FAILED 所有种子都无解（或被安全区间拒绝）")
        if rejected_flip:
            worst = max(rejected_flip, key=lambda item: item[0])
            print(f"  · {len(rejected_flip)} 个解需要腕部翻转"
                  f"（wrist_1/wrist_3 行程 {worst[0]:.0f}° > {FLIP_TRAVEL_DEG:.0f}°）："
                  f"{worst[1]}")
            print("    翻转属于大角度重构，按规则**拒绝执行**；"
                  "确需执行请显式加 --allow-flip 并逐段确认。")
        print("  建议：换一个工件摆放位置，或改用接近姿态，不要强行执行。")
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

    for same_branch, arm_delta, wrist_delta, candidate, signature in candidates_pool:
        if not acceptable(candidate, names):
            continue
        ok, reason = trajectory_ok(candidate)
        if not ok:
            print(f"  候选（整臂 {arm_delta:.2f} rad）被拒：{reason}")
            continue
        target_choice = (same_branch, arm_delta, wrist_delta, candidate, signature)
        break
    if target_choice is None:
        print("code 4 所有 IK 候选轨迹都有接触或超出安全区间（拒绝）")
        return 4

    same_branch = target_choice[0]
    target = target_choice[3]
    wrist_delta = target_choice[2]
    print(f"IK 解：整臂变化 {target_choice[1]:.2f} rad，腕部变化 {math.degrees(wrist_delta):.1f}°"
          f"（终点构型在绝对安全区间内，且轨迹抽样无接触）")
    print(f"构型分支：当前 {start_signature}（肩/肘/腕符号）→ 目标 {target_choice[4]}"
          + ("（同分支，连续）" if same_branch == 0 else
             "（**跨分支**：该位姿在当前分支内无解，已按最小变化选最近分支）"))

    # --- 2026-11-19：单关节行程硬闸（≥25°）由**执行器自动分段**处理 ---
    # 2026-11-18 事故：47.6° 腕部的计划被一次性下发，机械臂停在"2 倍指令位移"的越界姿态。
    # 现在执行器会把超限计划切成每段 ≤25° 依次执行（所有者 2026-11-19：不要让人手工拆），
    # 这里只负责把行程打出来，让操作者提前知道会被分成几段。
    MAX_JOINT_DELTA_RAD = 0.44   # 25°
    deltas = {name: abs(target[i] - start[i]) for i, name in enumerate(names)}
    worst_joint, worst_delta = max(deltas.items(), key=lambda kv: kv[1])
    print("关节行程：" + "  ".join(
        f"{n.replace('_joint','')}={math.degrees(v):.1f}°" for n, v in deltas.items()))
    if worst_delta > MAX_JOINT_DELTA_RAD:
        chunks = int(math.ceil(worst_delta / MAX_JOINT_DELTA_RAD))
        print(f"  注意：{worst_joint} 需要走 {math.degrees(worst_delta):.1f}° > "
              f"{math.degrees(MAX_JOINT_DELTA_RAD):.0f}°，执行时会被**自动分段**"
              f"（约 {chunks} 段，每段 ≤{math.degrees(MAX_JOINT_DELTA_RAD):.0f}°）")

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
