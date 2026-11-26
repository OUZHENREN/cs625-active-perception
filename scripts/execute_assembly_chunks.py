#!/usr/bin/env python3
"""Execute pre-planned assembly chunks slowly, with visible dwell between chunks.

Assembly means grasp / insert / extract.  The input JSON chunks use the same
[name, points, time_from_start] layout as the temporary planners.

Example:
  python3 scripts/execute_assembly_chunks.py '/tmp/grasp_chunk*.json' \
      --time-scale 2.0 --dwell 1.0
  # simulation (same trajectory, different controller name):
  python3 scripts/execute_assembly_chunks.py '/tmp/grasp_chunk*.json' \
      --action /joint_trajectory_controller/follow_joint_trajectory
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import pathlib
import subprocess
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


# Driver status codes, for turning a bare failure into a cause.  These mirror
# eli_common_interface/msg/{SafetyMode,RobotMode}.msg; they are repeated here so
# the script still starts when only the repo overlay is sourced.
SAFETY_MODE_NAMES = {
    1: "NORMAL", 2: "REDUCED", 3: "PROTECTIVE_STOP", 4: "RECOVERY",
    5: "SAFEGUARD_STOP", 6: "SYSTEM_EMERGENCY_STOP", 7: "ROBOT_EMERGENCY_STOP",
    8: "VIOLATION", 9: "FAULT", 10: "VALIDATE_JOINT_ID", 11: "UNDEFINED",
    12: "AUTOMATIC_MODE_SAFEGUARD_STOP", 13: "SYSTEM_THREE_POSITION_ENABLING_STOP",
    14: "TP_THREE_POSITION_ENABLING_STOP",
}
ROBOT_MODE_NAMES = {
    -2: "UNKNOWN", -1: "NO_CONTROLLER", 0: "DISCONNECTED", 1: "CONFIRM_SAFETY",
    2: "BOOTING", 3: "POWER_OFF", 4: "POWER_ON", 5: "IDLE", 6: "BACKDRIVE",
    7: "RUNNING", 8: "UPDATING_FIRMWARE", 9: "WAITING_CALIBRATION",
}
# FollowJointTrajectory result error codes.
TRAJECTORY_ERROR_NAMES = {
    0: "SUCCESSFUL", -1: "INVALID_GOAL", -2: "INVALID_JOINTS",
    -3: "OLD_HEADER_TIMESTAMP", -4: "PATH_TOLERANCE_VIOLATED",
    -5: "GOAL_TOLERANCE_VIOLATED",
}

# Real-arm action name.  Simulation uses a differently named controller
# (/joint_trajectory_controller/follow_joint_trajectory, see
# cs625_bringup/config/sim_controllers.yaml); config/tasks/world_sim.yaml
# overrides it through this flag, so the default stays the real-robot contract.
DEFAULT_ACTION = "/arm_controller/follow_joint_trajectory"

# --- 2026-11-19 硬性安全闸（所有者要求：速度与行程不允许任何逾越）-----------------
# 执行器**不再信任计划文件里的时间戳**：它按这两个上限重新配时/拒绝执行。
# 2026-11-18 事故：一份关节行程 47.6° 的计划被直接下发，机械臂最终停在"2 倍指令位移"
# 的位置并越界（J1 128.7°、J6 156.8°），随后驱动失联。此后速度与行程由代码强制。
MAX_JOINT_SPEED_RAD_S = 0.10     # 单关节速度硬上限 ≈ 5.7 °/s（所有者 2026-11-20 批准，原 0.05）
# 为什么提高：2.86°/s 时控制周期（8 ms）内只前进 0.023°，已接近伺服死区，
# 表现为「来回抖动」、累积落后；落后超过控制器的 0.2 rad 终点容差就会
# Aborted due to state tolerance violation（走不到终点）。5.7°/s 仍远低于
# 任何示教/装配需求，是原有速度的 2 倍。
MAX_JOINT_DELTA_RAD = 0.44       # 单个计划文件里任一关节的行程硬上限 = 25°

# 2026-11-24 所有者批准：**绝对安全的换位段**可以按 0.20 rad/s（11.5°/s）跑。
# 只对下面这些阶段生效（白名单在代码里，不在调用方），其余一律 0.10。
# 依据：这些段是"去固定示教观测位"和"从观测位到预抓取位上方"，不携带工件、
# 不接触任何东西，也不在工件的 0.2 m 接近半径内。
HARD_MAX_JOINT_SPEED_RAD_S = 0.20   # 任何调用方都不得超过这个绝对上限
SAFE_TRANSIT_PHASES = (
    "observe:workpiece_coarse_efficient",
    "observe:tables",
    "observe:slot_coarse",
    "observe:slot_fine",
    "baseline",             # 已验证的中间换位点（z≈0.73，高位自由空间）
    "pre_grasp",            # 只覆盖"关节空间到预抓取位上方 200 mm"那一段；
                            # 最后 200 mm 由 move_tcp_linear.py 执行，不带 --phase，
                            # 因此仍然是 0.10
    "goto_park_pose",       # 一键回停车位（goto_park_pose.py）
    "goto_workpiece_coarse",  # 一键回工件粗观测位（goto_workpiece_coarse.py）
)


# 绝对安全关节区间（弧度）。超出即认为"不该在这里"，执行中立即取消。
SAFE_JOINT_LIMITS = (
    ("shoulder_pan_joint", math.radians(-100.0), math.radians(100.0)),
    # 2026-11-24 与 plan_pose_ik 的 ABSOLUTE_LIMITS 对齐（所有者当天批准）：
    # 肩部 -130、肘部 -60。旧值（-110/-100）比规划器严，已验证的抓取位姿
    # （lift=-124.5°、elbow=-69~-102°）会在执行中被看门狗误判 JOINT_OUT_OF_RANGE。
    ("shoulder_lift_joint", math.radians(-130.0), math.radians(10.0)),
    ("elbow_joint", math.radians(-160.0), math.radians(-60.0)),
    ("wrist_1_joint", math.radians(-150.0), math.radians(150.0)),
    ("wrist_2_joint", math.radians(-150.0), math.radians(150.0)),
    ("wrist_3_joint", math.radians(-150.0), math.radians(150.0)),
)
MIN_SEGMENT_SEC = 0.25           # 每段最短时间，避免时间戳过密导致控制器插值抖动


def _arm_indices(names: list[str]) -> list[int]:
    return [j for j, n in enumerate(names) if "finger" not in n]


def govern_timestamps(data: dict, time_scale: float,
                      speed_cap: float = MAX_JOINT_SPEED_RAD_S,
                      min_segment_sec: float = MIN_SEGMENT_SEC) -> tuple[list[float], dict]:
    """按 speed_cap 重新配时：只会更慢，绝不会比计划更快。

    返回 (times, info)。times 已经乘过 time_scale（>=1，只会更慢）。
    speed_cap 由 resolve_speed_cap() 决定：默认 MAX_JOINT_SPEED_RAD_S，
    只有白名单阶段且调用方显式要求时才升到 HARD_MAX_JOINT_SPEED_RAD_S。
    """
    names = data["joint_names"]
    points = data["points"]
    idx = _arm_indices(names)
    times = [0.0]
    for i in range(1, len(points)):
        plan_dt = float(points[i]["t"]) - float(points[i - 1]["t"])
        dq = max(abs(points[i]["q"][j] - points[i - 1]["q"][j]) for j in idx)
        needed = dq / speed_cap
        times.append(times[-1] + max(plan_dt, needed, min_segment_sec))
    # 2026-11-20：治理器已经是速度的唯一权威（MAX_JOINT_SPEED_RAD_S 就是硬上限）。
    # 以前这里再乘 time_scale，调用方习惯性传 3.0，实际速度变成上限的 1/3
    # （J4 转 94° 要 98 s，所有者反馈"太慢"）。现在 time_scale 不再叠加。
    # 需要额外放慢时，让计划自己的时间戳更慢（缩放因子），而不是乘一个倍数。
    if time_scale > 1.0 + 1e-9:
        print(f"注意：--time-scale {time_scale} 自 2026-11-20 起不再叠加到速度上；"
              f"慢速请改计划时间戳或 --max-joint-speed（否则按治理上限执行）")
    info = {
        "plan_duration": float(points[-1]["t"]),
        "exec_duration": times[-1],
        "max_speed": max(
            (max(abs(points[i]["q"][j] - points[i - 1]["q"][j]) for j in idx)
             / (times[i] - times[i - 1]))
            for i in range(1, len(times))
        ) if len(times) > 1 else 0.0,
    }
    return times, info


def safe_transit_min_segment(phase: str) -> float:
    """安全换位段的时间戳地板。

    MIN_SEGMENT_SEC（0.25 s）是为了避免"时间戳过密导致控制器抖动"而设的**每段**
    地板。但 MoveIt 给出的轨迹点很密（160 点 / 16 s ≈ 0.1 s 一点），逐点套 0.25 s
    会把 16 s 的轨迹拉成 39 s——2026-11-24 实测"计划 15.6 s、墙钟 69 s"就是这么来的。
    换位段不接触任何东西，抖动风险低，地板降到 0.05 s；接触/装配段保持 0.25 s。
    """
    if phase.strip() in SAFE_TRANSIT_PHASES:
        return 0.05
    return MIN_SEGMENT_SEC


def safe_joint_limit_map() -> dict[str, tuple[float, float]]:
    """SAFE_JOINT_LIMITS 的 {关节名: (下限, 上限)} 视图。

    2026-11-25 真机故障：旧代码在这里写 ``dict(SAFE_JOINT_LIMITS)``，而该常量是
    (name, low, high) 三元组的元组 —— dict() 直接抛 ValueError，于是"下发前检查"
    变成"下发前崩溃"，整段轨迹发不出去（机械臂一步没动，退出码 1）。
    同一个错误在 test 里也出现过一次、当时只改了 test 没改源码，这次一并修正。
    """
    return {name: (low, high) for name, low, high in SAFE_JOINT_LIMITS}


def trajectory_within_safe_limits(data: dict) -> tuple[bool, str]:
    """下发前逐点检查轨迹是否在绝对安全关节区间内。

    返回 (ok, reason)。ok=False 时调用方拒绝下发（exit 19）——这是"下发前"的最后一道
    静态检查，与执行中的在线看门狗互为补充（短轨迹的看门狗可能一次都不跑）。
    """
    limits = safe_joint_limit_map()
    indices = _arm_indices(data["joint_names"])
    for index, point in enumerate(data["points"]):
        for j in indices:
            name = data["joint_names"][j]
            low, high = limits.get(name, (-math.inf, math.inf))
            value = float(point["q"][j])
            if not (low - 1e-9 <= value <= high + 1e-9):
                return False, (f"计划第 {index} 点 {name}="
                               f"{math.degrees(value):.1f}° 超出安全区间 "
                               f"[{math.degrees(low):.0f}, {math.degrees(high):.0f}]")
    return True, ""


def resolve_speed_cap(requested: float, phase: str) -> tuple[float, str]:
    """决定这次下发用多大的关节速度上限。

    规则（所有者 2026-11-24）：默认 0.10（5.7°/s）；只有 SAFE_TRANSIT_PHASES
    白名单里的阶段可以提到最多 HARD_MAX_JOINT_SPEED_RAD_S（0.20）。任何超出
    硬上限的请求都不是"降级放行"，而是**夹到硬上限并打印**——不再允许把速度
    上限当成调用方随手可调的参数（2026-11-18 的速度事故就是这么来的）。
    """
    try:
        asked = float(requested)
    except (TypeError, ValueError):
        return MAX_JOINT_SPEED_RAD_S, f"非法请求 {requested!r}"
    if asked <= 0:
        return MAX_JOINT_SPEED_RAD_S, "默认"
    if asked < MAX_JOINT_SPEED_RAD_S - 1e-9:
        # 调用方显式请求"比默认更慢"：照办（2026-11-24 audit 发现旧代码会静默抬到 0.10，
        # 任何"更慢"的请求都被反向加速）。plan 时间戳本身更慢时，治理器取 max 不会冲突。
        return asked, f"调用方请求更慢 {asked:.3f}"
    if phase.strip() not in SAFE_TRANSIT_PHASES:
        return MAX_JOINT_SPEED_RAD_S, f"阶段 {phase or '(空)'} 不在可提速白名单"
    cap = min(asked, HARD_MAX_JOINT_SPEED_RAD_S)
    note = "安全换位段" if cap < HARD_MAX_JOINT_SPEED_RAD_S - 1e-9 else "安全换位段（硬上限）"
    return cap, note


def split_plan(data: dict, max_delta_rad: float) -> list[dict]:
    """把超出行程上限的计划切成若干段，每段任一关节的累计行程 ≤ max_delta_rad。

    所有者 2026-11-19：超过 25° 硬闸、但终点仍在绝对角度区间内时，程序应自己分段执行，
    不该让人手工拆。做法：先把关节空间折线加密（每步 ≤ 半个上限），再贪心切段；
    相邻段共享边界点，因此拼起来与原轨迹完全重合（关节线性插值等分，几何一致）。
    """
    names = data["joint_names"]
    points = data["points"]
    idx = _arm_indices(names)
    if len(points) < 2:
        return [data]
    half = max_delta_rad * 0.5
    dense = [{"q": list(points[0]["q"]), "t": float(points[0]["t"])}]
    for prev, cur in zip(points, points[1:]):
        span = max(abs(cur["q"][j] - prev["q"][j]) for j in idx)
        steps = max(1, int(math.ceil(span / half)))
        for step in range(1, steps + 1):
            alpha = step / steps
            q = [prev["q"][i] + (cur["q"][i] - prev["q"][i]) * alpha
                 for i in range(len(prev["q"]))]
            dense.append({"q": q,
                          "t": float(prev["t"]) + (float(cur["t"]) - float(prev["t"])) * alpha})
    chunks = []
    start = 0
    while start < len(dense) - 1:
        base = dense[start]["q"]
        end = start + 1
        while end + 1 < len(dense):
            if max(abs(dense[end + 1]["q"][j] - base[j]) for j in idx) > max_delta_rad:
                break
            end += 1
        chunk_points = [{"q": list(dense[k]["q"]),
                         "t": float(dense[k]["t"]) - float(dense[start]["t"])}
                        for k in range(start, end + 1)]
        chunks.append({"joint_names": names, "points": chunk_points})
        start = end
    return chunks


def trajectory_contact_free(node, joint_names: list[str], samples: list[list[float]],
                            steps: int = 8):
    """在发送前逐点校验轨迹是否有接触（2026-11-20 所有者：这是绝对底线）。

    规划器已经抽检过，但自动分段会把一条轨迹切成多段，**每段都必须自己再过一遍**，
    不能因为"上游检查过"就跳过。返回 (ok, reason)：
      · ok=True  → 可以发；
      · ok=False → 有接触，拒绝发送；
      · ok=None  → /check_state_validity 不可用，调用方按"拒绝"处理（不做无验证的移动）。
    """
    from moveit_msgs.msg import RobotState
    from moveit_msgs.srv import GetStateValidity

    client = node.create_client(GetStateValidity, "/check_state_validity")
    if not client.wait_for_service(timeout_sec=5.0):
        return None, "/check_state_validity 不可用"
    count = max(2, int(steps))
    indices = sorted({int(round(i * (len(samples) - 1) / (count - 1)))
                      for i in range(count)})
    for index in indices:
        request = GetStateValidity.Request()
        request.group_name = "cs625_arm"
        state = RobotState()
        state.joint_state.name = list(joint_names)
        state.joint_state.position = [float(v) for v in samples[index]]
        request.robot_state = state
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=8.0)
        result = future.result()
        if result is None:
            return None, f"碰撞服务无响应（sample {index}）"
        if not result.valid and result.contacts:
            pairs = [(c.contact_body_1, c.contact_body_2) for c in result.contacts[:3]]
            return False, f"point {index} 接触 {pairs}"
        if not result.valid and not result.contacts:
            return False, f"point {index} 状态无效（无接触信息）"
    return True, ""


def plan_jobs(files, allow_large_move: bool = False,
              no_auto_split: bool = False,
              single_shot_phase: str = "",
              single_segment_within_safe: bool = False) -> tuple[list[tuple[str, dict]], int | None]:
    """读入计划文件并按需要自动分段。

    返回 (jobs, refused)。refused 非 None 时需要直接返回该退出码（用于 --no-auto-split）。
    glob.glob 返回的是**字符串**，这里统一转成 Path（2026-11-19 真机踩过：字符串没有
    read_text，导致执行器在第一步就崩、机械臂一步都没动）。
    """
    jobs: list[tuple[str, dict]] = []
    for raw_path in files:
        path = pathlib.Path(raw_path)
        data = json.loads(path.read_text(encoding="utf-8"))
        deltas = joint_deltas(data)
        worst_joint, worst_delta = max(deltas.items(), key=lambda kv: kv[1])
        # 2026-11-24 所有者批准：安全换位段（白名单）不自动分段。
        # 分段的真实作用是"每 ≤25° 停一次核对终点、把失控损失截在 25° 内"，
        # 它不改变路径（分段前后几何完全一致），因此"会不会撞"与分段无关。
        # 白名单里的段都是已验证的自由空间换位，且在线关节区间看门狗（JOINT_OUT_OF_RANGE）
        # 与整段到位复核仍然生效；接触/装配段保持 25° 自动分段不变。
        single_shot = single_shot_phase.strip() in SAFE_TRANSIT_PHASES
        # 2026-11-26 所有者批准：**终点全程落在绝对安全关节区间内时，不自动分段**。
        # 理由：分段不改变几何路径，只提供"每 ≤25° 停一次核对"的责任边界；
        # 当整条轨迹的每个采样点都已在 SAFE_JOINT_LIMITS 内、且碰撞校验通过时，
        # 在线关节区间看门狗（JOINT_OUT_OF_RANGE）+ 到位复核 + 逐点先验检查仍然生效，
        # 因此允许一次性执行。**区间外的目标仍然自动分段/拒绝**（该保护不变）。
        if (single_segment_within_safe and worst_delta > MAX_JOINT_DELTA_RAD + 1e-9):
            inside, why = trajectory_within_safe_limits(data)
            if inside:
                table = "  ".join(f"{n.replace('_joint', '')}={math.degrees(v):.1f}°"
                                  for n, v in deltas.items())
                print(f"[{path.name}] 终点全在绝对安全区间内：单关节最大行程 "
                      f"{math.degrees(worst_delta):.1f}° 一次性执行（不分段）。{table}")
                print("           仍生效：逐点先验区间检查 + 碰撞校验 + 在线看门狗 + 到位复核")
                jobs.append((str(path), data))
                continue
            print(f"[{path.name}] --single-segment-within-safe 被拒绝：{why} → 继续自动分段")
        if worst_delta <= MAX_JOINT_DELTA_RAD + 1e-9 or allow_large_move or single_shot:
            if single_shot and worst_delta > MAX_JOINT_DELTA_RAD + 1e-9:
                print(f"[{path.name}] 安全换位段 {single_shot_phase}：单关节行程 "
                      f"{math.degrees(worst_delta):.1f}° 一次性执行（不自动分段；"
                      f"在线关节区间看门狗 + 到位复核仍生效）")
            jobs.append((str(path), data))
            continue
        if no_auto_split:
            table = "  ".join(f"{n.replace('_joint','')}={math.degrees(v):.1f}°"
                              for n, v in deltas.items())
            print(
                f"REFUSED_LARGE_MOVE: {path.name} asks {worst_joint} to travel "
                f"{math.degrees(worst_delta):.1f}° (> {math.degrees(MAX_JOINT_DELTA_RAD):.0f}° "
                f"hard limit) and --no-auto-split was given.  Joint deltas: {table}.  "
                "Nothing was sent."
            )
            return jobs, 13
        chunks = split_plan(data, MAX_JOINT_DELTA_RAD)
        print(f"[{path.name}] 单关节行程 {math.degrees(worst_delta):.1f}° > "
              f"{math.degrees(MAX_JOINT_DELTA_RAD):.0f}°，**自动分 {len(chunks)} 段**执行"
              f"（每段 ≤{math.degrees(MAX_JOINT_DELTA_RAD):.0f}°）")
        for part, chunk in enumerate(chunks, 1):
            jobs.append((f"{path.name}#{part}/{len(chunks)}", chunk))
    return jobs, None


def joint_deltas(data: dict) -> dict[str, float]:
    """每个关节在整个计划里的**累计**行程（弧度）。

    2026-11-24 audit：旧实现用 |末点−起点| 净值做 25° 闸，"摆出去再摆回来"
    的大幅振荡计划可以绕过自动分段。累计行程才是"这个关节实际走了多远"。
    """
    names = data["joint_names"]
    points = data["points"]
    travel = {names[j]: 0.0 for j in _arm_indices(names)}
    for i in range(1, len(points)):
        for j in _arm_indices(names):
            travel[names[j]] += abs(points[i]["q"][j] - points[i - 1]["q"][j])
    return travel


def build_parser() -> argparse.ArgumentParser:
    """Build the CLI parser without touching ROS.

    Kept separate from main() so the argument contract can be tested without a
    ROS runtime; main() still owns every motion-related code path.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("pattern")
    parser.add_argument("--time-scale", type=float, default=1.0,
                        help="已废弃：速度由治理器（MAX_JOINT_SPEED_RAD_S）统一决定，"
                             "这个倍数不再叠加到配时上。<1 仍会被拒绝（历史教训："
                             "2026-10-30 把 0.5 当成'半速'，实际是双倍速并撞了工作台）。")
    parser.add_argument("--tail-fraction", type=float, default=0.0,
                        help="last fraction of the trajectory to run slower (0 disables)")
    parser.add_argument("--tail-scale", type=float, default=2.0,
                        help="extra time factor applied to the tail section")
    parser.add_argument("--verify-final", type=float, default=0.0,
                        help="rad; after success compare the measured joints to the last point")
    parser.add_argument("--dwell", type=float, default=1.0)
    parser.add_argument("--goal-tolerance", type=float, default=0.02)
    parser.add_argument("--max-start-offset", type=float, default=0.03,
                        help="refuse to execute if the trajectory start is far from the measured state")
    parser.add_argument("--max-wait", type=float, default=0.0,
                        help="optional hard cap (s); 0 = disabled, wait for the controller verdict")
    parser.add_argument("--fault-poll", type=float, default=0.2,
                        help="safety/robot mode polling period for early abort (s)")
    parser.add_argument("--wait-scale", type=float, default=1.0,
                        help="Multiply the wait budget. Gazebo on a CPU renderer "
                             "advances slower than wall time, so a simulation run "
                             "needs a larger budget or it gives up on a trajectory "
                             "that is still executing.")
    parser.add_argument("--no-motion-window", type=float, default=10.0,
                        help="seconds to wait for the first real joint motion before "
                             "reporting NO_MOTION; only applies to a trajectory that "
                             "actually asks the arm to move")
    parser.add_argument("--auto-recover", action="store_true", default=True,
                        help="when a chunk shows NO_MOTION while safety/robot modes "
                             "are healthy, re-send the external_control script once "
                             "and retry (2026-11-03: the reverse-control channel drops "
                             "silently and the arm then ignores accepted goals)")
    parser.add_argument("--no-auto-recover", dest="auto_recover", action="store_false")
    parser.add_argument("--launch-hold", type=float, default=1.2,
                        help="每段开头原地保持的秒数，给伺服启动留时间（默认 1.2，0=关闭）")
    parser.add_argument("--skip-command-sync", action="store_true",
                        help="跳过下发前的控制器记忆校正（不推荐）")
    parser.add_argument("--allow-large-move", action="store_true",
                        help="不自动分段，直接一次性下发（默认会自动分段）。加这个参数前"
                             "必须已打印 6 个关节角并得到所有者明确同意（AGENTS 安全红线）")
    parser.add_argument("--skip-contact-check", action="store_true",
                        help="跳过发送前的逐点碰撞校验（**不推荐**，默认每段都校验）")
    parser.add_argument("--phase", default="",
                        help="阶段名；只有 SAFE_TRANSIT_PHASES 白名单里的阶段才允许提速")
    parser.add_argument("--max-joint-speed", type=float, default=MAX_JOINT_SPEED_RAD_S,
                        help=f"请求的关节速度上限（rad/s）；默认 {MAX_JOINT_SPEED_RAD_S}，"
                             f"硬上限 {HARD_MAX_JOINT_SPEED_RAD_S}")
    parser.add_argument("--single-segment-within-safe", action="store_true",
                        help="终点全程落在绝对安全关节区间内且碰撞校验通过时，不自动分段"
                             "直接一次性执行（所有者 2026-11-26 批准）。区间外的目标仍然"
                             "自动分段；在线看门狗、逐点区间检查、到位复核全部保留。")
    parser.add_argument("--no-auto-split", action="store_true",
                        help="超过 25° 行程时拒绝而不是自动分段（默认分段，见 split_plan）")
    parser.add_argument("--action", default=DEFAULT_ACTION,
                        help="FollowJointTrajectory action name; the real-arm "
                             "controller by default, overridden for simulation "
                             f"(default: {DEFAULT_ACTION})")
    return parser


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    args = build_parser().parse_args(argv)
    if args.time_scale < 1.0:
        raise SystemExit(
            f"REFUSED: --time-scale {args.time_scale} would play the trajectory "
            "FASTER than planned (<1 multiplies timestamps down). Use >= 1.0; "
            "larger is slower."
        )
    return args


def resend_external_script(timeout: float = 30.0) -> bool:
    """Re-send the robot's external_control script (restores the reverse ports).

    Measured 2026-11-03 (twice): after a protective stop the three reverse-control
    connections are gone while safety_mode/robot_mode still look healthy; the
    controller accepts goals and the arm never moves.  The service that fixes it
    is /io_and_status_controller/resend_external_script.
    """
    command = ["ros2", "service", "call",
               "/io_and_status_controller/resend_external_script",
               "std_srvs/srv/Trigger", "{}"]
    try:
        done = subprocess.run(command, capture_output=True, text=True, timeout=timeout)
    except (OSError, subprocess.TimeoutExpired):
        return False
    text = (done.stdout or "") + (done.stderr or "")
    return "success=True" in text


def reverse_ports_up() -> bool:
    try:
        out = subprocess.run(["ss", "-tn"], capture_output=True, text=True,
                             timeout=5).stdout
    except (OSError, subprocess.TimeoutExpired):
        return False
    return any(f":{port}" in line and "ESTAB" in line
               for line in out.splitlines()
               for port in ("50001", "50003", "50004"))


def main() -> int:
    sys.stdout.reconfigure(line_buffering=True)
    args = parse_args()
    files = sorted(glob.glob(args.pattern))
    if not files:
        print("NO_CHUNKS")
        return 2

    rclpy.init()
    node = Node("execute_assembly_chunks")
    client = ActionClient(node, FollowJointTrajectory, args.action)
    if not client.wait_for_server(timeout_sec=10.0):
        print("NO_ACTION_SERVER")
        return 3

    # Defence in depth: refuse a trajectory whose first point is far from the
    # measured joint state.  Executing such a trajectory makes the arm snap
    # back to the stale start pose (this caused a visible oscillation).
    from sensor_msgs.msg import JointState

    latest: dict[str, float] = {}
    efforts: list[float] = []

    def _on_joints(msg: JointState) -> None:
        latest.update(dict(zip(msg.name, msg.position)))
        effort = dict(zip(msg.name, msg.effort))
        order = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                 "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        if all(k in effort for k in order):
            values = [effort[k] for k in order]
            if all(math.isfinite(v) and abs(v) < 1.0e4 for v in values):
                efforts.append(sum(abs(v) for v in values))
        del efforts[:-64]  # keep only the newest window

    def robot_status_snapshot() -> dict:
        """抓一次 safety_mode / robot_mode，用于给 EFFORT_DEAD 定性。

        只报"力矩死流"无法区分：保护停止（safety_mode 变了）、模式切换
        （robot_mode 复归）还是单纯话题掉线——三者处置完全不同。
        """
        snapshot: dict = {}
        try:
            from eli_common_interface.msg import RobotMode, SafetyMode
        except Exception as error:  # noqa: BLE001
            return {"note": f"status msg unavailable: {error}"}

        captured: dict = {}

        def _on_safety(msg) -> None:
            captured["safety"] = int(getattr(msg, "mode", -1))

        def _on_mode(msg) -> None:
            captured["mode"] = int(getattr(msg, "mode", -1))

        # 这两个话题是 transient_local（latched）：用默认 volatile 订阅收不到
        # 历史的那一条，只有状态变化时才可能收到。必须显式声明 durability。
        from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                               ReliabilityPolicy)
        latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        subs = [
            node.create_subscription(SafetyMode, "/io_and_status_controller/safety_mode",
                                     _on_safety, latched),
            node.create_subscription(RobotMode, "/io_and_status_controller/robot_mode",
                                     _on_mode, latched),
        ]
        # 这两个话题是"状态变化时发布 + 低频重发"，1.5 s 有时抓不到（实测 3 s 更稳）
        deadline = time.time() + 3.0
        while time.time() < deadline and len(captured) < 2:
            rclpy.spin_once(node, timeout_sec=0.1)
        for sub in subs:
            node.destroy_subscription(sub)
        return captured or {"note": "no safety/mode samples within 3 s"}

    def effort_alive() -> tuple[str, float]:
        """The torque interface must show a plausible load before any motion.

        On 2026-11-03 a controller switch left the effort interface reading ~0 N
        while the arm was carrying a 6.8 kg part; every contact detector would
        then silently see nothing, and the run collided with the fixture.  A dead
        stream reads < 1.0; a loaded arm reads >= 8 on joint 2 alone.

        2026-11-18: the old version waited a fixed 0.8 s and reported
        ``EFFORT_DEAD`` whenever fewer than 10 samples had arrived.  With many
        participants (driver + several move_group + recorder + camera + panel) DDS
        discovery can take longer than that, so the gate fired with
        ``peak 0.000`` while the stream was actually alive (measured separately:
        Σ|effort| = 8.44 at the same moment).  The two situations need different
        actions, so they are now distinguished:

            "dead"    — samples arrived, but the load stays below 1.0  → real fault
            "unknown" — no samples at all (DDS discovery / no /joint_states) → retry
            "alive"   — plausible load observed

        It now waits for the first sample (up to 5 s) and only then requires a
        real window, instead of assuming 0.8 s is always enough.
        """
        first_deadline = time.time() + 5.0
        while not efforts and time.time() < first_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if not efforts:
            return "unknown", 0.0
        window_deadline = time.time() + 1.5
        while len(efforts) < 10 and time.time() < window_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if len(efforts) < 10:
            return "unknown", 0.0
        recent = efforts[-16:]
        peak = float(max(recent))
        return ("alive" if peak >= 1.0 else "dead"), peak

    node.create_subscription(JointState, "/joint_states", _on_joints, 10)
    probe_deadline = time.time() + 3.0
    while not latest and time.time() < probe_deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    def first_point_far(data) -> float:
        if not latest:
            return 0.0
        worst = 0.0
        for i, name in enumerate(data["joint_names"]):
            if name in latest:
                worst = max(worst, abs(data["points"][0]["q"][i] - latest[name]))
        return worst

    # Robot state, so that "the goal was accepted and then nothing happened" can
    # name its cause.  A protective stop, a powered-off robot and a missing
    # external-control script all look identical to an action client: the goal is
    # accepted (REJECTED would mean otherwise) and the result never arrives.
    state: dict[str, int] = {}

    def _status(name: str):
        def _callback(message) -> None:
            state[name] = int(message.mode)
        return _callback

    try:
        from eli_common_interface.msg import RobotMode, SafetyMode
        from rclpy.qos import DurabilityPolicy, QoSProfile

        # These two topics are latched (TRANSIENT_LOCAL) and published on change;
        # a volatile subscription never receives the current value, which is why
        # this probe used to print safety_mode=? and could not notice an E-stop.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(
            SafetyMode, "/io_and_status_controller/safety_mode", _status("safety"),
            latched)
        node.create_subscription(
            RobotMode, "/io_and_status_controller/robot_mode", _status("robot"),
            latched)
    except Exception as exc:  # driver messages are an underlay dependency
        print(f"robot-state probe unavailable ({exc}); failures report raw status only")

    # Protective-stop marker (2026-11-03 collision): a protective stop drops the
    # reverse-control channel, so the NEXT move would look like NO_MOTION and the
    # auto-recovery would silently resend the script and retry -- straight back
    # into whatever just collided.  The marker makes the executor refuse to
    # auto-recover within 60 s of a protective stop: the operator must look.
    estop_marker = pathlib.Path("/tmp/teaching_estop_marker.json")
    recent_estop = False
    try:
        if estop_marker.exists():
            marker = json.loads(estop_marker.read_text())
            if time.time() - float(marker.get("t", 0.0)) < 60.0:
                recent_estop = True
                print(f"recent protective stop ({marker.get('reason', '?')}); "
                      "auto-recovery is disabled -- operator review required")
    except (OSError, ValueError):
        pass

    def robot_state_text() -> str:
        safety = state.get("safety", "?")
        robot = state.get("robot", "?")
        return (f"safety_mode={safety}({SAFETY_MODE_NAMES.get(safety, '?')}) "
                f"robot_mode={robot}({ROBOT_MODE_NAMES.get(robot, '?')})")

    # --- 2026-11-19 硬闸 1：单关节行程（超限自动分段，不再让人手工拆）---
    jobs, refused = plan_jobs(files, allow_large_move=args.allow_large_move,
                              single_segment_within_safe=args.single_segment_within_safe,
                              no_auto_split=args.no_auto_split,
                              single_shot_phase=args.phase)
    if refused is not None:
        return refused

    for index, (path, data) in enumerate(jobs, 1):
        # --- 2026-11-19 硬闸 2：速度（不信任计划时间戳，按上限重新配时）---
        speed_cap, cap_note = resolve_speed_cap(args.max_joint_speed, args.phase)
        governed, gov = govern_timestamps(
            data, args.time_scale, speed_cap,
            min_segment_sec=safe_transit_min_segment(args.phase))
        print(f"[{index}/{len(jobs)}] speed governor: plan {gov['plan_duration']:.2f}s -> "
              f"exec {gov['exec_duration']:.2f}s, max joint speed "
              f"{gov['max_speed']:.3f} rad/s ({math.degrees(gov['max_speed']):.1f}°/s, "
              f"cap {speed_cap:.3f} [{cap_note}])")
        if gov["max_speed"] > speed_cap + 1e-6:
            print(f"REFUSED_SPEED: governed trajectory still exceeds the cap; nothing sent.")
            return 14

        # --- 2026-11-24 硬闸 2.5：下发前全轨迹点安全区间检查 ---
        # 执行中的看门狗（JOINT_OUT_OF_RANGE）只在等待循环里轮询，短轨迹可能一次都不跑；
        # 这里在发送前就把轨迹的每个点与 SAFE_JOINT_LIMITS 比对，越界直接拒绝（exit 19）。
        in_range, reason = trajectory_within_safe_limits(data)
        if not in_range:
            print(f"REFUSED_OUT_OF_RANGE_PLAN: {reason}；不下发")
            return 19

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = data["joint_names"]
        # 2026-11-20：每段开头加一个"保持原地"的死区（默认 1.2 s）。
        # 机械臂从静止起步有约 1.0–1.5 s 的伺服启动延迟；速度提到 5.7°/s 后，
        # 25° 的段只有 4.4 s，延迟占比过大 → 控制器在 t≈1.5 s 检查时发现
        # "目标已在动、机械臂还没动"，判 PATH_TOLERANCE_VIOLATED 直接中止
        # （2026-11-20 21:12 任务⑪ 就是这样失败的）。
        # 开头保持不动后，这段时间期望位置与实际位置都是静止，检查必然通过。
        lead = max(0.0, float(args.launch_hold))
        if lead > 0.0:
            hold = JointTrajectoryPoint()
            hold.positions = list(data["points"][0]["q"])
            hold.time_from_start = Duration(sec=0, nanosec=0)
            goal.trajectory.points.append(hold)
        for point, stamp in zip(data["points"], governed):
            message = JointTrajectoryPoint()
            message.positions = point["q"]
            shifted = stamp + lead
            message.time_from_start = Duration(
                sec=int(shifted), nanosec=int((shifted - int(shifted)) * 1e9)
            )
            goal.trajectory.points.append(message)
        scaled_total = governed[-1]
        # Slow the final approach instead of splitting the trajectory into
        # chunks: the tail keeps the same geometry (smooth) but has more time, so
        # contact and settling happen at low speed while the path stays single.
        if args.tail_fraction > 0.0 and args.tail_scale > 1.0:
            last_time = (goal.trajectory.points[-1].time_from_start.sec
                         + goal.trajectory.points[-1].time_from_start.nanosec * 1e-9)
            tail_start = last_time * (1.0 - args.tail_fraction)
            for message in goal.trajectory.points:
                now = (message.time_from_start.sec
                       + message.time_from_start.nanosec * 1e-9)
                if now > tail_start:
                    stretched = tail_start + (now - tail_start) * args.tail_scale
                    message.time_from_start = Duration(
                        sec=int(stretched), nanosec=int((stretched - int(stretched)) * 1e9))
        # Tolerances (2026-11-05): a goal that carried only a POSITION tolerance left the
        # velocity tolerance at its zero default, so a trajectory that finished with a
        # residual tracking velocity (measured: 0.001 rad on elbow) was reported as
        # GOAL_TOLERANCE_VIOLATED even though the arm had completed the move.  Both
        # tolerances are now explicit, and a generous path tolerance stops the controller
        # from aborting on transient deviations along the way.
        for name in data["joint_names"]:
            tolerance = JointTolerance()
            tolerance.name = name
            tolerance.position = args.goal_tolerance
            tolerance.velocity = max(0.05, args.goal_tolerance * 5.0)
            goal.goal_tolerance.append(tolerance)

            path_tolerance = JointTolerance()
            path_tolerance.name = name
            path_tolerance.position = max(0.15, args.goal_tolerance * 10.0)
            path_tolerance.velocity = 1.0
            goal.path_tolerance.append(path_tolerance)

        # +5 s, not +25: the controller must say "I did not make it" quickly.
        goal.goal_time_tolerance = Duration(sec=int(scaled_total + 5))

        # 2026-11-20 硬闸 0：下发前校正"控制器记忆位置"。
        # 人手拖动机械臂后，控制器记忆仍停在旧目标；此时重连/重发脚本会让它
        # 全速冲向旧位置（当天三次事故）。这里在**静止状态、下发之前**把记忆
        # 对齐到实测位置，所以不需要所有者手动归位。
        sync = pathlib.Path(__file__).resolve().parent / "sync_command_position.py"
        if sync.exists() and not args.skip_command_sync:
            check = subprocess.run([sys.executable, str(sync)],
                                   capture_output=True, text=True, timeout=120)
            if check.returncode == 1:      # 需要校正
                print("控制器记忆与实际位置不一致，先校正（旧记忆会被作废）：")
                for line in (check.stdout or "").strip().splitlines()[-3:]:
                    print("  " + line)
                fix = subprocess.run([sys.executable, str(sync), "--apply", "--force"],
                                     capture_output=True, text=True, timeout=180)
                for line in (fix.stdout or "").strip().splitlines()[-4:]:
                    print("  " + line)
                if fix.returncode != 0:
                    print("命令位置校正失败；拒绝下发轨迹（不做无同步的移动）")
                    return 18
        offset = first_point_far(data)
        if offset > args.max_start_offset:
            print(
                f"FIRST_POINT_FAR offset={offset:.3f} rad > {args.max_start_offset}; "
                "refusing to execute a stale trajectory"
            )
            return 7
        effort_state, peak = effort_alive()
        if effort_state != "alive":
            # 记录失败瞬间的机器人状态：只报"力矩死流"无法区分是保护停止、
            # 模式切换还是话题掉线，而这三者的处置完全不同。
            status = robot_status_snapshot()
            if effort_state == "unknown":
                print(
                    "EFFORT_UNKNOWN: no /joint_states samples arrived while probing "
                    "(DDS discovery or a dead publisher), so the load cannot be "
                    f"judged.  Nothing was executed.  robot_status={status}"
                )
                return 12
            print(
                f"EFFORT_DEAD: the torque interface shows no plausible load "
                f"(peak {peak:.3f}); every contact detector would be blind, so "
                "this trajectory is refused.  Restart the driver stack and "
                f"re-check before moving again.  robot_status={status}"
            )
            return 11
        # --- 2026-11-20 硬闸 3：发送前逐点碰撞校验（每段独立校验，不信任上游）---
        if not args.skip_contact_check:
            samples = [[float(v) for v in point.positions] for point in goal.trajectory.points]
            ok, reason = trajectory_contact_free(node, list(goal.trajectory.joint_names),
                                                 samples)
            if ok is None:
                print(f"CONTACT_CHECK_UNAVAILABLE: {reason}；拒绝发送（不做无验证的移动）")
                return 16
            if not ok:
                print(f"REFUSED_CONTACT: {reason}；这一段不发送")
                return 15
            print(f"[{index}/{len(jobs)}] 碰撞校验：{len(samples)} 点全部无接触 ✅")
        print(f"[{index}/{len(jobs)}] start {path} "
              f"effort_peak={peak:.2f}")
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        handle = future.result()
        if handle is None or not handle.accepted:
            print("REJECTED")
            return 4
        result_future = handle.get_result_async()

        # Wait for the controller's own verdict instead of guessing.  The
        # controller aborts a trajectory at (duration + goal_time_tolerance),
        # which with the +25 s set above means 2*duration + 25 s; the previous
        # window was duration + 20 s, so the client gave up while the server was
        # still running the goal and then queued the next chunk on top of it.
        duration = scaled_total
        # The controller aborts at duration + 5 s now; give the result 20 s to
        # arrive instead of 2*duration + 40 (a failed 60 s insert used to keep the
        # caller blind for almost two minutes while the Qt panel already showed
        # the failure).
        # The FIRST goal-timeout (controller deadline) is duration + 5 s, so wait
        # for the verdict, not a wall clock: long inserts legitimately run for
        # minutes.  --max-wait remains as an optional hard stop for a call that
        # has already timed out once, not for normal execution.
        wait_budget = (duration + 20.0) * args.wait_scale
        if args.max_wait > 0 and args.max_wait < wait_budget:
            wait_budget = args.max_wait
        expected_span = max(
            (abs(a - b) for a, b in zip(data["points"][-1]["q"], data["points"][0]["q"])),
            default=0.0,
        )
        start_positions = dict(latest)
        started = time.time()
        # A hold-only chunk is not expected to move; anything else must start
        # moving, otherwise the robot is not executing what it accepted.
        motion_seen = expected_span <= 0.02
        while not result_future.done() and time.time() - started < wait_budget:
            rclpy.spin_once(node, timeout_sec=args.fault_poll)
            # An E-stop / protective stop ends the trajectory immediately; report
            # it as such instead of waiting for the controller's timeout.
            safety_now = state.get("safety")
            robot_now = state.get("robot")
            if (safety_now not in (None, 1)) or (robot_now not in (None, 7)):
                handle.cancel_goal_async()
                print(f"ESTOP_OR_PROTECTIVE_STOP: {robot_state_text()}; "
                      "the trajectory was cancelled")
                try:
                    estop_marker.write_text(json.dumps({
                        "t": time.time(),
                        "reason": f"safety={safety_now} robot={robot_now}",
                        "file": str(path),
                    }))
                except OSError:
                    pass
                return 9
            if not motion_seen and start_positions:
                motion_seen = max(
                    (abs(latest.get(name, value) - value)
                     for name, value in start_positions.items()),
                    default=0.0,
                ) > 0.002
            if not motion_seen and time.time() - started > args.no_motion_window:
                handle.cancel_goal_async()
                print(
                    f"NO_MOTION: this chunk asks for {expected_span:.3f} rad of joint "
                    f"motion but the arm has not moved in {args.no_motion_window:.0f} s; "
                    f"{robot_state_text()}. The controller accepted the goal, so the "
                    "robot is not executing it: release the protective stop and power "
                    "the robot on at the teach pendant, then run "
                    "scripts/recover_control.sh."
                )
                # Auto-recovery (2026-11-03, owner request): health looks fine but
                # the reverse-control channel silently dropped.  Re-send the
                # external_control script and let the caller retry this chunk.
                # EXCEPT right after a protective stop: the channel drop then is
                # the stop's side effect, and retrying would drive straight back
                # into whatever just collided.
                if recent_estop:
                    print("  a protective stop happened < 60 s ago; refusing to "
                          "auto-recover -- operator must review first")
                    return 8
                # 2026-11-20 事故：这里以前会自动 resend_external_script。
                # 重发脚本会让控制器把它记着的"旧目标"重新下发 —— 机械臂会
                # 不受控地快速冲向旧位置（当天两次：一次抽搐着跳回轨迹终点，
                # 一次大角度偏转越过 J6 ±150° 安全区间，所有者按下急停）。
                # **运动脚本绝不能有"重新武装机器人"这种副作用。**
                # 现在只报告事实，由所有者确认臂位后再手工恢复。
                if not reverse_ports_up():
                    print("  reverse-control ports are down (protective stop / robot "
                          "powered off).")
                    print("  REFUSING TO AUTO-RECOVER: re-sending external_control "
                          "makes the controller replay its held target and the arm "
                          "can move uncontrolled.")
                    print("  Owner action: confirm the arm's pose on the pendant, "
                          "then run scripts/recover_control.sh deliberately.")
                return 8
        # 2026-11-20 硬闸：等待执行时持续采样关节角，任一关节超出绝对安全区间
        # 立即取消目标。原因：当天出现过"臂走到 J6=157°，越过 ±150° 安全区间"，
        # 而原来的实现只在执行结束后才校验。
        deadline = time.time() + wait_budget
        out_of_range = None
        while not result_future.done() and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            for name, low, high in SAFE_JOINT_LIMITS:
                value = latest.get(name)
                if value is not None and not (low <= value <= high):
                    out_of_range = (name, value, low, high)
                    break
            if out_of_range:
                print(f"JOINT_OUT_OF_RANGE during execution: {out_of_range[0]}="
                      f"{out_of_range[1]:.3f} rad outside [{out_of_range[2]:.3f}, "
                      f"{out_of_range[3]:.3f}]; cancelling the goal NOW")
                cancel = client.cancel_goal_async(handle)
                rclpy.spin_until_future_complete(node, cancel, timeout_sec=5.0)
                return 17
        if not result_future.done():
            print(f"TIMEOUT after {wait_budget:.0f} s; {robot_state_text()}")
            return 5
        result = result_future.result()
        print(
            f"[{index}/{len(jobs)}] status={result.status} "
            f"error={result.result.error_code}"
            f"({TRAJECTORY_ERROR_NAMES.get(result.result.error_code, '?')})"
        )
        if result.status != 4:
            # Name the joints that are actually off, not just the error code: a
            # GOAL_TOLERANCE_VIOLATED with 0.001 rad is noise, with 1 rad it is a
            # joint that never moved and the two need different fixes.
            try:
                target = data["points"][-1]["q"]
                names = data["joint_names"]
                worst = sorted(
                    ((abs(q - latest.get(name, q)), name)
                     for name, q in zip(names, target)), reverse=True)[:3]
                print("terminal error (rad): "
                      + ", ".join(f"{name}={err:.3f}" for err, name in worst))
            except Exception as exc:  # noqa: BLE001 - diagnostics must not mask it
                print(f"terminal error unavailable: {type(exc).__name__}: {exc}")
            return 6
        if args.verify_final > 0.0:
            target = data["points"][-1]["q"]
            names = data["joint_names"]
            worst = sorted(
                ((abs(q - latest.get(name, q)), name)
                 for name, q in zip(names, target)), reverse=True)[:3]
            deviation = worst[0][0] if worst else 0.0
            print("measured deviation (rad): "
                  + ", ".join(f"{name}={err:.4f}" for err, name in worst))
            if deviation > args.verify_final:
                print(f"FINAL_CHECK_FAILED deviation={deviation:.4f} rad "
                      f"> {args.verify_final}")
                return 10
        if args.dwell > 0:
            time.sleep(args.dwell)

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    # 2026-11-20 事故：控制器把"最后一条轨迹的终点"当作保持目标。
    # 之后人手拖动机械臂，控制器仍记着这个旧目标；一旦重挂外部控制脚本
    # （ensure_robot_ready --force / recover_control.sh），它会把这个旧目标
    # 重新下发给机器人 —— 机械臂就会"抽搐着快速跳回旧位置"。
    # 所以每次成功执行都记下来，供 ensure_robot_ready 判断"是否被人手动挪过"。
    try:
        final = jobs[-1][1]["points"][-1]["q"]
        names = jobs[-1][1]["joint_names"]
        target = pathlib.Path("runtime/last_commanded_pose.json")
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(json.dumps({
            "joint_names": names,
            "positions": [float(v) for v in final],
            "stamp": time.time(),
            "source": "execute_assembly_chunks",
        }, ensure_ascii=False, indent=2), encoding="utf-8")
    except Exception:  # noqa: BLE001 - 记录失败不影响执行结果
        pass
    print("ALL_ASSEMBLY_CHUNKS_OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
