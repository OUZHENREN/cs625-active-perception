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
MAX_JOINT_SPEED_RAD_S = 0.05     # 单关节速度硬上限 ≈ 2.9 °/s（远低于任何示教/装配需求）
MAX_JOINT_DELTA_RAD = 0.44       # 单个计划文件里任一关节的行程硬上限 = 25°
MIN_SEGMENT_SEC = 0.25           # 每段最短时间，避免时间戳过密导致控制器插值抖动


def _arm_indices(names: list[str]) -> list[int]:
    return [j for j, n in enumerate(names) if "finger" not in n]


def govern_timestamps(data: dict, time_scale: float) -> tuple[list[float], dict]:
    """按 MAX_JOINT_SPEED_RAD_S 重新配时：只会更慢，绝不会比计划更快。

    返回 (times, info)。times 已经乘过 time_scale（>=1，只会更慢）。
    """
    names = data["joint_names"]
    points = data["points"]
    idx = _arm_indices(names)
    times = [0.0]
    for i in range(1, len(points)):
        plan_dt = float(points[i]["t"]) - float(points[i - 1]["t"])
        dq = max(abs(points[i]["q"][j] - points[i - 1]["q"][j]) for j in idx)
        needed = dq / MAX_JOINT_SPEED_RAD_S
        times.append(times[-1] + max(plan_dt, needed, MIN_SEGMENT_SEC))
    scale = max(1.0, float(time_scale))
    times = [value * scale for value in times]
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


def joint_deltas(data: dict) -> dict[str, float]:
    """每个关节在整个计划里的总行程（弧度）。"""
    names = data["joint_names"]
    points = data["points"]
    return {names[j]: abs(points[-1]["q"][j] - points[0]["q"][j])
            for j in _arm_indices(names)}


def build_parser() -> argparse.ArgumentParser:
    """Build the CLI parser without touching ROS.

    Kept separate from main() so the argument contract can be tested without a
    ROS runtime; main() still owns every motion-related code path.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("pattern")
    parser.add_argument("--time-scale", type=float, default=2.0,
                        help="MULTIPLIES the trajectory timestamps: >1 plays SLOWER, "
                             "<1 plays FASTER and is refused.  2026-10-30: time-scale "
                             "0.5 was passed believing it meant 'half speed'; it meant "
                             "double speed and the arm hit the workpiece table.")
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
    parser.add_argument("--allow-large-move", action="store_true",
                        help="不自动分段，直接一次性下发（默认会自动分段）。加这个参数前"
                             "必须已打印 6 个关节角并得到所有者明确同意（AGENTS 安全红线）")
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
    jobs: list[tuple[str, dict]] = []
    for path in files:
        data = json.loads(path.read_text(encoding="utf-8"))
        deltas = joint_deltas(data)
        worst_joint, worst_delta = max(deltas.items(), key=lambda kv: kv[1])
        needs_split = worst_delta > MAX_JOINT_DELTA_RAD + 1e-9
        if not needs_split or args.allow_large_move:
            jobs.append((str(path), data))
            continue
        if args.no_auto_split:
            table = "  ".join(f"{n.replace('_joint','')}={math.degrees(v):.1f}°"
                              for n, v in deltas.items())
            print(
                f"REFUSED_LARGE_MOVE: {path.name} asks {worst_joint} to travel "
                f"{math.degrees(worst_delta):.1f}° (> {math.degrees(MAX_JOINT_DELTA_RAD):.0f}° "
                f"hard limit) and --no-auto-split was given.  Joint deltas: {table}.  "
                "Nothing was sent."
            )
            return 13
        chunks = split_plan(data, MAX_JOINT_DELTA_RAD)
        print(f"[{path.name}] 单关节行程 {math.degrees(worst_delta):.1f}° > "
              f"{math.degrees(MAX_JOINT_DELTA_RAD):.0f}°，**自动分 {len(chunks)} 段**执行"
              f"（每段 ≤{math.degrees(MAX_JOINT_DELTA_RAD):.0f}°）")
        for part, chunk in enumerate(chunks, 1):
            jobs.append((f"{path.name}#{part}/{len(chunks)}", chunk))

    for index, (path, data) in enumerate(jobs, 1):
        # --- 2026-11-19 硬闸 2：速度（不信任计划时间戳，按上限重新配时）---
        governed, gov = govern_timestamps(data, args.time_scale)
        print(f"[{index}/{len(jobs)}] speed governor: plan {gov['plan_duration']:.2f}s -> "
              f"exec {gov['exec_duration']:.2f}s, max joint speed "
              f"{gov['max_speed']:.3f} rad/s ({math.degrees(gov['max_speed']):.1f}°/s, "
              f"cap {MAX_JOINT_SPEED_RAD_S:.3f})")
        if gov["max_speed"] > MAX_JOINT_SPEED_RAD_S + 1e-6:
            print(f"REFUSED_SPEED: governed trajectory still exceeds the cap; nothing sent.")
            return 14

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = data["joint_names"]
        for point, stamp in zip(data["points"], governed):
            message = JointTrajectoryPoint()
            message.positions = point["q"]
            message.time_from_start = Duration(
                sec=int(stamp), nanosec=int((stamp - int(stamp)) * 1e9)
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
        print(f"[{index}/{len(jobs)}] start {path} time_scale={args.time_scale} "
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
                if args.auto_recover and not reverse_ports_up():
                    print("  reverse-control ports are down; re-sending "
                          "external_control on the robot")
                    if resend_external_script():
                        time.sleep(2.0)
                        print("  resend succeeded; the caller may retry this chunk")
                        return 10
                    print("  resend FAILED; owner intervention needed")
                return 8
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
    print("ALL_ASSEMBLY_CHUNKS_OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
