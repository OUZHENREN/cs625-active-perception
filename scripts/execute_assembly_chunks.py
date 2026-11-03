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

    def effort_alive() -> tuple[bool, float]:
        """The torque interface must show a plausible load before any motion.

        On 2026-11-03 a controller switch left the effort interface reading ~0 N
        while the arm was carrying a 19 kg part; every contact detector would
        then silently see nothing, and the run collided with the fixture.  A dead
        stream reads < 1.0; a loaded arm reads >= 8 on joint 2 alone.

        Spins to collect a real window first: the startup probe exits as soon as
        POSITIONS arrive, which can leave the effort window empty at the first
        chunk (measured: the gate fired with peak 0.000 although the stream was
        alive at 15.9).
        """
        probe = time.time() + 0.8
        while time.time() < probe:
            rclpy.spin_once(node, timeout_sec=0.05)
        if len(efforts) < 10:
            return False, 0.0
        recent = efforts[-16:]
        return max(recent) >= 1.0, float(max(recent))

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

    for index, path in enumerate(files, 1):
        data = json.load(open(path, encoding="utf-8"))
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = data["joint_names"]
        for point in data["points"]:
            message = JointTrajectoryPoint()
            message.positions = point["q"]
            scaled = point["t"] * args.time_scale
            message.time_from_start = Duration(
                sec=int(scaled), nanosec=int((scaled - int(scaled)) * 1e9)
            )
            goal.trajectory.points.append(message)
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
        for name in data["joint_names"]:
            tolerance = JointTolerance()
            tolerance.name = name
            tolerance.position = args.goal_tolerance
            goal.goal_tolerance.append(tolerance)
        scaled_total = (
            goal.trajectory.points[-1].time_from_start.sec
            + goal.trajectory.points[-1].time_from_start.nanosec * 1e-9
        )
        # +5 s, not +25: the controller must say "I did not make it" quickly.
        goal.goal_time_tolerance = Duration(sec=int(scaled_total + 5))

        offset = first_point_far(data)
        if offset > args.max_start_offset:
            print(
                f"FIRST_POINT_FAR offset={offset:.3f} rad > {args.max_start_offset}; "
                "refusing to execute a stale trajectory"
            )
            return 7
        alive, peak = effort_alive()
        if not alive:
            print(
                f"EFFORT_DEAD: the torque interface shows no plausible load "
                f"(peak {peak:.3f}); every contact detector would be blind, so "
                "this trajectory is refused.  Restart the driver stack and "
                "re-check before moving again."
            )
            return 11
        print(f"[{index}/{len(files)}] start {path} time_scale={args.time_scale} "
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
            f"[{index}/{len(files)}] status={result.status} "
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
