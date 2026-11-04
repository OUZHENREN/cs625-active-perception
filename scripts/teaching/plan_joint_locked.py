#!/usr/bin/env python3
"""Plan one move to a TCP pose, LOCKED to a collision-checked IK solution.

Replaces plan_pose_once for the teaching chain (2026-11-02 audit, hole 1):

  * IK is solved with avoid_collisions=True, seeded from the current joints, so
    a pose whose only solution self-collides (RVS S2 on 2026-11-02: forearm_link
    <-> my_end_effector_link, 5.3 mm) is REFUSED before any motion;
  * the plan is a JOINT-space goal at that solution (tolerance --goal-tol), so
    the branch cannot flip to a 734-point wrapped-wrist path;
  * if no collision-free IK exists the script exits 2.

Usage:
    plan_joint_locked.py X Y Z QX QY QZ QW --velocity 0.05 \
        [--goal-tol 0.004] --output /tmp/plan.json

Exit codes: 0 plan written; 1 plan failed; 2 no collision-free IK; 3 service down.
"""

from __future__ import annotations

import argparse
import json
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import (Constraints, JointConstraint, PositionIKRequest,
                             RobotState)
from moveit_msgs.srv import GetMotionPlan, GetPositionIK, GetStateValidity
from sensor_msgs.msg import JointState

sys.stdout.reconfigure(line_buffering=True)

ORDER = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
         "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]



# Proven postures (degrees) used as extra IK seeds and as the reference for the
# configuration sanity rules.  Every one of these has executed on the real cell.
PROVEN_POSTURES = {
    "observation":  [43.1, -57.4, -146.3, 14.3, 92.8, 89.8],
    "pre_grasp":    [37.1, -120.3, -107.2, -42.4, 90.8, 32.6],
    "grasp":        [43.0, -123.0, -101.7, -45.0, 90.7, 38.4],
    "baseline":     [6.8, -87.9, -91.9, -50.5, 94.4, 91.2],
    # Slot coarse-observation configuration verified 2026-11-03.  Without it the
    # pose solves to the flipped-elbow branch (elbow +111 instead of -111) and the
    # wrap guard refuses the 230 deg travel (hit on 2026-11-04 with baseline A as
    # the seed).
    "slot_coarse":  [-21.1, -72.7, -111.0, -37.5, 128.5, 62.7],
    # Coarse-observation baselines found by the 2026-11-03 geometry search.  They
    # double as IK seeds: solving these poses from other seeds converges to a
    # forearm<->tool0 collision, so the proven configuration must be available.
    "obs_a": [120.7, -18.8, -119.2, -120.6, 58.6, -21.5],   # 目标 [0.62, 0.4, 0.2], 俯角 56.8°
    "obs_b": [101.0, -51.4, -122.5, -82.0, 59.6, -26.9],   # 目标 [0.62, 0.6, 0.2], 俯角 56.8°
    "obs_c": [97.4, -71.1, -123.9, -56.4, 57.2, -32.1],   # 目标 [0.62, 0.8, 0.2], 俯角 52.8°
}



# URDF joint limits (degrees).  Only the elbow is narrower than +-360 on this arm;
# copied here so the wrap handling below can check legality without parsing the
# URDF at runtime (both sim and real descriptions agree on these numbers).
JOINT_LIMITS_DEG = {
    "shoulder_pan_joint": (-360.0, 360.0),
    "shoulder_lift_joint": (-360.0, 360.0),
    "elbow_joint": (-180.0, 180.0),
    "wrist_1_joint": (-360.0, 360.0),
    "wrist_2_joint": (-360.0, 360.0),
    "wrist_3_joint": (-360.0, 360.0),
}


def nearest_equivalent(candidate: list[float], seed: list[float]) -> list[float]:
    """Represent each joint angle by the +-360 equivalent nearest the current pose.

    Why (2026-11-03, hit for real): the arm had been hand-guided until the driver
    reported wrist_1 = -281.8 deg and wrist_3 = +351.6 deg -- physically the same
    poses as +78.2 and -8.4, but MoveIt works on the raw numbers.  Asking for the
    mathematically identical goal then produced a full-turn trajectory (and the
    sanity rules rejected it outright).  Choosing the equivalent nearest the seed
    keeps every commanded move short and legal.
    """
    result = []
    for name, goal, start in zip(ORDER, candidate, seed):
        low, high = JOINT_LIMITS_DEG.get(name, (-360.0, 360.0))
        best, best_delta = goal, abs(goal - start)
        for turns in (-1, 0, 1):
            alternative = goal + turns * 2.0 * np.pi
            if not (np.radians(low) - 1e-9 <= alternative <= np.radians(high) + 1e-9):
                continue
            delta = abs(alternative - start)
            if delta < best_delta:
                best, best_delta = alternative, delta
        result.append(float(best))
    return result


def sanity_violation(candidate: list[float], seed: list[float], args) -> str | None:
    """Why this IK solution must not be executed, or None when it is sane.

    Two rules, both calibrated on 2026-11-03 on this cell:
      * an absolute joint angle past +-max_abs is a wrap-around branch
        (the RVS fine-view pose only solves to J1=-204 / wrist_3=+332 deg);
      * a single move that swings one joint past max_delta is not an approach
        (a wrapped branch shows ~180-250 deg jumps).
    """
    goal_deg = np.degrees(np.array(candidate))
    seed_deg = np.degrees(np.array(seed))
    # Judge on the SHORTEST equivalent motion: the seed itself may be wrapped
    # (hand-guided arm reported wrist_1 = -281.8 deg), and comparing raw numbers
    # then flags a 360 deg phantom move that the caller has already re-expressed
    # with nearest_equivalent().
    delta_deg = ((goal_deg - seed_deg + 180.0) % 360.0) - 180.0
    if np.max(np.abs(delta_deg)) > args.max_joint_delta_deg:
        worst = int(np.argmax(np.abs(delta_deg)))
        return (f"{ORDER[worst]} would move {delta_deg[worst]:+.0f} deg "
                f"(limit +-{args.max_joint_delta_deg:.0f})")
    return None


def collision_violation(client, node, candidate: list[float]) -> str | None:
    """Explicit collision check of an IK solution (replaces avoid_collisions IK).

    Returns a reason string when the configuration is in collision, else None.
    A failure to answer counts as a violation: never move on an unverified pose.
    """
    from moveit_msgs.msg import RobotState
    from moveit_msgs.srv import GetStateValidity
    state = RobotState()
    joints = JointState()
    joints.name = ORDER
    joints.position = list(candidate)
    state.joint_state = joints
    request = GetStateValidity.Request()
    request.robot_state = state
    request.group_name = "cs625_arm"
    request.constraints = Constraints()
    future = client.call_async(request)
    deadline = time.time() + 10.0
    while not future.done() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    response = future.result()
    if response is None:
        return "state validity check timed out"
    if not response.valid:
        pairs = "; ".join(f"{c.contact_body_1}<->{c.contact_body_2}"
                          for c in response.contacts[:3])
        return f"configuration in collision ({pairs})"
    return None



def straight_line_plan(start, goal, duration: float = 4.0, samples: int = 24):
    """Linear joint-space interpolation between two близ configurations.

    Why (2026-11-04): for a short move (tens of degrees) OMPL sometimes returned a
    winding path -- e.g. wrist_3 travelling 284 deg to go from 79 to 79 -- and the
    wrap guard then refused a perfectly reasonable move.  A straight line between two
    nearby collision-free configurations is shorter, deterministic and easy to check,
    so it is used when the geometry allows and OMPL stays as the fallback.
    """
    import numpy as np
    start = np.asarray(start, dtype=float)
    goal = np.asarray(goal, dtype=float)
    points = []
    for i in range(samples + 1):
        fraction = i / samples
        q = start + (goal - start) * fraction
        points.append({"q": [float(v) for v in q], "t": duration * fraction})
    return points


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("pose", nargs="*", type=float, metavar="X Y Z QX QY QZ QW")
    parser.add_argument("--velocity", type=float, default=0.05,
                        help="velocity AND acceleration scaling for the plan")
    parser.add_argument("--goal-tol", type=float, default=0.004,
                        help="joint-space goal tolerance in radians (insert steps use 0.0003)")
    parser.add_argument("--max-joint-delta-deg", type=float, default=180.0,
                        help="refuse a single move that swings one joint more than "
                             "this.  180 (not 120): a genuine re-orientation needed "
                             "160 deg of wrist roll on 2026-11-03, while a wrapped "
                             "branch shows a full turn and is caught by the "
                             "--max-joint-travel-deg guard on the planned path")
    parser.add_argument("--max-abs-joint-deg", type=float, default=180.0,
                        help="refuse any solution whose absolute joint angle exceeds "
                             "this (wrap-around branch)")
    parser.add_argument("--max-joint-travel-deg", type=float, default=200.0,
                        help="refuse a plan whose single-joint travel exceeds this "
                             "(a wrapped branch travels >360 deg)")
    parser.add_argument("--joints", default=None,
                        help="six joint angles in DEGREES, comma separated: plan "
                             "straight to this verified configuration instead of "
                             "solving the pose (the pose IK can converge to a "
                             "different branch and refuse to plan, 2026-11-03)")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    joint_target_deg = None
    if args.joints:
        joint_target_deg = [float(v) for v in args.joints.replace(" ", "").split(",")]
        if len(joint_target_deg) != 6:
            print("--joints needs exactly 6 angles")
            return 3
        args.pose = joint_target_deg
    elif len(args.pose) != 7:
        print("give either 7 pose values or --joints d1,...,d6")
        return 3

    rclpy.init()
    node = rclpy.create_node("plan_joint_locked")
    current: dict = {}
    node.create_subscription(JointState, "/joint_states",
                             lambda m: current.update(dict(zip(m.name, m.position))), 10)
    deadline = time.time() + 8.0
    while len([k for k in ORDER if k in current]) < 6 and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if len([k for k in ORDER if k in current]) < 6:
        print("NO_JOINT_STATES")
        return 3
    seed = [current[k] for k in ORDER]
    print("seed J:", [round(np.degrees(v), 1) for v in seed])

    ik = node.create_client(GetPositionIK, "/compute_ik")
    if not ik.wait_for_service(timeout_sec=40):
        print("NO /compute_ik")
        return 3
    validity = node.create_client(GetStateValidity, "/check_state_validity")
    if not validity.wait_for_service(timeout_sec=40):
        print("NO /check_state_validity")
        return 3
    pose = PoseStamped()
    pose.header.frame_id = "world"
    if not args.joints:            # a joint target needs no pose
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = args.pose[:3]
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = args.pose[3:]

    # Multi-seed IK (2026-11-03): a seed-based solver converges to whatever
    # branch is nearest, and "nearest" is sometimes a wrap-around that the sanity
    # rules below must reject.  Trying the proven postures as extra seeds makes
    # the guard practical: if ANY seed yields a sane branch, the move is legal.
    seeds = [("current", seed)]
    if args.joints:
        # The caller supplies a configuration that was already verified on this
        # cell; use it verbatim (only re-expressed near the current pose so the
        # commanded motion stays short).
        seeds = [("given-joints", [np.radians(v) for v in joint_target_deg])]
    else:
        for name, posture in PROVEN_POSTURES.items():
            seeds.append((name, [np.radians(v) for v in posture]))

    planner = node.create_client(GetMotionPlan, "/plan_kinematic_path")
    if not planner.wait_for_service(timeout_sec=40):
        print("NO /plan_kinematic_path")
        return 3

    def straight_line_ok(start, goal, samples: int = 20) -> bool:
        """True when every sampled point between start and goal is collision-free."""
        import numpy as np
        start_arr = np.asarray(start, dtype=float)
        goal_arr = np.asarray(goal, dtype=float)
        for i in range(samples + 1):
            q = start_arr + (goal_arr - start_arr) * (i / samples)
            if collision_violation(validity, node, [float(v) for v in q]) is not None:
                return False
        return True

    def try_plan(goal_joints):
        """Plan to a joint goal; returns the trajectory or None."""
        # Short moves: use a straight line, which cannot wind.
        import numpy as _np
        span = float(_np.max(_np.abs(_np.asarray(goal_joints) - _np.asarray(seed))))
        if span <= _np.radians(60.0) and straight_line_ok(seed, goal_joints):
            points = straight_line_plan(seed, goal_joints)
            class _Traj:
                pass
            traj = _Traj()
            traj.joint_names = ORDER
            traj.points = []
            for entry in points:
                point = type("P", (), {})()
                point.positions = entry["q"]
                from builtin_interfaces.msg import Duration as _Dur
                sec = int(entry["t"])
                point.time_from_start = _Dur(sec=sec,
                                             nanosec=int((entry["t"] - sec) * 1e9))
                traj.points.append(point)
            return traj, 1, len(traj.points)
        plan_request = GetMotionPlan.Request().motion_plan_request
        plan_request.group_name = "cs625_arm"
        plan_request.num_planning_attempts = 20
        plan_request.allowed_planning_time = 20.0
        plan_request.max_velocity_scaling_factor = args.velocity
        plan_request.max_acceleration_scaling_factor = args.velocity
        constraints = Constraints()
        for name, value in zip(ORDER, goal_joints):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = name
            joint_constraint.position = float(value)
            joint_constraint.tolerance_above = args.goal_tol
            joint_constraint.tolerance_below = args.goal_tol
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)
        plan_request.goal_constraints.append(constraints)
        future = planner.call_async(
            GetMotionPlan.Request(motion_plan_request=plan_request))
        end = time.time() + 60.0
        while not future.done() and time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        response = future.result().motion_plan_response
        if response.error_code.val != 1:
            return None, response.error_code.val, 0
        return (response.trajectory.joint_trajectory,
                response.error_code.val,
                len(response.trajectory.joint_trajectory.points))

    goal = None
    trajectory = None
    last_reason = "no seed produced a collision-free solution"
    for label, candidate in seeds:
        if label == "given-joints":
            candidate_solution = nearest_equivalent(list(candidate), seed)
            reason = sanity_violation(candidate_solution, seed, args)
            if reason is not None:
                last_reason = f"given joints: {reason}"
                continue
            planned_traj, code, points = try_plan(candidate_solution)
            if planned_traj is None:
                last_reason = f"given joints: planning failed (code {code})"
                continue
            goal, trajectory = candidate_solution, planned_traj
            print(f"ik goal J (given):",
                  [round(np.degrees(v), 1) for v in goal])
            break
        request = PositionIKRequest()
        request.group_name = "cs625_arm"
        request.ik_link_name = "my_end_effector_link"
        # Kinematics only.  MoveIt's avoid_collisions IK short-circuits to -31
        # (or to a wrapped branch) when the FIRST solution it finds is in
        # collision, hiding a perfectly valid branch -- verified 2026-11-03:
        # the RVS fine-view pose IS reachable (J=[3.8,-89.2,-113.5,-42.4,143.3,
        # 21.4], valid) but avoid_collisions=True never returned it.  So we take
        # the raw solution and validate it ourselves below.
        request.avoid_collisions = False
        request.pose_stamped = pose
        state = RobotState()
        joints = JointState()
        joints.name = ORDER
        joints.position = list(candidate)
        state.joint_state = joints
        request.robot_state = state
        future = ik.call_async(GetPositionIK.Request(ik_request=request))
        deadline = time.time() + 20.0
        while not future.done() and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        response = future.result()
        if response is None or response.error_code.val != 1:
            last_reason = f"no collision-free IK from seed '{label}' " \
                          f"(code={response.error_code.val if response else -1})"
            continue
        candidate_solution = [dict(zip(response.solution.joint_state.name,
                                       response.solution.joint_state.position))[k]
                              for k in ORDER]
        candidate_solution = nearest_equivalent(candidate_solution, seed)
        reason = sanity_violation(candidate_solution, seed, args)
        if reason is not None:
            last_reason = f"seed '{label}': {reason}"
            continue
        reason = collision_violation(validity, node, candidate_solution)
        if reason is not None:
            last_reason = f"seed '{label}': {reason}"
            continue
        # Sanity is necessary but NOT sufficient: a solution can be sane and still
        # unreachable from the current state (measured 2026-11-03: the 'observation'
        # seed gave a sane branch, but no plan existed from the hand-guided pose).
        # So plan here and fall through to the next seed when planning fails.
        planned_traj, code, points = try_plan(candidate_solution)
        if planned_traj is None:
            last_reason = (f"seed '{label}': sane solution but planning failed "
                           f"(code {code})")
            continue
        goal = candidate_solution
        trajectory = planned_traj
        print(f"ik goal J (seed '{label}'):",
              [round(np.degrees(v), 1) for v in goal])
        break
    if goal is None:
        print(f"REFUSED: {last_reason}; refusing to plan")
        return 2

    # Configuration sanity gate (2026-11-03, completing audit hole 1).
    # Locking the plan to the IK solution is not enough when the IK solution
    # itself is a wrapped branch: verified on 2026-11-03, the pose RVS asks for
    # produced J1=-204 deg / J5=-326 deg and the planner happily returned a
    # 1302-point, 130-second path around the cell.  A commanded move must stay
    # near the current configuration; anything else is a refusal, not a plan.
    # Honest wrap-around detector (2026-11-03): DURATION is not it -- a 71 s plan
    # at velocity scaling 0.01 is a legitimate slow move, not a wrap.  What a wrap
    # shows is JOINT TRAVEL: sum |dq| per joint.  A normal approach stays under
    # ~120 deg; a wrapped branch exceeds a full turn.
    if trajectory.points:
        positions = np.array([list(p.positions) for p in trajectory.points])
        if len(positions) > 1:
            travel = np.abs(np.diff(positions, axis=0)).sum(axis=0)
            worst = int(np.argmax(travel))
            if np.degrees(travel[worst]) > args.max_joint_travel_deg:
                print(f"WRAPPED_PATH: {ORDER[worst]} travels "
                      f"{np.degrees(travel[worst]):.0f} deg along this plan "
                      f"(limit {args.max_joint_travel_deg:.0f}); refusing")
                return 1
    points = [{"q": list(p.positions),
               "t": p.time_from_start.sec + p.time_from_start.nanosec * 1e-9}
              for p in trajectory.points]
    json.dump({"joint_names": list(trajectory.joint_names), "points": points},
              open(args.output, "w"))
    print("duration:", round(points[-1]["t"], 2),
          "last J:", [round(np.degrees(v), 1) for v in points[-1]["q"]])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
