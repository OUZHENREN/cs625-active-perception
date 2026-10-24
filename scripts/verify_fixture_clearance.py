#!/usr/bin/env python3
"""在夹具碰撞体存在时，用 MoveIt 的**碰撞检查**逐段验证插入链。

为什么不能只看 `run_pick_and_place.py --plan-only`
--------------------------------------------------
`run_pick_and_place.py` 对槽内紧间隙阶段（insert / straighten / d2b / d2c /
d4 / d5）故意用 `avoid_collisions=False`（见该文件 `NO_COLLISION_CHECK_PHASES`
与 `plan_cartesian(..., avoid_collisions=not _no_collision_check(name))`）：
真机上工件与槽壁本来就接触，让 FCL 判定"接触即失败"会把整条插入链判死。
所以 plan-only 的 ok=True 只证明"运动学走得到"，**不证明**夹具几何不挡路。

本脚本用同一个 MoveIt 实例、同一条 TCP 链，但强制：

    compute_cartesian_path(avoid_collisions=True)
    check_state_validity(contacts=True)

逐段跑 6 个插入相关阶段，失败时打印到底哪个 link 撞到哪里（接触点坐标）。
它不改任何配置、不发运动，只读规划场景 + 调服务。

用法
----
    python3 scripts/verify_fixture_clearance.py
    python3 scripts/verify_fixture_clearance.py --json /tmp/fixture_clearance.json

前置：MoveIt 在跑、`scripts/apply_real_cell_scene.py` 已发布夹具碰撞体。
"""

from __future__ import annotations

import argparse
import json
import sys

import numpy as np
import yaml

sys.stdout.reconfigure(line_buffering=True)

REPO_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
PRE_INSERT = [1.0996, -0.0125, 0.3773]
INSERT = [1.1574, -0.0132, 0.0321]
PRE_INSERT_Q = [-0.004, 0.997, -0.001, 0.083]
INSERT_Q = [-0.003, 1.000, -0.002, -0.004]
EXTRACT_STEP_M = 0.020


def quat_to_matrix(quat) -> np.ndarray:
    """(x, y, z, w) -> 旋转矩阵（与 MoveIt 的 xyzw 约定一致）。"""

    x, y, z, w = (float(v) for v in quat)
    norm = float(np.sqrt(x * x + y * y + z * z + w * w))
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def flange_position(tip_xyz, quat, tool_length: float) -> list[float]:
    """把"夹爪尖 TCP"换算成 MoveIt 的 my_end_effector_link（法兰）位置。

    应用描述里 my_end_effector_link 就是法兰：工具网格从法兰沿 +Z 伸出
    0.207 m（`gripper_0920.stl` 的 bbox z 是 -0.007..0.207）。任务配置里的
    grasp/insert/pre_insert 是**夹爪尖**位置（真实控制器的 TCP）。不做这个
    换算，法兰会被摆到夹爪尖的位置，工具本体就穿进桌面 207 mm —— 碰撞检查
    必然失败，而且失败原因与夹具几何无关。
    """

    rot = quat_to_matrix(quat)
    return (np.asarray(tip_xyz, dtype=float) - rot @ np.array([0.0, 0.0, float(tool_length)])).tolist()


def stage_specs() -> list[dict]:
    """6 个插入相关阶段的笛卡尔目标（位置 + 姿态），顺序即运动顺序。"""

    return [
        {"name": "insert", "start_xyz": PRE_INSERT, "start_q": PRE_INSERT_Q,
         "goal_xyz": INSERT, "goal_q": PRE_INSERT_Q},
        {"name": "straighten", "start_xyz": INSERT, "start_q": PRE_INSERT_Q,
         "goal_xyz": INSERT, "goal_q": INSERT_Q},
        {"name": "d2b_entrance_tilted", "start_xyz": PRE_INSERT, "start_q": PRE_INSERT_Q,
         "goal_xyz": INSERT, "goal_q": PRE_INSERT_Q},
        {"name": "d2c_straighten", "start_xyz": INSERT, "start_q": PRE_INSERT_Q,
         "goal_xyz": INSERT, "goal_q": INSERT_Q},
        {"name": "d4_tilt_for_removal", "start_xyz": INSERT, "start_q": INSERT_Q,
         "goal_xyz": INSERT, "goal_q": PRE_INSERT_Q},
    ]


class Checker:
    def __init__(self, group: str, link: str, seed_joints: list[float],
                 tool_length: float = 0.0):
        self.tool_length = float(tool_length)
        import rclpy
        from moveit_msgs.srv import (
            GetCartesianPath, GetPositionFK, GetPositionIK, GetStateValidity,
        )
        from rclpy.node import Node

        self.rclpy = rclpy
        self.node = Node("verify_fixture_clearance")
        self.group = group
        self.link = link
        self.seed = list(seed_joints)
        self._ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self._fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self._cart = self.node.create_client(GetCartesianPath, "/compute_cartesian_path")
        self._valid = self.node.create_client(GetStateValidity, "/check_state_validity")
        for client, name in ((self._ik, "compute_ik"), (self._fk, "compute_fk"),
                             (self._cart, "compute_cartesian_path"),
                             (self._valid, "check_state_validity")):
            if not client.wait_for_service(timeout_sec=20.0):
                raise SystemExit(f"missing MoveIt service /{name} — is move_group running?")

    # -- primitives -------------------------------------------------------
    def ik(self, xyz, quat, seed=None):
        from geometry_msgs.msg import Pose
        from moveit_msgs.msg import RobotState
        from moveit_msgs.srv import GetPositionIK
        from sensor_msgs.msg import JointState

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (float(v) for v in xyz)
        (pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w) = (float(v) for v in quat)
        request = GetPositionIK.Request()
        request.ik_request.group_name = self.group
        request.ik_request.ik_link_name = self.link
        request.ik_request.pose_stamped.header.frame_id = "world"
        request.ik_request.pose_stamped.pose = pose
        request.ik_request.avoid_collisions = False
        request.ik_request.timeout.sec = 2
        seed = seed or self.seed
        request.ik_request.robot_state = RobotState()
        request.ik_request.robot_state.joint_state = JointState(
            name=list(REPO_JOINTS), position=[float(v) for v in seed])
        future = self._ik.call_async(request)
        self.rclpy.spin_until_future_complete(self.node, future, timeout_sec=10)
        result = future.result()
        if result is None or result.error_code.val != 1:
            return None
        values = {name: value for name, value in zip(
            result.solution.joint_state.name, result.solution.joint_state.position)}
        return [float(values[name]) for name in REPO_JOINTS]

    def fk(self, joints):
        from moveit_msgs.msg import RobotState
        from moveit_msgs.srv import GetPositionFK
        from sensor_msgs.msg import JointState

        request = GetPositionFK.Request()
        request.header.frame_id = "world"
        request.fk_link_names = [self.link]
        request.robot_state = RobotState()
        request.robot_state.joint_state = JointState(
            name=list(REPO_JOINTS), position=[float(v) for v in joints])
        future = self._fk.call_async(request)
        self.rclpy.spin_until_future_complete(self.node, future, timeout_sec=10)
        result = future.result()
        if result is None or not result.pose_stamped:
            return None
        return result.pose_stamped[0].pose

    def check(self, joints):
        from moveit_msgs.msg import RobotState
        from moveit_msgs.srv import GetStateValidity
        from sensor_msgs.msg import JointState

        request = GetStateValidity.Request()
        request.group_name = self.group
        request.robot_state = RobotState()
        request.robot_state.joint_state = JointState(
            name=list(REPO_JOINTS), position=[float(v) for v in joints])
        request.robot_state.is_diff = False
        future = self._valid.call_async(request)
        self.rclpy.spin_until_future_complete(self.node, future, timeout_sec=10)
        result = future.result()
        if result is None:
            return {"valid": None, "contacts": []}
        contacts = []
        for contact in result.contacts:
            contacts.append({
                "body_1": contact.contact_body_1 or "(world)",
                "body_2": contact.contact_body_2 or "(world)",
                "point": [round(float(contact.position.x), 4),
                          round(float(contact.position.y), 4),
                          round(float(contact.position.z), 4)],
                "frame": contact.header.frame_id,
            })
        return {"valid": bool(result.valid), "contacts": contacts[:8]}

    def cartesian(self, start_joints, goal_xyz, goal_q):
        from geometry_msgs.msg import Pose
        from moveit_msgs.msg import RobotState
        from moveit_msgs.srv import GetCartesianPath
        from sensor_msgs.msg import JointState

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (float(v) for v in goal_xyz)
        (pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w) = (float(v) for v in goal_q)
        request = GetCartesianPath.Request()
        request.header.frame_id = "world"
        request.start_state = RobotState()
        request.start_state.joint_state = JointState(
            name=list(REPO_JOINTS), position=[float(v) for v in start_joints])
        request.start_state.is_diff = False
        request.group_name = self.group
        request.link_name = self.link
        request.waypoints = [pose]
        request.max_step = 0.003
        request.jump_threshold = 0.0
        request.avoid_collisions = True
        request.max_velocity_scaling_factor = 0.05
        request.max_acceleration_scaling_factor = 0.05
        future = self._cart.call_async(request)
        self.rclpy.spin_until_future_complete(self.node, future, timeout_sec=30)
        result = future.result()
        if result is None:
            return None
        points = [list(point.positions) for point in result.solution.joint_trajectory.points]
        return {"fraction": float(result.fraction), "points": points}

    def scene_fixture_ids(self):
        from moveit_msgs.srv import GetPlanningScene
        from moveit_msgs.msg import PlanningSceneComponents

        client = self.node.create_client(GetPlanningScene, "/get_planning_scene")
        if not client.wait_for_service(timeout_sec=10):
            return []
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        future = client.call_async(request)
        self.rclipy_wait(future)
        result = future.result()
        if result is None:
            return []
        return [(obj.id, sum(len(mesh.triangles) for mesh in obj.meshes))
                for obj in result.scene.world.collision_objects]

    def rclipy_wait(self, future):
        self.rclpy.spin_until_future_complete(self.node, future, timeout_sec=20)


def run(seed_joints: list[float], json_out: str = "", tool_length: float = 0.0) -> int:
    import rclpy

    rclpy.init()
    checker = Checker("cs625_arm", "my_end_effector_link", seed_joints, tool_length)
    try:
        scene = checker.scene_fixture_ids()
        print("=" * 96)
        print("夹具碰撞体 + 插入链严格验证（avoid_collisions=True, contacts=True）")
        print("=" * 96)
        print("规划场景里的世界碰撞体:",
              ", ".join(f"{name}({triangles} 面)" for name, triangles in scene) or "无")
        print(f"TCP 口径: 任务位姿按夹爪尖解释，法兰 = 夹爪尖 - {tool_length:.3f} m × 工具 +Z"
              if tool_length else
              "TCP 口径: 任务位姿直接当 my_end_effector_link（法兰）用（tool_length=0）")
        if not any("slot_fixture" in name for name, _ in scene):
            print("FAILED: 场景里没有 real_cell_slot_fixture；先跑 "
                  "python3 scripts/apply_real_cell_scene.py")
            return 2

        # 起始状态：pre_insert（-10° 倾角）的 IK
        start = checker.ik(flange_position(PRE_INSERT, PRE_INSERT_Q, checker.tool_length),
                           PRE_INSERT_Q)
        if start is None:
            print("FAILED: pre_insert 位姿无 IK")
            return 2
        start_check = checker.check(start)
        print(f"pre_insert IK ok；该状态 valid={start_check['valid']}")
        if start_check["contacts"]:
            print("  接触:", start_check["contacts"])

        results = []
        current = start
        for spec in stage_specs():
            goal_xyz = flange_position(spec["goal_xyz"], spec["goal_q"], checker.tool_length)
            path = checker.cartesian(current, goal_xyz, spec["goal_q"])
            entry = {"stage": spec["name"]}
            if path is None:
                entry.update({"fraction": 0.0, "ok": False, "reason": "service failure"})
            else:
                entry["fraction"] = round(path["fraction"], 4)
                ok = path["fraction"] >= 0.98
                entry["ok"] = ok
                if ok:
                    current = path["points"][-1]
                    state = checker.check(current)
                    entry["end_valid"] = state["valid"]
                    entry["contacts"] = state["contacts"]
                    if not state["valid"]:
                        entry["ok"] = False
                        entry["reason"] = "end state in collision"
                else:
                    # 失败点：在"最后一个合法关节角"与"目标 IK"之间二分，找到第一个
                    # 碰撞状态，并取回 contacts（哪个 link、接触点在哪）。
                    if path["points"]:
                        last = path["points"][-1]
                        goal_ik = checker.ik(goal_xyz, spec["goal_q"], seed=last)
                        entry["blocked_at_tcp"] = _pose_xyz(checker.fk(last))
                        entry["contacts"] = checker.check(last)["contacts"]
                        if goal_ik is not None:
                            low, high = np.asarray(last), np.asarray(goal_ik)
                            for _ in range(12):
                                mid = 0.5 * (low + high)
                                if checker.check(mid)["valid"]:
                                    low = mid
                                else:
                                    high = mid
                            bad = checker.check(high)
                            entry["contacts"] = bad["contacts"] or entry["contacts"]
                            entry["first_invalid_tcp"] = _pose_xyz(checker.fk(high))
                    entry["reason"] = f"cartesian fraction {path['fraction']:.3f} < 0.98"
            results.append(entry)
            status = "PASS" if entry["ok"] else "FAIL"
            print(f"[{spec['name']:<22}] {status}  fraction={entry.get('fraction')}"
                  + (f"  {entry.get('reason','')}" if not entry["ok"] else ""))
            if entry.get("blocked_at_tcp"):
                print(f"    最后合法 TCP {entry['blocked_at_tcp']}"
                      + (f"；第一个碰撞 TCP {entry['first_invalid_tcp']}"
                         if entry.get("first_invalid_tcp") else ""))
            for contact in entry.get("contacts") or []:
                print(f"    接触 {contact['body_1']} <-> {contact['body_2']} @ "
                      f"{contact['point']} ({contact['frame']})")

        print()
        print("汇总:", "全部通过" if all(item["ok"] for item in results)
              else "存在失败段（见上）")
        if json_out:
            with open(json_out, "w", encoding="utf-8") as handle:
                json.dump({"scene": scene, "results": results}, handle,
                          ensure_ascii=False, indent=1)
            print("JSON ->", json_out)
        return 0 if all(item["ok"] for item in results) else 1
    finally:
        checker.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _pose_xyz(pose):
    if pose is None:
        return None
    return [round(float(pose.position.x), 4), round(float(pose.position.y), 4),
            round(float(pose.position.z), 4)]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--json", default="")
    parser.add_argument("--tool-length", type=float, default=0.0,
                        help="夹爪尖到法兰的距离（m）。应用描述里工具网格 +Z 长 0.207 m；"
                             "任务配置的位姿按夹爪尖给出的，所以验证时要减掉它。")
    parser.add_argument("--start-joints", default=(
        "0.559681835159803,-0.886110693244363,-2.6479421412927606,"
        "0.004946539382850723,1.8827091289653233,1.4481534658974855"))
    arguments = parser.parse_args()
    seed = [float(v) for v in arguments.start_joints.split(",") if v.strip()]
    if len(seed) != 6:
        raise SystemExit(f"--start-joints needs 6 values, got {len(seed)}")
    return run(seed, arguments.json, arguments.tool_length)


if __name__ == "__main__":
    raise SystemExit(main())
