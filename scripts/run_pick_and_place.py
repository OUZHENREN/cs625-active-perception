#!/usr/bin/env python3
"""Orchestrated pick-and-place for CS625 (skeleton, parameterised).

Encodes the flow verified manually on 2026-10-22:

    observe -> pre_grasp -> gripper init -> grip -> grasp feed -> lift
    -> transit -> staging -> pre_insert -> insert (tilted) -> straighten
    -> release -> pure translation withdrawal -> slot baseline

Design rules baked in (from the two E-stop incidents):

* every phase is executed in small chunks with a dwell between them;
* after release the arm moves only along the insertion axis until clear;
* on a failed chunk the controller is recovered once and the chunk retried;
* every phase checks TCP against its target before continuing;
* under ``--world sim`` that acceptance uses the workpiece pose instead: Gazebo has no
  /tcp_pose_broadcaster, so the part itself must follow the gripper (relative pose held
  within 10 mm / 3°) and end up inside the slot volume; the real path is unchanged;
* nothing is executed unless ``--confirm`` is passed.

Usage:
  python3 scripts/run_pick_and_place.py --config config/pick_place_real.yaml
  python3 scripts/run_pick_and_place.py --config ... --confirm --phases pre_grasp,grasp
"""

from __future__ import annotations

import argparse
import os
import sys
import json
import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]


def run(command, timeout: float = 180.0):
    """跑一个子进程，返回 (退出码, 合并后的输出)。阶段里调用外部脚本统一走这里。"""
    completed = subprocess.run(command, capture_output=True, text=True, timeout=timeout)
    return completed.returncode, (completed.stdout or "") + (completed.stderr or "")

import numpy as np
import rclpy
import tf2_ros
import yaml
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from moveit_msgs.srv import (
    GetCartesianPath,
    GetMotionPlan,
    GetPositionFK,
    GetPositionIK,
)
from rclpy.duration import Duration as RDuration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String

JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
GRIPPER_PS1 = r"C:\Users\Public\hitbot_cmd.ps1"
# Absolute, so the runner behaves the same when it is started from anywhere.  The
# old relative strings only worked when the caller happened to be in the repository
# root; control_health.py in particular was invoked from ``try:``-free code and a
# FileNotFoundError there produced a bare traceback with no hint.
SCRIPTS_DIR = Path(__file__).resolve().parent
RECOVER_SH = str(SCRIPTS_DIR / "recover_control.sh")
HEALTH_PY = str(SCRIPTS_DIR / "control_health.py")
CHUNK_PY = str(SCRIPTS_DIR / "execute_assembly_chunks.py")
VERIFY_PY = str(SCRIPTS_DIR / "verify_pose.py")
# Goals are sent to a controller that may be inactive even though the driver and
# the action server are up; give the operator a bounded, named failure instead of
# hanging (measured 2026-10-26: a missing move_group left the runner silent for
# ever, with no output at all).
MOVEIT_SERVICE_TIMEOUT_S = 20.0


@dataclass
class SpeedProfile:
    """Per-phase speed plan: fast cruise plus an optional slow final segment."""

    velocity: float = 0.03
    time_scale: float = 1.5
    chunks: int = 4
    slow_tail_mm: float = 0.0
    slow_velocity: float = 0.005
    slow_time_scale: float = 3.0
    # Axial band, measured as the BACKOFF distance from the seated insert pose
    # along the insertion axis, where the gripper must move slowly.  The gripper
    # carries nothing when it enters or leaves, so only the band around the
    # workpiece handles needs the slow treatment -- the rest of the travel does
    # not and used to waste most of the phase's time.
    slow_band_mm: tuple[float, float] | None = None

    @staticmethod
    def from_dict(raw: dict | None, fallback_velocity: float, fallback_scale: float,
                  fallback_chunks: int) -> "SpeedProfile":
        if not raw:
            return SpeedProfile(fallback_velocity, fallback_scale, fallback_chunks)
        return SpeedProfile(
            velocity=float(raw.get("velocity", fallback_velocity)),
            time_scale=float(raw.get("time_scale", fallback_scale)),
            chunks=int(raw.get("chunks", fallback_chunks)),
            slow_tail_mm=float(raw.get("slow_tail_mm", 0.0)),
            slow_velocity=float(raw.get("slow_velocity", 0.005)),
            slow_time_scale=float(raw.get("slow_time_scale", 3.0)),
            slow_band_mm=(
                (float(raw["slow_band_mm"][0]), float(raw["slow_band_mm"][1]))
                if raw.get("slow_band_mm") else None),
        )


@dataclass
class Config:
    pre_grasp: np.ndarray
    grasp: np.ndarray
    grasp_q: tuple[float, float, float, float]
    transit: np.ndarray
    staging: np.ndarray
    pre_insert: np.ndarray
    insert: np.ndarray
    insert_q: tuple[float, float, float, float]
    pre_insert_q: tuple[float, float, float, float]
    baseline: np.ndarray
    baseline_q: tuple[float, float, float, float]
    advance_axis: np.ndarray
    grip_width_mm: float
    # 夹持电流（A）。以前只在调用处用 getattr(..., 0.1) 兜底，配置里改了也不生效。
    grip_current: float = 0.1
    # 测量结果参与抓取点修正：measure_object 的输出路径 + 我们的 CAD 系到示教 box 系的固定偏移
    # （offset = [x,y,z,qx,qy,qz,qw]；未标定时为空，抓取仍用常数并在日志里说明）。
    workpiece_measurement: str = "runtime/measurements/workpiece_latest.json"
    workpiece_frame_offset: list = field(default_factory=list)
    workpiece_reference: dict = field(default_factory=dict)
    # CAD 包围盒（米，6 个数）：用来检查"抓取点是否落在实测工件里"，不依赖坐标系换算。
    workpiece_bbox: list = field(default_factory=list)
    lift_total_mm: float = 155.0
    safe_clear_mm: float = 150.0
    velocity_scale: float = 0.02
    assembly_velocity_scale: float = 0.01
    time_scale: float = 2.0
    dwell_sec: float = 1.0
    goal_tolerance: float = 0.002
    # 关节目标容差（rad）：传给 MoveIt 的 JointConstraint.tolerance_above/below。
    # 默认 0.02 = 真机一直用的值；仿真覆盖层会把它收紧（见 config/tasks/world_sim.yaml
    # 的 goal_joint_tolerance），原因：仿真靠"位姿跟随"表达夹持，抓取时工具停在哪里、
    # 工件就带着那个相对位姿走，规划端点偏差会一直传到插槽；真机是物理夹紧，夹爪夹到
    # 哪儿就是哪儿。
    goal_joint_tolerance: float = 0.02
    max_chunks: int = 7
    phases: list[str] = field(default_factory=list)
    speeds: dict = field(default_factory=dict)
    # safe      = conservative: many segments, every phase measured afterwards.
    # efficient = one segment per phase (no chunking, no dwell), fast cruise,
    #             assembly speed unchanged.  The caps below are per mode.
    mode: str = "safe"
    scope: str = ""
    world: str = "real"
    gripper_max_opening_mm: float = 50.0
    gripper_span_min_mm: float = 204.0
    gripper_span_max_mm: float = 246.57
    action: str = "/arm_controller/follow_joint_trajectory"
    observe: dict = field(default_factory=dict)
    observe_settle_sec: float = 10.0
    nbv_distance_m: float = 0.65
    nbv_elevation_deg: float = 25.0
    camera_fast_during_assembly: bool = True
    # 观测位检查命令的按模式覆盖：{waypoint: "shell 命令"}。
    # 位姿仍由 common.yaml 的 observe 提供，这里只换"用什么算子检查/测量"，
    # 这样效率模式可以换成本仓库的测量管线，而示教/安全模式继续用原检查。
    observe_checks: dict = field(default_factory=dict)
    process_images: bool = True
    verify: bool = True
    verify_pos_mm: float = 3.0
    verify_rot_rad: float = 0.01
    # 仿真里被搬运的工件模型名。仿真验收要跑 `gz model -m <model> --pose` 读工件
    # 真值位姿，模型名由 config/tasks/world_sim.yaml 在 --world sim 时下发；
    # 真机不使用本字段（真机判据仍是 verify_pose.py 的 TCP 位姿）。
    sim_workpiece_model: str = "shielding_module"
    # 仿真夹爪预压行程（米/侧）。0 = 用 EFG 开度的线性映射（21.285 mm 满行程）。
    sim_gripper_stroke_m: float = 0.0


def load_config(path: str) -> Config:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    g = lambda key: np.array(raw[key], dtype=float)  # noqa: E731
    q = lambda key: tuple(float(v) for v in raw[key])  # noqa: E731
    axis = g("insert") - g("pre_insert")
    verification = dict(raw.get("verification", {}) or {})
    cfg = Config(
        pre_grasp=g("pre_grasp"),
        grasp=g("grasp"),
        grasp_q=q("grasp_q"),
        transit=g("transit"),
        staging=g("staging"),
        pre_insert=g("pre_insert"),
        insert=g("insert"),
        insert_q=q("insert_q"),
        pre_insert_q=q("pre_insert_q"),
        baseline=g("baseline"),
        baseline_q=q("baseline_q"),
        advance_axis=axis / np.linalg.norm(axis),
        grip_width_mm=float(raw.get("grip_width_mm", 52.2)),
        grip_current=float(raw.get("grip_current", 0.1)),
        workpiece_measurement=str(raw.get("workpiece_measurement")
                                  or "runtime/measurements/workpiece_latest.json"),
        workpiece_frame_offset=list(raw.get("workpiece_frame_offset") or []),
        workpiece_reference=dict(raw.get("workpiece_reference") or {}),
        workpiece_bbox=list(raw.get("workpiece_bbox") or []),
        lift_total_mm=float(raw.get("lift_total_mm", 155.0)),
        safe_clear_mm=float(raw.get("safe_clear_mm", 150.0)),
        velocity_scale=float(raw.get("velocity_scale", 0.02)),
        assembly_velocity_scale=float(raw.get("assembly_velocity_scale", 0.01)),
        time_scale=float(raw.get("time_scale", 2.0)),
        dwell_sec=float(raw.get("dwell_sec", 1.0)),
        goal_tolerance=float(raw.get("goal_tolerance", 0.002)),
        goal_joint_tolerance=float(raw.get("goal_joint_tolerance", 0.02)),
        max_chunks=int(raw.get("max_chunks", 7)),
        phases=list(raw.get("phases", [])),
        speeds=dict(raw.get("phase_speeds", {}) or {}),
        mode=str(raw.get("mode", "safe")),
        scope=str(raw.get("scope", "")),
        world=str(raw.get("world", "real")),
        gripper_max_opening_mm=float(raw.get("gripper_max_opening_mm", 50.0)),
        gripper_span_min_mm=float(raw.get("gripper_span_min_mm", 204.0)),
        gripper_span_max_mm=float(raw.get("gripper_span_max_mm", 246.57)),
        action=str(raw.get("action", "/arm_controller/follow_joint_trajectory")),
        observe=dict(raw.get("observe", {}) or {}),
        observe_settle_sec=float(raw.get("observe_settle_sec", 10.0)),
        nbv_distance_m=float(raw.get("nbv_distance_m", 0.65)),
        nbv_elevation_deg=float(raw.get("nbv_elevation_deg", 25.0)),
        camera_fast_during_assembly=bool(raw.get("camera_fast_during_assembly", True)),
        observe_checks=dict(raw.get("observe_checks", {}) or {}),
        process_images=bool(raw.get("process_images", True)),
        verify=bool(verification.get("enabled", True)),
        verify_pos_mm=float(verification.get("pos_mm", 3.0)),
        verify_rot_rad=float(verification.get("rot_rad", 0.01)),
        sim_workpiece_model=str(raw.get("sim_workpiece_model", "shielding_module")),
        sim_gripper_stroke_m=float(raw.get("sim_gripper_stroke_m", 0.0)),
    )
    validate(cfg)
    return cfg


# Hard limits.  "Proven" means the number has actually been executed on the real
# cell (config/pick_place_real.yaml, 2026-10-22): transit/baseline 0.08,
# pre_insert/withdraw 0.03, staging 0.05.  A validator must never be stricter
# than a configuration that already ran, otherwise it rejects proven practice;
# equally it must not silently allow more.
MAX_APPROACH_VELOCITY = {"safe": 0.08, "efficient": 0.12}
MAX_ASSEMBLY_VELOCITY = {"safe": 0.03, "efficient": 0.03}
# Inside the slot the per-side clearance is about 1 mm.  Two things were proven
# necessary there and are enforced regardless of mode: uniform slow speed (a fast
# cruise with a slow tail binds) and no speed-up in the "efficient" profile.
# "d2b_entrance_tilted" is deliberately NOT here: it is the approach to the slot
# mouth with an empty gripper and ran at 0.03 m/s on the real cell.  The phases
# below are the ones where the part is already near the fixture walls.
TIGHT_CLEARANCE_PHASES = {"insert", "straighten", "d2c_straighten",
                          "d4_tilt_for_removal", "d5_extract"}
# Phases whose geometry is tighter than the collision mesh, so the planner must
# not check collisions there: the tight set plus the empty-gripper entry through
# the slot mouth (it passes the same ~1 mm gap but keeps the proven 0.03 m/s).
NO_COLLISION_CHECK_PHASES = TIGHT_CLEARANCE_PHASES | {"d2b_entrance_tilted"}
MAX_TIGHT_CLEARANCE_VELOCITY = 0.005
MIN_SAFE_CLEAR_MM = 100.0         # axial clearance before any rotation
MAX_GOAL_TOLERANCE = 0.005        # rad; loose tolerances fake success
# Joint-goal tolerance is what MoveIt accepts as "at the goal" for every planned
# phase.  The real cell uses 0.02 rad and the simulation overlay tightens it to
# 0.002; anything looser than this makes "planned" meaningless.
MAX_GOAL_JOINT_TOLERANCE = 0.05
# Upper bound for per-phase chunk counts; a typo used to slip through validation.
MAX_CHUNKS = 12
# Phases that physically enter the part or the fixture: their speed is checked
# against the assembly cap no matter what the profile file says.
ASSEMBLY_PHASES = {"grasp", "pre_insert", "insert", "straighten", "withdraw",
                   "d2a_pre_insert", "d2b_entrance_tilted", "d2c_straighten",
                   "d4_tilt_for_removal", "d5_extract", "d3c_pull_test",
                   "d3e_return", "d7b_place_grasp"}


def _is_tight(phase: str) -> bool:
    if phase in TIGHT_CLEARANCE_PHASES:
        return True
    return any(phase.startswith(prefix) for prefix in TIGHT_CLEARANCE_PHASES)


def chunk_count(total_points: int, requested: int) -> int:
    """Largest useful chunk count for a trajectory with ``total_points`` points.

    A chunk is a slice ``points[start:end + 1]``, so ``n`` chunks need at least
    ``n + 1`` points to avoid duplicate or single-point segments.  The old code
    used the requested count blindly: with 3 points and 6 chunks it produced
    ``[1, 2, 1, 1, 2, 1]`` points per chunk -- repeated motion the arm had already
    made (the "twitch" seen on 2026-10-2x) and 1-point trajectories the controller
    can reject.  Returns 0 when there is nothing to execute.
    """
    if total_points < 2 or requested < 1:
        return 0
    return min(int(requested), total_points - 1)


def _no_collision_check(phase: str) -> bool:
    """Tight geometry: the slot walls guide the motion, not the planner."""
    if phase in NO_COLLISION_CHECK_PHASES:
        return True
    return any(phase.startswith(prefix) for prefix in NO_COLLISION_CHECK_PHASES)


def phase_speed_problems(speeds: dict, mode: str) -> list[str]:
    """Check every per-phase speed against the cap for its class and mode.

    Shared by the pick_place and the unload loaders: the unload path used to
    validate only the two global scalars, so a profile file could put any speed
    inside the slot without anything noticing.
    """
    problems: list[str] = []
    approach_cap = MAX_APPROACH_VELOCITY.get(mode, MAX_APPROACH_VELOCITY["safe"])
    assembly_cap = MAX_ASSEMBLY_VELOCITY.get(mode, MAX_ASSEMBLY_VELOCITY["safe"])
    for name, raw in (speeds or {}).items():
        if not isinstance(raw, dict):
            continue
        if "chunks" in raw:
            try:
                requested = int(raw["chunks"])
            except (TypeError, ValueError):
                problems.append(f"phase {name}.chunks {raw['chunks']!r} is not an integer")
            else:
                if requested < 1:
                    # chunks <= 0 used to execute nothing and report success.
                    problems.append(f"phase {name}.chunks {requested} < 1")
                elif requested > MAX_CHUNKS:
                    problems.append(f"phase {name}.chunks {requested} > {MAX_CHUNKS}")
        if _is_tight(str(name)):
            cap = MAX_TIGHT_CLEARANCE_VELOCITY
            if float(raw.get("slow_tail_mm", 0.0)) > 0.0:
                problems.append(
                    f"phase {name}: slow_tail_mm inside the tight slot clearance "
                    "(uniform slow speed only)")
        elif str(name) in ASSEMBLY_PHASES:
            cap = assembly_cap
        else:
            cap = approach_cap
        klass = ("tight" if _is_tight(str(name))
                 else "assembly" if str(name) in ASSEMBLY_PHASES else mode)
        for key in ("velocity", "slow_velocity"):
            if key in raw and float(raw[key]) > cap:
                problems.append(f"phase {name}.{key} {raw[key]} > {cap} ({klass})")
    return problems


def validate(cfg: Config) -> None:
    problems: list[str] = []
    if cfg.mode not in MAX_APPROACH_VELOCITY:
        problems.append(f"mode {cfg.mode!r} is not one of {sorted(MAX_APPROACH_VELOCITY)}")
        raise SystemExit("config rejected:\n  " + "\n  ".join(problems))
    if cfg.velocity_scale > MAX_APPROACH_VELOCITY[cfg.mode]:
        problems.append(f"velocity_scale {cfg.velocity_scale} > "
                        f"{MAX_APPROACH_VELOCITY[cfg.mode]} ({cfg.mode})")
    if cfg.assembly_velocity_scale > MAX_ASSEMBLY_VELOCITY[cfg.mode]:
        problems.append(f"assembly_velocity_scale {cfg.assembly_velocity_scale} > "
                        f"{MAX_ASSEMBLY_VELOCITY[cfg.mode]} ({cfg.mode})")
    problems.extend(phase_speed_problems(cfg.speeds, cfg.mode))
    if cfg.safe_clear_mm < MIN_SAFE_CLEAR_MM:
        problems.append(f"safe_clear_mm {cfg.safe_clear_mm} < {MIN_SAFE_CLEAR_MM}")
    if cfg.goal_tolerance > MAX_GOAL_TOLERANCE:
        problems.append(f"goal_tolerance {cfg.goal_tolerance} > {MAX_GOAL_TOLERANCE}")
    if not 0.0 < cfg.goal_joint_tolerance <= MAX_GOAL_JOINT_TOLERANCE:
        problems.append(f"goal_joint_tolerance {cfg.goal_joint_tolerance} not in "
                        f"(0, {MAX_GOAL_JOINT_TOLERANCE}]")
    stroke = float(np.linalg.norm(cfg.insert - cfg.pre_insert)) * 1000.0
    if stroke < 100.0:
        problems.append(f"insert stroke {stroke:.1f} mm looks wrong")
    if problems:
        raise SystemExit("config rejected:\n  " + "\n  ".join(problems))


# ============================================================================
# 仿真阶段验收：按"工件跟着夹爪走 + 进插槽"判定（真机仍按 verify_pose.py 的 TCP 判据）
# ============================================================================
# 当前问题：仿真里没有 /tcp_pose_broadcaster/pose（那是真机驱动一侧的控制器），
# 所以 verify_pose.py 在 Gazebo 下永远返回 NO_POSE。日志里 verified=false 既不代表
# 任务失败，也看不出工件到底有没有被带走。
#
# 仿真的物理事实：世界文件里没有 DetachableJoint，"夹住"是
# scripts/sim_carry_workpiece.py 把工件位姿按 20 Hz 跟随到工具位姿实现的。因此唯一
# 能证明"任务做成"的量是 **工件自己的世界位姿**：
#   A) grasp 结束时（grasp_confirmed 发布后）记下 工具→工件 的相对位姿 = 夹持关系，
#      以及工件当时的世界位姿 = 初始位姿；
#   B) 之后的阶段结束时，相对位姿必须还是那个夹持关系，且工件确实被搬走了；
#   C) insert / straighten 结束时，工件包围盒中心必须落在插槽体积内。
#
# 数据来源全部是仓库已有产物，不硬编码任何坐标：
#   工件真值位姿  gz model -m <model> --pose（模型名由 config/tasks/world_sim.yaml 下发，
#                 解析用 package 里的权威解析器 cs625_task_orchestrator.p7_gazebo_pose）
#   工具位姿      TF world -> my_end_effector_link（仿真里由 robot_state_publisher 提供）
#   插槽体积      config/cs625_real_cell_scene.yaml（生成物，权威）
#                 fixture.pose_world + fixture.bbox_min_m/bbox_max_m → 世界 AABB → 外扩容差
SIM_SCENE_CONFIG = (Path(__file__).resolve().parent.parent
                    / "config" / "cs625_real_cell_scene.yaml")
SIM_WORKPIECE_MODEL = "shielding_module"   # 配置缺失时的兜底；正常由 world_sim.yaml 下发
SIM_FOLLOW_MAX_MM = 10.0     # 抓取后 工具→工件 相对位姿允许漂移 10 mm
SIM_FOLLOW_MAX_DEG = 3.0     # 同上，姿态允许 3°
SIM_MIN_TRAVEL_M = 0.30      # 工件世界位移下限：证明它真被搬走，而不是"手臂空跑"
SIM_TRAVEL_RATIO = 0.90      # 计划搬运距离不到 0.30 m 的阶段（只有 lift）改用 90% 计划距离
SIM_SLOT_TOL_M = 0.030       # 插槽 AABB 外扩容差（生成物的 bbox 是网格极值，本身就偏保守）
# 判据起作用的阶段（load 流程的搬运链）。unload 的 D 系列不在此列，行为与以前一致。
SIM_FOLLOW_PHASES = ("lift", "transit", "staging", "pre_insert", "insert", "straighten")
SIM_SLOT_PHASES = ("insert", "straighten")
# sim_base 每次启动给 Gazebo 生成唯一分区名并写在这里；`gz model` 必须用同一个
# 分区才连得上那台服务器，否则调用静默超时、判据退化成"无法测量"。
GZ_PARTITION_FILE = (Path(__file__).resolve().parent.parent
                     / "runtime" / "gz_partition.txt")


def sim_gz_environment() -> dict:
    """给 `gz model` 子进程用的环境：带上本次仿真的 Gazebo 分区名。

    先看环境变量（有人手工 export 就照用），再看 sim_base 写下的运行时文件。
    两者都没有时原样返回——这时 `gz model` 会超时，判据按"无法测量"记录，
    不会假装通过。
    """
    env = dict(os.environ)
    partition = (os.environ.get("IGN_PARTITION", "").strip()
                 or os.environ.get("GZ_PARTITION", "").strip())
    if not partition:
        try:
            partition = GZ_PARTITION_FILE.read_text(encoding="utf-8").strip()
        except OSError:
            partition = ""
    if partition:
        env["IGN_PARTITION"] = partition
        env["GZ_PARTITION"] = partition
    return env


def relative_pose(tool_pose: tuple, object_pose: tuple) -> tuple[np.ndarray, np.ndarray]:
    """工件位姿在**工具坐标系**里的表示（位置 + 四元数）。

    物理含义：夹住之后这个量应当恒定——真机靠夹爪摩擦夹紧，仿真靠
    sim_carry_workpiece 每 50 ms 把工件摆到同一个相对位姿上。它变了就说明
    "工件没跟着走"。
    """
    tool_xyz, tool_quat = tool_pose
    object_xyz, object_quat = object_pose
    tool_rot = R.from_quat(tool_quat)
    rel_rot = tool_rot.inv() * R.from_quat(object_quat)
    rel_pos = tool_rot.inv().apply(np.asarray(object_xyz, float)
                                   - np.asarray(tool_xyz, float))
    return rel_pos, rel_rot.as_quat()


def relative_pose_delta(a: tuple, b: tuple) -> tuple[float, float]:
    """两帧相对位姿之差：平移 mm + 姿态 deg（姿态差取两四元数夹角，与转向无关）。"""
    mm = float(np.linalg.norm(np.asarray(a[0], float) - np.asarray(b[0], float)) * 1000.0)
    angle = (R.from_quat(a[1]) * R.from_quat(b[1]).inv()).magnitude()
    return mm, float(np.degrees(angle))


def world_aabb(pose_xyz, rotation: R, bbox_min_m, bbox_max_m) -> tuple[np.ndarray, np.ndarray]:
    """模型系 AABB → 世界系 AABB（把 8 个角点变换到世界系后取各轴极值）。

    为什么不能只加平移：夹具只绕 yaw 转 0.0068 rad，但工件抓取时约 -58°、
    插入时接近 180°，姿态完全不可忽略；只有"角点极值"才是真正的轴向包围盒。
    """
    lo = np.asarray(bbox_min_m, dtype=float)
    hi = np.asarray(bbox_max_m, dtype=float)
    corners = np.array([[x, y, z] for x in (lo[0], hi[0])
                        for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    world = rotation.apply(corners) + np.asarray(pose_xyz, dtype=float)
    return world.min(axis=0), world.max(axis=0)


def load_scene_config(path: Path = SIM_SCENE_CONFIG) -> dict:
    """读生成场景配置里的 ros__parameters（生成物，权威，勿手改）。"""
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError(f"{path} is not a mapping")
    for value in raw.values():
        if isinstance(value, dict) and "ros__parameters" in value:
            return dict(value["ros__parameters"])
    raise ValueError(f"{path} has no ros__parameters block")


def slot_world_aabb(scene: dict,
                    tol_m: float = SIM_SLOT_TOL_M) -> tuple[np.ndarray, np.ndarray]:
    """插槽体积：fixture.pose_world + bbox_min_m/bbox_max_m → 世界 AABB，再按 tol 外扩。"""
    fixture = dict(scene.get("fixture") or {})
    pose = fixture.get("pose_world")
    bbox_min = fixture.get("bbox_min_m")
    bbox_max = fixture.get("bbox_max_m")
    if not pose or not bbox_min or not bbox_max:
        raise ValueError("fixture.pose_world / bbox_min_m / bbox_max_m missing")
    rotation = R.from_euler("xyz", [float(v) for v in pose[3:6]])
    lo, hi = world_aabb([float(v) for v in pose[:3]], rotation, bbox_min, bbox_max)
    return lo - float(tol_m), hi + float(tol_m)


def object_bbox_centre_local(scene: dict) -> np.ndarray:
    """工件包围盒中心在**模型系**里的位置（bbox_min/max 的中点）。

    用中点而不是 CAD origin：生成物已经记录 module.bbox_centre_world_m，
    并且注明它与实测 CAD 中心差 22.8 mm，说明"包围盒中心"才是可复现的量。
    """
    module = dict(scene.get("module") or {})
    bbox_min = module.get("bbox_min_m")
    bbox_max = module.get("bbox_max_m")
    if not bbox_min or not bbox_max:
        raise ValueError("module.bbox_min_m / bbox_max_m missing")
    return (np.asarray(bbox_min, float) + np.asarray(bbox_max, float)) / 2.0


def planned_carry_m(name: str, cfg: Config) -> float:
    """该阶段计划把工件搬多远：抓取点 TCP → 本阶段终点 TCP 的距离。

    用途：位移判据要有绝对下限（否则"手臂空跑"也能通过），但这个下限必须小于
    阶段本身能走的距离。straighten 的终点与 insert 同位置（只转姿态），所以两者
    计划距离相同；lift 只有 lift_total_mm。
    """
    targets = {"transit": cfg.transit, "staging": cfg.staging,
               "pre_insert": cfg.pre_insert, "insert": cfg.insert,
               "straighten": cfg.insert}
    if name == "lift":
        return float(cfg.lift_total_mm) / 1000.0
    target = targets.get(name)
    if target is None:
        return 0.0
    return float(np.linalg.norm(np.asarray(target, float) - np.asarray(cfg.grasp, float)))


class Runner(Node):
    def __init__(self) -> None:
        super().__init__("pick_and_place_runner")
        self._joints: JointState | None = None
        self._joints_at: float = 0.0
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        # Observation checks must run on a frame taken AFTER the arm moved.  The
        # camera streams ~1 fps on this cell and the old code slept 1.0 s and then
        # took whatever was in the buffer, which can be the pre-move view.  The
        # adapter publishes with sensor-data QoS, so this subscription must too.
        self._camera_stamp_ns: int = 0
        # 注意：launch 里 launch_sensor_adapter:=false 时"归一化话题"根本不存在，
        # 这时必须退回相机原始话题，否则这里的帧时间戳永远不更新，
        # 每个观测位都会报 "no camera frame newer than the motion within 10 s"。
        for topic in ("/sensors/camera/color/image", "/camera/color/image_raw"):
            self.create_subscription(Image, topic, self._on_camera_frame,
                                     qos_profile_sensor_data)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._ik = self.create_client(GetPositionIK, "/compute_ik")
        self._plan = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self._cart = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        # Bounded wait with the missing names printed.  Before 2026-10-26 this was
        # ``while not client.wait_for_service(): pass``: with MoveIt not running the
        # runner printed nothing at all and never timed out, which is exactly what
        # "the task will not start" looked like from the operator's side.
        services = (
            ("/compute_ik", self._ik),
            ("/plan_kinematic_path", self._plan),
            ("/compute_cartesian_path", self._cart),
        )
        missing: list[str] = []
        deadline = time.time() + MOVEIT_SERVICE_TIMEOUT_S
        for name, client in services:
            while not client.service_is_ready() and time.time() < deadline:
                time.sleep(0.25)
            if not client.service_is_ready():
                missing.append(name)
        if missing:
            raise SystemExit(
                "MoveIt services are not available: " + ", ".join(missing)
                + f"\n  waited {MOVEIT_SERVICE_TIMEOUT_S:.0f} s."
                "\n  Most likely: move_group was not started (ros2 launch cs625_bringup "
                "real_moveit.launch.py ...), or this shell uses a different ROS_DOMAIN_ID.\n"
                "  Nothing was commanded; fix the stack and re-run.")
        # Plan-only needs forward kinematics for a *simulated* joint state: the
        # arm never moves, so the next phase has to be planned from where the
        # previous phase WOULD have ended.
        self.action_name: str = "/arm_controller/follow_joint_trajectory"
        self.world: str = "real"
        # 仿真验收用的状态：工件模型名（由配置下发）、最近一次测量失败的原因、
        # 抓取基线（A）。真机路径不读这三个字段。
        self.sim_model: str = SIM_WORKPIECE_MODEL
        self.sim_error: str = ""
        self.grasp_baseline: dict | None = None
        # 仿真夹爪预压行程（来自 config/tasks/world_sim.yaml 的 sim_gripper_stroke_m）
        self.sim_gripper_stroke_m: float = 0.0
        self.gripper_geometry: dict = {"max": 50.0, "min": 204.0, "max_span": 246.57}
        # rad; how far a chunk endpoint may sit from the plan before we call it a
        # block.  0.05 rad is ~3 deg at the wrist, a few millimetres at the tool.
        self.chunk_endpoint_error: float = 0.05
        self._gripper_pub = None
        self._fk = self.create_client(GetPositionFK, "/compute_fk")
        self.joint_limits: dict[str, tuple[float, float]] = {}
        self._load_joint_limits()
        self._fk_ready = self._fk.wait_for_service(timeout_sec=5.0)
        self.sim_q: list[float] | None = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._grasp_event = self.create_publisher(String, "/cs625/grasp_state_event", 10)
        self._box_pose = self.create_publisher(PoseStamped, "/box_target_pose", latched)
        # 仿真附着状态：scripts/sim_carry_workpiece.py 收到 grasp/release 事件后，
        # 通过 Gazebo 的 DetachableJoint 真正焊住/松开工件，并在这里广播结果。
        # 这个事件是异步的（gz CLI 一次约 0.4 s）：不等它确认就继续抬臂，关节会
        # 在抬臂中途才建立，夹持关系就带着那一段位移，后面的跟随判据会假失败。
        self.carry_state: str = ""
        self.create_subscription(String, "/cs625/carry_state", self._on_carry_state, 10)

    def _on_carry_state(self, message: String) -> None:
        self.carry_state = (message.data or "").strip()

    def wait_for_carry_state(self, wanted: str, timeout_sec: float = 3.0) -> bool:
        """等 carry 节点确认 attached / detached（只在仿真里调用）。"""
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if self.carry_state == wanted:
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        self.get_logger().warn(
            f"等待 /cs625/carry_state == {wanted} 超时（当前 {self.carry_state!r}）："
            "仿真里工件可能没被真正夹住，跟随判据会如实失败")
        return False

    # ----------------------------------------------------------- primitives
    def _load_joint_limits(self) -> None:
        """Position limits straight from /robot_description (latched URDF)."""
        try:
            from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
            import xml.etree.ElementTree as ET

            got: dict = {}
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(
                String, "/robot_description",
                lambda msg: got.setdefault("xml", msg.data), latched)
            deadline = time.time() + 5.0
            while "xml" not in got and time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
            if "xml" not in got:
                return
            root = ET.fromstring(got["xml"])
            for joint in root.findall(".//joint"):
                name = joint.get("name")
                limit = joint.find("limit")
                if name in JOINTS and limit is not None:
                    lower = float(limit.get("lower", "nan"))
                    upper = float(limit.get("upper", "nan"))
                    if not (np.isnan(lower) or np.isnan(upper)):
                        self.joint_limits[name] = (lower, upper)
        except Exception as exc:  # never block a run on diagnostics
            self.get_logger().warn(f"joint limits unavailable: {exc}")

    def _within_limits(self, solution: list[float]) -> bool:
        if not self.joint_limits:
            return True
        for name, value in zip(JOINTS, solution):
            bounds = self.joint_limits.get(name)
            if bounds and not (bounds[0] - 1e-6 <= value <= bounds[1] + 1e-6):
                return False
        return True

    def _on_camera_frame(self, message) -> None:
        stamp = message.header.stamp
        ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        if ns > self._camera_stamp_ns:
            self._camera_stamp_ns = ns

    def camera_stamp_ns(self) -> int:
        return self._camera_stamp_ns

    def wait_camera_frame_after(self, stamp_ns: int, timeout: float = 10.0) -> bool:
        """Block until a frame stamped later than ``stamp_ns`` arrives.

        Comparing message stamps against each other (not against the wall clock)
        keeps this correct regardless of clock offsets.  At the cell's ~1 fps a
        fresh frame arrives within ~1-2 s; the 10 s bound only fires when the
        camera is dead.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._camera_stamp_ns > stamp_ns:
                return True
        return False

    def _on_joints(self, msg: JointState) -> None:
        self._joints = msg
        self._joints_at = time.time()

    def fresh_joints(self, timeout: float = 3.0) -> JointState:
        """A /joint_states sample that arrived after this call started.

        Nothing spins this node while a trajectory is being executed (the
        executor runs as a separate process), so the cached state goes stale the
        moment the arm starts moving.  Comparing a chunk start against that stale
        cache made the runner refuse chunks 2..N with "starts far from the current
        state" even though the arm was exactly where it should be.
        """
        started = time.time()
        deadline = started + timeout
        self._joints_at = 0.0
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._joints is not None and self._joints_at >= started:
                break
        return self.joints()

    def arm_positions(self) -> list[float]:
        """The six arm joints in JOINTS order (see arm_seed_from_state)."""
        state = self.joints()
        return arm_seed_from_state(state.name, state.position)

    def joints(self) -> JointState:
        deadline = time.time() + 10
        while self._joints is None and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
        if self._joints is None:
            raise RuntimeError("no /joint_states")
        return self._joints

    def tcp(self) -> np.ndarray:
        """Read the newest TCP pose from the broadcaster topic.

        TF lookups can return a transform from before the last motion, which made
        a successful move look like "no progress".  The topic always carries the
        latest sample, so it is the reliable source for progress checks.
        """
        got: dict = {}
        sub = self.create_subscription(
            PoseStamped, "/tcp_pose_broadcaster/pose",
            lambda msg: got.__setitem__("m", msg), 10,
        )
        deadline = time.time() + 3.0
        while "m" not in got and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(sub)
        msg = got.get("m")
        if msg is not None:
            position = msg.pose.position
            return np.array([position.x, position.y, position.z])
        # Gazebo has no tcp_pose_broadcaster (that controller belongs to the real
        # driver), so the same number comes out of TF: world -> the planning tip.
        for target in ("my_end_effector_link", "tool0"):
            try:
                tf = self._tf_buffer.lookup_transform(
                    "world", target, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=2.0))
            except Exception:  # noqa: BLE001 - try the next candidate
                continue
            translation = tf.transform.translation
            return np.array([translation.x, translation.y, translation.z])
        raise RuntimeError(
            "no TCP pose: neither /tcp_pose_broadcaster/pose nor TF world->"
            "my_end_effector_link is available")

    def gripper(self, command: str, value: float | None = None,
                current: float | None = None) -> dict:
        """Drive the gripper: the simulated controller in sim, the real bridge in real.

        A simulation run must never reach the real HIBTOT bridge: on this cell the
        bridge is a Windows PowerShell script, so a sim run would quietly actuate
        the physical gripper while the arm moved in Gazebo.
        """
        if self.world == "sim":
            return self._sim_gripper(command, value)
        args = ["powershell.exe", "-NoProfile", "-ExecutionPolicy", "Bypass",
                "-File", GRIPPER_PS1, "-Cmd", command]
        if value is not None:
            args += ["-Value", str(value)]
        if current is not None:
            args += ["-Current", str(current)]
        for attempt in range(3):
            out = subprocess.run(args, capture_output=True, timeout=30)
            stdout = (out.stdout or b"").decode("utf-8", "replace")
            stderr = (out.stderr or b"").decode("utf-8", "replace")
            lines = [l for l in stdout.strip().splitlines() if l.strip()]
            if lines:
                try:
                    data = json.loads(lines[-1])
                    if data.get("ok"):
                        return data
                except json.JSONDecodeError:
                    node = out  # keep the raw output for the error message
            time.sleep(1.0)
        raise RuntimeError(f"gripper {command} failed")

    def _sim_gripper(self, command: str, value: float | None) -> dict:
        """Simulated gripper: publish the mapped finger position, no hardware."""
        if self._gripper_pub is None:
            from std_msgs.msg import Float64MultiArray

            self._gripper_pub = self.create_publisher(
                Float64MultiArray, "/gripper_controller/commands", 10)
        from std_msgs.msg import Float64MultiArray

        if command == "grip" and value is not None:
            # 仿真里外撑夹爪是刚体：满行程（21.285 mm/侧）会把 7~16 mm 干涉量全部
            # 变成侧向推力，把工件推开。用配置里的"刚好压住"行程（默认 11.5 mm ≈
            # 1.5 mm 预压）；真机路径不走这个函数（真机是 EFG 的 PowerShell 桥）。
            stroke = float(self.sim_gripper_stroke_m) if self.sim_gripper_stroke_m else None
            if stroke:
                position = stroke
            else:
                position = finger_position_m(
                    value, self.gripper_geometry["max"], self.gripper_geometry["min"],
                    self.gripper_geometry["max_span"])
        elif command == "open":
            position = finger_position_m(
                self.gripper_geometry["max"] * 0.8, self.gripper_geometry["max"],
                self.gripper_geometry["min"], self.gripper_geometry["max_span"])
        elif command == "close":
            position = 0.0
        else:  # init
            return {"ok": True, "init": 5, "position_mm": None, "sim": True}
        # 两个手指关节各一个位置命令（外撑是左右对称的同一行程）
        message = Float64MultiArray(data=[float(position), float(position)])
        self._gripper_pub.publish(message)
        self.get_logger().info(
            f"[sim gripper] {command} value={value} -> finger {position * 1000:.2f} mm")
        return {"ok": True, "init": 5, "position_mm": position * 1000.0, "sim": True}

    def ensure_gripper_initialized(self) -> bool:
        """Always run the gripper calibration before a real run.

        The old version skipped init when the bridge reported ``init == 5`` to
        save ~10 s.  That flag survives an E-stop / power event and does not prove
        the jaws are referenced.  On 2026-10-26 the skip path was taken, the grip
        ran on an unverified opening, and the arm then drove into the workpiece.
        Ten seconds of calibration per run is cheap insurance, so it always runs.
        """
        status = self.gripper("status")
        if status.get("init") != 5:
            self.get_logger().info(
                f"gripper init flag {status.get('init')} != 5; calibrating")
        self.gripper("init")
        return self.wait_gripper_init()

    def wait_gripper_init(self, timeout_sec: float = 30.0) -> bool:
        """init takes ~10 s; issuing grip before it finishes leaves the jaws mid-way."""
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            status = self.gripper("status")
            if status.get("init") == 5:
                return True
            time.sleep(0.7)
        return False

    def grip_and_wait(self, width_mm: float, current: float = 0.1,
                      timeout_sec: float = 20.0) -> bool:
        """Wait until the jaws settle at the requested width (or stall on contact)."""
        start_position = self.gripper("status").get("position_mm")
        for attempt in range(2):
            self.gripper("grip", width_mm, current)
            deadline = time.time() + timeout_sec / 2
            moved = False
            while time.time() < deadline:
                status = self.gripper("status")
                position = status.get("position_mm")
                if position is not None and start_position is not None:
                    if abs(position - start_position) > 1.0:
                        moved = True
                if status.get("state") == 0 and position is not None and abs(position - width_mm) <= 1.0:
                    return True
                time.sleep(0.6)
            if not moved:
                self.get_logger().warn(
                    f"grip {attempt + 1} produced no motion; re-issuing after settle"
                )
                time.sleep(1.5)
        return False

    # Known-good real configurations, measured on this cell.  IK is seeded from
    # these as well: a single seed can land on a wildly different elbow/wrist
    # branch (seen: shoulder_pan 199 deg, wrist_1 -225 deg), and the planner then
    # rejects the goal.  Using several seeds and keeping the solution closest to
    # the current state keeps the chained plan on a physically sensible branch.
    FALLBACK_SEEDS = (
        (0.7375, -1.0201, -2.5573, 0.2778, 1.6366, 1.5576),   # coarse observation pose
        (0.5597, -0.8861, -2.6480, 0.0049, 1.8827, 1.4482),   # fine observation pose
    )
    # A solution further than this from where the arm actually is means a
    # different branch: refuse instead of planning an arm-flip.
    MAX_IK_BRANCH_JUMP = 2.6   # rad; beyond this the solver changed branch

    def _ik_once(self, xyz, quat, seed: list[float]) -> list[float] | None:
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (float(v) for v in xyz)
        (pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w) = (float(v) for v in quat)
        req = GetPositionIK.Request()
        req.ik_request.group_name = "cs625_arm"
        req.ik_request.robot_state.joint_state = JointState(
            name=list(JOINTS), position=[float(v) for v in seed])
        req.ik_request.pose_stamped.header.frame_id = "world"
        req.ik_request.pose_stamped.pose = pose
        req.ik_request.timeout.sec = 2
        future = self._ik.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10)
        result = future.result()
        if result is None or result.error_code.val != 1:
            return None
        values = dict(zip(result.solution.joint_state.name,
                          result.solution.joint_state.position))
        return [float(values[name]) for name in JOINTS]

    def ik(self, xyz, quat, seed: list[float] | None = None) -> JointState | None:
        base = ([float(v) for v in seed] if seed is not None
                else self.arm_positions())
        best: tuple[float, list[float]] | None = None
        for candidate in (base, *self.FALLBACK_SEEDS):
            solution = self._ik_once(xyz, quat, candidate)
            if solution is None:
                continue
            jump = max(abs(a - b) for a, b in zip(solution, base))
            if best is None or jump < best[0]:
                best = (jump, solution)
        if best is None:
            return None
        if not self._within_limits(best[1]):
            self.get_logger().warn("ik: solution outside the URDF joint limits; refused")
            return None
        if best[0] > self.MAX_IK_BRANCH_JUMP:
            print(f"ik: nearest solution is {best[0]:.2f} rad from the seed "
                  f"(limit {self.MAX_IK_BRANCH_JUMP}); refusing a branch flip",
                  file=sys.stderr)
            self.get_logger().warn(
                f"ik: nearest solution is {best[0]:.2f} rad from the current state "
                f"(limit {self.MAX_IK_BRANCH_JUMP}); refusing a branch flip")
            return None
        return JointState(name=list(JOINTS), position=best[1])

    def plan_joint(self, goal: JointState, speed: float,
                   start_positions: list[float] | None = None) -> list[dict] | None:
        constraints = Constraints()
        solution = dict(zip(goal.name, goal.position))
        for name in JOINTS:
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = float(solution[name])
            joint.tolerance_above = self.goal_joint_tolerance
            joint.tolerance_below = self.goal_joint_tolerance
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        req = GetMotionPlan.Request()
        motion = req.motion_plan_request
        motion.group_name = "cs625_arm"
        # leave start_state empty for the same staleness reason as above
        if start_positions is not None:
            motion.start_state = RobotState()
            motion.start_state.joint_state = JointState(
                name=list(JOINTS), position=[float(v) for v in start_positions])
            motion.start_state.is_diff = False
        motion.goal_constraints.append(constraints)
        motion.num_planning_attempts = 6
        motion.allowed_planning_time = 5.0
        motion.max_velocity_scaling_factor = speed
        motion.max_acceleration_scaling_factor = speed
        future = self._plan.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=20)
        result = future.result()
        if result is None or result.motion_plan_response.error_code.val != 1:
            code = (result.motion_plan_response.error_code.val
                    if result is not None else "None")
            self.get_logger().warn(
                f"plan_joint failed: error_code={code} "
                f"start={np.round(start_positions, 3).tolist() if start_positions else 'current'} "
                f"goal={np.round([float(solution[n]) for n in JOINTS], 3).tolist()}")
            return None
        traj = result.motion_plan_response.trajectory.joint_trajectory
        return [
            {"q": list(point.positions),
             "t": point.time_from_start.sec + point.time_from_start.nanosec * 1e-9}
            for point in traj.points
        ]

    def plan_cartesian(self, xyz, quat, speed: float,
                       start_positions: list[float] | None = None,
                       avoid_collisions: bool = True) -> list[dict] | None:
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (float(v) for v in xyz)
        (pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w) = (float(v) for v in quat)
        req = GetCartesianPath.Request()
        req.header.frame_id = "world"
        # start_state intentionally left empty: MoveIt then uses its own
        # monitored current state (125 Hz).  Injecting our last received
        # joint_states caused the arm to jump back to a stale start pose.
        # Plan-only is the exception: there the arm does not move, so the path
        # must start from the simulated state, not from the measured one.
        if start_positions is not None:
            req.start_state = RobotState()
            req.start_state.joint_state = JointState(
                name=list(JOINTS), position=[float(v) for v in start_positions])
            req.start_state.is_diff = False
        req.group_name = "cs625_arm"
        req.link_name = "my_end_effector_link"
        req.waypoints = [pose]
        req.max_step = 0.003
        req.jump_threshold = 0.0
        req.avoid_collisions = bool(avoid_collisions)
        req.max_velocity_scaling_factor = speed
        req.max_acceleration_scaling_factor = speed
        future = self._cart.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=25)
        result = future.result()
        if result is None or result.fraction < 0.98:
            return None
        traj = result.solution.joint_trajectory
        return [
            {"q": list(point.positions),
             "t": point.time_from_start.sec + point.time_from_start.nanosec * 1e-9}
            for point in traj.points
        ]

    # ------------------------------------------------------------- plan only
    def sim_tcp(self) -> np.ndarray | None:
        """TCP position for the simulated (planned-but-not-executed) joint state."""
        if self.sim_q is None or not self._fk_ready:
            return None
        req = GetPositionFK.Request()
        req.header.frame_id = "world"
        req.fk_link_names = ["my_end_effector_link"]
        req.robot_state.joint_state = JointState(
            name=list(JOINTS), position=[float(v) for v in self.sim_q])
        future = self._fk.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5)
        result = future.result()
        if result is None or result.error_code.val != 1 or not result.pose_stamped:
            return None
        position = result.pose_stamped[0].pose.position
        return np.array([position.x, position.y, position.z])

    def current_tcp(self, plan_only: bool) -> np.ndarray:
        """Where the next phase must start: the simulated state or the real one."""
        if plan_only:
            pose = self.sim_tcp()
            if pose is not None:
                return pose
            self.get_logger().warn(
                "plan-only: /compute_fk unavailable, falling back to the measured TCP")
        return self.tcp()

    def advance_sim(self, points: list[dict]) -> None:
        if points:
            self.sim_q = [float(v) for v in points[-1]["q"]]

    def verify_stage(self, name: str, xyz, quat, cfg: Config) -> tuple[bool, str]:
        """Check the *measured* TCP against a phase target (verify_pose.py)."""
        if not cfg.verify:
            return True, "verification disabled"
        run = subprocess.run(
            ["python3", "-u", "scripts/verify_pose.py",
             "--xyz", *[f"{float(v):.4f}" for v in xyz],
             "--quat", *[f"{float(v):.6f}" for v in quat],
             "--tol-pos", f"{cfg.verify_pos_mm / 1000.0:.4f}",
             "--tol-rot", f"{cfg.verify_rot_rad:.4f}"],
            capture_output=True)
        text = (run.stdout or b"").decode("utf-8", "replace").strip().splitlines()
        tail = text[-1] if text else "verify_pose produced no output"
        return run.returncode == 0, tail

    # --------------------------------------------------- sim workpiece checks
    # 下面四个方法只在 --world sim 下被调用；真机判据仍是 verify_stage()
    # （verify_pose.py 读 /tcp_pose_broadcaster/pose）。仿真没有那个广播器，
    # 所以改成读"工件自己的世界位姿"。
    def sim_object_pose(self) -> tuple[np.ndarray, np.ndarray] | None:
        """工件世界位姿（Gazebo 真值）：gz model -m <model> --pose。

        为什么不信 returncode：`gz model` 连不上仿真时也返回 0，只有输出文本
        说明失败（docs/worklogs/2026-09-19 §32.3）。所以一律看解析结果，解析
        失败就是"无法测量"，并把原因留在 self.sim_error 里。
        """
        command = ["gz", "model", "-m", self.sim_model, "--pose"]
        try:
            run = subprocess.run(command, capture_output=True, text=True, timeout=8,
                                 env=sim_gz_environment())
        except (OSError, subprocess.TimeoutExpired) as exc:
            self.sim_error = f"gz model {type(exc).__name__}: {exc}"
            return None
        # 解析器在仓库自己的 package 里（唯一权威实现：处理 ANSI 颜色码、| 分隔、
        # 要求唯一一条 Pose、模型名不符就 raise）。延迟导入：--dry-run / 真机路径
        # 从不走到这里，缺包也只降级成"无法测量"，不会让真机流程失败。
        try:
            from cs625_task_orchestrator.p7_gazebo_pose import parse_gz_model_pose
        except ImportError as exc:
            self.sim_error = f"pose parser unavailable: {exc}"
            return None
        try:
            pose = parse_gz_model_pose(run.stdout or "", self.sim_model)
        except ValueError as exc:
            raw = " ".join(((run.stdout or "") + (run.stderr or "")).split())[:160]
            self.sim_error = f"gz model: {exc} | output: {raw}"
            return None
        position = pose["position"]
        orientation = pose["orientation"]
        return (np.array([position["x"], position["y"], position["z"]], dtype=float),
                np.array([orientation["x"], orientation["y"], orientation["z"],
                          orientation["w"]], dtype=float))

    def sim_tool_pose(self) -> tuple[np.ndarray, np.ndarray] | None:
        """工具位姿（world -> my_end_effector_link），只走 TF。

        为什么不复用 tcp()：tcp() 只返回位置（运动链用它判断进度），而工件判据
        要完整的相对位姿（含姿态）。回退顺序与 tcp() 相同；真机路径一个字没改。
        """
        for target in ("my_end_effector_link", "tool0"):
            try:
                tf = self._tf_buffer.lookup_transform(
                    "world", target, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=2.0))
            except Exception:  # noqa: BLE001 - try the next candidate
                continue
            translation = tf.transform.translation
            rotation = tf.transform.rotation
            return (np.array([translation.x, translation.y, translation.z]),
                    np.array([rotation.x, rotation.y, rotation.z, rotation.w]))
        self.sim_error = "no tool pose: TF world->my_end_effector_link unavailable"
        return None

    def sim_settled_object_pose(self, timeout_sec: float = 2.0,
                                interval_sec: float = 0.2,
                                stable_mm: float = 1.0):
        """等工件停稳再读数：跟随节点按 20 Hz 追工具位姿，天然滞后一两拍。

        阶段刚结束的那一瞬间读到的是"跟随器落后了多少"，不是"有没有夹住"。
        相邻两次读数相差 < 1 mm 就认为停稳；2 s 还没停就返回最后一次读数。
        """
        previous: np.ndarray | None = None
        deadline = time.time() + timeout_sec
        while True:
            pose = self.sim_object_pose()
            if pose is None:
                return None
            if previous is not None and float(
                    np.linalg.norm(pose[0] - previous) * 1000.0) < stable_mm:
                return pose
            previous = pose[0]
            if time.time() >= deadline:
                return pose
            time.sleep(interval_sec)

    def sim_grasp_baseline(self) -> dict:
        """A) 抓取基线：工具→工件 的相对位姿 + 工件初始世界位姿。

        必须在 grasp_confirmed 之后调用：sim_carry_workpiece 收到该事件才开始
        跟随，此刻的相对位姿就是后续应保持的"夹持关系"。
        """
        tool = self.sim_tool_pose()
        if tool is None:
            return {"criterion": "grasp_baseline", "ok": None,
                    "detail": f"SKIPPED_UNMEASURED: {self.sim_error}"}
        object_pose = self.sim_settled_object_pose()
        if object_pose is None:
            return {"criterion": "grasp_baseline", "ok": None,
                    "detail": f"SKIPPED_UNMEASURED: {self.sim_error}"}
        rel_pos, rel_quat = relative_pose(tool, object_pose)
        self.grasp_baseline = {"rel": (rel_pos, rel_quat),
                               "object": object_pose[0].copy()}
        return {
            "criterion": "grasp_baseline",
            "ok": True,
            "detail": (f"baseline tool->workpiece rel=[{rel_pos[0]:+.3f}, "
                       f"{rel_pos[1]:+.3f}, {rel_pos[2]:+.3f}] m "
                       f"|rel|={float(np.linalg.norm(rel_pos)):.3f} m; "
                       f"workpiece world=[{object_pose[0][0]:.3f}, "
                       f"{object_pose[0][1]:.3f}, {object_pose[0][2]:.3f}]"),
            "rel_m": [round(float(v), 4) for v in rel_pos],
            "object_xyz": [round(float(v), 4) for v in object_pose[0]],
        }

    def sim_workpiece_acceptance(self, name: str, cfg: Config) -> dict | None:
        """B) 跟着走 + C) 进插槽。没有判据的阶段返回 None。

        返回 dict 的 ok 有三种含义，调用方必须区别对待：
          True  → 判据通过；
          False → 判据不过（该阶段记 ok=false，run_task 整体返回非 0）；
          None  → 无法测量（gz 子进程失败 / TF 不可用 / 场景配置缺失）→ 记 skipped
                  并写原因，绝不静默当成通过。
        """
        if name not in SIM_FOLLOW_PHASES:
            return None
        if self.grasp_baseline is None:
            return {"criterion": "follow", "ok": None,
                    "detail": "SKIPPED_UNMEASURED: no grasp baseline recorded"}
        tool = self.sim_tool_pose()
        if tool is None:
            return {"criterion": "follow", "ok": None,
                    "detail": f"SKIPPED_UNMEASURED: {self.sim_error}"}
        object_pose = self.sim_settled_object_pose()
        if object_pose is None:
            return {"criterion": "follow", "ok": None,
                    "detail": f"SKIPPED_UNMEASURED: {self.sim_error}"}

        rel_pos, rel_quat = relative_pose(tool, object_pose)
        follow_mm, follow_deg = relative_pose_delta((rel_pos, rel_quat),
                                                    self.grasp_baseline["rel"])
        travel_m = float(np.linalg.norm(object_pose[0] - self.grasp_baseline["object"]))
        # 位移下限：计划搬运距离 >= 0.30 m 的阶段用 0.30 m 绝对下限；lift 只走
        # lift_total_mm(155 mm)，用 0.30 m 判它必然失败，所以改用"达到计划距离的
        # 90%"——工件留在桌上时位移是 0，同样会被抓到。
        expected_m = planned_carry_m(name, cfg)
        floor_m = (SIM_MIN_TRAVEL_M if expected_m >= SIM_MIN_TRAVEL_M
                   else SIM_TRAVEL_RATIO * expected_m)
        follow_ok = (follow_mm < SIM_FOLLOW_MAX_MM
                     and follow_deg < SIM_FOLLOW_MAX_DEG
                     and travel_m > floor_m)
        parts = [
            f"follow {follow_mm:.1f} mm / {follow_deg:.2f} deg "
            f"(< {SIM_FOLLOW_MAX_MM:.0f} mm / {SIM_FOLLOW_MAX_DEG:.0f} deg)",
            f"travel {travel_m:.3f} m (expected {expected_m:.3f} m, "
            f"floor {floor_m:.3f} m)",
        ]
        numbers = {"follow_mm": round(follow_mm, 2),
                   "follow_deg": round(follow_deg, 3),
                   "travel_m": round(travel_m, 4),
                   "expected_m": round(expected_m, 4)}
        criterion = "follow"
        verdict = follow_ok
        slot_measured = True
        if name in SIM_SLOT_PHASES:
            criterion = "follow+slot"
            try:
                scene = load_scene_config()
                lo, hi = slot_world_aabb(scene)
                centre = object_pose[0] + R.from_quat(object_pose[1]).apply(
                    object_bbox_centre_local(scene))
            except (OSError, ValueError, KeyError) as exc:
                # 插槽体积读不到 ⇒ C 判据 skipped（不是通过）。跟随判据本身仍然
                # 有效，但整体只能记"无法测量"，让日志如实反映。
                slot_measured = False
                parts.append(f"slot SKIPPED_UNMEASURED: {exc}")
                numbers["slot_ok"] = None
            else:
                inside = bool(np.all(centre >= lo) and np.all(centre <= hi))
                parts.append(
                    f"slot centre=[{centre[0]:.3f}, {centre[1]:.3f}, {centre[2]:.3f}] "
                    f"{'inside' if inside else 'OUTSIDE'} world AABB "
                    f"[{np.round(lo, 3).tolist()}]..[{np.round(hi, 3).tolist()}] "
                    f"(+{SIM_SLOT_TOL_M * 1000:.0f} mm)")
                numbers["slot_centre"] = [round(float(v), 4) for v in centre]
                numbers["slot_ok"] = inside
                verdict = verdict and inside
        ok: bool | None = verdict if slot_measured else None
        return {"criterion": criterion, "ok": ok, "detail": "; ".join(parts),
                **numbers}

    # ------------------------------------------------------------ execution
    def run_chunks(self, points: list[dict], chunks: int, time_scale: float,
                   dwell: float) -> bool:
        total = len(points)
        chunks = chunk_count(total, chunks)
        if chunks == 0:
            # A one-point plan means "already at the target"; there is nothing to
            # send.  The old formula would have divided by ``chunks`` and produced
            # repeated single-point chunks instead.
            self.get_logger().warn("trajectory has fewer than 2 points; no motion sent")
            return True
        bounds = [round(i * (total - 1) / chunks) for i in range(chunks + 1)]
        for index in range(chunks):
            chunk = points[bounds[index]: bounds[index + 1] + 1]
            path = Path(f"/tmp/pp_chunk_{index}.json")
            base = chunk[0]["t"]
            path.write_text(json.dumps({
                "joint_names": list(JOINTS),
                "points": [{"q": p["q"], "t": round(p["t"] - base, 6)} for p in chunk],
            }), encoding="utf-8")
            if self._first_point_far(chunk[0]):
                self.get_logger().error(
                    f"chunk {index} starts far from the current state; refusing to replay"
                )
                return False
            ok = self._exec_chunk(path, time_scale, dwell)
            if not ok:
                # Never replay: the arm may already be inside this chunk, and
                # replaying would snap it back to the chunk start.
                self.get_logger().warn(
                    f"chunk {index} failed; returning for a fresh replan"
                )
                return False
            # Did the arm actually go where the chunk ended?  The controller can
            # report success while the arm is short of the endpoint (it only has
            # to reach the trajectory's own last point).  A large gap means the
            # arm is blocked by something: stop instead of pushing further.
            state = self.fresh_joints()
            current = dict(zip(state.name, state.position))
            worst = max(abs(chunk[-1]["q"][i] - current.get(name, chunk[-1]["q"][i]))
                        for i, name in enumerate(JOINTS))
            if worst > self.chunk_endpoint_error:
                self.get_logger().error(
                    f"chunk {index} ended {worst:.3f} rad away from the planned "
                    "endpoint; the arm is blocked -- stopping (no replan, no push)")
                return False
        return True

    def _first_point_far(self, point: dict, limit_rad: float = 0.35) -> bool:
        """Guard against executing a trajectory that starts far from reality."""
        state = self.fresh_joints()
        current = dict(zip(state.name, state.position))
        return any(abs(point["q"][i] - current.get(name, point["q"][i])) > limit_rad
                   for i, name in enumerate(JOINTS))

    def _exec_chunk(self, path: Path, time_scale: float, dwell: float) -> bool:
        command = ["python3", "-u", "scripts/execute_assembly_chunks.py", str(path),
                   "--time-scale", str(time_scale), "--dwell", str(dwell),
                   "--goal-tolerance", str(self.goal_tolerance)]
        if self.action_name:
            command += ["--action", self.action_name]
        if self.world == "sim":
            # Gazebo on a CPU renderer advances slower than wall time: give the
            # trajectory the extra wall-clock budget it actually needs.
            command += ["--wait-scale", "4.0"]
            # The simulated position plant also lags a fast joint-space move and the
            # controller then aborts with GOAL_TOLERANCE_VIOLATED (measured: wrist_1
            # 1.25 rad short) while a real arm tracks the same trajectory.  Slowing
            # the SIMULATED execution is the honest fix: the real timings are
            # untouched and what the simulation validates (sequence, geometry,
            # clearances) does not change.
            command[command.index("--time-scale") + 1] = str(time_scale * 1.6)
        result = subprocess.run(command, capture_output=True, text=True, timeout=600)
        # The executor already distinguishes its failures with exit codes 2..8; the
        # old test grepped stdout for "ALL_ASSEMBLY_CHUNKS_OK", so a truncated log
        # or a crash just before the print looked like a rejection.
        ok = result.returncode == 0
        if not ok:
            # Say why: a silent False here sent the whole simulated run down a
            # "replan" path with no visible cause.
            tail = ((result.stdout or "") + (result.stderr or "")).strip().splitlines()
            self.get_logger().error(
                f"chunk executor failed (rc={result.returncode}): "
                + " | ".join(tail[-4:] or ["no output"]))
        return ok

    def robot_modes(self, timeout: float = 5.0) -> tuple[int | None, int | None]:
        """Read the two driver status topics once, freshly.

        AGENTS.md requires all four before any motion: safety_mode = 1, robot_mode
        = 7, arm_controller active, control_health three True.  Only the last two
        were ever checked; a dead driver keeps publishing its old safety_mode, so
        these are read right before the phase that is about to move, not once at
        start-up.
        """
        try:
            from eli_common_interface.msg import RobotMode, SafetyMode
        except Exception:  # noqa: BLE001 - report, do not crash
            self.get_logger().error(
                "cannot read safety_mode/robot_mode: eli_common_interface is not "
                "available in this shell (source the underlay)")
            return None, None
        values: dict[str, int] = {}
        # These are latched (TRANSIENT_LOCAL) topics published on change.  A plain
        # volatile subscription never receives the current value, which made the
        # runner report safety_mode=None and refuse to move even though the driver
        # was healthy (measured 2026-10-26).
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        subscriptions = [
            self.create_subscription(
                SafetyMode, "/io_and_status_controller/safety_mode",
                lambda msg: values.__setitem__("safety", int(msg.mode)), latched),
            self.create_subscription(
                RobotMode, "/io_and_status_controller/robot_mode",
                lambda msg: values.__setitem__("robot", int(msg.mode)), latched),
        ]
        deadline = time.time() + timeout
        while len(values) < 2 and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        for subscription in subscriptions:
            self.destroy_subscription(subscription)
        return values.get("safety"), values.get("robot")

    def ensure_control_ready(self) -> bool:
        """Fail fast instead of waiting: check and auto-recover the control chain.

        The real cell has a driver, a controller manager and three reverse-control
        sockets; scripts/control_health.py knows how to revive all of that.  In
        Gazebo there is no driver and no socket, so the same script would sit and
        time out - the simulation only needs the two controllers that move the arm
        and the gripper to be active.
        """
        if self.world == "sim":
            return self._ensure_sim_control_ready()
        safety, robot = self.robot_modes()
        if safety != 1 or robot != 7:
            self.get_logger().error(
                f"refusing to move: safety_mode={safety} (want 1 NORMAL), "
                f"robot_mode={robot} (want 7 RUNNING). Release the E-stop / clear "
                "the protective stop, then re-check with scripts/control_health.py")
            return False
        try:
            result = subprocess.run(
                [sys.executable, HEALTH_PY, "--fix"], capture_output=True,
                text=True, timeout=90, cwd=str(SCRIPTS_DIR.parent))
        except subprocess.TimeoutExpired:
            self.get_logger().error(
                "control_health.py --fix timed out after 90 s (controller manager "
                "unresponsive); refusing to move")
            return False
        healthy = result.returncode == 0
        if not healthy:
            self.get_logger().error("control chain unhealthy after recovery")
            self.get_logger().error((result.stdout or "").strip()[:400])
        return healthy

    def _ensure_sim_control_ready(self) -> bool:
        """Sim: no driver, no sockets. Ask the two things that matter directly.

        The real profile uses scripts/control_health.py, which knows about the
        driver and the reverse-control sockets - in Gazebo that script waits for
        hardware that will never appear.  Shelling out to ``ros2 control
        list_controllers`` is equally fragile (the CLI can take tens of seconds
        under load), so this checks the same facts over the existing clients:
        /joint_states is alive and the trajectory action server is up.
        """
        if self.joints() is None:
            self.get_logger().error("sim has no /joint_states yet")
            return False
        # Execution is delegated to scripts/execute_assembly_chunks.py, which needs
        # this action server; probe it with a throw-away client.
        from control_msgs.action import FollowJointTrajectory
        from rclpy.action import ActionClient

        probe = ActionClient(self, FollowJointTrajectory, self.action_name)
        available = probe.wait_for_server(timeout_sec=20.0)
        probe.destroy()
        if not available:
            self.get_logger().error(
                f"sim trajectory action {self.action_name} is not available; is "
                "joint_trajectory_controller active?")
            return False
        # The gripper is a plain ForwardCommandController: a publisher is enough.
        return True

# --------------------------------------------------------------------------- #
# Unload (取出) flow
# --------------------------------------------------------------------------- #
@dataclass
class UnloadConfig:
    insert_tcp: np.ndarray
    insert_tcp_q: tuple[float, float, float, float]
    entrance_q: tuple[float, float, float, float]
    pre_insert: np.ndarray
    staging: np.ndarray
    transit: np.ndarray
    place_pre_grasp: np.ndarray
    place_grasp: np.ndarray
    place_grasp_q: tuple[float, float, float, float]
    baseline: np.ndarray
    baseline_q: tuple[float, float, float, float]
    box_in_slot: np.ndarray
    slot_origin: np.ndarray
    grip_width_mm: float
    grip_current: float = 0.1
    # 测量结果参与抓取点修正（见 Config 里同名字段的说明）。
    workpiece_measurement: str = "runtime/measurements/workpiece_latest.json"
    workpiece_frame_offset: list = field(default_factory=list)
    workpiece_reference: dict = field(default_factory=dict)
    # CAD 包围盒（米，6 个数）：用来检查"抓取点是否落在实测工件里"，不依赖坐标系换算。
    workpiece_bbox: list = field(default_factory=list)
    extract_steps_mm: float = 40.0
    lift_total_mm: float = 155.0
    velocity_scale: float = 0.02
    assembly_velocity_scale: float = 0.01
    time_scale: float = 2.0
    dwell_sec: float = 1.0
    goal_tolerance: float = 0.002
    # 关节目标容差（rad）：传给 MoveIt 的 JointConstraint.tolerance_above/below。
    # 默认 0.02 = 真机一直用的值；仿真覆盖层会把它收紧（见 config/tasks/world_sim.yaml
    # 的 goal_joint_tolerance），原因：仿真靠"位姿跟随"表达夹持，抓取时工具停在哪里、
    # 工件就带着那个相对位姿走，规划端点偏差会一直传到插槽；真机是物理夹紧，夹爪夹到
    # 哪儿就是哪儿。
    goal_joint_tolerance: float = 0.02
    max_chunks: int = 7
    speeds: dict = field(default_factory=dict)
    axis: np.ndarray = field(default_factory=lambda: np.zeros(3))
    mode: str = "safe"
    scope: str = ""
    world: str = "real"
    action: str = "/arm_controller/follow_joint_trajectory"
    observe: dict = field(default_factory=dict)
    # 观测位检查命令的按模式覆盖：{waypoint: "shell 命令"}。
    # 位姿仍由 common.yaml 的 observe 提供，这里只换"用什么算子检查/测量"，
    # 这样效率模式可以换成本仓库的测量管线，而示教/安全模式继续用原检查。
    observe_checks: dict = field(default_factory=dict)
    phases: list[str] = field(default_factory=list)
    process_images: bool = True


def load_unload_config(path: str) -> UnloadConfig:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    g = lambda key: np.array(raw[key], dtype=float)  # noqa: E731
    q = lambda key: tuple(float(v) for v in raw[key])  # noqa: E731
    axis = g("insert_tcp") - g("pre_insert")
    cfg = UnloadConfig(
        insert_tcp=g("insert_tcp"),
        insert_tcp_q=q("insert_tcp_q"),
        entrance_q=q("entrance_q"),
        pre_insert=g("pre_insert"),
        staging=g("staging"),
        transit=g("transit"),
        place_pre_grasp=g("place_pre_grasp"),
        place_grasp=g("place_grasp"),
        place_grasp_q=q("place_grasp_q"),
        baseline=g("baseline"),
        baseline_q=q("baseline_q"),
        box_in_slot=g("box_in_slot"),
        slot_origin=g("slot_origin"),
        grip_width_mm=float(raw.get("grip_width_mm", 52.2)),
        grip_current=float(raw.get("grip_current", 0.1)),
        workpiece_measurement=str(raw.get("workpiece_measurement")
                                  or "runtime/measurements/workpiece_latest.json"),
        workpiece_frame_offset=list(raw.get("workpiece_frame_offset") or []),
        workpiece_reference=dict(raw.get("workpiece_reference") or {}),
        workpiece_bbox=list(raw.get("workpiece_bbox") or []),
        extract_steps_mm=float(raw.get("extract_steps_mm", 40.0)),
        lift_total_mm=float(raw.get("lift_total_mm", 155.0)),
        velocity_scale=float(raw.get("velocity_scale", 0.02)),
        assembly_velocity_scale=float(raw.get("assembly_velocity_scale", 0.01)),
        time_scale=float(raw.get("time_scale", 2.0)),
        dwell_sec=float(raw.get("dwell_sec", 1.0)),
        goal_tolerance=float(raw.get("goal_tolerance", 0.002)),
        goal_joint_tolerance=float(raw.get("goal_joint_tolerance", 0.02)),
        max_chunks=int(raw.get("max_chunks", 7)),
        speeds=dict(raw.get("phase_speeds", {}) or {}),
        axis=axis / np.linalg.norm(axis),
        mode=str(raw.get("mode", "safe")),
        scope=str(raw.get("scope", "")),
        world=str(raw.get("world", "real")),
        action=str(raw.get("action", "/arm_controller/follow_joint_trajectory")),
        observe=dict(raw.get("observe", {}) or {}),
        observe_checks=dict(raw.get("observe_checks", {}) or {}),
        phases=list(raw.get("phases", []) or []),
        process_images=bool(raw.get("process_images", True)),
    )
    stroke = float(np.linalg.norm(cfg.insert_tcp - cfg.pre_insert)) * 1000.0
    problems = []
    mode = getattr(cfg, "mode", "safe")
    if mode not in MAX_APPROACH_VELOCITY:
        problems.append(f"mode {mode!r} is not one of {sorted(MAX_APPROACH_VELOCITY)}")
        mode = "safe"
    if cfg.velocity_scale > MAX_APPROACH_VELOCITY[mode]:
        problems.append(f"velocity_scale {cfg.velocity_scale} > "
                        f"{MAX_APPROACH_VELOCITY[mode]} ({mode})")
    if cfg.assembly_velocity_scale > MAX_ASSEMBLY_VELOCITY[mode]:
        problems.append(f"assembly_velocity_scale {cfg.assembly_velocity_scale} > "
                        f"{MAX_ASSEMBLY_VELOCITY[mode]} ({mode})")
    problems.extend(phase_speed_problems(cfg.speeds, mode))
    if cfg.goal_tolerance > MAX_GOAL_TOLERANCE:
        problems.append("goal_tolerance too loose")
    if not 0.0 < cfg.goal_joint_tolerance <= MAX_GOAL_JOINT_TOLERANCE:
        problems.append(f"goal_joint_tolerance {cfg.goal_joint_tolerance} not in "
                        f"(0, {MAX_GOAL_JOINT_TOLERANCE}]")
    if stroke < 100.0:
        problems.append(f"extraction stroke {stroke:.1f} mm looks wrong")
    if problems:
        raise SystemExit("unload config rejected:\n  " + "\n  ".join(problems))
    return cfg


def run_observe_list(node, cfg: UnloadConfig, entries: list[tuple[str, str]],
                     args, plan_only: bool) -> list[dict]:
    """Run the observation waypoints that bracket the unload flow.

    Only the bracket observes are handled here: the unload flow owns its own
    phase sequence, so the configured job of this helper is the environment
    modelling prefix and the final parking observation.
    """
    results: list[dict] = []
    for name, kind in entries:
        if kind != "observe":
            continue
        waypoint = name.split(":", 1)[1]
        pose = (cfg.observe or {}).get(waypoint)
        if not pose:
            results.append({"phase": name, "ok": False,
                            "detail": f"OBSERVE_POSE_MISSING ({waypoint})"})
            continue
        xyz = np.asarray(pose["xyz"], dtype=float)
        quat = tuple(float(v) for v in pose["quat"])
        profile = profile_for(cfg.speeds, waypoint, cfg.velocity_scale, cfg.time_scale, 3)
        goal = node.ik(xyz, quat, seed=list(node.sim_q) if plan_only and node.sim_q else None)
        points = (node.plan_joint(goal, profile.velocity,
                                  list(node.sim_q) if plan_only and node.sim_q else None)
                  if goal else None)
        if not points:
            reason = "IK_FAILED" if goal is None else "PLAN_FAILED"
            results.append({"phase": name, "ok": False, "detail": reason})
            continue
        if plan_only:
            node.advance_sim(points)
            results.append({"phase": name, "ok": True,
                            "detail": f"plan-only points={len(points)} "
                                      f"dur={points[-1]['t']:.1f}s"})
            continue
        if not node.run_chunks(points, profile.chunks, profile.time_scale, 0.0):
            results.append({"phase": name, "ok": False, "detail": "execution failed"})
            continue
        time.sleep(1.0)
        if cfg.process_images:
            capture_process_image(f"observe:{waypoint}", "enter")
        override = (cfg.observe_checks or {}).get(waypoint)
        # 约定：覆盖值写 "none" 表示这个观测位不做检查（新管线不需要旧的建模检查时用）
        if isinstance(override, str) and override.strip().lower() in ("none", "skip"):
            check = ""
        else:
            check = (override or pose.get("check") or "").strip()
        if check and cfg.world == "sim":
            # 仿真里这一步只走流程，不真正执行检查。
            #
            # 两条理由：
            #   1) 这些检查用真机照片训练的 YOLO，Gazebo 渲染没有贴图，必然 "no mask"；
            #   2) 仿真的规划本来就用**已知的正确位姿**（config/tasks/common.yaml 里的
            #      grasp/pre_insert/insert 等常量，也就是真机实测出来的目标），
            #      不依赖任何视觉结果。
            # 所以启动子进程只是白等 20~40 秒再得到一次必然失败的结论。真机仍然照跑、仍然阻断。
            results.append({"phase": name, "ok": True,
                            "detail": f"check {check}: skipped in sim "
                                      f"(planning uses the known ground-truth poses)"})
        elif check:
            ok_check, detail = run_observation_check(check, pose)
            # The pose checks use a YOLO model trained on REAL photographs; a Gazebo
            # render has no texture, so the model reports "no mask" and the check can
            # never pass.  It says nothing about the motion, so in simulation it is
            # advisory: report it and keep the flow going.  Real runs stay blocking.
            advisory = cfg.world == "sim" and not ok_check
            if advisory:
                print(f"[sim] {name}: check failed (advisory, no texture in the "
                      f"Gazebo render): {detail}", file=sys.stderr)
            results.append({"phase": name, "ok": ok_check or advisory,
                            "detail": f"check {check}: {detail}"
                                      + (" (sim advisory)" if advisory else "")})
        else:
            results.append({"phase": name, "ok": True, "detail": "observed"})
    return results


def run_unload(node, args, cfg: UnloadConfig) -> list[dict]:
    """Reverse of the verified pick-and-place flow (senior D1-D9)."""
    record: list[dict] = []
    plan_only = bool(getattr(args, "plan_only", False))

    skip = {s.strip() for s in args.skip.split(",") if s.strip()}

    def current_tcp() -> np.ndarray:
        return node.current_tcp(plan_only)

    def start_positions() -> list[float] | None:
        return list(node.sim_q) if plan_only and node.sim_q else None

    def step(name: str, ok: bool, detail: str = "") -> bool:
        record.append({"phase": name, "ok": ok, "detail": detail,
                       "tcp": current_tcp().round(4).tolist(),
                       "plan_only": plan_only})
        publish_task_log(node, f"[{name}] {'OK' if ok else 'FAIL'} {detail}")
        # 面板的"当前流程/当前阶段"靠这个实时更新（以前只在开头结尾发一次）
        publish_task_state(node, {"flow": getattr(cfg, "scope", "") or args.flow,
                                  "phase": name, "ok": ok, "detail": detail})
        node.get_logger().info(f"[{name}] ok={ok} {detail}")
        Path(args.log).write_text(json.dumps(record, indent=2), encoding="utf-8")
        if not ok:
            raise SystemExit(f"phase {name} failed: {detail}")
        if args.until and name == args.until:
            raise SystemExit(f"stopped after requested phase {name}")
        return ok

    def cart(name: str, xyz, quat, speed: float, chunks: int | None = None) -> bool:
        if name in skip:
            node.get_logger().info(f"[{name}] skipped (already satisfied)")
            return True
        profile = profile_for(cfg.speeds, name, speed, cfg.time_scale,
                              chunks or cfg.max_chunks)
        if plan_only:
            legs = build_cartesian_legs(node.current_tcp(True), np.asarray(xyz, float),
                                        profile, (cfg.insert_tcp, cfg.axis))
            total, count = 0.0, 0
            for end_point, velocity, time_scale, chunks in legs:
                points = node.plan_cartesian(end_point, quat, velocity,
                                             start_positions(),
                                             avoid_collisions=not _no_collision_check(name))
                if not points:
                    return step(name, False, "plan failed")
                node.advance_sim(points)
                total += points[-1]["t"]
                count += len(points)
            return step(name, True, f"plan-only v={profile.velocity} points={count} "
                                    f"dur={total:.1f}s legs={len(legs)}")
        band = (cfg.insert_tcp, cfg.axis)
        if run_cartesian_profile(node, name, xyz, quat, profile, cfg.dwell_sec,
                                 node.get_logger(), band=band,
                                 tight=_no_collision_check(name)):
            return step(name, True, f"v={profile.velocity} ts={profile.time_scale}"
                                   f" band={profile.slow_band_mm}")
        for attempt in range(3):
            if not node.ensure_control_ready():
                continue
            points = node.plan_cartesian(xyz, quat, speed)
            if not points:
                node.get_logger().warn(f"{name}: planning attempt {attempt + 1} failed")
                continue
            if node.run_chunks(points, chunks or cfg.max_chunks,
                               cfg.time_scale, cfg.dwell_sec):
                return step(name, True, f"attempt {attempt + 1}")
            node.get_logger().warn(f"{name}: execution failed; replanning from current state")
            subprocess.run(["bash", RECOVER_SH], check=False,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return step(name, False, "3 attempts failed")

    def joint(name: str, xyz, quat) -> bool:
        if name in skip:
            node.get_logger().info(f"[{name}] skipped (already satisfied)")
            return True
        profile = profile_for(cfg.speeds, name, cfg.velocity_scale, 2.0, 6)
        if plan_only:
            # Use the phase profile here too: this path used to plan at the
            # global velocity_scale, so the cruise speed in the config file
            # never reached the unload transit phases.
            goal = node.ik(xyz, quat, seed=start_positions())
            points = (node.plan_joint(goal, profile.velocity, start_positions())
                      if goal else None)
            if not points:
                return step(name, False, "ik/plan failed")
            node.advance_sim(points)
            return step(name, True, f"plan-only v={profile.velocity} "
                                    f"points={len(points)} dur={points[-1]['t']:.1f}s")
        for attempt in range(3):
            if not node.ensure_control_ready():
                continue
            goal = node.ik(xyz, quat)
            points = node.plan_joint(goal, profile.velocity) if goal else None
            if not points:
                node.get_logger().warn(f"{name}: ik/planning attempt {attempt + 1} failed")
                continue
            if node.run_chunks(points, profile.chunks, profile.time_scale, cfg.dwell_sec):
                return step(name, True, f"attempt {attempt + 1}")
            node.get_logger().warn(f"{name}: execution failed; replanning from current state")
            subprocess.run(["bash", RECOVER_SH], check=False,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return step(name, False, "3 attempts failed")

    # D1 approach from the slot baseline to the outer staging point
    if not joint("d1_staging", cfg.staging, cfg.entrance_q):
        return record
    # Init before entering the slot: a residual finger gap would collide.
    if plan_only:
        step("d2_init_before_slot", True, "gripper phase skipped (plan-only)")
    elif node.ensure_control_ready() and node.ensure_gripper_initialized():
        step("d2_init_before_slot", True)
    # D2a approach to the pre-insert point (350 mm out, tilted)
    if not cart("d2a_pre_insert", cfg.pre_insert, cfg.entrance_q,
                cfg.velocity_scale):
        return record
    # D2b straight tilted travel into the entrance (position = insert TCP)
    if not cart("d2b_entrance_tilted", cfg.insert_tcp, cfg.entrance_q,
                cfg.assembly_velocity_scale):
        return record
    # D2c straighten so the gripper matches the seated workpiece
    if not cart("d2c_straighten", cfg.insert_tcp, cfg.insert_tcp_q,
                cfg.assembly_velocity_scale):
        return record
    # D3 initialise the gripper, then grip
    if plan_only:
        step("d3a_gripper_init", True, "gripper phase skipped (plan-only)")
        step("d3b_grip", True, "gripper phase skipped (plan-only)")
    else:
        if not node.ensure_control_ready():
            return record
        if not node.ensure_gripper_initialized():
            return step("d3a_gripper_init", False, "init did not finish") and record
        time.sleep(1.2)  # let the firmware settle before the first position command
        step("d3a_gripper_init", True)
        if not node.grip_and_wait(cfg.grip_width_mm, getattr(cfg, "grip_current", 0.1)):
            return step("d3b_grip", False, "grip did not settle") and record
        step("d3b_grip", True, f"{cfg.grip_width_mm} mm")
    # D3c small pull test along the axis to confirm the workpiece follows
    test_back = current_tcp() - cfg.axis * 0.003
    before = current_tcp()
    if not cart("d3c_pull_test", test_back, cfg.insert_tcp_q,
                cfg.assembly_velocity_scale, chunks=1):
        return record
    if plan_only:
        step("d3d_pull_check", True, "planned only: no measured pull to report")
    else:
        moved = float(np.linalg.norm(current_tcp() - before)) * 1000.0
        step("d3d_pull_check", True, f"moved {moved:.1f} mm")
    if not cart("d3e_return", cfg.insert_tcp, cfg.insert_tcp_q,
                cfg.assembly_velocity_scale, chunks=1):
        return record
    # D4 tilt back for extraction (two steps to avoid binding)
    if not cart("d4_tilt_for_removal", cfg.insert_tcp, cfg.entrance_q,
                cfg.assembly_velocity_scale):
        return record
    # D5 extract along the tilted axis in small steps
    if any(name.startswith("d5_extract") for name in skip):
        node.get_logger().info("[d5_extract] skipped (already satisfied)")
        step_m = None
    else:
        step_m = cfg.extract_steps_mm / 1000.0
    total_m = float(np.linalg.norm(cfg.insert_tcp - cfg.pre_insert))
    done = 0.0
    index = 0
    while step_m is not None and done < total_m - 1e-6:
        index += 1
        piece = min(step_m, total_m - done)
        done += piece
        target = cfg.insert_tcp - cfg.axis * done
        if not cart(f"d5_extract_{index}", target, cfg.entrance_q,
                    cfg.assembly_velocity_scale, chunks=2):
            return record
    # D6 lift and transit while holding the workpiece
    lift = current_tcp() + np.array([0.0, 0.0, cfg.lift_total_mm / 1000.0])
    if not cart("d6a_lift", lift, cfg.entrance_q, cfg.assembly_velocity_scale):
        return record
    if not joint("d6b_transit", cfg.transit, cfg.place_grasp_q):
        return record
    # D7 place: approach the original table pose
    if not joint("d7a_place_pre_grasp", cfg.place_pre_grasp, cfg.place_grasp_q):
        return record
    if not cart("d7b_place_grasp", cfg.place_grasp, cfg.place_grasp_q,
                cfg.assembly_velocity_scale):
        return record
    # D8 release — never release without explicit approval for this run.
    if plan_only:
        step("d8_release", True, "gripper phase skipped (plan-only)")
        step("d8b_init_before_retreat", True, "gripper phase skipped (plan-only)")
    else:
        if not getattr(args, "allow_release", False):
            return step(
                "d8_release",
                False,
                "refused: gripper release requires --allow-release (owner approval)",
            ) and record
        if not node.ensure_control_ready():
            return record
        node.gripper("grip", max(0.0, cfg.grip_width_mm - 20.0))
        node.gripper("grip", 20.0)
        node.gripper("init")
        node._grasp_event.publish(String(data="release_confirmed"))
        step("d8_release", True)
        # Init again before retreating: the gap must be gone before any translation.
        if node.ensure_control_ready() and node.ensure_gripper_initialized():
            step("d8b_init_before_retreat", True)
    # D9 retreat: pure translation, then lift, then baseline
    approach_dir = cfg.place_grasp - cfg.place_pre_grasp
    approach_dir = approach_dir / np.linalg.norm(approach_dir)
    back = current_tcp() - approach_dir * 0.120
    if not cart("d9a_withdraw", back, cfg.place_grasp_q, cfg.assembly_velocity_scale):
        return record
    lift2 = current_tcp() + np.array([0.0, 0.0, cfg.lift_total_mm / 1000.0])
    if not cart("d9b_lift", lift2, cfg.place_grasp_q, cfg.assembly_velocity_scale):
        return record
    joint("d9c_baseline", cfg.baseline, cfg.baseline_q)
    return record



# --------------------------------------------------------------------------- #
# Speed application helpers (uneven speed: fast cruise, slow final segment)
# --------------------------------------------------------------------------- #
# TIGHT_CLEARANCE_PHASES and the speed caps live next to validate(); there is
# deliberately only one definition so the validator and the profile enforcer can
# never disagree about which phases are inside the slot.
ALLOW_UNEVEN_CLEARANCE = False


# Pushing phases: the tool is in contact with the part/fixture, so a blocked
# trajectory must be detected early.  They are executed as several short chunks
# and every chunk end is measured against the plan (see run_chunks).
CONTACT_PHASES = {"grasp", "insert", "straighten", "d5_extract"}
MIN_CONTACT_CHUNKS = 4


def profile_for(speeds: dict, name: str, fallback_velocity: float,
                fallback_scale: float, fallback_chunks: int) -> SpeedProfile:
    raw = speeds.get(name)
    if raw is None:
        for key in speeds:
            if name.startswith(key) and name != key:
                raw = speeds[key]
                break
    profile = SpeedProfile.from_dict(raw, fallback_velocity, fallback_scale,
                                     fallback_chunks)
    if profile.slow_tail_mm > 0 and not ALLOW_UNEVEN_CLEARANCE:
        if any(name.startswith(prefix) for prefix in TIGHT_CLEARANCE_PHASES):
            # Tight-clearance assembly motion: force a uniform profile.
            profile.slow_tail_mm = 0.0
            profile.velocity = min(profile.velocity, profile.slow_velocity)
            profile.time_scale = max(profile.time_scale, profile.slow_time_scale)
    if name in CONTACT_PHASES:
        # Split the push into short chunks so a blocked arm is detected after a few
        # millimetres instead of after the whole trajectory (2026-10-26: the grasp
        # pushed into the workpiece for the entire 7 s move).
        profile.chunks = max(profile.chunks, MIN_CONTACT_CHUNKS)
    return profile


def split_band_legs(start, target, band, sample: int = 200):
    """Split one straight move into legs so only the band is slow.

    ``band`` is ``(origin, axis, low_m, high_m)``: a point is inside the band when
    ``dot(origin - point, axis)`` lies in ``[low_m, high_m]``.  That quantity is
    the backoff distance from the seated pose, so the same band works for both
    the retreat (distance grows) and the entry (distance shrinks).

    Returns a list of ``(end_point, is_slow)`` legs in travel order.  Sampling is
    used only to find the crossings; the legs themselves are planned as straight
    cartesian segments, so the result is still exact.
    """
    import numpy as _np

    if band is None:
        return [(target, False)]
    origin, axis, low, high = band
    start = _np.asarray(start, dtype=float)
    target = _np.asarray(target, dtype=float)
    span = target - start
    length = float(_np.linalg.norm(span))
    if length < 1e-6:
        return [(target, False)]

    def inside(point) -> bool:
        value = float(_np.dot(_np.asarray(origin, dtype=float) - point, axis))
        return low <= value <= high

    crossings = []
    for index in range(sample + 1):
        t = index / sample
        inside_now = inside(start + span * t)
        if index == 0:
            previous = inside_now
            continue
        if inside_now != previous:
            crossings.append(t)
        previous = inside_now

    cuts = sorted({0.0, 1.0, *crossings})
    legs = []
    for index in range(len(cuts) - 1):
        t0, t1 = cuts[index], cuts[index + 1]
        if t1 - t0 < 1e-6:
            continue
        middle = start + span * ((t0 + t1) / 2.0)
        legs.append((start + span * t1, inside(middle)))
    return legs or [(target, False)]


def build_cartesian_legs(current, target, profile: SpeedProfile, band=None):
    """All the straight legs of one cartesian phase, with their speeds.

    Shared by the executing path and the plan-only path so a rehearsal predicts
    exactly the speeds the real run will use.
    """
    import numpy as _np

    current = _np.asarray(current, dtype=float)
    target = _np.asarray(target, dtype=float)
    span = target - current
    length = float(_np.linalg.norm(span))
    if length < 1e-6:
        return []
    band_band = None
    if profile.slow_band_mm and band is not None:
        origin, axis = band
        low, high = profile.slow_band_mm
        band_band = (origin, axis, low / 1000.0, high / 1000.0)
    tail = min(profile.slow_tail_mm / 1000.0, max(length - 0.001, 0.0))
    legs = []
    if band_band is not None:
        for end_point, slow in split_band_legs(current, target, band_band):
            if slow:
                legs.append((end_point, profile.slow_velocity,
                             profile.slow_time_scale, 2))
            else:
                legs.append((end_point, profile.velocity,
                             profile.time_scale, profile.chunks))
    elif tail > 0.002:
        direction = span / length
        legs.append((target - direction * tail, profile.velocity,
                     profile.time_scale, profile.chunks))
        legs.append((target, profile.slow_velocity,
                     profile.slow_time_scale, 2))
    else:
        legs.append((target, profile.velocity, profile.time_scale,
                     profile.chunks))
    return legs


def run_cartesian_profile(node, name: str, xyz, quat, profile: SpeedProfile,
                          dwell: float, log=None, band=None,
                          tight: bool = False) -> bool:
    """Move to xyz, cruising fast and slowing down only where the profile says.

    Two slowing mechanisms exist and can be combined:
      * ``slow_tail_mm`` -- uniform slow final approach (assembly segments);
      * ``slow_band_mm`` -- an axial window (the workpiece handle region), used
        by the gripper-only moves that carry nothing and have no assembly fit.
    """
    import numpy as _np

    for attempt in range(3):
        if not node.ensure_control_ready():
            continue
        target = _np.array(xyz, dtype=float)
        current = node.tcp()
        legs = build_cartesian_legs(current, target, profile, band)
        if not legs:
            return True
        ok = True
        for leg_target, velocity, time_scale, chunks in legs:
            points = node.plan_cartesian(leg_target, quat, velocity,
                                         avoid_collisions=not tight)
            if not points:
                ok = False
                break
            if not node.run_chunks(points, chunks, time_scale, dwell):
                ok = False
                break
        if ok:
            return True
        if log:
            log.warn(f"{name}: leg failed on attempt {attempt + 1}; replanning from current")
        subprocess.run(["bash", RECOVER_SH], check=False,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return False


_CAMERA_RESTORE_REGISTERED = [False]


def switch_camera_mode(mode: str) -> bool:
    """切换相机档位：fast=640×480 高帧率（装配段观察用），modeling=1280×960。

    切到 fast 时注册 atexit 恢复 modeling——流程无论正常结束还是异常退出，
    都不会把相机留在低分辨率档上。
    """
    try:
        completed = subprocess.run(
            [sys.executable, "-u", str(REPO / "scripts" / "camera_mode.py"), mode],
            capture_output=True, text=True, timeout=180)
        ok = completed.returncode == 0
    except Exception:  # noqa: BLE001
        return False
    if mode == "fast" and not _CAMERA_RESTORE_REGISTERED[0]:
        import atexit
        atexit.register(lambda: switch_camera_mode("modeling"))
        _CAMERA_RESTORE_REGISTERED[0] = True
    return ok


def publish_task_log(node, text: str) -> None:
    """把阶段日志发到 /task_log（面板订阅它，就能看到"终端里跑的流程"的日志）。"""
    try:
        if getattr(node, "task_log_pub", None) is None:
            node.task_log_pub = node.create_publisher(String, "/task_log", 10)
        node.task_log_pub.publish(String(data=str(text)[:1000]))
    except Exception:  # noqa: BLE001 - 日志推送不能影响流程
        pass


def publish_task_state(node, state: dict) -> None:
    """Publish the live flow/phase state for the Qt and RViz panels.

    Two consumers with different capabilities: the Qt panel and any ROS tool can
    subscribe to ``/task_state`` (JSON string), and a plain file is written as well
    so a UI that is not part of this ROS graph (or a shell script) can still see
    what the arm is doing.  Best effort: never let a UI break a phase.
    """
    import json as _json

    payload = dict(state)
    payload.setdefault("stamp", time.time())
    text = _json.dumps(payload, ensure_ascii=False)
    try:
        if getattr(node, "task_state_pub", None) is None:
            node.task_state_pub = node.create_publisher(String, "/task_state", 10)
        node.task_state_pub.publish(String(data=text))
    except Exception:  # noqa: BLE001 - the UI feed must never stop a phase
        pass
    try:
        out = Path("runtime/task_state.json")
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(text, encoding="utf-8")
    except Exception:  # noqa: BLE001
        pass


def arm_seed_from_state(names, positions) -> list[float]:
    """The six arm joints in JOINTS order, matched by NAME.

    /joint_states is not ordered the way this file needs it: the real driver
    publishes its own order and Gazebo publishes the joints alphabetically with the
    two gripper fingers in the middle:

        elbow, gripper_left_finger, gripper_right_finger, shoulder_lift,
        shoulder_pan, wrist_1, wrist_2, wrist_3

    Using that raw array as an IK seed silently pairs values with the wrong joint
    names, the solver returns a flipped branch, and the 2.6 rad branch guard then
    refuses an otherwise reachable pose.  That is exactly how the first simulated
    runs failed (ten "nearest solution is 2.6-3.1 rad away" refusals) while the
    same poses planned fine from --start-joints.
    """
    by_name = dict(zip(names, positions))
    missing = [name for name in JOINTS if name not in by_name]
    if missing:
        raise RuntimeError(
            f"/joint_states is missing arm joints {missing}; got {sorted(by_name)}")
    return [float(by_name[name]) for name in JOINTS]


def finger_position_m(width_mm: float, max_opening_mm: float = 50.0,
                      span_min_mm: float = 204.0, span_max_mm: float = 246.57) -> float:
    """Map the EFG opening command to the simulated finger joint position.

    The real tool is an expanding brace on a HIBFOT EFG80: the driver's
    ``position_mm`` is the jaw opening (0–50 mm, owner-confirmed maximum), and the
    linkage converts it into the outward span between the two support arms
    (204.00 mm fully closed, 246.57 mm fully open, SolidWorks).  The simulated
    fingers travel half of that span change, so this is linear:

        joint = (width / max_opening) * (span_max - span_min) / 2

    Keeping it a pure function means the mapping is testable without ROS and the
    numbers come from one place.
    """
    if max_opening_mm <= 0.0:
        raise ValueError("max_opening_mm must be positive")
    ratio = max(0.0, min(1.0, float(width_mm) / float(max_opening_mm)))
    return ratio * (float(span_max_mm) - float(span_min_mm)) / 2.0 / 1000.0


def notify_recorder(phase: str, boundary: str) -> None:
    """Tell the running experiment recorder which phase just started/ended.

    Best effort: no recorder, no script or a broken call must never abort a
    phase.  The recorder owns the timeseries; this only labels it.
    """
    script = Path(__file__).resolve().parent / "experiment_recorder.py"
    pidfile = Path("runtime/recorder.pid")
    if not script.exists() or not pidfile.exists():
        return
    subprocess.run(["python3", "-u", str(script), "set-phase", "--tag",
                    f"{phase}:{boundary}"],
                   capture_output=True, timeout=20, check=False)


def capture_process_image(tag: str, boundary: str) -> None:
    """Best-effort labelled frame at a phase/observation boundary.

    The capture helper is optional: a missing script or a camera that is not
    publishing must never abort a task phase.
    """
    script = Path(__file__).resolve().parent / "capture_process_images.py"
    if not script.exists():
        return
    subprocess.run(
        ["python3", "-u", str(script), "--tag", f"{tag}:{boundary}",
         "--out", "runtime/process_images"],
        capture_output=True, timeout=60, check=False)


def run_observation_check(command: str, pose: dict) -> tuple[bool, str]:
    """Run the configured check for an observation waypoint.

    The check is a plain shell command so the pose logic stays in the pose
    scripts (ICP, environment gate, ...) instead of growing inside the runner.
    Exit code 0 means the observation is acceptable.
    """
    extra = ""
    if pose.get("check_args"):
        extra = " " + " ".join(str(v) for v in pose["check_args"])
    completed = subprocess.run(command + extra, shell=True, capture_output=True,
                               timeout=600, text=True)
    output = ((completed.stdout or "") + (completed.stderr or "")).strip().splitlines()
    # 失败时"最后一行"往往只是"写入 xxx.json"，把真正的原因（verdict/reasons）找出来
    if completed.returncode != 0:
        reason_lines = [line.strip() for line in output
                        if "verdict" in line or "reasons" in line or "< " in line]
        if reason_lines:
            return False, " | ".join(reason_lines[:2])
    tail = output[-1] if output else "no output"
    return completed.returncode == 0, tail


def resolve_phase_list(cfg) -> list[tuple[str, str]]:
    """Phase list to run: the config owns it when present, else the built-in list.

    Entries are either a plain phase name or ``"observe:<waypoint>"``, which maps
    to the ``observe`` phase kind and looks the pose up in ``cfg.observe``.
    """
    configured = list(getattr(cfg, "phases", []) or [])
    if not configured:
        return phase_plan()
    resolved = []
    for entry in configured:
        entry = str(entry).strip()
        if not entry:
            continue
        if entry.startswith("measure:") or entry.startswith("measure_"):
            name = entry.split(":", 1)[1] if ":" in entry else entry[len("measure_"):]
            resolved.append((f"measure:{name}", "measure"))
            continue
        if entry.startswith("nbv:") or entry.startswith("nbv_"):
            name = entry.split(":", 1)[1] if ":" in entry else entry[len("nbv_"):]
            resolved.append((f"nbv:{name}", "nbv"))
            continue
        if entry.startswith("observe:") or entry.startswith("observe_"):
            name = entry.split(":", 1)[1] if ":" in entry else entry[len("observe_"):]
            resolved.append((f"observe:{name}", "observe"))
            continue
        known = dict(phase_plan())
        resolved.append((entry, known.get(entry, "joint")))
    return resolved


# --- 抓取接近的几何细节（与示教流程 teaching_sequence.py 保持一致）-------------
APPROACH_HEIGHT_M = 0.150      # 从抓取点正上方 150 mm 垂直接近
GRASP_CLEARANCE_M = 0.010      # 抓取点沿工具 x 回退 10 mm（避开夹爪电机保护座）
GRASP_SNAP_VERTICAL = True     # 姿态压垂直：工具 z 指向世界 -z，保留实测 yaw
GRASP_SLOW_RADIUS_M = 0.15     # 距目标这么近时按距离降速
GRASP_SLOW_VELOCITY = 0.01     # 最近处的速度标度


def snap_vertical_quat(quat) -> tuple[float, float, float, float]:
    """把姿态压成严格垂直（工具 z → 世界 -z），保留 yaw。"""
    from scipy.spatial.transform import Rotation as _R
    matrix = _R.from_quat(quat).as_matrix()
    jaw = matrix @ np.array([1.0, 0.0, 0.0])
    yaw = float(np.arctan2(jaw[1], jaw[0]))
    snapped = (_R.from_euler("z", yaw) * _R.from_euler("x", 180.0, degrees=True)).as_quat()
    return tuple(float(v) for v in snapped)


def grasp_pose_with_clearance(xyz, quat) -> tuple[np.ndarray, tuple]:
    """抓取点：压垂直后沿工具 x 回退固定距离（避免电机保护座蹭工件）。"""
    from scipy.spatial.transform import Rotation as _R
    orientation = snap_vertical_quat(quat) if GRASP_SNAP_VERTICAL else quat
    rotation = _R.from_quat(orientation).as_matrix()
    back = rotation @ np.array([GRASP_CLEARANCE_M, 0.0, 0.0])
    return np.asarray(xyz, dtype=float) - back, orientation


def approach_pose(grasp_xyz) -> np.ndarray:
    """预抓取点：抓取点正上方 APPROACH_HEIGHT_M（垂直接近，而不是横向 120 mm）。"""
    point = np.asarray(grasp_xyz, dtype=float).copy()
    point[2] += APPROACH_HEIGHT_M
    return point


def proximity_velocity(distance_m: float, base_velocity: float) -> float:
    """按距离降速：越靠近目标越慢，最少降到 GRASP_SLOW_VELOCITY。"""
    if distance_m >= GRASP_SLOW_RADIUS_M:
        return base_velocity
    ratio = max(0.0, distance_m) / GRASP_SLOW_RADIUS_M
    return max(GRASP_SLOW_VELOCITY, base_velocity * (0.3 + 0.7 * ratio))


# --- 用实测工件位姿修正抓取点 -------------------------------------------------
# 本仓库的测量管线输出 T_base_object（CAD 系）；示教常数用的是另一套"box 系"。
# 两者之间差一个固定变换，用 config 的 workpiece_frame_offset 标定一次即可：
#   box_pose = T_base_object @ offset
# 没标定时保持原行为（用常数），并在日志里说明"测量结果未参与抓取计算"。
def measured_box_pose(measurement_path: str, offset_cfg) -> np.ndarray | None:
    from scipy.spatial.transform import Rotation as _R
    path = Path(measurement_path)
    if not path.exists():
        return None
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        group = (data.get("per_group") or [{}])[0]
        place = group.get("pose_base") or {}
        translation = place.get("translation")
        quaternion = place.get("quaternion")
        if translation is None or quaternion is None:
            return None
    except (OSError, ValueError, IndexError):
        return None
    base = np.eye(4)
    base[:3, :3] = _R.from_quat(quaternion).as_matrix()
    base[:3, 3] = translation
    if not offset_cfg:
        return base
    offset = np.eye(4)
    offset[:3, :3] = _R.from_quat(list(offset_cfg)[3:7]).as_matrix()
    offset[:3, 3] = list(offset_cfg)[:3]
    return base @ offset


def adjust_grasp_with_measurement(grasp_xyz, pre_xyz, measured: np.ndarray | None,
                                  reference, calibrated: bool = False
                                  ) -> tuple[np.ndarray, np.ndarray, str]:
    """按"实测工件 box 位姿 vs 参考 box 位姿"的差，把抓取点/预抓取点平移过去。

    前提：workpiece_frame_offset 已标定 —— 测量输出是【CAD 系】，示教常数是【box 系】，
    两者差一个固定变换。没标定时若照样平移，会按错误的坐标系差把抓取点推偏
    （实测这会把抓取点推 137 mm），所以这里必须显式要求 calibrated。
    """
    if not calibrated:
        return grasp_xyz, pre_xyz, "测量未参与（workpiece_frame_offset 未标定）"
    if measured is None or reference is None:
        return grasp_xyz, pre_xyz, "测量未参与（缺测量文件或参考位姿）"
    reference_xyz = np.asarray(reference.get("xyz"), dtype=float)
    delta = measured[:3, 3] - reference_xyz
    return (np.asarray(grasp_xyz, float) + delta, np.asarray(pre_xyz, float) + delta,
            f"已按实测位姿平移 {np.round(delta, 4).tolist()} m")


def phase_plan() -> list[tuple[str, str]]:
    return [
        ("pre_grasp", "joint"),
        ("gripper_init", "special"),
        ("grasp", "joint"),
        ("grip", "special"),
        ("lift", "cartesian"),
        ("transit", "joint"),
        ("staging", "joint"),
        ("pre_insert", "cartesian"),
        ("insert", "cartesian"),
        ("straighten", "cartesian"),
        ("release", "special"),
        ("withdraw", "cartesian"),
        ("baseline", "joint"),
    ]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--confirm", action="store_true",
                        help="required: actually execute motion")
    parser.add_argument("--plan-only", action="store_true",
                        help="plan every phase without executing: each phase is "
                             "planned from where the previous planned phase would "
                             "have ended (simulated state), so the whole flow can "
                             "be validated while the arm stands still")
    parser.add_argument("--start-joints", default="",
                        help="plan-only only: six comma-separated joint angles to "
                             "chain the plans from, instead of the measured state. "
                             "Use it to rehearse from the pose the arm will really "
                             "be in without touching the robot")
    parser.add_argument("--phases", default="")
    parser.add_argument("--log", default="/tmp/pick_place_run.json")
    parser.add_argument("--flow", default="pick_place", choices=["pick_place", "unload"])
    parser.add_argument("--until", default="", help="stop after this phase name")
    parser.add_argument("--skip", default="", help="comma list of phases already satisfied")
    parser.add_argument("--allow-uneven-insert", action="store_true",
                        help="allow fast+slow profiles inside the tight slot clearance "
                             "(only after real-cell validation)")
    parser.add_argument("--allow-release", action="store_true",
                        help="required for any gripper release: owner must approve each time")
    args = parser.parse_args()

    global ALLOW_UNEVEN_CLEARANCE
    ALLOW_UNEVEN_CLEARANCE = bool(getattr(args, "allow_uneven_insert", False))
    rclpy.init()
    node = Runner()

    if args.flow == "unload":
        ucfg = load_unload_config(args.config)
        node.goal_tolerance = ucfg.goal_tolerance
        node.goal_joint_tolerance = ucfg.goal_joint_tolerance
        node.action_name = ucfg.action
        # 之前漏了这两行：Runner 默认 world="real"，于是 --world sim 的 unload 会去
        # 调真机夹爪的 PowerShell 桥（真把夹爪撑开）、并等真机的 control_health。
        node.world = ucfg.world
        node.sim_model = getattr(ucfg, "sim_workpiece_model", SIM_WORKPIECE_MODEL)
        node.sim_gripper_stroke_m = getattr(ucfg, "sim_gripper_stroke_m", 0.0)
        node.gripper_geometry = {
            "max": getattr(ucfg, "gripper_max_opening_mm", 50.0),
            "min": getattr(ucfg, "gripper_span_min_mm", 204.0),
            "max_span": getattr(ucfg, "gripper_span_max_mm", 246.57),
        }
        if not args.confirm and not args.plan_only:
            node.get_logger().warn("dry run: pass --confirm to execute motion "
                                   "(or --plan-only to plan without moving)")
            for name in ("d1_staging", "d2_init_before_slot", "d2a_pre_insert",
                         "d2b_entrance_tilted", "d2c_straighten",
                         "d3a_gripper_init", "d3b_grip", "d3c_pull_test",
                         "d4_tilt_for_removal", "d5_extract_*", "d6a_lift",
                         "d6b_transit", "d7a_place_pre_grasp", "d7b_place_grasp",
                         "d8_release", "d8b_init_before_retreat",
                         "d9a_withdraw", "d9b_lift", "d9c_baseline"):
                node.get_logger().info(f"planned unload phase {name}")
            return 0
        if args.plan_only:
            if args.start_joints:
                seed = [float(v) for v in args.start_joints.split(",") if v.strip()]
                if len(seed) != len(JOINTS):
                    raise SystemExit(
                        f"--start-joints needs {len(JOINTS)} values, got {len(seed)}")
                node.get_logger().info(
                    f"plan-only: starting from --start-joints {np.round(seed, 3).tolist()}")
            else:
                seed = node.arm_positions()
                node.get_logger().info(
                    f"plan-only: starting from measured joints {np.round(seed, 3).tolist()}")
            node.sim_q = list(seed)
        phases = resolve_phase_list(ucfg)
        first = next((i for i, (_n, k) in enumerate(phases) if k != "observe"), len(phases))
        last = max((i for i, (_n, k) in enumerate(phases) if k != "observe"), default=-1)
        leading, trailing = phases[:first], phases[last + 1:]
        bracket: list[dict] = []
        if leading:
            node.get_logger().info(
                f"environment-modelling prefix: {[n for n, _ in leading]}")
            bracket.extend(run_observe_list(node, ucfg, leading, args, args.plan_only))
            if any(not item["ok"] for item in bracket):
                print(json.dumps(bracket[-1], indent=2))
                return 1
        record = run_unload(node, args, ucfg)
        if trailing:
            node.get_logger().info(f"parking observation: {[n for n, _ in trailing]}")
            record = record + run_observe_list(node, ucfg, trailing, args, args.plan_only)
        record = bracket + record
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print(json.dumps(record[-1] if record else {}, indent=2))
        return 0 if record and all(item["ok"] for item in record) else 1

    cfg = load_config(args.config)
    wanted = [p.strip() for p in args.phases.split(",") if p.strip()]
    node.goal_tolerance = cfg.goal_tolerance
    node.goal_joint_tolerance = cfg.goal_joint_tolerance
    node.action_name = cfg.action
    node.world = cfg.world
    # 仿真验收要按名字读工件真值位姿；真机不用这个字段（判据仍是 verify_pose.py）。
    node.sim_model = cfg.sim_workpiece_model
    node.sim_gripper_stroke_m = cfg.sim_gripper_stroke_m
    node.gripper_geometry = {"max": cfg.gripper_max_opening_mm,
                             "min": cfg.gripper_span_min_mm,
                             "max_span": cfg.gripper_span_max_mm}
    record: list[dict] = []
    plan_only = bool(args.plan_only)

    def current_tcp() -> np.ndarray:
        """Plan-only works on the simulated state; a real run on the measured one."""
        return node.current_tcp(plan_only)

    collision_applied = {"done": False}

    def ensure_workpiece_collision() -> bool:
        """把【实测】工件位姿发布到 MoveIt 规划场景。

        为什么必须做：run_pick_and_place 从不发布碰撞场景，MoveIt 在"空世界"里规划，
        路径可以贴着工件甚至穿过它（2026-11-11 真机差点撞上工件就是这么来的）。
        apply_environment_collision.py 的 --workpiece-xyz/quat 正好吃【CAD 系】位姿，
        和测量管线输出一致。
        """
        if collision_applied["done"]:
            return True
        measured = measured_box_pose(
            str(getattr(cfg, "workpiece_measurement", "") or ""), None)
        if measured is None:
            node.get_logger().warn("[collision] 没有测量结果，跳过工件碰撞体发布")
            return False
        from scipy.spatial.transform import Rotation as _R
        quat = _R.from_matrix(measured[:3, :3].copy()).as_quat()
        xyz = measured[:3, 3]
        code, text = run([sys.executable, "-u",
                          str(REPO / "scripts" / "apply_environment_collision.py"),
                          "--workpiece-xyz", *[f"{v:.4f}" for v in xyz],
                          "--workpiece-quat", *[f"{v:.5f}" for v in quat]],
                         timeout=180.0)
        tail = (text or "").strip().splitlines()
        node.get_logger().info(
            f"[collision] 工件碰撞体 {'已发布' if code == 0 else '发布失败'}："
            f"{tail[-1] if tail else 'no output'}")
        collision_applied["done"] = code == 0
        return code == 0

    grip_targets_cache: dict = {}

    def grip_targets() -> tuple[np.ndarray, tuple, np.ndarray, str]:
        """抓取点 / 预抓取点（含实测位姿修正）——整轮只算一次，两个阶段共用。"""
        if grip_targets_cache:
            item = grip_targets_cache["value"]
            return item
        grasp_xyz, grasp_quat = grasp_pose_with_clearance(cfg.grasp, cfg.grasp_q)
        pre_xyz = approach_pose(grasp_xyz)
        measured = measured_box_pose(
            str(getattr(cfg, "workpiece_measurement", "") or ""),
            getattr(cfg, "workpiece_frame_offset", None))
        # 合理性检查：把抓取点投到实测工件坐标里，必须落在 CAD 包围盒外扩 60 mm 内。
        # 这条不依赖任何"两套坐标系"的换算——直接用测量出来的工件位姿反查。
        plausibility = "抓取点合理性未检查（没有可用测量）"
        if measured is not None and getattr(cfg, "workpiece_bbox", None):
            bbox_min = np.asarray(cfg.workpiece_bbox[:3], dtype=float)
            bbox_max = np.asarray(cfg.workpiece_bbox[3:6], dtype=float)
            local = np.linalg.inv(measured) @ np.append(np.asarray(grasp_xyz, float), 1.0)
            inside = np.all(local[:3] >= bbox_min - 0.06) and np.all(local[:3] <= bbox_max + 0.06)
            plausibility = (f"抓取点在工件局部坐标 {np.round(local[:3],3)}（"
                            f"{'OK' if inside else 'OUTSIDE bbox+60mm'}）")
            if not inside:
                raise SystemExit(f"grasp target rejected: {plausibility}；"
                                 f"请先标定 workpiece_frame_offset 或重新示教抓取点")
        grasp_xyz, pre_xyz, note = adjust_grasp_with_measurement(
            grasp_xyz, pre_xyz, measured,
            getattr(cfg, "workpiece_reference", None),
            calibrated=bool(getattr(cfg, "workpiece_frame_offset", None)))
        note = f"{note}；{plausibility}"
        item = (grasp_xyz, grasp_quat, pre_xyz, note)
        grip_targets_cache["value"] = item
        node.get_logger().info(f"[grasp target] {note}; grasp={np.round(grasp_xyz,4)}")
        return item

    def record_verdict(verdict: dict) -> None:
        """把一条仿真判据结果写进刚结束的阶段记录（verified / verify_detail / 数字）。"""
        record[-1]["verified"] = verdict["ok"]
        record[-1]["verify_detail"] = verdict["detail"]
        record[-1]["sim_check"] = {key: value for key, value in verdict.items()
                                   if key not in ("ok", "detail")}
        if verdict["ok"] is not True:
            node.get_logger().warn(f"[{record[-1]['phase']}] {verdict['detail']}")
        Path(args.log).write_text(json.dumps(record, indent=2), encoding="utf-8")

    def capture_grasp_baseline() -> None:
        """A) 仿真抓取基线，必须在 grasp_confirmed 发布之后立即取。

        sim_carry_workpiece 收到 grasp_confirmed 才开始跟随，此刻的 工具→工件
        相对位姿就是后续阶段应保持的夹持关系。真机不需要（真机是物理夹紧，
        判据仍由 verify() 调 verify_pose.py）。
        """
        record_verdict(node.sim_grasp_baseline())

    def step(name: str, ok: bool, detail: str = "") -> None:
        record.append({"phase": name, "ok": ok, "detail": detail,
                       "tcp": current_tcp().round(4).tolist(),
                       "plan_only": plan_only})
        publish_task_log(node, f"[{name}] {'OK' if ok else 'FAIL'} {detail}")
        # 面板的"当前流程/当前阶段"靠这个实时更新（以前只在开头结尾发一次）
        publish_task_state(node, {"flow": getattr(cfg, "scope", "") or args.flow,
                                  "phase": name, "ok": ok, "detail": detail})
        # 仿真：阶段验收换成工件判据（工件跟着走 / 进插槽）。放在这里而不是各个
        # 阶段分支里，是为了让搬运链上每个阶段都得到同一条判据，而真机路径
        # （cfg.world != "sim"）一行都不执行。verification.enabled=false 时两边
        # 都不判，行为一致。
        if ok and not plan_only and cfg.verify and cfg.world == "sim":
            verdict = node.sim_workpiece_acceptance(name, cfg)
            if verdict is not None:
                record_verdict(verdict)
                if verdict["ok"] is False:
                    # D) 判据不过 ⇒ 该阶段 ok=false，run_task 整体返回非 0
                    record[-1]["ok"] = False
                    record[-1]["detail"] = ((detail + " | " if detail else "")
                                            + verdict["detail"])
        ok = record[-1]["ok"]
        notify_recorder(name, "exit")
        publish_task_state(node, {"flow": getattr(cfg, "scope", "") or args.flow,
                                  "mode": getattr(cfg, "mode", ""),
                                  "world": getattr(cfg, "world", "real"),
                                  "phase": name, "phase_index": index + 1,
                                  "phase_total": len(phases),
                                  "status": "ok" if ok else "failed",
                                  "detail": record[-1]["detail"],
                                  "plan_only": plan_only})
        node.get_logger().info(f"[{name}] ok={ok} {record[-1]['detail']}")
        Path(args.log).write_text(json.dumps(record, indent=2), encoding="utf-8")
        # --until：跑到指定阶段就停（这个 helper 之前漏了，导致 --until 不生效）
        if args.until and name == args.until:
            raise SystemExit(f"stopped after requested phase {name}")

    def detail(goal, points) -> str:
        """Say why a phase failed instead of a bare ok=False."""
        if goal is None:
            return "IK_FAILED"
        if points is None:
            return "PLAN_FAILED"
        duration = points[-1]["t"] if points else 0.0
        return f"points={len(points)} dur={duration:.1f}s"

    def deliver(points: list[dict] | None, profile: SpeedProfile) -> bool:
        """Execute a planned phase, or (plan-only) advance the simulated state."""
        if not points:
            return False
        if plan_only:
            node.advance_sim(points)
            return True
        return node.run_chunks(points, profile.chunks, profile.time_scale,
                               cfg.dwell_sec)

    def verify(name: str, xyz, quat) -> bool:
        """Measured-pose acceptance after an executed phase (skipped in plan-only).

        Returns True when the measurement is inside tolerance.  A failure now also
        turns the phase record into ``ok=False``: before 2026-10-26 it only logged
        a warning, so a run whose insert ended 50 mm off still exited 0 -- the one
        promise "safe mode = every phase measured afterwards" was not kept.
        """
        if plan_only or not cfg.verify or not record:
            return True
        if cfg.world == "sim":
            # 仿真里没有 /tcp_pose_broadcaster/pose（那是真机驱动一侧的控制器），
            # verify_pose.py 只会打印 NO_POSE。仿真的验收改在 step() 里按工件位姿
            # 做（见 sim_workpiece_acceptance / capture_grasp_baseline）。
            return True
        ok, text = node.verify_stage(name, xyz, quat, cfg)
        record[-1]["verified"] = ok
        record[-1]["verify_detail"] = text
        if not ok:
            record[-1]["ok"] = False
            record[-1]["detail"] = ((record[-1]["detail"] + " | ")
                                    if record[-1]["detail"] else "") + text
            node.get_logger().error(
                f"[{name}] measured pose outside tolerance: {text} "
                "(phase marked failed; the flow stops here)")
        Path(args.log).write_text(json.dumps(record, indent=2), encoding="utf-8")
        return ok

    seed = None
    if plan_only:
        if args.start_joints:
            seed = [float(v) for v in args.start_joints.split(",") if v.strip()]
            if len(seed) != len(JOINTS):
                raise SystemExit(f"--start-joints needs {len(JOINTS)} values, got {len(seed)}")
            node.get_logger().info(
                f"plan-only: starting from --start-joints {np.round(seed, 3).tolist()}")
        else:
            seed = node.arm_positions()
            node.get_logger().info(
                f"plan-only: starting from measured joints {np.round(seed, 3).tolist()}")
        node.sim_q = list(seed)

    phases = resolve_phase_list(cfg)
    if wanted:
        known = {name for name, _kind in phases}
        unknown = [name for name in wanted if name not in known]
        if unknown:
            # A typo used to skip every phase, leave the record empty and return 0,
            # which looked exactly like a successful run.
            raise SystemExit(f"--phases {unknown} not in this flow; allowed: "
                             f"{sorted(known)}")
    if not args.confirm and not plan_only:
        node.get_logger().warn("dry run: pass --confirm to execute motion "
                               "(or --plan-only to plan without moving)")
        for name, kind in phases:
            if wanted and name not in wanted:
                continue
            node.get_logger().info(f"planned phase {name} ({kind})")
        return 0

    start_positions = lambda: list(node.sim_q) if plan_only and node.sim_q else None  # noqa: E731

    for index, (name, kind) in enumerate(phases):
        if wanted and name not in wanted:
            continue
        notify_recorder(name, "enter")
        publish_task_state(node, {"flow": getattr(cfg, "scope", "") or args.flow,
                                  "mode": getattr(cfg, "mode", ""),
                                  "world": getattr(cfg, "world", "real"),
                                  "phase": name, "phase_index": index + 1,
                                  "phase_total": len(phases), "status": "running",
                                  "plan_only": plan_only})
        if not plan_only and not node.ensure_control_ready():
            step(name, False, "control chain unhealthy")
            break
        records_before = len(record)
        try:
            if kind == "special":
                if plan_only:
                    step(name, True, "gripper phase skipped (plan-only)")
                    continue
                if name == "gripper_init":
                    # 真机 init 是 ~10 s 的校准动作；以前只发命令就记成功，夹爪停在
                    # init=1 时下一条 grip 会把手指停在中途（10-23 的事故就是这几步）。
                    ready = node.ensure_gripper_initialized()
                    step(name, ready, "" if ready else "gripper init did not reach 5")
                elif name == "grip":
                    # 涨紧夹持：init 已把手指闭合归零，进入抓取位后再张开到 grip_width，
                    # 靠"从内侧撑开"夹住手柄（示教模式的 [7] open to grip width 同义）。
                    gripped = node.grip_and_wait(cfg.grip_width_mm, cfg.grip_current)
                    position = node.gripper("status").get("position_mm")
                    # 示教经验（2026-11-12 所有者确认）：必须张开到 50 mm 才能完全撑住扶手，
                    # 所以判据用"接近目标开度"，而不是历史遗留的 40 mm。
                    held = (position is not None
                            and position >= float(cfg.grip_width_mm) - 5.0)
                    if gripped and not held:
                        # 手指没到位 = 没撑到工件内侧，继续抬臂会把工件留在原地
                        gripped = False
                    step(name, gripped,
                         f"张开涨紧 {cfg.grip_width_mm} mm（实测 {position} mm）"
                         if gripped else f"grip did not hold (position {position} mm)")
                else:  # release
                    # 释放 = 张开手指，必须由所有者当次授权（与 unload 的 d8 同一门槛）。
                    if not getattr(args, "allow_release", False):
                        step(name, False,
                             "refused: gripper release requires --allow-release "
                             "(owner approval)")
                        break
                    node.gripper("grip", max(0.0, cfg.grip_width_mm - 20))
                    node.gripper("grip", 20.0)
                    node.gripper("init")
                    # init 完成才代表手指收拢/归位；以前发完就往下走 withdraw，
                    # 手指还撑在工件里时向后拉会把工件/夹具一起拖。
                    released = node.wait_gripper_init()
                    node._grasp_event.publish(String(data="release_confirmed"))
                    step(name, released,
                         "" if released else "release did not confirm init=5")
                if not record[-1]["ok"]:
                    break
            elif name == "pre_grasp":
                if cfg.world == "real" and not plan_only:
                    ensure_workpiece_collision()
                # 抓取点先算出来：压垂直 + 沿工具 x 回退（与示教流程一致），
                # 预抓取点取"正上方 150 mm"，而不是配置里那个横向 120 mm 的旧位姿。
                grasp_xyz, grasp_quat, pre, note = grip_targets()
                profile = profile_for(cfg.speeds, name, cfg.velocity_scale,
                                      cfg.time_scale, 6)
                here = current_tcp()
                distance = float(np.linalg.norm(pre - here)) if here is not None else 1.0
                velocity = proximity_velocity(distance, min(profile.velocity, 0.02))
                goal = node.ik(pre, grasp_quat, seed=start_positions())
                points = (node.plan_joint(goal, velocity, start_positions())
                          if goal else None)
                ok = deliver(points, profile)
                step(name, ok, f"v={velocity:.3f} ts={profile.time_scale}"
                               f" 上方{APPROACH_HEIGHT_M*1000:.0f}mm {detail(goal, points)}")
                if ok:
                    ok = verify(name, pre, grasp_quat)
            elif name == "grasp":
                # 装配段不用相机测量，但所有者要盯着看 → 切到 640×480 高帧率
                if getattr(cfg, "camera_fast_during_assembly", True) and not plan_only:
                    ok_fast = switch_camera_mode("fast")
                    node.get_logger().info(f"[camera] 切到 fast 档 {'成功' if ok_fast else '失败'}")
                grasp_xyz, grasp_quat, _pre, _note = grip_targets()
                profile = profile_for(cfg.speeds, name, cfg.assembly_velocity_scale,
                                      cfg.time_scale, 4)
                here = current_tcp()
                distance = float(np.linalg.norm(grasp_xyz - here)) if here is not None else 1.0
                velocity = proximity_velocity(distance, min(profile.velocity, 0.01))
                goal = node.ik(grasp_xyz, grasp_quat, seed=start_positions())
                points = (node.plan_joint(goal, velocity, start_positions())
                          if goal else None)
                ok = deliver(points, profile)
                step(name, ok, f"v={velocity:.3f} ts={profile.time_scale}"
                               f" 回退{GRASP_CLEARANCE_M*1000:.0f}mm {detail(goal, points)}")
                if ok:
                    ok = verify(name, grasp_xyz, grasp_quat)
                if ok and not plan_only:
                    # 抓取点实测超差时绝不继续"夹住/抬臂"：以前这里不看 verify 结果，
                    # 偏 50 mm 也会照常发 grasp_confirmed 并把工件带上去。
                    node._grasp_event.publish(String(data="grasp_confirmed"))
                    if cfg.world == "sim":
                        # 等 carry 节点确认"真的焊住了"再取基线/继续抬臂：
                        # DetachableJoint 是按 attach 那一刻的相对位姿建关节，
                        # 若关节在抬臂中途才建立，工件会被留在桌上、夹持关系
                        # 带着那一段位移（跟随判据会假失败）。
                        node.wait_for_carry_state("attached")
                    if cfg.world == "sim" and cfg.verify:
                        # A) 抓取基线：必须在 attached 之后取（见上面
                        # capture_grasp_baseline 的说明）。真机走不到这里。
                        capture_grasp_baseline()
            elif kind == "measure":
                # 原地测量：NBV 已经把机械臂放到精观测位，这里不再移动，只跑测量检查
                target = name.split(":", 1)[1]
                check = (cfg.observe_checks or {}).get(target) or ""
                if not check:
                    step(name, False, f"MEASURE_CHECK_MISSING ({target})")
                    continue
                if plan_only:
                    step(name, True, f"measure {target} (planned only)")
                    continue
                ok_check, check_detail = run_observation_check(check, {})
                # 先 step()（它会 append 记录），再补 check 明细；
                # 之前先写 record[-1] 在"第一个阶段就是 measure"时会 IndexError。
                step(name, ok_check, f"measure {target}: {check_detail}")
                record[-1]["check"] = {"name": check, "ok": ok_check,
                                       "detail": check_detail}
                Path(args.log).write_text(json.dumps(record, indent=2), encoding="utf-8")
            elif kind == "nbv":
                # NBV = "把机械臂移到能拍到工件大部分的位置"，用已验证视角的相对关系推导：
                #   0° 标定过一个"能测准"的相机位姿 → 换成相对工件的位姿 →
                #   按当前工件转角重新落到世界系（scripts/nbv_from_reference.py）。
                # 这样既不需要搜索，也不会像转腕那样把相机甩出去。
                target = name.split(":", 1)[1]
                if plan_only:
                    step(name, True, f"NBV {target}（plan-only 跳过换位）")
                    continue
                reference_cmd = [sys.executable, "-u",
                                 str(REPO / "scripts" / "nbv_from_reference.py"),
                                 "--reference-measurement",
                                 str(REPO / "runtime" / "measurements" / "workpiece_zero.json"),
                                 "--current-measurement",
                                 str(REPO / "runtime" / "measurements" / "workpiece_coarse.json"),
                                 "--view-key", "0", "--execute"]
                code, text = run(reference_cmd, timeout=900.0)
                tail = (text or "").strip().splitlines()
                detail = tail[-1] if tail else "no output"
                step(name, code == 0, f"NBV {target}: {detail}")
            elif kind == "observe":
                waypoint = name.split(":", 1)[1]
                pose = (cfg.observe or {}).get(waypoint)
                if not pose:
                    step(name, False, f"OBSERVE_POSE_MISSING ({waypoint})")
                    continue
                xyz = np.asarray(pose["xyz"], dtype=float)
                quat = tuple(float(v) for v in pose["quat"])
                profile = profile_for(cfg.speeds, waypoint, cfg.velocity_scale,
                                      cfg.time_scale, 3)
                camera_before = node.camera_stamp_ns()
                goal = node.ik(xyz, quat, seed=start_positions())
                points = (node.plan_joint(goal, profile.velocity, start_positions())
                          if goal else None)
                ok = deliver(points, profile)
                # detail() already distinguishes IK_FAILED from PLAN_FAILED; the
                # old "if ok else PLAN_FAILED" hid every IK failure behind a plan
                # failure and made the first simulated runs look unsolvable.
                detail_text = (f"observe {waypoint} v={profile.velocity} "
                               f"{detail(goal, points)}")
                step(name, ok, detail_text)
                if not ok:
                    continue
                if plan_only:
                    continue
                # Wait for a frame taken AFTER the move instead of sleeping and
                # hoping.  The camera runs at ~1 fps; without this the process
                # image and the check could be computed on the pre-move view.
                if cfg.world == "real":
                    fresh = node.wait_camera_frame_after(camera_before)
                    if not fresh:
                        node.get_logger().warn(
                            f"[{name}] no camera frame newer than the motion within "
                            "10 s; the check below may look at an old view")
                # 机械臂惯性大：到位后先等它稳定再拍，否则拍到的是运动中/刚停下的帧
                settle = float(getattr(cfg, "observe_settle_sec", 10.0) or 0.0)
                if settle > 0:
                    node.get_logger().info(f"[{name}] 等待 {settle:.0f} s 稳定后拍摄")
                    time.sleep(settle)
                if cfg.process_images:
                    capture_process_image(f"observe:{waypoint}", "enter")
                override = (cfg.observe_checks or {}).get(waypoint)
                # 约定：覆盖值写 "none" 表示这个观测位不做检查
                if isinstance(override, str) and override.strip().lower() in ("none", "skip"):
                    check = ""
                else:
                    check = (override or pose.get("check") or "").strip()
                if check and cfg.world == "sim":
                    # 仿真里只走流程，不真正跑检查。理由同 observe_all：
                    # Gazebo 渲染无贴图 → YOLO 必然 "no mask"；而仿真的规划用的是
                    # 已知的正确位姿（common.yaml 的常量 = 真机实测目标），不依赖视觉。
                    record[-1]["check"] = {
                        "name": check,
                        "ok": True,
                        "detail": "skipped in sim (planning uses the known ground-truth poses)",
                    }
                    Path(args.log).write_text(json.dumps(record, indent=2),
                                              encoding="utf-8")
                    node.get_logger().info(f"[{name}] check {check} skipped in sim")
                elif check:
                    ok_check, check_detail = run_observation_check(check, pose)
                    record[-1]["check"] = {"name": check, "ok": ok_check,
                                           "detail": check_detail}
                    Path(args.log).write_text(json.dumps(record, indent=2),
                                              encoding="utf-8")
                    node.get_logger().info(
                        f"[{name}] check {check} ok={ok_check} {check_detail}")
                    # 真机侧仍然是阻断判据。
                    if not ok_check:
                        step(name, False, f"check {check} failed: {check_detail}")
                        break
            elif kind == "joint":
                target = {"transit": (cfg.transit, cfg.grasp_q),
                          "staging": (cfg.staging, cfg.pre_insert_q),
                          "baseline": (cfg.baseline, cfg.baseline_q)}[name]
                profile = profile_for(cfg.speeds, name, cfg.velocity_scale, 1.5, 3)
                goal = node.ik(target[0], target[1], seed=start_positions())
                points = (node.plan_joint(goal, profile.velocity, start_positions())
                          if goal else None)
                ok = deliver(points, profile)
                step(name, ok, f"v={profile.velocity} ts={profile.time_scale}"
                               f" {detail(goal, points)}")
            elif kind == "cartesian":
                if name == "lift":
                    xyz = current_tcp() + np.array([0, 0, cfg.lift_total_mm / 1000.0])
                    quat = cfg.grasp_q
                    speed = cfg.assembly_velocity_scale
                elif name == "pre_insert":
                    xyz, quat, speed = cfg.pre_insert, cfg.pre_insert_q, cfg.assembly_velocity_scale
                elif name == "insert":
                    xyz, quat, speed = cfg.insert, cfg.pre_insert_q, cfg.assembly_velocity_scale
                elif name == "straighten":
                    xyz, quat, speed = cfg.insert, cfg.insert_q, cfg.assembly_velocity_scale
                else:  # withdraw
                    xyz = current_tcp() - cfg.advance_axis * (cfg.safe_clear_mm / 1000.0)
                    quat, speed = cfg.insert_q, cfg.assembly_velocity_scale
                profile = profile_for(cfg.speeds, name, speed,
                                      cfg.time_scale, cfg.max_chunks)
                # The handle band is defined along the insertion axis, so the
                # same band serves the retreat (distance grows) and the re-entry
                # (distance shrinks).
                band = (cfg.insert, cfg.advance_axis)
                # Inside the slot the per-side clearance (~1 mm) is smaller than the
                # collision-mesh error, so a collision check would reject a motion the
                # real cell performs every time (the slot walls guide it).  Tight
                # phases are therefore planned without collision checking; every other
                # phase keeps it on.
                tight = _no_collision_check(name)
                if plan_only:
                    legs = build_cartesian_legs(current_tcp(), np.asarray(xyz, float),
                                                profile, band)
                    total, count, ok = 0.0, 0, True
                    for end_point, velocity, time_scale, chunks in legs:
                        points = node.plan_cartesian(end_point, quat, velocity,
                                                     start_positions(),
                                                     avoid_collisions=not tight)
                        if not points:
                            ok = False
                            break
                        node.advance_sim(points)
                        total += points[-1]["t"]
                        count += len(points)
                    if not ok:
                        step(name, False, "CARTESIAN_PLAN_FAILED "
                                          f"(target {np.round(np.asarray(xyz), 3).tolist()})")
                        continue
                    step(name, True, f"points={count} dur={total:.1f}s "
                                     f"legs={len(legs)} band={profile.slow_band_mm}")
                else:
                    ok = run_cartesian_profile(node, name, xyz, quat, profile,
                                               cfg.dwell_sec, node.get_logger(),
                                               band=band, tight=tight)
                    step(name, ok, f"v={profile.velocity} ts={profile.time_scale}"
                                   f" tail={profile.slow_tail_mm}mm"
                                   f" band={profile.slow_band_mm}")
                if ok and name in ("insert", "straighten"):
                    verify(name, xyz, quat)
            else:
                step(name, False, "unknown phase")
        except Exception as exc:  # noqa: BLE001 - recorded, then stop
            step(name, False, f"{type(exc).__name__}: {exc}")
            break
        if len(record) > records_before and not record[-1]["ok"]:
            node.get_logger().error(
                f"[{name}] phase failed; stopping the flow instead of moving on "
                "with a wrong result (the remaining phases would act on a wrong state)")
            break

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    print(json.dumps(record[-1] if record else {}, indent=2))
    # An empty record is not success: it means nothing ran (for example --phases
    # matched nothing).  ``all([])`` is True, which used to return 0.
    return 0 if record and all(item["ok"] for item in record) else 1


if __name__ == "__main__":
    raise SystemExit(main())
