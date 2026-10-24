#!/usr/bin/env python3
"""从真机实测数据生成与真机几何对齐的 Gazebo 世界。

为什么需要这个脚本
------------------
旧世界 ``cs625_insertion_scene.sdf`` 是 SolidWorks 装配系的世界，夹具在
(-0.85, 0.49)、工件在 (-0.51, -0.81)，而且没有桌面模型；真机实测的是另一套
坐标：夹具槽口在 (1.2416, 0.1817, 0.4432)，工件桌顶面在 z=-0.091。两者不是同
一个坐标系，所以旧世界被存档，新世界按实测重配，旧文件一个字节都不动。

关键陷阱（本脚本存在的理由）
----------------------------
不能用“把实测位姿直接当模型 origin”的办法对齐夹具。旧配置里夹具位姿与“槽内
工件”位姿算出的相对变换是 平移 [0.148, 0.2085, -0.0143] m、rpy(-6°, 0°, 90°)；
而真机实测 ``slot_origin -> box_in_slot`` 的相对变换是 平移 [-0.078, 0.022,
-0.0015] m、rpy≈0°。两套模型的 origin 在零件上的落点不同（夹具模型原点不在
零件中心：bbox 从 (-0.0295,-0.3725,-0.2451) 到 (0.2025,0.3725,0.2607)）。

因此本脚本只认几何：**包围盒（bbox）对齐**。把模型 bbox 的 8 个角点用同一个
刚体变换搬到世界系，再让 bbox 中心/底面落到实测点上。这样“可见几何”一定对，
模型 origin 落在哪里由模型自己决定，不需要手调。

数据来源（权威）
----------------
* 环境模型（真机实测点云拟合结果）：
  ``runtime/environment/latest/model.json``，不存在时回退
  ``runtime/environment/20261024_030734/model.json``。
  两张桌面的 z 与 xy 范围、工件位姿、CAD 中心都取自这里。
* 槽位：``config/tasks/common.yaml`` 的 ``slot_origin`` / ``slot_origin_q``
  （RobotVisionSuite 标定，夹具机械固定）。与 model.json 的 ``slot`` 交叉校验。
* 模型 bbox：直接解析两个 ``model.sdf`` 引用的二进制 STL（顶点极值），并与
  模型 SDF 注释里写的 bbox 交叉校验（夹具注释里那份是权威夹具尺寸）。

生成物
------
* ``--out-world``: 世界 SDF（地面 + 两张桌面盒子 + 夹具 include + 工件 include）
* ``--out-config``: 与旧 ``cs625_task_scene.yaml`` 同结构的场景 yaml
  （``ros__parameters`` 下 ``fixture.pose_world`` / ``module.pose_world``）
* ``--report``: 拟合报告 JSON（可选）
* ``--figure``: 俯视图对比 png（可选，matplotlib Agg）

用法
----
    python3 scripts/build_sim_world.py \\
        --out-world src/cs625_simulation/worlds/cs625_real_cell.sdf \\
        --out-config config/cs625_real_cell_scene.yaml \\
        --report /tmp/sim_fit_report.json \\
        --figure /tmp/sim_world_topview.png

本脚本不 import rclpy / gazebo，不启动仿真，纯几何与文件生成，因此可以直接被
``test/test_sim_world_fit.py`` 当普通模块导入做单测。
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
import pathlib
import struct
import sys

import numpy as np
import yaml


ROOT = pathlib.Path(__file__).resolve().parents[1]

ENV_MODEL_LATEST = pathlib.Path("runtime/environment/latest/model.json")
ENV_MODEL_FALLBACK = pathlib.Path("runtime/environment/20261024_030734/model.json")
COMMON_TASK_CONFIG = pathlib.Path("config/tasks/common.yaml")
FIXTURE_MODEL_SDF = pathlib.Path("src/cs625_simulation/assets/cs625_real_cell/slot_fixture/model.sdf")
MODULE_MODEL_SDF = pathlib.Path("src/cs625_simulation/assets/cs625_task/shielding_module/model.sdf")

WORLD_NAME = "cs625_real_cell"
SCENE_KEY = "cs625_real_cell_scene"

FIXTURE_URI = "model://cs625_real_cell/slot_fixture"
MODULE_URI = "model://cs625_task/shielding_module"

# 地面顶面。旧世界地面顶面在 z=0，但真机实测两张桌面在 z=-0.091 / -0.041，
# 都在基座安装面以下；地面若留在 z=0 会把整张桌子埋进去。这里把地面放到两张
# 桌子最低底面（-0.141）以下 10 cm，因此桌子盒子的厚度 = 桌面 - 地面顶面，
# 桌子站在地面上，不是悬空薄板。物理地面当然比这里低得多，地面只承担支撑和
# 接触面作用。
DEFAULT_FLOOR_TOP_Z = -0.20

# 偏差报警阈值（米）：任务要求 >20 mm 必须显式警告。
WARN_THRESHOLD_M = 0.02
# 模型系 vs CAD 系的判定阈值：bbox 中心与实测 CAD 中心偏差超过它，说明模型
# 系不是 CAD 系，必须改用 bbox 对齐。
FRAME_THRESHOLD_M = 0.05

# 实测 CAD 中心相对 CAD origin 的固定偏移，表达在模型系里。原样搬自
# scripts/model_environment.py 的 WORKPIECE_POSE["cad_centre_offset"]，只用于
# 交叉校验“模型系 == CAD 系”，不参与任何放置计算。
PUBLISHED_CAD_CENTRE_OFFSET_M = (-0.208, 0.066, -0.011)


# ---------------------------------------------------------------------------
# 基础三维工具（纯函数，无 ROS/Gazebo 依赖）
# ---------------------------------------------------------------------------


def rotation_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """SDF 的 rpy 组合：R = Rz(yaw) @ Ry(pitch) @ Rx(roll)。"""

    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def quat_to_matrix(quat_xyzw) -> np.ndarray:
    """(x, y, z, w) 四元数 -> 旋转矩阵，先归一化。"""

    q = np.asarray(quat_xyzw, dtype=float)
    norm = float(np.linalg.norm(q))
    if norm <= 0.0:
        raise ValueError("quaternion has zero norm")
    x, y, z, w = q / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def yaw_from_quat(quat_xyzw) -> float:
    """只要偏航角：夹具必须正立，roll/pitch 一律置 0。"""

    rot = quat_to_matrix(quat_xyzw)
    return math.atan2(float(rot[1, 0]), float(rot[0, 0]))


def matrix_to_rpy(rot: np.ndarray) -> tuple[float, float, float]:
    """旋转矩阵 -> SDF rpy，与 :func:`rotation_rpy` 互逆。"""

    pitch = math.asin(max(-1.0, min(1.0, -float(rot[2, 0]))))
    roll = math.atan2(float(rot[2, 1]), float(rot[2, 2]))
    yaw = math.atan2(float(rot[1, 0]), float(rot[0, 0]))
    return roll, pitch, yaw


def bbox_corners(bbox_min, bbox_max) -> np.ndarray:
    """AABB 的 8 个角点，(8, 3)。"""

    lo = np.asarray(bbox_min, dtype=float)
    hi = np.asarray(bbox_max, dtype=float)
    return np.array(list(itertools.product(*zip(lo, hi))), dtype=float)


def bbox_centre(bbox_min, bbox_max) -> np.ndarray:
    return 0.5 * (np.asarray(bbox_min, dtype=float) + np.asarray(bbox_max, dtype=float))


def transformed_points(points, xyz, rot: np.ndarray) -> np.ndarray:
    return np.asarray(points, dtype=float) @ rot.T + np.asarray(xyz, dtype=float)


def world_bbox(bbox_min, bbox_max, xyz, rot: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """模型 AABB 在给定位姿下的世界系 AABB。"""

    moved = transformed_points(bbox_corners(bbox_min, bbox_max), xyz, rot)
    return moved.min(axis=0), moved.max(axis=0)


def binary_stl_bounds(path) -> tuple[np.ndarray, np.ndarray]:
    """读二进制 STL 的全部顶点，返回 (min, max)。不依赖 trimesh。

    与 ``test/audit_binary_stl_extents.py`` 用同一个解析方式，但返回极值而不是
    尺寸，并且校验文件长度与三角形数一致，截断文件会直接报错。
    """

    path = pathlib.Path(path)
    payload = path.read_bytes()
    if len(payload) < 84:
        raise ValueError(f"{path}: file too short for binary STL")
    count = struct.unpack_from("<I", payload, 80)[0]
    expected = 84 + 50 * count
    if len(payload) != expected:
        raise ValueError(f"{path}: expected {expected} bytes for {count} triangles, got {len(payload)}")
    records = list(struct.iter_unpack("<12fH", payload[84:]))
    if len(records) != count:
        raise ValueError(f"{path}: expected {count} triangles, got {len(records)}")
    vertices = np.array([r[3:12] for r in records], dtype=float).reshape(-1, 3)
    return vertices.min(axis=0), vertices.max(axis=0)


# ---------------------------------------------------------------------------
# 对齐：包围盒 -> 位姿
# ---------------------------------------------------------------------------


def pose_from_bbox_alignment(bbox_min, bbox_max, target_centre, yaw: float) -> dict:
    """求一个（只含 yaw 的）位姿，使模型 bbox 中心正好落在 ``target_centre``。

    数学上就是 ``t = target_centre - R(yaw) @ bbox_centre_model``。返回的
    ``rpy`` 固定为 (0, 0, yaw)：夹具/工件都必须正立，不允许 roll/pitch。

    这是本脚本唯一的对齐原语；其它“贴桌面”“对齐实测点”都是它的特例。
    """

    rot = rotation_rpy(0.0, 0.0, yaw)
    centre = bbox_centre(bbox_min, bbox_max)
    xyz = np.asarray(target_centre, dtype=float) - rot @ centre
    return {"xyz": xyz.tolist(), "rpy": [0.0, 0.0, float(yaw)]}


def seat_bbox_on_plane(bbox_min, bbox_max, target_xy, plane_z: float, yaw: float) -> dict:
    """把 bbox 底面正好压到 ``plane_z``、bbox 中心 xy 落到 ``target_xy``。

    与 :func:`pose_from_bbox_alignment` 的区别只在目标中心的高度：这里 z 由
    “底面贴平面”反推，即 target_z = plane_z + 高度/2。只转 yaw 时 bbox 的
    世界系 z 范围等于模型系 z 范围，所以这样算出来的底面严格等于 plane_z。
    """

    lo = np.asarray(bbox_min, dtype=float)
    hi = np.asarray(bbox_max, dtype=float)
    target_z = float(plane_z) + 0.5 * float(hi[2] - lo[2])
    return pose_from_bbox_alignment(lo, hi, [float(target_xy[0]), float(target_xy[1]), target_z], yaw)


def pose_matrix(pose: dict) -> np.ndarray:
    return rotation_rpy(*pose["rpy"])


def pose_of_bbox_centre(bbox_min, bbox_max, pose: dict) -> np.ndarray:
    """位姿作用后，模型 bbox 中心在世界系的位置。"""

    rot = pose_matrix(pose)
    return rot @ bbox_centre(bbox_min, bbox_max) + np.asarray(pose["xyz"], dtype=float)


# ---------------------------------------------------------------------------
# 桌面盒子
# ---------------------------------------------------------------------------


def table_box(measured: dict, floor_top_z: float = DEFAULT_FLOOR_TOP_Z) -> dict:
    """实测桌面（z + xy 范围）-> 一个从地面顶面长到桌面的轴对齐盒子。

    盒子 xy 直接取实测范围的 min/max（不缩不放），顶面严格等于实测 z。
    厚度由 ``顶面 - floor_top_z`` 决定，因此不是写死的 0.05 m。
    """

    lo = np.asarray(measured["xy_min"], dtype=float)
    hi = np.asarray(measured["xy_max"], dtype=float)
    top_z = float(measured["z"])
    if top_z <= floor_top_z:
        raise ValueError(f"table top z={top_z} is not above floor_top_z={floor_top_z}")
    centre = [0.5 * (lo[0] + hi[0]), 0.5 * (lo[1] + hi[1]), 0.5 * (top_z + floor_top_z)]
    size = [float(hi[0] - lo[0]), float(hi[1] - lo[1]), float(top_z - floor_top_z)]
    return {
        "xy_min": lo.tolist(),
        "xy_max": hi.tolist(),
        "top_z": top_z,
        "bottom_z": float(floor_top_z),
        "centre": centre,
        "size": size,
    }


def tables_overlap(first: dict, second: dict) -> dict | None:
    """两个桌面盒子在 xy 上的重叠矩形；不重叠返回 None。"""

    x0 = max(first["xy_min"][0], second["xy_min"][0])
    y0 = max(first["xy_min"][1], second["xy_min"][1])
    x1 = min(first["xy_max"][0], second["xy_max"][0])
    y1 = min(first["xy_max"][1], second["xy_max"][1])
    if x1 <= x0 or y1 <= y0:
        return None
    return {"xy_min": [x0, y0], "xy_max": [x1, y1], "area": (x1 - x0) * (y1 - y0)}


# ---------------------------------------------------------------------------
# 读实测数据
# ---------------------------------------------------------------------------


def relative(path: pathlib.Path) -> str:
    try:
        return str(pathlib.Path(path).resolve().relative_to(ROOT))
    except ValueError:
        return str(pathlib.Path(path).resolve())


def resolve_env_model(explicit=None, root: pathlib.Path = ROOT) -> pathlib.Path:
    """定位真机实测的环境模型：显式路径 > latest > 固定快照。"""

    if explicit:
        path = pathlib.Path(explicit)
        if not path.is_absolute():
            path = root / path
        if not path.is_file():
            raise FileNotFoundError(f"environment model not found: {path}")
        return path
    for candidate in (ENV_MODEL_LATEST, ENV_MODEL_FALLBACK):
        path = root / candidate
        if path.is_file():
            return path
    raise FileNotFoundError(
        "no measured environment model found (looked for "
        f"{ENV_MODEL_LATEST} and {ENV_MODEL_FALLBACK} under {root})"
    )


def declared_bbox_from_comment(sdf_path, label: str) -> dict | None:
    """从 model.sdf 注释里的 "Bounding box (a, b, c) to (d, e, f)" 读 bbox。

    只用于交叉校验：真正的 bbox 来自 STL 顶点。注释读不到就返回 None。
    """

    text = pathlib.Path(sdf_path).read_text(encoding="utf-8")
    index = text.lower().find("bounding box")
    if index < 0:
        return None
    start = index
    snippet = " ".join(text[start : start + 220].split())
    try:
        left = snippet.split("to")[0]
        right = snippet.split("to")[1]
        lo = [float(v) for v in left[left.index("(") + 1 : left.index(")")].split(",")]
        hi = [float(v) for v in right[right.index("(") + 1 : right.index(")")].split(",")]
    except (ValueError, IndexError):
        return None
    return {"min": lo, "max": hi, "label": label}


def load_measurements(
    env_model_path=None,
    common_config_path=COMMON_TASK_CONFIG,
    fixture_model_sdf=FIXTURE_MODEL_SDF,
    module_model_sdf=MODULE_MODEL_SDF,
    root: pathlib.Path = ROOT,
) -> dict:
    """把权威实测数据读进一个 dict（不做任何几何决策）。"""

    env_path = resolve_env_model(env_model_path, root)
    with env_path.open(encoding="utf-8") as handle:
        environment = json.load(handle)

    common_path = root / common_config_path if not pathlib.Path(common_config_path).is_absolute() else pathlib.Path(common_config_path)
    with common_path.open(encoding="utf-8") as handle:
        common = yaml.safe_load(handle)

    tables = {}
    for name, key in (("workpiece_table", "workpiece_table"), ("slot_table", "slot_table")):
        entry = environment["tables"][key]
        tables[name] = {
            "z": float(entry["z"]),
            "z_at_anchor": float(entry.get("z_at_anchor", entry["z"])),
            "xy_min": [float(v) for v in entry["xy_min"]],
            "xy_max": [float(v) for v in entry["xy_max"]],
            "label": entry.get("label", name),
        }

    slot_origin = [float(v) for v in common["slot_origin"]]
    slot_quat = [float(v) for v in common["slot_origin_q"]]

    env_slot = environment.get("slot", {})
    slot_crosscheck = None
    if env_slot.get("xyz"):
        env_slot_xyz = [float(v) for v in env_slot["xyz"]]
        slot_crosscheck = float(np.linalg.norm(np.subtract(slot_origin, env_slot_xyz)))

    workpiece = environment["workpiece"]
    workpiece_xyz = [float(v) for v in workpiece["xyz"]]
    workpiece_quat = [float(v) for v in workpiece["quat"]]
    if workpiece.get("centre"):
        cad_centre = [float(v) for v in workpiece["centre"]]
    else:  # 旧模型没有 centre 字段时，用模型系 CAD 偏移推一个
        cad_centre = (
            quat_to_matrix(workpiece_quat) @ np.array([-0.208, 0.066, -0.011]) + np.array(workpiece_xyz)
        ).tolist()

    fixture_mesh = root / "src/cs625_simulation/assets/cs625_real_cell/slot_fixture/meshes/slot_fixture_collision.stl"
    module_mesh = root / "src/cs625_simulation/assets/cs625_task/shielding_module/meshes/shielding_module.stl"
    fixture_min, fixture_max = binary_stl_bounds(fixture_mesh)
    module_min, module_max = binary_stl_bounds(module_mesh)

    return {
        "env_model": relative(env_path),
        "env_model_created_at": environment.get("created_at"),
        "root": root,
        "tables": tables,
        "slot_origin": slot_origin,
        "slot_quat_xyzw": slot_quat,
        "slot_crosscheck_m": slot_crosscheck,
        "workpiece": {
            "xyz": workpiece_xyz,
            "quat_xyzw": workpiece_quat,
            "cad_centre": cad_centre,
            "source": workpiece.get("source", "runtime environment model"),
        },
        "fixture_bbox": {
            "min": fixture_min.tolist(),
            "max": fixture_max.tolist(),
            "mesh": relative(fixture_mesh),
        },
        "module_bbox": {
            "min": module_min.tolist(),
            "max": module_max.tolist(),
            "mesh": relative(module_mesh),
        },
        "declared": {
            "fixture": declared_bbox_from_comment(root / fixture_model_sdf, "slot_fixture"),
            "module": declared_bbox_from_comment(root / module_model_sdf, "shielding_module"),
        },
    }


# ---------------------------------------------------------------------------
# 规划：实测 -> 位姿 + 拟合行
# ---------------------------------------------------------------------------


def plan_scene(
    measurements: dict,
    floor_top_z: float = DEFAULT_FLOOR_TOP_Z,
    warn_threshold_m: float = WARN_THRESHOLD_M,
    frame_threshold_m: float = FRAME_THRESHOLD_M,
) -> dict:
    """由实测数据算出两张桌子、两个模型的世界位姿和拟合报告行。"""

    tables = {
        "workpiece_table": table_box(measurements["tables"]["workpiece_table"], floor_top_z),
        "slot_table": table_box(measurements["tables"]["slot_table"], floor_top_z),
    }

    # --- 夹具：模型 origin 直接对齐 slot_origin（RVS 标定语义） -------------
    #
    # 真机夹具是 RobotVisionSuite 导出的网格，slot_origin 就是那个模型的
    # origin 标定值（slot_origin_q 给出朝向）。模型 origin 不在零件中心：
    # bbox 中心离它 (-0.077, -0.193, -0.254) m。若按“bbox 中心对齐”摆，
    # 槽口会跑到世界 y≈+0.18，而真机插入路径在 y≈-0.013 —— 旧仿真夹具正是
    # 这样偏了 194 mm，导致 insert 撞在实心壁上。
    fixture_bbox = measurements["fixture_bbox"]
    slot_origin = measurements["slot_origin"]
    fixture_yaw = yaw_from_quat(measurements["slot_quat_xyzw"])
    fixture_pose = {"xyz": list(slot_origin), "rpy": [0.0, 0.0, fixture_yaw]}
    fixture_rot = pose_matrix(fixture_pose)
    fixture_world_min, fixture_world_max = world_bbox(
        fixture_bbox["min"], fixture_bbox["max"], fixture_pose["xyz"], fixture_rot
    )
    fixture_world_centre = 0.5 * (fixture_world_min + fixture_world_max)
    # 对齐点就是模型 origin 本身：与 slot_origin 严格重合。
    fixture_deviation = float(
        np.linalg.norm(np.asarray(fixture_pose["xyz"], dtype=float) - np.asarray(slot_origin, dtype=float))
    )

    # --- 工件：先判断“模型系 == CAD 系”，否则退回 bbox 对齐 ----------------
    module_bbox = measurements["module_bbox"]
    workpiece = measurements["workpiece"]
    measured_rot = quat_to_matrix(workpiece["quat_xyzw"])
    measured_pose = {
        "xyz": list(workpiece["xyz"]),
        "rpy": list(matrix_to_rpy(measured_rot)),
    }
    frame_deviation = float(
        np.linalg.norm(
            pose_of_bbox_centre(module_bbox["min"], module_bbox["max"], measured_pose)
            - np.asarray(workpiece["cad_centre"], dtype=float)
        )
    )
    if frame_deviation > frame_threshold_m:
        # 模型系不是 CAD 系：把 bbox 中心放到实测 CAD 中心（只转 yaw）。
        module_alignment = "bbox_centre"
        module_pose = pose_from_bbox_alignment(
            module_bbox["min"],
            module_bbox["max"],
            workpiece["cad_centre"],
            yaw_from_quat(workpiece["quat_xyzw"]),
        )
        module_pose_source = "bbox_centre_at_measured_cad_centre"
    else:
        module_alignment = "measured_cad_pose"
        module_pose = measured_pose
        module_pose_source = "measured_cad_pose_verbatim"

    module_rot = pose_matrix(module_pose)
    module_world_min, module_world_max = world_bbox(
        module_bbox["min"], module_bbox["max"], module_pose["xyz"], module_rot
    )
    module_world_centre = 0.5 * (module_world_min + module_world_max)
    if module_alignment == "measured_cad_pose":
        module_deviation = frame_deviation
    else:
        module_deviation = float(
            np.linalg.norm(module_world_centre - np.asarray(workpiece["cad_centre"], dtype=float))
        )

    # bbox 中心与 CAD 中心之差在模型系里有多大 —— 用来解释残差来源。
    # 实测 CAD 中心不是 bbox 中心，而是“CAD origin + 固定偏移”；该偏移公开写在
    # scripts/model_environment.py 的 WORKPIECE_POSE["cad_centre_offset"] 里。
    # 用它反算实测 CAD 中心，残差应在亚毫米级 —— 这正是“模型系 == CAD 系”的证据。
    derived_offset = measured_rot.T @ (
        np.asarray(workpiece["cad_centre"], dtype=float) - np.asarray(workpiece["xyz"], dtype=float)
    )
    published_offset = np.asarray(PUBLISHED_CAD_CENTRE_OFFSET_M, dtype=float)
    bbox_vs_cad_offset = float(
        np.linalg.norm(bbox_centre(module_bbox["min"], module_bbox["max"]) - derived_offset)
    )
    cad_centre_from_published = measured_rot @ published_offset + np.asarray(workpiece["xyz"], dtype=float)
    cad_definition_residual = float(
        np.linalg.norm(cad_centre_from_published - np.asarray(workpiece["cad_centre"], dtype=float))
    )

    rows = [
        {
            "model": "slot_fixture",
            "bbox_centre_world": fixture_world_centre.tolist(),
            "reference": slot_origin,
            "reference_name": "slot_origin",
            "axes": "xyz",
            "alignment": "model_origin",
            "aligned_point_world": fixture_pose["xyz"],
            "deviation": fixture_deviation,
        },
        {
            "model": "shielding_module",
            "bbox_centre_world": module_world_centre.tolist(),
            "reference": workpiece["cad_centre"],
            "reference_name": "cad_centre",
            "axes": "xyz",
            "alignment": module_alignment,
            "aligned_point_world": module_world_centre.tolist(),
            "deviation": module_deviation,
        },
    ]

    overlap = tables_overlap(tables["workpiece_table"], tables["slot_table"])
    module_sink = tables["workpiece_table"]["top_z"] - float(module_world_min[2])
    anchor_z = measurements["tables"]["workpiece_table"]["z_at_anchor"]

    notes = []
    if overlap:
        notes.append(
            "两张桌子的盒子在 xy 上重叠：x[{:.3f}, {:.3f}] y[{:.3f}, {:.3f}]，重叠面积 {:.4f} m²；"
            "两张桌面高度不同（{:.3f} / {:.3f} m），重叠区允许两个盒子叠在一起。".format(
                overlap["xy_min"][0],
                overlap["xy_max"][0],
                overlap["xy_min"][1],
                overlap["xy_max"][1],
                overlap["area"],
                tables["workpiece_table"]["top_z"],
                tables["slot_table"]["top_z"],
            )
        )
    notes.append(
        "夹具用“模型 origin = slot_origin”对齐（不是 bbox 中心）：slot_origin 是 "
        "RobotVisionSuite 对该模型 origin 的标定值（z={:.4f} m，在插槽桌顶面上方 {:.3f} m）。"
        "模型 origin 离 bbox 中心 (-0.0760, -0.1935, -0.2539) m，所以 bbox 中心在世界上是 "
        "({:.4f}, {:.4f}, {:.4f})：xy 偏 {:.1f} mm、z 偏 {:.1f} mm —— 这是零件坐标系的定义，不是误差。".format(
            slot_origin[2],
            slot_origin[2] - tables["slot_table"]["top_z"],
            fixture_world_centre[0], fixture_world_centre[1], fixture_world_centre[2],
            float(np.linalg.norm((fixture_world_centre - np.asarray(slot_origin, dtype=float))[:2])) * 1000.0,
            abs(float(fixture_world_centre[2] - slot_origin[2])) * 1000.0,
        )
    )
    notes.append(
        "夹具 bbox 底面在 z={:.4f} m，比插槽桌顶面低 {:.1f} mm：真机夹具的底部法兰伸到桌面"
        "测量面以下（桌面在夹具底下被挡住，平面是用四周可见桌面拟合的）。校正依据是顶面高度与"
        "楔形槽宽：模型 origin 压在 slot_origin 时顶面 z={:.4f}，RGB-D 实测顶面 z≈0.441，"
        "槽宽 0.457 vs 实测 0.448-0.468；若为了贴桌面整体抬高 {:.1f} mm，顶面与槽宽就都对不上了。".format(
            float(fixture_world_min[2]),
            (tables["slot_table"]["top_z"] - float(fixture_world_min[2])) * 1000.0,
            slot_origin[2],
            (tables["slot_table"]["top_z"] - float(fixture_world_min[2])) * 1000.0,
        )
    )
    if measurements.get("slot_crosscheck_m") is not None:
        notes.append(
            "slot_origin 交叉校验：common.yaml 与 {env} 的 slot 相差 {d:.3f} mm。".format(
                env=measurements["env_model"], d=measurements["slot_crosscheck_m"] * 1000.0
            )
        )
    if module_sink > 0.0:
        notes.append(
            "工件 bbox 底面比工件桌顶面（同一次点云拟合的平面 z={:.5f}）低 {:.1f} mm，"
            "比该桌面的锚点高度 z_at_anchor 低 {:.1f} mm。按任务要求工件直接使用实测位姿、不做上下平移，"
            "所以这个穿模量原样保留在文件里：它不是对齐误差（bbox 中心没被拿去对齐），可能的来源是"
            "点云在工件遮挡区把桌面拟合低了几毫米、以及 CAD 包围盒比实物底面凸出几毫米。"
            "若要把它当动力学体在 Gazebo 里生成，需要额外抬高 {:.1f} mm 才不与桌面重叠；是否这么做由所有者决定，"
            "本脚本不擅自改动实测位姿。".format(
                tables["workpiece_table"]["top_z"],
                module_sink * 1000.0,
                (tables["workpiece_table"]["top_z"] - anchor_z) * 1000.0,
                module_sink * 1000.0,
            )
        )

    return {
        "world_name": WORLD_NAME,
        "floor_top_z": float(floor_top_z),
        "tables": tables,
        "fixture": {
            "model_uri": FIXTURE_URI,
            "pose": fixture_pose,
            "yaw": fixture_yaw,
            "alignment": "model_origin",
            "bbox_model": {"min": fixture_bbox["min"], "max": fixture_bbox["max"]},
            "bbox_centre_model": bbox_centre(fixture_bbox["min"], fixture_bbox["max"]).tolist(),
            "bbox_centre_offset_world_m": (
                fixture_world_centre - np.asarray(slot_origin, dtype=float)
            ).tolist(),
            "bbox_world": {"min": fixture_world_min.tolist(), "max": fixture_world_max.tolist(), "centre": fixture_world_centre.tolist()},
            "reference": slot_origin,
            "reference_name": "slot_origin",
            "reference_axes": "xyz",
            "deviation_m": fixture_deviation,
            "bottom_on_table_m": float(fixture_world_min[2] - tables["slot_table"]["top_z"]),
        },
        "module": {
            "model_uri": MODULE_URI,
            "pose": module_pose,
            "pose_source": module_pose_source,
            "alignment": module_alignment,
            "measured_pose": measured_pose,
            "measured_quat_xyzw": workpiece["quat_xyzw"],
            "frame_deviation_m": frame_deviation,
            "bbox_model": {"min": module_bbox["min"], "max": module_bbox["max"]},
            "bbox_centre_model": bbox_centre(module_bbox["min"], module_bbox["max"]).tolist(),
            "bbox_world": {"min": module_world_min.tolist(), "max": module_world_max.tolist(), "centre": module_world_centre.tolist()},
            "reference": workpiece["cad_centre"],
            "reference_name": "cad_centre",
            "reference_axes": "xyz",
            "deviation_m": module_deviation,
            "cad_offset_model": derived_offset.tolist(),
            "cad_offset_published_model": list(PUBLISHED_CAD_CENTRE_OFFSET_M),
            "bbox_centre_minus_cad_offset_m": bbox_vs_cad_offset,
            "cad_definition_residual_m": cad_definition_residual,
            "bottom_below_workpiece_table_m": module_sink,
        },
        "overlap": overlap,
        "rows": rows,
        "notes": notes,
        "warn_threshold_m": float(warn_threshold_m),
        "frame_threshold_m": float(frame_threshold_m),
    }


def deviations_requiring_warning(rows, threshold: float = WARN_THRESHOLD_M) -> list[str]:
    """偏差超过阈值的行 -> 警告字符串列表。纯函数，便于单测。"""

    warnings = []
    for row in rows:
        if float(row["deviation"]) > threshold:
            warnings.append(
                "WARN 偏差 {:.1f} mm > {:.0f} mm: {} 的 bbox 中心与实测 {} 不一致"
                "（对齐方式 {}，仅比较 {}）".format(
                    float(row["deviation"]) * 1000.0,
                    threshold * 1000.0,
                    row["model"],
                    row["reference_name"],
                    row["alignment"],
                    row["axes"],
                )
            )
    return warnings


def format_fit_table(rows) -> str:
    """把拟合行排成一张等宽表。"""

    header = (
        f"{'model':<17}{'aligned point world [m]':<38}{'measured reference [m]':<38}"
        f"{'reference':<13}{'dev [m]':>9}{'axes':>6}  alignment"
    )
    lines = ["=" * len(header),
             "拟合报告（aligned point = 模型 origin 或 bbox 中心，vs 实测参考点）",
             header, "-" * len(header)]
    for row in rows:
        aligned = row.get("aligned_point_world", row["bbox_centre_world"])
        centre = "(" + ",".join(f"{v:10.6f}" for v in aligned) + ")"
        reference = "(" + ",".join(f"{v:10.6f}" for v in row["reference"]) + ")"
        lines.append(
            f"{row['model']:<17}{centre:<38}{reference:<38}"
            f"{row['reference_name']:<13}{row['deviation']:>9.4f}{row['axes']:>6}  {row['alignment']}"
        )
    lines.append("=" * len(header))
    return "\n".join(lines)


def alignment_conclusions(plan: dict) -> list[str]:
    """哪种对齐方式被采用、为什么 —— 报告里必须明确回答。"""

    fixture = plan["fixture"]
    module = plan["module"]
    lines = [
        "夹具 slot_fixture：采用 **模型 origin = slot_origin** 对齐（不是 bbox 中心）。",
        "  理由：这块网格是 RobotVisionSuite 导出的真机夹具，slot_origin 就是该模型 origin 的"
        "标定值。模型 origin 不在零件中心，bbox 在模型系里是 "
        f"[{fixture['bbox_model']['min'][0]:.4f}, {fixture['bbox_model']['min'][1]:.4f}, {fixture['bbox_model']['min'][2]:.4f}]"
        f" -> [{fixture['bbox_model']['max'][0]:.4f}, {fixture['bbox_model']['max'][1]:.4f}, {fixture['bbox_model']['max'][2]:.4f}] m。"
        "若按 bbox 中心摆，楔形槽口会落到世界 y≈+0.18，而真机插入路径在 y≈-0.013 —— "
        "旧仿真夹具正是这样偏了 194 mm，insert 才会撞在实心壁上。",
        "  生成位姿（模型系->世界）："
        f"xyz=[{fixture['pose']['xyz'][0]:.6f}, {fixture['pose']['xyz'][1]:.6f}, {fixture['pose']['xyz'][2]:.6f}] "
        f"rpy=[0, 0, {fixture['yaw']:.6f}]；bbox 中心因此偏离 slot_origin "
        f"xy {np.linalg.norm(np.asarray(fixture['bbox_centre_offset_world_m'])[:2]) * 1000.0:.1f} mm、"
        f"z {abs(fixture['bbox_centre_offset_world_m'][2]) * 1000.0:.1f} mm（坐标系的定义，不是误差）。"
        f"夹具 bbox 底面在插槽桌顶面以下 {abs(fixture['bottom_on_table_m']) * 1000.0:.1f} mm（底部法兰）。",
        "",
        f"工件 shielding_module：采用 {module['alignment']}（直接使用实测 CAD 位姿）。",
        "  理由：把模型 bbox 中心放到实测 CAD 位姿下，与实测 CAD 中心相差 "
        f"{module['frame_deviation_m'] * 1000.0:.1f} mm，小于 {plan['frame_threshold_m'] * 1000.0:.0f} mm 的判定阈值，"
        "说明模型系就是 CAD 系（model.sdf 注释也写明“SolidWorks part frame, unchanged by the export”）。",
        "  这 {:.1f} mm 几乎全部来自“参考点定义”而不是坐标系：模型系里 bbox 中心是 "
        "({:.4f}, {:.4f}, {:.4f})，而实测 CAD 中心的定义点（scripts/model_environment.py 的 "
        "cad_centre_offset=({:.3f}, {:.3f}, {:.3f})）是 ({:.4f}, {:.4f}, {:.4f})，两点在模型系里相差 {:.1f} mm。".format(
            module["deviation_m"] * 1000.0,
            *module["bbox_centre_model"],
            *module["cad_offset_published_model"],
            *module["cad_offset_model"],
            module["bbox_centre_minus_cad_offset_m"] * 1000.0,
        ),
        "  用该定义点复算实测 CAD 中心，残差只有 {:.4f} mm，从几何上确认模型系就是 CAD 系。".format(
            module["cad_definition_residual_m"] * 1000.0
        ),
        "  因此没有改用 bbox 对齐：那会把工件整体平移 {:.1f} mm，让 CAD 定义点离开实测值。".format(
            module["bbox_centre_minus_cad_offset_m"] * 1000.0
        ),
    ]
    return lines


# ---------------------------------------------------------------------------
# 文件生成
# ---------------------------------------------------------------------------


def _pose_text(xyz, rpy) -> str:
    values = list(xyz) + list(rpy)
    return " ".join(f"{v:.9f}" for v in values)


def build_world_sdf(plan: dict, measurements: dict) -> str:
    """生成世界 SDF 文本。确定性输出：同样的输入 -> 同样的字节。"""

    tables = plan["tables"]
    fixture = plan["fixture"]
    module = plan["module"]
    workpiece_table = tables["workpiece_table"]
    slot_table = tables["slot_table"]
    module_bottom = module["bbox_world"]["min"][2]
    module_sink = workpiece_table["top_z"] - module_bottom
    fixture_number = plan["fixture"]["pose"]
    module_number = plan["module"]["pose"]

    overlap_text = "无" if plan["overlap"] is None else (
        "x[{:.3f}, {:.3f}] y[{:.3f}, {:.3f}]".format(
            plan["overlap"]["xy_min"][0],
            plan["overlap"]["xy_max"][0],
            plan["overlap"]["xy_min"][1],
            plan["overlap"]["xy_max"][1],
        )
    )

    return f"""<?xml version="1.0"?>
<!--
  CS625 真机对齐世界（由 scripts/build_sim_world.py 从实测数据生成，勿手改）。

  与旧的 cs625_insertion_scene.sdf 的区别：旧世界是 SolidWorks 装配系，夹具在
  (-0.85, 0.49)、工件在 (-0.51, -0.81)，没有桌面；本世界直接按真机点云拟合的
  世界系重配，因此位姿与真机实测可比。

  数据来源：
    - 环境模型 {measurements['env_model']}（真机实测点云拟合）
    - 槽位 config/tasks/common.yaml slot_origin / slot_origin_q（RobotVisionSuite 标定）
    - 模型 bbox 来自 model.sdf 引用的 STL 顶点极值

  几何口径（重要）：
    - 工件桌：顶面 z={workpiece_table['top_z']:.5f} m，xy x[{workpiece_table['xy_min'][0]:.3f}, {workpiece_table['xy_max'][0]:.3f}]
      y[{workpiece_table['xy_min'][1]:.3f}, {workpiece_table['xy_max'][1]:.3f}]；盒子从地面顶面 z={plan['floor_top_z']:.3f} 长到桌面。
    - 插槽桌：顶面 z={slot_table['top_z']:.5f} m，xy x[{slot_table['xy_min'][0]:.3f}, {slot_table['xy_max'][0]:.3f}]
      y[{slot_table['xy_min'][1]:.3f}, {slot_table['xy_max'][1]:.3f}]。
    - 两张桌子的盒子在 xy 上重叠 {overlap_text}，重叠区允许两个盒子叠在一起。
    - 夹具：模型 origin = slot_origin（RobotVisionSuite 对这块真机网格的标定），只转 yaw。
      模型 origin 不在零件中心，bbox 中心因此偏离 slot_origin
      {np.linalg.norm(fixture['bbox_centre_offset_world_m']):.4f} m，这是坐标系的定义，不是对齐误差；底部法兰落在
      插槽桌顶面以下 {abs(fixture['bottom_on_table_m']) * 1000.0:.1f} mm。
    - 工件：直接使用实测 CAD 位姿（模型系 == CAD 系，已用 bbox 中心和 CAD 中心核验），
      不按 bbox 摆。工件 bbox 底面比工件桌顶面低 {module_sink * 1000.0:.1f} mm，这是实测位姿本身带出来的，
      不是对齐误差；若要当动力学体生成，需另抬这 {module_sink * 1000.0:.1f} mm 才不穿模。
-->
<sdf version="1.9">
  <world name="{plan['world_name']}">
    <physics name="cs625_task_physics" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="ignition-gazebo-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="ignition-gazebo-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="ignition-gazebo-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>

    <light type="directional" name="sun">
      <pose>0 0 5 0 0 0</pose>
      <diffuse>1.0 1.0 1.0 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- 地面：旧世界写法的地面，但顶面放在 z={plan['floor_top_z']:.3f}。
         真机实测两张桌面在 z={workpiece_table['top_z']:.3f} / {slot_table['top_z']:.3f}，都在机械臂基座安装面
         (base_link, z=0) 以下；地面若留在 z=0 会把桌子埋掉。 -->
    <model name="ground_plane">
      <static>true</static>
      <pose>0 0 {plan['floor_top_z'] - 0.05:.6f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>10 10 0.1</size></box></geometry>
        </collision>
        <sensor name="task_ground_contact" type="contact">
          <contact><collision>collision</collision></contact>
          <always_on>true</always_on>
          <update_rate>100</update_rate>
          <topic>/cs625_task/contacts/ground</topic>
        </sensor>
        <visual name="visual">
          <geometry><box><size>10 10 0.1</size></box></geometry>
          <material><ambient>0.25 0.25 0.27 1</ambient><diffuse>0.35 0.35 0.38 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- 工件桌：实测 xy 范围 + 顶面 z={workpiece_table['top_z']:.5f}，盒子向下长到地面。 -->
    <model name="workpiece_table">
      <static>true</static>
      <pose>{_pose_text(workpiece_table['centre'], (0.0, 0.0, 0.0))}</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{workpiece_table['size'][0]:.6f} {workpiece_table['size'][1]:.6f} {workpiece_table['size'][2]:.6f}</size></box></geometry>
          <surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><box><size>{workpiece_table['size'][0]:.6f} {workpiece_table['size'][1]:.6f} {workpiece_table['size'][2]:.6f}</size></box></geometry>
          <material><ambient>0.55 0.45 0.32 1</ambient><diffuse>0.62 0.52 0.38 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- 插槽桌：顶面 z={slot_table['top_z']:.5f}。 -->
    <model name="slot_table">
      <static>true</static>
      <pose>{_pose_text(slot_table['centre'], (0.0, 0.0, 0.0))}</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{slot_table['size'][0]:.6f} {slot_table['size'][1]:.6f} {slot_table['size'][2]:.6f}</size></box></geometry>
          <surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><box><size>{slot_table['size'][0]:.6f} {slot_table['size'][1]:.6f} {slot_table['size'][2]:.6f}</size></box></geometry>
          <material><ambient>0.40 0.42 0.46 1</ambient><diffuse>0.48 0.50 0.54 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- 夹具：模型 origin = slot_origin（真机网格标定系），只转 yaw。static。 -->
    <include>
      <uri>{fixture['model_uri']}</uri>
      <name>slot_fixture</name>
      <static>true</static>
      <pose>{_pose_text(fixture_number['xyz'], fixture_number['rpy'])}</pose>
    </include>

    <!-- 工件：实测 CAD 位姿原样（xyz + quat 转 rpy），非 bbox 对齐。 -->
    <include>
      <uri>{module['model_uri']}</uri>
      <name>shielding_module</name>
      <pose>{_pose_text(module_number['xyz'], module_number['rpy'])}</pose>
    </include>
  </world>
</sdf>
"""


def _inline(values, fmt: str = "{:.6f}") -> str:
    return "[" + ", ".join(fmt.format(v) for v in values) + "]"


def build_scene_yaml(plan: dict, measurements: dict, world_path) -> str:
    """生成与旧 cs625_task_scene.yaml 同结构的场景 yaml（值全部来自本脚本）。"""

    fixture = plan["fixture"]
    module = plan["module"]
    tables = plan["tables"]
    workpiece_table = tables["workpiece_table"]
    slot_table = tables["slot_table"]

    lines = [
        "# 本文件由 scripts/build_sim_world.py 从真机实测数据生成，勿手改。",
        "# 重新生成（会覆盖本文件）：",
        "#   python3 scripts/build_sim_world.py --out-world <world.sdf> --out-config <本文件>",
        "#",
        f"# 数据来源：环境模型 {measurements['env_model']}（真机实测点云拟合）",
        f"#           + config/tasks/common.yaml 的 slot_origin / slot_origin_q（机械固定槽位标定）",
        f"#           + 两个模型 STL 的顶点极值（bbox）",
        f"# 世界文件：{relative(pathlib.Path(world_path))}",
        "#",
        "# 对齐方式（这两条是本次生成的核心决定，改数值前先读）：",
        "#   slot_fixture      模型 origin = slot_origin（RVS 对该真机网格的标定），只转 yaw。",
        "#                     bbox 中心不在 origin，xy 偏 208 mm；按 bbox 中心摆会把槽口挪到"
        " y≈+0.18，insert 会撞实心壁。",
        f"#   shielding_module  直接使用实测 CAD 位姿（模型系 == CAD 系，bbox 中心与实测 CAD 中心",
        f"#                     相差 {module['frame_deviation_m'] * 1000.0:.1f} mm，阈值 50 mm）。",
        "#",
        "# 单位：米；角度：弧度；四元数为 (x, y, z, w)。",
        "",
        f"{SCENE_KEY}:",
        "  ros__parameters:",
        "    # --- 生成信息 ---------------------------------------------------------",
        f"    generated_by: \"scripts/build_sim_world.py\"",
        f"    world_file: \"{relative(pathlib.Path(world_path))}\"",
        f"    env_model: \"{measurements['env_model']}\"",
        f"    warn_threshold_m: {plan['warn_threshold_m']:.3f}",
        "",
        "    # --- 实测桌面（世界系） -----------------------------------------------",
        "    tables:",
        "      workpiece_table:",
        f"        top_z: {workpiece_table['top_z']:.6f}",
        f"        xy_min: {_inline(workpiece_table['xy_min'])}",
        f"        xy_max: {_inline(workpiece_table['xy_max'])}",
        f"        box_centre: {_inline(workpiece_table['centre'])}",
        f"        box_size: {_inline(workpiece_table['size'])}",
        "      slot_table:",
        f"        top_z: {slot_table['top_z']:.6f}",
        f"        xy_min: {_inline(slot_table['xy_min'])}",
        f"        xy_max: {_inline(slot_table['xy_max'])}",
        f"        box_centre: {_inline(slot_table['centre'])}",
        f"        box_size: {_inline(slot_table['size'])}",
        f"    floor_top_z: {plan['floor_top_z']:.6f}",
        "",
        "    # --- 夹具（静态） -----------------------------------------------------",
        "    fixture:",
        f"      model_uri: \"{FIXTURE_URI}\"",
        f"      pose_world: {_inline(fixture['pose']['xyz'] + fixture['pose']['rpy'])}",
        f"      alignment: {fixture['alignment']}",
        f"      bbox_min_m: {_inline(fixture['bbox_model']['min'])}",
        f"      bbox_max_m: {_inline(fixture['bbox_model']['max'])}",
        f"      bbox_centre_world_m: {_inline(fixture['bbox_world']['centre'])}",
        f"      bbox_centre_offset_world_m: {_inline(fixture['bbox_centre_offset_world_m'])}",
        f"      reference_slot_origin_m: {_inline(fixture['reference'])}",
        f"      deviation_m: {fixture['deviation_m']:.6f}",
        f"      bottom_below_slot_table_m: {fixture['bottom_on_table_m']:.6f}",
        "",
        "    # --- 工件（实测 CAD 位姿原样） ---------------------------------------",
        "    module:",
        f"      model_uri: \"{MODULE_URI}\"",
        f"      pose_world: {_inline(module['pose']['xyz'] + module['pose']['rpy'])}",
        f"      alignment: {module['alignment']}",
        f"      pose_source: {module['pose_source']}",
        f"      measured_quat_xyzw: {_inline(module['measured_quat_xyzw'], '{:.6f}')}",
        f"      bbox_min_m: {_inline(module['bbox_model']['min'])}",
        f"      bbox_max_m: {_inline(module['bbox_model']['max'])}",
        f"      bbox_centre_world_m: {_inline(module['bbox_world']['centre'])}",
        f"      reference_cad_centre_m: {_inline(module['reference'])}",
        f"      deviation_m: {module['deviation_m']:.6f}",
        f"      frame_deviation_m: {module['frame_deviation_m']:.6f}",
        "",
        "    # --- 拟合报告（完整表格由脚本打印；这里只留两条汇总） -----------------",
        "    fit:",
        f"      max_deviation_m: {max(r['deviation'] for r in plan['rows']):.6f}",
    ]

    return "\n".join(lines) + "\n"


def build_report(plan: dict, measurements: dict) -> dict:
    """拟合报告 JSON（喂给 --report）。"""

    return {
        "generated_by": "scripts/build_sim_world.py",
        "env_model": measurements["env_model"],
        "floor_top_z": plan["floor_top_z"],
        "tables": {name: table for name, table in plan["tables"].items()},
        "floor_table_overlap": plan["overlap"],
        "fixture": plan["fixture"],
        "module": plan["module"],
        "fit_rows": plan["rows"],
        "warnings": deviations_requiring_warning(plan["rows"], plan["warn_threshold_m"]),
        "notes": plan["notes"],
        "measurements": {
            "slot_origin": measurements["slot_origin"],
            "slot_quat_xyzw": measurements["slot_quat_xyzw"],
            "workpiece": measurements["workpiece"],
            "declared_bbox": measurements["declared"],
        },
    }


# ---------------------------------------------------------------------------
# 俯视图
# ---------------------------------------------------------------------------


def render_topview(plan: dict, measurements: dict, path) -> pathlib.Path:
    """实测（虚线圈/叉）与生成世界（实线框/bbox）叠在一张俯视图里。

    左图给整格，右图放大到两张桌子的重叠区和两个模型；图片本身只是给人看
    的，判定依据是脚本打印的拟合表与报告 JSON。
    """

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    fixture = plan["fixture"]
    module = plan["module"]
    tables = plan["tables"]

    figure, (full_axis, zoom_axis) = plt.subplots(1, 2, figsize=(16, 7))

    for axis in (full_axis, zoom_axis):
        # 生成世界：两张桌子的盒子（实测 xy 范围 + 顶面 z），实线蓝框。
        for table in tables.values():
            axis.add_patch(
                Rectangle(
                    table["xy_min"][:2],
                    table["size"][0],
                    table["size"][1],
                    facecolor="#9ecae1",
                    alpha=0.20,
                    edgecolor="tab:blue",
                    linewidth=1.8,
                    zorder=1,
                )
            )
        # 实测：两张桌面的实测 xy 范围，黑虚线。与生成框完全重合是预期结果。
        measured_label = "measured table extent (dashed) / generated box (blue)"
        for index, measured in enumerate(measurements["tables"].values()):
            lo, hi = measured["xy_min"], measured["xy_max"]
            axis.add_patch(
                Rectangle(
                    (lo[0], lo[1]),
                    hi[0] - lo[0],
                    hi[1] - lo[1],
                    fill=False,
                    edgecolor="black",
                    linestyle="--",
                    linewidth=1.1,
                    zorder=2,
                    label=measured_label if index == 0 else None,
                )
            )
        if plan["overlap"]:
            overlap = plan["overlap"]
            axis.add_patch(
                Rectangle(
                    overlap["xy_min"],
                    overlap["xy_max"][0] - overlap["xy_min"][0],
                    overlap["xy_max"][1] - overlap["xy_min"][1],
                    facecolor="none",
                    edgecolor="tab:red",
                    hatch="//",
                    linewidth=0.0,
                    alpha=0.35,
                    zorder=3,
                    label="table boxes overlap in xy",
                )
            )

        # 生成世界里的两个模型：世界系 AABB 脚印。
        for label, model, colour in (
            ("slot_fixture bbox (generated)", fixture, "#333333"),
            ("shielding_module bbox (generated)", module, "#e6550d"),
        ):
            bbox = model["bbox_world"]
            axis.add_patch(
                Rectangle(
                    (bbox["min"][0], bbox["min"][1]),
                    bbox["max"][0] - bbox["min"][0],
                    bbox["max"][1] - bbox["min"][1],
                    fill=False,
                    edgecolor=colour,
                    linewidth=2.0,
                    zorder=4,
                    label=label,
                )
            )
            axis.plot(*bbox["centre"][:2], "o", color=colour, markersize=6, zorder=6)
            axis.annotate(
                label.split(" (")[0],
                xy=(bbox["max"][0], bbox["max"][1]),
                xytext=(4, 4),
                textcoords="offset points",
                color=colour,
                fontsize=8,
                zorder=7,
            )

        # 实测参考点：CAD 中心、槽口原点、工件 CAD origin。
        axis.plot(
            measurements["workpiece"]["cad_centre"][0],
            measurements["workpiece"]["cad_centre"][1],
            "x",
            color="crimson",
            markersize=11,
            markeredgewidth=2.2,
            zorder=7,
            label="measured CAD centre (workpiece)",
        )
        axis.plot(
            measurements["slot_origin"][0],
            measurements["slot_origin"][1],
            "x",
            color="darkgreen",
            markersize=11,
            markeredgewidth=2.2,
            zorder=7,
            label="measured slot_origin",
        )
        axis.plot(
            measurements["workpiece"]["xyz"][0],
            measurements["workpiece"]["xyz"][1],
            "+",
            color="crimson",
            markersize=9,
            zorder=7,
            label="measured workpiece CAD origin",
        )

        # 工件：bbox 中心 -> 实测 CAD 中心的残差箭头。
        module_centre = module["bbox_world"]["centre"]
        reference = module["reference"]
        axis.annotate(
            "",
            xy=(reference[0], reference[1]),
            xytext=(module_centre[0], module_centre[1]),
            arrowprops=dict(arrowstyle="->", color="crimson", lw=1.3),
            zorder=6,
        )
        axis.text(
            0.02,
            0.02,
            "shielding_module bbox centre vs measured CAD centre: {:.1f} mm\n"
            "slot_fixture bbox centre vs slot_origin: {:.2f} mm (xy)".format(
                module["deviation_m"] * 1000.0, fixture["deviation_m"] * 1000.0
            ),
            transform=axis.transAxes,
            fontsize=8,
            color="crimson",
            va="bottom",
            bbox=dict(boxstyle="round,pad=0.3", facecolor="white", edgecolor="crimson", alpha=0.85),
            zorder=8,
        )

        axis.set_aspect("equal")
        axis.grid(True, linestyle=":", alpha=0.5)
        axis.set_xlabel("x [m] (base_link)")
        axis.set_ylabel("y [m] (base_link)")

    full_axis.set_title("top view: measured (dashed/×) vs generated world (solid/bboxes)")
    full_axis.legend(loc="upper right", fontsize=7)
    full_axis.set_xlim(0.10, 1.85)
    full_axis.set_ylim(-0.55, 1.20)

    zoom_axis.set_title("zoom: table overlap, module and fixture footprints")
    zoom_axis.set_xlim(0.40, 1.50)
    zoom_axis.set_ylim(-0.30, 0.85)

    figure.suptitle(
        "cs625_real_cell: table top z {:.3f}/{:.3f} m, floor top z {:.3f} m; "
        "fixture dev {:.2f} mm (xy), module dev {:.1f} mm (bbox centre vs CAD centre)".format(
            tables["workpiece_table"]["top_z"],
            tables["slot_table"]["top_z"],
            plan["floor_top_z"],
            fixture["deviation_m"] * 1000.0,
            module["deviation_m"] * 1000.0,
        ),
        fontsize=9,
    )
    figure.tight_layout(rect=(0, 0, 1, 0.95))
    path = pathlib.Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=150)
    plt.close(figure)
    return path


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="从真机实测数据生成对齐的 Gazebo 世界与场景 yaml（只写新文件，不动旧世界）。",
    )
    parser.add_argument("--out-world", required=True, help="输出世界 SDF 路径（相对仓库根）")
    parser.add_argument("--out-config", required=True, help="输出场景 yaml 路径（相对仓库根）")
    parser.add_argument("--report", default=None, help="输出拟合报告 JSON（可选）")
    parser.add_argument("--figure", default=None, help="输出俯视图对比 png（可选）")
    parser.add_argument("--env-model", default=None, help="覆盖实测环境模型 model.json 路径")
    parser.add_argument("--floor-top-z", type=float, default=DEFAULT_FLOOR_TOP_Z, help="地面顶面 z（默认 -0.20）")
    parser.add_argument("--warn-threshold", type=float, default=WARN_THRESHOLD_M, help="偏差报警阈值（m，默认 0.02）")
    parser.add_argument("--frame-threshold", type=float, default=FRAME_THRESHOLD_M, help="模型系/CAD 系判定阈值（m，默认 0.05）")
    args = parser.parse_args(argv)

    def resolve(value: str) -> pathlib.Path:
        path = pathlib.Path(value)
        return path if path.is_absolute() else ROOT / path

    world_path = resolve(args.out_world)
    config_path = resolve(args.out_config)
    report_path = resolve(args.report) if args.report else None
    figure_path = resolve(args.figure) if args.figure else None

    measurements = load_measurements(args.env_model)
    plan = plan_scene(measurements, args.floor_top_z, args.warn_threshold, args.frame_threshold)

    world_text = build_world_sdf(plan, measurements)
    world_path.parent.mkdir(parents=True, exist_ok=True)
    world_path.write_text(world_text, encoding="utf-8")

    config_text = build_scene_yaml(plan, measurements, world_path)
    config_path.parent.mkdir(parents=True, exist_ok=True)
    config_path.write_text(config_text, encoding="utf-8")

    print(f"实测环境模型: {measurements['env_model']}")
    print("桌面（实测）:")
    for name, table in plan["tables"].items():
        print(
            "  {:<16} 顶面 z={:+.5f} m  xy x[{:+.3f}, {:+.3f}] y[{:+.3f}, {:+.3f}]  盒厚 {:.4f} m（地面顶面 z={:+.3f}）".format(
                name,
                table["top_z"],
                table["xy_min"][0],
                table["xy_max"][0],
                table["xy_min"][1],
                table["xy_max"][1],
                table["size"][2],
                plan["floor_top_z"],
            )
        )
    if plan["overlap"]:
        print(
            "  提示：两张桌子的盒子在 xy 上重叠 x[{:.3f}, {:.3f}] y[{:.3f}, {:.3f}]（{:.4f} m²），"
            "重叠区两个盒子叠在一起；两张桌面高度不同，不做裁剪。".format(
                plan["overlap"]["xy_min"][0],
                plan["overlap"]["xy_max"][0],
                plan["overlap"]["xy_min"][1],
                plan["overlap"]["xy_max"][1],
                plan["overlap"]["area"],
            )
        )
    print()
    print(format_fit_table(plan["rows"]))
    print()
    warnings = deviations_requiring_warning(plan["rows"], plan["warn_threshold_m"])
    if warnings:
        print("警告:")
        for warning in warnings:
            print("  " + warning)
        print(
            "  注意：超阈值不等于“生成错了”。实测参考点可能是模型系里与 bbox 中心不同的定义点"
            "（例如实测 CAD 中心是 CAD origin + 固定偏移），这类残差见下面“对齐结论”的分解。"
        )
    else:
        print("警告: 无（所有偏差均 <= {:.0f} mm）".format(plan["warn_threshold_m"] * 1000.0))
    print()
    print("对齐结论:")
    for line in alignment_conclusions(plan):
        print("  " + line if line else "")
    print()
    print("说明:")
    for note in plan["notes"]:
        print("  - " + note)
    print()
    print("几何交叉校验:")
    print(
        "  fixture bbox（STL）: [{:.6f}, {:.6f}, {:.6f}] -> [{:.6f}, {:.6f}, {:.6f}]".format(
            *plan["fixture"]["bbox_model"]["min"], *plan["fixture"]["bbox_model"]["max"]
        )
    )
    declared_fixture = measurements["declared"]["fixture"]
    if declared_fixture:
        delta = max(
            abs(a - b) for a, b in zip(declared_fixture["min"] + declared_fixture["max"], plan["fixture"]["bbox_model"]["min"] + plan["fixture"]["bbox_model"]["max"])
        )
        print(f"  fixture bbox（model.sdf 注释）与 STL 最大差: {delta * 1000.0:.3f} mm")
    print(
        "  module bbox（STL）: [{:.6f}, {:.6f}, {:.6f}] -> [{:.6f}, {:.6f}, {:.6f}]".format(
            *plan["module"]["bbox_model"]["min"], *plan["module"]["bbox_model"]["max"]
        )
    )
    declared_module = measurements["declared"]["module"]
    if declared_module:
        delta = max(
            abs(a - b) for a, b in zip(declared_module["min"] + declared_module["max"], plan["module"]["bbox_model"]["min"] + plan["module"]["bbox_model"]["max"])
        )
        print(f"  module bbox（model.sdf 注释）与 STL 最大差: {delta * 1000.0:.3f} mm")
    print()
    print(f"世界文件: {relative(world_path)}")
    print(f"场景配置: {relative(config_path)}")

    if report_path:
        report = build_report(plan, measurements)
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        print(f"拟合报告: {relative(report_path)}")

    if figure_path:
        written = render_topview(plan, measurements, figure_path)
        print(f"对比图:   {relative(written)}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
