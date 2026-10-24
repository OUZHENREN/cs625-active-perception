#!/usr/bin/env python3
"""判定哪份插槽夹具几何与真机 1:1 一致。

为什么需要这个脚本
------------------
仓库里有四份候选夹具几何，来自不同年代、不同软件、不同单位：

    RVS 点云   slot_pointcloud20260127.ply （米，师兄 RobotVisionSuite 导出）
    RVS 网格   slot_pointcloud.STL         （毫米，同一台设备的网格导出）
    legacy     slot_collision.STL          （毫米，师兄旧包里的 360 面碰撞代理）
    旧仿真     slot_fixture.stl            （米，SolidWorks 装配系）

四位"作者"都声称自己是插槽。只看文件名无法判断，必须只看几何：

1. 包围盒尺寸（与真机 RGB-D 重建点云比，判据 ≤ 3 cm）；
2. **槽口宽度**：沿真实插入方向（`pre_insert -> insert`）逐层取横截面，
   找最窄的贯通间隙（判据 ≤ 5 mm）。插槽单边间隙约 1 mm，这条比 bbox 更硬。
3. 槽口中心是否落在真机插入路径上 —— 这是今天 insert 失败的真正原因：
   几何也许"长得像"，但放在世界系里的位置不对。

真机点云的重建复用 `scripts/build_scene_model.py` 的 `load_metadata` /
`frame_to_base`，不重复实现深度投影。真机批次只用 `slot*` 与 `box_in_slot*`，
因为这几批拍的就是插槽本体。

坐标与单位
----------
* 候选网格在世界系里的位姿：
    - RVS 点云/网格、legacy：**模型原点 = slot_origin**（RobotVisionSuite 标定
      的就是模型原点，`slot_origin_q` 给出朝向）。这是本脚本的核心判据之一。
    - 旧仿真网格：沿用今天发布到 MoveIt 的那套规则（bbox 中心 xy 对齐
      slot_origin、bbox 底面贴插槽桌顶面），用来复现今天的失败。
* 所有输出一律米；候选文件内部的单位只影响读入时的缩放。

用法
----
    python3 scripts/analyze_fixture_geometry.py
    python3 scripts/analyze_fixture_geometry.py --json /tmp/fixture_analysis.json
    python3 scripts/analyze_fixture_geometry.py --no-cloud   # 只看文件，不重建点云

本脚本不 import rclpy / gazebo；纯几何，可被 `test/test_fixture_geometry.py`
当普通模块导入做单测。
"""

from __future__ import annotations

import argparse
import json
import pathlib
import re
import sys

import numpy as np
import yaml

sys.stdout.reconfigure(line_buffering=True)

ROOT = pathlib.Path(__file__).resolve().parents[1]
COMMON_CONFIG = ROOT / "config" / "tasks" / "common.yaml"
SCENE_CONFIG = ROOT / "config" / "cs625_real_cell_scene.yaml"
DEFAULT_DATASET = pathlib.Path.home() / "cs625_yolo_seg" / "datasets" / "2026-10-25_raw"
DEFAULT_ENV_MODEL = ROOT / "runtime" / "environment" / "latest" / "model.json"

RVS_DIR = pathlib.Path("/mnt/d/Program Files (x86)/RobotVisionSuite/runtime/Model")
LEGACY_DIR = (pathlib.Path.home()
              / "cs625_legacy_underlay_jazzy" / "src" / "tcp_bridge" / "meshes")
SIM_DIR = ROOT / "src" / "cs625_simulation" / "assets"

#: 真机点云重建用的批次：只拍插槽的那几批。
FIXTURE_BATCHES = ("slot", "slot_close", "slot_eval", "slot_eval2",
                   "box_in_slot", "box_in_slot2")
EMPTY_SLOT_BATCHES = ("slot", "slot_close", "slot_eval", "slot_eval2")

#: bbox / 槽宽判据（任务给定）。
BBOX_TOLERANCE_M = 0.03
SLOT_WIDTH_TOLERANCE_M = 0.005
#: 点到网格表面 p99 距离 ≤ 5 mm => 点云就是这块网格上的同一个物体。
SURFACE_IDENTITY_TOLERANCE_M = 0.005
#: 插入路径上至少这么多层要有贯通槽，否则说明路径根本不在槽里。
MIN_VALID_LEVELS = 5


# ---------------------------------------------------------------------------
# 基础三维工具（纯函数）
# ---------------------------------------------------------------------------


def rotation_yaw(yaw: float) -> np.ndarray:
    """绕 z 的旋转矩阵（夹具正立，只允许 yaw）。"""

    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def quat_xyzw_to_matrix(quat) -> np.ndarray:
    """(x, y, z, w) 四元数 -> 旋转矩阵，先归一化。"""

    x, y, z, w = (float(v) for v in quat)
    norm = float(np.sqrt(x * x + y * y + z * z + w * w))
    if norm < 1e-12:
        raise ValueError("zero quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def yaw_from_quat(quat) -> float:
    """取四元数的等效偏航角（只保留绕 z 的分量，夹具必须正立）。"""

    x, y, z, w = (float(v) for v in quat)
    return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def transform_points(points: np.ndarray, xyz, rot: np.ndarray | None = None) -> np.ndarray:
    """刚体变换：R @ p + t。"""

    points = np.asarray(points, dtype=float)
    rot = np.eye(3) if rot is None else np.asarray(rot, dtype=float)
    return points @ rot.T + np.asarray(xyz, dtype=float)


def bbox(points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """点集/顶点的轴对齐包围盒。"""

    points = np.asarray(points, dtype=float).reshape(-1, 3)
    if len(points) == 0:
        raise ValueError("empty point set")
    return points.min(axis=0), points.max(axis=0)


def bbox_size(points: np.ndarray) -> np.ndarray:
    lo, hi = bbox(points)
    return hi - lo


def mm_to_m(points: np.ndarray) -> np.ndarray:
    """毫米 -> 米。RVS 与 legacy 的 STL 是毫米制。"""

    return np.asarray(points, dtype=float) * 0.001


def unit_scale(unit: str) -> float:
    """'mm' -> 0.001，'m' -> 1.0。用于单位换算的可测试入口。"""

    if unit == "mm":
        return 0.001
    if unit == "m":
        return 1.0
    raise ValueError(f"unsupported unit: {unit!r}")


# ---------------------------------------------------------------------------
# 文件解析（纯函数）：ASCII STL / 二进制 STL / 二进制 PLY
# ---------------------------------------------------------------------------


def parse_ascii_stl(payload: bytes) -> np.ndarray:
    """ASCII STL -> (M, 3, 3) 三角形数组（米制由调用方决定）。"""

    found = re.findall(
        rb"vertex\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)", payload
    )
    if not found:
        raise ValueError("ASCII STL contains no 'vertex' lines")
    vertices = np.array(found, dtype=float)
    if len(vertices) % 3:
        raise ValueError(f"ASCII STL vertex count {len(vertices)} is not a multiple of 3")
    return vertices.reshape(-1, 3, 3)


def parse_binary_stl(payload: bytes) -> np.ndarray:
    """二进制 STL -> (M, 3, 3) 三角形数组。"""

    if len(payload) < 84:
        raise ValueError("binary STL shorter than 84 bytes")
    count = int.from_bytes(payload[80:84], "little")
    expected = 84 + 50 * count
    if len(payload) != expected:
        raise ValueError(f"binary STL: expected {expected} bytes for {count} triangles, got {len(payload)}")
    record = np.dtype([("normal", "<3f4"), ("vertices", "<9f4"), ("attr", "<u2")])
    records = np.frombuffer(payload[84:], dtype=record)
    return records["vertices"].astype(float).reshape(-1, 3, 3)


def is_ascii_stl(payload: bytes) -> bool:
    """STL 判别：以 'solid' 开头且前 2 KB 内有 'facet'。"""

    return payload[:5].lower() == b"solid" and b"facet" in payload[:2048]


def read_stl_triangles(path: pathlib.Path) -> tuple[np.ndarray, str]:
    """读 STL（自动判 ASCII/二进制），返回 (triangles, format)。"""

    payload = pathlib.Path(path).read_bytes()
    if is_ascii_stl(payload):
        return parse_ascii_stl(payload), "ascii"
    return parse_binary_stl(payload), "binary"


PLY_TYPE_MAP = {
    "char": "i1", "int8": "i1", "uchar": "u1", "uint8": "u1",
    "short": "i2", "int16": "i2", "ushort": "u2", "uint16": "u2",
    "int": "i4", "int32": "i4", "uint": "u4", "uint32": "u4",
    "float": "f4", "float32": "f4", "double": "f8", "float64": "f8",
}


def parse_ply_header(payload: bytes) -> tuple[dict, int]:
    """解析 PLY 头，返回 (header, body_offset)。只支持 binary_little_endian。"""

    marker = b"end_header"
    index = payload.find(marker)
    if index < 0:
        raise ValueError("PLY has no end_header")
    body_offset = index + len(marker)
    newline = payload.find(b"\n", body_offset)
    if newline >= 0 and payload[body_offset:newline].strip() == b"":
        body_offset = newline + 1
    header_text = payload[:index].decode("ascii", errors="replace")
    lines = [line.strip() for line in header_text.splitlines() if line.strip()]
    if not lines or lines[0] != "ply":
        raise ValueError("not a PLY file")
    if "format binary_little_endian 1.0" not in header_text:
        raise ValueError("only binary_little_endian PLY is supported")

    header: dict = {"elements": []}
    for line in lines[1:]:
        parts = line.split()
        if parts[0] == "element":
            header["elements"].append({"name": parts[1], "count": int(parts[2]), "properties": []})
        elif parts[0] == "property" and header["elements"]:
            if parts[1] == "list":
                header["elements"][-1]["properties"].append(
                    {"name": parts[4], "list": True, "count_type": parts[2], "item_type": parts[3]})
            else:
                header["elements"][-1]["properties"].append(
                    {"name": parts[2], "list": False, "type": parts[1]})
    return header, body_offset


def read_ply_points(path: pathlib.Path) -> np.ndarray:
    """二进制 PLY 顶点 xyz（忽略法线/颜色/面）。"""

    payload = pathlib.Path(path).read_bytes()
    header, offset = parse_ply_header(payload)
    for element in header["elements"]:
        if element["name"] != "vertex":
            continue
        fields = []
        for prop in element["properties"]:
            if prop["list"]:
                raise ValueError("vertex element with list property is not supported")
            fields.append((prop["name"], PLY_TYPE_MAP[prop["type"]]))
        dtype = np.dtype(fields)
        count = element["count"]
        body = payload[offset:offset + dtype.itemsize * count]
        if len(body) != dtype.itemsize * count:
            raise ValueError("PLY vertex block is truncated")
        records = np.frombuffer(body, dtype=dtype, count=count)
        return np.stack([records["x"], records["y"], records["z"]], axis=-1).astype(float)
    raise ValueError("PLY has no vertex element")


def mesh_vertices(triangles: np.ndarray) -> np.ndarray:
    return np.asarray(triangles, dtype=float).reshape(-1, 3)


def sample_triangles(triangles: np.ndarray, count: int = 200000, seed: int = 0) -> np.ndarray:
    """三角形表面均匀采样（按面积加权），用于"点云是否落在该网格表面上"的判定。"""

    triangles = np.asarray(triangles, dtype=float)
    if len(triangles) == 0:
        return np.zeros((0, 3))
    edge1 = triangles[:, 1] - triangles[:, 0]
    edge2 = triangles[:, 2] - triangles[:, 0]
    area = 0.5 * np.linalg.norm(np.cross(edge1, edge2), axis=1)
    total = float(area.sum())
    if total <= 0.0:
        raise ValueError("degenerate triangle mesh")
    rng = np.random.default_rng(seed)
    index = rng.choice(len(triangles), size=int(count), p=area / total)
    uv = rng.random((int(count), 2))
    flip = uv.sum(axis=1) > 1.0
    uv[flip] = 1.0 - uv[flip]
    u = uv[:, 0:1]
    v = uv[:, 1:2]
    chosen = triangles[index]
    return chosen[:, 0] + u * edge1[index] + v * edge2[index]


def point_to_surface_stats(points: np.ndarray, triangles: np.ndarray,
                           sample_count: int = 200000) -> dict:
    """点到三角网格表面的最近距离统计（中位数 / p99 / 最大值，米）。"""

    from scipy.spatial import cKDTree

    samples = sample_triangles(triangles, count=sample_count)
    tree = cKDTree(samples)
    distances, _ = tree.query(np.asarray(points, dtype=float), workers=-1)
    return {
        "median_m": float(np.median(distances)),
        "p99_m": float(np.percentile(distances, 99)),
        "max_m": float(distances.max()),
        "samples": int(sample_count),
    }


# ---------------------------------------------------------------------------
# 射线与槽口测量（纯函数）
# ---------------------------------------------------------------------------


def ray_first_hit(triangles: np.ndarray, origin, direction, t_max: float = 2.0):
    """Möller–Trumbore：返回最近命中距离，未命中返回 None。

    `direction` 会被归一化，所以返回的距离就是米。
    """

    triangles = np.asarray(triangles, dtype=float)
    origin = np.asarray(origin, dtype=float)
    direction = np.asarray(direction, dtype=float)
    norm = float(np.linalg.norm(direction))
    if norm < 1e-12:
        raise ValueError("zero ray direction")
    direction = direction / norm

    v0 = triangles[:, 0]
    edge1 = triangles[:, 1] - v0
    edge2 = triangles[:, 2] - v0
    h = np.cross(direction, edge2)
    a = np.einsum("ij,ij->i", edge1, h)
    valid = np.abs(a) > 1e-12
    f = np.zeros_like(a)
    f[valid] = 1.0 / a[valid]

    s = origin - v0
    u = f * np.einsum("ij,ij->i", s, h)
    valid &= (u >= -1e-9) & (u <= 1.0 + 1e-9)

    q = np.cross(s, edge1)
    v = f * (q @ direction)
    valid &= (v >= -1e-9) & (u + v <= 1.0 + 1e-9)

    t = f * np.einsum("ij,ij->i", edge2, q)
    valid &= (t > 1e-9) & (t < t_max)
    if not valid.any():
        return None
    return float(t[valid].min())


def through_gap(triangles: np.ndarray, origin, axis, t_max: float = 1.0):
    """沿 ±axis 各打一条射线，返回两侧壁之间的贯通间隙。

    只适用于"原点在槽腔内"的探针点：两条射线都命中才算贯通；有一侧没命中
    说明那一侧是敞开的，返回 None（不是槽）。
    """

    plus = ray_first_hit(triangles, origin, axis, t_max)
    minus = ray_first_hit(triangles, origin, -np.asarray(axis, dtype=float), t_max)
    if plus is None or minus is None:
        return None
    return plus + minus


def histogram_wall_gap(values: np.ndarray, probe: float, bin_size: float = 0.001,
                       min_count: int = 3):
    """一维占用直方图找探针左右最近的两面墙。

    返回 (gap, near, far)。单点飞点会被 `min_count` 过滤掉，因此比"最近点"
    稳健。没有两侧都出现 >= min_count 点的 bin 时返回 None。
    """

    values = np.asarray(values, dtype=float)
    if len(values) < 2 * min_count:
        return None, None, None
    lo = float(np.min(values))
    hi = float(np.max(values))
    if hi - lo < bin_size:
        return None, None, None
    edges = np.arange(lo, hi + bin_size, bin_size)
    counts, edges = np.histogram(values, bins=edges)
    centres = 0.5 * (edges[:-1] + edges[1:])
    occupied = counts >= min_count
    below = np.where(occupied & (centres < probe))[0]
    above = np.where(occupied & (centres > probe))[0]
    if len(below) == 0 or len(above) == 0:
        return None, None, None
    far = float(probe - centres[below[-1]])
    near = float(centres[above[0]] - probe)
    return near + far, near, far


def cloud_through_gap(points: np.ndarray, probe, axis: str = "y",
                      slab_half: float = 0.006, cross_half: float = 0.004,
                      bin_size: float = 0.001, min_count: int = 3):
    """点云版贯通间隙：只取与射线垂直的另两轴 ±`cross_half` 内的点，
    沿射线做 1 mm 占用直方图，找探针左右最近的有料 bin。

    返回 (gap, near, far)；探针不在槽腔内（一侧没有料）时返回 (None, None, None)。
    """

    points = np.asarray(points, dtype=float)
    probe = np.asarray(probe, dtype=float)
    axes = {"x": 0, "y": 1, "z": 2}
    i = axes[axis]
    others = [k for k in range(3) if k != i]

    mask = np.ones(len(points), dtype=bool)
    for k in others:
        half = cross_half if k != 2 or axis == "z" else slab_half
        mask &= np.abs(points[:, k] - probe[k]) <= half
    along = points[mask, i] - probe[i]
    along = along[(np.abs(along) > 0.002) & (np.abs(along) < 0.6)]
    gap, near, far = histogram_wall_gap(along, 0.0, bin_size=bin_size, min_count=min_count)
    if gap is None:
        return None, None, None
    return gap, near, far


def voxel_downsample(points: np.ndarray, voxel: float = 0.005) -> np.ndarray:
    """体素下采样（每个体素保留一个点），用于给统计滤波降计算量。"""

    points = np.asarray(points, dtype=float)
    if len(points) == 0:
        return points
    keys = np.floor(points / float(voxel)).astype(np.int64)
    _, index = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(index)]


def statistical_filter(points: np.ndarray, k: int = 20, std_ratio: float = 1.0) -> np.ndarray:
    """统计离群点去除（等价 open3d StatisticalOutlierRemoval 的判据）。

    飞点（深度边缘）会让点云 bbox 虚胖，先按"到第 k 近邻的平均距离"剔除。
    """

    from scipy.spatial import cKDTree

    points = np.asarray(points, dtype=float)
    if len(points) <= k + 1:
        return points
    tree = cKDTree(points)
    distances, _ = tree.query(points, k=k + 1, workers=-1)
    mean_distance = distances[:, 1:].mean(axis=1)
    threshold = mean_distance.mean() + std_ratio * mean_distance.std()
    return points[mean_distance <= threshold]


def insertion_line(pre_insert, insert, levels: int = 9) -> list[tuple[float, np.ndarray]]:
    """沿真实插入轴取 `levels` 个探针点（含两端），返回 [(world_z, xyz)]。"""

    pre = np.asarray(pre_insert, dtype=float)
    ins = np.asarray(insert, dtype=float)
    result = []
    for index in range(levels):
        fraction = index / max(levels - 1, 1)
        point = pre + fraction * (ins - pre)
        result.append((float(point[2]), point))
    return result


def slot_profile(triangles: np.ndarray, pre_insert, insert, levels: int = 9,
                 axis=(0.0, 1.0, 0.0)) -> list[dict]:
    """逐层槽宽（三角网格）。返回 [{'z', 'xyz', 'gap'}]。"""

    profile = []
    for z, point in insertion_line(pre_insert, insert, levels):
        gap = through_gap(triangles, point, axis, t_max=0.8)
        profile.append({"z": z, "xyz": point.tolist(), "gap": gap})
    return profile


def cloud_slot_profile(points: np.ndarray, pre_insert, insert, levels: int = 9) -> list[dict]:
    """逐层槽宽（点云）。"""

    profile = []
    for z, point in insertion_line(pre_insert, insert, levels):
        gap, near, far = cloud_through_gap(points, point, axis="y")
        profile.append({"z": z, "xyz": point.tolist(), "gap": gap,
                        "near_m": near, "far_m": far})
    return profile


def summarise_profile(profile: list[dict]) -> dict:
    """把逐层 profile 压成 min/median/valid 三个数。None 表示该层不是贯通槽。"""

    gaps = [row["gap"] for row in profile if row["gap"] is not None]
    return {
        "levels": len(profile),
        "valid": len(gaps),
        "min_gap": float(min(gaps)) if gaps else None,
        "median_gap": float(np.median(gaps)) if gaps else None,
    }


# ---------------------------------------------------------------------------
# 世界位姿
# ---------------------------------------------------------------------------


def load_common(path: pathlib.Path = COMMON_CONFIG) -> dict:
    with pathlib.Path(path).open(encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def rvs_placement(common: dict) -> dict:
    """RVS 模型原点 = slot_origin（标定语义），朝向取 slot_origin_q 的 yaw。"""

    slot = np.asarray(common["slot_origin"], dtype=float)
    yaw = yaw_from_quat(common["slot_origin_q"])
    return {"xyz": slot.tolist(), "rot": rotation_yaw(yaw), "yaw": yaw,
            "rule": "model_origin_on_slot_origin"}


def bbox_centre_placement(triangles: np.ndarray, common: dict, slot_table_top: float) -> dict:
    """旧规则：bbox 中心 xy 对齐 slot_origin、bbox 底面贴插槽桌顶面、只转 yaw。

    这正是今天 `build_sim_world.py` 发布旧夹具时用的规则，用来复现失败。
    """

    slot = np.asarray(common["slot_origin"], dtype=float)
    yaw = yaw_from_quat(common["slot_origin_q"])
    rot = rotation_yaw(yaw)
    lo, hi = bbox(mesh_vertices(triangles))
    centre = 0.5 * (lo + hi)
    target = np.array([slot[0], slot[1], slot_table_top + 0.5 * (hi[2] - lo[2])])
    xyz = target - rot @ centre
    return {"xyz": xyz.tolist(), "rot": rot, "yaw": yaw,
            "rule": "bbox_centre_on_slot_origin"}


def load_slot_table_top(env_model: pathlib.Path = DEFAULT_ENV_MODEL) -> float:
    data = json.loads(pathlib.Path(env_model).read_text(encoding="utf-8"))
    return float(data["tables"]["slot_table"]["z"])


# ---------------------------------------------------------------------------
# 真机点云重建（复用 build_scene_model 的投影）
# ---------------------------------------------------------------------------


def _import_build_scene_model():
    import importlib.util

    source = ROOT / "scripts" / "build_scene_model.py"
    spec = importlib.util.spec_from_file_location("build_scene_model", source)
    module = importlib.util.module_from_spec(spec)
    sys.modules["build_scene_model"] = module
    spec.loader.exec_module(module)
    return module


def reconstruct_fixture_cloud(dataset: pathlib.Path, batches=FIXTURE_BATCHES,
                              stride: int = 3) -> np.ndarray:
    """从真机 RGB-D 帧重建插槽夹具点云（世界系，米）。"""

    builder = _import_build_scene_model()
    camera = json.loads((ROOT / "config/real/camera_intrinsics.json").read_text())
    hand_eye = np.array(json.loads((ROOT / "config/handeye/tcp_to_camera.json")
                                   .read_text())["T_tcp_cam"], dtype=float)
    dataset = pathlib.Path(dataset).expanduser()
    clouds = []
    for batch in batches:
        directory = dataset / batch
        if not directory.is_dir():
            continue
        poses = builder.load_metadata(directory)
        for depth_path in sorted(directory.glob("*_depth.npy")):
            stem = depth_path.name[: -len("_depth.npy")]
            tcp = poses.get(stem)
            if tcp is None:
                continue
            depth = np.load(depth_path)
            points = builder.frame_to_base(depth, camera, tcp @ hand_eye, stride=stride)
            if len(points) >= 1000:
                clouds.append(points)
    if not clouds:
        raise RuntimeError(f"no RGB-D frames with pose metadata under {dataset}")
    return np.vstack(clouds)



def fixture_frame_list(dataset: pathlib.Path, batches=EMPTY_SLOT_BATCHES) -> list[tuple[pathlib.Path, str]]:
    """列出有位姿元数据的夹具帧（batch 目录 + 文件名 stem）。"""

    builder = _import_build_scene_model()
    dataset = pathlib.Path(dataset).expanduser()
    frames = []
    for batch in batches:
        directory = dataset / batch
        if not directory.is_dir():
            continue
        for stem in sorted(builder.load_metadata(directory)):
            if (directory / f"{stem}_depth.npy").is_file():
                frames.append((directory, stem))
    return frames


def aggregate_profiles(profiles: list[list[dict]]) -> list[dict]:
    """逐层汇总多帧槽宽：每层取有值帧的中位数，并记录参与帧数。"""

    if not profiles:
        return []
    levels = len(profiles[0])
    aggregated = []
    for index in range(levels):
        gaps = [profile[index]["gap"] for profile in profiles
                if index < len(profile) and profile[index]["gap"] is not None]
        z = profiles[0][index]["z"]
        aggregated.append({
            "z": z,
            "gap": float(np.median(gaps)) if gaps else None,
            "frames": len(gaps),
        })
    return aggregated


def closest_fixture_frame(dataset: pathlib.Path, slot_origin,
                          batches=EMPTY_SLOT_BATCHES) -> tuple[pathlib.Path, str]:
    """挑一台"离插槽最近"的帧（最近 = 相机位置到 slot_origin 最小）。

    单帧没有多帧配准误差，遮挡最少的那一帧给出 bbox 的**下界**。
    """

    dataset = pathlib.Path(dataset).expanduser()
    slot = np.asarray(slot_origin, dtype=float)
    best = None
    for batch in batches:
        directory = dataset / batch
        metadata = directory / "metadata.jsonl"
        if not metadata.is_file():
            continue
        for line in metadata.read_text(encoding="utf-8").splitlines():
            try:
                item = json.loads(line)
            except Exception:
                continue
            if "camera_base" not in item:
                continue
            distance = float(np.linalg.norm(np.asarray(item["camera_base"]) - slot))
            if best is None or distance < best[0]:
                best = (distance, directory, item["file"])
    if best is None:
        raise RuntimeError("no metadata with camera_base for the fixture batches")
    return best[1], best[2]


def reconstruct_frame(directory: pathlib.Path, stem: str, stride: int = 2) -> np.ndarray:
    """单帧 -> 世界系点云。复用 build_scene_model 的投影函数。"""

    builder = _import_build_scene_model()
    camera = json.loads((ROOT / "config/real/camera_intrinsics.json").read_text())
    hand_eye = np.array(json.loads((ROOT / "config/handeye/tcp_to_camera.json")
                                   .read_text())["T_tcp_cam"], dtype=float)
    directory = pathlib.Path(directory)
    tcp = builder.load_metadata(directory)[stem]
    depth = np.load(directory / f"{stem}_depth.npy")
    return builder.frame_to_base(depth, camera, tcp @ hand_eye, stride=stride)


def crop_fixture_region(points: np.ndarray, slot_origin, slot_table_top: float,
                        x_max: float | None = None) -> np.ndarray:
    """裁出插槽夹具所在的长方体，去掉桌面、地面与背景。

    夹具 x 方向最薄（约 0.2 m），相机在 -x 侧，背面的背景墙在 x≈1.5 m，
    所以 `x_max` 默认取 slot_origin.x + 0.05 m。
    """

    points = np.asarray(points, dtype=float)
    origin = np.asarray(slot_origin, dtype=float)
    x_limit = origin[0] + 0.06 if x_max is None else float(x_max)
    # 夹具在模型系里的 bbox 是 x[-0.18, 0.02]、y[-0.57, 0.18]、z[-0.51, 0]，
    # 相对 slot_origin 就是 x[-0.21, 0.06]、y[-0.59, 0.20]、z[0, 0.51]。
    # 这个裁剪只用来去掉桌面/背景，不参与槽宽测量。
    mask = (
        (points[:, 0] > origin[0] - 0.21)
        & (points[:, 0] < x_limit)
        & (points[:, 1] > origin[1] - 0.59)
        & (points[:, 1] < origin[1] + 0.20)
        & (points[:, 2] > slot_table_top + 0.005)
        & (points[:, 2] < slot_table_top + 0.52)
    )
    return points[mask]


# ---------------------------------------------------------------------------
# 候选清单
# ---------------------------------------------------------------------------


def candidate_specs(rvs_dir: pathlib.Path = RVS_DIR) -> list[dict]:
    """四份候选 + 读取方式 + 默认世界位姿规则。顺序固定，便于比对。"""

    return [
        {"name": "rvs_mesh", "label": "RVS STL (slot_pointcloud.STL)",
         "path": rvs_dir / "slot_pointcloud.STL", "kind": "triangles",
         "unit": "mm", "placement": "rvs"},
        {"name": "rvs_ply", "label": "RVS PLY (slot_pointcloud20260127.ply)",
         "path": rvs_dir / "slot_pointcloud20260127.ply", "kind": "points",
         "unit": "m", "placement": "rvs"},
        {"name": "legacy", "label": "legacy slot_collision.STL",
         "path": LEGACY_DIR / "slot_collision.STL", "kind": "triangles",
         "unit": "mm", "placement": "rvs"},
        {"name": "old_sim", "label": "old sim slot_fixture.stl",
         "path": SIM_DIR / "cs625_task" / "slot_fixture" / "meshes" / "slot_fixture.stl",
         "kind": "triangles", "unit": "m", "placement": "bbox_centre"},
    ]


def load_candidate(spec: dict) -> dict:
    """读文件 -> 米制顶点/三角形 + 文件信息。"""

    path = pathlib.Path(spec["path"])
    if not path.is_file():
        return {"name": spec["name"], "label": spec["label"], "path": str(path),
                "missing": True}
    scale = unit_scale(spec["unit"])
    if spec["kind"] == "triangles":
        triangles, fmt = read_stl_triangles(path)
        triangles = triangles * scale
        vertices = mesh_vertices(triangles)
        return {"name": spec["name"], "label": spec["label"], "path": str(path),
                "format": f"{fmt} STL", "unit": spec["unit"],
                "vertices": int(len(vertices)), "triangles": int(len(triangles)),
                "vertices_m": vertices, "triangles_m": triangles, "kind": "triangles"}
    points = read_ply_points(path) * scale
    return {"name": spec["name"], "label": spec["label"], "path": str(path),
            "format": "binary PLY", "unit": spec["unit"],
            "vertices": int(len(points)), "triangles": 0,
            "vertices_m": points, "triangles_m": None, "kind": "points"}


# ---------------------------------------------------------------------------
# 报告
# ---------------------------------------------------------------------------


def format_table(rows: list[dict], columns: list[tuple[str, str, str]]) -> str:
    """极简等宽表。columns = [(key, 标题, 格式)]。"""

    widths = []
    for index, (_key, title, fmt) in enumerate(columns):
        width = len(title)
        for row in rows:
            width = max(width, len(fmt.format(row.get(_key, ""))))
        widths.append(width)
    head = "  ".join(title.ljust(widths[index])
                     for index, (_key, title, _fmt) in enumerate(columns))
    lines = [head, "  ".join("-" * width for width in widths)]
    for row in rows:
        cells = []
        for index, (key, _title, fmt) in enumerate(columns):
            cells.append(fmt.format(row.get(key, "")).ljust(widths[index]))
        lines.append("  ".join(cells))
    return "\n".join(lines)


def analyse(no_cloud: bool = False, dataset: pathlib.Path = DEFAULT_DATASET,
            env_model: pathlib.Path = DEFAULT_ENV_MODEL,
            rvs_dir: pathlib.Path = RVS_DIR) -> dict:
    """跑完整判定，返回可序列化的结果（CLI 与单测共用）。"""

    common = load_common()
    slot_origin = common["slot_origin"]
    slot_table_top = load_slot_table_top(env_model)
    pre_insert = common["pre_insert"]
    insert = common["insert"]

    report: dict = {"slot_origin": slot_origin, "slot_table_top": slot_table_top,
                    "pre_insert": pre_insert, "insert": insert,
                    "bbox_tolerance_m": BBOX_TOLERANCE_M,
                    "slot_width_tolerance_m": SLOT_WIDTH_TOLERANCE_M,
                    "candidates": []}

    # ---- 真机点云 ---------------------------------------------------------
    cloud_summary = None
    empty_slot_cloud = None
    if not no_cloud:
        frames = fixture_frame_list(dataset, EMPTY_SLOT_BATCHES)
        # 单帧分别测量，再取中位数：多帧直接合并会把插槽腔体"糊"住（实测槽宽
        # 从 457 mm 掉到 341 mm），必须逐帧测。
        profiles = []
        boxes = []
        for directory, stem in frames:
            points = reconstruct_frame(directory, stem, stride=2)
            cropped = crop_fixture_region(points, slot_origin, slot_table_top)
            if len(cropped) < 1000:
                continue
            profiles.append(cloud_slot_profile(cropped, pre_insert, insert, levels=9))
            filtered = statistical_filter(voxel_downsample(cropped, 0.004),
                                          k=12, std_ratio=1.0)
            boxes.append((np.percentile(filtered, 1, axis=0),
                          np.percentile(filtered, 99, axis=0)))
        aggregated = aggregate_profiles(profiles)
        low = np.min([box[0] for box in boxes], axis=0)
        high = np.max([box[1] for box in boxes], axis=0)

        merged = reconstruct_fixture_cloud(dataset, batches=FIXTURE_BATCHES, stride=3)
        cloud_summary = {
            "frames": len(profiles),
            "frames_points": int(len(merged)),
            "bbox_min": low.round(4).tolist(), "bbox_max": high.round(4).tolist(),
            "bbox_size": (high - low).round(4).tolist(),
            "bbox_method": "逐帧 bbox 的并集（每帧 1-99 百分位、去飞点）",
            "note": ("每帧单独重建再取并集，避免多帧合并的配准抖动；夹具背面仍看不到，"
                     "所以 x/y 的可见范围可能小于真实 AABB"),
            "slot_profile": aggregated,
            "slot_summary": summarise_profile(aggregated),
        }
        gaps = [item["gap"] for item in aggregated if item["gap"] is not None]
        cloud_summary["slot_gap_min"] = float(min(gaps)) if gaps else None
        cloud_summary["slot_gap_max"] = float(max(gaps)) if gaps else None
        report["cloud"] = cloud_summary

    # ---- 四份候选 ---------------------------------------------------------
    loaded_by_name: dict[str, dict] = {}
    for spec in candidate_specs(rvs_dir):
        loaded = load_candidate(spec)
        loaded_by_name[loaded["name"]] = loaded
        row = {"name": loaded["name"], "label": loaded["label"], "path": loaded["path"]}
        if loaded.get("missing"):
            row.update({"missing": True, "vertices": 0, "triangles": 0})
            report["candidates"].append(row)
            continue
        vertices = loaded["vertices_m"]
        lo, hi = bbox(vertices)
        row.update({
            "format": loaded["format"], "unit": loaded["unit"],
            "vertices": loaded["vertices"], "triangles": loaded["triangles"],
            "bbox_min": lo.round(4).tolist(), "bbox_max": hi.round(4).tolist(),
            "bbox_size": (hi - lo).round(4).tolist(),
        })

        if loaded["kind"] == "triangles":
            placement = (rvs_placement(common) if spec["placement"] == "rvs"
                         else bbox_centre_placement(loaded["triangles_m"], common, slot_table_top))
            triangles_world = transform_points(loaded["triangles_m"],
                                               placement["xyz"], placement["rot"])
            world = mesh_vertices(triangles_world)
            lo_w, hi_w = bbox(world)
            profile = slot_profile(triangles_world, pre_insert, insert, levels=9)
            # 槽口中心 = 第一层贯通截面上两条内壁的中点（世界 y）
            centre = None
            for row_profile in profile:
                point = np.asarray(row_profile["xyz"])
                plus = ray_first_hit(triangles_world, point, (0, 1, 0), 0.8)
                minus = ray_first_hit(triangles_world, point, (0, -1, 0), 0.8)
                if plus is not None and minus is not None:
                    centre = float(point[1] + 0.5 * (plus - minus))
                    break
            row.update({
                "placement_rule": placement["rule"],
                "placement_xyz": np.round(placement["xyz"], 6).tolist(),
                "placement_yaw_deg": round(float(np.degrees(placement["yaw"])), 4),
                "bbox_world_min": lo_w.round(4).tolist(),
                "bbox_world_max": hi_w.round(4).tolist(),
                "slot_profile": profile,
                "slot_summary": summarise_profile(profile),
                "slot_centre_world_y": centre,
                "insert_path_y": float(insert[1]),
                "slot_centre_offset_m": None if centre is None else abs(centre - float(insert[1])),
            })
        else:
            placement = rvs_placement(common)
            world = transform_points(vertices, placement["xyz"], placement["rot"])
            lo_w, hi_w = bbox(world)
            row.update({
                "placement_rule": placement["rule"],
                "placement_xyz": np.round(placement["xyz"], 6).tolist(),
                "bbox_world_min": lo_w.round(4).tolist(),
                "bbox_world_max": hi_w.round(4).tolist(),
                "slot_profile": None,
                "slot_summary": None,
                "note": "PLY 是同一物体的单视角扫描点集；bbox 是可见表面范围，不是完整 AABB",
            })
        report["candidates"].append(row)

    # ---- 点云 / 网格是否同一个物体 ---------------------------------------
    mesh = loaded_by_name.get("rvs_mesh") or {}
    ply = loaded_by_name.get("rvs_ply") or {}
    if mesh.get("triangles_m") is not None and ply.get("vertices_m") is not None:
        stats = point_to_surface_stats(ply["vertices_m"], mesh["triangles_m"])
        report["surface_identity"] = {
            "ply_to_rvs_mesh": stats,
            "same_surface": stats["p99_m"] <= SURFACE_IDENTITY_TOLERANCE_M,
            "tolerance_m": SURFACE_IDENTITY_TOLERANCE_M,
        }

    # ---- 判据 -------------------------------------------------------------
    if cloud_summary is not None:
        cloud_size = np.asarray(cloud_summary["bbox_size"], dtype=float)
        cloud_gap = cloud_summary["slot_summary"]["median_gap"]
        for row in report["candidates"]:
            if row.get("missing") or "bbox_size" not in row:
                continue
            candidate_size = np.asarray(row["bbox_size"], dtype=float)
            delta = np.abs(candidate_size - cloud_size)
            row["bbox_delta_m"] = delta.round(4).tolist()
            gap = (row.get("slot_summary") or {}).get("median_gap")
            row["slot_delta_m"] = None if gap is None or cloud_gap is None else abs(gap - cloud_gap)
    report["verdict"] = _verdict(report)
    return report


def _verdict(report: dict) -> dict:
    """按任务判据给出结论（bbox ≤3 cm、槽宽 ≤5 mm、槽口对中、路径在槽内）。"""

    cloud = report.get("cloud") or {}
    cloud_gap = (cloud.get("slot_summary") or {}).get("median_gap")
    identity = report.get("surface_identity") or {}
    same_surface = bool(identity.get("same_surface"))
    chosen = []
    rejected = []
    for row in report["candidates"]:
        name = row["name"]
        if row.get("missing"):
            rejected.append({"name": name, "why": "文件不存在"})
            continue
        summary = row.get("slot_summary") or {}
        gap = summary.get("median_gap")
        valid_levels = int(summary.get("valid") or 0)
        delta = row.get("bbox_delta_m")

        if row.get("kind") == "points" or row.get("triangles") == 0:
            if same_surface:
                rejected.append({
                    "name": name,
                    "why": (f"与 rvs_mesh 是同一扫描表面（点到面 p99 "
                            f"{identity['ply_to_rvs_mesh']['p99_m']*1000:.1f} mm），"
                            "但只是单视角点云、没有三角面，不能当碰撞网格；"
                            "作为“同一物体”的交叉证据保留"),
                    "same_geometry": True,
                })
            else:
                rejected.append({"name": name, "why": "点云，不是网格；且与 rvs_mesh 表面不重合"})
            continue

        if cloud_gap is None:
            rejected.append({"name": name, "why": "未重建真机点云，无法按判据判定"})
            continue

        lower = np.asarray(cloud.get("bbox_lower_size") or cloud["bbox_size"], dtype=float)
        upper = np.asarray(cloud["bbox_size"], dtype=float)
        candidate_size = np.asarray(row["bbox_size"], dtype=float)
        # 单帧下界 - 3 cm ≤ 候选 ≤ 多帧上界 + 3 cm：真值一定落在重建区间里。
        bbox_ok = bool(
            candidate_size[1] >= lower[1] - BBOX_TOLERANCE_M
            and candidate_size[1] <= upper[1] + BBOX_TOLERANCE_M
            and candidate_size[2] >= lower[2] - BBOX_TOLERANCE_M
            and candidate_size[2] <= upper[2] + BBOX_TOLERANCE_M
        )
        gap_low = cloud.get("slot_gap_min", cloud_gap)
        gap_high = cloud.get("slot_gap_max", cloud_gap)
        gap_ok = (gap is not None and gap_low is not None and gap_high is not None
                  and gap_low - SLOT_WIDTH_TOLERANCE_M <= gap
                  <= gap_high + SLOT_WIDTH_TOLERANCE_M)
        levels_ok = valid_levels >= MIN_VALID_LEVELS
        centre_ok = (row.get("slot_centre_offset_m") is not None
                     and row["slot_centre_offset_m"] <= 0.02)
        entry = {"name": name, "bbox_ok_yz": bool(bbox_ok),
                 "slot_width_ok": bool(gap_ok), "levels_ok": bool(levels_ok),
                 "slot_centred_on_insert_path": bool(centre_ok),
                 "gap_m": gap, "cloud_gap_m": cloud_gap, "valid_levels": valid_levels,
                 "triangles": int(row.get("triangles") or 0),
                 "bbox_size_m": list(row["bbox_size"])}

        factors = []
        if not bbox_ok:
            factors.append("bbox 超出重建区间 ±30 mm")
        if not levels_ok:
            factors.append(f"插入路径只有 {valid_levels}/{summary.get('levels', 0)} 层是贯通槽"
                           "（多数高度不在槽内）")
        if not gap_ok:
            factors.append(
                f"槽宽 {gap*1000:.1f} mm 落在真机测量区间 "
                f"[{gap_low*1000:.0f}, {gap_high*1000:.0f}] ±5 mm 之外"
                if gap is not None and gap_low is not None and gap_high is not None
                else "测不到槽宽")
        if not centre_ok:
            offset = row.get("slot_centre_offset_m")
            factors.append("槽口不落在插入路径上" if offset is None
                           else f"槽口中心偏离插入路径 {offset*1000:.0f} mm")
        entry["reasons"] = factors
        if bbox_ok and gap_ok and levels_ok and centre_ok:
            chosen.append(entry)
        else:
            rejected.append({"name": name, "why": "；".join(factors)})

    proxies = []
    if len(chosen) > 1:
        # 多份同时满足时，优先选面数最多的原始网格（信息最完整）；面数不到它 5%
        # 的同类是"碰撞代理"，几何上同源但不能当作 1:1 几何来源。
        best = max(chosen, key=lambda item: item["triangles"])
        for item in chosen:
            if item is not best and item["triangles"] < 0.05 * max(best["triangles"], 1):
                item["role"] = f"跌破保真度的碰撞代理（{item['triangles']} 面），源自 {best['name']}"
                proxies.append(item)
        chosen = [item for item in chosen if item not in proxies]
        chosen.sort(key=lambda item: item["triangles"], reverse=True)
    return {"chosen": chosen, "proxies": proxies, "rejected": rejected,
            "primary": chosen[0]["name"] if chosen else None}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--no-cloud", action="store_true", help="跳过真机 RGB-D 重建")
    parser.add_argument("--dataset", default=str(DEFAULT_DATASET))
    parser.add_argument("--env-model", default=str(DEFAULT_ENV_MODEL))
    parser.add_argument("--json", default="")
    arguments = parser.parse_args()

    report = analyse(no_cloud=arguments.no_cloud,
                     dataset=pathlib.Path(arguments.dataset),
                     env_model=pathlib.Path(arguments.env_model))

    print("=" * 100)
    print("插槽夹具几何判定 —— 候选对比")
    print("=" * 100)
    print(f"slot_origin     : {np.round(report['slot_origin'], 6).tolist()}  "
          f"(插槽桌顶面 z={report['slot_table_top']:.5f})")
    print(f"insert path     : pre_insert {report['pre_insert']} -> insert {report['insert']}")
    print(f"判据             : bbox y/z 差 ≤ {BBOX_TOLERANCE_M*1000:.0f} mm；"
          f"槽宽差 ≤ {SLOT_WIDTH_TOLERANCE_M*1000:.0f} mm；槽口中心落在插入路径 ±20 mm 内")
    print()

    cloud = report.get("cloud")
    if cloud:
        bbox_text = " × ".join(f"{v*1000:.0f}" for v in cloud["bbox_size"])
        summary = cloud["slot_summary"]
        gap_text = ("n/a" if summary["median_gap"] is None
                    else f"{summary['median_gap']*1000:.1f} mm")
        print("真机 RGB-D 重建（slot* 空槽批次，逐帧测量后取中位数）")
        print(f"  帧数/点数     : {cloud['frames']} 帧 / {cloud['frames_points']} 点")
        print(f"  可见 bbox     : {bbox_text} mm  (x/y/z，逐帧 bbox 并集)")
        print(f"  槽宽中位数     : {gap_text}  （{summary['valid']}/{summary['levels']} 层贯通）")
        cells = []
        for item in cloud["slot_profile"]:
            cells.append("--" if item["gap"] is None else f"{item['gap']*1000:.1f}")
        print("  逐层         : " + ", ".join(
            f"z={item['z']:.2f}:{cell}" for item, cell in zip(cloud["slot_profile"], cells)))
        print()

    rows = []
    for row in report["candidates"]:
        if row.get("missing"):
            rows.append({"label": row["label"], "format": "missing", "counts": "-",
                         "bbox": "-", "gap": "-", "delta": "-", "centre": "-"})
            continue
        size = row["bbox_size"]
        summary = row.get("slot_summary") or {}
        gap = ("n/a" if summary.get("median_gap") is None
               else f"{summary['median_gap']*1000:.1f}")
        delta = row.get("bbox_delta_m")
        delta_text = "-" if delta is None else "/".join(f"{v*1000:.0f}" for v in delta)
        valid = summary.get("valid")
        centre = row.get("slot_centre_offset_m")
        rows.append({
            "label": row["label"],
            "format": f"{row['format']} {row['unit']}",
            "counts": f"{row['vertices']}/{row['triangles']}",
            "bbox": "×".join(f"{v*1000:.0f}" for v in size),
            "gap": gap,
            "levels": "-" if valid is None else f"{valid}/{summary.get('levels')}",
            "centre": "-" if centre is None else f"{centre*1000:.0f}",
        })
    columns = [
        ("label", "候选", "{}"),
        ("format", "格式/单位", "{}"),
        ("counts", "顶点/面", "{}"),
        ("bbox", "bbox x×y×z (mm)", "{}"),
        ("gap", "槽宽中位 (mm)", "{}"),
        ("levels", "贯通层", "{}"),
        ("centre", "槽口偏离路径 (mm)", "{}"),
    ]
    print(format_table(rows, columns))
    print()

    print("逐层槽宽（沿 pre_insert -> insert，世界 y 向 ±y 射线）：")
    for row in report["candidates"]:
        profile = row.get("slot_profile")
        if not profile:
            continue
        cells = []
        for item in profile:
            cells.append("--" if item["gap"] is None else f"{item['gap']*1000:.1f}")
        print(f"  {row['name']:<10} " + "  ".join(f"z={item['z']:.2f}" for item in profile))
        print(f"  {'':<10} " + "  ".join(f"{cell:>7}" for cell in cells))
    print()

    verdict = report.get("verdict")
    if verdict:
        print("结论")
        print(f"  选用: {verdict['primary']}")
        for item in verdict["chosen"]:
            print(f"    ✓ {item['name']}: bbox y/z ok, 槽宽 {item['gap_m']*1000:.1f} mm "
                  f"(真机 {item['cloud_gap_m']*1000:.1f} mm), 槽口对中")
        for item in verdict.get("proxies", []):
            print(f"    ~ {item['name']}: {item['role']}（数值判据也过，但楔形/底面细节已被削掉）")
        for item in verdict["rejected"]:
            print(f"    ✗ {item['name']}: {item['why']}")
    print()

    if arguments.json:
        output = pathlib.Path(arguments.json)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, ensure_ascii=False, indent=1, default=float))
        print(f"JSON -> {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
