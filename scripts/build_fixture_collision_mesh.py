#!/usr/bin/env python3
"""把真机插槽夹具几何转成 MoveIt/Gazebo 用的米制碰撞网格。

输入
----
`slot_pointcloud.STL`：师兄 RobotVisionSuite 导出的真机夹具网格（ASCII STL，
毫米制，96,760 面，模型坐标系的原点在夹具顶面/前角，不是零件中心）。
`scripts/analyze_fixture_geometry.py` 已经证明它和真机 RGB-D 重建的槽宽、
槽口位置、顶面高度一致；仓库里的 PLY 只是同一表面的单视角点云子集。

输出
----
`src/cs625_simulation/assets/cs625_real_cell/slot_fixture/meshes/slot_fixture_collision.stl`
二进制 STL、**米制**、**模型坐标系**（与 `slot_origin_q` 标定同一个系）。
面数 ≤ 80k；槽口形状用 `analyze_fixture_geometry.slot_profile()` 复核：
抽稀前后槽宽差必须 ≤ 1 mm，bbox 必须完全一致。

为什么不用 legacy 的 360 面 `slot_collision.STL`
----------------------------------------------
它是同一几何的碰撞代理：底部被裁掉 16.8 mm，楔形槽被简化成直槽。数值上
槽宽刚好在容差内，但"插入是否可行"恰恰取决于楔形与槽口形状，所以本任务
要的是原始几何，不是代理。

用法
----
    python3 scripts/build_fixture_collision_mesh.py
    python3 scripts/build_fixture_collision_mesh.py --target-faces 60000 --report /tmp/x.json
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import pathlib
import struct
import sys

import numpy as np

sys.stdout.reconfigure(line_buffering=True)

ROOT = pathlib.Path(__file__).resolve().parents[1]
RVS_STL = pathlib.Path("/mnt/d/Program Files (x86)/RobotVisionSuite/runtime/Model/slot_pointcloud.STL")
OUTPUT = (ROOT / "src" / "cs625_simulation" / "assets" / "cs625_real_cell"
          / "slot_fixture" / "meshes" / "slot_fixture_collision.stl")
DEFAULT_TARGET_FACES = 60000
MAX_FACES = 80000


def _load_analysis_module():
    """复用判定脚本里的 STL 解析 / 槽宽测量，避免第二套实现。"""

    source = ROOT / "scripts" / "analyze_fixture_geometry.py"
    spec = importlib.util.spec_from_file_location("analyze_fixture_geometry", source)
    module = importlib.util.module_from_spec(spec)
    sys.modules["analyze_fixture_geometry"] = module
    spec.loader.exec_module(module)
    return module


def write_binary_stl(path: pathlib.Path, triangles: np.ndarray) -> int:
    """(M, 3, 3) 三角形 -> 二进制 STL；返回写入的面数。"""

    triangles = np.asarray(triangles, dtype=np.float32).reshape(-1, 3, 3)
    path = pathlib.Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as handle:
        handle.write(b"cs625 real fixture collision mesh".ljust(80, b"\0"))
        handle.write(struct.pack("<I", len(triangles)))
        for triangle in triangles:
            normal = np.cross(triangle[1] - triangle[0], triangle[2] - triangle[0])
            norm = float(np.linalg.norm(normal))
            normal = normal / norm if norm > 1e-12 else np.zeros(3)
            handle.write(struct.pack("<3f", *normal))
            handle.write(struct.pack("<9f", *triangle.reshape(-1)))
            handle.write(struct.pack("<H", 0))
    return int(len(triangles))


def decimate_triangles(triangles: np.ndarray, target_faces: int) -> np.ndarray:
    """二次误差抽稀到 `target_faces` 面，返回 (M, 3, 3)。

    STL 的顶点是逐面重复的，先焊接（remove_duplicated_vertices）再抽稀，
    否则二次误差度量会把每个三角形当成独立面片。
    """

    import open3d as o3d

    triangles = np.asarray(triangles, dtype=float)
    vertices = triangles.reshape(-1, 3)
    faces = np.arange(len(vertices), dtype=np.int32).reshape(-1, 3)
    mesh = o3d.geometry.TriangleMesh(
        o3d.utility.Vector3dVector(vertices),
        o3d.utility.Vector3iVector(faces),
    )
    mesh.remove_duplicated_vertices()
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_unreferenced_vertices()
    simplified = mesh.simplify_quadric_decimation(int(target_faces))
    return np.asarray(simplified.vertices)[np.asarray(simplified.triangles)].astype(float)


def build(input_path: pathlib.Path = RVS_STL, output_path: pathlib.Path = OUTPUT,
          target_faces: int = DEFAULT_TARGET_FACES, common=None) -> dict:
    """读入毫米制真机 STL -> 米制抽稀碰撞网格 -> 写文件 + 自检。"""

    analysis = _load_analysis_module()
    if target_faces > MAX_FACES:
        raise SystemExit(f"--target-faces {target_faces} exceeds the {MAX_FACES} hard limit")

    triangles_mm, source_format = analysis.read_stl_triangles(pathlib.Path(input_path))
    triangles_m = triangles_mm * analysis.unit_scale("mm")
    original_faces = int(len(triangles_m))
    original_bbox = analysis.bbox(triangles_m.reshape(-1, 3))
    original_size = original_bbox[1] - original_bbox[0]

    reduced = decimate_triangles(triangles_m, target_faces)
    reduced_bbox = analysis.bbox(reduced.reshape(-1, 3))
    reduced_size = reduced_bbox[1] - reduced_bbox[0]

    common = common or analysis.load_common()
    placement = analysis.rvs_placement(common)
    original_world = analysis.transform_points(triangles_m, placement["xyz"], placement["rot"])
    reduced_world = analysis.transform_points(reduced, placement["xyz"], placement["rot"])
    original_profile = analysis.slot_profile(
        original_world, common["pre_insert"], common["insert"], levels=9)
    reduced_profile = analysis.slot_profile(
        reduced_world, common["pre_insert"], common["insert"], levels=9)
    original_summary = analysis.summarise_profile(original_profile)
    reduced_summary = analysis.summarise_profile(reduced_profile)

    gap_delta = None
    if original_summary["median_gap"] is not None and reduced_summary["median_gap"] is not None:
        gap_delta = abs(original_summary["median_gap"] - reduced_summary["median_gap"])

    written = write_binary_stl(pathlib.Path(output_path), reduced)
    size_bytes = pathlib.Path(output_path).stat().st_size

    report = {
        "input": str(input_path),
        "input_format": source_format,
        "input_faces": original_faces,
        "output": str(pathlib.Path(output_path).relative_to(ROOT)
                      if pathlib.Path(output_path).is_relative_to(ROOT) else output_path),
        "output_faces": written,
        "output_vertices": int(len(reduced.reshape(-1, 3))),
        "output_bytes": int(size_bytes),
        "output_unit": "m",
        "bbox_min_m": reduced_bbox[0].round(6).tolist(),
        "bbox_max_m": reduced_bbox[1].round(6).tolist(),
        "bbox_size_m": reduced_size.round(6).tolist(),
        "original_bbox_size_m": original_size.round(6).tolist(),
        "slot_width_original_m": original_summary["median_gap"],
        "slot_width_reduced_m": reduced_summary["median_gap"],
        "slot_width_delta_m": gap_delta,
        "profile_original_m": [item["gap"] for item in original_profile],
        "profile_reduced_m": [item["gap"] for item in reduced_profile],
    }
    report["bbox_preserved"] = bool(np.allclose(original_size, reduced_size, atol=1e-6))
    report["slot_width_preserved"] = bool(gap_delta is not None and gap_delta <= 0.001)
    report["within_face_limit"] = bool(written <= MAX_FACES)
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--input", default=str(RVS_STL))
    parser.add_argument("--output", default=str(OUTPUT))
    parser.add_argument("--target-faces", type=int, default=DEFAULT_TARGET_FACES)
    parser.add_argument("--report", default="")
    arguments = parser.parse_args()

    report = build(pathlib.Path(arguments.input), pathlib.Path(arguments.output),
                   arguments.target_faces)
    print("=" * 92)
    print("真机夹具碰撞网格转换（mm -> m，抽稀）")
    print("=" * 92)
    print(f"输入    : {report['input']}  ({report['input_format']} STL, "
          f"{report['input_faces']} 面, 毫米)")
    print(f"输出    : {report['output']}")
    print(f"输出    : {report['output_faces']} 面 / {report['output_vertices']} 顶点 / "
          f"{report['output_bytes']/1048576:.2f} MiB / 米制")
    print(f"bbox    : {np.round(report['bbox_min_m'], 4).tolist()} -> "
          f"{np.round(report['bbox_max_m'], 4).tolist()} m")
    print(f"尺寸    : {' × '.join(f'{v:.4f}' for v in report['bbox_size_m'])} m")
    print(f"槽宽    : 原始 {report['slot_width_original_m']:.6f} m -> "
          f"抽稀 {report['slot_width_reduced_m']:.6f} m "
          f"(差 {report['slot_width_delta_m']*1000:.2f} mm)")
    print(f"自检    : bbox 保持 {report['bbox_preserved']}；"
          f"槽宽保持 {report['slot_width_preserved']}；"
          f"面数 ≤ {MAX_FACES} {report['within_face_limit']}")
    if not (report["bbox_preserved"] and report["slot_width_preserved"] and report["within_face_limit"]):
        print("FAILED: 抽稀改变了 bbox 或槽宽")
        return 1

    if arguments.report:
        path = pathlib.Path(arguments.report)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report, ensure_ascii=False, indent=1))
        print(f"JSON -> {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
