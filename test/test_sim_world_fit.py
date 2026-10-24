#!/usr/bin/env python3
"""Unit tests for scripts/build_sim_world.py (pure geometry, no ROS/Gazebo).

What is actually being pinned here
----------------------------------
1. The bbox alignment primitive: a pose computed from a bounding box must put
   that box's centre exactly on the requested point, and (for the seating
   variant) put its bottom face exactly on the requested plane.  The check is
   numeric on the 8 transformed corners, not a restatement of the formula.
2. The table boxes: their xy extents and top surfaces must equal the measured
   numbers, with no shrink/scale factor.
3. The generated world: valid XML, exactly two table models, exactly two
   ``<include>`` entries, and poses that match the plan.
4. The report: a row above the warning threshold must raise a warning, one
   below must not.
5. The frame decision: when the module model frame equals the CAD frame the
   measured pose is used verbatim; when a synthetic model frame is shifted, the
   code must fall back to bbox alignment.  Both branches are exercised.
6. The committed artifacts must be exactly what the generator produces, so the
   world/yaml cannot drift from the measurements without a test failing.

Everything imports the script by path, the same way test/test_sync_task_world.py
does, so running ``python3 -m pytest test/test_sim_world_fit.py -q`` from the
repository root needs no environment sourcing.
"""

from __future__ import annotations

import copy
import importlib.util
import pathlib
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml


HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
SCRIPT = ROOT / "scripts" / "build_sim_world.py"

spec = importlib.util.spec_from_file_location("build_sim_world", SCRIPT)
build_sim_world = importlib.util.module_from_spec(spec)
sys.modules["build_sim_world"] = build_sim_world
spec.loader.exec_module(build_sim_world)


CONFIG_PATH = ROOT / "config/cs625_real_cell_scene.yaml"
WORLD_PATH = ROOT / "src/cs625_simulation/worlds/cs625_real_cell.sdf"


def _committed_env_model():
    """交付物自己记录的实测快照；用它而不是 latest，测试才不会随新采集漂移。"""

    if not CONFIG_PATH.is_file():
        return None
    try:
        scene = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
        return scene[build_sim_world.SCENE_KEY]["ros__parameters"]["env_model"]
    except (KeyError, TypeError, yaml.YAMLError):
        return None


try:
    MEASUREMENTS = build_sim_world.load_measurements(_committed_env_model())
    PLAN = build_sim_world.plan_scene(MEASUREMENTS)
except FileNotFoundError as error:  # pragma: no cover - 只在没有实测数据时发生
    pytest.skip(f"measured environment model unavailable: {error}", allow_module_level=True)


# ---------------------------------------------------------------------------
# 1. bbox alignment primitive
# ---------------------------------------------------------------------------


def test_bbox_alignment_puts_bbox_centre_on_target_and_bottom_on_plane():
    """数值验证：变换后 8 个角点的中心 == 目标点，底面 == 桌面。"""

    bbox_min, bbox_max = MEASUREMENTS["fixture_bbox"]["min"], MEASUREMENTS["fixture_bbox"]["max"]
    target = np.array([1.241629, 0.181709, 0.442000])
    yaw = 0.37  # 故意用非零 yaw，验证旋转参与了对齐

    pose = build_sim_world.pose_from_bbox_alignment(bbox_min, bbox_max, target, yaw)

    rot = build_sim_world.rotation_rpy(*pose["rpy"])
    moved = build_sim_world.transformed_points(build_sim_world.bbox_corners(bbox_min, bbox_max), pose["xyz"], rot)
    assert np.allclose(moved.mean(axis=0), target, atol=1e-12)
    # 只允许 yaw：roll/pitch 必须严格为 0，夹具与工件都得正立。
    assert pose["rpy"][0] == 0.0 and pose["rpy"][1] == 0.0
    assert pose["rpy"][2] == pytest.approx(yaw, abs=0.0)

    # 贴桌面变体：底面严格等于 plane_z，中心 xy 严格等于目标 xy。
    plane_z = -0.04073
    seated = build_sim_world.seat_bbox_on_plane(bbox_min, bbox_max, target[:2], plane_z, yaw)
    rotated = build_sim_world.world_bbox(bbox_min, bbox_max, seated["xyz"], build_sim_world.rotation_rpy(*seated["rpy"]))
    centre = 0.5 * (rotated[0] + rotated[1])
    assert rotated[0][2] == pytest.approx(plane_z, abs=1e-12)
    assert centre[0] == pytest.approx(target[0], abs=1e-12)
    assert centre[1] == pytest.approx(target[1], abs=1e-12)


def test_fixture_model_origin_lands_on_slot_origin():
    """夹具用真机 RVS 几何：模型原点就是 slot_origin（标定语义），只取 yaw。

    旧仿真夹具用的是"bbox 中心对齐"，那套规则在真机夹具上会把整个包围盒偏移
    （模型原点不在零件中心），所以 2026-10-24 起改成 RVS 语义。
    """

    fixture = PLAN["fixture"]
    slot_origin = MEASUREMENTS["slot_origin"]

    xyz = fixture["pose"]["xyz"]
    rpy = fixture["pose"]["rpy"]
    assert xyz[0] == pytest.approx(slot_origin[0], abs=1e-9)
    assert xyz[1] == pytest.approx(slot_origin[1], abs=1e-9)
    assert xyz[2] == pytest.approx(slot_origin[2], abs=1e-9)
    # 只允许绕 z 的 yaw，roll/pitch 必须是 0（夹具正立）
    assert rpy[0] == pytest.approx(0.0, abs=1e-12)
    assert rpy[1] == pytest.approx(0.0, abs=1e-12)
    # 夹具本体必须落进插槽桌上方：bbox 底面不高于桌面 5 cm
    slot_table_top = PLAN["tables"]["slot_table"]["top_z"]
    assert fixture["bbox_world"]["min"][2] >= slot_table_top - 0.05
    # 模型原点对齐 slot_origin，所以"偏差"指的是原点对 reference 的差
    assert fixture["alignment"] == "model_origin"
    assert fixture["model_uri"] == build_sim_world.FIXTURE_URI


# ---------------------------------------------------------------------------
# 2. table boxes
# ---------------------------------------------------------------------------


def test_table_boxes_match_measured_limits():
    """桌面盒子：xy = 实测范围，顶面 = 实测 z，没有缩放。"""

    measured = MEASUREMENTS["tables"]
    tables = PLAN["tables"]

    workpiece = tables["workpiece_table"]
    assert workpiece["xy_min"] == pytest.approx([0.26, 0.28])
    assert workpiece["xy_max"] == pytest.approx([1.18, 1.08])
    assert workpiece["top_z"] == pytest.approx(measured["workpiece_table"]["z"], abs=0.0)
    assert workpiece["centre"][:2] == pytest.approx([0.72, 0.68])
    assert workpiece["size"][:2] == pytest.approx([0.92, 0.80])
    assert workpiece["centre"][2] == pytest.approx((workpiece["top_z"] + build_sim_world.DEFAULT_FLOOR_TOP_Z) / 2.0)

    slot = tables["slot_table"]
    assert slot["xy_min"] == pytest.approx([0.78, -0.28])
    assert slot["xy_max"] == pytest.approx([1.70, 0.64])
    assert slot["top_z"] == pytest.approx(measured["slot_table"]["z"], abs=0.0)
    assert slot["centre"][:2] == pytest.approx([1.24, 0.18])
    assert slot["size"][:2] == pytest.approx([0.92, 0.92])

    # 盒子从地面顶面长到桌面：底面统一、厚度由实测 z 决定（不是写死的 0.05）。
    for table in tables.values():
        assert table["bottom_z"] == pytest.approx(build_sim_world.DEFAULT_FLOOR_TOP_Z)
        assert table["size"][2] == pytest.approx(table["top_z"] - table["bottom_z"])
        assert table["top_z"] > table["bottom_z"]

    # 实测范围确实叠在一起，且提示信息被算出来了（脚本会打印）。
    overlap = PLAN["overlap"]
    assert overlap is not None
    assert overlap["xy_min"] == pytest.approx([0.78, 0.28])
    assert overlap["xy_max"] == pytest.approx([1.18, 0.64])
    assert any("重叠" in note for note in PLAN["notes"])


# ---------------------------------------------------------------------------
# 3. generated world file structure
# ---------------------------------------------------------------------------


def test_generated_world_parses_with_two_tables_and_two_includes():
    """世界必须是合法 XML，含 2 张桌子 + 2 个 include，位姿与规划一致。"""

    text = build_sim_world.build_world_sdf(PLAN, MEASUREMENTS)
    assert "nan" not in text.lower()
    root = ET.fromstring(text)
    assert root.tag == "sdf"
    world = root.find("world")
    assert world is not None
    assert world.get("name") == build_sim_world.WORLD_NAME

    models = {model.get("name"): model for model in world.findall("model")}
    assert {"workpiece_table", "slot_table"} <= set(models)
    assert "ground_plane" in models

    for name in ("workpiece_table", "slot_table"):
        table = PLAN["tables"][name]
        pose = [float(v) for v in models[name].find("pose").text.split()]
        assert pose[:3] == pytest.approx(table["centre"], abs=1e-9)
        size = [float(v) for v in models[name].find(".//box/size").text.split()]
        assert size == pytest.approx(table["size"], abs=1e-6)

    includes = world.findall("include")
    assert len(includes) == 2
    by_name = {element.find("name").text: element for element in includes}
    assert by_name.keys() == {"slot_fixture", "shielding_module"}
    assert by_name["slot_fixture"].find("uri").text == build_sim_world.FIXTURE_URI
    assert by_name["shielding_module"].find("uri").text == "model://cs625_task/shielding_module"
    assert by_name["slot_fixture"].find("static") is not None

    for key, name in (("fixture", "slot_fixture"), ("module", "shielding_module")):
        pose = [float(v) for v in by_name[name].find("pose").text.split()]
        assert pose == pytest.approx(PLAN[key]["pose"]["xyz"] + PLAN[key]["pose"]["rpy"], abs=1e-9)


def test_committed_world_and_yaml_are_what_the_generator_writes():
    """交付物不能被手改：磁盘上的文件必须等于“用同样的实测数据重新生成”的结果。"""

    assert WORLD_PATH.is_file(), f"missing generated world: {WORLD_PATH}"
    assert CONFIG_PATH.is_file(), f"missing generated scene config: {CONFIG_PATH}"

    assert WORLD_PATH.read_text(encoding="utf-8") == build_sim_world.build_world_sdf(PLAN, MEASUREMENTS)
    assert CONFIG_PATH.read_text(encoding="utf-8") == build_sim_world.build_scene_yaml(PLAN, MEASUREMENTS, WORLD_PATH)

    scene = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    parameters = scene[build_sim_world.SCENE_KEY]["ros__parameters"]
    assert parameters["fixture"]["pose_world"] == pytest.approx(PLAN["fixture"]["pose"]["xyz"] + PLAN["fixture"]["pose"]["rpy"], abs=1e-6)
    assert parameters["module"]["pose_world"] == pytest.approx(PLAN["module"]["pose"]["xyz"] + PLAN["module"]["pose"]["rpy"], abs=1e-6)
    assert parameters["tables"]["workpiece_table"]["top_z"] == pytest.approx(PLAN["tables"]["workpiece_table"]["top_z"], abs=1e-9)
    assert parameters["tables"]["slot_table"]["top_z"] == pytest.approx(PLAN["tables"]["slot_table"]["top_z"], abs=1e-9)
    assert parameters["module"]["alignment"] == PLAN["module"]["alignment"]


# ---------------------------------------------------------------------------
# 4. report warnings
# ---------------------------------------------------------------------------


def test_report_warns_only_above_threshold():
    """偏差 > 阈值的行必须出警告；恰好等于阈值的行不出。"""

    rows = [
        {"model": "small", "bbox_centre_world": [0, 0, 0], "reference": [0, 0, 0],
         "reference_name": "ref", "axes": "xyz", "alignment": "bbox_centre", "deviation": 0.004},
        {"model": "exactly_threshold", "bbox_centre_world": [0, 0, 0], "reference": [0, 0, 0],
         "reference_name": "ref", "axes": "xyz", "alignment": "bbox_centre", "deviation": 0.02},
        {"model": "large", "bbox_centre_world": [0, 0, 0], "reference": [0, 0, 0],
         "reference_name": "ref", "axes": "xyz", "alignment": "bbox_centre", "deviation": 0.023},
    ]
    warnings = build_sim_world.deviations_requiring_warning(rows, threshold=0.02)
    assert len(warnings) == 1
    assert "large" in warnings[0] and "23.0 mm" in warnings[0]

    # 真实数据：夹具 0 mm 不报警，工件 22.8 mm 报警（且是唯一一条）。
    real = build_sim_world.deviations_requiring_warning(PLAN["rows"], PLAN["warn_threshold_m"])
    assert len(real) == 1
    assert "shielding_module" in real[0]

    table = build_sim_world.format_fit_table(PLAN["rows"])
    assert "slot_fixture" in table and "shielding_module" in table
    assert "slot_origin" in table and "cad_centre" in table


# ---------------------------------------------------------------------------
# 5. frame decision (module: measured CAD pose vs bbox alignment)
# ---------------------------------------------------------------------------


def test_module_uses_measured_cad_pose_when_the_model_frame_is_the_cad_frame():
    """bbox 中心与 CAD 中心差 22.8 mm < 50 mm -> 模型系就是 CAD 系，用实测位姿。"""

    module = PLAN["module"]
    assert module["alignment"] == "measured_cad_pose"
    assert module["frame_deviation_m"] < build_sim_world.FRAME_THRESHOLD_M
    assert module["pose"]["xyz"] == pytest.approx(MEASUREMENTS["workpiece"]["xyz"], abs=1e-12)
    assert module["deviation_m"] == pytest.approx(module["frame_deviation_m"], abs=1e-12)

    # 22.8 mm 的来源是“参考点定义”，不是坐标系：用公开的 CAD 偏移复算实测 CAD
    # 中心，残差必须是亚毫米；bbox 中心与那个偏移点相差 ~22.8 mm。
    assert module["cad_definition_residual_m"] < 1e-3
    assert module["bbox_centre_minus_cad_offset_m"] == pytest.approx(module["frame_deviation_m"], abs=1e-9)
    assert module["bbox_centre_minus_cad_offset_m"] > 0.02
    published = np.array(build_sim_world.PUBLISHED_CAD_CENTRE_OFFSET_M)
    assert np.allclose(module["cad_offset_model"], published, atol=5e-4)


def test_module_falls_back_to_bbox_alignment_when_the_model_frame_is_shifted():
    """把模型 bbox 整体挪 0.1 m（等价于换了一套模型系）-> 必须改用 bbox 对齐。"""

    shifted = copy.deepcopy(MEASUREMENTS)
    shifted["module_bbox"]["min"] = [v + [0.10, 0.0, 0.0][i] for i, v in enumerate(shifted["module_bbox"]["min"])]
    shifted["module_bbox"]["max"] = [v + [0.10, 0.0, 0.0][i] for i, v in enumerate(shifted["module_bbox"]["max"])]
    shifted_plan = build_sim_world.plan_scene(shifted)

    module = shifted_plan["module"]
    assert module["alignment"] == "bbox_centre"
    assert module["pose_source"] == "bbox_centre_at_measured_cad_centre"
    assert module["frame_deviation_m"] > build_sim_world.FRAME_THRESHOLD_M
    # bbox 对齐后，bbox 中心正好落在实测 CAD 中心上（只差数值噪声）。
    assert module["deviation_m"] < 1e-9
    assert module["bbox_world"]["centre"] == pytest.approx(MEASUREMENTS["workpiece"]["cad_centre"], abs=1e-9)
