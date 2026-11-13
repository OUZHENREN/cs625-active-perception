"""离线回归测试：真机场景碰撞体（`scripts/apply_real_cell_scene.py`）。

2026-11-13 的现场症状是"脚本打印 published 1 collision objects，但 MoveIt 场景里
0 个对象"。根因是它往 `/planning_scene` 话题裸发、从不确认对方收到。修复后改走
`/apply_planning_scene` 服务并回查场景。

这些测试**不需要 move_group**：只测纯构建部分（id / 帧 / 操作类型 / 尺寸），
这正是"看起来对、实际放错"最容易发生的地方。服务调用与回查需要真实 move_group，
属于真机/仿真的验收范围，不在这里伪装。
"""

from __future__ import annotations

import importlib.util
import pathlib
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "apply_real_cell_scene", ROOT / "scripts" / "apply_real_cell_scene.py"
)
applier = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(applier)


BOX_SPEC = {
    "id": "real_cell_workpiece_table",
    "kind": "box",
    "frame": "world",
    "pose": {"xyz": [0.7, 0.3, -0.1], "quat_xyzw": [0.0, 0.0, 0.0, 1.0]},
    "size": [1.0, 0.8, 0.05],
}


def test_add_box_spec_produces_one_world_object():
    scene = applier.build_scene([BOX_SPEC])
    assert scene.is_diff is True
    assert len(scene.world.collision_objects) == 1
    obj = scene.world.collision_objects[0]
    assert obj.id == BOX_SPEC["id"]
    assert obj.header.frame_id == "world"
    assert obj.operation == obj.ADD
    assert obj.primitives[0].dimensions == pytest.approx(BOX_SPEC["size"])
    assert obj.primitive_poses[0].position.x == pytest.approx(0.7)


def test_remove_spec_uses_remove_operation_and_no_geometry():
    scene = applier.build_scene([BOX_SPEC], remove=True)
    obj = scene.world.collision_objects[0]
    assert obj.operation == obj.REMOVE
    assert list(obj.primitives) == []
    assert list(obj.meshes) == []


def test_real_scene_config_yields_the_fixture_in_the_world_frame():
    """默认场景（只发夹具）必须给出 real_cell_slot_fixture，帧为 world。"""
    scene = applier.load_scene(ROOT / "config" / "cs625_real_cell_scene.yaml")
    specs = applier.body_specs(scene)
    ids = [spec["id"] for spec in specs]
    assert "real_cell_slot_fixture" in ids
    fixture = [spec for spec in specs if spec["id"] == "real_cell_slot_fixture"][0]
    assert fixture["kind"] == "mesh"
    assert fixture["frame"] == "world"
    assert pathlib.Path(fixture["mesh"]).is_file()


def test_default_cli_selection_matches_the_documented_behaviour():
    """默认不发表面盒与工件；--include-tables/--include-module 才加。"""
    scene = applier.load_scene(ROOT / "config" / "cs625_real_cell_scene.yaml")
    specs = applier.body_specs(scene)
    default = [s for s in specs if "shielding_module" not in s["id"] and s["kind"] != "box"]
    assert {s["id"] for s in default} == {"real_cell_slot_fixture"}
    assert any(s["kind"] == "box" for s in specs), "配置里应有桌子盒，供 --include-tables 使用"
