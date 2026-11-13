"""离线回归测试：仿真模型注入器 `scripts/sim_robot_description.py`。

为什么需要（2026-11-13）：工具碰撞代理找了 `my_end_effector_link`，而工具碰撞体
实际挂在 `tool0` 上（前者是自闭合、无几何的 TCP 框架）。于是代理**从未注入**——
仿真里一直是张开态 240 mm 碰撞（文档记录的老问题：预抓取把工件推走 0.4 m）——
而且每次启动都往 stderr 告警，被 launch 的 `Command(on_stderr='fail')` 当成失败，
直接中止仿真启动。这个脚本此前没有任何测试，正是它能静默失效这么久的原因。

这些测试只做纯字符串变换，不需要 ROS/Gazebo。
"""

from __future__ import annotations

import importlib.util
import pathlib

ROOT = pathlib.Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "sim_robot_description", ROOT / "scripts" / "sim_robot_description.py"
)
injector = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(injector)

PROXY = "0.060 0.140 0.214"


def tool_xml(link_name: str, *, self_closing: bool = False) -> str:
    """Minimal URDF with one tool link; the collision mesh lives on it."""
    if self_closing:
        return (
            '<robot name="cs">\n'
            f'  <link name="{link_name}"/>\n'
            '</robot>\n'
        )
    return (
        '<robot name="cs">\n'
        f'  <link name="{link_name}">\n'
        '    <visual><geometry><mesh filename="tool.stl"/></geometry></visual>\n'
        '    <collision><geometry><mesh filename="tool.stl"/></geometry></collision>\n'
        '  </link>\n'
        '</robot>\n'
    )


def test_proxy_replaces_the_collision_on_the_link_that_has_one():
    xml = tool_xml("tool0")
    out = injector.inject_tool_collision_proxy(xml, "0.060 0.140 0.214", "0 0 -0.253")
    assert PROXY in out
    # 只替换碰撞体：collision 只剩一个且不再引用原 mesh，visual 必须原样保留
    assert out.count("<collision>") == 1
    assert 'filename="tool.stl"' in out.split("<collision>")[0], "visual 应保留原网格"
    assert 'filename="tool.stl"' not in out.split("<collision>")[1].split("</collision>")[0]


def test_proxy_falls_back_to_tool0_when_tcp_frame_has_no_geometry():
    """真实描述的样子：TCP 框架自闭合，几何在 tool0 上。"""
    xml = (
        '<robot name="cs">\n'
        '  <link name="my_end_effector_link"/>\n'
        + tool_xml("tool0").split("\n", 1)[1]  # 复用带 collision 的 tool0 定义
    )
    out = injector.inject_tool_collision_proxy(xml, "0.060 0.140 0.214", "0 0 -0.253")
    assert PROXY in out, "TCP 框架没有几何时必须退回 tool0，而不是静默跳过"
    assert out.count("<collision>") == 1


def test_proxy_is_a_no_op_when_no_candidate_carries_a_collision():
    xml = '<robot name="cs">\n  <link name="my_end_effector_link"/>\n  <link name="tool0"/>\n</robot>\n'
    out = injector.inject_tool_collision_proxy(xml, "0.060 0.140 0.214", "0 0 -0.253")
    assert out == xml
    assert PROXY not in out


def test_detachable_joint_injection_is_idempotent():
    xml = tool_xml("tool0").replace(
        '<link name="tool0">', '<link name="wrist_3_link">'
    ).replace("</link>", "</link>", 1)
    once = injector.inject_detachable_joint(xml)
    twice = injector.inject_detachable_joint(once)
    assert once.count("gz-sim-detachable-joint-system") == 1
    assert twice.count("gz-sim-detachable-joint-system") == 1
    assert "cs625/carry/attach" in once
