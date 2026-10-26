#!/usr/bin/env python3
"""Create the renderer-safe URDF used exclusively for Gazebo simulation.

The vendor CS625 description contains DAE visual meshes.  On the supported
WSL2 + Gazebo Harmonic + Ogre2 software-renderer path, loading those meshes in
an RGB-D render scene can block sensor initialization.  This utility runs the
same Xacro used by robot_state_publisher, then removes only mesh-based visual
elements.  It leaves links, joints, inertials, collisions, ros2_control and
Gazebo sensor elements intact.

``--keep-visuals`` keeps the mesh visuals instead, which is what makes the arm
visible in the Gazebo GUI.  It is opt-in because the mesh-loading path is the
one that can stall the sensor render scene; sensor work must stay on the
default mesh-free model until that path has been re-validated.

``--preprocess`` 是可选的一层包装：先让一个"形状与 xacro 相同"的 python 脚本
（例如仓库的 ``scripts/sim_robot_description.py``，它负责往 ros2_control 的
``<hardware>`` 块里注入刚度参数、并把厂商工具网格换成对称的 gripper_0920）跑出
URDF，再由本脚本解析并去掉网格视觉。必须"先注入、后去网格"：``--keep-visuals``
要保留的正是注入换上的那把对称夹爪网格，顺序反了它就会被当成厂商网格删掉。
不传 ``--preprocess`` 时，行为与直接调用 xacro 完全一致。
"""

from __future__ import annotations

import argparse
import pathlib
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as element_tree


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=pathlib.Path)
    parser.add_argument("--xacro", required=True)
    parser.add_argument(
        "--keep-visuals",
        action="store_true",
        help=(
            "Keep mesh-based visual elements so the arm renders in the Gazebo "
            "GUI. Default is to strip them for renderer safety."
        ),
    )
    parser.add_argument(
        "--preprocess",
        default=None,
        help=(
            "可选：与 xacro 同调用形状的包装脚本（python 文件）。给了以后实际调用变成 "
            "python3 <preprocess> <xacro> <description_file> [key:=value ...]，"
            "让它在 xacro 输出的 URDF 上做注入/替换，然后再由本脚本去掉网格视觉。"
            "不传时行为与直接调用 xacro 完全一致。"
        ),
    )
    parser.add_argument("description_file", type=pathlib.Path)
    parser.add_argument("xacro_arguments", nargs=argparse.REMAINDER)
    return parser.parse_args()


def normalize_xacro_arguments(arguments: list[str]) -> list[str]:
    """Join launch substitutions split around ``name:=value`` arguments."""
    normalized: list[str] = []
    index = 0
    while index < len(arguments):
        argument = arguments[index]
        if argument.endswith(":=") and index + 1 < len(arguments):
            normalized.append(argument + arguments[index + 1])
            index += 2
        else:
            normalized.append(argument)
            index += 1
    return normalized


def main() -> int:
    arguments = parse_arguments()
    xacro_command = [
        arguments.xacro,
        str(arguments.description_file),
        *normalize_xacro_arguments(arguments.xacro_arguments),
    ]
    if arguments.preprocess:
        # 包装脚本只做纯文本注入/替换，不改厂商文件；用当前解释器（与启动本脚本的
        # python3 是同一个）而不是再查一次 PATH，保证两个脚本跑在同一个 Python 上。
        xacro_command = [sys.executable, arguments.preprocess, *xacro_command]
    result = subprocess.run(
        xacro_command,
        check=True,
        capture_output=True,
        text=True,
    )
    root = element_tree.fromstring(result.stdout)
    mesh_visuals = [
        visual
        for link in root.findall("link")
        for visual in link.findall("visual")
        if visual.find("./geometry/mesh") is not None
    ]
    if not arguments.keep_visuals:
        for link in root.findall("link"):
            for visual in list(link.findall("visual")):
                if visual.find("./geometry/mesh") is not None:
                    link.remove(visual)

    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode="wb",
        dir=arguments.output.parent,
        prefix=f".{arguments.output.name}.",
        delete=False,
    ) as temporary_file:
        temporary_path = pathlib.Path(temporary_file.name)
        element_tree.ElementTree(root).write(
            temporary_file,
            encoding="utf-8",
            xml_declaration=True,
        )
    temporary_path.replace(arguments.output)
    if arguments.keep_visuals:
        print(
            f"Prepared Gazebo model at {arguments.output} "
            f"(kept {len(mesh_visuals)} mesh visual elements for GUI rendering)."
        )
    else:
        print(
            f"Prepared Gazebo model at {arguments.output} "
            f"(removed {len(mesh_visuals)} mesh visual elements)."
        )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except subprocess.CalledProcessError as error:
        sys.stderr.write(error.stderr)
        raise
