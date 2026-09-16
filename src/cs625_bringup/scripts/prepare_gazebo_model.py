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
    result = subprocess.run(
        [
            arguments.xacro,
            str(arguments.description_file),
            *normalize_xacro_arguments(arguments.xacro_arguments),
        ],
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
