#!/usr/bin/env python3
"""Print axis-aligned extents for binary STL files without extra packages."""

from __future__ import annotations

import pathlib
import struct
import sys


def vertices(path: pathlib.Path):
    payload = path.read_bytes()
    expected = struct.unpack_from("<I", payload, 80)[0]
    records = list(struct.iter_unpack("<12fH", payload[84:]))
    if len(records) != expected:
        raise ValueError(f"{path}: expected {expected} triangles, got {len(records)}")
    for record in records:
        yield record[3:6]
        yield record[6:9]
        yield record[9:12]


def main() -> None:
    for argument in sys.argv[1:]:
        path = pathlib.Path(argument)
        points = list(vertices(path))
        extents = [max(point[axis] for point in points) - min(point[axis] for point in points) for axis in range(3)]
        print(f"{path.name}: " + " ".join(f"{extent:.6f}" for extent in extents))


if __name__ == "__main__":
    main()
