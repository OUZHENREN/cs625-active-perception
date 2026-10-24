"""Minimal trimesh shim: only what tcp_bridge nodes use.

Provides ``load(path, force='mesh')`` returning an object with ``vertices``,
``faces`` and ``is_empty`` so senior nodes can run without the real package.
Supports binary and ASCII STL.
"""

from __future__ import annotations

import struct
from pathlib import Path

import numpy as np


class _Mesh:
    def __init__(self, vertices: np.ndarray, faces: np.ndarray) -> None:
        self.vertices = vertices
        self.faces = faces

    @property
    def is_empty(self) -> bool:
        return len(self.vertices) == 0 or len(self.faces) == 0


def _load_stl(raw: bytes) -> _Mesh:
    head = raw[:2000]
    if raw[:5].lower() == b"solid" and b"facet" in head:
        verts = []
        for line in raw.decode("ascii", "replace").splitlines():
            parts = line.strip().split()
            if parts and parts[0] == "vertex":
                verts.append([float(parts[1]), float(parts[2]), float(parts[3])])
        vertices = np.array(verts, dtype=np.float64)
        faces = np.arange(len(vertices)).reshape(-1, 3)
        return _Mesh(vertices, faces)
    count = struct.unpack("<I", raw[80:84])[0]
    dtype = np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")])
    data = np.frombuffer(raw[84:], dtype=dtype, count=count)
    vertices = data["v"].reshape(-1, 3).astype(np.float64)
    faces = np.arange(len(vertices)).reshape(-1, 3)
    return _Mesh(vertices, faces)


def load(path, force: str | None = None, **kwargs) -> _Mesh:  # noqa: ARG001
    raw = Path(str(path)).read_bytes()
    lowered = str(path).lower()
    if lowered.endswith(".stl"):
        return _load_stl(raw)
    raise NotImplementedError(f"trimesh shim only supports STL, got {path}")
