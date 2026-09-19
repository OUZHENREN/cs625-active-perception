#!/usr/bin/env python3
"""Report geometry and mass properties for a binary STL.

Companion to ``audit_binary_stl_extents.py``.  That script only prints the
axis-aligned extents; this one also reports triangle counts, signed volume,
watertightness, centroid, and the inertia tensor about the centroid, which is
what a URDF/SDF ``<inertial>`` block needs.

The STL format carries no units and no density, so both are supplied on the
command line.  The defaults are millimetres and aluminium 6061, the two values
the current CS625 task uses.

    test/inspect_stl_mass_properties.py model.stl
    test/inspect_stl_mass_properties.py --unit m --density 7850 part.stl

Volume and the second moment are integrated exactly over the tetrahedra formed
by each triangle and the origin.  For a closed, consistently wound mesh the
signed volumes cancel outside the solid and the sum is the volume; for an open
mesh the result is meaningless, so watertightness is checked and reported
alongside.  The inertia tensor uses the closed-form tetrahedron second-moment
integral (Mirtich 1996) followed by a parallel-axis shift to the centroid.
"""

from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

import numpy as np

UNIT_SCALE = {"mm": 1.0e-3, "cm": 1.0e-2, "m": 1.0}
DENSITY = {"aluminium_6061": 2700.0, "steel": 7850.0, "glass_borosilicate": 2230.0}

# Integrals over the unit simplex {s >= 0, sum(s) <= 1}.
SIMPLEX_INT_1 = 1.0 / 6.0
SIMPLEX_INT_S = np.full(3, 1.0 / 24.0)


def _simplex_int_ss() -> np.ndarray:
    matrix = np.full((3, 3), 1.0 / 120.0)
    np.fill_diagonal(matrix, 1.0 / 60.0)
    return matrix


SIMPLEX_INT_SS = _simplex_int_ss()


def read_binary_stl(path: Path) -> np.ndarray:
    """Return an (n, 3, 3) float array of triangle vertices."""
    payload = path.read_bytes()
    if len(payload) < 84:
        raise ValueError(f"{path}: too short to be a binary STL")
    expected = struct.unpack_from("<I", payload, 80)[0]
    body = payload[84:]
    if len(body) != expected * 50:
        raise ValueError(
            f"{path}: header claims {expected} triangles ({expected * 50} bytes) "
            f"but the body is {len(body)} bytes; this is probably an ASCII STL"
        )
    records = np.frombuffer(body, dtype=np.dtype([("d", "<12f4"), ("attr", "<u2")]))
    return records["d"][:, 3:12].reshape(-1, 3, 3).astype(np.float64)


def open_edge_counts(triangles: np.ndarray) -> tuple[int, int]:
    """Return (edges used once, edges used more than twice)."""
    corners = np.round(triangles, 9).reshape(-1, 3)
    _, inverse = np.unique(corners, axis=0, return_inverse=True)
    faces = inverse.reshape(-1, 3)
    edges = np.sort(np.concatenate(
        [faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]]), axis=1)
    _, counts = np.unique(edges, axis=0, return_counts=True)
    return int((counts == 1).sum()), int((counts > 2).sum())


def connected_components(triangles: np.ndarray) -> np.ndarray:
    """Label triangles by shared-vertex connectivity (union-find).

    STL assemblies arrive as many disjoint shells.  Orientation must be fixed
    per shell, not over the whole mesh, because each shell's global winding
    sense is independent of the others'.
    """
    count = len(triangles)
    corners = np.round(triangles, 9).reshape(-1, 3)
    _, inverse = np.unique(corners, axis=0, return_inverse=True)
    faces = inverse.reshape(-1, 3)

    parent = np.arange(count)

    def find(index: int) -> int:
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    shared: dict[tuple[int, int], int] = {}
    for index, (a, b, c) in enumerate(faces):
        for u, v in ((a, b), (b, c), (c, a)):
            key = (u, v) if u < v else (v, u)
            first = shared.setdefault(key, index)
            root_first, root_here = find(first), find(index)
            if root_first != root_here:
                parent[root_first] = root_here

    roots = np.array([find(index) for index in range(count)])
    _, labels = np.unique(roots, return_inverse=True)
    return labels


def normalize_shell_orientation(triangles: np.ndarray) -> tuple[np.ndarray, int, int, float, float]:
    """Make each closed shell wind outward.

    Returns (triangles, faces flipped, shell count, signed volume, absolute
    volume).  A closed shell whose signed volume is negative is wound inward, so
    it is reversed.  ``signed`` and ``absolute`` agreeing means every shell is a
    separate outward solid; a mismatch means the shells nest, and the caller
    should not trust a single number.
    """
    labels = connected_components(triangles)
    repaired = triangles.copy()
    flipped = 0
    signed = 0.0
    absolute = 0.0
    shells = int(labels.max()) + 1 if len(labels) else 0
    for shell in range(shells):
        mask = labels == shell
        block = repaired[mask]
        a, b, c = block[:, 0], block[:, 1], block[:, 2]
        det = np.einsum("ij,ij->i", a, np.cross(b, c))
        volume = det.sum() / 6.0
        signed += volume
        absolute += abs(volume)
        if volume < 0.0:
            repaired[mask] = block[:, ::-1]
            flipped += int(mask.sum())
    return repaired, flipped, shells, signed, absolute


def integrate(triangles: np.ndarray) -> tuple[float, np.ndarray, np.ndarray, np.ndarray]:
    """Return (signed volume, first moment, second moment, per-face signed dets).

    Every integral is a *signed* sum over the tetrahedra formed by the origin and
    each triangle.  Taking |det| instead would be wrong whenever the origin lies
    outside the solid: those tetrahedra overlap, and only the signed sum cancels
    the material counted twice.
    """
    a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
    det = np.einsum("ij,ij->i", a, np.cross(b, c))

    volume = det.sum() * SIMPLEX_INT_1
    first = np.zeros(3)
    second = np.zeros((3, 3))
    for face, d in zip(triangles, det):
        matrix = face.T                      # columns are the three edge vectors
        first += d * (matrix @ SIMPLEX_INT_S)
        second += d * (matrix @ SIMPLEX_INT_SS @ matrix.T)
    return volume, first, second, det


def mass_properties(triangles: np.ndarray, density: float) -> dict:
    repaired, flipped, shells, signed, absolute = normalize_shell_orientation(triangles)
    volume, first, second, _ = integrate(repaired)
    if volume <= 0.0:
        raise ValueError("degenerate mesh: non-positive enclosed volume")

    mass = density * volume
    centroid = first / volume
    # ``second`` is the purely geometric integral of r*r^T dV, so it still needs
    # the density before it can be compared with the parallel-axis term.
    inertia_about_origin = density * (np.trace(second) * np.eye(3) - second)
    shift = mass * ((centroid @ centroid) * np.eye(3) - np.outer(centroid, centroid))
    return {
        "volume_m3": volume,
        "mass_kg": mass,
        "centroid_m": centroid,
        "inertia_about_centroid": inertia_about_origin - shift,
        "faces_flipped": flipped,
        "shells": shells,
        "signed_volume_m3": signed,
        "absolute_volume_m3": absolute,
    }


def report(path: Path, unit: str, density: float) -> None:
    triangles = read_binary_stl(path) * UNIT_SCALE[unit]
    try:
        results = mass_properties(triangles, density)
    except ValueError as error:
        print(f"== {path.name} ==\n  ERROR: {error}\n")
        return

    low = triangles.reshape(-1, 3).min(axis=0)
    high = triangles.reshape(-1, 3).max(axis=0)
    once, over = open_edge_counts(triangles)
    extents = high - low
    inertia = results["inertia_about_centroid"]
    centroid = results["centroid_m"]

    print(f"== {path.name} ==")
    print(f"  unit in file      : {unit}")
    print(f"  density (kg/m^3)  : {density:.1f}")
    print(f"  triangles         : {len(triangles)}")
    print(f"  bounds min (m)    : {low[0]:.6f} {low[1]:.6f} {low[2]:.6f}")
    print(f"  bounds max (m)    : {high[0]:.6f} {high[1]:.6f} {high[2]:.6f}")
    print(f"  extents (mm)      : {1000*extents[0]:.2f} {1000*extents[1]:.2f} {1000*extents[2]:.2f}")
    print(f"  volume (cm^3)     : {results['volume_m3']*1e6:.3f}")
    print(f"  watertight        : "
          f"{'YES' if once == 0 and over == 0 else 'NO'}"
          f"  (open edges {once}, non-manifold edges {over})")
    print(f"  winding repair    : {results['shells']} shell(s), "
          f"{results['faces_flipped']} faces reversed")
    if abs(results["signed_volume_m3"] - results["absolute_volume_m3"]) > 1e-3 * results["absolute_volume_m3"]:
        print("  WARNING           : signed and absolute shell volumes disagree; "
              "shells nest or overlap, so the single volume/inertia below is suspect")
    print(f"  mass (kg)         : {results['mass_kg']:.6f}")
    print(f"  centroid (m)      : {centroid[0]:.6f} {centroid[1]:.6f} {centroid[2]:.6f}")
    print("  inertia about centroid (kg*m^2):")
    for row in inertia:
        print(f"      {row[0]: .8e} {row[1]: .8e} {row[2]: .8e}")
    print()


def main() -> int:
    parser = argparse.ArgumentParser(description="Report STL mass properties.")
    parser.add_argument("stl", nargs="+", type=Path)
    parser.add_argument("--unit", choices=sorted(UNIT_SCALE), default="mm")
    parser.add_argument(
        "--density", default="aluminium_6061",
        help="one of " + ", ".join(sorted(DENSITY)) + ", or a number in kg/m^3",
    )
    arguments = parser.parse_args()

    density = DENSITY.get(arguments.density)
    if density is None:
        density = float(arguments.density)

    for path in arguments.stl:
        report(path, arguments.unit, density)
    return 0


if __name__ == "__main__":
    sys.exit(main())
