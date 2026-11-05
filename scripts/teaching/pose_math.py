#!/usr/bin/env python3
"""Pose math shared by the teaching chain (mirrors the senior's tcp_server).

The senior's ``tcp_server`` converts the object poses that RVS returns into the
tool poses the robot needs.  The same numbers are reproduced here so the teaching
program does not have to guess or copy constants by hand:

    T_b_tcp = diag(-1, 1, -1) + (-0.004, -0.215, -0.411)   box frame  -> grasp TCP
    T_s_tcp = diag(-1, 1, -1) + (-0.082, -0.193, -0.4125)  slot frame -> insert TCP

Derived poses
-------------
    grasp     = box_pose      * T_b_tcp
    pre_grasp = grasp         * translate(+120 mm in TCP x)      (senior: T_offset)
    insert    = slot_pose     * T_s_tcp
    rotated   = insert        * Ry(-10 deg)                      (the tilted entry)
    pre_insert= rotated       * translate(-350 mm in TCP z)

Everything is expressed in the base frame, metres, quaternion (x, y, z, w).
"""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation as R

T_B_TCP = np.array([
    [-1.0, 0.0, 0.0, -0.004],
    [0.0, 1.0, 0.0, -0.215],
    [0.0, 0.0, -1.0, -0.411],
    [0.0, 0.0, 0.0, 1.0],
])

T_S_TCP = np.array([
    [-1.0, 0.0, 0.0, -0.082],
    [0.0, 1.0, 0.0, -0.193],
    [0.0, 0.0, -1.0, -0.4125],
    [0.0, 0.0, 0.0, 1.0],
])

PRE_GRASP_OFFSET_M = 0.120      # senior: box_pre_grasp = grasp * (x + 120 mm)
PRE_INSERT_BACKOFF_M = 0.350    # senior: pre_insert = rotated * (z - 350 mm)
TILT_DEG = -10.0                # senior: insert is entered with a 10 deg tilt


def to_matrix(xyz, quat) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, :3] = R.from_quat(quat).as_matrix()
    matrix[:3, 3] = xyz
    return matrix


def to_pose(matrix: np.ndarray) -> tuple[list[float], list[float]]:
    xyz = [float(v) for v in matrix[:3, 3]]
    quat = [float(v) for v in R.from_matrix(matrix[:3, :3]).as_quat()]
    return xyz, quat


def translation(matrix: np.ndarray, dx: float, dy: float, dz: float) -> np.ndarray:
    """Offset in the frame of ``matrix`` (i.e. matrix * T(dx, dy, dz))."""
    offset = np.eye(4)
    offset[:3, 3] = [dx, dy, dz]
    return matrix @ offset


def grasp_from_box(box_xyz, box_quat):
    grasp = to_matrix(box_xyz, box_quat) @ T_B_TCP
    pre_grasp = translation(grasp, PRE_GRASP_OFFSET_M, 0.0, 0.0)
    return to_pose(grasp), to_pose(pre_grasp)


def insert_from_slot(slot_xyz, slot_quat, tilt_deg=None, extra_height_m=0.0,
                     vertical_approach=False):
    """Insert chain from the slot pose.

    tilt_deg defaults to the senior's -10 deg entry tilt.  The owner chose to derive
    everything from TODAY's slot measurement (2026-11-04), so teaching_sequence passes
    tilt_deg=0.0: the entry attitude then comes purely from the R1 orientation instead
    of carrying an extra -10 deg that was tuned for the senior's own setup.
    """
    insert = to_matrix(slot_xyz, slot_quat) @ T_S_TCP
    tilt = np.eye(4)
    angle = TILT_DEG if tilt_deg is None else tilt_deg
    tilt[:3, :3] = R.from_euler("y", angle, degrees=True).as_matrix()
    rotated = insert @ tilt
    if vertical_approach:
        # Kept as an OPTION only.  Owner clarification 2026-11-04: the insertion runs along
        # the TOOL z axis, so x MUST change in world coordinates -- that is the slot's
        # direction, not an error.  (My earlier reading of "only z changes" as world z was
        # wrong: it also discarded the operator's lateral translation of the pre-insert.)
        # The default chain is the senior's: back off along the tilted tool z.
        pre_insert = rotated.copy()
        pre_insert[0, 3] = insert[0, 3]
        pre_insert[1, 3] = insert[1, 3]
        pre_insert[2, 3] = insert[2, 3] + PRE_INSERT_BACKOFF_M
    else:
        pre_insert = translation(rotated, 0.0, 0.0, -PRE_INSERT_BACKOFF_M)
    # Owner 2026-11-04: the pre-insert standing height is raised further (the 350 mm
    # back-off alone put the tool too low to inspect the entry).  The extra height is a
    # world +z offset applied after the back-off, so the insert end point is unchanged.
    if extra_height_m:
        pre_insert = pre_insert.copy()
        pre_insert[2, 3] += float(extra_height_m)
    return to_pose(insert), to_pose(rotated), to_pose(pre_insert)


if __name__ == "__main__":
    # sanity check against the values the senior's tcp_server printed on 2026-10-27
    box = ([0.5957, 0.7492, 0.3554], [0.0069, -0.0022, 0.5412, 0.8408])
    grasp, pre_grasp = grasp_from_box(*box)
    print("grasp    ", [round(v, 4) for v in grasp[0]])
    print("pre_grasp", [round(v, 4) for v in pre_grasp[0]])
