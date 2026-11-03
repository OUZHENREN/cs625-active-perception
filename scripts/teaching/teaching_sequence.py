#!/usr/bin/env python3
"""One-pass teaching sequence: the order that is expected to pass in a single run.

Design rule from the owner (2026-10-27):
    the recorded program must succeed in one go, so every step that failed during
    the trial must NOT appear here.  The failures were:
      * rising with the jaws still open
      * skipping the existing transit waypoint and heading straight for staging
      * rotating while descending towards the slot
    The order below is the corrected one; each motion is separated on purpose.

Order (matching the senior's flow, with the observation prefix our repo needs)
-----------------------------------------------------------------------------
    0. environment model      offline point cloud -> tables/fixture collision (no motion)
    1. slot coarse (T0)       move to slot_coarse, trigger T0, receive S0
    2. slot fine  (T1)        move to S0 (the pose RVS asked for), trigger T1 -> R1
    3. workpiece coarse (T2)  at workpiece_coarse, trigger T2, receive S2
    4. workpiece fine  (T3)   move to S2 (the pose RVS asked for), trigger T3 -> S3
    5. pre-grasp              jaws initialised, closed
    6. enter grasp            jaws closed, low speed, verified
    7. open to grip width     expanding grasp onto the handles
    8. lift 0.15 m            the only proof the part is held
    9. raise to safe height   straight up, attitude unchanged
   10. transit                existing proven waypoint, cannot be skipped
   11. staging                slot outer preparation point
   12. rotate to -10 deg      rotation only, at safe height
   13. descend                vertical only, attitude unchanged
   14. insert                 along the tilted axis, 0.005 m/s
   15. straighten             rotate level, 0.1 m/s
   16. release                REQUIRES explicit owner approval
   17. withdraw               pure translation out of the slot
   18. baseline               parking pose

Design rule from the owner (2026-10-30):
    the two COARSE observation poses (slot_coarse, workpiece_coarse) are fixed
    constants.  Everything RVS answers is NOT fixed: S0/S2 (the fine view poses)
    and S3 (workpiece pose -> grasp/pre-grasp via pose_math) must be taken from
    RVS at run time.  Verified 2026-10-30: standing on the old fixed slot_fine
    constant made T1 answer NG; moving to the S0 RVS returned made T1 succeed
    in 3.6 s.

Speeds (owner-confirmed 2026-10-27): only ``insert`` and the unload extraction stay
at 0.005 m/s; ``straighten`` runs at 0.1 m/s; everything else uses the proven
approach speeds below.
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
PLAN = REPO / "scripts" / "teaching" / "plan_joint_locked.py"
EXECUTE = REPO / "scripts" / "execute_assembly_chunks.py"
RVS_CLIENT = REPO / "scripts" / "teaching" / "rvs_pose_client.py"
GRIPPER = REPO / "scripts" / "teaching" / "gripper.py"
ENV_MODEL = REPO / "scripts" / "model_environment.py"
sys.path.insert(0, str(Path(__file__).resolve().parent))
from pose_math import grasp_from_box, insert_from_slot  # noqa: E402

# --- speeds: (velocity, time_scale, tail_fraction, tail_scale) ------------------
SPEED = {
    "pre_grasp":  (0.02, 1.0, 0.0, 1.0),
    "grasp":      (0.01, 1.5, 0.2, 2.0),
    "lift":       (0.03, 1.5, 0.0, 1.0),
    "transit":    (0.08, 1.0, 0.0, 1.0),
    "staging":    (0.05, 1.5, 0.0, 1.0),
    "pre_insert": (0.03, 1.5, 0.4, 1.67),
    "insert":     (0.005, 3.0, 0.0, 1.0),   # tight clearance: do not change
    "straighten": (0.1, 1.0, 0.0, 1.0),     # owner: 0.1 is allowed here
    "withdraw":   (0.03, 1.5, 0.0, 1.0),
    "baseline":   (0.08, 1.0, 0.0, 1.0),
    "observe":    (0.05, 1.5, 0.0, 1.0),
}

# --- poses: proven values from config/tasks/common.yaml ------------------------
# Fixed constants: ONLY the two coarse observation poses (owner, 2026-10-30).
# The fine view poses (S0/S2) and the grasp pose come from RVS at run time;
# keeping fixed fine-pose constants here is what made T1 answer NG on 2026-10-30.
def _observation_pose(name: str, fallback):
    """Coarse-observation baseline, from config when present (owner workflow).

    These two poses are constants, but their VALUES are re-registered whenever the
    workpiece is placed where the old pose cannot see it (2026-11-03):
        python3 scripts/teaching/set_observation_pose.py --name <name> --from-current
    Config wins; the literal below is the fallback so the file still runs standalone.
    """
    try:
        import yaml
        config = yaml.safe_load((REPO / "config" / "tasks" / "common.yaml").read_text())
        entry = (config.get("teaching_observation_poses") or {}).get(name) or {}
        if entry.get("position") and entry.get("quat"):
            return ([float(v) for v in entry["position"]],
                    [float(v) for v in entry["quat"]])
    except Exception:
        pass
    return fallback


POSE = {
    "workpiece_coarse": _observation_pose(
        "workpiece_coarse",
        ([0.5886, 0.3482, 0.7075], [0.2954, -0.7023, -0.2539, -0.5958])),
    "slot_coarse": _observation_pose(
        "slot_coarse",
        ([0.7802, -0.1458, 0.7986], [0.1834, -0.8502, -0.1133, -0.4803])),
    "transit": ([0.95, 0.45, 0.55], [-0.5412, 0.8408, 0.0069, 0.0022]),
    "staging": ([1.0748, -0.0122, 0.5252], [-0.5412, 0.8408, 0.0069, 0.0022]),
    "pre_insert": ([1.0996, -0.0125, 0.3773], [-0.004, 0.997, -0.001, 0.083]),
    "insert": ([1.1574, -0.0132, 0.0321], [-0.004, 0.997, -0.001, 0.083]),
    "straight": ([1.1574, -0.0132, 0.0321], [-0.003, 1.000, -0.002, -0.004]),
    "withdraw": ([1.0996, -0.0125, 0.3773], [-0.003, 1.000, -0.002, -0.004]),
    "baseline": ([1.044, -0.011, 0.726], [-0.059, 0.938, 0.060, 0.336]),
}
SAFE_HEIGHT_Z = 0.530          # raise here before any rotation near the slot


def run(args: list[str], timeout: float = 240.0) -> tuple[int, str]:
    done = subprocess.run(args, capture_output=True, timeout=timeout)
    text = ((done.stdout or b"") + (done.stderr or b"")).decode("utf-8", "replace")
    return done.returncode, text.strip()


# --- descent safety (2026-10-30 accident; owner 2026-10-31) --------------------
# Steps that move near the tables are never sent as one long trajectory: the
# plan must first pass the tool-envelope check against the two measured tables
# (the seat is part of the SolidWorks tool body), and then each segment is sent
# alone and verified before the next one starts.
# Carried steps (jaws open + part attached) add the carried-box envelope because
# the opened support arms (146-203 mm forward, unmodelled) and the workpiece
# extend the real tool well beyond the tool body.  The LIFT stays tool-only:
# it is a vertical motion out of the part's own footprint and starts with the
# part resting on the table BY DESIGN, so a geometric clearance gate would
# always refuse it; the lateral transport (transit/staging) is where the part
# sweeps across the cell and must keep the full margin.
SEGMENTED = {"pre_grasp": 4, "grasp": 6, "lift": 2, "transit": 2, "staging": 2}
CARRIED = {"transit", "staging"}
ENVELOPE_CHECK = REPO / "scripts" / "check_tool_envelope.py"


def execute_plan(path: Path, time_scale: float, tail_fraction: float,
                 tail_scale: float, goal_tol: float, segments: int = 1) -> bool:
    """Execute one trajectory JSON; ``segments > 1`` splits it into verified
    chunks with a shared seam point so each chunk starts from the verified end
    of the previous one."""
    if segments <= 1:
        code, text = run([sys.executable, "-u", str(EXECUTE), str(path),
                          "--time-scale", str(time_scale),
                          "--tail-fraction", str(tail_fraction),
                          "--tail-scale", str(tail_scale),
                          "--goal-tolerance", str(goal_tol),
                          "--verify-final", str(goal_tol),
                          "--fault-poll", "0.2"], timeout=600.0)
        for line in text.splitlines()[-3:]:
            print("   ", line)
        return "ALL_ASSEMBLY_CHUNKS_OK" in text
    data = json.loads(path.read_text())
    points = data["points"]
    step = max(2, math.ceil((len(points) - 1) / segments))
    ok = True
    for i in range(segments):
        start = i * step
        end = min((i + 1) * step + 1, len(points))  # +1 shared seam point
        # Re-time from zero: each segment is its own trajectory goal, so the
        # first timestamp must be 0.  Keeping the original absolute times made
        # segment 3 start at t=16 s after scaling and the executor reported
        # NO_MOTION while the controller was still waiting (2026-10-31).
        t0 = points[start]["t"]
        seg_points = [{"q": p["q"], "t": p["t"] - t0} for p in points[start:end]]
        seg = Path(f"/tmp/teach_{path.stem}_seg{i:02d}.json")
        seg.write_text(json.dumps({"joint_names": data["joint_names"],
                                   "points": seg_points}))
        print(f"   segment {i + 1}/{segments}: points {start + 1}..{end} of {len(points)}")
        # Exit code 10 means the executor auto-recovered the reverse-control
        # channel (resend_external_control) and the caller is expected to retry
        # this segment (2026-11-03: the channel dropped three times in one day).
        attempts = 0
        while True:
            attempts += 1
            code, text = run([sys.executable, "-u", str(EXECUTE), str(seg),
                              "--time-scale", str(time_scale),
                              "--tail-fraction", "0.0", "--tail-scale", "1.0",
                              "--goal-tolerance", str(goal_tol),
                              "--verify-final", str(goal_tol),
                              "--fault-poll", "0.2"], timeout=600.0)
            for line in text.splitlines()[-2:]:
                print("   ", line)
            ok = "ALL_ASSEMBLY_CHUNKS_OK" in text
            if code == 10 and attempts < 3:
                print(f"   reverse-control channel was down and has been re-sent "
                      f"(attempt {attempts}); retrying this segment")
                continue
            break
        if not ok:
            if code == 9:
                print("   PROTECTIVE STOP during this segment: the sequence is "
                      "HALTED.  Do not recover-and-retry -- inspect what the arm "
                      "hit first (2026-11-03 rule).")
            elif code == 11:
                print("   TORQUE INTERFACE DEAD during this segment: the sequence "
                      "is HALTED.  Restart the driver stack before any motion.")
            else:
                print("   segment failed; refusing to continue the descent")
            break
    return ok


def move(name: str, xyz, quat, label: str = "", goal_tol: float = 0.008) -> bool:
    """Plan and execute one move.

    Planning is done by ``plan_joint_locked.py`` (2026-11-02 audit, hole 1): the
    pose must have a collision-free IK solution seeded from the current
    configuration, and the plan is a joint-space goal at that solution, so a
    flipped-wrist branch or a self-colliding RVS stance is refused instead of
    executed.  Descent and carried steps additionally pass the tool-envelope
    check (with the carried box for ``CARRIED``) and are executed segment by
    segment.
    """
    velocity, time_scale, tail_fraction, tail_scale = SPEED[name]
    plan = Path(f"/tmp/teach_{name}.json")
    code, text = run([sys.executable, "-u", str(PLAN),
                      *[f"{v:.6f}" for v in (*xyz, *quat)],
                      "--velocity", str(velocity),
                      "--goal-tol", str(min(goal_tol, 0.004)),
                      "--output", str(plan)], timeout=300.0)
    if code != 0:
        for line in text.splitlines():
            print("   ", line)
        print(f"  plan refused (exit {code}); not moving")
        return False
    segments = SEGMENTED.get(name, 1)
    if segments > 1:
        check_args = [sys.executable, "-u", str(ENVELOPE_CHECK), str(plan),
                      "--sample", "3", "--margin", "0.02"]
        if name in CARRIED:
            check_args += ["--carried"]
        code, text = run(check_args, timeout=600.0)
        for line in text.splitlines():
            print("   ", line)
        if code != 0:
            print(f"   envelope check refused the {name} move; not executing")
            return False
    ok = execute_plan(plan, time_scale, tail_fraction, tail_scale, goal_tol,
                      segments)
    print(f"  {label or name}: {'OK' if ok else 'FAILED'}")
    return ok


def gripper(command: str) -> dict:
    code, text = run([sys.executable, "-u", str(GRIPPER), command], timeout=120.0)
    try:
        return json.loads(text.splitlines()[-1])
    except Exception:
        return {}


def grip_still_holding(min_open_mm: float = 40.0) -> bool:
    """P1-6: refuse to insert when the jaws are no longer on the handles.

    The expanding grasp holds the part with the jaws OPEN at ~50 mm; if the
    position fell back near zero the part is no longer gripped.
    """
    status = gripper("status")
    position = status.get("position_mm")
    held = position is not None and position >= min_open_mm
    print(f"   gripper hold: position={position} mm (need >= {min_open_mm}) -> "
          f"{'HELD' if held else 'NOT HELD'}")
    return held


def verify_pose_gate(xyz, quat, pos_tol_mm: float = 1.0,
                     rot_tol_deg: float = 0.1) -> bool:
    """Refuse a tight move when the TCP is off target beyond the gate.

    Default is the insert gate, recalibrated 2026-11-03 to 1.0 mm: the floor is
    the controller's joint tracking error (~0.0005-0.0008 rad = 0.5-1.0 mm at
    the 353 mm lever), so the old 0.1 mm could never pass and would abort every
    insert before it started.  1.0 mm is still 35x finer than the slot-pose
    error this chain is fighting."""
    code, text = run([sys.executable, "-u", str(REPO / "scripts" / "teaching" / "verify_pose.py"),
                      "--x", str(xyz[0]), "--y", str(xyz[1]), "--z", str(xyz[2]),
                      "--qx", str(quat[0]), "--qy", str(quat[1]),
                      "--qz", str(quat[2]), "--qw", str(quat[3]),
                      "--pos-tol-mm", str(pos_tol_mm),
                      "--rot-tol-deg", str(rot_tol_deg)], timeout=60.0)
    for line in text.splitlines():
        print("   ", line)
    return code == 0


# --- capture validation (2026-11-02 audit, hole 2) -----------------------------
# A capture is only usable when it is plausible AND fresh: the workpiece moves
# between sessions (97 mm measured 10-30 -> 11-02), so a stale or bogus capture
# must never silently drive a grasp.
MAX_CAPTURE_AGE_H = 6.0
WORKSPACE_BOX = {"x": (-0.6, 1.8), "y": (-1.2, 1.6), "z": (-0.9, 1.6)}
# per-command z plausibility, calibrated against every capture on disk
# (2026-11-03 verification):
#   T1 slot pose        9 real answers: 0.4391 / 0.4427; six-mode reference 0.4446
#   T3 workpiece pose   every ACCEPTED answer: 0.3539-0.3558 (nine frames)
#                       the two rejected ones: -0.067 (2026-10-30 garbage),
#                       0.192 at y=1.05 (a bad 10-27 match)
#   T0/T2 view poses    accepted answers: 0.3353-0.6432
#                       the garbage ones: -0.0666 / -0.0771 (below the table!)
# The windows below accept every good capture and reject every known-bad one.
# They are plausibility heuristics for THIS cell, not a physics proof: if the
# cell geometry ever changes, re-derive them from the new captures.
Z_LIMITS = {"T1": (0.30, 0.70), "T3": (0.20, 0.60),
            "T0": (0.20, 1.40), "T2": (0.20, 1.40)}


def validate_capture(record: dict, command: str) -> str | None:
    """Return a reason why the record is unusable, or None when it is fine."""
    p = record.get("position")
    q = record.get("orientation")
    if not p or not q or len(p) != 3 or len(q) != 4:
        return "malformed record"
    for i, axis in enumerate("xyz"):
        low, high = WORKSPACE_BOX[axis]
        if not (low <= p[i] <= high):
            return f"position {axis}={p[i]:.3f} outside the workspace box"
    norm = math.sqrt(sum(v * v for v in q))
    if not (0.99 <= norm <= 1.01):
        return "quaternion norm invalid"
    z_low, z_high = Z_LIMITS.get(command, (-0.9, 1.6))
    if not (z_low <= p[2] <= z_high):
        return f"z={p[2]:.3f} implausible for {command} (limits {z_low}..{z_high})"
    try:
        stamp = time.strptime(record.get("stamp", ""), "%Y-%m-%dT%H:%M:%S")
        age_h = (time.time() - time.mktime(stamp)) / 3600.0
    except (TypeError, ValueError):
        return "missing/unparseable stamp"
    if age_h > MAX_CAPTURE_AGE_H:
        return f"capture is {age_h:.1f} h old (> {MAX_CAPTURE_AGE_H:.0f} h); " \
               "the workpiece may have moved"
    return None


def latest_capture(command: str) -> dict | None:
    """Newest VALID, fresh capture for this command, or None with the reason."""
    folder = REPO / "runtime" / "rvs_teaching"
    files = sorted(folder.glob(f"{command}_*.json"), key=lambda q: q.stat().st_mtime)
    for path in reversed(files):
        record = json.loads(path.read_text())
        reason = validate_capture(record, command)
        if reason is None:
            return record
        print(f"   [capture] {path.name} unusable: {reason}")
    if files:
        print(f"   [capture] no valid {command} capture among {len(files)} files")
    return None


def rvs(command: str) -> dict | None:
    # Owner, 2026-10-30: one trigger rarely succeeds, so rvs_pose_client collects
    # >= 3 plausible frames (never more than 10 attempts) and reports the spread.
    code, text = run([sys.executable, "-u", str(RVS_CLIENT), command,
                      "--timeout", "300",
                      "--min-frames", "3", "--max-attempts", "10"],
                     timeout=3600.0)
    for line in text.splitlines():
        if line.startswith("answer:"):
            print("   ", line)
            return {"line": line}
    print("   ", text.splitlines()[-1] if text else "no answer")
    return None


# --- slot history comparison (owner, 2026-10-30) ------------------------------
# The slot is nearly fixed, so every fresh R1 is compared against past
# measurements and the six-mode reference derived from the proven insert
# constant.  Report only: a deviation is printed loudly but never aborts.
SLOT_HISTORY = REPO / "runtime" / "rvs_teaching" / "slot_history.json"
SLOT_POS_WARN_MM = 30.0
SLOT_ROT_WARN_DEG = 3.0


def six_mode_slot_pose():
    """Slot pose implied by the six-mode insert constant (proven 2026-10-27).

    Inverts the senior's UNTILTED convention (insert = slot * T_s_tcp) with
    R_slot = identity, which matches the R1s measured on 2026-10-30
    (quat ~ identity)."""
    import numpy as np
    from pose_math import T_S_TCP, to_matrix, to_pose
    # untilted insert orientation in the senior convention: diag(-1,1,-1) = Ry(180 deg),
    # the same quaternion as POSE["straight"]; R_slot then comes out identity,
    # matching the R1s measured on 2026-10-30.
    insert = to_matrix([1.1574, -0.0132, 0.0321], [0.0, 1.0, 0.0, 0.0])
    return to_pose(insert @ np.linalg.inv(T_S_TCP))


def check_slot_history(newest_r1: dict | None = None) -> None:
    """Compare the newest R1 against the stored slot history and the six-mode
    reference, then append it.  A missing history file only means there is
    nothing to compare against yet."""
    import numpy as np
    from scipy.spatial.transform import Rotation as R

    history = []
    if SLOT_HISTORY.exists():
        history = json.loads(SLOT_HISTORY.read_text()).get("entries", [])
    r1 = newest_r1 or latest_capture("T1")
    if r1 is None:
        print("   [slot check] no R1 capture yet; nothing to compare")
        return
    ref_xyz, ref_q = six_mode_slot_pose()
    entry = {"stamp": r1.get("stamp", "?"), "position": r1["position"],
             "orientation": r1["orientation"], "source": "T1"}
    if not history or history[-1].get("stamp") != entry["stamp"]:
        history.append(entry)
        SLOT_HISTORY.write_text(json.dumps({"entries": history}, indent=2))

    p_new = np.array(entry["position"], float)
    q_new = R.from_quat(entry["orientation"])

    def diff(other) -> str:
        dp = float(np.linalg.norm(p_new - np.array(other["position"])) * 1000.0)
        da = float(np.degrees(
            (q_new.inv() * R.from_quat(other["orientation"])).magnitude()))
        return f"{dp:.1f} mm / {da:.2f} deg"

    print("   [slot check] newest R1 vs:")
    print("     six-mode reference (10-27 insert constant):",
          diff({"position": ref_xyz, "orientation": ref_q}))
    for prev in history[-3:-1]:
        print(f"     {prev.get('stamp', '?')} ({prev.get('source', '?')}):", diff(prev))
    dp = float(np.linalg.norm(p_new - np.array(ref_xyz)) * 1000.0)
    da = float(np.degrees((q_new.inv() * R.from_quat(ref_q)).magnitude()))
    if dp > SLOT_POS_WARN_MM or da > SLOT_ROT_WARN_DEG:
        print(f"     WARN: beyond {SLOT_POS_WARN_MM:.0f} mm / {SLOT_ROT_WARN_DEG:.1f} deg;"
              " the slot should be nearly fixed")
    else:
        print("     PASS (within the usual small drift)")




# --- RVS view poses are FLANGE poses (owner ruling 2026-11-03) -----------------
# S0/S2 describe where the FLANGE must go; our MoveIt target is the TCP, which
# sits (-0.046, 0, 0.353) in the flange frame.  Using them unchanged moves the arm
# 353 mm off along the tool axis (the mistake made on 2026-10-30/11-02).
FLANGE_TO_TCP = (-0.046, 0.0, 0.353)
VALID_VIEW_FRAMES = ("tcp", "flange")


def view_pose_frame() -> str:
    """The reference frame of RVS view poses, from the ONE place that records it.

    ``config/tasks/common.yaml`` -> ``rvs_view_pose_frame``.  Reading it here (and
    nowhere else) is what stops the flange/TCP confusion from coming back: a new
    script cannot hold a different opinion than the config, and every capture
    record stores the interpretation that was in force.
    """
    try:
        import yaml
        config = yaml.safe_load((REPO / "config" / "tasks" / "common.yaml").read_text())
        value = str(config.get("rvs_view_pose_frame", "tcp")).strip().lower()
    except Exception:
        value = "tcp"
    if value not in VALID_VIEW_FRAMES:
        raise SystemExit(f"config rvs_view_pose_frame={value!r} is not one of "
                         f"{VALID_VIEW_FRAMES}")
    return value


def view_pose_to_tcp(xyz, quat) -> tuple[list[float], list[float]]:
    """Convert an RVS view pose into a TCP pose for MoveIt.

    Identity when the published value is already a TCP pose (verified 2026-11-03);
    otherwise shifts it by the flange -> TCP offset along the tool axis.
    """
    if view_pose_frame() == "tcp":
        return [float(v) for v in xyz], [float(v) for v in quat]
    import numpy as np
    from scipy.spatial.transform import Rotation as R
    rotation = R.from_quat(quat).as_matrix()
    tcp = np.asarray(xyz, dtype=float) + rotation @ np.array(FLANGE_TO_TCP)
    return [float(v) for v in tcp], [float(v) for v in quat]



# --- slot pose: the senior's rule, plus a 180-degree ambiguity guard ----------
# The senior's chain derives the insertion from the LIVE slot pose:
#     insert     = slot_pose x T_s_tcp
#     rotated    = insert    x Ry(-10 deg)
#     pre_insert = rotated   x translate(z -350 mm)
# That stays as it is (owner 2026-11-03: copy his mode, do not substitute a
# constant).  One thing his chain leaves unhandled is the template-matching
# ambiguity: his own projects contain R1 records whose attitude differs by 180 deg
# for the same fixed slot
#     R1,1189.329,154.476,444.371  rpy(179.8,179.5,179.5)
#     R1,1232.115,159.616,443.912  rpy(179.7,179.7,179.3)
#     R1,1236.952,175.494,444.395  rpy( -0.2, -0.1,  0.1)   <- flipped
# so RVS returns both solutions.  Seating a rectangular part into a fixed slot has
# exactly ONE correct attitude, therefore the two candidates are scored against the
# proven reference attitude from the six-mode program (POSE["insert"]) and the
# closer one wins.  A 180 deg mistake here is what "the part attitude does not
# match the slot mouth" looks like on the robot.
REFERENCE_INSERT_QUAT = [-0.004, 0.997, -0.001, 0.083]   # POSE["insert"], tilted entry
SLOT_AMBIGUITY_WARN_DEG = 30.0


def resolve_slot_ambiguity(slot_xyz, slot_quat):
    """Pick between the two template-matching solutions (as-is vs flipped 180)."""
    import numpy as np
    from scipy.spatial.transform import Rotation as R
    from pose_math import insert_from_slot

    reference = R.from_quat(REFERENCE_INSERT_QUAT)
    candidates = []
    for label, flip in (("as-is", 0.0), ("flipped+180", 180.0)):
        quat = slot_quat if flip == 0.0 else (
            R.from_quat(slot_quat) * R.from_euler("z", flip, degrees=True)).as_quat()
        _, tilted, _ = insert_from_slot(slot_xyz, [float(v) for v in quat])
        angle = float(np.degrees((R.from_quat(tilted[1]).inv() * reference).magnitude()))
        candidates.append((angle, label, [float(v) for v in quat]))
    candidates.sort(key=lambda item: item[0])
    best_angle, best_label, best_quat = candidates[0]
    other_angle = candidates[1][0]
    print(f"   [slot] attitude ambiguity: '{best_label}' is {best_angle:.1f} deg from the "
          f"proven reference (other candidate {other_angle:.1f} deg)")
    if min(best_angle, other_angle) > SLOT_AMBIGUITY_WARN_DEG and \
            abs(best_angle - other_angle) < 15.0:
        print("   [slot] WARN: neither candidate is close to the reference attitude; "
              "the slot match may be unreliable today")
    return best_quat, best_label


def slot_pose_for_insert():
    """Live R1 (the senior's source) with the ambiguity resolved.

    The mechanical-calibration value is printed for information only -- it is NOT
    used to replace R1 (owner 2026-11-03).
    """
    import numpy as np
    r1 = latest_capture("T1")
    if r1 is None:
        print("   [slot] no fresh R1; falling back to the proven six-mode slot pose")
        return [1.2395, 0.1798, 0.4446], [0.0, 0.0, 0.0, 1.0], "six-mode constant"
    try:
        import yaml
        scene = yaml.safe_load((REPO / "config" / "cs625_real_cell_scene.yaml").read_text())
        fixture = scene["cs625_real_cell_scene"]["ros__parameters"]["fixture"]["pose_world"]
        gap = float(np.linalg.norm(np.array(r1["position"]) - np.array(fixture[:3])) * 1000.0)
        print(f"   [slot] R1 vs the old calibration record: {gap:.1f} mm (info only)")
    except Exception:
        pass
    quat, label = resolve_slot_ambiguity(r1["position"], r1["orientation"])
    return [float(v) for v in r1["position"]], quat, f"live R1 ({label})"



def view_drift_mm(command: str, xyz, quat) -> float | None:
    """Deviation of a coarse-match OUTPUT from the taught reference, in mm.

    The same fixed target returns coarse matches that drift 21-39 mm between
    sessions (T0: 39 mm on 2026-11-03; T2: 21 mm).  Standing away from the
    taught stance then biases the fine match (R1 by ~50 mm).  Returns None when
    no reference exists for the command.
    """
    import numpy as np
    import yaml
    from scipy.spatial.transform import Rotation as R
    try:
        config = yaml.safe_load((REPO / "config" / "tasks" / "common.yaml").read_text())
        taught = (config.get("rvs_taught_views") or {}).get(command)
    except Exception:
        taught = None
    if not taught or "position" not in taught:
        return None
    position = float(np.linalg.norm(np.asarray(xyz) - np.asarray(taught["position"])) * 1000.0)
    angle = float(np.degrees((R.from_quat(quat).inv() * R.from_quat(taught["quat"])).magnitude()))
    print(f"   [view] {command} vs taught: {position:.1f} mm / {angle:.2f} deg")
    return position

def aim_target():
    """Where the fine-view camera should look: the freshest plausible part pose.

    The 6 h freshness rule that guards a GRASP is deliberately relaxed here: an
    aim hint only has to be good enough to keep the camera on the part (the
    search tolerates 12 deg, and the part moves decimetres at most).  The newest
    T3 that passes the workspace / z plausibility checks is used, and its age is
    printed so a stale hint is visible rather than silent.
    """
    folder = REPO / "runtime" / "rvs_teaching"
    files = sorted(folder.glob("T3_*.json"), key=lambda q: q.stat().st_mtime)
    for path in reversed(files):
        try:
            record = json.loads(path.read_text())
        except (OSError, ValueError):
            continue
        reason = validate_capture(record, "T3")
        if reason is not None and "old" not in reason:
            continue
        try:
            stamp = time.strptime(record.get("stamp", ""), "%Y-%m-%dT%H:%M:%S")
            age_h = (time.time() - time.mktime(stamp)) / 3600.0
        except (TypeError, ValueError):
            age_h = float("nan")
        if age_h > 6.0:
            print(f"   [aim] hint from {path.name} is {age_h:.1f} h old "
                  "(fine for aiming, not for a grasp)")
        return [float(v) for v in record["position"]]
    model_path = REPO / "runtime" / "environment" / "latest" / "model.json"
    try:
        model = json.loads(model_path.read_text())
        workpiece = model.get("workpiece") or {}
        if workpiece.get("xyz"):
            print("   [aim] no T3 capture on disk; using the environment model")
            return [float(v) for v in workpiece["xyz"]]
    except (OSError, ValueError):
        pass
    return None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--confirm", action="store_true",
                        help="actually move; without it only prints the plan")
    parser.add_argument("--allow-release", action="store_true",
                        help="owner approval for opening the jaws (step 16)")
    parser.add_argument("--skip-env", action="store_true")
    parser.add_argument("--skip-observe", action="store_true",
                        help="skip the observation prefix (steps 0-4) and reuse saved captures")
    args = parser.parse_args()

    steps = [
        "0 环境建模（离线，不动机械臂）",
        "1 插槽粗观测 T0",
        "2 插槽精观测 T1",
        "3 工件粗观测 T2",
        "4 工件精观测 T3",
        "5 预抓取位",
        "6 init + 闭合进入抓取位",
        "7 张开夹爪（张紧式抓取）",
        "8 抬升 0.15 m",
        "9 抬到安全高度 0.53",
        "10 transit",
        "11 staging",
        "12 纯旋转 −10°",
        "13 纯下降到预插入高度",
        "14 插入（0.005 m/s）",
        "15 整平（0.1 m/s）",
        "16 释放（需 --allow-release）",
        "17 纯平移退出",
        "18 baseline",
    ]
    for line in steps:
        print(line)
    if not args.confirm:
        print("\ndry run: pass --confirm to execute")
        return 0

    if not args.skip_observe:
        if not args.skip_env:
            print("\n[0] environment model")
            code, text = run([sys.executable, "-u", str(ENV_MODEL),
                              "--out", "runtime/environment"], timeout=600.0)
            print("   ", text.splitlines()[-1] if text else f"exit {code}")
            if code != 0:
                print("   [0] WARN: environment model failed; this chain does "
                      "not consume it, but the six-mode gate would reject it")

        print("\n[1] slot coarse observation")
        if not move("observe", *POSE["slot_coarse"], "slot_coarse"):
            return 2
        if not rvs("T0"):
            return 3

        # T0 answers with S0 = the pose the FINE slot capture must be taken from.
        # Standing on a fixed constant instead is what made T1 answer NG with
        # 'NG,POSE,0,0,0,0,0' on 2026-10-30: RVS only matches from the pose it asks for.
        slot_view = latest_capture("T0")
        if slot_view is None:
            print("missing T0 capture; cannot tell where RVS wants the fine slot view")
            return 4
        # 2026-11-03: the coarse match drifts 21-39 mm between sessions, and
        # standing off the taught stance biases the fine match by the same
        # amount (41 mm off taught -> R1 biased ~50 mm).  Refuse to build the
        # fine-view stance on a drifted coarse match.
        # WARN-only (2026-11-03 measurement): S0 drift does NOT predict the fine
        # match.  Standing on the taught S0 (drift 0) gave R1 y=0.148, standing on
        # today's S0 (drift 34.5) gave R1 y=0.143 -- nearly identical.  RVS's slot
        # match is biased by tens of mm across sessions no matter where the camera
        # stands, so refusing here only blocked a legitimate run.  Keep the number
        # as evidence; the real protection is the fresh R1 + effort margin + 13b.
        drift = view_drift_mm("S0", slot_view["position"], slot_view["orientation"])
        if drift is not None and drift > 20.0:
            print(f"   [view] NOTE: S0 drifts {drift:.1f} mm from the taught stance; "
                  "recorded (does not predict the fine match -- see 2026-11-03 data)")
        slot_tcp, slot_quat = view_pose_to_tcp(slot_view["position"],
                                               slot_view["orientation"])
        print(f"\n[2] slot fine observation: RVS S0 (flange) "
              f"{[round(v, 4) for v in slot_view['position']]} "
              f"-> TCP {[round(v, 4) for v in slot_tcp]}")
        if not move("observe", slot_tcp, slot_quat, "slot_fine"):
            return 2
        if not rvs("T1"):
            return 3
        check_slot_history()

        print("\n[3] workpiece coarse observation")
        if not move("observe", *POSE["workpiece_coarse"], "workpiece_coarse"):
            return 2
        if not rvs("T2"):
            return 3

        # Same contract for the workpiece: T2 answers with S2, the fine view pose.
        box_view = latest_capture("T2")
        if box_view is None:
            print("missing T2 capture; cannot tell where RVS wants the fine box view")
            return 4
        # Same drift check as S0, but warn-only here: the workpiece side
        # tolerated 21 mm on 2026-11-03 (S3 still came out 3 mm from 11-02).
        view_drift_mm("S2", box_view["position"], box_view["orientation"])
        box_tcp, box_quat = view_pose_to_tcp(box_view["position"],
                                             box_view["orientation"])
        print(f"\n[4] workpiece fine observation: RVS S2 (flange) "
              f"{[round(v, 4) for v in box_view['position']]} "
              f"-> TCP {[round(v, 4) for v in box_tcp]}")
        if not move("observe", box_tcp, box_quat, "workpiece_fine"):
            # The requested stance may be unadoptable with the real tool (2026-11-03:
            # the RVS fine-view pose folds the 294.6 mm gripper into the forearm by
            # 8.1 mm) or stale (it aimed 22.7 deg off the part).  Ask the
            # aim-based search for the nearest stance that still looks at the part.
            aim = aim_target()
            if aim is None:
                print("   no aim target (no fresh T3, no workpiece in the model); "
                      "cannot substitute a viewing stance")
                return 2
            print(f"   substituting a stance that aims at {[round(v, 4) for v in aim]}")
            code, text = run([sys.executable, "-u",
                              str(REPO / "scripts" / "teaching" / "nearest_feasible_view.py"),
                              *[f"{v:.6f}" for v in (*box_view["position"], *box_view["orientation"])],
                              "--target", *[f"{v:.6f}" for v in aim],
                              "--json", "/tmp/teach_substitute_view.json"], timeout=600.0)
            for line in text.splitlines():
                print("   ", line)
            if code != 0:
                print("   no adoptable stance found; stopping")
                return 2
            substitute = json.loads(pathlib.Path("/tmp/teach_substitute_view.json").read_text())
            if not move("observe", substitute["xyz"], substitute["quat"],
                        "workpiece_fine(substitute)"):
                return 2
        if not rvs("T3"):
            return 3

    # grasp comes from the live S3 (the workpiece moves between runs; 2026-10-30:
    # today's S3 is 170 mm from the 10-27 one in x/y, z within 0.4 mm).
    # insert comes from the FIXED six-mode constants (owner, 2026-10-27: the slot
    # does not move).  OPEN QUESTION: today's R1-derived insert is
    # (1.1701, -0.0094, 0.0314) vs the constant (1.1574, -0.0132, 0.0321),
    # 13 mm apart.  Not changed here; needs an owner decision.
    box = latest_capture("T3")
    if box is None:
        print("missing T3 capture; run the observation steps or reuse a saved one")
        return 4
    grasp_pose, pre_grasp_pose = grasp_from_box(box["position"], box["orientation"])
    print(f"\n   grasp     {[round(v, 4) for v in grasp_pose[0]]}")
    print(f"   pre_grasp {[round(v, 4) for v in pre_grasp_pose[0]]}")

    # Environment collision scene (2026-11-02 专项, FULL enablement): with the
    # real-cell description (measured camera + SolidWorks tool, no 77 mm finger
    # collisions), the grasp/lift plans succeed WITH the table voxels in the
    # scene (acceptance 2026-11-02: grasp 77 pts, lift 66 pts, pre_grasp 6 pts,
    # negative control refused).  The workpiece region is subtracted at the
    # current S3; the carried part after the grasp is guarded by the offline
    # carried-box gate on transit/staging until the attach 专项 lands.
    print("\n[4b] apply the environment collision scene")
    env_cmd = [sys.executable, "-u",
               str(REPO / "scripts" / "apply_environment_collision.py"),
               "--workpiece-xyz", *[f"{v:.4f}" for v in box["position"]],
               "--workpiece-quat", *[f"{v:.4f}" for v in box["orientation"]]]
    code, text = run(env_cmd, timeout=120.0)
    for line in text.splitlines():
        print("   ", line)
    if code != 0:
        print("   WARN: scene apply failed; falling back to the empty scene "
              "(offline envelope gates still guard every descent)")
        code2, text2 = run([sys.executable, "-u",
                            str(REPO / "scripts" / "apply_environment_collision.py"),
                            "--remove"], timeout=120.0)
        for line in text2.splitlines():
            print("   ", line)
        if code2 != 0:
            print("   WARN: could not clear the scene either; check MoveIt before moving")

    print("\n[5] pre-grasp")
    if not move("pre_grasp", *pre_grasp_pose, "pre-grasp"):
        return 2
    # Gate calibrated 2026-11-03: IK is exact (residual 0.003 mm), so the floor is
    # the controller's joint tracking error (~0.0005-0.0008 rad under load, which
    # is 0.5-1.0 mm at the 353 mm lever).  0.5 mm was unreachable and the loop
    # oscillated; 1.5 mm still catches the real hazard (a 12 deg roll was 12 mm of
    # pad displacement) with margin.
    print("\n[5b] gate: TCP vs pre_grasp (1.5 mm / 0.10 deg)")
    if not verify_pose_gate(*pre_grasp_pose, pos_tol_mm=1.5, rot_tol_deg=0.10):
        print("   gate FAILED: aborting before entering the grasp area")
        return 7

    print("\n[6] init + enter grasp (jaws closed)")
    state = gripper("init")
    print(f"   gripper after init: {state}")
    if not move("grasp", *grasp_pose, "enter grasp"):
        return 2
    print("\n[6b] gate: TCP vs grasp (1.5 mm / 0.10 deg)")
    if not verify_pose_gate(*grasp_pose, pos_tol_mm=1.5, rot_tol_deg=0.10):
        print("   gate FAILED: aborting before opening the jaws")
        return 7

    print("\n[7] open to grip width")
    print(f"   {gripper('grip')}")
    if not grip_still_holding():
        print("   jaws did not reach the grip width; aborting before the lift")
        return 6

    # 专项 2 (2026-11-02): attach the carried workpiece so MoveIt checks it
    # against the environment scene during lift/transit/staging.  On failure the
    # offline carried-box gates still guard the transport.
    print("\n[7b] attach the carried workpiece in the planning scene")
    code, text = run([sys.executable, "-u",
                      str(REPO / "scripts" / "attach_workpiece.py"),
                      "--attach"], timeout=120.0)
    for line in text.splitlines():
        print("   ", line)
    if code != 0:
        print("   WARN: attach failed; offline carried-box gates still guard "
              "lift/transit/staging")

    print("\n[8] lift 0.15 m")
    lifted = [grasp_pose[0][0], grasp_pose[0][1], grasp_pose[0][2] + 0.15]
    if not move("lift", lifted, grasp_pose[1], "lift"):
        return 2

    print("\n[9] raise to safe height")
    raised = [lifted[0], lifted[1], SAFE_HEIGHT_Z]
    if not move("lift", raised, grasp_pose[1], "raise to 0.53"):
        return 2

    print("\n[10] transit")
    if not move("transit", POSE["transit"][0], grasp_pose[1], "transit"):
        return 2

    print("\n[11] staging")
    if not move("staging", POSE["staging"][0], grasp_pose[1], "staging"):
        return 2

    # Insertion chain (2026-11-03 revision of audit hole 5): derived from the
    # MECHANICAL slot calibration, with R1 used as a health check only.  RVS's
    # fine slot match carries a stance-dependent bias of tens of mm (36 mm on
    # 2026-11-03), which is far too coarse for a 1 mm-clearance insertion; the
    # calibrated pose agrees with the proven six-mode constants to 2 mm.
    slot_xyz, slot_quat, slot_source = slot_pose_for_insert()
    insert_flat, insert_tilted, pre_insert_pose = insert_from_slot(slot_xyz, slot_quat)
    print(f"\n   insert chain from {slot_source}:")
    print(f"     insert     {[round(v, 4) for v in insert_flat[0]]}")
    print(f"     tilted     {[round(v, 4) for v in insert_tilted[0]]}")
    print(f"     pre_insert {[round(v, 4) for v in pre_insert_pose[0]]}")

    pre_insert_xyz, pre_insert_q = pre_insert_pose
    insert_xyz, tilted_q = insert_tilted            # tilted entry attitude
    insert_flat_q = insert_flat[1]                  # straightened attitude

    print("\n[12] rotate to -10 deg at safe height (translation unchanged)")
    hold = POSE["staging"][0]
    if not move("pre_insert", [hold[0], hold[1], hold[2]], pre_insert_q,
                "rotate only", goal_tol=0.0003):
        return 2

    print("\n[13] descend vertically to the pre-insert height")
    if not move("pre_insert", pre_insert_xyz, pre_insert_q, "descend only",
                goal_tol=0.0003):
        return 2

    print("\n[13b] gate check: TCP vs pre_insert (<=1.0 mm / <=0.1 deg)")
    if not verify_pose_gate(pre_insert_xyz, pre_insert_q):
        print("   gate FAILED: aborting before insert")
        return 5

    print("\n[13c] grip-hold check before insert")
    if not grip_still_holding():
        print("   part no longer gripped; aborting")
        return 6

    print("\n[14] insert along the tilted axis (effort-supervised, 专项 3 A 期)")
    code, text = run([sys.executable, "-u",
                      str(REPO / "scripts" / "compliant_insert.py"),
                      *[f"{v:.6f}" for v in (*insert_tilted[0], *insert_tilted[1])],
                      "--segments", "6", "--tol-mm", "3.0",
                      "--contact", "0.68", "--safety", "1.63"], timeout=1800.0)
    for line in text.splitlines():
        print("   ", line)
    if code != 0:
        print("   insert aborted/incomplete; stopping the sequence")
        return 8

    print("\n[15] straighten (0.1 m/s)")
    if not move("straighten", insert_xyz, insert_flat_q, "straighten",
                goal_tol=0.0003):
        return 2

    if not args.allow_release:
        print("\n[16] release refused: pass --allow-release after the owner agrees")
        return 0
    print("\n[15b] detach the workpiece and place it in the world scene")
    code, text = run([sys.executable, "-u",
                      str(REPO / "scripts" / "attach_workpiece.py"),
                      "--detach",
                      "--place-xyz", *[f"{v:.4f}" for v in insert_flat[0]],
                      "--place-quat", *[f"{v:.4f}" for v in insert_flat[1]]],
                     timeout=120.0)
    for line in text.splitlines():
        print("   ", line)
    if code != 0:
        print("   WARN: detach failed; the scene may still hold the attached copy "
              "(recover with scripts/attach_workpiece.py --reset)")
    print("\n[16] release (close jaws)")
    print(f"   {gripper('release')}")

    print("\n[17] withdraw (pure translation)")
    if not move("withdraw", pre_insert_xyz, insert_flat_q, "withdraw"):
        return 2

    print("\n[18] baseline")
    if not move("baseline", *POSE["baseline"], "baseline"):
        return 2

    print("\nTEACHING_SEQUENCE_COMPLETE")
    return 0


if __name__ == "__main__":
    sys.exit(main())
