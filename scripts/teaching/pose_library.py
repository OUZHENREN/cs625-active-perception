#!/usr/bin/env python3
"""Pose library + gross-error gate for the teaching chain.

Owner, 2026-11-05: automated teaching runs need an obvious-wrong-pose filter.  Keep a
library of the poses that have already been accepted (workpiece and slot, separately) and
PAUSE the task with an explicit report when a new capture does not resemble anything in it,
instead of carrying a nonsense pose into the grasp or insert stage.

The library is deliberately tolerant -- the workpiece legitimately moves between runs and
the slot measurement drifts by tens of millimetres -- so it only catches GROSS errors
(hundreds of millimetres, flipped orientation), not normal variation.  The per-command
fitness floor and the frame-spread checks remain the primary quality gates.

Storage: runtime/pose_library.json
    {"workpiece": [{"position": [...], "quat": [...], "stamp": ..., "fitness": ...}],
     "slot":      [...]}

Usage:
    python3 scripts/teaching/pose_library.py --show
    python3 scripts/teaching/pose_library.py --kind slot --check 1.25 0.15 0.44
    python3 scripts/teaching/pose_library.py --kind slot --add 1.25 0.15 0.44 \
        --quat 0 0 0 1 --fitness 0.99

Exit codes: 0 ok / within tolerance; 3 rejected as an outlier; 2 storage error.
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation as R

sys.stdout.reconfigure(line_buffering=True)

REPO = pathlib.Path(__file__).resolve().parents[2]
STORE = REPO / "runtime" / "pose_library.json"

# Gross-error limits per kind.  Generous on purpose: this gate exists to stop nonsense,
# not to police normal variation.
LIMITS = {
    "workpiece": {"dist_mm": 250.0, "angle_deg": 45.0},
    "slot": {"dist_mm": 120.0, "angle_deg": 25.0},
}
MIN_ENTRIES = 2          # below this there is nothing to compare against
MAX_ENTRIES = 40         # keep the file small


def load() -> dict:
    if not STORE.exists():
        return {"workpiece": [], "slot": []}
    try:
        data = json.loads(STORE.read_text())
    except json.JSONDecodeError:
        return {"workpiece": [], "slot": []}
    for key in ("workpiece", "slot"):
        data.setdefault(key, [])
    return data


def save(data: dict) -> None:
    STORE.parent.mkdir(parents=True, exist_ok=True)
    for key in ("workpiece", "slot"):
        data[key] = data[key][-MAX_ENTRIES:]
    STORE.write_text(json.dumps(data, indent=2))


def distance(position, entry) -> float:
    return float(np.linalg.norm(np.array(position) - np.array(entry["position"])) * 1000.0)


def angle(position, quat, entry) -> float:
    if not quat or not entry.get("quat"):
        return 0.0
    return float(np.degrees((R.from_quat(quat).inv()
                             * R.from_quat(entry["quat"])).magnitude()))


def nearest(kind: str, position, quat=None):
    """Closest library entry: (distance_mm, angle_deg, entry) or None when empty."""
    entries = load().get(kind) or []
    if not entries:
        return None
    scored = [(distance(position, e), angle(position, quat, e), e) for e in entries]
    return min(scored, key=lambda item: item[0])


def check(kind: str, position, quat=None):
    """(ok, reason, nearest_info).  `ok` False means pause the task."""
    entries = load().get(kind) or []
    if len(entries) < MIN_ENTRIES:
        return True, f"library has {len(entries)}/{MIN_ENTRIES} entries; gate inactive", None
    best = nearest(kind, position, quat)
    dist, ang, entry = best
    limits = LIMITS[kind]
    if dist > limits["dist_mm"] or ang > limits["angle_deg"]:
        reason = (f"OUTLIER {kind}: {dist:.0f} mm / {ang:.1f} deg from the nearest accepted "
                  f"pose (limits {limits['dist_mm']:.0f} mm / {limits['angle_deg']:.0f} deg; "
                  f"nearest was {entry.get('stamp', '?')})")
        return False, reason, best
    return True, f"{dist:.0f} mm / {ang:.1f} deg from the nearest accepted pose", best


def add(kind: str, position, quat=None, fitness=None, note: str = "") -> None:
    data = load()
    entry = {"position": [float(v) for v in position],
             "quat": [float(v) for v in quat] if quat else None,
             "stamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
             "fitness": fitness, "note": note}
    # Skip near-duplicates so the file does not fill with the same pose.
    best = nearest(kind, position, quat)
    if best is not None and best[0] < 5.0 and best[1] < 1.0:
        return
    data[kind].append(entry)
    save(data)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--kind", choices=sorted(LIMITS), default="slot")
    parser.add_argument("--show", action="store_true")
    parser.add_argument("--check", nargs=3, type=float, metavar=("X", "Y", "Z"))
    parser.add_argument("--add", nargs=3, type=float, metavar=("X", "Y", "Z"))
    parser.add_argument("--quat", nargs=4, type=float)
    parser.add_argument("--fitness", type=float, default=None)
    args = parser.parse_args()

    data = load()
    if args.show or not (args.check or args.add):
        for kind in ("workpiece", "slot"):
            entries = data.get(kind) or []
            print(f"{kind}: {len(entries)} entries  limits "
                  f"{LIMITS[kind]['dist_mm']:.0f} mm / {LIMITS[kind]['angle_deg']:.0f} deg")
            for entry in entries[-6:]:
                pos = ", ".join(f"{v:.4f}" for v in entry["position"])
                print(f"   {entry['stamp']}  ({pos})  fitness={entry.get('fitness')}")
        return 0

    if args.add:
        add(args.kind, args.add, args.quat, args.fitness)
        print(f"added to {args.kind}: {[round(v, 4) for v in args.add]}")
        return 0

    ok, reason, best = check(args.kind, args.check, args.quat)
    print(("ACCEPTED: " if ok else "REJECTED: ") + reason)
    return 0 if ok else 3


if __name__ == "__main__":
    raise SystemExit(main())
