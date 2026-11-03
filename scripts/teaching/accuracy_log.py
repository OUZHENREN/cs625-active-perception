#!/usr/bin/env python3
"""Record RVS recognition accuracy over many workpiece placements.

Owner workflow (2026-11-03): place the workpiece by hand wherever, ask for a
measurement, and keep a running accuracy table.  This script drives the existing
rvs_pose_client, then appends one line per trial to a CSV so every placement is
accounted for -- including the failures, which are the interesting half.

Usage:
    # one trial from the current arm pose
    python3 scripts/teaching/accuracy_log.py --command T2 --baseline A --pose 30

    # show the running table
    python3 scripts/teaching/accuracy_log.py --report

Columns:
    stamp, command, baseline, workpiece_yaw_deg, success, n_frames,
    attempts, spread_mm, spread_deg, x, y, z, seconds, note

"success" means the client returned at least --min-frames plausible answers
(the same criterion the teaching chain uses), not merely that RVS replied.

Exit codes: 0 recorded; 1 the trial failed (still recorded); 2 could not run.
"""

from __future__ import annotations

import argparse
import csv
import pathlib
import subprocess
import sys
import time

sys.stdout.reconfigure(line_buffering=True)

REPO = pathlib.Path(__file__).resolve().parents[2]
CLIENT = REPO / "scripts" / "teaching" / "rvs_pose_client.py"
CAPTURES = REPO / "runtime" / "rvs_teaching"
LOG = REPO / "runtime" / "teaching_accuracy.csv"
FIELDS = ["stamp", "command", "baseline", "workpiece_yaw_deg", "success",
          "n_frames", "n_outliers", "attempts", "spread_mm", "spread_deg",
          "x", "y", "z", "seconds", "note"]
# A frame further than this from the median answer is a wrong-feature match, not
# a measurement (measured 2026-11-03: first frame 746 mm off the other two).
INLIER_MM = 30.0


def parse_frames(text: str) -> list[tuple[float, float, float]]:
    """All "valid #k [x, y, z]" answers the client printed, in order."""
    import re
    frames = []
    for line in text.splitlines():
        if "valid #" in line:
            match = re.search(r"\[([-0-9., ]+)\]", line)
            if match:
                values = [float(v) for v in match.group(1).split(",") if v.strip()]
                if len(values) >= 3:
                    frames.append((values[0], values[1], values[2]))
    return frames


def run_trial(command: str, min_frames: int, attempts: int, timeout: float) -> dict:
    """Drive the RVS client once and summarise what happened."""
    started = time.time()
    done = subprocess.run(
        [sys.executable, str(CLIENT), command,
         "--min-frames", str(min_frames),
         "--max-attempts", str(attempts),
         "--timeout", str(timeout)],
        capture_output=True, text=True, timeout=timeout * attempts + 120.0)
    text = (done.stdout or "") + (done.stderr or "")
    frames = parse_frames(text)
    # Consistency filter: RVS occasionally locks onto a wrong feature, so a raw
    # frame count overstates success.  Keep the frames that agree with the median.
    n_outliers = 0
    spread_mm = spread_deg = ""
    kept = frames
    if frames:
        import statistics
        med = tuple(statistics.median([f[i] for f in frames]) for i in range(3))
        kept, outliers = [], []
        for frame in frames:
            distance = sum((frame[i] - med[i]) ** 2 for i in range(3)) ** 0.5 * 1000.0
            (kept if distance <= INLIER_MM else outliers).append(frame)
        n_outliers = len(outliers)
        if not kept:
            kept = []
    success = len(kept) >= min_frames
    if len(kept) >= 2:
        xs = [f[0] for f in kept]; ys = [f[1] for f in kept]; zs = [f[2] for f in kept]
        spread_mm = f"{1000.0 * max(max(xs)-min(xs), max(ys)-min(ys), max(zs)-min(zs)):.1f}"
    x = y = z = ""
    if kept:
        import statistics
        x = f"{statistics.median([f[0] for f in kept]):.4f}"
        y = f"{statistics.median([f[1] for f in kept]):.4f}"
        z = f"{statistics.median([f[2] for f in kept]):.4f}"
    for line in text.splitlines():
        if line.startswith("spread over"):
            parts = line.replace("/", " ").split()
            for i, token in enumerate(parts):
                if token == "mm":
                    spread_mm = parts[i - 1]
                if token == "deg":
                    spread_deg = parts[i - 1]
    attempts_used = text.count("[attempt ")
    return {
        "success": int(success),
        "n_frames": len(kept),
        "n_outliers": n_outliers,
        "attempts": attempts_used,
        "spread_mm": spread_mm,
        "spread_deg": spread_deg,
        "x": x, "y": y, "z": z,
        "seconds": f"{time.time() - started:.1f}",
        "raw": text,
    }


def append(row: dict) -> None:
    LOG.parent.mkdir(parents=True, exist_ok=True)
    new = not LOG.exists()
    with LOG.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS)
        if new:
            writer.writeheader()
        writer.writerow({k: row.get(k, "") for k in FIELDS})


def report() -> int:
    if not LOG.exists():
        print("no trials recorded yet")
        return 0
    with LOG.open(encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        print("no trials recorded yet")
        return 0
    print(f"{'时间':<20}{'命令':<6}{'基准':<6}{'工件角':<8}{'结果':<6}{'帧':<4}{'剔除':<4}{'离散(mm)':<10}{'耗时(s)':<8}")
    for row in rows:
        print(f"{row['stamp']:<20}{row['command']:<6}{row['baseline']:<6}"
              f"{row['workpiece_yaw_deg']:<8}"
              f"{'成功' if row['success'] == '1' else '失败':<6}"
              f"{row['n_frames']:<4}{row.get('n_outliers',''):<4}"
              f"{row['spread_mm']:<10}{row['seconds']:<8}")
    total = len(rows)
    ok = sum(1 for r in rows if r["success"] == "1")
    by_base: dict[str, list[int]] = {}
    for row in rows:
        by_base.setdefault(row["baseline"], []).append(int(row["success"]))
    print(f"\n总计 {total} 次，成功 {ok} 次，准确率 {100.0 * ok / total:.1f}%")
    for name, values in sorted(by_base.items()):
        print(f"  基准 {name}: {len(values)} 次，成功 {sum(values)} 次，"
              f"准确率 {100.0 * sum(values) / len(values):.1f}%")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--command", default="T2", choices=["T0", "T1", "T2", "T3"],
                        help="which RVS trigger to test (coarse = T0/T2)")
    parser.add_argument("--baseline", default="A", help="which arm pose is in use")
    parser.add_argument("--pose", default="", help="workpiece yaw in degrees (as placed)")
    parser.add_argument("--note", default="")
    parser.add_argument("--min-frames", type=int, default=3)
    parser.add_argument("--attempts", type=int, default=4)
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument("--report", action="store_true", help="print the table and exit")
    args = parser.parse_args()

    if args.report:
        return report()

    result = run_trial(args.command, args.min_frames, args.attempts, args.timeout)
    row = {
        "stamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "command": args.command,
        "baseline": args.baseline,
        "workpiece_yaw_deg": args.pose,
        "note": args.note,
        **{k: v for k, v in result.items() if k != "raw"},
    }
    append(row)
    print(f"[{row['stamp']}] {args.command} 基准{args.baseline} 工件角 {args.pose or '?'}° → "
          f"{'成功' if result['success'] else '失败'}  "
          f"一致帧 {result['n_frames']}/{args.min_frames}（剔除错特征 {result['n_outliers']}）"
          f"  spread {result['spread_mm']} mm  {result['seconds']} s")
    print(result["raw"].strip().splitlines()[-1] if result["raw"].strip() else "")
    return 0 if result["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
