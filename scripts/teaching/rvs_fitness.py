#!/usr/bin/env python3
"""Read RVS's own pose-fitness value out of its log file.

Owner, 2026-11-05: only answers above ~0.8 accuracy are meaningful, and the first few
triggers after a move sit around 0.6 while later ones settle above 0.8.  RVS writes that
number itself:

    <2026-11-05 19:34:41.522875*>[rvs_icp](info) [H] pose fitness [0] : 0.844751

The log lives on the Windows side and is reachable from WSL, so the number is usable
without touching RVS or the frozen bridge.  Measured on 2026-11-05: the frames that jumped
around by hundreds of millimetres scored 0.60-0.67, while the accepted frame scored 0.846.

API:
    latest(since=None, log_dir=None) -> list[(datetime, float)]
    for_time(when, window=12.0, log_dir=None)  -> (datetime, float) | None
    show_log_dir() -> str

Usage:
    python3 scripts/teaching/rvs_fitness.py             # last 10 values
    python3 scripts/teaching/rvs_fitness.py --min 0.8   # only passing values

Exit codes: 0 ok; 2 the log directory cannot be read.
"""

from __future__ import annotations

import argparse
import datetime as dt
import os
import pathlib
import re
import sys
import time

sys.stdout.reconfigure(line_buffering=True)

DEFAULT_LOG_DIRS = (
    pathlib.Path("/mnt/d/Program Files (x86)/RobotVisionSuite/runtime/log/rvs_all"),
    pathlib.Path("/mnt/c/Program Files/RobotVisionSuite/runtime/log/rvs_all"),
)
LINE = re.compile(
    r"<(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d+)\*?>\s*\[rvs_icp\]\(info\).*?"
    r"pose fitness \[\d+\] : ([\d.]+)")
FITNESS_MIN = 0.8


def log_dir(explicit: str | None = None) -> pathlib.Path | None:
    if explicit:
        candidate = pathlib.Path(explicit)
        return candidate if candidate.is_dir() else None
    env = os.environ.get("CS625_RVS_LOG_DIR")
    if env and pathlib.Path(env).is_dir():
        return pathlib.Path(env)
    for candidate in DEFAULT_LOG_DIRS:
        if candidate.is_dir():
            return candidate
    return None


def _newest_log(directory: pathlib.Path) -> pathlib.Path | None:
    logs = sorted(directory.glob("rvs_all_*.log"), key=lambda p: p.stat().st_mtime)
    return logs[-1] if logs else None


def latest(since: dt.datetime | None = None, directory: str | None = None):
    """All (timestamp, fitness) pairs from the newest log, oldest first."""
    folder = log_dir(directory)
    if folder is None:
        return []
    path = _newest_log(folder)
    if path is None:
        return []
    text = path.read_text(encoding="utf-8", errors="ignore")
    rows = []
    for stamp, value in LINE.findall(text):
        try:
            when = dt.datetime.strptime(stamp[:26], "%Y-%m-%d %H:%M:%S.%f")
        except ValueError:
            continue
        if since is not None and when < since:
            continue
        rows.append((when, float(value)))
    return rows


def for_time(when: dt.datetime, window: float = 12.0, directory: str | None = None):
    """Fitness logged closest to `when` (within `window` seconds), or None."""
    best = None
    for stamp, value in latest(since=when - dt.timedelta(seconds=window), directory=directory):
        gap = abs((stamp - when).total_seconds())
        if gap <= window and (best is None or gap < best[0]):
            best = (gap, stamp, value)
    return None if best is None else (best[1], best[2])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--min", type=float, default=None, help="only show values >= this")
    parser.add_argument("--last", type=int, default=10)
    parser.add_argument("--dir", default=None, help="override the RVS log directory")
    args = parser.parse_args()

    folder = log_dir(args.dir)
    if folder is None:
        print("REFUSED: RVS log directory not found; set CS625_RVS_LOG_DIR")
        return 2
    rows = latest(directory=args.dir)
    print(f"log: {folder}")
    if not rows:
        print("no pose-fitness lines in the newest log")
        return 0
    shown = [r for r in rows if args.min is None or r[1] >= args.min][-args.last:]
    for stamp, value in shown:
        mark = "ok" if value >= FITNESS_MIN else "--"
        print(f"  {stamp.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}  {value:.6f}  {mark}")
    values = [v for _, v in rows]
    passing = sum(1 for v in values if v >= FITNESS_MIN)
    print(f"\n{len(values)} samples, {passing} above {FITNESS_MIN} "
          f"({passing / len(values) * 100:.0f}%), newest {values[-1]:.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
