#!/usr/bin/env python3
"""Trim one recorded run and archive it under ``$HOME`` with an index entry.

The recorder writes *everything*, including the idle time before the first
motion and the seconds of standing still after the last one.  That is right for
forensics and wrong for a dataset, so this script keeps the useful window and
throws the rest away:

1. **首尾空闲段** — everything before the first state change and after the last
   one is dropped; ``--pad`` seconds of context are kept on each side.
2. **连续无变化段** — a run of samples where nothing moved for longer than
   ``--max-still`` (default 3 s) and no phase change happened inside collapses to
   its first and last sample, so the dwell stays readable without the bulk.
3. **重复帧** — a sample whose payload is identical (within ``--eps``) to the one
   just kept is dropped.

Then it copies the cropped CSV, ``phases.json``, a *selected* image subset
(one per phase enter, one per phase exit, plus one every ``--image-every``
seconds), ``run.json``, a regenerated ``summary.md`` and a ``README.md`` into::

    ${CS625_EXPERIMENT_ARCHIVE:-$HOME/cs625_experiment_archive}/<experiment id>/

and appends one row to ``docs/evidence/experiment_index.md``.  It never touches
the raw run directory, so the archive can be rebuilt at any time.

Usage
-----
::

    python3 scripts/archive_experiment.py --run-dir runtime/experiments/<id>
    # or point it at the run directory of a manual recording, add --dry-run
    # to see the cropping report without writing anything.

Exit codes: 0 archived, 2 bad arguments, 3 archive exists, 4 bad run directory.
"""

from __future__ import annotations

import argparse
import json
import os
import pathlib
import re
import shutil
import sys
import time

sys.stdout.reconfigure(line_buffering=True)

REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "scripts"))

import experiment_recorder as er  # noqa: E402  (same directory, pure helpers)

DEFAULT_INDEX = REPO_ROOT / "docs" / "evidence" / "experiment_index.md"
DEFAULT_ARCHIVE = pathlib.Path.home() / "cs625_experiment_archive"
ENV_ARCHIVE = "CS625_EXPERIMENT_ARCHIVE"

IMAGE_NAME_RE = re.compile(
    r"^img_(\d+)_(\d{8})_(\d{6})_(\d{3})_(.+)\.(png|ppm)$")

EXIT_OK = 0
EXIT_BAD_ARGS = 2
EXIT_ARCHIVE_EXISTS = 3
EXIT_BAD_RUN = 4


# ---------------------------------------------------------------------------
# cropping: pure functions over in-memory rows
# ---------------------------------------------------------------------------
def payload_close(a: dict, b: dict, eps: float) -> bool:
    """True when every numeric payload column agrees within ``eps``.

    A value that appears (``None`` -> ``0.0``) is *not* motion: the first
    joint_states message arriving is a data-quality event, not a robot movement,
    and treating it as activity would stop the edge trim from removing the idle
    seconds before the flow started.
    """
    for column in er.PAYLOAD_COLUMNS:
        left = a.get(column)
        right = b.get(column)
        if left is None or right is None:
            continue
        if abs(float(left) - float(right)) > eps:
            return False
    return True


def _segments(rows: list[dict], eps: float) -> list[tuple[int, int]]:
    """Split rows into runs where each row is payload-close to the previous one."""
    if not rows:
        return []
    runs: list[tuple[int, int]] = []
    start = 0
    for index in range(1, len(rows)):
        same = (payload_close(rows[index], rows[index - 1], eps)
                and (rows[index].get("phase") or "") == (rows[index - 1].get("phase") or ""))
        if not same:
            runs.append((start, index - 1))
            start = index
    runs.append((start, len(rows) - 1))
    return runs


def trim_idle_edges(rows: list[dict], *, pad_s: float = 0.5,
                    eps: float = 1e-4) -> tuple[list[dict], dict]:
    """Drop the idle head/tail around the first and last actual state change.

    A run that never changed at all is returned untouched: a failed experiment
    that only logged a frozen arm is still evidence, and trimming it to zero
    rows would destroy it.
    """
    report = {"head_dropped": 0, "tail_dropped": 0, "had_motion": False,
              "first_activity_s": None, "last_activity_s": None}
    if len(rows) < 2:
        return list(rows), report

    active: list[int] = []
    for index in range(1, len(rows)):
        moved = not payload_close(rows[index], rows[index - 1], eps)
        phase_changed = ((rows[index].get("phase") or "")
                         != (rows[index - 1].get("phase") or ""))
        if moved or phase_changed:
            active.append(index)
    if not active:
        return list(rows), report

    first, last = active[0], active[-1]
    report["had_motion"] = True
    report["first_activity_s"] = rows[first].get("t_wall")
    report["last_activity_s"] = rows[last].get("t_wall")
    low = float(rows[first]["t_wall"]) - float(pad_s)
    high = float(rows[last]["t_wall"]) + float(pad_s)
    kept = [r for r in rows
            if r.get("t_wall") is not None and low <= float(r["t_wall"]) <= high]
    report["head_dropped"] = sum(
        1 for r in rows if r.get("t_wall") is not None and float(r["t_wall"]) < low)
    report["tail_dropped"] = len(rows) - len(kept) - report["head_dropped"]
    return kept, report


def collapse_static_runs(rows: list[dict], *, max_still_s: float = 3.0,
                         eps: float = 1e-4) -> tuple[list[dict], dict]:
    """Collapse motionless runs longer than ``max_still_s`` to first + last row."""
    report = {"static_runs": 0, "static_dropped": 0, "runs": []}
    if len(rows) < 3:
        return list(rows), report
    kept: list[dict] = []
    for start, end in _segments(rows, eps):
        block = rows[start:end + 1]
        if len(block) < 3:
            kept.extend(block)
            continue
        first_t = block[0].get("t_wall")
        last_t = block[-1].get("t_wall")
        if first_t is None or last_t is None:
            kept.extend(block)
            continue
        duration = float(last_t) - float(first_t)
        if duration <= max_still_s:
            kept.extend(block)
            continue
        first_row = dict(block[0])
        last_row = dict(block[-1])
        # The last row is the only record of how long the dwell lasted; the
        # duplicate-frame rule must not erase it (see drop_duplicate_rows).
        last_row["_collapsed_end"] = True
        kept.append(first_row)
        kept.append(last_row)
        report["static_runs"] += 1
        report["static_dropped"] += len(block) - 2
        report["runs"].append({"start_s": float(first_t), "end_s": float(last_t),
                               "duration_s": round(duration, 3),
                               "rows": len(block)})
    return kept, report


def drop_duplicate_rows(rows: list[dict], *, eps: float = 1e-4,
                        span_s: float = 0.25) -> tuple[list[dict], dict]:
    """Thin consecutive identical samples that repeat within ``span_s``.

    ``span_s`` is what separates a *duplicate frame* from a *hold*: three
    identical samples inside 0.15 s are a re-sampled message, while a 2 s
    standstill is real data whose duration a reader must still be able to see.
    Within a hold the rule keeps roughly one row per ``span_s`` instead of
    deleting the whole interval.
    """
    report = {"duplicate_dropped": 0}
    kept: list[dict] = []
    run_start: float | None = None
    for row in rows:
        repeated = (bool(kept)
                    and payload_close(row, kept[-1], eps)
                    and (row.get("phase") or "") == (kept[-1].get("phase") or ""))
        if repeated:
            if run_start is None and kept[-1].get("t_wall") is not None:
                run_start = float(kept[-1]["t_wall"])
            # A collapsed static run keeps its first and last row on purpose:
            # the pair is what makes "the arm stood still for 4.2 s" readable.
            span = (float(row["t_wall"]) - run_start
                    if run_start is not None and row.get("t_wall") is not None
                    else 0.0)
            if span <= span_s and not row.get("_collapsed_end"):
                report["duplicate_dropped"] += 1
                continue
        run_start = None
        kept.append(row)
    return kept, report


def crop_rows(rows: list[dict], *, eps: float = 1e-4, max_still_s: float = 3.0,
              pad_s: float = 0.5,
              dup_span_s: float = 0.25) -> tuple[list[dict], dict]:
    """Apply the three cropping rules in order and report what each one did."""
    ordered = sorted(rows, key=lambda r: float(r.get("t_wall") or 0.0))
    report: dict = {"rows_before": len(ordered), "pad_s": pad_s,
                    "max_still_s": max_still_s, "eps": eps,
                    "dup_span_s": dup_span_s}

    trimmed, edge_report = trim_idle_edges(ordered, pad_s=pad_s, eps=eps)
    report.update({k: v for k, v in edge_report.items() if k != "runs"})
    report["head_dropped"] = edge_report["head_dropped"]
    report["tail_dropped"] = edge_report["tail_dropped"]

    collapsed, static_report = collapse_static_runs(trimmed, max_still_s=max_still_s,
                                                    eps=eps)
    report["static_runs"] = static_report["static_runs"]
    report["static_dropped"] = static_report["static_dropped"]
    report["static_run_details"] = static_report["runs"]

    deduped, dup_report = drop_duplicate_rows(collapsed, eps=eps,
                                              span_s=dup_span_s)
    report["duplicate_dropped"] = dup_report["duplicate_dropped"]

    kept = deduped or trimmed or ordered
    report["rows_after"] = len(kept)
    report["rows_dropped"] = report["rows_before"] - report["rows_after"]
    report["kept_t_start"] = kept[0].get("t_wall") if kept else None
    report["kept_t_end"] = kept[-1].get("t_wall") if kept else None
    report["kept_phases"] = sorted({str(r.get("phase")) for r in kept
                                    if r.get("phase")})
    return kept, report


# ---------------------------------------------------------------------------
# image selection
# ---------------------------------------------------------------------------
def parse_image_stamp(name: str) -> float | None:
    """``img_00007_20261024_202700_123_grasp.png`` -> epoch seconds."""
    match = IMAGE_NAME_RE.match(name)
    if not match:
        return None
    _, date, clock, millis, _, _ = match.groups()
    try:
        moment = time.mktime(time.strptime(date + clock, "%Y%m%d%H%M%S"))
    except (ValueError, OverflowError):
        return None
    return moment + int(millis) / 1000.0


def image_entries_from_dir(run_dir: pathlib.Path) -> list[dict]:
    """Fallback: rebuild the image index from file names when jsonl is missing."""
    entries: list[dict] = []
    images_dir = pathlib.Path(run_dir) / "images"
    if not images_dir.is_dir():
        return entries
    for path in sorted(images_dir.iterdir()):
        if not path.is_file():
            continue
        stamp = parse_image_stamp(path.name)
        if stamp is None:
            continue
        entries.append({"file": f"images/{path.name}", "t_wall": stamp,
                        "phase": ""})
    return entries


def load_images(run_dir: pathlib.Path) -> list[dict]:
    """Read ``images.jsonl`` (written by the recorder) or reconstruct it."""
    index = pathlib.Path(run_dir) / "images.jsonl"
    entries: list[dict] = []
    if index.exists():
        for line in index.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if not line:
                continue
            try:
                record = json.loads(line)
            except ValueError:
                continue
            if record.get("t_wall") is None:
                record["t_wall"] = parse_image_stamp(
                    pathlib.Path(str(record.get("file", ""))).name)
            if record.get("t_wall") is not None:
                entries.append(record)
        if entries:
            return sorted(entries, key=lambda e: float(e["t_wall"]))
    return image_entries_from_dir(run_dir)


def _nearest(images: list[dict], moment: float,
             tolerance: float) -> dict | None:
    best = None
    for image in images:
        distance = abs(float(image["t_wall"]) - float(moment))
        if distance <= tolerance and (best is None or distance < best[0]):
            best = (distance, image)
    return None if best is None else best[1]


def select_images(images: list[dict], phases: list[dict], *,
                  t_start: float | None, t_end: float | None,
                  keep_every_s: float = 30.0,
                  tolerance_s: float = 2.5) -> tuple[list[dict], dict]:
    """Keep one image per phase enter, one per phase exit, plus a periodic grid.

    The camera publishes at ~0.8 fps, so "one image at the boundary" means the
    closest frame within ``tolerance_s``.  If no frame is close enough, that
    boundary simply has no picture instead of a misleading one.
    """
    report = {"images_before": len(images), "images_after": 0,
              "phase_enter": 0, "phase_exit": 0, "periodic": 0}
    if not images:
        return [], report
    in_window = [image for image in images
                 if (t_start is None or float(image["t_wall"]) >= float(t_start))
                 and (t_end is None or float(image["t_wall"]) <= float(t_end))]
    if not in_window:
        in_window = list(images)

    chosen: dict[str, dict] = {}

    def add(image: dict | None, reason: str) -> None:
        if image is None:
            return
        name = str(image.get("file") or "")
        if name in chosen:
            return
        chosen[name] = {**image, "reason": reason}
        report[reason] += 1

    windows = er.phase_windows(phases, t_end)
    for window in windows:
        add(_nearest(in_window, window["start"], tolerance_s), "phase_enter")
        if window["end"] is not None:
            add(_nearest(in_window, window["end"], tolerance_s), "phase_exit")

    span = [float(image["t_wall"]) for image in in_window]
    grid_start = float(t_start) if t_start is not None else min(span)
    grid_end = float(t_end) if t_end is not None else max(span)
    moment = grid_start
    while moment <= grid_end + tolerance_s and keep_every_s > 0:
        add(_nearest(in_window, moment, tolerance_s), "periodic")
        moment += float(keep_every_s)

    if not chosen:
        # Nothing matched a boundary or the grid: keep the closest frame to the
        # middle of the window so the archive still shows what was in view.
        middle = (grid_start + grid_end) / 2.0
        add(_nearest(in_window, middle, float("inf")), "periodic")

    kept = sorted(chosen.values(), key=lambda image: float(image["t_wall"]))
    report["images_after"] = len(kept)
    report["images_dropped"] = report["images_before"] - len(kept)
    return kept, report


# ---------------------------------------------------------------------------
# archive writing
# ---------------------------------------------------------------------------
def resolve_archive_root(value: str | None) -> pathlib.Path:
    """Archive root from CLI, then ``$CS625_EXPERIMENT_ARCHIVE``, then ``$HOME``.

    The root must stay inside the home directory: archives hold real captures and
    must never end up as repository content.
    """
    if value:
        root = pathlib.Path(value).expanduser()
    elif os.environ.get(ENV_ARCHIVE):
        root = pathlib.Path(os.environ[ENV_ARCHIVE]).expanduser()
    else:
        root = DEFAULT_ARCHIVE
    root = root.resolve()
    home = pathlib.Path.home().resolve()
    if root != home and home not in root.parents:
        raise ValueError(f"archive root must be inside {home}, got {root}")
    return root


def build_readme(run: dict, crop: dict, image_report: dict,
                 archive_dir: pathlib.Path) -> str:
    """Self-describing README: what this is, how to read it, how to reproduce."""
    lines: list[str] = []
    lines.append(f"# {run['id']} — 实验数据归档")
    lines.append("")
    lines.append("## 这是什么")
    lines.append("")
    lines.append(f"一次 CS625 机械臂实验的裁剪后数据归档。世界/场景 `{run.get('world')}`，"
                 f"任务范围 `{run.get('scope')}`，运行模式 `{run.get('mode')}`，"
                 f"采样来源 `{run.get('source')}`。")
    lines.append(f"原始记录目录：`{run.get('run_dir')}`（未改动，可重新裁剪）。")
    lines.append(f"归档时间：{run.get('archive', {}).get('archived_at')}。")
    lines.append("")
    lines.append("## 怎么读")
    lines.append("")
    lines.append("- `timeseries.csv`：一行一个采样点（默认 20 Hz），时间列为 Unix 纪元秒；")
    lines.append("- `phases.json`：阶段边界列表（标签、进入时刻、所在行号）；")
    lines.append("- `images/`：每阶段进/出各一张 + 每 30 秒一张的相机原图；")
    lines.append("- `run.json`：本次运行的元数据与结果码；")
    lines.append("- `summary.md`：中文摘要，含 TCP 行程、最大速度、最大力矩、每阶段时长。")
    lines.append("")
    lines.append("读成表格（只用标准库）：")
    lines.append("")
    lines.append("```python")
    lines.append("import csv")
    lines.append("rows = list(csv.DictReader(open('timeseries.csv')))")
    lines.append("print(len(rows), rows[0]['t_wall'], rows[-1]['t_wall'])")
    lines.append("```")
    lines.append("")
    lines.append("## 列与单位")
    lines.append("")
    lines.append("| 列 | 单位 | 含义 |")
    lines.append("| --- | --- | --- |")
    lines.append("| `t_wall` | s | Unix 纪元秒（墙钟），裁剪/对齐都基于它 |")
    lines.append("| `t_ros` | s | 采样时刻的 ROS 时钟；`use_sim_time` 开启时是仿真时间 |")
    for index in range(1, er.JOINT_COUNT + 1):
        lines.append(f"| `joint_pos_{index}` | rad | 关节 {index} 位置（顺序见下） |")
    for index in range(1, er.JOINT_COUNT + 1):
        lines.append(f"| `joint_vel_{index}` | rad/s | 关节 {index} 速度 |")
    for index in range(1, er.JOINT_COUNT + 1):
        lines.append(f"| `joint_effort_{index}` | N·m | 关节 {index} 力矩（无则空） |")
    lines.append("| `tcp_x`, `tcp_y`, `tcp_z` | m | 工具中心点在 `base` 坐标系的位置 |")
    lines.append("| `tcp_qx`..`tcp_qw` | — | TCP 姿态四元数（`base` 坐标系，单位四元数） |")
    lines.append("| `fx`, `fy`, `fz` | N | TCP 力（无 F/T 数据时为空） |")
    lines.append("| `tx`, `ty`, `tz` | N·m | TCP 力矩（无 F/T 数据时为空） |")
    lines.append("| `phase` | — | 当前阶段标签，例如 `grasp:enter` |")
    lines.append("")
    joint_names = run.get("joint_names") or list(er.CANONICAL_JOINT_ORDER)
    lines.append("关节列顺序：" + "、".join(
        f"`joint_pos_{i}` = `{name}`" for i, name in
        enumerate(joint_names[:er.JOINT_COUNT], start=1)))
    lines.append("")
    lines.append("空单元格表示“该时刻没有可用数据”，不是 0。")
    lines.append("")
    lines.append("## 裁剪规则")
    lines.append("")
    lines.append(f"- 首尾空闲：去掉开始前 {crop.get('head_dropped')} 行、结束后 "
                 f"{crop.get('tail_dropped')} 行，两侧各保留 {crop.get('pad_s')} s 上下文；")
    lines.append(f"- 连续静止：静止超过 {crop.get('max_still_s')} s 且无阶段变化的段折叠为首尾两行，"
                 f"共 {crop.get('static_runs')} 段、去掉 {crop.get('static_dropped')} 行；")
    lines.append(f"- 重复帧：载荷在 {crop.get('eps')} 容差内重复、且与上一保留样本相距不超过 "
                 f"{crop.get('dup_span_s')} s 的行去掉 {crop.get('duplicate_dropped')} 行"
                 "（静止段按约 0.25 s 保留一行，不整段删除）。")
    lines.append(f"- 行数：{crop.get('rows_before')} → {crop.get('rows_after')}；"
                 f"图像：{image_report.get('images_before')} → "
                 f"{image_report.get('images_after')}。")
    lines.append("")
    lines.append("## 怎么复现")
    lines.append("")
    params = run.get("params") or {}
    lines.append("```bash")
    lines.append("source scripts/source_dev_env.sh --full")
    lines.append(f"python3 scripts/experiment_recorder.py start --world {run.get('world')} "
                 f"--scope {run.get('scope')} --mode {run.get('mode')} "
                 f"--rate {params.get('rate_hz')}"
                 + (" --images" if params.get("images") else ""))
    lines.append("python3 scripts/experiment_recorder.py stop")
    lines.append(f"python3 scripts/archive_experiment.py --run-dir {run.get('run_dir')}")
    lines.append("```")
    lines.append("")
    lines.append(f"复现同一条命令得到的新实验 id 会不同；归档路径：`{archive_dir}`。")
    lines.append("")
    return "\n".join(lines)


def append_index(index_path: pathlib.Path, row: dict) -> bool:
    """Append one markdown table row; returns ``False`` when the id is present."""
    index_path.parent.mkdir(parents=True, exist_ok=True)
    header = (
        "# 实验数据索引（自动生成）\n\n"
        "本文件由 `scripts/archive_experiment.py` 在每次归档后追加一行。\n"
        "归档根目录默认 `~/cs625_experiment_archive/`"
        "（可用环境变量 `CS625_EXPERIMENT_ARCHIVE` 覆盖）。\n\n"
        "| id | 日期 | world | scope | mode | 耗时 (s) | 结果 | 归档路径 |\n"
        "| --- | --- | --- | --- | --- | --- | --- | --- |\n"
    )
    existing = ""
    if index_path.exists():
        existing = index_path.read_text(encoding="utf-8")
        if not existing.strip():
            existing = ""
    if f"| {row['id']} |" in existing:
        return False
    if not existing:
        index_path.write_text(header, encoding="utf-8")
        existing = header
    if not existing.endswith("\n"):
        existing += "\n"
    line = (f"| {row['id']} | {row['date']} | {row['world']} | {row['scope']} | "
            f"{row['mode']} | {row['duration_s']} | {row['result']} | "
            f"{row['archive_dir']} |\n")
    index_path.write_text(existing + line, encoding="utf-8")
    return True


def archive_run(run_dir: pathlib.Path, archive_root: pathlib.Path, *,
                eps: float, max_still_s: float, pad_s: float,
                keep_every_s: float, tolerance_s: float, dup_span_s: float = 0.25,
                index_path: pathlib.Path | None, force: bool = False,
                dry_run: bool = False, out=print) -> dict:
    """Crop, copy and index one run; returns the machine-readable report."""
    run_dir = pathlib.Path(run_dir).expanduser().resolve()
    csv_path = run_dir / "timeseries.csv"
    if not run_dir.is_dir() or not csv_path.exists():
        raise FileNotFoundError(f"{run_dir} is not a recorded run (no timeseries.csv)")
    run = {}
    run_json_path = run_dir / "run.json"
    if run_json_path.exists():
        run = json.loads(run_json_path.read_text(encoding="utf-8"))
    experiment_id = str(run.get("id") or run_dir.name)
    phases_payload: dict = {}
    phases_path = run_dir / "phases.json"
    if phases_path.exists():
        phases_payload = json.loads(phases_path.read_text(encoding="utf-8"))
    phases = phases_payload.get("phases") or []

    rows = er.read_timeseries(csv_path)
    kept, crop = crop_rows(rows, eps=eps, max_still_s=max_still_s, pad_s=pad_s,
                           dup_span_s=dup_span_s)
    stats = er.compute_stats(kept)

    images = load_images(run_dir)
    selected, image_report = select_images(
        images, phases, t_start=crop.get("kept_t_start"),
        t_end=crop.get("kept_t_end"), keep_every_s=keep_every_s,
        tolerance_s=tolerance_s)
    crop.update({k: v for k, v in image_report.items()
                 if k.startswith("images_")})

    out(f"裁剪前: rows={crop['rows_before']} phases={len(phases)} "
        f"images={image_report['images_before']}")
    out(f"[1/3] 首尾空闲: 去掉头部 {crop['head_dropped']} 行 / 尾部 "
        f"{crop['tail_dropped']} 行 (pad={pad_s}s, had_motion={crop['had_motion']})")
    out(f"[2/3] 连续静止段: 折叠 {crop['static_runs']} 段, 去掉 "
        f"{crop['static_dropped']} 行 (max_still={max_still_s}s)")
    out(f"[3/3] 重复帧: 去掉 {crop['duplicate_dropped']} 行 (eps={eps})")
    out(f"裁剪后: rows={crop['rows_after']} 保留阶段={crop['kept_phases']} "
        f"图像保留={image_report['images_after']} "
        f"(进 {image_report['phase_enter']} / 出 {image_report['phase_exit']} / "
        f"周期 {image_report['periodic']})")

    archive_dir = pathlib.Path(archive_root) / experiment_id
    if archive_dir.exists() and not force:
        raise FileExistsError(f"{archive_dir} already exists (use --force to replace)")
    if dry_run:
        out(f"[dry-run] 将归档到 {archive_dir}")
        return {"experiment_id": experiment_id, "archive_dir": str(archive_dir),
                "crop": crop, "stats": stats, "images": image_report,
                "dry_run": True}

    archive_dir.mkdir(parents=True, exist_ok=True)
    er.write_timeseries(archive_dir / "timeseries.csv", kept)

    if phases_payload:
        payload = dict(phases_payload)
        payload["crop"] = {k: crop[k] for k in (
            "rows_before", "rows_after", "kept_t_start", "kept_t_end")}
        (archive_dir / "phases.json").write_text(
            json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8")
    elif phases_path.exists():
        shutil.copy2(phases_path, archive_dir / "phases.json")

    images_out = archive_dir / "images"
    images_out.mkdir(exist_ok=True)
    kept_images: list[dict] = []
    for entry in selected:
        source = run_dir / str(entry.get("file"))
        if not source.exists():
            continue
        shutil.copy2(source, images_out / source.name)
        kept_images.append(entry)
    with (archive_dir / "images.jsonl").open("w", encoding="utf-8") as handle:
        for entry in kept_images:
            handle.write(json.dumps(entry, ensure_ascii=False) + "\n")

    merged_run = dict(run)
    merged_run["run_dir"] = str(run_dir)
    merged_run["archive_dir"] = str(archive_dir)
    # The summary must describe the archive, not the raw run: 15 raw frames
    # ending up as 2 archived ones is exactly what the reader needs to see.
    merged_run["images_original"] = run.get("images")
    merged_run["images"] = len(kept_images)
    merged_run["archive"] = {
        "archived_at": er.iso_local(time.time()),
        "source_run_dir": str(run_dir),
        "crop": crop,
        "images": image_report,
        "stats": stats,
        "index": str(index_path) if index_path else None,
    }
    (archive_dir / "run.json").write_text(
        json.dumps(merged_run, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8")
    summary = er.render_summary(
        merged_run, stats, phases, crop=crop,
        phase_row_counts=er.count_rows_per_phase(kept))
    (archive_dir / "summary.md").write_text(summary, encoding="utf-8")
    (archive_dir / "README.md").write_text(
        build_readme(merged_run, crop, image_report, archive_dir),
        encoding="utf-8")
    log_path = run_dir / "recorder.log"
    if log_path.exists():
        shutil.copy2(log_path, archive_dir / "recorder.log")

    indexed = False
    if index_path is not None:
        date = (merged_run.get("started_at") or "")[:10]
        if not date:
            date = time.strftime("%Y-%m-%d", time.localtime(
                float(crop.get("kept_t_start") or time.time())))
        row = {
            "id": experiment_id,
            "date": date,
            "world": merged_run.get("world") or "unknown",
            "scope": merged_run.get("scope") or "unknown",
            "mode": merged_run.get("mode") or "unknown",
            "duration_s": (f"{stats['duration_s']:.2f}"
                           if stats.get("duration_s") is not None else "—"),
            "result": merged_run.get("result") or "unknown",
            "archive_dir": str(archive_dir),
        }
        indexed = append_index(index_path, row)
        out(f"索引: {'已追加' if indexed else '该 id 已存在，跳过'} -> {index_path}")
    out(f"归档完成: {archive_dir}")
    return {"experiment_id": experiment_id, "archive_dir": str(archive_dir),
            "crop": crop, "stats": stats, "images": image_report,
            "indexed": indexed, "index_path": str(index_path) if index_path else None}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Crop a recorded experiment run and archive it with an index row.")
    parser.add_argument("--run-dir", required=True,
                        help="runtime/experiments/<id> directory written by the recorder")
    parser.add_argument("--archive", default=None,
                        help="archive root (default $CS625_EXPERIMENT_ARCHIVE or "
                             "~/cs625_experiment_archive)")
    parser.add_argument("--eps", type=float, default=1e-4,
                        help="tolerance for 'unchanged' (m/rad/N/N·m), default 1e-4")
    parser.add_argument("--max-still", type=float, default=3.0,
                        help="collapse motionless runs longer than this many seconds")
    parser.add_argument("--pad", type=float, default=0.5,
                        help="context kept before/after the active window")
    parser.add_argument("--dup-window", type=float, default=0.25,
                        help="identical samples closer than this many seconds "
                             "count as a repeated frame (default 0.25)")
    parser.add_argument("--image-every", type=float, default=30.0,
                        help="keep one image every N seconds (default 30)")
    parser.add_argument("--image-tolerance", type=float, default=2.5,
                        help="how far an image may be from a boundary (default 2.5 s)")
    parser.add_argument("--index", default=str(DEFAULT_INDEX),
                        help="markdown index file to append to")
    parser.add_argument("--no-index", action="store_true",
                        help="do not touch the markdown index")
    parser.add_argument("--force", action="store_true",
                        help="replace an existing archive directory")
    parser.add_argument("--dry-run", action="store_true",
                        help="print the cropping report without writing files")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        archive_root = resolve_archive_root(args.archive)
    except ValueError as exc:
        print(f"[archive_experiment] {exc}", file=sys.stderr)
        return EXIT_BAD_ARGS
    index_path = None if args.no_index else pathlib.Path(args.index).expanduser()
    try:
        archive_run(pathlib.Path(args.run_dir), archive_root,
                    eps=args.eps, max_still_s=args.max_still,
                    pad_s=args.pad, keep_every_s=args.image_every,
                    dup_span_s=args.dup_window,
                    tolerance_s=args.image_tolerance, index_path=index_path,
                    force=args.force, dry_run=args.dry_run)
    except FileNotFoundError as exc:
        print(f"[archive_experiment] {exc}", file=sys.stderr)
        return EXIT_BAD_RUN
    except FileExistsError as exc:
        print(f"[archive_experiment] {exc}", file=sys.stderr)
        return EXIT_ARCHIVE_EXISTS
    return EXIT_OK


if __name__ == "__main__":
    raise SystemExit(main())
