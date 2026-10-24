#!/usr/bin/env python3
"""Record one experiment run as a fixed-rate numeric timeseries + phase log.

What this script is for
-----------------------
Every pick-and-place run should leave a machine-readable trace that answers
"how long did each stage take, how far did the TCP travel, how fast did it move,
how much force did it see".  The task driver only owns *motion*; this recorder
owns *recording*.  It never publishes a command, never creates a controller and
never touches the robot: it subscribes, samples on its own clock and writes
files.

What it subscribes to (all read-only)
-------------------------------------
==============================  ==========================  ==================
topic                           type                        used for
==============================  ==========================  ==================
`/joint_states`                 sensor_msgs/JointState      pos/vel/effort
`/tcp_pose_broadcaster/pose`    geometry_msgs/PoseStamped   tcp x..qw (base)
wrench candidates               geometry_msgs/WrenchStamped f/t columns
`/arm_controller/controller_state` control_msgs/...        liveness only
`/camera/color/image_raw`       sensor_msgs/Image           images/ (0.8 fps)
`/pipeline_state` (optional)    std_msgs/String             phase tag
==============================  ==========================  ==================

Two findings from the live graph (2026-10-24) that shape the code:

* ``/pipeline_state`` in this repository is *MoveIt's* ``PipelineState``
  (``moveit_msgs/PipelineState``), not a task-phase topic.  The recorder
  therefore only treats it as a phase source when its type is ``std_msgs/String``;
  otherwise the phase comes from the external ``set-phase`` command, which is
  what the task driver should call at every stage boundary.
* ``/cs625/tcp_force`` does not exist on this cell.  The real wrench topic is
  ``/force_torque_sensor_broadcaster/wrench`` (and it currently has no data in
  Gazebo because the simulated arm has no F/T sensor).  Candidate topics are
  probed in order and the resolution is written to ``run.json``; when nothing
  publishes, ``fx..tz`` stay empty rather than being invented.

The CSV
-------
One row per sampling tick (default 20 Hz), never per incoming message, so the
file has a predictable shape::

    t_wall, t_ros, joint_pos_1..6, joint_vel_1..6, joint_effort_1..6,
    tcp_x, tcp_y, tcp_z, tcp_qx..qw, fx, fy, fz, tx, ty, tz, phase

Units are SI: metres, radians, newtons, newton-metres, seconds.  ``t_wall`` is
Unix epoch seconds; ``t_ros`` is the ROS clock at the tick (simulation time when
``use_sim_time`` is on).  Empty fields mean "not available", never zero.

Usage
-----
::

    # start a background recorder; writes runtime/recorder.pid
    python3 scripts/experiment_recorder.py start \
        --world sim --scope pick_and_place --mode efficient --images

    # optional: tag a stage boundary from the task driver (idempotent, cheap)
    python3 scripts/experiment_recorder.py set-phase --tag grasp:enter

    # stop: finalises run.json / summary.md and exits 0
    python3 scripts/experiment_recorder.py stop

    # peek at the running recorder
    python3 scripts/experiment_recorder.py status

Exit codes: 0 ok, 2 bad arguments, 3 nothing running, 4 start failed,
5 ROS unavailable, 6 stop timed out.
"""

from __future__ import annotations

import argparse
import csv
import datetime
import json
import math
import os
import pathlib
import re
import signal
import subprocess
import sys
import time

sys.stdout.reconfigure(line_buffering=True)

REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
STATE_DIR = REPO_ROOT / "runtime"
PIDFILE = STATE_DIR / "recorder.pid"
STATEFILE = STATE_DIR / "recorder.state.json"
DEFAULT_OUT = STATE_DIR / "experiments"

WORLDS = ("real", "sim")
SOURCES = ("ros", "synthetic")

TOPIC_JOINTS = "/joint_states"
TOPIC_TCP = "/tcp_pose_broadcaster/pose"
TOPIC_CONTROLLER = "/arm_controller/controller_state"
TOPIC_CAMERA = "/camera/color/image_raw"
TOPIC_PHASE_SPEC = "/pipeline_state"
# Probed in order; the first one that publishes wins and is written to run.json.
WRENCH_CANDIDATES = (
    "/cs625/tcp_force",
    "/force_torque_sensor_broadcaster/wrench",
    "/cs625/tcp_force_filtered",
)

# The kinematic chain order of the CS625.  joint_pos_1 is the base joint and
# joint_pos_6 the flange, so a column name has a stable physical meaning even
# when the driver publishes the joints in another order.
CANONICAL_JOINT_ORDER = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
JOINT_COUNT = 6

COLUMNS = (
    ["t_wall", "t_ros"]
    + [f"joint_pos_{i}" for i in range(1, JOINT_COUNT + 1)]
    + [f"joint_vel_{i}" for i in range(1, JOINT_COUNT + 1)]
    + [f"joint_effort_{i}" for i in range(1, JOINT_COUNT + 1)]
    + ["tcp_x", "tcp_y", "tcp_z", "tcp_qx", "tcp_qy", "tcp_qz", "tcp_qw"]
    + ["fx", "fy", "fz", "tx", "ty", "tz", "phase"]
)
# Columns compared when deciding "did the state change".
PAYLOAD_COLUMNS = [c for c in COLUMNS if c not in ("t_wall", "t_ros", "phase")]
NUMERIC_COLUMNS = [c for c in COLUMNS if c != "phase"]

PHASE_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.:-]{0,63}$")
ID_TOKEN_RE = re.compile(r"[^A-Za-z0-9_.-]+")

EXIT_OK = 0
EXIT_BAD_ARGS = 2
EXIT_NOT_RUNNING = 3
EXIT_START_FAILED = 4
EXIT_NO_ROS = 5
EXIT_STOP_TIMEOUT = 6


# ---------------------------------------------------------------------------
# pure helpers (imported by archive_experiment.py and test/test_experiment_recorder.py)
# ---------------------------------------------------------------------------
def sanitize_token(text: str, *, default: str = "unknown") -> str:
    """Collapse anything that is not ``[A-Za-z0-9_.-]`` so it is safe in a path."""
    cleaned = ID_TOKEN_RE.sub("_", str(text or "").strip()).strip("_.-")
    return cleaned[:40] or default


def make_experiment_id(now: float, world: str, scope: str, mode: str) -> str:
    """``<YYYYmmdd_HHMMSS>_<world>_<scope>_<mode>`` in local time."""
    stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime(float(now)))
    return "_".join((stamp, sanitize_token(world), sanitize_token(scope),
                     sanitize_token(mode)))


def iso_local(ts: float | None) -> str | None:
    """Epoch seconds -> local ISO-8601 with an offset, or ``None``."""
    if ts is None or not math.isfinite(float(ts)):
        return None
    return datetime.datetime.fromtimestamp(
        float(ts)).astimezone().isoformat(timespec="seconds")


def _number(value) -> float | None:
    """Best-effort float; NaN/inf/''/garbage all become ``None``."""
    if value is None:
        return None
    if isinstance(value, bool):
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(parsed):
        return None
    return parsed


def format_cell(value) -> str:
    """CSV cell text: six decimals, empty when the value is unknown."""
    number = _number(value)
    if number is None:
        return ""
    return f"{number:.6f}"


def format_row_values(row: dict) -> list[str]:
    """One CSV record in column order; ``phase`` stays a string, numbers do not."""
    values: list[str] = []
    for column in COLUMNS:
        if column == "phase":
            values.append(str(row.get("phase") or ""))
        else:
            values.append(format_cell(row.get(column)))
    return values


def git_commit(repo: str | pathlib.Path) -> tuple[str | None, bool | None]:
    """``(HEAD sha, dirty)``; ``(None, None)`` outside a git checkout."""
    try:
        head = subprocess.run(["git", "-C", str(repo), "rev-parse", "HEAD"],
                              capture_output=True, text=True, timeout=5)
        if head.returncode != 0:
            return None, None
        status = subprocess.run(["git", "-C", str(repo), "status", "--porcelain"],
                                capture_output=True, text=True, timeout=5)
        dirty = bool(status.stdout.strip()) if status.returncode == 0 else None
        return head.stdout.strip(), dirty
    except (OSError, subprocess.SubprocessError):
        return None, None


def validate_phase_tag(tag: str) -> str:
    """Normalise a phase tag: ``"grasp:enter"``, ``"observe:left"`` or ``"grasp"``."""
    text = str(tag or "").strip()
    if not text or not PHASE_RE.match(text):
        raise ValueError(
            f"phase tag must match {PHASE_RE.pattern}, got {tag!r}")
    return text


def write_phases_json(path: pathlib.Path, experiment_id: str,
                      entries: list[dict]) -> None:
    """Atomically rewrite ``phases.json`` (the reader may be a human)."""
    payload = {
        "experiment_id": experiment_id,
        "updated_at": iso_local(time.time()),
        "count": len(entries),
        "phases": entries,
    }
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
                   encoding="utf-8")
    os.replace(tmp, path)


def write_timeseries(path: pathlib.Path, rows: list[dict]) -> None:
    """Write a header + rows using the canonical column order."""
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle, lineterminator="\n")
        writer.writerow(COLUMNS)
        for row in rows:
            writer.writerow(format_row_values(row))


def read_timeseries(path: pathlib.Path) -> list[dict]:
    """Read ``timeseries.csv`` into typed rows (``None`` for empty cells)."""
    rows: list[dict] = []
    with pathlib.Path(path).open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            return rows
        missing = [c for c in COLUMNS if c not in reader.fieldnames]
        if missing:
            raise ValueError(f"{path} is missing columns: {missing}")
        for raw in reader:
            row: dict = {}
            for column in COLUMNS:
                if column == "phase":
                    row[column] = (raw.get(column) or "").strip()
                else:
                    row[column] = _number(raw.get(column))
            rows.append(row)
    return rows


def phase_windows(phases: list[dict], t_end: float | None) -> list[dict]:
    """Turn phase *entries* into ``{tag, start, end, duration}`` windows."""
    ordered = sorted(phases, key=lambda p: float(p.get("t_wall") or 0.0))
    windows: list[dict] = []
    for index, entry in enumerate(ordered):
        start = _number(entry.get("t_wall"))
        if start is None:
            continue
        if index + 1 < len(ordered):
            end = _number(ordered[index + 1].get("t_wall"))
        else:
            end = _number(t_end)
        windows.append({
            "tag": str(entry.get("tag") or ""),
            "start": start,
            "end": end if end is not None and end >= start else None,
        })
    for window in windows:
        end = window["end"]
        window["duration"] = (end - window["start"]) if end is not None else None
    return windows


def compute_stats(rows: list[dict]) -> dict:
    """Key statistics of one timeseries (pure; used by the summary writer)."""
    stats: dict = {
        "rows": len(rows),
        "t_start": None,
        "t_end": None,
        "duration_s": None,
        "tcp_travel_m": None,
        "max_tcp_speed_m_s": None,
        "max_tcp_speed_at_s": None,
        "joint_travel_rad": None,
        "max_joint_speed_rad_s": None,
        "max_force_n": None,
        "max_torque_nm": None,
        "tcp_samples": 0,
        "joint_samples": 0,
        "wrench_samples": 0,
    }
    if not rows:
        return stats

    walls = [r["t_wall"] for r in rows if r.get("t_wall") is not None]
    if walls:
        stats["t_start"] = min(walls)
        stats["t_end"] = max(walls)
        stats["duration_s"] = stats["t_end"] - stats["t_start"]

    travel = 0.0
    have_travel = False
    joint_travel = 0.0
    have_joint_travel = False
    peak_speed = None
    peak_speed_at = None
    peak_joint_speed = None
    peak_force = None
    peak_torque = None

    previous = None
    for row in rows:
        tcp = [_number(row.get(c)) for c in ("tcp_x", "tcp_y", "tcp_z")]
        joints = [_number(row.get(f"joint_pos_{i}")) for i in range(1, JOINT_COUNT + 1)]
        if all(v is not None for v in tcp):
            stats["tcp_samples"] += 1
        if all(v is not None for v in joints):
            stats["joint_samples"] += 1

        wrench = [_number(row.get(c)) for c in ("fx", "fy", "fz", "tx", "ty", "tz")]
        if any(v is not None for v in wrench):
            stats["wrench_samples"] += 1
            force = [v for v in wrench[:3] if v is not None]
            torque = [v for v in wrench[3:] if v is not None]
            if len(force) == 3:
                norm = math.sqrt(sum(v * v for v in force))
                peak_force = norm if peak_force is None else max(peak_force, norm)
            if len(torque) == 3:
                norm = math.sqrt(sum(v * v for v in torque))
                peak_torque = norm if peak_torque is None else max(peak_torque, norm)

        speed = None
        for name in ("joint_vel_1", "joint_vel_2", "joint_vel_3",
                     "joint_vel_4", "joint_vel_5", "joint_vel_6"):
            value = _number(row.get(name))
            if value is not None:
                speed = abs(value) if speed is None else max(speed, abs(value))
        if speed is not None:
            peak_joint_speed = speed if peak_joint_speed is None else max(peak_joint_speed, speed)

        if previous is not None and all(v is not None for v in tcp) \
                and all(v is not None for v in previous["tcp"]):
            step = math.dist(tcp, previous["tcp"])
            travel += step
            have_travel = True
            dt = None
            if row.get("t_wall") is not None and previous["t_wall"] is not None:
                dt = float(row["t_wall"]) - float(previous["t_wall"])
            if dt is not None and dt > 1e-6:
                speed = step / dt
                if peak_speed is None or speed > peak_speed:
                    peak_speed = speed
                    peak_speed_at = float(row["t_wall"])
        if previous is not None and all(v is not None for v in joints) \
                and all(v is not None for v in previous["joints"]):
            joint_travel += math.dist(joints, previous["joints"])
            have_joint_travel = True
        previous = {"tcp": tcp, "joints": joints, "t_wall": row.get("t_wall")}

    stats["tcp_travel_m"] = travel if have_travel else None
    stats["joint_travel_rad"] = joint_travel if have_joint_travel else None
    stats["max_tcp_speed_m_s"] = peak_speed
    stats["max_tcp_speed_at_s"] = peak_speed_at
    stats["max_joint_speed_rad_s"] = peak_joint_speed
    stats["max_force_n"] = peak_force
    stats["max_torque_nm"] = peak_torque
    return stats


def _stat_cell(value, unit: str, digits: int = 4) -> str:
    number = _number(value)
    if number is None:
        return "未采集"
    return f"{number:.{digits}f} {unit}"


def count_rows_per_phase(rows: list[dict]) -> dict[str, int]:
    """How many sampled rows carry each phase tag."""
    counts: dict[str, int] = {}
    for row in rows:
        tag = str(row.get("phase") or "")
        counts[tag] = counts.get(tag, 0) + 1
    return counts


def render_summary(run: dict, stats: dict, phases: list[dict],
                   crop: dict | None = None,
                   phase_row_counts: dict[str, int] | None = None) -> str:
    """Chinese per-run summary: what ran, how long, key stats, phase table."""
    lines: list[str] = []
    lines.append(f"# 实验数据摘要 — {run['id']}")
    lines.append("")
    lines.append("## 1. 这次实验做了什么")
    lines.append("")
    lines.append(f"- 实验 id：`{run['id']}`")
    lines.append(f"- 世界/场景：`{run.get('world')}`（source={run.get('source')}）")
    lines.append(f"- 任务范围 scope：`{run.get('scope')}`")
    lines.append(f"- 运行模式 mode：`{run.get('mode')}`")
    lines.append(f"- 采样频率：{run.get('params', {}).get('rate_hz')} Hz，"
                 f"图像：{'开启' if run.get('params', {}).get('images') else '关闭'}")
    lines.append(f"- 结果：`{run.get('result')}`（结果码 {run.get('result_code')}）")
    commit = run.get("git_commit") or "未知"
    dirty = run.get("git_dirty")
    lines.append(f"- git commit：`{commit}`"
                 + ("（工作区有未提交改动）" if dirty else
                    "（工作区干净）" if dirty is False else ""))
    lines.append(f"- 图像数量：{run.get('images', 0)}")
    lines.append("")
    lines.append("## 2. 耗时")
    lines.append("")
    lines.append(f"- 开始：{iso_local(stats.get('t_start')) or run.get('started_at') or '未知'}")
    lines.append(f"- 结束：{iso_local(stats.get('t_end')) or run.get('ended_at') or '未知'}")
    if stats.get("duration_s") is not None:
        lines.append(f"- 总时长：{stats['duration_s']:.2f} s"
                     f"（{stats['rows']} 行）")
    else:
        lines.append("- 总时长：无有效样本")
    lines.append("")
    lines.append("## 3. 关键统计")
    lines.append("")
    lines.append("| 指标 | 数值 | 说明 |")
    lines.append("| --- | --- | --- |")
    lines.append(f"| 采样行数 | {stats.get('rows', 0)} | 固定频率采样点 |")
    lines.append(f"| TCP 行程 | {_stat_cell(stats.get('tcp_travel_m'), 'm')} | 相邻 TCP 位置差的累加 |")
    lines.append(f"| 最大 TCP 线速度 | {_stat_cell(stats.get('max_tcp_speed_m_s'), 'm/s')} | 位置差分估计 |")
    lines.append(f"| 关节行程 | {_stat_cell(stats.get('joint_travel_rad'), 'rad')} | 六维关节空间路径长度 |")
    lines.append(f"| 最大关节速度 | {_stat_cell(stats.get('max_joint_speed_rad_s'), 'rad/s')} | 来自 /joint_states.velocity |")
    lines.append(f"| 最大力 | {_stat_cell(stats.get('max_force_n'), 'N')} | ‖(fx,fy,fz)‖ 的最大值 |")
    lines.append(f"| 最大力矩 | {_stat_cell(stats.get('max_torque_nm'), 'N·m')} | ‖(tx,ty,tz)‖ 的最大值 |")
    lines.append("")
    lines.append("## 4. 每阶段时长")
    lines.append("")
    windows = phase_windows(phases, stats.get("t_end"))
    if not windows:
        lines.append("（本次没有记录到阶段变化；`phase` 列全程为空。）")
    else:
        lines.append("| # | 阶段标签 | 进入时刻 t_wall (s) | 时长 (s) | 行数 |")
        lines.append("| --- | --- | --- | --- | --- |")
        for index, window in enumerate(windows):
            duration = window["duration"]
            duration_text = f"{duration:.2f}" if duration is not None else "—"
            count = (phase_row_counts or {}).get(window["tag"], 0)
            lines.append(f"| {index + 1} | `{window['tag']}` | "
                         f"{window['start']:.3f} | {duration_text} | {count} |")
        lines.append("")
        total = sum(w["duration"] for w in windows if w["duration"] is not None)
        lines.append(f"阶段总时长：{total:.2f} s")
    if crop:
        lines.append("")
        lines.append("## 5. 裁剪与归档")
        lines.append("")
        lines.append(f"- 裁剪前 {crop.get('rows_before')} 行 → 裁剪后 "
                     f"{crop.get('rows_after')} 行（去掉 {crop.get('rows_dropped')} 行）")
        lines.append(f"- 首尾空闲：去掉头部 {crop.get('head_dropped')} 行、"
                     f"尾部 {crop.get('tail_dropped')} 行")
        lines.append(f"- 连续静止段：折叠 {crop.get('static_runs')} 段，"
                     f"去掉 {crop.get('static_dropped')} 行")
        lines.append(f"- 重复帧：去掉 {crop.get('duplicate_dropped')} 行")
        lines.append(f"- 图像：输入 {crop.get('images_before')} 张，保留 "
                     f"{crop.get('images_after')} 张")
        lines.append(f"- 归档路径：`{run.get('archive_dir')}`")
    lines.append("")
    lines.append("## 6. 已知限制")
    lines.append("")
    limitations: list[str] = []
    if not stats.get("wrench_samples"):
        limitations.append(
            "本次没有收到力/力矩数据（话题 `/cs625/tcp_force` 不存在，"
            "`/force_torque_sensor_broadcaster/wrench` 无数据；仿真机械臂没有 F/T 传感器），"
            "`fx..tz` 列为空，不应按 0 处理。")
    if not stats.get("tcp_samples"):
        limitations.append(
            "本次没有收到有效 TCP 位姿（`/tcp_pose_broadcaster/pose` 全为 NaN），"
            "TCP 行程/速度无法计算。")
    if not run.get("params", {}).get("images"):
        limitations.append("本次未开启图像采集（`--images` 关闭）。")
    if not limitations:
        limitations.append("无（本次所有预期话题都有数据）。")
    lines.extend(f"- {item}" for item in limitations)
    lines.append("")
    return "\n".join(lines)


def build_run_json(*, experiment_id: str, run_dir: pathlib.Path,
                   world: str, scope: str, mode: str, source: str,
                   started_at: float, ended_at: float | None,
                   rows: int, images: int, phases: int,
                   result: str, result_code: int, params: dict,
                   duration_s: float | None = None,
                   joint_names: list[str] | None = None,
                   topics: dict | None = None,
                   archive_dir: str | None = None,
                   extra: dict | None = None) -> dict:
    """The machine-readable receipt of one run (the set of fields is tested)."""
    commit, dirty = git_commit(REPO_ROOT)
    payload = {
        "id": experiment_id,
        "run_dir": str(run_dir),
        "started_at": iso_local(started_at),
        "ended_at": iso_local(ended_at),
        "duration_s": (round(float(duration_s), 3) if duration_s is not None
                       else (None if ended_at is None else
                             round(float(ended_at) - float(started_at), 3))),
        "world": world,
        "scope": scope,
        "mode": mode,
        "source": source,
        "git_commit": commit,
        "git_dirty": dirty,
        "params": params,
        "rows": rows,
        "images": images,
        "phases": phases,
        "result": result,
        "result_code": result_code,
        "joint_names": joint_names or [],
        "topics": topics or {},
        "archive_dir": archive_dir,
    }
    if extra:
        payload.update(extra)
    return payload


# ---------------------------------------------------------------------------
# image writing (numpy/cv2 are optional: the recorder still runs without them)
# ---------------------------------------------------------------------------
def save_image(path_stem: pathlib.Path, encoding: str, width: int, height: int,
               data: bytes) -> pathlib.Path | None:
    """Write one ROS image to PNG; ``None`` when the encoding is unsupported.

    Returns the path actually written (``.png``, or ``.ppm`` without OpenCV).
    """
    try:
        import cv2
        import numpy as np
    except ImportError:  # pragma: no cover - exercised only on a bare install
        return _save_ppm(path_stem, encoding, width, height, data)
    enc = (encoding or "").lower()
    array = np.frombuffer(data, dtype=np.uint8)
    try:
        if enc in ("bgr8", "rgb8", "8uc3"):
            image = array.reshape(height, width, 3)
            if enc == "rgb8":
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        elif enc in ("bgra8", "rgba8", "8uc4"):
            image = array.reshape(height, width, 4)
            image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        elif enc in ("mono8", "8uc1"):
            image = array.reshape(height, width)
        elif enc in ("mono16", "16uc1"):
            image = np.frombuffer(data, dtype=np.uint16).reshape(height, width)
        else:
            return None
    except ValueError:
        return None
    path = path_stem.with_suffix(".png")
    if not cv2.imwrite(str(path), image):
        return None
    return path


def _save_ppm(path_stem: pathlib.Path, encoding: str, width: int, height: int,
              data: bytes) -> pathlib.Path | None:
    """Fallback: binary PPM/PGM, so images survive without OpenCV."""
    enc = (encoding or "").lower()
    if enc in ("bgr8", "rgb8", "8uc3") and width * height * 3 == len(data):
        magic = b"P6"
    elif enc in ("mono8", "8uc1") and width * height == len(data):
        magic = b"P5"
    else:
        return None
    path = path_stem.with_suffix(".ppm")
    path.write_bytes(b"%s\n%d %d\n255\n" % (magic, width, height) + data)
    return path


# ---------------------------------------------------------------------------
# the recording session (source independent: ROS and synthetic share it)
# ---------------------------------------------------------------------------
class RecorderSession:
    """Owns the run directory, the CSV, phases.json and the image index."""

    def __init__(self, *, run_dir: pathlib.Path, experiment_id: str,
                 rate_hz: float, want_images: bool, image_period: float,
                 t_start: float) -> None:
        self.run_dir = pathlib.Path(run_dir)
        self.experiment_id = experiment_id
        self.rate_hz = float(rate_hz)
        self.want_images = bool(want_images)
        self.image_period = float(image_period)
        self.t_start = float(t_start)
        self.rows = 0
        self.images = 0
        self.phase = ""
        self.phase_entries: list[dict] = []
        self.image_entries: list[dict] = []
        self.joint_names: list[str] = []
        self.topics: dict = {}
        self.t_last_row: float | None = None
        self.t_first_row: float | None = None
        # The wall clock can step backwards (NTP/WSL time sync).  A CSV whose
        # t_wall goes backwards cannot be cropped or plotted, so the recorder
        # clamps it and counts the steps in run.json instead of hiding them.
        self._last_wall: float | None = None
        self._wall_offset = 0.0
        self.clock_backsteps = 0
        self._phase_chars_seen = 0
        self._csv_handle = None
        self._csv_writer = None
        self._last_image_ts = 0.0
        self._fields: dict = {
            "joints": [None] * JOINT_COUNT,
            "joint_vel": [None] * JOINT_COUNT,
            "joint_effort": [None] * JOINT_COUNT,
            "tcp": [None] * 7,
            "wrench": [None] * 6,
        }

    # -- setup --------------------------------------------------------------
    def open(self) -> None:
        (self.run_dir / "images").mkdir(parents=True, exist_ok=True)
        self._csv_handle = (self.run_dir / "timeseries.csv").open(
            "w", newline="", encoding="utf-8")
        self._csv_writer = csv.writer(self._csv_handle, lineterminator="\n")
        self._csv_writer.writerow(COLUMNS)
        self._csv_handle.flush()
        (self.run_dir / "phase.inbox").touch()
        self.write_phases()

    def close(self) -> None:
        if self._csv_handle is not None:
            self._csv_handle.flush()
            self._csv_handle.close()
            self._csv_handle = None

    # -- data in ------------------------------------------------------------
    def update(self, *, joints=None, joint_vel=None, joint_effort=None,
               tcp=None, wrench=None) -> None:
        """Replace the latest snapshot; the tick loop decides when to write."""
        if joints is not None:
            self._fields["joints"] = list(joints)[:JOINT_COUNT]
        if joint_vel is not None:
            self._fields["joint_vel"] = list(joint_vel)[:JOINT_COUNT]
        if joint_effort is not None:
            self._fields["joint_effort"] = list(joint_effort)[:JOINT_COUNT]
        if tcp is not None:
            self._fields["tcp"] = list(tcp)[:7]
        if wrench is not None:
            self._fields["wrench"] = list(wrench)[:6]

    def set_phase(self, tag: str, t_wall: float, t_ros: float | None) -> None:
        """Append a phase boundary (no file churn for a repeated tag)."""
        tag = str(tag or "").strip()
        if not tag or tag == self.phase:
            return
        validate_phase_tag(tag)
        t_wall = self.monotonic_wall(t_wall)
        self.phase = tag
        phase_name, _, boundary = tag.partition(":")
        self.phase_entries.append({
            "index": len(self.phase_entries),
            "tag": tag,
            "phase": phase_name,
            "boundary": boundary or None,
            "t_wall": round(float(t_wall), 6),
            "t_ros": None if t_ros is None else round(float(t_ros), 6),
            "row": self.rows,
        })
        self.write_phases()

    def write_phases(self) -> None:
        write_phases_json(self.run_dir / "phases.json", self.experiment_id,
                          self.phase_entries)

    # -- ticking ------------------------------------------------------------
    def monotonic_wall(self, candidate: float) -> float:
        """Never return a wall timestamp smaller than the previous one.

        A backwards step (NTP / WSL time sync) is repaired by shifting the rest
        of the run by a constant offset.  Shifting instead of clamping keeps the
        *intervals* intact, so speeds and stage durations stay correct.
        """
        candidate = float(candidate)
        if (self._last_wall is not None
                and candidate + self._wall_offset <= self._last_wall):
            self._wall_offset = self._last_wall - candidate + 1e-6
            self.clock_backsteps += 1
        value = candidate + self._wall_offset
        self._last_wall = value
        return value

    def tick(self, t_wall: float, t_ros: float | None) -> None:
        """Write exactly one CSV row from the latest snapshot."""
        t_wall = self.monotonic_wall(t_wall)
        if self.t_first_row is None:
            self.t_first_row = t_wall
        row: dict = {"t_wall": t_wall,
                     "t_ros": None if t_ros is None else float(t_ros)}
        for name, column in zip(("joints", "joint_vel", "joint_effort"), (
                "joint_pos", "joint_vel", "joint_effort")):
            for index in range(JOINT_COUNT):
                row[f"{column}_{index + 1}"] = self._fields[name][index]
        tcp = self._fields["tcp"]
        for key, value in zip(("tcp_x", "tcp_y", "tcp_z", "tcp_qx",
                               "tcp_qy", "tcp_qz", "tcp_qw"), tcp):
            row[key] = value
        for key, value in zip(("fx", "fy", "fz", "tx", "ty", "tz"),
                              self._fields["wrench"]):
            row[key] = value
        row["phase"] = self.phase
        self._csv_writer.writerow(format_row_values(row))
        self.rows += 1
        self.t_last_row = float(t_wall)
        if self.rows % 20 == 0:
            self._csv_handle.flush()

    def image_due(self, now: float) -> bool:
        """True when the image period has elapsed (camera runs at ~0.8 fps)."""
        return (self.want_images
                and (now - self._last_image_ts) >= self.image_period)

    def maybe_save_image(self, now: float, *, encoding: str, width: int,
                         height: int, data: bytes) -> pathlib.Path | None:
        """Save at most one image per ``image_period`` (camera is ~0.8 fps)."""
        if not self.image_due(now):
            return None
        phase_slug = sanitize_token(self.phase or "idle", default="idle")
        stem = (f"img_{self.images:05d}_"
                f"{time.strftime('%Y%m%d_%H%M%S', time.localtime(now))}_"
                f"{int(round((now % 1.0) * 1000)):03d}_{phase_slug}")
        written = save_image(self.run_dir / "images" / stem, encoding,
                             width, height, data)
        if written is None:
            return None
        self.images += 1
        self._last_image_ts = now
        self.image_entries.append({
            "seq": self.images - 1,
            "file": f"images/{written.name}",
            "t_wall": round(float(now), 6),
            "phase": self.phase,
            "width": int(width),
            "height": int(height),
            "encoding": encoding,
        })
        return written

    def drain_phase_inbox(self, now: float, t_ros: float | None) -> None:
        """Apply every phase tag written by ``set-phase`` since the last tick."""
        path = self.run_dir / "phase.inbox"
        if not path.exists():
            return
        text = path.read_text(encoding="utf-8", errors="replace")
        if len(text) <= self._phase_chars_seen:
            return
        chunk = text[self._phase_chars_seen:]
        self._phase_chars_seen = len(text)
        for line in chunk.splitlines():
            line = line.strip()
            if not line:
                continue
            try:
                self.set_phase(line, now, t_ros)
            except ValueError:
                continue


# ---------------------------------------------------------------------------
# ROS source (read-only subscriptions; imports rclpy lazily)
# ---------------------------------------------------------------------------
def _joint_order(names: list[str]) -> list[str]:
    """Map a joint-state name list onto the canonical chain order."""
    names = [str(n) for n in names]
    if all(name in names for name in CANONICAL_JOINT_ORDER):
        return list(CANONICAL_JOINT_ORDER)
    numbered = [n for n in names if re.search(r"\d+", n)]
    if len(numbered) >= JOINT_COUNT:
        return sorted(numbered)[:JOINT_COUNT]
    return sorted(names)[:JOINT_COUNT]


class _RosSource:
    """Subscriptions plus the rate-limited image pump."""

    def __init__(self, session: RecorderSession, *, use_sim_time: bool,
                 phase_topic: str | None, log) -> None:
        import rclpy
        from control_msgs.msg import JointTrajectoryControllerState
        from geometry_msgs.msg import PoseStamped, WrenchStamped
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image, JointState
        from std_msgs.msg import String

        self.session = session
        self.log = log
        self._rclpy = rclpy
        self._qos_sensor = qos_profile_sensor_data
        rclpy.init(args=None)
        self.node = Node("experiment_recorder")
        if use_sim_time:
            self.node.set_parameters([
                rclpy.parameter.Parameter("use_sim_time",
                                          rclpy.parameter.Parameter.Type.BOOL,
                                          True)])
        self._latest_image = None
        self._image_seq = 0
        self.phase_topic = None
        self._phase_sub = None
        self._counts = {TOPIC_JOINTS: 0, TOPIC_TCP: 0, TOPIC_CONTROLLER: 0,
                        TOPIC_CAMERA: 0}
        self.wrench_topic = None

        self.node.create_subscription(JointState, TOPIC_JOINTS,
                                      self._on_joints, 10)
        self.node.create_subscription(PoseStamped, TOPIC_TCP, self._on_tcp, 10)
        self.node.create_subscription(JointTrajectoryControllerState,
                                      TOPIC_CONTROLLER, self._on_controller, 10)
        if self.session.want_images:
            self.node.create_subscription(Image, TOPIC_CAMERA, self._on_image,
                                          self._qos_sensor)

        self._wrench_subs = {}
        for topic in WRENCH_CANDIDATES:
            # Subscribing to a missing topic is harmless; the first publisher
            # that shows up becomes the reported wrench source.
            try:
                self.node.create_subscription(
                    WrenchStamped, topic,
                    lambda msg, name=topic: self._on_wrench(name, msg), 10)
                self._wrench_subs[topic] = 0
            except Exception as exc:  # pragma: no cover - defensive
                self.log(f"wrench topic {topic} not subscribable: {exc}")

        self._resolve_phase_topic(phase_topic, String)
        self._spin_thread = None

    def _resolve_phase_topic(self, requested: str | None, string_type) -> None:
        candidates = []
        if requested:
            candidates.append(requested)
        elif TOPIC_PHASE_SPEC:
            candidates.append(TOPIC_PHASE_SPEC)
        for topic in candidates:
            # The graph is queried right after node creation; on a busy machine
            # discovery can take a moment, and a false "absent" would silently
            # downgrade the phase source.
            deadline = time.monotonic() + 2.0
            types: list[str] = []
            while time.monotonic() < deadline:
                types = dict(self.node.get_topic_names_and_types()).get(topic, [])
                if types:
                    break
                time.sleep(0.1)
            if any(t.endswith("std_msgs/msg/String") for t in types):
                self._phase_sub = self.node.create_subscription(
                    string_type, topic, self._on_phase, 10)
                self.phase_topic = topic
                return
            self.log(f"phase topic {topic} not used: type is {types or 'absent'}")

    # -- callbacks ----------------------------------------------------------
    def _on_joints(self, msg) -> None:
        self._counts[TOPIC_JOINTS] += 1
        order = _joint_order(list(msg.name))
        self.session.joint_names = order
        positions = self._pick(msg.name, msg.position, order)
        velocities = self._pick(msg.name, msg.velocity, order)
        efforts = self._pick(msg.name, msg.effort, order)
        self.session.update(joints=positions, joint_vel=velocities,
                            joint_effort=efforts)

    @staticmethod
    def _pick(names, values, order):
        values = list(values or [])
        table = dict(zip(names, values))
        return [float(table[name]) if name in table else None for name in order]

    def _on_tcp(self, msg) -> None:
        self._counts[TOPIC_TCP] += 1
        pose = msg.pose
        self.session.update(tcp=[pose.position.x, pose.position.y, pose.position.z,
                                 pose.orientation.x, pose.orientation.y,
                                 pose.orientation.z, pose.orientation.w])

    def _on_controller(self, msg) -> None:  # liveness only
        self._counts[TOPIC_CONTROLLER] += 1

    def _on_wrench(self, topic: str, msg) -> None:
        self._wrench_subs[topic] = self._wrench_subs.get(topic, 0) + 1
        if self.wrench_topic is None:
            self.wrench_topic = topic
        if topic != self.wrench_topic:
            return
        wrench = msg.wrench
        self.session.update(wrench=[wrench.force.x, wrench.force.y, wrench.force.z,
                                    wrench.torque.x, wrench.torque.y, wrench.torque.z])

    def _on_image(self, msg) -> None:
        self._counts[TOPIC_CAMERA] += 1
        self._latest_image = (msg.encoding, int(msg.width), int(msg.height),
                              bytes(msg.data))
        self._image_seq += 1

    def _on_phase(self, msg) -> None:
        try:
            self.session.set_phase(str(msg.data), time.time(),
                                   self.ros_time())
        except ValueError:
            pass

    # -- loop ---------------------------------------------------------------
    def sample(self, t_wall: float) -> None:
        """Nothing to do here: the subscriptions keep the snapshot fresh."""
        return None

    def start_spin(self) -> None:
        import threading
        self._spin_thread = threading.Thread(
            target=self._rclpy.spin, args=(self.node,), daemon=True)
        self._spin_thread.start()

    def ros_time(self) -> float | None:
        try:
            now = self.node.get_clock().now().to_msg()
            return float(now.sec) + float(now.nanosec) * 1e-9
        except Exception:  # pragma: no cover - defensive
            return None

    def pump_image(self, now: float) -> None:
        if self._latest_image is None or not self.session.image_due(now):
            return  # keep the frame; the next tick may be allowed to save it
        encoding, width, height, data = self._latest_image
        self.session.maybe_save_image(now, encoding=encoding, width=width,
                                      height=height, data=data)
        self._latest_image = None  # never save the same frame twice

    def topics_report(self) -> dict:
        report: dict = {}
        for topic, count in self._counts.items():
            try:
                publishers = self.node.count_publishers(topic)
            except Exception:  # pragma: no cover - defensive
                publishers = None
            report[topic] = {"received": count, "publishers": publishers}
        for topic, count in self._wrench_subs.items():
            report[topic] = {"received": count,
                             "publishers": self.node.count_publishers(topic),
                             "selected": topic == self.wrench_topic}
        report["phase_source"] = (f"topic:{self.phase_topic}"
                                  if self.phase_topic else "external:set-phase")
        return report

    def close(self) -> None:
        try:
            self.node.destroy_node()
        except Exception:  # pragma: no cover - defensive
            pass
        try:
            self._rclpy.shutdown()
        except Exception:  # pragma: no cover - defensive
            pass


# ---------------------------------------------------------------------------
# synthetic source: a deterministic 20 Hz stand-in, no ROS involved
# ---------------------------------------------------------------------------
class _SyntheticSource:
    """Deterministic stand-in used to self-test the whole pipeline.

    The joints swing with two different periods, the TCP follows a small arc and
    the wrench carries a load step, so the summary statistics are non-trivial.
    """

    PHASE_PLAN = ((0.0, "idle"), (1.0, "pre_grasp:enter"), (5.0, "grasp:enter"),
                  (9.0, "lift:enter"), (13.0, "transit:enter"))

    def __init__(self, session: RecorderSession, *, t_start: float) -> None:
        self.session = session
        self.t_start = float(t_start)
        self._next_phase = 1

    def sample(self, t_wall: float) -> None:
        t = float(t_wall) - self.t_start
        joints = [0.20 * math.sin(0.8 * t + i * 0.5) for i in range(JOINT_COUNT)]
        velocities = [0.16 * math.cos(0.8 * t + i * 0.5) for i in range(JOINT_COUNT)]
        efforts = [0.5 * math.sin(0.8 * t + i * 0.5) + 0.1 * i for i in range(JOINT_COUNT)]
        tcp = [0.45 + 0.12 * math.sin(0.6 * t),
               0.05 + 0.10 * math.sin(0.4 * t + 0.7),
               0.30 + 0.06 * math.cos(0.5 * t),
               0.0, 0.0, math.sin(0.25 * t / 2), math.cos(0.25 * t / 2)]
        wrench = [0.5 * math.sin(1.1 * t), 0.3 * math.cos(0.9 * t),
                  -6.0 + 0.4 * math.sin(2.0 * t), 0.02 * math.sin(t),
                  0.03 * math.cos(1.3 * t), 0.01 * math.sin(0.7 * t)]
        self.session.update(joints=joints, joint_vel=velocities,
                            joint_effort=efforts, tcp=tcp, wrench=wrench)
        while (self._next_phase < len(self.PHASE_PLAN)
               and t >= self.PHASE_PLAN[self._next_phase][0]):
            tag = self.PHASE_PLAN[self._next_phase][1]
            self._next_phase += 1
            if tag != "idle":
                self.session.set_phase(tag, t_wall, t)

    def pump_image(self, now: float) -> None:
        if not self.session.image_due(now):
            return
        try:
            import numpy as np
        except ImportError:  # pragma: no cover
            return
        width, height = 320, 240
        phase_slug = self.session.phase or "idle"
        level = (sum(ord(c) for c in phase_slug) % 200) + 30
        image = np.zeros((height, width, 3), dtype=np.uint8)
        image[:, :, 0] = level
        image[:, :, 1] = np.linspace(0, 255, width, dtype=np.uint8)[None, :]
        image[:, :, 2] = np.linspace(255, 0, height, dtype=np.uint8)[:, None]
        self.session.maybe_save_image(now, encoding="bgr8", width=width,
                                      height=height, data=image.tobytes())

    def topics_report(self) -> dict:
        return {"source": "synthetic", "phase_source": "synthetic:PHASE_PLAN"}

    def close(self) -> None:
        return None


# ---------------------------------------------------------------------------
# worker entry point (runs in a detached process started by ``start``)
# ---------------------------------------------------------------------------
def _write_state(payload: dict) -> None:
    STATE_DIR.mkdir(parents=True, exist_ok=True)
    tmp = STATEFILE.with_name(STATEFILE.name + ".tmp")
    tmp.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    os.replace(tmp, STATEFILE)


def _read_state() -> dict | None:
    try:
        return json.loads(STATEFILE.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


def _pid_alive(pid: int) -> bool:
    if not pid:
        return False
    try:
        os.kill(int(pid), 0)
    except (OSError, ValueError):
        return False
    return True


def _log_to(run_dir: pathlib.Path, message: str) -> None:
    line = f"[{time.strftime('%H:%M:%S')}] {message}"
    print(line, flush=True)
    try:
        with (run_dir / "recorder.log").open("a", encoding="utf-8") as handle:
            handle.write(line + "\n")
    except OSError:
        pass


def worker_main(args) -> int:
    run_dir = pathlib.Path(args.run_dir).expanduser().resolve()
    run_dir.mkdir(parents=True, exist_ok=True)
    started = time.time()
    started_mono = time.monotonic()
    session = RecorderSession(
        run_dir=run_dir, experiment_id=args.experiment_id,
        rate_hz=args.rate, want_images=args.images,
        image_period=args.image_period, t_start=started)
    session.open()

    stop = {"flag": False}

    def _handle(signum, frame):  # noqa: ARG001 - signal API
        stop["flag"] = True

    try:
        signal.signal(signal.SIGTERM, _handle)
        signal.signal(signal.SIGINT, _handle)
    except ValueError:  # pragma: no cover - not the main thread
        pass

    source = None
    ros_error = None
    if args.source == "ros":
        try:
            source = _RosSource(session, use_sim_time=args.use_sim_time,
                                phase_topic=args.phase_topic,
                                log=lambda m: _log_to(run_dir, m))
            source.start_spin()
        except Exception as exc:  # rclpy missing, DDS failure, ...
            ros_error = f"{type(exc).__name__}: {exc}"
            _log_to(run_dir, f"ROS source unavailable: {ros_error}")
    if source is None and args.source == "ros":
        session.close()
        _finalize(run_dir, session, args, started, result="error",
                  result_code=EXIT_NO_ROS, extra={"error": ros_error},
                  source_name="ros", topics={})
        return EXIT_NO_ROS
    if source is None:
        source = _SyntheticSource(session, t_start=started)

    _write_state({
        "id": args.experiment_id,
        "pid": os.getpid(),
        "run_dir": str(run_dir),
        "started_at": iso_local(started),
        "world": args.world,
        "scope": args.scope,
        "mode": args.mode,
        "source": args.source,
        "rate_hz": args.rate,
        "pidfile": str(PIDFILE),
    })
    try:
        PIDFILE.write_text(str(os.getpid()) + "\n", encoding="utf-8")
    except OSError as exc:
        _log_to(run_dir, f"cannot write pidfile: {exc}")

    period = 1.0 / max(1e-3, float(args.rate))
    next_tick = time.monotonic()
    result = "ok"
    result_code = EXIT_OK
    _log_to(run_dir, f"recording {args.experiment_id} at {args.rate} Hz "
                     f"(source={args.source}, images={args.images})")
    try:
        while not stop["flag"]:
            now = time.time()
            if (run_dir / ".stop").exists():
                result = "ok"
                break
            if args.duration and (now - started) >= float(args.duration):
                result = "duration_reached"
                break
            source.sample(now)
            session.drain_phase_inbox(now, source.ros_time() if hasattr(source, "ros_time") else None)
            t_ros = source.ros_time() if hasattr(source, "ros_time") else (now - started)
            session.tick(now, t_ros)
            source.pump_image(now)
            next_tick += period
            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:  # fell behind: resynchronise instead of spinning
                next_tick = time.monotonic()
        if stop["flag"]:
            result = "signal"
    except Exception as exc:  # pragma: no cover - keep the files we have
        result = "error"
        result_code = 1
        _log_to(run_dir, f"recorder error: {type(exc).__name__}: {exc}")
        extra_error = f"{type(exc).__name__}: {exc}"
    else:
        extra_error = None

    topics: dict = {}
    try:
        if hasattr(source, "topics_report"):
            topics = source.topics_report()
    except Exception:  # pragma: no cover - defensive
        topics = {}
    session.close()
    try:
        source.close()
    except Exception:  # pragma: no cover - defensive
        pass
    _finalize(run_dir, session, args, started, result=result,
              result_code=result_code,
              extra={"error": extra_error} if extra_error else None,
              source_name=args.source, topics=topics,
              duration_s=time.monotonic() - started_mono)
    return result_code


def _finalize(run_dir: pathlib.Path, session: RecorderSession, args,
              started: float, *, result: str, result_code: int,
              extra: dict | None = None, source_name: str,
              topics: dict | None = None,
              duration_s: float | None = None) -> None:
    ended = time.time()
    rows = session.rows
    if rows >= 2 and session.t_first_row is not None and session.t_last_row is not None:
        # Duration of the recorded data, not of process start-up: the summary
        # and the CSV must agree about how long the experiment ran.
        duration_s = session.t_last_row - session.t_first_row
    data_rows = read_back(run_dir)
    stats = compute_stats(data_rows)
    params = {
        "rate_hz": args.rate,
        "images": bool(args.images),
        "image_period_s": args.image_period,
        "duration_s": args.duration,
        "use_sim_time": bool(getattr(args, "use_sim_time", False)),
        "phase_topic": getattr(args, "phase_topic", None),
        "argv": sys.argv[1:],
    }
    topics = dict(topics or {})
    extra = dict(extra or {})
    extra["clock_backsteps"] = session.clock_backsteps
    payload = build_run_json(
        experiment_id=args.experiment_id, run_dir=run_dir,
        world=args.world, scope=args.scope, mode=args.mode,
        source=source_name, started_at=started, ended_at=ended,
        rows=rows, images=session.images, phases=len(session.phase_entries),
        result=result, result_code=result_code, params=params,
        duration_s=duration_s,
        joint_names=session.joint_names, topics=topics, extra=extra)
    if session.image_entries:
        payload["images_index"] = "images.jsonl"
    (run_dir / "run.json").write_text(
        json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8")
    if session.image_entries:
        with (run_dir / "images.jsonl").open("w", encoding="utf-8") as handle:
            for entry in session.image_entries:
                handle.write(json.dumps(entry, ensure_ascii=False) + "\n")
    summary = render_summary(payload, stats, session.phase_entries,
                             phase_row_counts=count_rows_per_phase(data_rows))
    (run_dir / "summary.md").write_text(summary, encoding="utf-8")
    (run_dir / "exit_code").write_text(str(result_code) + "\n", encoding="utf-8")
    try:
        (run_dir / ".stop").unlink(missing_ok=True)
    except OSError:
        pass
    if _read_state() and int(_read_state().get("pid") or 0) == os.getpid():
        for path in (PIDFILE, STATEFILE):
            try:
                path.unlink(missing_ok=True)
            except OSError:
                pass
    _log_to(run_dir, f"finalised: rows={rows} images={session.images} "
                     f"result={result} code={result_code}")


def read_back(run_dir: pathlib.Path) -> list[dict]:
    """Rows currently on disk (used for the summary at finalize time)."""
    path = pathlib.Path(run_dir) / "timeseries.csv"
    try:
        return read_timeseries(path)
    except (OSError, ValueError):
        return []


# ---------------------------------------------------------------------------
# CLI: start / stop / status / set-phase / _worker
# ---------------------------------------------------------------------------
def _tail(path: pathlib.Path, lines: int = 12) -> str:
    try:
        text = path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        return ""
    return "\n".join(text[-lines:])


def cmd_start(args) -> int:
    STATE_DIR.mkdir(parents=True, exist_ok=True)
    state = _read_state()
    if state and _pid_alive(int(state.get("pid") or 0)):
        print(f"[experiment_recorder] already running: id={state.get('id')} "
              f"pid={state.get('pid')}\n  run_dir={state.get('run_dir')}")
        return EXIT_OK
    if state:
        print(f"[experiment_recorder] clearing stale state for "
              f"id={state.get('id')} pid={state.get('pid')}")
        for path in (PIDFILE, STATEFILE):
            try:
                path.unlink(missing_ok=True)
            except OSError:
                pass

    experiment_id = make_experiment_id(time.time(), args.world, args.scope, args.mode)
    out_root = pathlib.Path(args.out).expanduser().resolve()
    out_root.mkdir(parents=True, exist_ok=True)
    run_dir = out_root / experiment_id
    suffix = 1
    while run_dir.exists():
        suffix += 1
        run_dir = out_root / f"{experiment_id}_r{suffix}"
    run_dir.mkdir(parents=True)

    command = [
        sys.executable, "-u", str(pathlib.Path(__file__).resolve()), "_worker",
        "--run-dir", str(run_dir), "--id", run_dir.name,
        "--world", args.world, "--scope", args.scope, "--mode", args.mode,
        "--rate", str(args.rate), "--image-period", str(args.image_period),
        "--source", args.source,
    ]
    if args.images:
        command.append("--images")
    if args.duration:
        command += ["--duration", str(args.duration)]
    if args.use_sim_time:
        command.append("--use-sim-time")
    if args.phase_topic:
        command += ["--phase-topic", args.phase_topic]

    log_path = run_dir / "recorder.log"
    with log_path.open("w", encoding="utf-8") as log_handle:
        process = subprocess.Popen(command, stdout=log_handle,
                                   stderr=subprocess.STDOUT,
                                   start_new_session=True, cwd=str(REPO_ROOT))
    deadline = time.monotonic() + 20.0
    state = None
    while time.monotonic() < deadline:
        state = _read_state()
        if state and state.get("pid") == process.pid:
            break
        if process.poll() is not None:
            state = None
            break
        time.sleep(0.1)
    if not state or state.get("pid") != process.pid:
        print(f"[experiment_recorder] start FAILED for {run_dir.name}; "
              f"child pid={process.pid} exited={process.poll()}", file=sys.stderr)
        print(_tail(log_path), file=sys.stderr)
        if process.poll() is None:
            process.terminate()
        # Propagate "ROS unavailable" so the caller can tell a missing DDS
        # environment from a recorder that started and then died.
        if process.poll() == EXIT_NO_ROS:
            return EXIT_NO_ROS
        return EXIT_START_FAILED
    print(f"[experiment_recorder] started id={run_dir.name} pid={process.pid}")
    print(f"  run_dir={run_dir}")
    print(f"  pidfile={PIDFILE}")
    print(f"  source={args.source} rate={args.rate}Hz images={bool(args.images)}")
    if args.images and args.image_period < 1.0:
        print("  warning: camera runs at ~0.8 fps; --image-period below 1 s "
              "will skip frames instead of speeding the camera up")
    return EXIT_OK


def _resolve_run_dir(args) -> pathlib.Path | None:
    if getattr(args, "run_dir", None):
        return pathlib.Path(args.run_dir).expanduser().resolve()
    state = _read_state()
    if state and state.get("run_dir"):
        return pathlib.Path(state["run_dir"]).expanduser().resolve()
    return None


def _latest_finalized_run() -> pathlib.Path | None:
    """Newest run directory that already has an ``exit_code`` (for diagnostics)."""
    try:
        candidates = [p for p in pathlib.Path(DEFAULT_OUT).glob("*")
                      if (p / "exit_code").exists()]
    except OSError:
        return None
    if not candidates:
        return None
    return max(candidates, key=lambda p: (p / "exit_code").stat().st_mtime)


def cmd_stop(args) -> int:
    run_dir = _resolve_run_dir(args)
    state = _read_state()
    if run_dir is None:
        finished = _latest_finalized_run()
        if finished is not None:
            # A recorder started with --duration finalises itself and removes the
            # pidfile; stop must still be safe and explain what it found.
            print(f"[experiment_recorder] nothing running; newest finalised run: "
                  f"{finished.name} (exit_code="
                  f"{(finished / 'exit_code').read_text().strip()})")
            print(f"  run_dir={finished}")
            return EXIT_OK
        print("[experiment_recorder] nothing to stop (no pidfile/state)")
        return EXIT_OK
    exit_file = run_dir / "exit_code"
    if exit_file.exists():
        code = int(exit_file.read_text().strip() or 0)
        print(f"[experiment_recorder] already finalized: {run_dir.name} "
              f"exit_code={code}")
        print(f"  run_dir={run_dir}")
        _cleanup_state(state)
        return EXIT_OK
    pid = int(state.get("pid") or 0) if state else 0
    if pid and not _pid_alive(pid):
        print(f"[experiment_recorder] recorder pid={pid} is gone and "
              f"{run_dir.name} has no exit_code; inspect {run_dir}/recorder.log",
              file=sys.stderr)
        _cleanup_state(state)
        return EXIT_START_FAILED
    (run_dir / ".stop").write_text("stop\n", encoding="utf-8")
    deadline = time.monotonic() + float(args.timeout)
    while time.monotonic() < deadline:
        if exit_file.exists():
            break
        time.sleep(0.1)
    if not exit_file.exists():
        print(f"[experiment_recorder] stop TIMEOUT after {args.timeout}s "
              f"(pid={pid}); tail of recorder.log:", file=sys.stderr)
        print(_tail(run_dir / "recorder.log"), file=sys.stderr)
        return EXIT_STOP_TIMEOUT
    code = int(exit_file.read_text().strip() or 0)
    run_json = {}
    try:
        run_json = json.loads((run_dir / "run.json").read_text(encoding="utf-8"))
    except (OSError, ValueError):
        pass
    print(f"[experiment_recorder] stopped id={run_dir.name} exit_code={code} "
          f"rows={run_json.get('rows')} images={run_json.get('images')}")
    print(f"  run_dir={run_dir}")
    print(f"  summary={run_dir / 'summary.md'}")
    _cleanup_state(state)
    return code


def _cleanup_state(state: dict | None) -> None:
    pid = int((state or {}).get("pid") or 0)
    if pid and _pid_alive(pid):
        return
    for path in (PIDFILE, STATEFILE):
        try:
            path.unlink(missing_ok=True)
        except OSError:
            pass


def cmd_status(args) -> int:
    state = _read_state()
    if not state:
        print("[experiment_recorder] no recorder running (no state file)")
        return EXIT_NOT_RUNNING
    alive = _pid_alive(int(state.get("pid") or 0))
    run_dir = pathlib.Path(state.get("run_dir") or "")
    rows = 0
    csv_path = run_dir / "timeseries.csv"
    if csv_path.exists():
        with csv_path.open("rb") as handle:
            rows = max(0, sum(1 for _ in handle) - 1)
    print(json.dumps({**state, "alive": alive, "rows": rows}, indent=2))
    return EXIT_OK if alive else EXIT_NOT_RUNNING


def cmd_set_phase(args) -> int:
    try:
        tag = validate_phase_tag(args.tag)
    except ValueError as exc:
        print(f"[experiment_recorder] {exc}", file=sys.stderr)
        return EXIT_BAD_ARGS
    run_dir = _resolve_run_dir(args)
    if run_dir is None or not (run_dir / "phase.inbox").exists():
        print(f"[experiment_recorder] no active recorder for tag {tag!r}",
              file=sys.stderr)
        return EXIT_NOT_RUNNING
    with (run_dir / "phase.inbox").open("a", encoding="utf-8") as handle:
        handle.write(tag + "\n")
        handle.flush()
        os.fsync(handle.fileno())
    print(f"[experiment_recorder] phase {tag} -> {run_dir.name}")
    return EXIT_OK


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Record experiment timeseries + phases (read-only).")
    sub = parser.add_subparsers(dest="command", required=True)

    def add_common(p):
        p.add_argument("--out", default=str(DEFAULT_OUT),
                       help=f"run directory root (default {DEFAULT_OUT})")
        p.add_argument("--world", choices=WORLDS, default="sim")
        p.add_argument("--scope", default="pick_and_place")
        p.add_argument("--mode", default="default")
        p.add_argument("--rate", type=float, default=20.0,
                       help="sampling rate in Hz (default 20)")
        p.add_argument("--images", action="store_true",
                       help="also save camera frames into images/")
        p.add_argument("--image-period", type=float, default=2.0,
                       help="seconds between saved images (default 2.0)")
        p.add_argument("--duration", type=float, default=None,
                       help="stop automatically after N seconds")
        p.add_argument("--source", choices=SOURCES, default="ros",
                       help="ros (real topics) or synthetic (self-test)")
        p.add_argument("--use-sim-time", action="store_true",
                       help="use the ROS simulation clock for t_ros")
        p.add_argument("--phase-topic", default=None,
                       help="std_msgs/String topic carrying phase tags")

    start = sub.add_parser("start", help="start a background recorder")
    add_common(start)
    start.set_defaults(func=cmd_start)

    stop = sub.add_parser("stop", help="stop the recorder and finalise files")
    stop.add_argument("--run-dir", default=None,
                      help="run directory to stop (default: from state file)")
    stop.add_argument("--timeout", type=float, default=45.0)
    stop.set_defaults(func=cmd_stop)

    status = sub.add_parser("status", help="show the running recorder")
    status.set_defaults(func=cmd_status)

    set_phase = sub.add_parser("set-phase", help="tag a phase boundary")
    set_phase.add_argument("--tag", required=True,
                           help="e.g. grasp:enter or observe:left")
    set_phase.add_argument("--run-dir", default=None)
    set_phase.set_defaults(func=cmd_set_phase)

    worker = sub.add_parser("_worker", help=argparse.SUPPRESS)
    worker.add_argument("--run-dir", required=True)
    worker.add_argument("--id", dest="experiment_id", required=True)
    worker.add_argument("--world", choices=WORLDS, required=True)
    worker.add_argument("--scope", required=True)
    worker.add_argument("--mode", required=True)
    worker.add_argument("--rate", type=float, default=20.0)
    worker.add_argument("--image-period", type=float, default=2.0)
    worker.add_argument("--duration", type=float, default=None)
    worker.add_argument("--source", choices=SOURCES, default="ros")
    worker.add_argument("--images", action="store_true")
    worker.add_argument("--use-sim-time", action="store_true")
    worker.add_argument("--phase-topic", default=None)
    worker.set_defaults(func=worker_main)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return int(args.func(args))
    except KeyboardInterrupt:
        return EXIT_OK


if __name__ == "__main__":
    raise SystemExit(main())
