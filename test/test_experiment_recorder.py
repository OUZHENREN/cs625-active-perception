"""Unit tests for the experiment recorder and the archive cropper.

Only pure functions are covered: no ROS, no camera, no robot.  The two things
that can be wrong silently are (a) the CSV/run.json schema drifting away from
what downstream readers and the archive expect and (b) the cropper eating the
useful part of a run or keeping the idle part.  Both are pinned here.
"""

from __future__ import annotations

import json
import pathlib
import pathlib
import sys

import pytest

REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "scripts"))

import archive_experiment as ax  # noqa: E402
import experiment_recorder as er  # noqa: E402


# ---------------------------------------------------------------------------
# helpers used to build synthetic runs
# ---------------------------------------------------------------------------
def make_row(t: float, *, q0: float = 0.0, x: float = 0.30,
             phase: str = "", fx: float | None = None) -> dict:
    row = {"t_wall": t, "t_ros": t - 1_790_000_000.0, "phase": phase}
    for index in range(1, 7):
        row[f"joint_pos_{index}"] = q0 + 0.001 * index
        row[f"joint_vel_{index}"] = 0.0
        row[f"joint_effort_{index}"] = 0.0
    row["tcp_x"], row["tcp_y"], row["tcp_z"] = x, 0.0, 0.4
    row["tcp_qx"], row["tcp_qy"], row["tcp_qz"], row["tcp_qw"] = 0.0, 0.0, 0.0, 1.0
    if fx is None:  # no F/T data at all -> every wrench column stays empty
        row["fx"] = row["fy"] = row["fz"] = None
        row["tx"] = row["ty"] = row["tz"] = None
    else:
        row["fx"], row["fy"], row["fz"] = fx, 0.0, 0.0
        row["tx"], row["ty"], row["tz"] = 0.0, 0.0, 0.0
    return row


def still(rows: list[dict], t0: float, t1: float, *, step: float = 0.05) -> None:
    """Append identical rows from t0 to t1 inclusive (a frozen robot)."""
    base = rows[-1] if rows else make_row(t0)
    count = int(round((t1 - t0) / step)) + 1
    for index in range(count):
        row = dict(base)
        row["t_wall"] = t0 + index * step
        rows.append(row)


def moving(rows: list[dict], t0: float, t1: float, *, step: float = 0.05,
           speed: float = 0.4, phase: str = "grasp:enter") -> None:
    """Append rows whose payload changes every step (a moving robot)."""
    count = int(round((t1 - t0) / step)) + 1
    for index in range(count):
        t = t0 + index * step
        rows.append(make_row(t, q0=0.3 * (index * step), x=0.3 + speed * index * step,
                             phase=phase))


# ---------------------------------------------------------------------------
# schema
# ---------------------------------------------------------------------------
def test_columns_are_exactly_the_required_schema() -> None:
    expected = (
        ["t_wall", "t_ros"]
        + [f"joint_pos_{i}" for i in range(1, 7)]
        + [f"joint_vel_{i}" for i in range(1, 7)]
        + [f"joint_effort_{i}" for i in range(1, 7)]
        + ["tcp_x", "tcp_y", "tcp_z", "tcp_qx", "tcp_qy", "tcp_qz", "tcp_qw"]
        + ["fx", "fy", "fz", "tx", "ty", "tz", "phase"]
    )
    assert er.COLUMNS == expected
    assert len(er.COLUMNS) == len(set(er.COLUMNS)) == 34


def test_csv_roundtrip_keeps_columns_and_empty_cells(tmp_path) -> None:
    rows = [make_row(100.0, fx=None), make_row(100.05, fx=1.5)]
    rows[1]["tcp_x"] = None  # "not available" must not become 0
    path = tmp_path / "timeseries.csv"
    er.write_timeseries(path, rows)

    header = path.read_text().splitlines()[0].split(",")
    assert header == er.COLUMNS

    loaded = er.read_timeseries(path)
    assert len(loaded) == 2
    assert loaded[0]["fx"] is None
    assert loaded[1]["fx"] == pytest.approx(1.5)
    assert loaded[1]["tcp_x"] is None
    assert loaded[0]["phase"] == ""
    assert loaded[0]["t_wall"] == pytest.approx(100.0)


def test_csv_keeps_phase_tags_as_text(tmp_path) -> None:
    """Regression: the phase column is text and must not be passed to float()."""
    rows = [make_row(1.0, phase="grasp:enter"), make_row(1.05, phase="lift:enter")]
    path = tmp_path / "timeseries.csv"
    er.write_timeseries(path, rows)
    loaded = er.read_timeseries(path)
    assert [row["phase"] for row in loaded] == ["grasp:enter", "lift:enter"]
    assert er.format_row_values(rows[0])[er.COLUMNS.index("phase")] == "grasp:enter"


def test_experiment_id_format_and_phase_tag_validation() -> None:
    now = 1_790_000_000.0
    experiment_id = er.make_experiment_id(now, "sim", "pick_and_place", "efficient")
    assert experiment_id == time_format(now) + "_sim_pick_and_place_efficient"
    # characters that would break a path are collapsed
    assert er.make_experiment_id(now, "real", "pick place", "a/b") == \
        time_format(now) + "_real_pick_place_a_b"
    assert er.validate_phase_tag("grasp:enter") == "grasp:enter"
    for bad in ("", "  ", "-broken", "a" * 80):
        with pytest.raises(ValueError):
            er.validate_phase_tag(bad)


def time_format(ts: float) -> str:
    import time
    return time.strftime("%Y%m%d_%H%M%S", time.localtime(ts))


def test_run_json_has_the_required_fields(tmp_path) -> None:
    payload = er.build_run_json(
        experiment_id="20261024_202700_sim_pick_and_place_efficient",
        run_dir=tmp_path, world="sim", scope="pick_and_place",
        mode="efficient", source="synthetic", started_at=1_790_000_000.0,
        ended_at=1_790_000_015.0, rows=300, images=7, phases=4,
        result="ok", result_code=0,
        params={"rate_hz": 20.0, "images": True},
        joint_names=list(er.CANONICAL_JOINT_ORDER),
        topics={"/joint_states": {"received": 300}})
    for key in ("id", "started_at", "ended_at", "duration_s", "world", "scope",
                "mode", "source", "git_commit", "params", "rows", "images",
                "phases", "result", "result_code", "joint_names", "topics"):
        assert key in payload, key
    assert payload["rows"] == 300
    assert payload["images"] == 7
    assert payload["result_code"] == 0
    assert payload["duration_s"] == pytest.approx(15.0)
    assert payload["params"]["rate_hz"] == 20.0
    # the file must be JSON-serialisable (it is written with json.dumps)
    json.dumps(payload)


# ---------------------------------------------------------------------------
# cropping
# ---------------------------------------------------------------------------
def build_ten_second_still_run() -> list[dict]:
    """The acceptance scenario: 10 s still + motion + 8 s still + 3 repeats.

    The three repeated frames sit *inside* the motion on purpose: a repeat at the
    very end of the motion is indistinguishable from the trailing idle and would
    be removed by the edge trim (which is also correct behaviour).
    """
    rows: list[dict] = []
    still(rows, 0.0, 10.0)                          # opening idle
    moving(rows, 10.05, 12.0)                       # first half of the motion
    frozen = dict(rows[-1])                         # three identical frames
    for index in range(3):
        duplicate = dict(frozen)
        duplicate["t_wall"] = 12.05 + 0.05 * index
        rows.append(duplicate)
    moving(rows, 12.2, 14.0)                        # motion resumes
    still(rows, 16.0, 24.0)                         # trailing idle
    return rows


def test_trim_idle_edges_removes_head_and_tail() -> None:
    rows = build_ten_second_still_run()
    kept, report = ax.trim_idle_edges(rows, pad_s=0.0, eps=1e-4)
    assert report["had_motion"] is True
    assert report["head_dropped"] > 150
    assert report["tail_dropped"] > 150
    assert kept[0]["t_wall"] == pytest.approx(10.05)
    assert kept[-1]["t_wall"] == pytest.approx(14.0)
    assert report["first_activity_s"] == pytest.approx(10.05)


def test_trim_idle_edges_keeps_a_frozen_run_untouched() -> None:
    rows: list[dict] = []
    still(rows, 0.0, 5.0)
    kept, report = ax.trim_idle_edges(rows, pad_s=0.0)
    assert report["had_motion"] is False
    assert len(kept) == len(rows)


def test_collapse_static_runs_keeps_first_and_last_sample() -> None:
    rows: list[dict] = []
    moving(rows, 0.0, 1.0)
    still(rows, 1.05, 6.0)      # 5 s dwell, > 3 s
    moving(rows, 6.1, 7.0)
    kept, report = ax.collapse_static_runs(rows, max_still_s=3.0)
    assert report["static_runs"] == 1
    assert report["static_dropped"] > 90
    dwell = [r for r in kept if 1.0 <= r["t_wall"] <= 6.05]
    assert [round(r["t_wall"], 2) for r in dwell] == [1.0, 6.0]

    # A short dwell is information (e.g. a settle), so it stays complete.
    short: list[dict] = []
    moving(short, 0.0, 1.0)
    still(short, 1.05, 2.5)
    kept_short, report_short = ax.collapse_static_runs(short, max_still_s=3.0)
    assert report_short["static_runs"] == 0
    assert len(kept_short) == len(short)


def test_drop_duplicate_rows_removes_repeats_only() -> None:
    rows = build_ten_second_still_run()
    kept, report = ax.drop_duplicate_rows(rows, eps=1e-4)
    assert report["duplicate_dropped"] >= 3
    # a moving sequence must not lose anything
    moving_rows: list[dict] = []
    moving(moving_rows, 0.0, 2.0)
    kept_moving, report_moving = ax.drop_duplicate_rows(moving_rows, eps=1e-4)
    assert report_moving["duplicate_dropped"] == 0
    assert len(kept_moving) == len(moving_rows)


def test_drop_duplicate_rows_thins_a_short_hold_instead_of_deleting_it() -> None:
    """A 2 s standstill is data: keep ~one row per duplicate window, not one."""
    rows: list[dict] = []
    still(rows, 0.0, 2.0)                       # 41 identical samples over 2 s
    kept, report = ax.drop_duplicate_rows(rows, eps=1e-4, span_s=0.25)
    assert report["duplicate_dropped"] > 30
    assert 5 <= len(kept) <= 12
    assert kept[0]["t_wall"] == pytest.approx(0.0)
    assert kept[-1]["t_wall"] > 1.5              # the hold still spans its time


def test_crop_rows_full_pipeline() -> None:
    rows = build_ten_second_still_run()
    kept, report = ax.crop_rows(rows, eps=1e-4, max_still_s=3.0, pad_s=0.0)

    # the whole opening and trailing idle is gone
    assert kept[0]["t_wall"] >= 10.0
    assert kept[-1]["t_wall"] <= 14.3
    assert report["rows_before"] == len(rows)
    assert report["rows_after"] == len(kept)
    assert report["rows_dropped"] == report["rows_before"] - report["rows_after"]
    assert report["rows_dropped"] > 300
    assert report["duplicate_dropped"] >= 3
    assert report["head_dropped"] > 150
    assert report["tail_dropped"] > 150
    assert report["kept_phases"] == ["grasp:enter"]
    # the kept window really is the moving part
    span = kept[-1]["t_wall"] - kept[0]["t_wall"]
    assert 3.5 < span < 5.0
    # every kept timestamp is inside the motion window, repeats included once
    window = [r["t_wall"] for r in kept]
    assert min(window) >= 10.0 and max(window) <= 14.3
    assert window == sorted(window)


def test_crop_rows_keeps_both_anchors_of_a_long_dwell() -> None:
    """The dwell's last row must survive dedupe, otherwise its length is lost."""
    rows: list[dict] = []
    moving(rows, 0.0, 1.0)
    still(rows, 1.05, 6.0)      # 5 s standstill in the middle of the run
    moving(rows, 6.1, 7.0)
    kept, report = ax.crop_rows(rows, max_still_s=3.0, pad_s=0.0)
    assert report["static_runs"] == 1
    dwell = [r for r in kept if 1.0 <= r["t_wall"] <= 6.0]
    assert [round(r["t_wall"], 2) for r in dwell] == [1.0, 6.0]
    assert any(r.get("_collapsed_end") for r in kept)


def test_trim_does_not_mistake_the_first_data_arrival_for_motion() -> None:
    """None -> number is a data-quality event, not robot movement."""
    rows: list[dict] = []
    for index in range(10):                       # recorder started, no data yet
        row = make_row(index * 0.05)
        for column in er.PAYLOAD_COLUMNS:
            row[column] = None
        rows.append(row)
    for index in range(10, 40):                   # data arrives, arm still still
        rows.append(make_row(index * 0.05,
                             phase="grasp:enter" if index >= 20 else ""))
    kept, report = ax.trim_idle_edges(rows, pad_s=0.0)
    assert report["first_activity_s"] == pytest.approx(20 * 0.05)
    assert report["head_dropped"] == 20
    assert kept[0]["t_wall"] == pytest.approx(20 * 0.05)


def test_wall_clock_backstep_is_repaired_without_losing_intervals(tmp_path) -> None:
    session = er.RecorderSession(run_dir=tmp_path, experiment_id="clock",
                                 rate_hz=20.0, want_images=False,
                                 image_period=2.0, t_start=0.0)
    session.open()
    for moment in (100.0, 100.05, 99.90, 99.95, 100.00):
        session.tick(moment, None)
    session.close()
    rows = er.read_timeseries(tmp_path / "timeseries.csv")
    walls = [row["t_wall"] for row in rows]
    assert walls == sorted(walls)                 # the CSV stays orderable
    assert session.clock_backsteps == 1
    deltas = [round(b - a, 6) for a, b in zip(walls, walls[1:])]
    assert deltas[0] == pytest.approx(0.05, abs=1e-6)
    assert deltas[2] == pytest.approx(0.05, abs=1e-5)   # spacing preserved
    assert deltas[3] == pytest.approx(0.05, abs=1e-5)
    assert session.t_first_row == pytest.approx(walls[0])
    assert session.t_last_row == pytest.approx(walls[-1])


def test_crop_rows_pad_keeps_context() -> None:
    rows = build_ten_second_still_run()
    kept, report = ax.crop_rows(rows, pad_s=1.0)
    assert kept[0]["t_wall"] == pytest.approx(9.05)
    assert report["head_dropped"] > 150


# ---------------------------------------------------------------------------
# summary + statistics
# ---------------------------------------------------------------------------
def test_compute_stats_reports_travel_speed_and_force() -> None:
    rows: list[dict] = []
    for index in range(21):
        t = index * 0.05
        rows.append(make_row(t, x=0.30 + 0.01 * index, fx=1.0))
    stats = er.compute_stats(rows)
    assert stats["rows"] == 21
    assert stats["tcp_travel_m"] == pytest.approx(0.20, abs=1e-6)
    assert stats["max_tcp_speed_m_s"] == pytest.approx(0.2, abs=1e-6)
    assert stats["max_force_n"] == pytest.approx(1.0, abs=1e-6)
    assert stats["max_torque_nm"] == pytest.approx(0.0, abs=1e-9)


def test_compute_stats_without_wrench_does_not_invent_numbers() -> None:
    rows = [make_row(t * 0.05, fx=None) for t in range(10)]
    stats = er.compute_stats(rows)
    assert stats["wrench_samples"] == 0
    assert stats["max_force_n"] is None
    assert stats["max_torque_nm"] is None


def test_render_summary_contains_phase_table_and_chinese_sections() -> None:
    rows: list[dict] = []
    for index in range(40):
        rows.append(make_row(index * 0.05, x=0.30 + 0.005 * index,
                             phase="grasp:enter" if index < 20 else "lift:enter"))
    stats = er.compute_stats(rows)
    phases = [
        {"index": 0, "tag": "grasp:enter", "phase": "grasp", "boundary": "enter",
         "t_wall": rows[0]["t_wall"]},
        {"index": 1, "tag": "lift:enter", "phase": "lift", "boundary": "enter",
         "t_wall": rows[20]["t_wall"]},
    ]
    run = {
        "id": "20261024_202700_sim_pick_and_place_efficient",
        "world": "sim", "scope": "pick_and_place", "mode": "efficient",
        "source": "synthetic", "result": "ok", "result_code": 0,
        "git_commit": "abc1234", "git_dirty": True,
        "params": {"rate_hz": 20.0, "images": True},
        "images": 3, "started_at": "2026-10-24T20:27:00+08:00",
        "ended_at": "2026-10-24T20:27:02+08:00",
    }
    summary = er.render_summary(
        run, stats, phases, phase_row_counts=er.count_rows_per_phase(rows))
    for text in ("实验数据摘要", "这次实验做了什么", "关键统计", "每阶段时长",
                 "TCP 行程", "最大关节速度", "最大力矩"):
        assert text in summary
    assert "`grasp:enter`" in summary and "`lift:enter`" in summary
    assert "| 1 |" in summary and "| 2 |" in summary
    # per-phase row counts came from the data, not from a guess
    assert "| 20 |" in summary


def test_render_summary_flags_missing_wrench() -> None:
    rows = [make_row(index * 0.05, fx=None) for index in range(10)]
    stats = er.compute_stats(rows)
    run = {"id": "x", "world": "sim", "scope": "s", "mode": "m",
           "source": "synthetic", "result": "ok", "result_code": 0,
           "params": {"rate_hz": 20.0, "images": False}}
    summary = er.render_summary(run, stats, [])
    assert "没有记录到阶段变化" in summary
    assert "没有收到力/力矩数据" in summary
    assert "未开启图像采集" in summary
    assert "未采集" in summary  # the stat cell, not a fabricated 0


# ---------------------------------------------------------------------------
# image selection + archive helpers
# ---------------------------------------------------------------------------
def test_select_images_keeps_phase_edges_and_periodic_grid() -> None:
    phases = [
        {"tag": "grasp:enter", "t_wall": 100.0},
        {"tag": "lift:enter", "t_wall": 110.0},
        {"tag": "transit:enter", "t_wall": 120.0},
    ]
    # camera at 1.25 s: 100..140 s, i.e. 33 frames, all inside the window
    images = [{"file": f"images/img_{i:05d}.png", "t_wall": 100.0 + 1.25 * i,
               "phase": ""} for i in range(33)]
    kept, report = ax.select_images(images, phases, t_start=100.0, t_end=140.0,
                                    keep_every_s=30.0, tolerance_s=2.5)
    reasons = {entry["reason"] for entry in kept}
    assert "phase_enter" in reasons and "periodic" in reasons
    kept_walls = {round(float(entry["t_wall"]), 2) for entry in kept}
    assert {100.0, 110.0, 120.0} <= kept_walls   # every phase boundary has a frame
    assert report["phase_enter"] >= 1 and report["phase_exit"] >= 1
    assert report["images_after"] == len(kept) < report["images_before"]
    assert report["images_after"] <= 6
    assert all(entry["file"] in {image["file"] for image in images}
               for entry in kept)
    # no image is listed twice
    assert len({entry["file"] for entry in kept}) == len(kept)


def test_select_images_without_phases_still_returns_something() -> None:
    images = [{"file": "images/a.png", "t_wall": 50.0}]
    kept, report = ax.select_images(images, [], t_start=50.0, t_end=50.0,
                                    keep_every_s=30.0)
    assert len(kept) == 1
    assert report["images_after"] == 1
    assert kept[0]["reason"] == "periodic"


def test_archive_root_must_live_under_home(monkeypatch) -> None:
    root = ax.resolve_archive_root(None)
    assert root == pathlib.Path.home() / "cs625_experiment_archive"
    custom = pathlib.Path.home() / "cs625_archive_test"
    monkeypatch.setenv("CS625_EXPERIMENT_ARCHIVE", str(custom))
    assert ax.resolve_archive_root(None) == custom
    with pytest.raises(ValueError):
        ax.resolve_archive_root("/tmp/not-in-home")


def test_append_index_is_idempotent(tmp_path) -> None:
    index = tmp_path / "docs" / "evidence" / "experiment_index.md"
    row = {"id": "20261024_202700_sim_pick_and_place_efficient",
           "date": "2026-10-24", "world": "sim", "scope": "pick_and_place",
           "mode": "efficient", "duration_s": "15.00", "result": "ok",
           "archive_dir": str(pathlib.Path.home() / "cs625_experiment_archive" / "abc")}
    assert ax.append_index(index, row) is True
    assert ax.append_index(index, row) is False
    text = index.read_text()
    assert text.count(row["id"]) == 1
    assert "| id | 日期 | world | scope | mode | 耗时 (s) | 结果 | 归档路径 |" in text
