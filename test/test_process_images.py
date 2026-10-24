"""Unit tests for scripts/capture_process_images.py (pure helpers only).

The capture itself needs a live camera, so these tests pin the parts that can
be wrong silently: when a new run starts, what a metadata record must contain,
which boundaries are legal, and how ROS stamps become file/meta timestamps.
"""

from __future__ import annotations

import importlib.util
import pathlib
import re

import pytest

SCRIPT = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "capture_process_images.py"
spec = importlib.util.spec_from_file_location("capture_process_images", SCRIPT)
capture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(capture)


def test_next_run_id_thirty_minute_rule() -> None:
    now = 1_760_000_000.0

    # First capture ever: a run must start.
    assert capture.next_run_id(now, None) == capture.format_run_id(now)

    # A gap shorter than 30 min keeps the current run.
    assert capture.next_run_id(now, now - 1.0) is None
    assert capture.next_run_id(now, now - 1799.9) is None

    # Exactly 30 min still belongs to the running session (strict ">" rule).
    assert capture.next_run_id(now, now - 30 * 60) is None

    # Anything older than 30 min opens a new run.
    assert capture.next_run_id(now, now - 30 * 60 - 1) == capture.format_run_id(now)
    assert capture.next_run_id(now, now - 7200.0) == capture.format_run_id(now)

    # The generated id is itself a parseable run id (second resolution).
    new_id = capture.next_run_id(now, None)
    assert re.fullmatch(r"\d{8}_\d{6}", new_id)
    parsed = capture.parse_run_id(new_id)
    assert parsed is not None and abs(parsed - now) < 1.0
    assert capture.parse_run_id("not-a-timestamp") is None

    with pytest.raises(ValueError):
        capture.next_run_id(float("nan"), None)


def test_build_record_fields_complete() -> None:
    tcp = {
        "xyz": [0.1, 0.2, 0.3],
        "quaternion": [0.0, 0.0, 0.0, 1.0],
        "frame_id": "base",
    }
    record = capture.build_record(
        "grasp", "enter", tcp, {"sec": 1_760_000_000, "nanosec": 123_456_789},
        "20251024_201630", "unit-test")

    required = {"phase", "boundary", "stamp", "tcp_base", "run_id", "note",
                "frame_id", "run_started_at"}
    assert required <= set(record)
    assert record["phase"] == "grasp"
    assert record["boundary"] == "enter"
    assert record["run_id"] == "20251024_201630"
    assert record["note"] == "unit-test"
    assert record["frame_id"] == "base"
    assert record["run_started_at"] == "2025-10-24T12:16:30.000000000Z"

    pose = record["tcp_base"]
    assert pose["xyz"] == pytest.approx([0.1, 0.2, 0.3])
    assert pose["quaternion"] == pytest.approx([0.0, 0.0, 0.0, 1.0])
    assert len(pose["matrix"]) == 4
    assert all(len(row) == 4 for row in pose["matrix"])
    assert pose["matrix"][3] == [0.0, 0.0, 0.0, 1.0]
    assert pose["matrix"][0][3] == pytest.approx(0.1)

    # A missing pose topic must degrade to null, never to a stale pose; a NaN
    # pose (broadcaster before the robot has a valid state) must do the same.
    nan_pose = capture.build_record(
        "grasp", "enter",
        {"xyz": [float("nan"), 0.0, 0.0], "quaternion": [0.0, 0.0, 0.0, 1.0]},
        {"sec": 1_760_000_000, "nanosec": 0}, "20251024_201630")
    assert nan_pose["tcp_base"] is None
    assert nan_pose["frame_id"] == "base"

    without_pose = capture.build_record(
        "insert", "exit", None, {"sec": 1_760_000_000, "nanosec": 0},
        "20251024_201630")
    assert without_pose["tcp_base"] is None
    assert without_pose["frame_id"] == "base"
    assert without_pose["note"] == ""


def test_invalid_boundary_and_tag_rejected() -> None:
    stamp = {"sec": 1_760_000_000, "nanosec": 0}
    for boundary in ("start", "ENTERED", "", "begin"):
        with pytest.raises(ValueError):
            capture.build_record("grasp", boundary, None, stamp, "20251024_201630")

    # Enter/exit are legal in any letter case; the record normalises them.
    assert capture.build_record("grasp", "ENTER", None, stamp,
                                "20251024_201630")["boundary"] == "enter"

    with pytest.raises(ValueError):
        capture.parse_tag("grasp")            # no boundary
    with pytest.raises(ValueError):
        capture.parse_tag("grasp:enter:exit")  # too many separators
    with pytest.raises(ValueError):
        capture.parse_tag(":enter")           # empty phase
    with pytest.raises(ValueError):
        capture.parse_tag("grasp:sideways")   # unknown boundary

    assert capture.parse_tag("d2a_pre_insert:enter") == ("d2a_pre_insert", "enter")
    assert capture.parse_tag("grasp:EXIT") == ("grasp", "exit")


def test_timestamp_formats() -> None:
    # Fixed, timezone-independent ISO formatting (UTC, nanosecond width).
    assert capture.stamp_to_iso(0, 0) == "1970-01-01T00:00:00.000000000Z"
    assert (capture.stamp_to_iso(1_729_772_190, 123_456_789)
            == "2024-10-24T12:16:30.123456789Z")
    assert re.fullmatch(
        r"\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}\.\d{9}Z",
        capture.stamp_to_iso(1_760_000_000, 1))

    # File stamps follow the repository's local-time convention.
    stamp = capture.file_stamp(1_760_000_000, 123_456_789)
    assert re.fullmatch(r"\d{8}_\d{6}_\d{3}", stamp)
    assert stamp.endswith("_123")

    with pytest.raises(ValueError):
        capture.stamp_to_iso(0, -1)
    with pytest.raises(ValueError):
        capture.stamp_to_iso(0, 1_000_000_000)
