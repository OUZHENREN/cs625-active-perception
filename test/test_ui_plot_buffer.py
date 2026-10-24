"""Checks for the oscilloscope buffer and range maths used by the CS625 panel.

The curve widget lives in ``scripts/ui_plots.py`` (``RealtimePlot``); this file
covers the pure, Qt-free logic it relies on, so it runs without a display:

    python3 -m pytest test/test_ui_plot_buffer.py -q

Covered:
* :class:`ui_plots.RingBuffer` -- FIFO order, capacity cap, window trim, clear;
* :func:`ui_plots.trim_window` -- only samples inside ``[t-now, t]`` survive;
* :func:`ui_plots.auto_range` -- 10 % margin, flat series, NaN handling;
* :class:`ui_plots.RealtimePlot` -- pause, clear and CSV export keep missing
  samples empty instead of writing zero (only if PyQt5 is importable).
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "scripts"))

ui_plots = pytest.importorskip("ui_plots")


# ------------------------------------------------------------------ ring
def test_ring_keeps_fifo_order_and_last():
    ring = ui_plots.RingBuffer(capacity=8)
    for index in range(5):
        ring.push(float(index), {"a": float(index)})
    assert len(ring) == 5
    assert ring.first()[0] == 0.0
    assert ring.last() == (4.0, {"a": 4.0})


def test_ring_capacity_drops_oldest_and_counts_dropped():
    ring = ui_plots.RingBuffer(capacity=4)
    for index in range(10):
        ring.push(float(index), {"a": float(index)})
    assert len(ring) == 4
    assert [sample[0] for sample in ring] == [6.0, 7.0, 8.0, 9.0]
    assert ring.dropped == 6


def test_ring_trim_before_is_the_rolling_window():
    ring = ui_plots.RingBuffer()
    for index in range(21):
        ring.push(float(index), {"a": float(index)})
    dropped = ring.trim_before(15.0)
    assert dropped == 15
    assert [sample[0] for sample in ring] == [15.0, 16.0, 17.0, 18.0, 19.0, 20.0]


def test_ring_clear_resets_everything():
    ring = ui_plots.RingBuffer()
    ring.push(1.0, {"a": 1.0})
    ring.clear()
    assert len(ring) == 0
    assert ring.first() is None
    assert ring.last() is None


def test_trim_window_keeps_only_visible_samples():
    samples = [(float(index), {"a": float(index)}) for index in range(10)]
    visible = ui_plots.trim_window(samples, 9.0, 3.0)
    assert [sample[0] for sample in visible] == [6.0, 7.0, 8.0, 9.0]
    # A future timestamp (clock jump) is dropped, a zero window is empty.
    assert ui_plots.trim_window([(10.0, {"a": 1.0})], 5.0, 10.0) == []
    assert ui_plots.trim_window(samples, 9.0, 0.0) == []


# ------------------------------------------------------------------ ranges
def test_auto_range_adds_ten_percent_headroom():
    low, high = ui_plots.auto_range([0.0, 10.0])
    assert low == pytest.approx(-1.0)
    assert high == pytest.approx(11.0)


def test_auto_range_flat_series_is_not_zero_height():
    low, high = ui_plots.auto_range([2.0, 2.0, 2.0])
    assert high > low
    assert low == pytest.approx(1.5)
    assert high == pytest.approx(2.5)


def test_auto_range_ignores_nan_and_empty_input():
    assert ui_plots.auto_range([]) == (0.0, 1.0)
    low, high = ui_plots.auto_range([math.nan, 1.0, math.inf, 2.0])
    assert low == pytest.approx(0.9)
    assert high == pytest.approx(2.1)


# --------------------------------------------------------------- widget api
@pytest.fixture(scope="module")
def qt_app():
    qt = pytest.importorskip("PyQt5.QtWidgets")
    app = qt.QApplication.instance()
    if app is None:
        app = qt.QApplication([])
    return app


def test_realtime_plot_push_pause_and_clear(qt_app):
    plot = ui_plots.RealtimePlot(("a", "b"), window_seconds=20.0)
    plot.push_sample(1.0, {"a": 1.0, "b": -1.0})
    assert plot.sample_count() == 1
    plot.set_paused(True)
    # Paused freezes the axis but still records (export must stay complete).
    plot.push_sample(2.0, {"a": 2.0})
    assert plot.sample_count() == 2
    plot.set_paused(False)
    plot.clear()
    assert plot.sample_count() == 0
    assert plot.known_channels() == []
    plot.deleteLater()


def test_realtime_plot_export_csv_keeps_missing_channels_empty(qt_app, tmp_path):
    plot = ui_plots.RealtimePlot(("a", "b"), window_seconds=20.0)
    plot.push_sample(100.0, {"a": 1.0})
    plot.push_sample(101.0, {"a": 2.0, "b": 3.5})
    target = tmp_path / "plot.csv"
    rows = plot.export_csv(str(target))
    assert rows == 2
    lines = target.read_text(encoding="utf-8").splitlines()
    assert lines[0] == "t,a,b"
    # The first sample has no "b": the cell is empty, never 0.
    assert lines[1].split(",")[2] == ""
    assert lines[2].split(",")[2] == "3.500000"
    plot.deleteLater()
