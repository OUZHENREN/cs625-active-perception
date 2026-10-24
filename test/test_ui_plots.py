"""Unit tests for ``scripts/ui_plots.py`` (RealtimePlot + pure helpers).

Two layers are covered here, because two different things can break silently:

1. **pure logic** -- ``trim_window``, ``auto_range``, ``RingBuffer``.  These
   carry the semantics the panel depends on (a rolling window, a 10 % padded
   axis, one window of retained samples) and they must work without a display.
2. **the widget** -- offscreen only (``QT_QPA_PLATFORM=offscreen``), so this
   file runs on a headless box: channel filtering, pause, fixed range, hover
   readout, CSV export, coalesced repaint, and the fact that the module still
   imports when PyQt5 is absent.

No ROS, no camera, no robot, no network.
"""

from __future__ import annotations

import csv
import os
import pathlib
import subprocess
import sys
import textwrap

import pytest

# Must be set before PyQt5 is imported anywhere in this process.
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
SCRIPTS_DIR = REPO_ROOT / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

pytest.importorskip("PyQt5")
from PyQt5 import QtCore, QtGui, QtWidgets  # noqa: E402

import ui_plots as up  # noqa: E402


@pytest.fixture(scope="session")
def app():
    """One offscreen QApplication for every widget test."""
    instance = QtWidgets.QApplication.instance()
    if instance is None:
        instance = QtWidgets.QApplication([])
    return instance


def make_plot(app, channels=("a", "b"), window_s=20.0, **kwargs):
    return up.RealtimePlot(channels=channels, window_seconds=window_s, **kwargs)


def feed(plot, count, rate=250.0, t0=0.0):
    """Push ``count`` samples at ``rate`` Hz starting at ``t0``.

    Channel ``a`` carries ``t * rate`` (= the sample index, absolute in time, so
    two consecutive feed() calls stay consistent) and ``b`` mirrors it.
    """
    dt = 1.0 / rate
    for index in range(count):
        t = t0 + index * dt
        value = t * rate
        plot.push_sample(t, {"a": value, "b": -value})
    return t0 + (count - 1) * dt


# ---------------------------------------------------------------------------
# 1. trim_window
# ---------------------------------------------------------------------------
def test_trim_window_keeps_only_recent_samples():
    samples = [(float(i), {"v": float(i)}) for i in range(10)]  # t = 0..9
    kept = up.trim_window(samples, t_now=9.0, window_s=3.0)
    assert [t for t, _ in kept] == [6.0, 7.0, 8.0, 9.0]
    # The pairs are reused, not rebuilt: the dict identity is preserved.
    assert kept[0][1] is samples[6][1]


def test_trim_window_edge_cases():
    samples = [(0.0, {"v": 0.0}), (1.0, {"v": 1.0}), (2.0, {"v": 2.0})]
    # Empty input, dead window, and negative window all yield no samples.
    assert up.trim_window([], 5.0, 2.0) == []
    assert up.trim_window(samples, 2.0, 0.0) == []
    assert up.trim_window(samples, 2.0, -1.0) == []
    # The window is closed on both ends: t_now-window and t_now survive ...
    assert up.trim_window(samples, 2.0, 1.0) == samples[1:3]
    # ... and a sample newer than t_now (clock jump) is dropped, not drawn
    # outside the axis.
    assert up.trim_window(samples, 1.0, 5.0) == samples[:2]


# ---------------------------------------------------------------------------
# 2. RingBuffer
# ---------------------------------------------------------------------------
def test_ring_buffer_capacity_drops_oldest():
    buffer = up.RingBuffer(capacity=5)
    for index in range(9):
        buffer.push(float(index), {"v": float(index)})
    assert len(buffer) == 5
    assert [t for t, _ in buffer] == [4.0, 5.0, 6.0, 7.0, 8.0]
    assert buffer.first()[0] == 4.0
    assert buffer.last()[0] == 8.0
    assert buffer.dropped == 4


def test_ring_buffer_trim_before_and_resize():
    buffer = up.RingBuffer(capacity=100)
    for index in range(10):
        buffer.push(float(index), {"v": float(index)})
    assert buffer.trim_before(7.0) == 7          # 0..6 dropped, 7 included
    assert [t for t, _ in buffer] == [7.0, 8.0, 9.0]
    assert buffer.trim_before(-1.0) == 0         # nothing older than -1
    buffer.set_capacity(2)                       # resize keeps the newest
    assert [t for t, _ in buffer] == [8.0, 9.0]
    buffer.clear()
    assert len(buffer) == 0 and buffer.first() is None and buffer.last() is None


# ---------------------------------------------------------------------------
# 3. auto_range
# ---------------------------------------------------------------------------
def test_auto_range_empty_and_non_finite():
    assert up.auto_range([]) == (0.0, 1.0)
    assert up.auto_range([float("nan"), float("inf")]) == (0.0, 1.0)
    # NaN is ignored, the finite value still defines a band.
    low, high = up.auto_range([float("nan"), 3.0, float("inf")])
    assert low < 3.0 < high


def test_auto_range_single_point():
    low, high = up.auto_range([2.0])
    assert low < 2.0 < high
    # Flat line / single value: +-max(0.5, |v|*0.1)
    assert (high - low) == pytest.approx(2.0 * max(0.5, 0.2))  # = 1.0
    assert high == pytest.approx(2.5) and low == pytest.approx(1.5)
    # A large value gets a proportional pad instead of a fixed one.
    low_big, high_big = up.auto_range([1000.0])
    assert (high_big - low_big) == pytest.approx(200.0)  # 10 % of 1000, each side


def test_auto_range_multiple_values_uses_margin():
    values = [-1.0, 0.0, 3.0]
    low, high = up.auto_range(values)
    span = 4.0
    assert low == pytest.approx(-1.0 - span * 0.1)
    assert high == pytest.approx(3.0 + span * 0.1)
    # A different margin is honoured, and a negative margin does not invert.
    low5, high5 = up.auto_range(values, margin=0.5)
    assert low5 == pytest.approx(-1.0 - span * 0.5)
    low0, high0 = up.auto_range(values, margin=-3.0)
    assert (low0, high0) == (-1.0, 3.0)
    # Ordering of the input does not matter.
    assert up.auto_range([3.0, -1.0, 0.0]) == (low, high)


# ---------------------------------------------------------------------------
# 4. the widget: window, filtering, range
# ---------------------------------------------------------------------------
def test_plot_keeps_only_one_window_of_samples(app):
    plot = make_plot(app, window_s=20.0)
    feed(plot, 5000, rate=250.0)  # 20 s at 250 Hz
    assert plot.sample_count() == pytest.approx(5000, abs=5)
    assert plot.time_now() == pytest.approx(4999 / 250.0)
    assert plot._buffer.first()[0] >= plot.time_now() - 20.0 - 1e-9
    # Pushing past the window keeps the count flat (it is not a full history).
    feed(plot, 250, rate=250.0, t0=20.0)
    assert plot.sample_count() == pytest.approx(5000, abs=5)
    # A shorter window drops older samples immediately.
    plot.set_window_seconds(5.0)
    assert plot.sample_count() == pytest.approx(1250, abs=5)
    plot.deleteLater()


def test_plot_channel_filter(app):
    plot = make_plot(app, channels=())
    feed(plot, 10)
    # Empty selection means "draw everything that was pushed".
    assert plot._visible_channels() == ["a", "b"]
    plot.set_channels(["b"])
    assert plot.channels() == ["b"]
    assert plot._visible_channels() == ["b"]
    # Filtering only limits what is drawn/exported; values are still buffered.
    assert set(plot.latest_values()) == {"a", "b"}
    assert plot.latest_values()["a"] == pytest.approx(9.0)
    plot.set_channels([])
    assert plot._visible_channels() == ["a", "b"]
    plot.deleteLater()


def test_plot_fixed_range_overrides_auto(app):
    plot = make_plot(app)
    feed(plot, 50)
    auto_lo, auto_hi = plot.y_range()
    assert auto_lo < 0.0 and auto_hi > 49.0
    plot.set_y_range(-2.0, 2.0)
    assert plot.y_range() == (-2.0, 2.0)
    assert not plot._auto_range
    # Invalid ranges are ignored rather than producing a broken axis.
    plot.set_y_range(5.0, -5.0)
    plot.set_y_range(float("nan"), 1.0)
    assert plot.y_range() == (-2.0, 2.0)
    plot.set_auto_range(True)
    assert plot.y_range() == pytest.approx((auto_lo, auto_hi))
    plot.deleteLater()


def test_plot_colors_cycle(app):
    plot = make_plot(app, channels=["c0", "c1", "c2", "c3", "c4"])
    mapping = plot._color_map(plot._visible_channels())
    assert mapping["c0"] == up.SERIES_COLORS[0]
    assert mapping["c3"] == up.SERIES_COLORS[3]
    assert mapping["c4"] == up.SERIES_COLORS[0]  # fifth channel wraps around
    plot.set_colors(["#101010", "#202020"])
    mapping = plot._color_map(plot._visible_channels())
    assert mapping["c0"] == "#101010" and mapping["c1"] == "#202020"
    assert mapping["c2"] == "#101010"
    plot.deleteLater()


# ---------------------------------------------------------------------------
# 5. the widget: pause, hover, repaint coalescing
# ---------------------------------------------------------------------------
def test_pause_freezes_view_but_keeps_recording(app):
    plot = make_plot(app)
    feed(plot, 100, rate=100.0)  # t_now = 0.99
    frozen = plot.time_now()
    plot.set_paused(True)
    feed(plot, 50, rate=100.0, t0=1.00)
    assert plot.time_now() == frozen          # the axis does not move
    assert plot.sample_count() > 100          # data is still recorded
    assert plot.is_paused()
    # latest_values() follows the frozen axis, not the hidden newest sample.
    assert plot.latest_values()["a"] == pytest.approx(99.0)
    plot.set_paused(False)
    assert plot.time_now() == pytest.approx(1.00 + 49 / 100.0)
    assert plot.latest_values()["a"] == pytest.approx(149.0)
    plot.deleteLater()


def test_repaints_are_coalesced_not_per_sample(app):
    plot = make_plot(app)
    feed(plot, 400)                # 400 pushes, no event loop: zero paints
    assert plot.push_count() == 400
    assert plot.repaint_count() <= 1
    before = plot.repaint_count()
    plot.grab()                    # forces one paint, independent of the event loop
    assert plot.repaint_count() >= before + 1
    # The cap is a 100 ms single-shot timer: one paint per 10 pushes at 100 Hz.
    assert up.MAX_REPAINT_HZ == 10
    assert plot._repaint_timer.interval() == 100
    assert plot._repaint_timer.isSingleShot()
    plot.deleteLater()


def test_plot_falls_back_to_english_without_cjk_font(app, monkeypatch):
    """No CJK font installed -> English labels, and painting must not raise."""
    monkeypatch.setattr(up, "_has_cjk_font", lambda: False)
    plot = make_plot(app)
    plot.resize(400, 200)
    assert plot._lang == "en"
    assert plot._labels["paused"] == "PAUSED"
    assert plot._labels["no_data"] == "no data"
    feed(plot, 100)                # data -> curves
    plot.grab()                    # English axis/legend/badge path
    plot.set_paused(True)
    plot.grab()                    # PAUSED badge path
    assert not plot.grab().isNull()
    plot.deleteLater()


def test_hover_readout_returns_nearest_sample(app):
    plot = make_plot(app)
    feed(plot, 5000, rate=250.0)   # a full 20 s window, so the cursor time lands
    assert plot.hover_readout() is None  # on real samples

    plot.resize(800, 320)
    point = QtCore.QPoint(int(800 * 0.4), 160)
    event = QtGui.QMouseEvent(
        QtCore.QEvent.MouseMove, QtCore.QPointF(point),
        QtCore.Qt.NoButton, QtCore.Qt.NoButton, QtCore.Qt.NoModifier)
    plot.mouseMoveEvent(event)
    readout = plot.hover_readout()
    assert readout is not None
    t_sample, values = readout
    expected = plot._time_at_x(point.x())
    assert abs(t_sample - expected) <= 1.0 / 250.0 + 1e-9  # within one sample
    assert values["a"] == pytest.approx(t_sample * 250.0)
    plot.deleteLater()


# ---------------------------------------------------------------------------
# 6. CSV export
# ---------------------------------------------------------------------------
def test_export_csv_row_count_and_header(app, tmp_path):
    plot = make_plot(app, channels=["a", "b"], window_s=1.0)
    feed(plot, 100, rate=100.0)    # exactly the 1 s window
    target = tmp_path / "nested" / "plot.csv"   # parent must be created
    rows = plot.export_csv(str(target))
    assert rows == 100 == plot.sample_count()
    with target.open(newline="", encoding="utf-8") as handle:
        table = list(csv.reader(handle))
    assert table[0] == ["t", "a", "b"]
    assert len(table) == rows + 1
    assert float(table[1][0]) == pytest.approx(0.0)
    assert float(table[1][1]) == pytest.approx(0.0)   # index 0 -> a = 0
    assert float(table[-1][2]) == pytest.approx(-99.0)
    plot.deleteLater()


def test_export_csv_leaves_missing_channel_blank(app, tmp_path):
    plot = up.RealtimePlot(channels=["a", "missing"], window_seconds=10.0)
    plot.push_sample(0.0, {"a": 1.0})
    plot.push_sample(0.5, {"a": 2.0})
    target = tmp_path / "sparse.csv"
    rows = plot.export_csv(str(target))
    lines = target.read_text(encoding="utf-8").splitlines()
    assert rows == 2
    assert lines[1].split(",") == ["0.000000", "1.000000", ""]
    assert lines[2].split(",") == ["0.500000", "2.000000", ""]
    plot.deleteLater()


# ---------------------------------------------------------------------------
# 7. robustness / headless guarantees
# ---------------------------------------------------------------------------
def test_push_sample_rejects_bad_input(app):
    plot = make_plot(app)
    plot.push_sample(0.0, {"a": 1.0, "junk": float("nan"), "s": "x"})
    plot.push_sample(0.5, {"a": float("inf")})      # nothing finite -> ignored
    plot.push_sample(float("nan"), {"a": 2.0})      # bad timestamp -> ignored
    plot.push_sample(-0.5, {"a": 3.0})              # out of order -> dropped
    assert plot.push_count() == 1
    assert plot.latest_values() == {"a": 1.0}
    assert plot._out_of_order == 1
    plot.deleteLater()


def test_offscreen_render_draws_series(app):
    plot = make_plot(app, channels=["a", "b"], window_s=10.0)
    plot.resize(640, 260)
    feed(plot, 500, rate=50.0)
    pixmap = plot.grab()
    assert not pixmap.isNull()
    assert pixmap.width() == 640 and pixmap.height() == 260
    image = pixmap.toImage()
    background = QtGui.QColor(up.BG_COLOR)
    painted = 0
    for y in range(0, image.height(), 5):
        for x in range(0, image.width(), 3):
            if image.pixelColor(x, y) != background:
                painted += 1
    # Grid, axis labels, curves and the legend all differ from the panel fill.
    assert painted > 500, f"only {painted} non-background pixels"
    plot.deleteLater()


def test_module_imports_and_degrades_without_qt(tmp_path):
    """The pure helpers must import and work when PyQt5 cannot be imported."""
    (tmp_path / "PyQt5.py").write_text(
        "raise ImportError('PyQt5 intentionally unavailable')\n",
        encoding="utf-8")
    script = textwrap.dedent(
        """
        import ui_plots as up
        assert up.QT_AVAILABLE is False
        assert up.trim_window([(0.0, {"v": 1.0})], 0.5, 1.0) == [(0.0, {"v": 1.0})]
        low, high = up.auto_range([1.0, 3.0])
        assert abs(low - 0.8) < 1e-9 and abs(high - 3.2) < 1e-9
        buffer = up.RingBuffer(capacity=2)
        buffer.push(0.0, {"v": 0.0})
        buffer.push(1.0, {"v": 1.0})
        buffer.push(2.0, {"v": 2.0})
        assert len(buffer) == 2
        try:
            up.RealtimePlot()
        except RuntimeError as exc:
            assert "PyQt5" in str(exc)
        else:
            raise AssertionError("RealtimePlot should need PyQt5")
        print("ok")
        """
    )
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join([str(tmp_path), str(SCRIPTS_DIR)])
    env["QT_QPA_PLATFORM"] = "offscreen"
    result = subprocess.run(
        [sys.executable, "-c", script],
        cwd=str(tmp_path), env=env, capture_output=True, text=True, timeout=120)
    assert result.returncode == 0, result.stderr
    assert "ok" in result.stdout
