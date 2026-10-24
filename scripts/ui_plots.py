#!/usr/bin/env python3
"""A self-drawn rolling-curve widget for the CS625 operator UI.

What this module is for
-----------------------
The operator UI needs a small live trend display (joint currents, TCP error,
force magnitude, ...).  pyqtgraph is not installed in this environment and
matplotlib redraws cost more than a 10 Hz panel can justify, so this module
draws the curves itself with ``QPainter`` on a plain ``QWidget``.  It depends on
PyQt5 only; there is no numpy, no pyqtgraph and no matplotlib in the import
graph.

The widget is deliberately independent of ``scripts/cs625_ui.py``: it knows
nothing about ROS, topics or the main window.  The caller owns the data and
calls ``push_sample(t, {name: value})``; the widget owns the buffering, the
rolling window, the range, the painting and the CSV export.

Two layers, on purpose
----------------------
1. **Pure functions** (top of this file): ``trim_window`` and ``auto_range``
   plus ``RingBuffer``.  They import no Qt at all, so they can be unit tested in
   a headless environment and reused from a non-GUI recorder.
2. **``RealtimePlot``** (``QWidget`` subclass): ring buffer + 10 Hz coalesced
   repaint + ``paintEvent``.  Qt is imported behind a ``try``/``except``: on a
   machine without PyQt5 the module still imports and the pure functions still
   work, only instantiating the widget raises.

Contract used by the main window
--------------------------------
::

    plot = RealtimePlot(channels=("j1", "j2"), window_seconds=20.0)
    plot.push_sample(t_seconds, {"j1": 0.31, "j2": -0.02})   # t monotonic
    plot.set_paused(True)  /  plot.set_paused(False)
    plot.set_channels(("j1",))  plot.set_colors(("#35d0c0",))
    plot.set_window_seconds(10.0)  plot.set_y_range(0.0, 1.0)
    rows = plot.export_csv("/tmp/run_plot.csv")
    latest = plot.latest_values()

Data flow::

    caller (20..250 Hz) -> push_sample -> RingBuffer (window-trimmed)
                                          -> QTimer 10 Hz -> paintEvent
                        -> export_csv / latest_values / hover_readout

Performance notes
-----------------
* Only samples inside the window are kept: every push drops the tail that fell
  out of ``t - window_seconds`` (O(1) amortised, no list rebuild).
* ``paintEvent`` never copies the buffer.  It walks the deque twice (bounds,
  then points) and decimates to at most one vertex per pixel column, so the
  polygon it draws is bounded by the widget width, not by the sample count.
* Repaints are coalesced by a single-shot 100 ms ``QTimer``: feeding 250 Hz
  costs one paint per 100 ms, not one per sample.

Known limits
------------
* One vertex per pixel column keeps the *last* value in that column; a signal
  oscillating faster than one cycle per pixel can alias.  For a 20 s window on
  a 1000 px panel that is 20 ms per pixel, far below the panel's purpose.
* No zoom/pan, no axis unit conversion: the caller passes values already in the
  unit it wants to display.

Headless
--------
``QT_QPA_PLATFORM=offscreen`` works for every code path used here (the demo and
the unit tests use it).  Chinese labels are used only when a CJK-capable font
family is installed; otherwise the widget falls back to English so it cannot
crash on a bare container.
"""

from __future__ import annotations

import csv
import math
import os
from collections import deque
from typing import Deque, Dict, Iterator, List, Optional, Sequence, Tuple

__all__ = [
    "RingBuffer",
    "RealtimePlot",
    "auto_range",
    "trim_window",
    "QT_AVAILABLE",
]

# ---------------------------------------------------------------------------
# palette (mirrors scripts/cs625_ui.py so the panel does not look foreign)
# ---------------------------------------------------------------------------
BG_COLOR = "#0f141a"        # panel background
GRID_COLOR = "#24303d"      # grid lines
AXIS_COLOR = "#8aa0b4"      # axis labels / ticks
TEXT_COLOR = "#dbe7f3"      # legend text
CARD_COLOR = "#161d26"      # hover tooltip background
MUTED_COLOR = "#5f7386"     # secondary text
WARN_COLOR = "#f2b04a"      # paused badge / warning

#: Four series colours, cycled by channel index.
SERIES_COLORS: Tuple[str, ...] = ("#35d0c0", "#f2b04a", "#ef6b6b", "#9d8cff")

#: Hard cap on repaint rate.  ``push_sample`` never paints directly.
MAX_REPAINT_HZ = 10
_REPAINT_INTERVAL_MS = int(round(1000.0 / MAX_REPAINT_HZ))

_DEFAULT_WINDOW_S = 20.0
_MIN_WINDOW_S = 0.05
#: Capacity rule: enough room for 500 Hz inside the window, and never less
#: than 4096 samples.  Time-based trimming is what actually bounds the buffer;
#: the capacity is only the safety net for a caller that pushes faster.
_MAX_SAMPLES_PER_SECOND = 500.0


# ---------------------------------------------------------------------------
# 1. pure logic -- no Qt below this comment above the Qt import block
# ---------------------------------------------------------------------------
def trim_window(
    samples: Sequence[Tuple[float, Dict[str, float]]],
    t_now: float,
    window_s: float,
) -> List[Tuple[float, Dict[str, float]]]:
    """Return the samples that belong to the window ``[t_now-window_s, t_now]``.

    ``samples`` is assumed to be ordered by time (the ring buffer guarantees
    it).  Order is preserved and no sample is copied: the returned list holds
    the same ``(t, dict)`` pairs.  A non-positive window returns ``[]``; samples
    newer than ``t_now`` (a clock jump) are dropped rather than drawn outside
    the axis.
    """
    if window_s <= 0 or not samples:
        return []
    t_now = float(t_now)
    window_s = float(window_s)
    t_min = t_now - window_s
    out: List[Tuple[float, Dict[str, float]]] = []
    for t, values in samples:
        if t_min <= t <= t_now:
            out.append((t, values))
    return out


def _range_from_bounds(lo: float, hi: float, margin: float = 0.1) -> Tuple[float, float]:
    """Expand ``[lo, hi]`` by ``margin`` of its span; used by :func:`auto_range`."""
    if not (math.isfinite(lo) and math.isfinite(hi)):
        return (0.0, 1.0)
    if hi < lo:
        lo, hi = hi, lo
    margin = max(0.0, float(margin))
    if hi - lo <= 1e-12:
        # A single value (or a flat line) still needs a visible band.
        pad = max(0.5, abs(lo) * 0.1)
        return (lo - pad, hi + pad)
    pad = (hi - lo) * margin
    return (lo - pad, hi + pad)


def auto_range(series: Sequence[float], margin: float = 0.1) -> Tuple[float, float]:
    """Auto range for a list of values, with 10 % headroom by default.

    * empty / all non-finite -> ``(0.0, 1.0)`` (a sane placeholder axis);
    * one value or a flat line -> ``v +- max(0.5, |v| * 0.1)``;
    * otherwise ``[lo - span*margin, hi + span*margin]``.

    Non-finite values (NaN/inf) are ignored instead of poisoning the axis.
    """
    vals = [float(v) for v in series if v is not None and math.isfinite(float(v))]
    if not vals:
        return (0.0, 1.0)
    return _range_from_bounds(min(vals), max(vals), margin)


class RingBuffer:
    """Fixed-capacity FIFO of ``(t, {channel: value})`` samples.

    Two independent bounds apply, both needed:

    * ``capacity`` -- hard cap, oldest sample is dropped (a caller pushing much
      faster than expected cannot grow memory without limit);
    * ``trim_before(t_min)`` -- the rolling window, called on every push by
      :class:`RealtimePlot` so the buffer holds one window of data.

    ``deque`` gives O(1) append/popleft and ordered iteration, which is what the
    painter walks.
    """

    __slots__ = ("_data", "capacity", "dropped")

    def __init__(self, capacity: int = 20000) -> None:
        self._data: Deque[Tuple[float, Dict[str, float]]] = deque()
        self.capacity = max(1, int(capacity))
        self.dropped = 0  # samples lost to the capacity cap (not window trim)

    # -- container protocol -------------------------------------------------
    def __len__(self) -> int:
        return len(self._data)

    def __iter__(self) -> Iterator[Tuple[float, Dict[str, float]]]:
        return iter(self._data)

    def __getitem__(self, index: int) -> Tuple[float, Dict[str, float]]:
        return self._data[index]

    def __bool__(self) -> bool:
        return bool(self._data)

    # -- mutation -----------------------------------------------------------
    def push(self, t: float, values: Dict[str, float]) -> None:
        self._data.append((float(t), values))
        while len(self._data) > self.capacity:
            self._data.popleft()
            self.dropped += 1

    def trim_before(self, t_min: float) -> int:
        """Drop samples with ``t < t_min``; return how many were dropped."""
        dropped = 0
        while self._data and self._data[0][0] < t_min:
            self._data.popleft()
            dropped += 1
        return dropped

    def clear(self) -> None:
        self._data.clear()
        self.dropped = 0

    def set_capacity(self, capacity: int) -> None:
        """Resize the cap, dropping the oldest samples if the buffer is larger."""
        self.capacity = max(1, int(capacity))
        while len(self._data) > self.capacity:
            self._data.popleft()
            self.dropped += 1

    # -- read-only helpers --------------------------------------------------
    def first(self) -> Optional[Tuple[float, Dict[str, float]]]:
        return self._data[0] if self._data else None

    def last(self) -> Optional[Tuple[float, Dict[str, float]]]:
        return self._data[-1] if self._data else None

    def to_list(self) -> List[Tuple[float, Dict[str, float]]]:
        """Explicit copy, for callers that really need a list (export/tests)."""
        return list(self._data)


# ---------------------------------------------------------------------------
# 2. Qt import guarded, so the module is importable without PyQt5
# ---------------------------------------------------------------------------
try:  # pragma: no cover - the happy path is covered by every GUI test
    from PyQt5 import QtCore, QtGui, QtWidgets

    QT_AVAILABLE = True
except Exception:  # pragma: no cover - exercised by the no-Qt subprocess test
    QtCore = QtGui = QtWidgets = None  # type: ignore[assignment]
    QT_AVAILABLE = False

if QT_AVAILABLE:
    _WidgetBase = QtWidgets.QWidget
else:  # pragma: no cover
    _WidgetBase = object


#: Labels per language; English is the fallback when no CJK font is installed.
_LABELS: Dict[str, Dict[str, str]] = {
    "zh": {
        "no_data": "无数据",
        "paused": "已暂停",
        "auto": "自动量程",
        "fixed": "固定量程",
        "samples": "样本",
        "window": "窗口",
        "seconds": "秒",
        "now": "现在",
        "time": "时间",
    },
    "en": {
        "no_data": "no data",
        "paused": "PAUSED",
        "auto": "auto",
        "fixed": "fixed",
        "samples": "n",
        "window": "win",
        "seconds": "s",
        "now": "now",
        "time": "t",
    },
}

#: Font families tried in order; the last two exist on a bare Debian/Ubuntu.
_FONT_FAMILIES = (
    "Noto Sans CJK SC",
    "Source Han Sans SC",
    "WenQuanYi Micro Hei",
    "Microsoft YaHei",
    "DejaVu Sans",
)
_MONO_FAMILIES = (
    "JetBrains Mono",
    "Cascadia Mono",
    "DejaVu Sans Mono",
    "WenQuanYi Micro Hei Mono",
    "monospace",
)
#: Substrings that mark a font family as able to render Chinese labels.
_CJK_MARKERS = ("cjk", "wenquanyi", "source han", "microsoft yahei", "simhei",
                "heiti", "noto sans sc", "droid sans fallback")


def _has_cjk_font() -> bool:
    """True when a CJK-capable family is installed (best effort, never raises)."""
    if not QT_AVAILABLE:  # pragma: no cover
        return False
    try:
        families = QtGui.QFontDatabase().families()
    except Exception:  # pragma: no cover - exotic platform plugin
        return False
    for family in families:
        low = str(family).lower()
        if any(marker in low for marker in _CJK_MARKERS):
            return True
    return False


def _make_font(point_size: float, mono: bool = False) -> "QtGui.QFont":
    font = QtGui.QFont()
    families = list(_MONO_FAMILIES if mono else _FONT_FAMILIES)
    try:
        font.setFamilies(families)  # Qt >= 5.13
    except Exception:  # pragma: no cover - older Qt
        try:
            font.setFamily(families[-1])
        except Exception:
            pass
    try:
        font.setPointSizeF(float(point_size))
    except Exception:  # pragma: no cover
        pass
    return font


def _text_width(metrics: "QtGui.QFontMetrics", text: str) -> float:
    """``horizontalAdvance`` with a fallback for older Qt bindings."""
    try:
        return float(metrics.horizontalAdvance(text))
    except Exception:  # pragma: no cover
        return float(metrics.width(text))


def _fmt_value(value: float, digits: int = 3) -> str:
    """Compact number for the legend / tooltip (never raises on NaN)."""
    try:
        value = float(value)
    except (TypeError, ValueError):  # pragma: no cover - defensive
        return "--"
    if not math.isfinite(value):
        return "--"
    mag = abs(value)
    if mag != 0.0 and (mag < 0.01 or mag >= 1000.0):
        return f"{value:.3g}"
    return f"{value:.{digits}f}"


def _axis_text(value: float, span: float) -> str:
    """Axis tick text with a precision that follows the visible span."""
    if not math.isfinite(value):
        return "--"
    if span <= 0 or not math.isfinite(span):
        span = 1.0
    try:
        decimals = int(max(0, min(6, 2 - math.floor(math.log10(span)))))
    except (ValueError, OverflowError):  # pragma: no cover
        decimals = 2
    return f"{value:.{decimals}f}"


class RealtimePlot(_WidgetBase):
    """Rolling multi-channel curve, drawn with QPainter (no pyqtgraph).

    Parameters
    ----------
    channels:
        Channel names to draw.  Empty means "draw every channel seen so far",
        which is what the constructor default gives the main window.
    window_seconds:
        Rolling window width in seconds (samples older than ``t_now - window``
        are dropped on every push).
    parent:
        Standard QWidget parent.
    max_samples:
        Optional explicit ring-buffer capacity.  When ``None`` the capacity is
        derived from ``window_seconds`` (500 Hz headroom, minimum 4096).
    colors:
        Optional override of the four default series colours.
    """

    #: Scaled with the widget height; the painter only needs a few margins.
    MARGIN_LEFT = 46.0
    MARGIN_RIGHT = 10.0
    MARGIN_TOP = 24.0
    MARGIN_BOTTOM = 20.0
    CORNER_RADIUS = 10.0

    def __init__(
        self,
        channels: Sequence[str] = (),
        window_seconds: float = _DEFAULT_WINDOW_S,
        parent=None,
        *,
        max_samples: Optional[int] = None,
        colors: Optional[Sequence[str]] = None,
    ) -> None:
        if not QT_AVAILABLE:  # pragma: no cover - no-Qt environment
            raise RuntimeError(
                "RealtimePlot needs PyQt5; the pure functions "
                "(trim_window/auto_range/RingBuffer) work without it"
            )
        super().__init__(parent)

        self._window_s = self._sanitize_window(window_seconds)
        self._channels: List[str] = [str(c) for c in channels]
        self._colors: Tuple[str, ...] = tuple(colors) if colors else SERIES_COLORS
        self._auto_capacity = max_samples is None
        capacity = (
            self._default_capacity(self._window_s) if max_samples is None
            else max(1, int(max_samples))
        )
        self._buffer = RingBuffer(capacity)
        self._known: Dict[str, None] = {}  # ordered set of channels ever pushed

        self._auto_range = True
        self._fixed_range: Tuple[float, float] = (0.0, 1.0)
        self._paused = False
        self._t_now = 0.0
        self._t_last: Optional[float] = None            # newest ingested sample
        self._hover_pos: Optional["QtCore.QPoint"] = None
        self._paint_count = 0
        self._push_count = 0
        self._dirty = False
        self._out_of_order = 0

        self._lang = "zh" if _has_cjk_font() else "en"
        self._labels = _LABELS[self._lang]
        self._font_axis = _make_font(8.0)
        self._font_legend = _make_font(8.5)
        self._font_badge = _make_font(8.0)
        self._font_hover = _make_font(8.5)
        self._font_hover_mono = _make_font(8.0, mono=True)

        self.setMinimumSize(280, 150)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding,
                           QtWidgets.QSizePolicy.Expanding)
        self.setMouseTracking(True)
        self.setAutoFillBackground(False)
        self.setAttribute(QtCore.Qt.WA_OpaquePaintEvent, False)
        self.setFocusPolicy(QtCore.Qt.NoFocus)
        self.setToolTip("")

        # One single-shot timer coalesces every push_sample into <= 10 Hz paint.
        self._repaint_timer = QtCore.QTimer(self)
        self._repaint_timer.setSingleShot(True)
        self._repaint_timer.setInterval(_REPAINT_INTERVAL_MS)
        self._repaint_timer.timeout.connect(self._on_repaint_timeout)

    # ------------------------------------------------------------------
    # static helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _sanitize_window(seconds: float) -> float:
        try:
            value = float(seconds)
        except (TypeError, ValueError):
            return _DEFAULT_WINDOW_S
        if not math.isfinite(value) or value <= 0:
            return _DEFAULT_WINDOW_S
        return max(_MIN_WINDOW_S, value)

    @staticmethod
    def _default_capacity(window_s: float) -> int:
        return max(4096, int(math.ceil(window_s * _MAX_SAMPLES_PER_SECOND)) + 8)

    # ------------------------------------------------------------------
    # public API
    # ------------------------------------------------------------------
    def push_sample(self, t: float, values: Dict[str, float]) -> None:
        """Append one sample; ``t`` is in seconds and must be monotonic.

        Non-finite timestamps or values are ignored (a UI must not die on a bad
        frame), an out-of-order timestamp is dropped and counted, and the ring
        buffer is trimmed to the window.  Painting is *not* triggered here: a
        100 ms coalescing timer schedules at most 10 repaints per second.
        """
        try:
            t = float(t)
        except (TypeError, ValueError):
            return
        if not math.isfinite(t):
            return

        clean: Dict[str, float] = {}
        for key, value in values.items():
            try:
                fv = float(value)
            except (TypeError, ValueError):
                continue
            if math.isfinite(fv):
                clean[str(key)] = fv
        if not clean:
            return
        if self._t_last is not None and t < self._t_last:
            self._out_of_order += 1
            return

        self._buffer.push(t, clean)
        self._t_last = t
        for key in clean:
            if key not in self._known:
                self._known[key] = None
        self._push_count += 1
        if not self._paused:
            self._t_now = t
        # Window trim is amortised O(1): only the expired head is popped.
        self._buffer.trim_before(t - self._window_s)
        if not self._repaint_timer.isActive():
            self._repaint_timer.start()

    def set_channels(self, channels: Sequence[str]) -> None:
        """Draw only these channels (any order); empty means "all seen"."""
        self._channels = [str(c) for c in channels]
        self._schedule_repaint()

    def channels(self) -> List[str]:
        """The explicitly selected channels (may be empty = all)."""
        return list(self._channels)

    def known_channels(self) -> List[str]:
        """Every channel name that has been pushed since the last clear()."""
        return list(self._known)

    def set_window_seconds(self, seconds: float) -> None:
        self._window_s = self._sanitize_window(seconds)
        if self._auto_capacity:
            self._buffer.set_capacity(self._default_capacity(self._window_s))
        if self._t_last is not None:
            self._buffer.trim_before(self._t_last - self._window_s)
        self._schedule_repaint()

    def window_seconds(self) -> float:
        return self._window_s

    def set_colors(self, colors: Sequence[str]) -> None:
        """Override the series colours; the list is cycled per channel."""
        palette = tuple(str(c) for c in colors)
        if palette:
            self._colors = palette
            self._schedule_repaint()

    def set_auto_range(self, enabled: bool = True) -> None:
        self._auto_range = bool(enabled)
        self._schedule_repaint()

    def set_y_range(self, low: float, high: float) -> None:
        """Pin the y axis to ``[low, high]`` (switches auto range off)."""
        low, high = float(low), float(high)
        if not (math.isfinite(low) and math.isfinite(high)) or high <= low:
            return
        self._fixed_range = (low, high)
        self._auto_range = False
        self._schedule_repaint()

    def y_range(self) -> Tuple[float, float]:
        """The range currently in use (auto or fixed), as painted."""
        if self._auto_range:
            lo, hi = self._visible_bounds()
            return _range_from_bounds(lo, hi)
        return self._fixed_range

    def set_paused(self, paused: bool) -> None:
        """Freeze the view (data keeps being recorded and exported).

        While paused ``push_sample`` still appends and trims the buffer, but
        ``t_now`` (the right edge of the axis) stops moving, so the picture
        stays still and catches up on resume.
        """
        paused = bool(paused)
        if paused == self._paused:
            return
        self._paused = paused
        if not paused and self._t_last is not None:
            self._t_now = self._t_last
        self._schedule_repaint()

    def is_paused(self) -> bool:
        return self._paused

    def clear(self) -> None:
        self._buffer.clear()
        self._known.clear()
        self._t_now = 0.0
        self._t_last = None
        self._hover_pos = None
        self._schedule_repaint()

    def export_csv(self, path: str) -> int:
        """Write the buffered window as CSV; return the number of sample rows.

        The header is ``t`` followed by the drawn channels (or every channel
        seen when none were selected).  A channel missing from a sample leaves
        an empty cell rather than a zero.  The directory is created if needed.
        """
        channels = self._export_channels()
        target = os.fspath(path)
        parent = os.path.dirname(os.path.abspath(target))
        if parent and not os.path.isdir(parent):
            os.makedirs(parent, exist_ok=True)
        rows = 0
        with open(target, "w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(["t"] + channels)
            for t, values in self._buffer:
                row = [f"{t:.6f}"]
                for channel in channels:
                    value = values.get(channel)
                    row.append("" if value is None else f"{value:.6f}")
                writer.writerow(row)
                rows += 1
        return rows

    def latest_values(self) -> Dict[str, float]:
        """Newest value per channel inside the visible window.

        Keyed by channel, so a channel that appears in every sample costs one
        dict entry, not one per sample.  Missing channels keep their last known
        value from an older sample in the window.
        """
        out: Dict[str, float] = {}
        cutoff = self._t_now if self._paused else None
        for t, values in reversed(self._buffer):
            if cutoff is not None and t > cutoff:
                continue
            for key, value in values.items():
                if key not in out:
                    out[key] = value
            if len(out) >= len(self._known):
                break
        return out

    def time_now(self) -> float:
        """Right edge of the axis (frozen while paused)."""
        return self._t_now

    def last_sample_time(self) -> Optional[float]:
        """Timestamp of the newest sample that was actually recorded."""
        sample = self._buffer.last()
        return None if sample is None else sample[0]

    def sample_count(self) -> int:
        """Samples currently retained (one window worth)."""
        return len(self._buffer)

    def push_count(self) -> int:
        return self._push_count

    def repaint_count(self) -> int:
        """How many paintEvent calls happened (proves the 10 Hz coalescing)."""
        return self._paint_count

    def hover_readout(self) -> Optional[Tuple[float, Dict[str, float]]]:
        """``(t, values)`` of the sample under the mouse, or ``None``."""
        if self._hover_pos is None:
            return None
        plot = self._plot_rect()
        if not plot.contains(QtCore.QPointF(self._hover_pos)) or not self._buffer:
            return None
        sample = self._nearest_sample(self._time_at_x(self._hover_pos.x()))
        if sample is None:
            return None
        return (sample[0], dict(sample[1]))

    def flush(self) -> None:
        """Paint immediately (used by screenshots and tests, not by the UI)."""
        self._repaint_timer.stop()
        self._dirty = False
        self.repaint()

    # ------------------------------------------------------------------
    # paint scheduling / mouse
    # ------------------------------------------------------------------
    def _schedule_repaint(self) -> None:
        if not QtCore.QCoreApplication.instance():  # pragma: no cover
            return
        if not self._repaint_timer.isActive():
            self._repaint_timer.start()

    def _on_repaint_timeout(self) -> None:
        self.update()

    def mouseMoveEvent(self, event) -> None:  # noqa: N802 - Qt naming
        self._hover_pos = event.pos()
        self._schedule_repaint()
        super().mouseMoveEvent(event)

    def leaveEvent(self, event) -> None:  # noqa: N802 - Qt naming
        self._hover_pos = None
        self._schedule_repaint()
        super().leaveEvent(event)

    # ------------------------------------------------------------------
    # geometry helpers
    # ------------------------------------------------------------------
    def _plot_rect(self) -> "QtCore.QRectF":
        rect = QtCore.QRectF(self.rect())
        left = rect.left() + self.MARGIN_LEFT
        right = max(left + 1.0, rect.right() - self.MARGIN_RIGHT)
        top = rect.top() + self.MARGIN_TOP
        bottom = max(top + 1.0, rect.bottom() - self.MARGIN_BOTTOM)
        return QtCore.QRectF(left, top, right - left, bottom - top)

    def _time_at_x(self, x: float) -> float:
        plot = self._plot_rect()
        if plot.width() <= 0:
            return self._t_now
        frac = (float(x) - plot.left()) / plot.width()
        frac = min(1.0, max(0.0, frac))
        return (self._t_now - self._window_s) + frac * self._window_s

    def _x_at_time(self, t: float) -> float:
        plot = self._plot_rect()
        t_left = self._t_now - self._window_s
        if self._window_s <= 0:
            return plot.right()
        frac = (t - t_left) / self._window_s
        return plot.left() + min(1.0, max(0.0, frac)) * plot.width()

    def _y_at_value(self, value: float, y_range: Tuple[float, float]) -> float:
        plot = self._plot_rect()
        lo, hi = y_range
        span = hi - lo
        if span <= 0:
            return plot.center().y()
        frac = (value - lo) / span
        return plot.bottom() - frac * plot.height()

    def _export_channels(self) -> List[str]:
        if self._channels:
            return list(self._channels)
        return list(self._known)

    def _visible_channels(self) -> List[str]:
        if self._channels:
            return list(self._channels)
        return list(self._known)

    def _color_map(self, channels: Sequence[str]) -> Dict[str, str]:
        palette = self._colors or SERIES_COLORS
        return {ch: palette[i % len(palette)] for i, ch in enumerate(channels)}

    def _visible_bounds(self) -> Tuple[float, float]:
        """Min/max of the visible samples across drawn channels (one pass)."""
        channels = self._visible_channels()
        if not channels or not self._buffer:
            return (math.inf, -math.inf)
        wanted = set(channels)
        t_left = self._t_now - self._window_s
        lo, hi = math.inf, -math.inf
        for t, values in self._buffer:
            if t < t_left or t > self._t_now:
                continue
            for key, value in values.items():
                if key not in wanted:
                    continue
                if value < lo:
                    lo = value
                if value > hi:
                    hi = value
        return (lo, hi)

    def _nearest_sample(self, t_target: float):
        best = None
        best_delta = math.inf
        for t, values in self._buffer:
            delta = abs(t - t_target)
            if delta < best_delta:
                best_delta = delta
                best = (t, values)
            elif t > t_target and delta > best_delta:
                break  # ordered buffer: distance only grows past the target
        return best

    # ------------------------------------------------------------------
    # paintEvent and its parts
    # ------------------------------------------------------------------
    def sizeHint(self) -> "QtCore.QSize":  # noqa: N802 - Qt naming
        return QtCore.QSize(640, 260)

    def minimumSizeHint(self) -> "QtCore.QSize":  # noqa: N802 - Qt naming
        return QtCore.QSize(280, 150)

    def paintEvent(self, event) -> None:  # noqa: N802 - Qt naming
        self._paint_count += 1
        painter = QtGui.QPainter(self)
        try:
            painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
            # Rounded panel, no outline: the parent's dark background shows in
            # the corners, which is what "no frame" means here.
            panel = QtGui.QPainterPath()
            panel.addRoundedRect(
                QtCore.QRectF(self.rect()),
                self.CORNER_RADIUS,
                self.CORNER_RADIUS,
            )
            painter.fillPath(panel, QtGui.QColor(BG_COLOR))
            painter.setClipPath(panel)
            self._draw(painter)
        finally:
            painter.end()

    def _draw(self, painter: "QtGui.QPainter") -> None:
        plot = self._plot_rect()
        if plot.width() <= 4 or plot.height() <= 4:
            return
        channels = self._visible_channels()
        y_range = self.y_range()
        self._draw_grid(painter, plot, y_range)
        self._draw_series(painter, plot, channels, y_range)
        self._draw_legend(painter, plot, channels)
        self._draw_hover(painter, plot, channels, y_range)
        self._draw_status(painter, plot)
        if not self._buffer:
            self._draw_placeholder(painter, plot)

    def _draw_grid(self, painter, plot, y_range) -> None:
        grid_pen = QtGui.QPen(QtGui.QColor(GRID_COLOR), 1.0)
        grid_pen.setCosmetic(True)
        text_pen = QtGui.QPen(QtGui.QColor(AXIS_COLOR), 1.0)
        painter.setFont(self._font_axis)
        lo, hi = y_range
        span = hi - lo
        divisions = 4
        for index in range(divisions + 1):
            frac = index / divisions
            y = plot.top() + frac * plot.height()
            painter.setPen(grid_pen)
            painter.drawLine(QtCore.QPointF(plot.left(), y),
                             QtCore.QPointF(plot.right(), y))
            painter.setPen(text_pen)
            value = hi - frac * span
            label = _axis_text(value, span)
            painter.drawText(
                QtCore.QRectF(plot.left() - self.MARGIN_LEFT + 2.0,
                              y - 7.0,
                              self.MARGIN_LEFT - 8.0,
                              14.0),
                QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter,
                label,
            )
        columns = 5
        for index in range(columns + 1):
            frac = index / columns
            x = plot.left() + frac * plot.width()
            painter.setPen(grid_pen)
            painter.drawLine(QtCore.QPointF(x, plot.top()),
                             QtCore.QPointF(x, plot.bottom()))
            painter.setPen(text_pen)
            relative = -self._window_s * (1.0 - frac)
            if self._window_s < 2.0:
                label = f"{relative:.1f}{self._labels['seconds']}"
            else:
                label = f"{relative:.0f}{self._labels['seconds']}"
            painter.drawText(
                QtCore.QRectF(x - 34.0, plot.bottom() + 3.0, 68.0,
                              self.MARGIN_BOTTOM - 4.0),
                QtCore.Qt.AlignHCenter | QtCore.Qt.AlignTop,
                label,
            )

    def _draw_series(self, painter, plot, channels, y_range) -> None:
        """Walk the buffer once, decimating to <= 1 vertex per pixel column.

        No copy of the sample array is made; the only allocation is the vertex
        list, whose length is bounded by the plot width.
        """
        if not channels or not self._buffer:
            return
        colors = self._color_map(channels)
        t_left = self._t_now - self._window_s
        width = plot.width()
        left = plot.left()
        scale = width / self._window_s if self._window_s > 0 else 0.0
        series: Dict[str, List["QtCore.QPointF"]] = {ch: [] for ch in channels}
        last_px: Dict[str, int] = {}
        lo, hi = y_range
        span = hi - lo if hi > lo else 1.0
        top = plot.top()
        height = plot.height()

        for t, values in self._buffer:
            if t < t_left or t > self._t_now:
                continue
            px = left + (t - t_left) * scale
            ipx = int(px)
            for channel in channels:
                value = values.get(channel)
                if value is None:
                    continue
                py = plot.bottom() - ((value - lo) / span) * height
                if py < top:
                    py = top
                elif py > top + height:
                    py = top + height
                points = series[channel]
                if last_px.get(channel) == ipx and points:
                    # Same pixel column: keep the newest value, not all of them.
                    points[-1] = QtCore.QPointF(px, py)
                else:
                    points.append(QtCore.QPointF(px, py))
                    last_px[channel] = ipx

        painter.save()
        painter.setClipRect(plot.adjusted(-1.0, -1.0, 1.0, 1.0))
        for channel in channels:
            points = series[channel]
            if len(points) < 2:
                continue
            pen = QtGui.QPen(QtGui.QColor(colors[channel]), 1.6)
            pen.setCosmetic(True)
            pen.setCapStyle(QtCore.Qt.RoundCap)
            pen.setJoinStyle(QtCore.Qt.RoundJoin)
            painter.setPen(pen)
            painter.setBrush(QtCore.Qt.NoBrush)
            painter.drawPolyline(QtGui.QPolygonF(points))
        painter.restore()

    def _draw_legend(self, painter, plot, channels) -> None:
        if not channels:
            return
        colors = self._color_map(channels)
        latest = self.latest_values()
        painter.setFont(self._font_legend)
        metrics = painter.fontMetrics()
        swatch = 7.0
        gap = 5.0
        item_gap = 12.0
        y = plot.top() - self.MARGIN_TOP + 2.0
        height = self.MARGIN_TOP - 5.0
        # Right-aligned, packed from the right edge, dropped when out of room.
        x = plot.right()
        for channel in reversed(channels):
            value = latest.get(channel)
            text = f"{channel} {_fmt_value(value) if value is not None else '--'}"
            item_width = swatch + gap + _text_width(metrics, text)
            if x - item_width < plot.left():
                break
            left = x - item_width
            painter.setPen(QtCore.Qt.NoPen)
            painter.setBrush(QtGui.QColor(colors[channel]))
            painter.drawRoundedRect(
                QtCore.QRectF(left, y + (height - swatch) / 2.0, swatch, swatch),
                2.0, 2.0)
            painter.setPen(QtGui.QPen(QtGui.QColor(TEXT_COLOR), 1.0))
            painter.drawText(
                QtCore.QRectF(left + swatch + gap, y, item_width, height),
                QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter, text)
            x = left - item_gap

    def _draw_hover(self, painter, plot, channels, y_range) -> None:
        if self._hover_pos is None or not self._buffer:
            return
        pos = QtCore.QPointF(self._hover_pos)
        if not plot.contains(pos):
            return
        sample = self._nearest_sample(self._time_at_x(pos.x()))
        if sample is None:
            return
        t, values = sample
        x = self._x_at_time(t)
        line_pen = QtGui.QPen(QtGui.QColor(AXIS_COLOR), 1.0)
        line_pen.setCosmetic(True)
        line_pen.setStyle(QtCore.Qt.DashLine)
        painter.setPen(line_pen)
        painter.drawLine(QtCore.QPointF(x, plot.top()),
                         QtCore.QPointF(x, plot.bottom()))

        colors = self._color_map(channels)
        for channel in channels:
            value = values.get(channel)
            if value is None:
                continue
            y = self._y_at_value(value, y_range)
            painter.setPen(QtGui.QPen(QtGui.QColor(colors[channel]), 1.4))
            painter.setBrush(QtGui.QColor(BG_COLOR))
            painter.drawEllipse(QtCore.QPointF(x, y), 3.0, 3.0)

        # Tooltip: time + each drawn channel's value at that sample.
        lines = [(f"t = {t:.3f} {self._labels['seconds']}", None)]
        for channel in channels:
            value = values.get(channel)
            lines.append(
                (f"{channel}  {_fmt_value(value) if value is not None else '--'}",
                 colors[channel]))
        painter.setFont(self._font_hover)
        metrics = painter.fontMetrics()
        pad = 6.0
        line_h = metrics.height() + 1.0
        box_w = max(_text_width(metrics, text) for text, _ in lines) + 2 * pad
        box_h = line_h * len(lines) + 2 * pad
        bx = x + 12.0
        if bx + box_w > plot.right():
            bx = x - 12.0 - box_w
        bx = max(plot.left() + 2.0, min(bx, plot.right() - box_w - 2.0))
        by = plot.top() + 6.0
        if by + box_h > plot.bottom():
            by = max(plot.top() + 2.0, plot.bottom() - box_h - 2.0)
        painter.setPen(QtGui.QPen(QtGui.QColor(GRID_COLOR), 1.0))
        painter.setBrush(QtGui.QColor(CARD_COLOR))
        painter.drawRoundedRect(QtCore.QRectF(bx, by, box_w, box_h), 6.0, 6.0)
        for index, (text, color) in enumerate(lines):
            painter.setPen(QtGui.QPen(
                QtGui.QColor(color if color else MUTED_COLOR), 1.0))
            if index == 0:
                painter.setFont(self._font_hover_mono)
            else:
                painter.setFont(self._font_hover)
            painter.drawText(
                QtCore.QRectF(bx + pad, by + pad + index * line_h,
                              box_w - 2 * pad, line_h),
                QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter, text)

    def _draw_status(self, painter, plot) -> None:
        """Top-left status line, with a paused badge that shifts it aside."""
        labels = self._labels
        top = plot.top() - self.MARGIN_TOP + 2.0
        height = self.MARGIN_TOP - 5.0
        x = plot.left()
        if self._paused:
            badge = labels["paused"]
            painter.setFont(self._font_badge)
            width = _text_width(painter.fontMetrics(), badge) + 14.0
            badge_rect = QtCore.QRectF(x, top + 1.0, width, height - 3.0)
            painter.setPen(QtGui.QPen(QtGui.QColor(WARN_COLOR), 1.0))
            painter.setBrush(QtGui.QColor(60, 40, 14, 220))
            painter.drawRoundedRect(badge_rect, 5.0, 5.0)
            painter.setPen(QtGui.QPen(QtGui.QColor(WARN_COLOR), 1.0))
            painter.drawText(badge_rect, QtCore.Qt.AlignCenter, badge)
            x += width + 8.0
        mode = labels["auto"] if self._auto_range else labels["fixed"]
        text = (f"{labels['samples']}={len(self._buffer)}  "
                f"{labels['window']}={self._window_s:g}s  {mode}")
        painter.setFont(self._font_axis)
        painter.setPen(QtGui.QPen(QtGui.QColor(MUTED_COLOR), 1.0))
        painter.drawText(
            QtCore.QRectF(x, top, max(1.0, plot.right() - x), height),
            QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter, text)

    def _draw_placeholder(self, painter, plot) -> None:
        painter.setFont(self._font_legend)
        painter.setPen(QtGui.QPen(QtGui.QColor(AXIS_COLOR), 1.0))
        painter.drawText(plot, QtCore.Qt.AlignCenter,
                         self._labels["no_data"])
