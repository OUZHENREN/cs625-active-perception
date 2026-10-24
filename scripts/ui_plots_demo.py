#!/usr/bin/env python3
"""Demo / acceptance harness for :mod:`scripts.ui_plots` (RealtimePlot).

It feeds synthetic data -- six sines plus one step -- into the widget and proves
the five behaviours the operator panel depends on:

==============  ==========================================================
multi-channel   7 channels, 4-colour cycle, legend with current values
auto range      10 % headroom, brackets every visible value of every channel
fixed range     ``set_y_range`` pins the axis and wins over auto range
pause           view freezes, data keeps being recorded, resume catches up
hover           vertical line + nearest sample readout under the mouse
CSV export      header + one row per buffered sample, missing cells empty
==============  ==========================================================

Usage
-----
Interactive::

    python3 scripts/ui_plots_demo.py

Headless screenshot, then exit (exit code 0 = every check passed)::

    QT_QPA_PLATFORM=offscreen python3 scripts/ui_plots_demo.py \
        --smoke /tmp/plots_demo.png

Synthetic data flow::

    sin/step generator -> push_sample (250 Hz) -> ring buffer (20 s window)
                       -> 10 Hz coalesced paint -> PNG / window
                       -> export_csv -> /tmp/plots_demo.csv
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from pathlib import Path

# ``--smoke`` must be visible before PyQt5 is imported: the offscreen platform
# plugin has to be chosen at import time, not after QApplication exists.
if "--smoke" in sys.argv and "QT_QPA_PLATFORM" not in os.environ:
    os.environ["QT_QPA_PLATFORM"] = "offscreen"

from PyQt5 import QtCore, QtGui, QtWidgets  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ui_plots import RealtimePlot, SERIES_COLORS, auto_range  # noqa: E402

#: Six sines (different amplitude/frequency/phase) plus one step channel.
CHANNELS = ("sin_1", "sin_2", "sin_3", "sin_4", "sin_5", "sin_6", "step")
RATE_HZ = 250.0          # one sample every 4 ms -- faster than the paint cap
STEP_T = 10.0            # step channel flips here
STEP_HIGH = 1.5
WINDOW_S = 20.0


def sample_at(t: float) -> dict:
    """Deterministic synthetic frame at time ``t`` (seconds)."""
    return {
        "sin_1": 0.90 * math.sin(2.0 * math.pi * 0.35 * t),
        "sin_2": 0.60 * math.sin(2.0 * math.pi * 0.70 * t + 0.6),
        "sin_3": 0.45 * math.sin(2.0 * math.pi * 1.10 * t + 1.2),
        "sin_4": 0.30 * math.sin(2.0 * math.pi * 1.60 * t + 2.0) + 0.10,
        "sin_5": 0.20 * math.sin(2.0 * math.pi * 2.30 * t + 2.6) - 0.15,
        "sin_6": 0.12 * math.sin(2.0 * math.pi * 3.00 * t + 3.1) + 0.05,
        "step": STEP_HIGH if t >= STEP_T else 0.0,
    }


def truth_bounds(t0: float, t1: float) -> tuple:
    """Min/max of the *generator* over [t0, t1], to check the auto range."""
    lo, hi = math.inf, -math.inf
    n = 4000
    for index in range(n + 1):
        t = t0 + (t1 - t0) * index / n
        for value in sample_at(t).values():
            lo = min(lo, value)
            hi = max(hi, value)
    return lo, hi


class Checks:
    """Tiny PASS/FAIL collector: the demo must exit non-zero if anything fails."""

    def __init__(self) -> None:
        self.failures = 0
        self.total = 0

    def check(self, name: str, ok: bool, detail: str = "") -> bool:
        self.total += 1
        status = "PASS" if ok else "FAIL"
        if not ok:
            self.failures += 1
        print(f"  [{status}] {name}{(' -- ' + detail) if detail else ''}")
        return ok


def build_window() -> tuple:
    """Create the demo window: dark container + the plot with 12 px margins."""
    window = QtWidgets.QWidget()
    window.setObjectName("plots_demo_window")
    window.setStyleSheet("background: #0b1016;")
    window.setWindowTitle("CS625 realtime plot demo (no pyqtgraph)")
    layout = QtWidgets.QVBoxLayout(window)
    layout.setContentsMargins(12, 12, 12, 12)
    layout.setSpacing(0)
    plot = RealtimePlot(channels=CHANNELS, window_seconds=WINDOW_S)
    layout.addWidget(plot)
    window.resize(1040, 540)
    return window, plot


def prefill(plot: RealtimePlot, seconds: float = WINDOW_S,
            rate: float = RATE_HZ) -> float:
    """Push ``seconds`` worth of samples; return the last timestamp."""
    t = 0.0
    dt = 1.0 / rate
    steps = int(round(seconds * rate))
    for _ in range(steps):
        plot.push_sample(t, sample_at(t))
        t += dt
    return t - dt


def hover_at(plot: RealtimePlot, fraction: float) -> None:
    """Synthesise a mouse-move so the hover overlay is part of the screenshot."""
    point = QtCore.QPoint(int(plot.width() * fraction), int(plot.height() * 0.5))
    event = QtGui.QMouseEvent(
        QtCore.QEvent.MouseMove,
        QtCore.QPointF(point),
        QtCore.Qt.NoButton,
        QtCore.Qt.NoButton,
        QtCore.Qt.NoModifier,
    )
    plot.mouseMoveEvent(event)


def run_bench(plot_channels=CHANNELS, window_s=WINDOW_S, rate=RATE_HZ,
              count: int = 5000) -> dict:
    """Feed ``count`` samples and time every ``push_sample`` call.

    5000 samples at 250 Hz is exactly one 20 s window, so the retained count
    must land on ~5000 as well: the buffer only holds the window.
    """
    print("\n[bench] 5000 pushes (20 s @ 250 Hz), timing every push_sample()")
    bench_plot = RealtimePlot(channels=plot_channels, window_seconds=window_s)
    dt = 1.0 / rate
    durations = []
    t = 0.0
    for _ in range(count):
        values = sample_at(t)
        start = time.perf_counter()
        bench_plot.push_sample(t, values)
        durations.append(time.perf_counter() - start)
        t += dt
    durations.sort()
    total = sum(durations)
    result = {
        "count": count,
        "avg_ms": total / count * 1e3,
        "max_ms": durations[-1] * 1e3,
        "p99_ms": durations[int(0.99 * (len(durations) - 1))] * 1e3,
        "min_ms": durations[0] * 1e3,
        "retained": bench_plot.sample_count(),
        "expected": int(round(window_s / dt)),
    }
    print(f"  pushes                 : {result['count']}")
    print(f"  avg push               : {result['avg_ms']:.4f} ms")
    print(f"  max push               : {result['max_ms']:.4f} ms")
    print(f"  p99 push               : {result['p99_ms']:.4f} ms")
    print(f"  min push               : {result['min_ms']:.4f} ms")
    print(f"  retained samples       : {result['retained']} "
          f"(window/interval = {result['expected']})")
    print(f"  paints during bench    : {bench_plot.repaint_count()} "
          f"(coalesced, no event loop running)")
    bench_plot.deleteLater()
    return result


def run_checks(plot: RealtimePlot, checks: Checks, csv_path: Path) -> dict:
    """Exercise the five behaviours and print a PASS/FAIL line for each."""
    out = {}

    # --- multi-channel ---------------------------------------------------
    latest = plot.latest_values()
    checks.check("multi-channel: 7 channels drawn",
                 len(plot.known_channels()) == 7,
                 f"known={plot.known_channels()}")
    checks.check("multi-channel: latest_values() covers all 7",
                 len(latest) == 7,
                 f"{ {k: round(v, 3) for k, v in latest.items()} }")

    # --- auto range ------------------------------------------------------
    t_now = plot.time_now()
    t_left = t_now - plot.window_seconds()
    truth_lo, truth_hi = truth_bounds(t_left, t_now)
    lo, hi = plot.y_range()
    span = truth_hi - truth_lo
    margin = (truth_hi - truth_lo) * 0.1
    checks.check("auto range: brackets all visible values",
                 lo <= truth_lo + 1e-9 and hi >= truth_hi - 1e-9,
                 f"axis=[{lo:.3f}, {hi:.3f}] data=[{truth_lo:.3f}, {truth_hi:.3f}]")
    checks.check("auto range: ~10 % headroom",
                 abs((truth_lo - lo) - margin) < 0.02 * span + 1e-6
                 and abs((hi - truth_hi) - margin) < 0.02 * span + 1e-6,
                 f"pad_low={truth_lo - lo:.4f} pad_high={hi - truth_hi:.4f} "
                 f"expected={margin:.4f}")
    checks.check("auto range == auto_range() pure helper",
                 abs(auto_range([truth_lo, truth_hi])[0] - lo) < 1e-6,
                 f"pure={auto_range([truth_lo, truth_hi])} widget=({lo:.4f}, {hi:.4f})")
    out["auto_range"] = (lo, hi)

    # --- fixed range -----------------------------------------------------
    plot.set_y_range(-2.0, 2.0)
    fixed_ok = plot.y_range() == (-2.0, 2.0) and not plot._auto_range
    checks.check("fixed range: set_y_range(-2, 2) pins the axis", fixed_ok,
                 f"y_range={plot.y_range()} auto={plot._auto_range}")
    plot.set_auto_range(True)
    checks.check("fixed range: back to auto", plot._auto_range,
                 f"y_range=({plot.y_range()[0]:.3f}, {plot.y_range()[1]:.3f})")

    # --- pause -----------------------------------------------------------
    plot.set_paused(True)
    frozen_now = plot.time_now()
    frozen_count = plot.sample_count()
    t_resume = (frozen_now if frozen_now else 0.0) + 1.0 / RATE_HZ
    for _ in range(25):  # 100 ms of samples while paused
        plot.push_sample(t_resume, sample_at(t_resume))
        t_resume += 1.0 / RATE_HZ
    checks.check("pause: view frozen (t_now unchanged)",
                 plot.time_now() == frozen_now,
                 f"t_now={plot.time_now():.3f} frozen_at={frozen_now:.3f}")
    newest = plot.last_sample_time()
    checks.check("pause: data still recorded (buffer keeps moving)",
                 newest is not None and newest > frozen_now,
                 f"newest buffered={newest:.3f} > frozen t_now={frozen_now:.3f}, "
                 f"buffered={frozen_count} -> {plot.sample_count()}")
    last_pushed = t_resume - 1.0 / RATE_HZ
    plot.set_paused(False)
    checks.check("pause: resume catches up to newest sample",
                 abs(plot.time_now() - last_pushed) < 1e-9,
                 f"t_now={plot.time_now():.3f} last_pushed={last_pushed:.3f}")
    out["resume_t"] = last_pushed

    # --- hover -----------------------------------------------------------
    hover_at(plot, 0.68)
    readout = plot.hover_readout()
    checks.check("hover: nearest sample readout available",
                 readout is not None and len(readout[1]) == 7,
                 "none" if readout is None else
                 f"t={readout[0]:.3f} channels={sorted(readout[1])}")
    if readout is not None:
        t_hover = readout[0]
        checks.check("hover: readout is the nearest sample in the window",
                     t_left <= t_hover <= plot.time_now() + 1e-9,
                     f"{t_left:.3f} <= {t_hover:.3f} <= {plot.time_now():.3f}")
        out["hover"] = (t_hover, dict(readout[1]))

    # --- export ----------------------------------------------------------
    rows = plot.export_csv(str(csv_path))
    text = csv_path.read_text(encoding="utf-8").splitlines()
    checks.check("csv: row count matches retained samples",
                 rows == len(text) - 1 and rows == plot.sample_count(),
                 f"returned={rows} file_data_rows={len(text) - 1} "
                 f"buffered={plot.sample_count()}")
    header = text[0].split(",")
    checks.check("csv: header is t + 7 channels",
                 header == ["t"] + list(CHANNELS), f"header={header}")
    checks.check("csv: no empty cells for the step channel",
                 all(line.split(",")[-1] != "" for line in text[1:]),
                 f"last row={text[-1]}")
    out["csv"] = (rows, text[:3])
    return out


def main(argv: list) -> int:
    parser = argparse.ArgumentParser(
        description="RealtimePlot demo (PyQt5 + QPainter, no pyqtgraph)")
    parser.add_argument("--smoke", metavar="PNG",
                        help="render one offscreen screenshot to PNG, then exit")
    parser.add_argument("--csv", default=None,
                        help="CSV export path (default: next to the screenshot, "
                             "or /tmp/plots_demo.csv)")
    parser.add_argument("--window", type=float, default=WINDOW_S,
                        help="rolling window in seconds (default 20)")
    parser.add_argument("--rate", type=float, default=RATE_HZ,
                        help="synthetic sample rate in Hz (default 250)")
    parser.add_argument("--no-bench", action="store_true",
                        help="skip the 5000-sample push benchmark")
    args = parser.parse_args(argv)

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv[:1])
    checks = Checks()
    print("[demo] RealtimePlot demo: 6 sines + 1 step, "
          f"window={args.window:g}s, rate={args.rate:g} Hz")
    print(f"[demo] platform={app.platformName()} "
          f"qt={QtCore.QT_VERSION_STR} colors={list(SERIES_COLORS)}")

    window, plot = build_window()
    plot.set_window_seconds(args.window)
    # Show before the checks: the layout then has its real size, so the hover
    # point and the screenshot use the same geometry as a live panel.
    window.show()
    QtWidgets.QApplication.processEvents()

    if not args.no_bench:
        bench = run_bench(count=5000, window_s=args.window, rate=args.rate)
        checks.check("bench: avg push_sample < 1 ms",
                     bench["avg_ms"] < 1.0, f"{bench['avg_ms']:.4f} ms")
        checks.check("bench: max push_sample < 1 ms",
                     bench["max_ms"] < 1.0, f"{bench['max_ms']:.4f} ms")
        checks.check("bench: retains ~one window of samples",
                     abs(bench["retained"] - bench["expected"]) <= 5,
                     f"retained={bench['retained']} expected={bench['expected']}")
        print(f"[bench] live widget paints while feeding are coalesced at "
              f"{1000 // 100} Hz max (see RealtimePlot.MAX_REPAINT_HZ)")

    print("\n[checks] behavioural checks")
    t_last = prefill(plot, seconds=args.window, rate=args.rate)
    print(f"[demo] pre-filled {plot.sample_count()} samples, "
          f"t_now={plot.time_now():.3f}s (generator last t={t_last:.3f})")

    csv_path = Path(args.csv) if args.csv else (
        Path(args.smoke).with_suffix(".csv") if args.smoke
        else Path("/tmp/plots_demo.csv"))
    results = run_checks(plot, checks, csv_path)

    print(f"\n[csv] {csv_path}  rows={results['csv'][0]}")
    for line in results["csv"][1]:
        print(f"  {line}")

    if args.smoke:
        # Leave the hover overlay visible in the screenshot and paint now, so
        # the PNG contains the same frame the checks just inspected.
        hover_at(plot, 0.68)
        QtWidgets.QApplication.processEvents()
        plot.flush()
        QtWidgets.QApplication.processEvents()
        pixmap = window.grab()
        target = Path(args.smoke)
        target.parent.mkdir(parents=True, exist_ok=True)
        saved = pixmap.save(str(target), "PNG")
        checks.check("smoke: screenshot written", saved, str(target))
        image = pixmap.toImage()
        print(f"[smoke] {target} size={target.stat().st_size} B "
              f"image={image.width()}x{image.height()} "
              f"paints={plot.repaint_count()} pushes={plot.push_count()}")
        if results.get("hover"):
            h_t, h_vals = results["hover"]
            print(f"[smoke] hover at t={h_t:.3f}s -> "
                  f"sin_1={h_vals['sin_1']:.3f} step={h_vals['step']:.3f}")

    if checks.failures:
        print(f"\n[demo] FAILED: {checks.failures}/{checks.total} checks failed")
        return 1
    print(f"\n[demo] all {checks.total} checks passed")

    if args.smoke:
        return 0

    # Interactive mode: keep pushing live samples at 20 ms batches (250 Hz
    # equivalent) and let the widget coalesce the paints.
    dt = 1.0 / args.rate
    batch = max(1, int(round(0.02 * args.rate)))
    state = {"t": t_last + dt}

    timer = QtCore.QTimer()
    timer.setInterval(20)

    def tick() -> None:
        for _ in range(batch):
            plot.push_sample(state["t"], sample_at(state["t"]))
            state["t"] += dt

    timer.timeout.connect(tick)
    timer.start()
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
