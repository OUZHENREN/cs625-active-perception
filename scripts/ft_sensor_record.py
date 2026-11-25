#!/usr/bin/env python3
"""六维力数据全速率记录 + 出图（2026-11-25）。

用途
----
`experiment_recorder` 只有 20 Hz，做力控/接触检测时不够；而这个脚本按话题真实速率
（KWR75 流模式可达 1 kHz）把六维力写进 CSV，并可直接画图，供论文与调试用。

用法
----
    # 终端 A：先起传感器节点
    python3 scripts/ft_sensor_node.py
    # 终端 B：记录（默认写到 runtime/ft_logs/<时间戳>.csv）
    python3 scripts/ft_sensor_record.py --duration 10
    python3 scripts/ft_sensor_record.py --duration 10 --plot
    python3 scripts/ft_sensor_record.py --csv <已有csv> --plot   # 只画图，不采集

输出
----
    runtime/ft_logs/<时间戳>.csv   列：t_wall, t_ros, fx, fy, fz, tx, ty, tz
    runtime/ft_logs/<时间戳>.png   （--plot：六个分量各一条曲线）
终端最后打印统计：均值/标准差/最小/最大 + 速率 —— 用来判断零漂与噪声水平。
"""

from __future__ import annotations

import argparse
import csv
import pathlib
import statistics
import sys
import time

sys.stdout.reconfigure(line_buffering=True)

REPO = pathlib.Path(__file__).resolve().parents[1]
OUTDIR = REPO / "runtime" / "ft_logs"
COLUMNS = ["t_wall", "t_ros", "fx", "fy", "fz", "tx", "ty", "tz"]


def summarise(rows: list[list[float]]) -> dict:
    stats = {}
    for index, name in enumerate(COLUMNS[2:], start=2):
        values = [row[index] for row in rows]
        stats[name] = {"mean": statistics.fmean(values),
                       "std": statistics.pstdev(values) if len(values) > 1 else 0.0,
                       "min": min(values), "max": max(values)}
    return stats


def print_summary(stats: dict, rate: float, path: pathlib.Path) -> None:
    print(f"\n记录完成：{path}（{rate:.1f} Hz）")
    print(f"  {'分量':6s}{'均值':>12s}{'标准差':>12s}{'最小':>12s}{'最大':>12s}")
    for name, s in stats.items():
        print(f"  {name:6s}{s['mean']:12.4f}{s['std']:12.4f}{s['min']:12.4f}{s['max']:12.4f}")
    # 零漂/噪声判据：静止空载时标准差应远小于量程
    worst = max(abs(s["mean"]) for s in stats.values())
    print(f"  最大静止偏置 |均值| = {worst:.4f}（空载静止时应接近 0；偏大说明未清零或未装正）")


def plot(csv_path: pathlib.Path) -> pathlib.Path:
    import matplotlib
    matplotlib.use("Agg")            # 无显示环境下也能出图
    import matplotlib.pyplot as plt

    with csv_path.open(encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise SystemExit(f"{csv_path} 没有数据")
    t = [float(r["t_wall"]) for r in rows]
    t0 = t[0]
    labels = {"fx": "Fx (N)", "fy": "Fy (N)", "fz": "Fz (N)",
              "tx": "Mx (Nm)", "ty": "My (Nm)", "tz": "Mz (Nm)"}
    unit = {"fx": "force", "fy": "force", "fz": "force",
            "tx": "torque", "ty": "torque", "tz": "torque"}
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    for key in ("fx", "fy", "fz"):
        axes[0].plot([x - t0 for x in t], [float(r[key]) for r in rows], label=labels[key])
    for key in ("tx", "ty", "tz"):
        axes[1].plot([x - t0 for x in t], [float(r[key]) for r in rows], label=labels[key])
    axes[0].set_ylabel("Force (N)"); axes[1].set_ylabel("Torque (Nm)")
    axes[1].set_xlabel("Time (s)")
    for axis in axes:
        axis.grid(alpha=0.3); axis.legend(loc="upper right", fontsize=9)
    fig.suptitle(f"KWR75 six-axis force log - {csv_path.name}")
    fig.tight_layout()
    out = csv_path.with_suffix(".png")
    fig.savefig(out, dpi=130)
    print(f"图已保存：{out}")
    return out


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--topic", default="/ft_sensor/wrench")
    parser.add_argument("--duration", type=float, default=10.0, help="记录时长（秒）")
    parser.add_argument("--out", default=None, help="CSV 路径（默认 runtime/ft_logs/<时间戳>.csv）")
    parser.add_argument("--csv", default=None, help="已有 CSV：跳过采集，只做统计/出图")
    parser.add_argument("--plot", action="store_true", help="记完直接出 PNG")
    args = parser.parse_args()

    if args.csv:
        path = pathlib.Path(args.csv)
        with path.open(encoding="utf-8") as handle:
            rows = [[float(r[c]) for c in COLUMNS] for r in csv.DictReader(handle)]
        span = rows[-1][0] - rows[0][0] if len(rows) > 1 else 1e-6
        print_summary(summarise(rows), len(rows) / max(1e-6, span), path)
        if args.plot:
            plot(path)
        return 0

    import rclpy
    from geometry_msgs.msg import WrenchStamped
    from rclpy.node import Node

    rclpy.init(args=None)
    node = Node("ft_sensor_record")
    rows: list[list[float]] = []

    def on_msg(msg: WrenchStamped) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        rows.append([time.time(), stamp,
                     msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
                     msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z])

    node.create_subscription(WrenchStamped, args.topic, on_msg, 100)
    print(f"采集 {args.topic}，{args.duration:.0f} s …（Ctrl+C 提前结束）")
    started = time.time()
    try:
        while rclpy.ok() and time.time() - started < args.duration:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:            # noqa: BLE001
        pass

    if not rows:
        print(f"没收到任何数据：{args.topic} 是否有发布者？（先起 scripts/ft_sensor_node.py）")
        return 2
    OUTDIR.mkdir(parents=True, exist_ok=True)
    path = pathlib.Path(args.out) if args.out else \
        OUTDIR / f"{time.strftime('%Y%m%d_%H%M%S')}.csv"
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(COLUMNS)
        writer.writerows(rows)
    span = rows[-1][0] - rows[0][0] if len(rows) > 1 else 1e-6
    print_summary(summarise(rows), len(rows) / max(1e-6, span), path)
    if args.plot:
        plot(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
