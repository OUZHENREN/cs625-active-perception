#!/usr/bin/env python3
"""一条命令看清：我们现在有哪些数据、多新、多大、关键指标是什么。

所有者 2026-11-25："我对我能拿到什么数据、数据都放在哪里完全没有掌控感。"
这个脚本把全部数据产品列出来，不再靠记忆猜。详见 docs/reference/data_map.md。

用法
----
    python3 scripts/data_report.py              # 全量清单
    python3 scripts/data_report.py --ros        # 额外检查实时话题是否有数据

退出码：0 正常；1 有数据项完全缺失（提示性，不阻断）。
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import pathlib
import subprocess
import sys
import time

sys.stdout.reconfigure(line_buffering=True)

REPO = pathlib.Path(__file__).resolve().parents[1]
ARCHIVE = pathlib.Path.home() / "cs625_experiment_archive"


def age(seconds: float) -> str:
    if seconds < 90:
        return f"{seconds:.0f} 秒前"
    if seconds < 5400:
        return f"{seconds / 60:.0f} 分钟前"
    if seconds < 172800:
        return f"{seconds / 3600:.1f} 小时前"
    return f"{seconds / 86400:.1f} 天前"


def newest(pattern: str, base: pathlib.Path | None = None) -> pathlib.Path | None:
    paths = sorted(glob.glob(str((base or REPO) / pattern)))
    return pathlib.Path(paths[-1]) if paths else None


def row(name: str, path, detail: str, ok: bool | None = None) -> None:
    mark = "✅" if ok else ("❌" if ok is False else "  ")
    shown = f"  {path}" if path else ""
    print(f"  {mark} {name:22s}{shown}")
    if detail:
        print(f"       {detail}")


def report_measurements(missing: list[str]) -> None:
    print("\n【测量结果】runtime/measurements/*.json")
    files = sorted(glob.glob(str(REPO / "runtime" / "measurements" / "*.json")),
                   key=os.path.getmtime, reverse=True)[:6]
    if not files:
        print("  ❌ 没有任何测量结果")
        missing.append("measurements")
        return
    for path in files:
        p = pathlib.Path(path)
        try:
            d = json.loads(p.read_text(encoding="utf-8"))
            cov = d.get("coverage_median")
            rep = d.get("repeatability") or {}
            detail = (f"{age(time.time() - p.stat().st_mtime)}｜verdict {d.get('verdict')}"
                      f"｜覆盖率 {cov if cov is None else round(float(cov), 3)}"
                      f"｜重复性 {rep.get('translation_mm')} mm / {rep.get('rotation_deg')}°"
                      f"｜dataset {os.path.basename(d.get('dataset') or '')}")
            row(p.name, "", detail, ok=(d.get("verdict") == "OK"))
        except Exception as error:      # noqa: BLE001
            row(p.name, "", f"读取失败：{error}", ok=False)


def report_datasets() -> None:
    print("\n【原始点云/深度/彩色】runtime/vision_dataset/measure_*/g*/")
    dirs = sorted(glob.glob(str(REPO / "runtime" / "vision_dataset" / "measure_*")),
                  key=os.path.getmtime, reverse=True)[:3]
    if not dirs:
        print("  ❌ 没有采集数据集")
        return
    for path in dirs:
        p = pathlib.Path(path)
        groups = sorted(p.glob("g*"))
        first = groups[0] if groups else None
        has_cloud = bool(first and (first / "cloud.npy").exists())
        has_meta = bool(first and (first / "metadata.jsonl").exists())
        size_mb = sum(f.stat().st_size for f in p.rglob("*") if f.is_file()) / 1e6
        row(p.name, "", f"{age(time.time() - p.stat().st_mtime)}｜{len(groups)} 组｜{size_mb:.1f} MB"
                        f"｜cloud.npy {'有' if has_cloud else '无'}"
                        f"｜环境建模用 metadata {'有' if has_meta else '无'}", ok=has_cloud)


def report_experiments(missing: list[str]) -> None:
    print("\n【真机实验】runtime/experiments/（原始）→ ~/cs625_experiment_archive/（归档）")
    raw = sorted(glob.glob(str(REPO / "runtime" / "experiments" / "*")),
                 key=os.path.getmtime, reverse=True)[:2]
    for path in raw:
        p = pathlib.Path(path)
        row(p.name, "", f"未归档原始数据｜{age(time.time() - p.stat().st_mtime)}")
    archived = sorted(glob.glob(str(ARCHIVE / "2026*")), key=os.path.getmtime, reverse=True)[:3]
    if not archived:
        print("  ❌ 没有归档实验")
        missing.append("archive")
        return
    for path in archived:
        p = pathlib.Path(path)
        csv = p / "timeseries.csv"
        rows = sum(1 for _ in csv.open()) - 1 if csv.exists() else 0
        imgs = len(list((p / "images").glob("*"))) if (p / "images").is_dir() else 0
        phases = 0
        if (p / "phases.json").exists():
            try:
                phases = len(json.loads((p / "phases.json").read_text(encoding="utf-8")))
            except Exception:      # noqa: BLE001
                phases = 0
        # 力矩列是否有值（这是所有者最关心的"力数据在哪"）
        effort_ok = False
        if csv.exists():
            with csv.open() as f:
                header = f.readline().strip().split(",")
                body = f.readline().strip().split(",")
                if "joint_effort_1" in header:
                    idx = header.index("joint_effort_1")
                    effort_ok = bool(body[idx].strip())
        row(p.name, "", f"{age(time.time() - p.stat().st_mtime)}｜时序 {rows} 行｜图像 {imgs} 张"
                        f"｜阶段 {phases} 条｜关节力矩列{'有值' if effort_ok else '空'}",
            ok=effort_ok)


def report_environment() -> None:
    print("\n【环境模型】runtime/environment/")
    latest = REPO / "runtime" / "environment" / "latest.json"
    if not latest.exists():
        print("  ❌ 没有环境模型（observe:tables / model:slot 未跑过）")
        return
    try:
        payload = json.loads(latest.read_text(encoding="utf-8"))
        model = REPO / "runtime" / "environment" / payload["model"]
        when = time.time() - model.stat().st_mtime
        detail = f"{age(when)}｜模型 {payload['latest']}"
        if when > 7200:
            detail += "｜⚠ 超过 2 小时新鲜度门限，环境门会拒绝"
        d = json.loads(model.read_text(encoding="utf-8"))
        for key in ("workpiece_table", "slot_table"):
            if isinstance(d.get(key), dict) and "z" in d[key]:
                detail += f"｜{key} z={d[key]['z']:.4f} m"
        row("model.json", "", detail, ok=(when <= 7200))
    except Exception as error:      # noqa: BLE001
        row("model.json", "", f"读取失败：{error}", ok=False)


def report_calibration() -> None:
    print("\n【标定与关键配置】")
    for name, path in (
        ("手眼标定", REPO / "config" / "handeye" / "tcp_to_camera.json"),
        ("插入标定", REPO / "runtime" / "insert_calibration.json"),
        ("插槽测量基线", REPO / "runtime" / "slot_measurement_baseline.json"),
        ("六维力配置", REPO / "config" / "ft_sensor" / "kwr75.yaml"),
    ):
        ok = path.exists()
        detail = age(time.time() - path.stat().st_mtime) if ok else "尚未生成"
        row(name, path.relative_to(REPO) if ok else "", detail, ok=ok)


def report_ros() -> None:
    print("\n【实时话题】（--ros）")
    for topic in ("/joint_states", "/tcp_pose_broadcaster/pose", "/ft_sensor/wrench"):
        try:
            out = subprocess.run(["timeout", "6", "ros2", "topic", "hz", topic, "-w", "2"],
                                 capture_output=True, text=True).stdout.strip()
            has = "average rate" in out
            row(topic, "", (out.splitlines()[-1] if out else "无输出"),
                ok=has)
        except Exception as error:      # noqa: BLE001
            row(topic, "", f"检查失败：{error}", ok=False)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ros", action="store_true", help="额外检查实时话题")
    args = parser.parse_args()

    print(f"数据现状（{time.strftime('%Y-%m-%d %H:%M:%S')}）")
    print("完整说明见 docs/reference/data_map.md")
    missing: list[str] = []
    report_measurements(missing)
    report_datasets()
    report_experiments(missing)
    report_environment()
    report_calibration()
    if args.ros:
        report_ros()
    print()
    if missing:
        print(f"提示：以下数据项当前为空：{missing}")
    print("下一步：① 论文图从归档 timeseries.csv + images/ 出；"
          "② 六维力接上后 fx fy fz tx ty tz 会自动填充")
    return 1 if missing else 0


if __name__ == "__main__":
    raise SystemExit(main())
