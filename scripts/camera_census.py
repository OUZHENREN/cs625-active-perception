#!/usr/bin/env python3
"""相机档位普查（D0）：逐档换档 → 采帧 → 存图 → 出指标表。

为什么要有这一步
----------------
整个月的位姿精度都建立在"相机给什么数据"之上。仓库里关于 1280×960 的结论曾经是
"深度有效点≈0，所以退回 640×480"，而那条测量早于 `laser_power:=100` 落地。
与其继续引用旧结论，不如把每一档都实测一遍：分辨率、帧率、有效点比例、中位深度、
点距、空洞率，并把原始图像与深度存下来，作为初步实验的记录。

产物（每档一个子目录 + 一份汇总）
--------------------------------
    runtime/experiments/camera_census_<时间戳>/
        summary.md            人读的结论表
        metrics.csv           机器读的指标表
        fast/color_000.png    一帧原始彩色
        fast/depth_000.npy    uint16 毫米深度（原样保存，不做缩放）
        fast/meta.json        内参、帧数、帧率、采集时间
        modeling/…  high/…

换档复用 `scripts/camera_mode.py`（它只动 runtime/camera.pid 记录的相机进程），
本脚本不碰机械臂、不碰夹爪、不碰 MoveIt。
"""

from __future__ import annotations

import argparse
import csv
import json
import pathlib
import subprocess
import sys
import time

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import polarizer_ab as pab                       # 复用指标实现，避免两套口径

REPO = pathlib.Path(__file__).resolve().parents[1]
CAMERA_MODE = REPO / "scripts" / "camera_mode.py"
DEFAULT_ROOT = REPO / "runtime" / "experiments"
REFERENCE_INTRINSICS = REPO / "config" / "real" / "camera_intrinsics.json"
MODES = ("fast", "modeling", "high")


def _qos(rclpy_qos):
    profile = rclpy_qos.QoSProfile(depth=5, history=rclpy_qos.HistoryPolicy.KEEP_LAST)
    profile.reliability = rclpy_qos.ReliabilityPolicy.RELIABLE
    return profile


def capture(frames: int, timeout: float) -> dict:
    """采 N 帧彩色+深度；返回内存中的数组与时间戳，不落盘。"""
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CameraInfo, Image

    rclpy.init(args=None)
    node = rclpy.create_node("camera_census")
    qos = _qos(rclpy.qos)
    state = {"depth": [], "colour": [], "colour_stamp": [], "depth_stamp": [],
             "info": None, "encoding": ""}
    seen: set[int] = set()

    def on_info(msg):
        if state["info"] is None:
            state["info"] = {"k": list(msg.k), "width": msg.width, "height": msg.height}

    def on_colour(msg):
        try:
            pixels = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(
                msg.height, msg.width, 3).copy()
        except ValueError:
            return
        if len(state["colour"]) < max(4, frames) * 2:      # 只在需要时留帧，避免爆内存
            state["colour"].append(pixels)
            state["colour_stamp"].append(float(msg.header.stamp.sec)
                                         + float(msg.header.stamp.nanosec) * 1e-9)
        else:
            return

    def on_depth(msg):
        key = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        if key in seen:
            return
        seen.add(key)
        state["encoding"] = msg.encoding
        if msg.encoding == "16UC1":
            depth_mm = np.frombuffer(bytes(msg.data), dtype=np.uint16).reshape(
                msg.height, msg.width).copy()
        elif msg.encoding == "32FC1":
            metres = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(
                msg.height, msg.width)
            depth_mm = np.where(np.isfinite(metres), metres * 1000.0, 0.0).astype(np.uint16)
        else:
            return
        state["depth"].append(depth_mm)
        state["depth_stamp"].append(float(msg.header.stamp.sec)
                                    + float(msg.header.stamp.nanosec) * 1e-9)

    node.create_subscription(Image, "/camera/color/image_raw", on_colour, qos)
    node.create_subscription(Image, "/camera/depth/image_raw", on_depth, qos)
    node.create_subscription(CameraInfo, "/camera/color/camera_info", on_info, qos)

    deadline = time.time() + timeout
    colour_done = None
    while time.time() < deadline:
        if len(state["depth"]) >= frames and len(state["colour"]) >= 1:
            break
        rclpy.spin_once(node, timeout_sec=0.2)
        now = time.time()
        if len(state["colour"]) >= frames and colour_done is None:
            colour_done = now
        # 有些档位只开彩色（camera_mode 的 fast 就是）：彩色够了、深度 6 s 还没来，
        # 就按"该档无深度"记录，不要空等满 capture_timeout。
        if colour_done is not None and not state["depth"] and now - colour_done > 6.0:
            break
    node.destroy_node()
    rclpy.shutdown()
    state["depth"] = state["depth"][:frames]
    state["colour"] = state["colour"][:frames]
    state["colour_stamp"] = state["colour_stamp"][:frames]
    state["depth_stamp"] = state["depth_stamp"][:frames]
    return state


def rate(stamps: list[float]) -> float:
    if len(stamps) < 2:
        return 0.0
    span = stamps[-1] - stamps[0]
    return (len(stamps) - 1) / span if span > 1e-6 else 0.0


def analyse(state: dict) -> dict:
    """一档的指标：分辨率、帧率、有效点、中位深度、点距、空洞率、亮度。"""
    result: dict = {"frames": len(state["depth"]), "encoding": state["encoding"]}
    if state["info"] is None:
        result["error"] = "没有收到 camera_info（相机没起 / 话题不对 / QoS 不匹配）"
        return result
    if not state["depth"]:
        if state["colour"]:
            result.update(width=state["colour"][0].shape[1],
                          height=state["colour"][0].shape[0],
                          colour_fps=round(rate(state["colour_stamp"]), 3),
                          depth_fps=0.0,
                          colour_mean=round(float(state["colour"][0][..., :3].mean()), 2),
                          note="该档只开彩色（无深度流）")
            return result
        result["error"] = "没有收到深度帧"
        return result
    k = state["info"]["k"]
    height, width = state["depth"][0].shape
    result.update(width=width, height=height,
                  colour_fps=round(rate(state["colour_stamp"]), 3),
                  depth_fps=round(rate(state["depth_stamp"]), 3))

    valid_ratios, medians, flyings, planarity, holes, spacings = [], [], [], [], [], []
    for depth_mm in state["depth"]:
        depth_m = depth_mm.astype(np.float32) / 1000.0
        valid = depth_m > 0.0
        valid_ratios.append(float(valid.mean()))
        if valid.any():
            medians.append(float(np.median(depth_m[valid])))
        points = pab.back_project(depth_m, k)
        if len(points) > 100:
            flying, planar = pab.flying_stats(points)
            flyings.append(flying)
            planarity.append(planar)
            spacings.append(pab.spacing_mm(points))
        holes.append(pab.hole_ratio(depth_m, k, None))
    result.update(
        valid_ratio=round(float(np.nanmedian(valid_ratios)), 4),
        median_m=round(float(np.nanmedian(medians)), 4) if medians else None,
        flying=round(float(np.nanmedian(flyings)), 4) if flyings else None,
        planarity=round(float(np.nanmedian(planarity)), 4) if planarity else None,
        spacing_mm=round(float(np.nanmedian(spacings)), 3) if spacings else None,
        hole_ratio=round(float(np.nanmedian(holes)), 4),
    )
    if width > 1280:
        result["note"] = "深度图已按彩色分辨率重采样（插值，不增加真实深度分辨率）"
    if state["colour"]:
        result["colour_mean"] = round(float(state["colour"][0][..., :3].mean()), 2)
    if REFERENCE_INTRINSICS.exists() and (width, height) == (1280, 960):
        reference = json.loads(REFERENCE_INTRINSICS.read_text())
        result["intrinsics_match_reference"] = all(
            abs(float(k[index]) - float(reference[key])) < 1e-3
            for index, key in ((0, "fx"), (4, "fy"), (2, "cx"), (5, "cy")))
    return result


def switch_mode(mode: str, timeout: float) -> tuple[int, list[str]]:
    try:
        finished = subprocess.run([sys.executable, str(CAMERA_MODE), mode],
                                  capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return 124, [f"camera_mode.py {mode} 超时（{timeout:.0f}s）"]
    lines = [line for line in (finished.stdout + finished.stderr).splitlines() if line.strip()]
    return finished.returncode, lines


def write_summary(root: pathlib.Path, rows: list[dict], log: list[str]) -> None:
    headers = ["mode", "width", "height", "colour_fps", "depth_fps", "valid_ratio",
               "median_m", "spacing_mm", "flying", "hole_ratio", "colour_mean", "error"]
    with (root / "metrics.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=headers, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in headers})

    lines = ["# 相机档位普查（D0）", "",
             f"采集时间：{time.strftime('%Y-%m-%d %H:%M:%S')}",
             "工具：`scripts/camera_census.py`（换档复用 `scripts/camera_mode.py`）", "",
             "| 档位 | 分辨率 | 彩色 fps | 深度 fps | 有效点 | 中位深度 | 点距 | 飞点 | 空洞率 | 备注 |",
             "|---|---|---|---|---|---|---|---|---|---|"]
    for row in rows:
        if row.get("error"):
            lines.append(f"| {row['mode']} | — | — | — | — | — | — | — | — | ❌ {row['error']} |")
            continue
        lines.append("| {mode} | {width}×{height} | {colour_fps} | {depth_fps} | {valid} | {median} | "
                     "{spacing} | {flying} | {hole} | {note} |".format(
                         mode=row["mode"], width=row.get("width"), height=row.get("height"),
                         colour_fps=row.get("colour_fps"), depth_fps=row.get("depth_fps"),
                         valid=f"{row['valid_ratio'] * 100:.1f}%" if row.get("valid_ratio") else "—",
                         median=f"{row['median_m']:.3f} m" if row.get("median_m") else "—",
                         spacing=f"{row['spacing_mm']:.2f} mm" if row.get("spacing_mm") else "—",
                         flying=f"{row['flying'] * 100:.1f}%" if row.get("flying") else "—",
                         hole=f"{row['hole_ratio'] * 100:.1f}%" if row.get("hole_ratio") else "—",
                         note=row.get("note", "")))
    lines += ["", "## 换档日志", "```text"] + log + ["```", ""]
    (root / "summary.md").write_text("\n".join(lines))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--modes", nargs="+", default=list(MODES), choices=list(MODES))
    parser.add_argument("--frames", type=int, default=5)
    parser.add_argument("--switch-timeout", type=float, default=180.0)
    parser.add_argument("--capture-timeout", type=float, default=90.0)
    parser.add_argument("--out", default=None)
    args = parser.parse_args()

    root = pathlib.Path(args.out) if args.out else (
        DEFAULT_ROOT / f"camera_census_{time.strftime('%Y%m%d_%H%M%S')}")
    root.mkdir(parents=True, exist_ok=True)

    rows: list[dict] = []
    log: list[str] = []
    for mode in args.modes:
        print(f"\n== {mode} ==", flush=True)
        code, lines = switch_mode(mode, args.switch_timeout)
        log.append(f"$ camera_mode.py {mode}  (exit {code})")
        log.extend(lines[-8:])
        for line in lines[-6:]:
            print(f"  {line}")
        row: dict = {"mode": mode}
        if code != 0:
            row["error"] = f"换档失败，退出码 {code}"
            rows.append(row)
            continue

        directory = root / mode
        directory.mkdir(parents=True, exist_ok=True)
        state = capture(args.frames, args.capture_timeout)
        row.update(analyse(state))
        if not row.get("error") and (state["depth"] or state["colour"]):
            import cv2
            if state["colour"]:                     # 只开彩色的档位也要留证
                cv2.imwrite(str(directory / "color_000.png"), state["colour"][0])
            for index, depth_mm in enumerate(state["depth"][:2]):
                np.save(directory / f"depth_{index:03d}.npy", depth_mm)
            (directory / "meta.json").write_text(json.dumps({
                "mode": mode,
                "captured_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
                "encoding": state["encoding"],
                "camera_info": state["info"],
                "frames": row.get("frames"),
            }, ensure_ascii=False, indent=2))
        rows.append(row)
        print(f"  -> {row}", flush=True)

    write_summary(root, rows, log)
    print(f"\n产物：{root}")
    print((root / "summary.md").read_text())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
