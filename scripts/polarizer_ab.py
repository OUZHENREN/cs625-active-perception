#!/usr/bin/env python3
"""Polarizer A/B: capture static RGB-D frames, then compare the two states.

Why a dedicated tool
--------------------
The comparison must vary **exactly one thing** (the film), so the arm may not
move between the two captures.  ``capture_dataset.py`` plans and moves the arm,
so it cannot be used here.  This tool assumes the arm is already parked at the
observation pose and only listens to the camera.

    # 0) 5-minute diagnostic BEFORE gluing anything: one film only, in front
    #    of the IR receiver, rotated in ~15 deg steps.  If the flying points on
    #    the coated face change with the angle, the projector is already
    #    polarised and the projector film can be skipped; if nothing changes,
    #    the projector is unpolarised and the crossed pair is mandatory.
    # 1) film NOT installed yet
    python3 scripts/polarizer_ab.py capture --state before
    # 2) install the film(s), rotate the analyser to the darkest, then
    python3 scripts/polarizer_ab.py capture --state after
    # 3) compare (pure numpy, no ROS needed)
    python3 scripts/polarizer_ab.py compare \
        --before runtime/polarizer_ab/<stamp>_before \
        --after  runtime/polarizer_ab/<stamp>_after \
        --roi 400 200 900 700          # optional: the workpiece window

What the numbers mean (this is the part that is easy to get wrong)
-----------------------------------------------------------------
Crossed polarizers remove the *specular* return.  On a mirror-like coated face
the failure mode is not "too few points" but "points on a phantom surface": the
projected pattern bounces off the coating and the receiver triangulates it to
the wrong depth.  After the film that face may legitimately become an *empty
hole* -- fewer points, better data.  So the comparison looks at:

  valid_ratio      share of pixels with a depth return (must not collapse)
  flying           share of points whose local neighbourhood is not planar,
                   3 mm off the local plane -- the phantom/noise points
  planarity        local planarity, smaller is flatter
  spacing_mm       true nearest-neighbour distance (stride-1 cloud)
  hole_ratio       empty cells (about 3 mm at the measured depth) inside the ROI
  saturation       colour pixels clipped to white (the specular blob in RGB)

Decision rule (printed by ``compare``):
  PASS  flying at most half, spacing not worse than +25 %, valid_ratio >= 80 %
  PART  flying drops but the light loss is too large -> raise laser_power/exposure
  FAIL  no change (film does not extinguish at 830 nm) or everything gets sparser

The arm is **never commanded** here.  Park it first, then run this.
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import sys
import time

import numpy as np

sys.stdout.reconfigure(line_buffering=True)

REPO = pathlib.Path(__file__).resolve().parents[1]
DEFAULT_ROOT = REPO / "runtime" / "polarizer_ab"
RELIABLE_QOS_TOPIC_HINT = (
    "hint: the publisher is probably BEST_EFFORT; re-run with --qos best_effort, "
    "or launch through scripts/camera_mode.py (which uses reliable QoS)."
)


# ---------------------------------------------------------------------------
# Pure metric core (importable without ROS; unit-testable)
# ---------------------------------------------------------------------------
def depth_to_metres(depth: np.ndarray) -> np.ndarray:
    """Normalise the driver's depth encoding to float32 metres.

    The real camera publishes 16UC1 millimetres, the simulation 32FC1 metres.
    Getting this wrong is a silent 1000x scale error, so the conversion is
    decided from the data, not from a flag.
    """
    array = np.asarray(depth)
    if array.dtype == np.uint16:
        return array.astype(np.float32) / 1000.0
    out = array.astype(np.float32)
    finite = out[np.isfinite(out) & (out > 0)]
    if finite.size and float(np.median(finite)) > 100.0:      # millimetres
        out = out / 1000.0
    return out


def roi_slice(shape, roi):
    """Clamp a (x0, y0, x1, y1) ROI to the image; ``None`` means the whole image."""
    if roi is None:
        return None
    height, width = shape
    x0, y0, x1, y1 = (int(v) for v in roi)
    x0, x1 = max(0, min(width - 1, x0)), max(0, min(width, x1))
    y0, y1 = max(0, min(height - 1, y0)), max(0, min(height, y1))
    if x1 <= x0 or y1 <= y0:
        return None
    return slice(y0, y1), slice(x0, x1)


def intrinsics(k: list[float]) -> tuple[float, float, float, float]:
    """(fx, fy, cx, cy) out of the row-major 3x3 K, which is [fx,0,cx, 0,fy,cy, 0,0,1]."""
    return float(k[0]), float(k[4]), float(k[2]), float(k[5])


def back_project(depth_m: np.ndarray, k: list[float]) -> np.ndarray:
    """Valid pixels -> (N,3) points in the camera optical frame."""
    valid = np.isfinite(depth_m) & (depth_m > 0.0)
    vs, us = np.nonzero(valid)
    z = depth_m[vs, us]
    fx, fy, cx, cy = intrinsics(k)
    u = us.astype(np.float32)
    v = vs.astype(np.float32)
    return np.column_stack([(u - cx) * z / fx, (v - cy) * z / fy, z])


def flying_stats(points: np.ndarray, k: int = 10, max_points: int = 120_000) -> tuple[float, float]:
    """Fraction of non-planar points and the median planarity.

    A point is "flying" when its k-neighbourhood does not lie on a plane: the
    smallest covariance eigenvalue is large compared to the others.  Phantom
    returns off the mirror show up here as a fat tail.
    """
    if len(points) < k + 1:
        return float("nan"), float("nan")
    from scipy.spatial import cKDTree

    if len(points) > max_points:
        keep = np.random.default_rng(0).choice(len(points), max_points, replace=False)
        points = points[keep]
    tree = cKDTree(points)
    _, idx = tree.query(points, k=k + 1)           # includes the point itself
    neighbours = points[idx[:, 1:]]                # (N, k, 3)
    centred = neighbours - neighbours.mean(axis=1, keepdims=True)
    cov = np.einsum("nki,nkj->nij", centred, centred) / float(k)
    eig = np.linalg.eigvalsh(cov)                  # ascending, vectorised
    total = eig.sum(axis=1)
    planarity = np.divide(eig[:, 0], total, out=np.zeros(len(eig)), where=total > 1e-18)
    residual = np.sqrt(np.maximum(eig[:, 0], 0.0))
    flying = float((residual > 0.003).mean())      # 3 mm off the local plane
    return flying, float(np.median(planarity))


def spacing_mm(points: np.ndarray, queries: int = 20_000) -> float:
    """True median nearest-neighbour distance in millimetres (stride-1 cloud)."""
    if len(points) < 10:
        return float("nan")
    from scipy.spatial import cKDTree

    tree = cKDTree(points)
    if len(points) > queries:
        pick = np.random.default_rng(0).choice(len(points), queries, replace=False)
        probe = points[pick]
    else:
        probe = points
    dist, _ = tree.query(probe, k=2)
    return float(np.median(dist[:, 1]) * 1000.0)


def hole_ratio(depth_m: np.ndarray, k: list[float], roi, cell_m: float = 0.003) -> float:
    """Share of ~3 mm cells (at the measured depth) without a single valid pixel."""
    valid = np.isfinite(depth_m) & (depth_m > 0.0)
    roi_sl = roi_slice(valid.shape, roi)
    crop = valid if roi_sl is None else valid[roi_sl]
    depth_crop = depth_m if roi_sl is None else depth_m[roi_sl]
    if crop.size == 0 or not crop.any():
        return 1.0
    median_z = float(np.median(depth_crop[np.isfinite(depth_crop) & (depth_crop > 0)]))
    cell_px = max(1, int(round(cell_m * intrinsics(k)[0] / max(median_z, 1e-6))))
    height, width = crop.shape
    hh, ww = (height // cell_px) * cell_px, (width // cell_px) * cell_px
    if hh == 0 or ww == 0:
        return 1.0 - float(crop.mean())
    cells = crop[:hh, :ww].reshape(hh // cell_px, cell_px, ww // cell_px, cell_px)
    return float(1.0 - cells.any(axis=(1, 3)).mean())


def cell_ratios(valid: np.ndarray, roi, cells: int = 3) -> np.ndarray:
    """把有效点掩膜切成 cells×cells 格，返回每格的有效点比例。

    偏振消光会让整幅图**均匀**变暗；遮挡/膜没盖严则会让某些格子整片变黑。
    这个网格就是用来区分这两者的。
    """
    roi_sl = roi_slice(valid.shape, roi)
    crop = valid if roi_sl is None else valid[roi_sl]
    height, width = crop.shape
    out = np.full((cells, cells), np.nan)
    for row in range(cells):
        for col in range(cells):
            block = crop[row * height // cells:(row + 1) * height // cells,
                         col * width // cells:(col + 1) * width // cells]
            if block.size:
                out[row, col] = float(block.mean())
    return out


def colour_saturation(colour: np.ndarray, roi) -> tuple[float, float]:
    """ROI colour stats: share of pixels clipped to white, and the mean brightness.

    The mean brightness is the cheap "did the film actually cut light" check: a
    working polariser in front of a bright scene drops roughly half the light; a
    film that barely changes it is either not over the aperture or transparent at
    the camera's wavelength.
    """
    crop = colour[roi_slice(colour.shape[:2], roi)] if roi else colour
    if crop.size == 0:
        return float("nan"), float("nan")
    clipped = float((crop[..., :3].min(axis=-1) >= 250).mean())
    brightness = float(crop[..., :3].mean())
    return clipped, brightness


def state_metrics(dataset: pathlib.Path, roi, stride: int) -> dict:
    """Aggregate metrics over every frame in a capture directory."""
    dataset = pathlib.Path(dataset)
    meta_path = dataset / "meta.json"
    if not meta_path.exists():
        raise SystemExit(f"missing meta.json in {dataset} -- run 'capture' first")
    meta = json.loads(meta_path.read_text())
    if not meta.get("camera_info"):
        raise SystemExit(f"no camera_info captured in {dataset}; re-run 'capture' with the camera up")
    k = meta["camera_info"]["k"]

    acc: dict[str, list[float]] = {key: [] for key in
                                   ("valid_ratio", "flying", "planarity",
                                    "hole", "spacing_mm", "saturation", "brightness")}
    grids: list[np.ndarray] = []
    for depth_path in sorted(dataset.glob("depth_*.npy")):
        depth_m = depth_to_metres(np.load(depth_path))
        valid = np.isfinite(depth_m) & (depth_m > 0.0)
        roi_sl = roi_slice(depth_m.shape, roi)
        acc["valid_ratio"].append(float(valid[roi_sl].mean()) if roi_sl else float(valid.mean()))
        grids.append(cell_ratios(valid, roi))
        # flying/planarity come from a strided cloud (speed); spacing from the
        # stride-1 cloud so its millimetres stay honest.
        if stride > 1:
            small = depth_m[::stride, ::stride]
            fx, fy, cx, cy = intrinsics(k)
            k_scaled = [fx / stride, 0.0, cx / stride, 0.0, fy / stride, cy / stride,
                        0.0, 0.0, 1.0]
            flying, planarity = flying_stats(back_project(small, k_scaled))
        else:
            flying, planarity = flying_stats(back_project(depth_m, k))
        acc["flying"].append(flying)
        acc["planarity"].append(planarity)
        acc["hole"].append(hole_ratio(depth_m, k, roi))
        acc["spacing_mm"].append(spacing_mm(back_project(depth_m, k)))
        colour_path = dataset / depth_path.name.replace("depth_", "color_").replace(".npy", ".png")
        if colour_path.exists():
            import cv2
            clipped, brightness = colour_saturation(cv2.imread(str(colour_path)), roi)
            acc["saturation"].append(clipped)
            acc["brightness"].append(brightness)
    result = {key: float(np.nanmedian(values)) for key, values in acc.items()
              if values and not all(math.isnan(v) for v in values)}
    result["valid_std"] = round(float(np.nanstd(acc["valid_ratio"])), 4)
    if grids:
        result["grid"] = np.nanmedian(np.stack(grids), axis=0)
    return result


# ---------------------------------------------------------------------------
# Capture (ROS; rclpy is imported lazily so ``compare`` works without it)
# ---------------------------------------------------------------------------
def _image_qos(qos, reliable: bool):
    profile = qos.QoSProfile(depth=10, history=qos.HistoryPolicy.KEEP_LAST)
    profile.reliability = (qos.ReliabilityPolicy.RELIABLE if reliable
                           else qos.ReliabilityPolicy.BEST_EFFORT)
    return profile


def cmd_capture(args) -> int:
    import cv2
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CameraInfo, Image

    out = pathlib.Path(args.out) if args.out else (
        DEFAULT_ROOT / f"{time.strftime('%Y%m%d_%H%M%S')}_{args.state}")
    out.mkdir(parents=True, exist_ok=True)
    qos = _image_qos(rclpy.qos, args.qos == "reliable")

    class Capture(Node):
        def __init__(self):
            super().__init__("polarizer_ab_capture")
            self.done = False
            self.saved = 0
            self.info = None
            self.latest_colour = None
            self.seen: set[int] = set()
            self.create_subscription(Image, "/camera/color/image_raw", self.on_colour, qos)
            self.create_subscription(Image, "/camera/depth/image_raw", self.on_depth, qos)
            self.create_subscription(CameraInfo, "/camera/color/camera_info", self.on_info, qos)

        def on_info(self, msg):
            if self.info is None:
                self.info = {"k": list(msg.k), "width": msg.width, "height": msg.height}

        def on_colour(self, msg):
            self.latest_colour = msg

        def on_depth(self, msg):
            key = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
            if key in self.seen:                  # same frame delivered twice
                return
            self.seen.add(key)
            depth_m = self.decode_depth(msg)
            np.save(out / f"depth_{self.saved:03d}.npy", depth_m)
            if self.latest_colour is not None:
                cv2.imwrite(str(out / f"color_{self.saved:03d}.png"),
                            self.decode_colour(self.latest_colour))
                self.latest_colour = None
            self.saved += 1
            if self.saved % 5 == 0 or self.saved >= args.frames:
                print(f"  captured {self.saved}/{args.frames}", flush=True)
            if self.saved >= args.frames:
                self.done = True

        @staticmethod
        def decode_depth(msg):
            if msg.encoding == "16UC1":
                raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
                return raw.astype(np.float32) / 1000.0
            if msg.encoding == "32FC1":
                raw = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
                return raw.copy()
            raise SystemExit(f"unsupported depth encoding: {msg.encoding}")

        @staticmethod
        def decode_colour(msg):
            if msg.encoding in ("bgr8", "8UC3"):
                return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3).copy()
            if msg.encoding == "rgb8":
                raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                return raw[..., ::-1].copy()
            raise SystemExit(f"unsupported colour encoding: {msg.encoding}")

    # sweep-capture 会在同一进程里连续采多组：那时上下文由外层管理，
    # 这里不能再 init/shutdown（同一进程里重复 init 会抛 RuntimeError）。
    manage_context = getattr(args, "manage_context", True)
    if manage_context and not rclpy.ok():
        rclpy.init()
    node = Capture()
    try:
        deadline = time.time() + args.timeout
        while time.time() < deadline and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        print("\n采集被中断：已保存的帧仍然有效")
    finally:
        node.destroy_node()
        if manage_context and rclpy.ok():
            rclpy.shutdown()

    if node.saved == 0:
        print("captured 0 frames.", file=sys.stderr)
        if args.qos == "reliable":
            print(RELIABLE_QOS_TOPIC_HINT, file=sys.stderr)
        return 1
    meta = {
        "state": args.state,
        "captured_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "frames": node.saved,
        "camera_info": node.info,
        "qos": args.qos,
        "note": args.note,
    }
    (out / "meta.json").write_text(json.dumps(meta, ensure_ascii=False, indent=2))
    print(f"saved {node.saved} frames -> {out}")
    return 0


# ---------------------------------------------------------------------------
# Compare
# ---------------------------------------------------------------------------
ROWS = [
    ("valid_ratio", "有效点比例", "{:.3f}", "不应明显下降"),
    ("flying", "飞点比例（局部非平面）", "{:.3f}", "应显著下降"),
    ("planarity", "局部平面性（越小越平）", "{:.4f}", "应变好"),
    ("spacing_mm", "平均点距 (mm)", "{:.2f}", "不应变稀"),
    ("hole", "空洞率（ROI 内 3 mm 格）", "{:.3f}", "可能上升（镜面变空）"),
    ("saturation", "彩色过曝比例", "{:.3f}", "参考"),
    ("brightness", "彩色平均亮度", "{:.1f}", "装膜后应明显变暗"),
    ("valid_std", "帧间有效点波动", "{:.4f}", "越小越稳；>0.05 说明采集时在动"),
]


def cmd_compare(args) -> int:
    before = state_metrics(args.before, args.roi, args.stride)
    after = state_metrics(args.after, args.roi, args.stride)
    print(f"\n{'指标':<28}{'before':>10}{'after':>10}{'变化':>10}   期望")
    print("-" * 80)
    for key, label, fmt, expect in ROWS:
        b, a = before.get(key), after.get(key)
        if b is None or a is None:
            print(f"{label:<28}{'-':>10}{'-':>10}{'-':>10}   {expect}")
            continue
        if abs(b) > 1e-12:
            delta_text = f"{(a - b) / b * 100.0:>9.1f}%"
        else:
            delta_text = f"{'n/a':>10}"
        print(f"{label:<28}{fmt.format(b):>10}{fmt.format(a):>10}{delta_text}   {expect}")
    print("-" * 80)

    grid_b, grid_a = before.get("grid"), after.get("grid")
    if grid_b is not None and grid_a is not None:
        with np.errstate(divide="ignore", invalid="ignore"):
            ratio = np.where(grid_b > 0.02, grid_a / np.maximum(grid_b, 1e-9), np.nan)
        print("\n空间分布（after / before 每格有效点比例，1.00 = 没变化）：")
        for row in ratio:
            print("   [" + "  ".join(f"{value:5.2f}" if not math.isnan(value) else "   --"
                                     for value in row) + "]")
        finite = ratio[~np.isnan(ratio)]
        if finite.size:
            worst, best = float(finite.min()), float(finite.max())
            mean = float(finite.mean())
            print(f"   最小 {worst:.2f} · 最大 {best:.2f} · 平均 {mean:.2f}", end="  →  ")
            if mean < 0.30:
                print("整体大幅衰减：图案被压没了（消光过强，或投射窗被膜/手挡住）")
            elif worst < 0.25 and best > 0.7:
                print("局部缺失：像遮挡/膜没盖严，不是均匀消光")
            elif best - worst < 0.35:
                print("整体均匀变化：符合偏振消光的特征")
            else:
                print("介于两者之间，需要结合图像判断")

    flying_b, flying_a = before.get("flying"), after.get("flying")
    if flying_b is None or flying_a is None or math.isnan(flying_b) or math.isnan(flying_a):
        print("判定：数据不足，无法判定")
        return 2
    spacing_b, spacing_a = before.get("spacing_mm"), after.get("spacing_mm")
    valid_b, valid_a = before.get("valid_ratio"), after.get("valid_ratio")
    density_ok = (spacing_a is None or spacing_b is None or spacing_a <= 1.25 * spacing_b)
    light_ok = (valid_a is None or valid_b is None or valid_a >= 0.8 * valid_b)
    if flying_a <= 0.5 * flying_b and density_ok:
        verdict = "PASS" if light_ok else "PART"
    elif flying_a < 0.85 * flying_b:
        verdict = "PART"
    else:
        verdict = "FAIL"
    advice = {
        "PASS": "偏振对有效：飞点显著下降，点密度与有效点没有明显损失。",
        "PART": "有收益但代价偏大：先提高 laser_power / 曝光，或只保留投射端的起偏片。",
        "FAIL": "没效果：先做镜面测试确认膜在 830 nm 能消光，再检查两片是否真的正交。",
    }[verdict]
    print(f"判定：{verdict} —— {advice}")
    return 0 if verdict != "FAIL" else 1


def cmd_frames(args) -> int:
    """逐帧打印一组采集的有效点/中位深度，用来看"前几帧是不是还没贴好"。"""
    directory = pathlib.Path(args.directory)
    files = sorted(directory.glob("depth_*.npy"))
    if not files:
        print(f"{directory} 里没有 depth_*.npy")
        return 1
    print(f"\n{directory.name}  共 {len(files)} 帧")
    print(f"{'帧':>4}{'有效点':>9}{'中位深度':>11}{'变化':>10}")
    print("-" * 38)
    previous = None
    for index, path in enumerate(files):
        depth = depth_to_metres(np.load(path))
        valid = depth > 0.0
        ratio = float(valid.mean())
        median = float(np.median(depth[valid])) if valid.any() else float("nan")
        delta = "" if previous is None else f"{100 * (ratio - previous):+7.1f}pp"
        print(f"{index:>4}{ratio * 100:8.1f}%{median:10.3f} m{delta:>10}")
        previous = ratio
    return 0


def cmd_sweep(args) -> int:
    """把多个 capture 目录与一个基线比较，按飞点比例从低到高排序。

    用于找投射端偏振片的最佳角度：每转 15–20° 采一组，`sweep` 直接告诉你哪一组飞点最少。
    """
    baseline = state_metrics(args.baseline, args.roi, args.stride)
    rows = []
    for directory in args.candidates:
        try:
            metrics = state_metrics(directory, args.roi, args.stride)
        except SystemExit as exc:
            print(f"  跳过 {directory}: {exc}")
            continue
        rows.append((metrics.get("flying", float("nan")), directory, metrics))
    if not rows:
        print("没有可比较的数据")
        return 2
    rows.sort(key=lambda item: (math.isnan(item[0]), item[0]))
    print(f"\n基线 {args.baseline}: 飞点 {baseline.get('flying', float('nan')):.4f} · "
          f"点距 {baseline.get('spacing_mm', float('nan')):.2f} mm · "
          f"有效点 {baseline.get('valid_ratio', float('nan')) * 100:.1f}%")
    print(f"\n{'候选':<40}{'飞点':>8}{'相对基线':>10}{'点距':>9}{'有效点':>9}")
    print("-" * 78)
    for flying, directory, metrics in rows:
        base_flying = baseline.get("flying")
        if base_flying and not math.isnan(flying) and base_flying > 1e-9:
            delta = f"{(flying - base_flying) / base_flying * 100:>8.1f}%"
        else:
            delta = f"{'n/a':>9}"
        print(f"{pathlib.Path(directory).name:<40}{flying:>8.4f}{delta:>10}"
              f"{metrics.get('spacing_mm', float('nan')):>9.2f}"
              f"{metrics.get('valid_ratio', float('nan')) * 100:>8.1f}%")
    best = rows[0]
    print("-" * 78)
    print(f"飞点最少：{pathlib.Path(best[1]).name}（{best[0]:.4f}）")
    return 0


def cmd_sweep_capture(args) -> int:
    """交互式角度扫描：转一下 → 按回车（或等倒计时）→ 自动采一组 → 下一步。

    为什么单独做这个：一只手扶着偏振片旋转时，没法同时敲完整条 capture 命令。
    这里把"节拍"固定在脚本里：每次只让操作者做一个动作，采完自动存盘，
    全部结束后自动按飞点比例排序。
    """
    import rclpy
    if not rclpy.ok():
        rclpy.init()

    root = pathlib.Path(args.out) if args.out else DEFAULT_ROOT
    stamp = time.strftime("%Y%m%d_%H%M%S")
    angles = [args.start + index * args.step for index in range(args.steps)]
    saved: list[str] = []

    print(f"\n=== 角度扫描：共 {len(angles)} 步，每步 {args.frames} 帧 "
          f"({'自动 ' + str(args.auto_seconds) + ' s 一拍' if args.auto_seconds else '按回车开始'}) ===")
    print("提示：膜先贴上去但不要粘死；每步只转投射端那一片，接收端的膜不要动。\n")

    for index, angle in enumerate(angles):
        have = f"已存 {len(saved)} 组" if saved else ""
        if args.auto_seconds:
            print(f"第 {index + 1}/{len(angles)} 步 ——→ 转到 {angle:3d}°（{args.auto_seconds:.0f} s 后自动采集）{have}")
            try:
                for remaining in range(int(args.auto_seconds), 0, -1):
                    print(f"   {remaining:2d} s ... 转到 {angle:3d}°", flush=True)
                    time.sleep(1.0)
            except KeyboardInterrupt:
                print("\n结束扫描（Ctrl+C）——已采的组会照常参与排序")
                break
        else:
            try:
                answer = input(f"第 {index + 1}/{len(angles)} 步 ——→ 转到 {angle:3d}°，"
                               f"转好后按回车采集（s=跳过，q=结束）{have}: ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                print("\n结束扫描")
                break
            if answer == "q":
                break
            if answer == "s":
                print("  跳过")
                continue

        directory = root / f"{stamp}_sweep_{angle:03d}deg"
        namespace = argparse.Namespace(state=f"sweep_{angle:03d}deg", frames=args.frames,
                                       out=str(directory), qos=args.qos,
                                       timeout=args.timeout, note=f"projector film at {angle} deg",
                                       manage_context=False)
        print(f"  采集中（{args.frames} 帧，约 {args.frames * 1.3:.0f} s）...", flush=True)
        try:
            ok = cmd_capture(namespace) == 0
        except KeyboardInterrupt:
            print("\n结束扫描（Ctrl+C）")
            break
        if ok:
            saved.append(str(directory))

    if not saved:
        print("一组都没采到。")
        if rclpy.ok():
            rclpy.shutdown()
        return 1
    print(f"\n共保存 {len(saved)} 组：")
    for directory in saved:
        print(f"  {directory}")
    if args.baseline:
        code = cmd_sweep(argparse.Namespace(baseline=args.baseline, candidates=saved,
                                            roi=args.roi, stride=args.stride))
    else:
        print("\n加上 --baseline <recv_only 目录> 可以在最后自动排序。")
        code = 0
    if rclpy.ok():
        rclpy.shutdown()
    return code


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)

    capture = sub.add_parser("capture", help="capture N settled frames at the current arm pose")
    capture.add_argument("--state", required=True,
                         help="free tag for this capture: before / recv_only / crossed / after ...")
    capture.add_argument("--frames", type=int, default=10)
    capture.add_argument("--out", default=None)
    capture.add_argument("--qos", choices=["reliable", "best_effort"], default="reliable")
    capture.add_argument("--timeout", type=float, default=180.0)
    capture.add_argument("--note", default="")
    capture.set_defaults(func=cmd_capture)

    compare = sub.add_parser("compare", help="compare two capture directories")
    compare.add_argument("--before", required=True)
    compare.add_argument("--after", required=True)
    compare.add_argument("--roi", type=int, nargs=4, default=None,
                         metavar=("X0", "Y0", "X1", "Y1"),
                         help="region of interest in colour-image pixels")
    compare.add_argument("--stride", type=int, default=2,
                         help="pixel stride for the flying-point cloud (default 2; spacing is always stride-1)")
    compare.set_defaults(func=cmd_compare)

    sweep = sub.add_parser("sweep", help="compare many captures against one baseline")
    sweep.add_argument("--baseline", required=True)
    sweep.add_argument("--candidates", nargs="+", required=True)
    sweep.add_argument("--roi", type=int, nargs=4, default=None,
                       metavar=("X0", "Y0", "X1", "Y1"))
    sweep.add_argument("--stride", type=int, default=2)
    sweep.set_defaults(func=cmd_sweep)

    frames = sub.add_parser("frames", help="print per-frame valid ratio for one capture")
    frames.add_argument("--dir", dest="directory", required=True)
    frames.set_defaults(func=cmd_frames)

    interactive = sub.add_parser("sweep-capture", help="interactive angle sweep: rotate, press Enter, repeat")
    interactive.add_argument("--baseline", default=None,
                             help="recv_only 基线目录；给了就在结束后自动排序")
    interactive.add_argument("--steps", type=int, default=8)
    interactive.add_argument("--step", type=int, default=20, help="每步转多少度")
    interactive.add_argument("--start", type=int, default=0)
    interactive.add_argument("--frames", type=int, default=4)
    interactive.add_argument("--auto-seconds", type=float, default=None,
                             help="不给就等回车；给了就每 N 秒自动采一拍（双手都占用时用）")
    interactive.add_argument("--qos", choices=["reliable", "best_effort"], default="reliable")
    interactive.add_argument("--timeout", type=float, default=60.0)
    interactive.add_argument("--out", default=None)
    interactive.add_argument("--roi", type=int, nargs=4, default=None)
    interactive.add_argument("--stride", type=int, default=2)
    interactive.set_defaults(func=cmd_sweep_capture)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
