#!/usr/bin/env python3
"""把示教器速度滑块设到"正好等于我们的关节速度上限"的比例（2026-11-20 所有者要求）。

为什么
------
现场有两个"开车的人"：我们下发的轨迹（治理到 MAX_JOINT_SPEED_RAD_S = 0.05 rad/s）
和机器人自己的伺服（速度 = 滑块比例 × 额定速度）。一旦"下发目标"与"实际位置"出现
跳变，控制权交给伺服，滑块就成了唯一还能限速的东西。2026-11-20 因为滑块被设成
100%，机械臂全速冲出安全区间。

做法
----
    fraction = 我们的上限 / 机器人额定关节速度
例如额定 2.094 rad/s（120°/s，来自 URDF 的 velocity 限位），
     fraction = 0.05 / 2.094 ≈ 0.024  → 2.4%

这样即使发生失控跳变，机器人最多也只能按我们的速度上限跑，而不会全速冲。
注意：滑块过小会让手动 jog 也变慢；需要临时快一点时用 --fraction 显式覆盖。

用法
----
    python3 scripts/set_speed_slider.py            # 按上限换算并设置
    python3 scripts/set_speed_slider.py --show     # 只显示会设置成多少
    python3 scripts/set_speed_slider.py --fraction 0.3   # 显式指定
"""

from __future__ import annotations

import argparse
import importlib.util
import pathlib
import subprocess
import sys

REPO = pathlib.Path(__file__).resolve().parents[1]
DEFAULT_RATED_RAD_S = 2.0943951023931953     # URDF 六轴 velocity 限位（120°/s）
SERVICE = "/io_and_status_controller/set_speed_slider"
MIN_FRACTION = 0.01
# 滑块必须比轨迹速度高几倍才有跟踪余量：等于轨迹速度时伺服会一直贴着极限跑，
# 表现为滞后、抖动、PATH_TOLERANCE_VIOLATED。3 倍余量足够跟踪，又能把失控跳变
# 限制在 8.6°/s（出事实测是 120°/s）。
TRACKING_HEADROOM = 3.0


def joint_speed_cap() -> float:
    """复用执行器的上限常量，避免两处写不同数字。"""
    spec = importlib.util.spec_from_file_location(
        "executor", REPO / "scripts" / "execute_assembly_chunks.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return float(module.MAX_JOINT_SPEED_RAD_S)


def rated_speed() -> float:
    try:
        done = subprocess.run(["ros2", "param", "get", "/robot_state_publisher",
                               "robot_description"],
                              capture_output=True, text=True, timeout=25)
        for token in (done.stdout or "").replace("\\n", " ").split():
            if token.startswith('velocity="'):
                value = float(token.split('"')[1])
                if value > 0.1:
                    return value
    except (OSError, subprocess.TimeoutExpired, ValueError):
        pass
    return DEFAULT_RATED_RAD_S


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--fraction", type=float, help="直接指定滑块比例（0–1）")
    parser.add_argument("--show", action="store_true", help="只显示，不调用服务")
    args = parser.parse_args()

    cap = joint_speed_cap()
    rated = rated_speed()
    fraction = args.fraction if args.fraction else cap * TRACKING_HEADROOM / rated
    fraction = max(MIN_FRACTION, min(1.0, fraction))
    max_speed = fraction * rated

    print(f"[slider] 我们的关节速度上限 : {cap:.4f} rad/s = {cap*57.2958:.2f}°/s")
    print(f"[slider] 机器人额定关节速度 : {rated:.4f} rad/s = {rated*57.2958:.1f}°/s")
    print(f"[slider] 应设滑块比例       : {fraction:.4f}（{fraction*100:.1f}%）"
          f" → 最大 {max_speed*57.2958:.2f}°/s"
          f"（轨迹上限 {cap*57.2958:.2f}°/s 的 {max_speed/cap:.1f} 倍余量）")
    if args.show:
        return 0

    command = ["ros2", "service", "call", SERVICE,
               "eli_common_interface/srv/SetSpeedSliderFraction",
               f"{{speed_slider_fraction: {fraction:.4f}}}"]
    try:
        done = subprocess.run(command, capture_output=True, text=True, timeout=25)
    except (OSError, subprocess.TimeoutExpired):
        print("[slider] 调用服务失败")
        return 2
    text = (done.stdout or "") + (done.stderr or "")
    if "success=True" not in text:
        print(f"[slider] 设置失败：{text.strip()[:120]}")
        return 3
    print("[slider] 已设置（success=True）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
