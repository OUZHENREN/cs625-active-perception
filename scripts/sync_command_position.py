#!/usr/bin/env python3
"""把控制器的"记忆位置"校正到机械臂的实际位置（自动，2026-11-20）。

为什么需要它
------------
arm_controller 是一个"位置记忆器"：它记着最后一次被要求去的目标。人手拖动机械臂
只改变机器人的**实际位置**，不会改变控制器的记忆。此时重连控制器/重发外部控制脚本，
控制器会把旧记忆重新下发，机械臂就会全速冲向旧位置——今天发生了三次：

```text
① --force 重挂 → 抽搐着跳回上一次轨迹终点（y 方向 240 mm）
② 执行器 NO_MOTION 时自动重挂 → 突然加速、大角度偏转，J6 越过 ±150°，所有者按下急停
③ 示教器 4 万多条 velocity violation（跳变被机器人拒收）
```

正确做法
--------
下发新轨迹**之前**，让控制器把"记忆"改成"当前实际位置"。改法只有一种可靠手段：
**失活再激活 arm_controller** —— 激活瞬间控制器会以当前 joint_states 作为命令基准。
代价：失活/激活会让伺服瞬间释放（手册记录过 16 mm 下坠），所以：

  · 只在**停车位姿**（J4 转轴与重力平行）附近执行，或由所有者显式 `--force`；
  · 执行前后各采样一次关节角，确认没有位移；
  · 之后才重发外部控制脚本恢复反控端口。

自动化的位置
------------
`execute_assembly_chunks.py` 会在**每条轨迹下发前**调用本脚本的纯检查部分；
本脚本的 `--apply` 才会真的动硬件，且必须在机械臂静止时使用。

用法
----
    python3 scripts/sync_command_position.py            # 只检查，报告偏差
    python3 scripts/sync_command_position.py --apply    # 校正（失活→激活→重发脚本）
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import subprocess
import sys
import time

import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import JointState

REPO = pathlib.Path(__file__).resolve().parents[1]
LAST_COMMANDED = REPO / "runtime" / "last_commanded_pose.json"
COMMON = REPO / "config" / "tasks" / "common.yaml"

ORDER = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
         "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
DRIFT_LIMIT_DEG = 2.0          # 记忆与实际超过这个偏差就必须校正
PARK_TOLERANCE_DEG = 6.0       # 距离停车位姿多近才允许自动失活/激活


def read_measured(timeout: float = 10.0) -> dict[str, float]:
    """读当前六轴（弧度）。"""
    rclpy.init()
    node = Node("sync_command_reader")
    box: dict = {}
    node.create_subscription(JointState, "/joint_states",
                             lambda m: box.setdefault("m", m), 10)
    deadline = time.time() + timeout
    while "m" not in box and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    message = box.get("m")
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    if message is None:
        return {}
    return dict(zip(message.name, message.position))


def park_distance_deg(measured: dict[str, float]) -> float | None:
    try:
        config = yaml.safe_load(COMMON.read_text(encoding="utf-8"))
        park = config["j4_park_pose"]["joints_deg"]
    except (OSError, KeyError, yaml.YAMLError):
        return None
    worst = 0.0
    for name, target in zip(ORDER, park):
        if name not in measured:
            return None
        worst = max(worst, abs(math.degrees(measured[name]) - float(target)))
    return worst


def command_drift_deg(measured: dict[str, float]) -> tuple[float, str] | None:
    """实测 vs 控制器记忆（runtime/last_commanded_pose.json）的最大偏差。"""
    try:
        payload = json.loads(LAST_COMMANDED.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    names = payload.get("joint_names") or []
    positions = payload.get("positions") or []
    if not names or len(names) != len(positions):
        return None
    worst, worst_name = 0.0, ""
    for name, target in zip(names, positions):
        if name not in measured:
            return None
        delta = abs(math.degrees(measured[name] - float(target)))
        if delta > worst:
            worst, worst_name = delta, name
    return worst, worst_name


def ports_up() -> bool:
    try:
        done = subprocess.run(["ss", "-tn"], capture_output=True, text=True, timeout=10)
    except (OSError, subprocess.TimeoutExpired):
        return False
    return any(f":5000{port} " in line for line in done.stdout.splitlines()
               for port in ("1", "3", "4"))


def switch(activate: bool) -> bool:
    action = "--activate" if activate else "--deactivate"
    command = ["ros2", "control", "switch_controllers",
               "--controller-manager", "/eli_ros2_control_node",
               action, "arm_controller", "--strict"]
    try:
        done = subprocess.run(command, capture_output=True, text=True, timeout=30)
    except (OSError, subprocess.TimeoutExpired):
        return False
    text = (done.stdout or "") + (done.stderr or "")
    return "Successfully" in text or "successful" in text.lower()


def resend_script() -> bool:
    command = ["ros2", "service", "call",
               "/io_and_status_controller/resend_external_script",
               "std_srvs/srv/Trigger", "{}"]
    try:
        done = subprocess.run(command, capture_output=True, text=True, timeout=30)
    except (OSError, subprocess.TimeoutExpired):
        return False
    return "success=True" in ((done.stdout or "") + (done.stderr or ""))


def write_baseline(measured: dict[str, float]) -> None:
    LAST_COMMANDED.parent.mkdir(parents=True, exist_ok=True)
    LAST_COMMANDED.write_text(json.dumps({
        "joint_names": list(measured.keys()),
        "positions": [float(v) for v in measured.values()],
        "stamp": time.time(),
        "source": "sync_command_position",
    }, ensure_ascii=False, indent=2), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--apply", action="store_true", help="真的执行校正（失活→激活→重发）")
    parser.add_argument("--force", action="store_true",
                        help="不在停车位附近也允许校正（风险自负）")
    args = parser.parse_args()

    measured = read_measured()
    if not measured:
        print("读不到 /joint_states，无法判断")
        return 2

    drift = command_drift_deg(measured)
    park = park_distance_deg(measured)
    print(f"[sync] 反控端口：{'已连通' if ports_up() else '断开'}")
    if drift is None:
        print("[sync] 没有控制器记忆基准（runtime/last_commanded_pose.json 缺失），"
              "无法比较；本次只报告")
    else:
        print(f"[sync] 记忆偏差：{drift[0]:.1f}°（{drift[1]}），限 {DRIFT_LIMIT_DEG:.1f}°")
    if park is not None:
        print(f"[sync] 距离停车位姿：{park:.1f}°（自动校正限 {PARK_TOLERANCE_DEG:.1f}°）")

    drift_ok = drift is not None and drift[0] <= DRIFT_LIMIT_DEG
    if not args.apply:
        if drift_ok and ports_up():
            print("[sync] 状态一致、端口正常，无需校正")
            return 0
        print("[sync] 需要校正（加 --apply 执行）")
        return 1

    # --- 真的校正 ---
    if park is not None and park > PARK_TOLERANCE_DEG and not args.force:
        print(f"[sync] 拒绝：当前离停车位姿 {park:.1f}°，校正会瞬间释放伺服，"
              "可能让机械臂下坠/摆动。")
        print("      请先把机械臂摆到停车位姿附近（J4 转轴与重力平行），"
              "或确认无风险后加 --force。")
        return 3

    before = measured
    if not switch(False):
        print("[sync] 失活 arm_controller 失败；未做任何改动")
        return 4
    if not switch(True):
        print("[sync] 重新激活失败；请运行 scripts/recover_driver_stack.sh")
        return 5
    print("[sync] 控制器已重新激活（记忆 = 当前实际位置）")

    if not ports_up():
        if resend_script():
            print("[sync] 已重发外部控制脚本，反控端口恢复")
        else:
            print("[sync] 重发外部控制脚本失败；请检查示教器与网络")
            return 6
    time.sleep(1.5)

    after = read_measured()
    if not after:
        print("[sync] 校正后读不到位姿，无法确认")
        return 7
    shift = 0.0
    for name in ORDER:
        if name in before and name in after:
            shift = max(shift, abs(math.degrees(after[name] - before[name])))
    print(f"[sync] 校正前后最大关节位移：{shift:.2f}°")
    if shift > 1.0:
        print("[sync] **警告：校正过程中机械臂发生了移动**，请立即检查现场")
        return 8

    write_baseline(after)
    print("[sync] 完成：记忆已对齐实际位置，可以安全下发轨迹")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
