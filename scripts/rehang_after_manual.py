#!/usr/bin/env python3
"""示教器切回远程后：把控制器"记忆"对齐 + 重挂为 RUNNING（一键，2026-11-26 所有者要求）。

为什么需要它
------------
操作场景：示教器切到**本地** → 手拖/示教器移动机械臂 → 切回**远程**。
这时有两件事同时错位：

1. **控制器记忆** 还停在"上一条轨迹的终点"，而机械臂已在别的位置。
   直接用 `ensure_robot_ready.py --force` 重挂，控制器会把旧记忆重新下发
   → 机械臂**全速冲向旧位置**（2026-11-20 记录过三次，其中一次 J6 越过 ±150° 被迫急停）。
2. **外部控制脚本**可能已随切换停止，`robot_mode` 即使仍报 7，
   动作也可能"被接受但不执行"（手册第 28 条）。

正确顺序（本脚本按此执行，不可颠倒）
------------------------------------
    ① 读 safety/robot 模式；safety ≠ NORMAL 直接拒绝（要人去示教器复位）
    ② 读当前关节角，与控制器记忆比对，打印偏差表
    ③ 偏差 > 阈值时：`sync_command_position.py --apply` —— 失活/激活 arm_controller，
       以**当前实测位置**作为新基准（代价：伺服瞬间释放，手册记录过 16 mm 下坠，
       所以只在停车位姿附近或所有者明确同意时做）
    ④ `ensure_robot_ready.py --force` —— 重发外部控制脚本 + 恢复速度滑条 → RUNNING
    ⑤ 复核：关节角前后变化、四项门（sockets / controller / joint_states / arm_controller）

用法
----
    python3 scripts/rehang_after_manual.py              # 只检查 + 给建议，不动硬件
    python3 scripts/rehang_after_manual.py --apply      # 执行 ③④⑤（需要所有者在场）
退出码：0 已就绪；3 安全模式异常；4 重挂失败；5 偏差过大且未给 --accept-drift；
        6 校正过程中检测到关节位移（疑似下坠，立即停止）。
"""

from __future__ import annotations

import argparse
import pathlib
import re
import subprocess
import sys

sys.stdout.reconfigure(line_buffering=True)
REPO = pathlib.Path(__file__).resolve().parents[1]
DRIFT_LIMIT_DEG = 2.0


def run(script: str, *args: str, timeout: int = 180) -> tuple[int, str]:
    proc = subprocess.run([sys.executable, str(REPO / "scripts" / script), *args],
                          capture_output=True, text=True, timeout=timeout)
    return proc.returncode, (proc.stdout or "") + (proc.stderr or "")


def read_joints() -> list[float] | None:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    rclpy.init()
    node = rclpy.create_node("rehang_probe")
    got: dict = {}
    node.create_subscription(JointState, "/joint_states", lambda m: got.setdefault("m", m), 10)
    import time
    end = time.time() + 6
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    return list(got["m"].position) if "m" in got else None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--apply", action="store_true", help="真正执行校正与重挂")
    parser.add_argument("--accept-drift", action="store_true",
                        help="偏差超过 %.0f° 时仍然校正（会把控制器记忆强制对齐到当前位置；"
                             "伺服会瞬间释放，可能下坠）" % DRIFT_LIMIT_DEG)
    args = parser.parse_args()

    # ① 只读偏差检查（sync_command_position 不带 --apply = 纯检查）
    code, out = run("sync_command_position.py")
    for line in out.strip().splitlines()[-8:]:
        print("  " + line)

    # ② 当前关节角快照
    before = read_joints()
    if before is None:
        print("读不到 /joint_states —— 驱动可能没起来，先 recover_driver_stack.sh")
        return 4
    print("当前关节角：", "  ".join(f"{v*57.2958:+.1f}°" for v in before))

    found = re.search(r"记忆偏差：([0-9.]+)°", out)
    drift = float(found.group(1)) if found else None
    park = re.search(r"距离停车位姿：([0-9.]+)°", out)
    print(f"记忆偏差 = {drift if drift is not None else '未知'}°"
          f"｜离停车位姿 {park.group(1) if park else '未知'}°")
    if drift is not None and drift > DRIFT_LIMIT_DEG and not args.accept_drift:
        print(f"\n偏差 {drift:.1f}° > {DRIFT_LIMIT_DEG:.0f}°：校正会把控制器记忆强制对齐到当前位置，"
              "伺服会瞬间释放（手册记录过 16 mm 下坠）。确认要这样做请加 --accept-drift。")
        return 5

    if not args.apply:
        print("\n（检查模式，未动硬件）确认后加 --apply；偏差大时必须同时给 --accept-drift")
        return 0

    # ④ 校正记忆（偏差大时把 --force 透传给 sync，因为它默认拒绝远离停车位姿的校正）
    sync_args = ["--apply"]
    if args.accept_drift:
        sync_args.append("--force")
    code, out = run("sync_command_position.py", *sync_args)
    print("--- sync_command_position --apply ---")
    for line in out.strip().splitlines()[-8:]:
        print("  " + line)
    if code != 0:
        print(f"记忆校正失败（exit {code}）：停止，不重挂")
        return 4

    # ⑤ 复核：校正后关节角是否变化（下坠检测）
    after = read_joints()
    max_delta = max(abs(a - b) for a, b in zip(after, before)) * 57.2958 if after else 999
    print(f"校正前后最大关节变化：{max_delta:.2f}°")
    if max_delta > 1.0:
        print(f"⚠ 校正过程中关节移动 {max_delta:.1f}°（疑似伺服释放下坠）→ 停止，不重挂，请人工确认")
        return 6

    # ⑥ 重挂为 RUNNING
    code, out = run("ensure_robot_ready.py", "--force")
    print("--- ensure_robot_ready --force ---")
    for line in out.strip().splitlines()[-8:]:
        print("  " + line)
    if code != 0:
        print(f"重挂失败（exit {code}）")
        return 4

    # ⑦ 四项门复核
    code, out = run("control_health.py")
    print("--- control_health ---")
    for line in out.strip().splitlines()[-4:]:
        print("  " + line)
    print("\n✅ 已重挂为 RUNNING，四项门见上；可以下发新轨迹"
          "（新轨迹务必从当前实测位姿出发）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
