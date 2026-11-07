#!/usr/bin/env python3
"""效率模式：示教链路的调试版（十一月视觉开发 / 十二月力控实验的入口）。

与「安全模式」的区别只在于**可调、可分阶段、可归档**，而不是另一条链路：

    安全模式   scripts/resume_teaching.sh
               → 门检全开、静置全开、失败自动重启并续跑，面向演示与取数据
    效率模式   本脚本
               → 可只跑某一步、可跳静置/门检、可改速度、可重复 N 次，
                 每次运行自动归档，供算法对比与论文出图

它不会（也不允许）跳过：保护停止规则、工件附近先抬升守卫、释放许可。

产物：
    runtime/debug_runs/<时间戳>/
        params.json    本次实际参数（命令行 + 关键常量快照）
        log.txt        完整输出
        captures/      本次用到的 T1/T2/T3 采集（运行后从 rvs_teaching 复制）
        effort.csv     力矩曲线（若该阶段产生）
        result.json    结果与指标（无传感器的项写 "未采集"）

用法：
    teaching_debug.py --stage 5                     # 只跑预抓取
    teaching_debug.py --from 12 --to 13 --no-settle # 只跑旋转+下降
    teaching_debug.py --stage 14 --repeat 3         # 12 月做方案 A/B/C 对比就靠这个
    teaching_debug.py --interactive                 # 逐项询问（VS Code 任务 ③ 用它）
    teaching_debug.py --stage 5 --dry-run           # 只打印将执行的命令，不动臂

退出码：0 全部成功；非 0 表示链路自身失败（与 teaching_sequence 一致）。
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import pathlib
import shutil
import subprocess
import sys
import time

sys.stdout.reconfigure(line_buffering=True)

REPO = pathlib.Path(__file__).resolve().parents[2]
CHAIN = REPO / "scripts" / "teaching" / "teaching_sequence.py"
RUNS = REPO / "runtime" / "debug_runs"

# 阶段编号 → 名称（与 teaching_sequence 的打印一致）
STAGES = {
    0: "环境模型", 1: "插槽精观测", 3: "工件粗观测", 4: "工件精观测",
    5: "预抓取", 6: "进入抓取", 7: "张爪夹持", 8: "抬升", 9: "抬到安全高度",
    10: "transit", 11: "staging", 12: "旋转", 13: "下降到预插入",
    14: "插入", 15: "转平", 16: "释放",
}
# 链路支持的 --start-from 取值（其余阶段只能用 --from 的最近入口 + --stop-after-stage）
START_FROM_VALUES = (1, 3, 5, 10, 12, 13, 14)


def nearest_start(stage: int) -> int:
    candidates = [v for v in START_FROM_VALUES if v <= stage]
    return max(candidates) if candidates else 1


def build_chain_args(args) -> list[str]:
    chain: list[str] = [sys.executable, "-u", str(CHAIN), "--confirm"]
    if args.from_stage is not None or args.to_stage is not None:
        first = args.from_stage if args.from_stage is not None else 1
        last = args.to_stage if args.to_stage is not None else first
        chain += ["--start-from", str(nearest_start(first)),
                  "--stop-after-stage", str(last)]
    elif args.stage is not None:
        chain += ["--start-from", str(nearest_start(args.stage)),
                  "--stop-after-stage", str(args.stage)]
    else:
        chain += ["--start-from", "1"]
    if args.no_gates:
        chain.append("--no-gates")
    if args.no_settle:
        chain.append("--no-settle")
    for override in args.speed_override:
        chain += ["--speed-override", override]
    if args.skip_env:
        chain.append("--skip-env")
    if args.allow_release:
        chain.append("--allow-release")
    return chain


def snapshot_params(args) -> dict:
    """Record the parameters AND the constants they interact with."""
    consts = {}
    chain_text = CHAIN.read_text()
    for name in ("GRASP_CLEARANCE_M", "GRASP_SNAP_VERTICAL", "CARRY_SPEED_CAP",
                 "APPROACH_HEIGHT_M", "SAFE_HEIGHT_Z", "SETTLE_BEFORE_CAPTURE",
                 "RVS_WAIT_BUDGET", "MAX_NO_MOTION"):
        for line in chain_text.splitlines():
            if line.strip().startswith(name + " ="):
                consts[name] = line.split("=", 1)[1].strip()
                break
    return {
        "stamp": dt.datetime.now().strftime("%Y-%m-%dT%H:%M:%S"),
        "requested": {"stage": args.stage, "from": args.from_stage, "to": args.to_stage,
                      "no_gates": args.no_gates, "no_settle": args.no_settle,
                      "speed_override": args.speed_override, "repeat": args.repeat},
        "chain_constants": consts,
    }


def collect_artifacts(run_dir: pathlib.Path, since: float) -> dict:
    """Copy the captures this run produced and summarise them."""
    summary = {"captures": [], "effort_csv": None}
    src = REPO / "runtime" / "rvs_teaching"
    dest = run_dir / "captures"
    dest.mkdir(exist_ok=True)
    if src.is_dir():
        for path in sorted(src.glob("*.json")):
            try:
                if path.stat().st_mtime < since:
                    continue
                record = json.loads(path.read_text())
            except (OSError, json.JSONDecodeError):
                continue
            shutil.copy2(path, dest / path.name)
            summary["captures"].append({
                "file": path.name,
                "command": record.get("command"),
                "fitness": record.get("fitness"),
                "position": [round(float(v), 4) for v in record.get("position", [])],
            })
    effort = pathlib.Path("/tmp/compliant_insert_effort.csv")
    if effort.exists() and effort.stat().st_mtime >= since:
        shutil.copy2(effort, run_dir / "effort.csv")
        summary["effort_csv"] = "effort.csv"
    return summary


def build_result(run_dir: pathlib.Path, code: int, started: float) -> dict:
    """Fill the metric skeleton from 实验指标体系 v3; unknown channels stay '未采集'."""
    captures = []
    cap_dir = run_dir / "captures"
    if cap_dir.is_dir():
        for path in sorted(cap_dir.glob("*.json")):
            try:
                captures.append(json.loads(path.read_text()))
            except (OSError, json.JSONDecodeError):
                pass
    fitness = [c["fitness"] for c in captures if c.get("fitness") is not None]
    return {
        "success": code == 0,
        "exit_code": code,
        "duration_s": round(time.time() - started, 1),
        "captures": len(captures),
        "fitness_mean": round(sum(fitness) / len(fitness), 4) if fitness else None,
        "fitness_min": round(min(fitness), 4) if fitness else None,
        # A 装配结果
        "phase_times": "未采集",
        "jam_count": "未采集",
        "pose_error_after_mm": "未采集",
        # B 接触力学（十二月六维力到位后填）
        "force_peak_n": "未采集", "force_rms_n": "未采集", "force_work_j": "未采集",
        "force_lateral_imbalance_n": "未采集",
        "moment_peak_nm": "未采集",
        # C 结构与执行器
        "joint_torque_peak": "未采集", "joint_torque_saturation_count": "未采集",
        "tcp_repeatability_mm": "未采集", "gripper_drift_mm": "未采集",
        "effort_jerk_peak": "未采集",
        # D 顺应与敏感性
        "admittance_scan": "未采集", "initial_offset_mm": "未采集", "clearance_mm": "未采集",
        # E 视触一致性
        "vision_residual_mm": "未采集", "force_correction_mm": "未采集",
        # F 仿真-实机
        "ansys_contact_prediction_error_pct": "未采集",
    }


def run_once(args, index: int, total: int) -> int:
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S") + (f"_{index}" if total > 1 else "")
    run_dir = RUNS / stamp
    run_dir.mkdir(parents=True, exist_ok=True)
    chain = build_chain_args(args)
    command = " ".join(chain)
    print(f"\n[debug] run {index}/{total} -> {run_dir.relative_to(REPO)}")
    print(f"[debug] {command}")
    (run_dir / "params.json").write_text(
        json.dumps(snapshot_params(args), ensure_ascii=False, indent=2))
    if args.dry_run:
        (run_dir / "result.json").write_text(json.dumps(
            {"success": None, "dry_run": True}, ensure_ascii=False, indent=2))
        return 0

    started = time.time()
    log_path = run_dir / "log.txt"
    with log_path.open("w", encoding="utf-8") as log_file:
        process = subprocess.Popen(chain, cwd=str(REPO), stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, text=True)
        try:
            assert process.stdout is not None
            for line in process.stdout:
                sys.stdout.write(line)
                log_file.write(line)
        except KeyboardInterrupt:
            print("\n[debug] Ctrl-C: stopping the chain")
            process.send_signal(2)
        code = process.wait()

    collect_artifacts(run_dir, started)
    result = build_result(run_dir, code, started)
    (run_dir / "result.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2))
    print(f"[debug] result: success={result['success']} "
          f"duration={result['duration_s']}s captures={result['captures']} "
          f"fitness_min={result['fitness_min']}")
    return code


def interactive() -> int:
    print("效率模式（调试版）——逐项配置，回车取默认值")
    stage = input("  阶段编号（0,1,3,4,5..15，回车=从 5 到 13）: ").strip()
    args = argparse.Namespace(stage=None, from_stage=None, to_stage=None,
                              no_gates=False, no_settle=False, speed_override=[],
                              repeat=1, skip_env=False, allow_release=False,
                              dry_run=False)
    if stage:
        args.stage = int(stage)
    else:
        args.from_stage, args.to_stage = 5, 13
    args.no_settle = input("  跳过 15 s 静置？[y/N]: ").strip().lower() == "y"
    args.no_gates = input("  跳过位姿门与位姿库？[y/N]: ").strip().lower() == "y"
    speed = input("  速度覆盖（如 pre_grasp=0.03，回车跳过）: ").strip()
    if speed:
        args.speed_override = [speed]
    repeat = input("  重复次数（回车=1）: ").strip()
    args.repeat = int(repeat) if repeat else 1
    args.dry_run = input("  只打印命令不执行？[y/N]: ").strip().lower() == "y"
    return dispatch(args)


def dispatch(args) -> int:
    code = 0
    for index in range(1, args.repeat + 1):
        code = run_once(args, index, args.repeat)
        if code != 0:
            print(f"[debug] stopping after failure (exit {code})")
            break
    return code


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--stage", type=int, choices=sorted(STAGES),
                        help="只执行这一阶段")
    parser.add_argument("--from", dest="from_stage", type=int, choices=sorted(STAGES),
                        help="起始阶段（与 --to 一起用）")
    parser.add_argument("--to", dest="to_stage", type=int, choices=sorted(STAGES),
                        help="结束阶段")
    parser.add_argument("--no-gates", action="store_true")
    parser.add_argument("--no-settle", action="store_true")
    parser.add_argument("--speed-override", action="append", default=[],
                        metavar="PHASE=VELOCITY")
    parser.add_argument("--repeat", type=int, default=1,
                        help="重复次数（方案 A/B/C 对比用）")
    parser.add_argument("--skip-env", action="store_true",
                        help="跳过环境模型重建（调试时可省时间）")
    parser.add_argument("--allow-release", action="store_true",
                        help="允许第 16 步释放（默认禁止）")
    parser.add_argument("--dry-run", action="store_true",
                        help="只打印命令与归档路径，不动机械臂")
    parser.add_argument("--interactive", action="store_true",
                        help="交互式配置（VS Code 任务 ③ 使用）")
    args = parser.parse_args()

    if args.interactive:
        return interactive()
    if args.stage is None and args.from_stage is None:
        parser.error("需要 --stage 或 --from/--to")
    if args.to_stage is not None and args.from_stage is None:
        parser.error("--to 需要同时给 --from")
    RUNS.mkdir(parents=True, exist_ok=True)
    return dispatch(args)


if __name__ == "__main__":
    raise SystemExit(main())
