#!/usr/bin/env python3
"""把一次精拍测量登记为站位表条目（2026-11-19 所有者定）。

背景
----
所有者要按 +30°…−45° 每 5° 精拍一遍示教位姿，并严格绕"工件角点"旋转。
每拍完一个角度，需要把这次测量的结果写进 `config/tasks/common.yaml` 的
`workpiece_views`：相机位姿（tcp/quat）、实测覆盖率、参考测量文件、优先级。

这个工具就是那一步：读测量 JSON → 校验质量 → 生成条目（默认只打印，--write 才改文件）。

用法
----
    # 只看会写入什么（默认，不改文件）
    python3 scripts/add_station_from_measurement.py \
        --measurement runtime/measurements/workpiece_at_0.json \
        --key '0@20261120' --yaw 0.0

    # 真正写入
    python3 scripts/add_station_from_measurement.py \
        --measurement runtime/measurements/workpiece_at_0.json \
        --key '0@20261120' --yaw 0.0 --write

相机位姿默认从当前机器人读取（/tcp_pose_broadcaster/pose）；也可以显式给：
    --tcp 0.63546,0.32805,0.46654 --quat 0.470235,-0.731014,-0.275024,-0.410926

新增字段含义见 config/tasks/common.yaml 里 workpiece_views 顶部的注释。
"""

from __future__ import annotations

import argparse
import datetime
import json
import pathlib
import sys

import numpy as np
import yaml

REPO = pathlib.Path(__file__).resolve().parents[1]
COMMON = REPO / "config" / "tasks" / "common.yaml"


def read_measurement(path: pathlib.Path) -> dict:
    payload = json.loads(path.read_text(encoding="utf-8"))
    groups = payload.get("per_group") or []
    if not groups:
        raise SystemExit(f"{path} 里没有 per_group，无法登记")
    return payload


def current_tcp(timeout: float = 15.0):
    """读当前 TCP 位姿；读不到就返回 None（离线登记时可以显式给 --tcp/--quat）。"""
    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.node import Node
    except Exception:  # noqa: BLE001 - 没有 ROS 时走显式参数
        return None
    import time as _time
    rclpy.init()
    node = Node("add_station_tcp")
    box: dict = {}
    node.create_subscription(PoseStamped, "/tcp_pose_broadcaster/pose",
                             lambda m: box.setdefault("m", m), 10)
    end = _time.time() + timeout
    while "m" not in box and _time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    message = box.get("m")
    if message is None:
        return None
    p = message.pose.position
    q = message.pose.orientation
    return [p.x, p.y, p.z], [q.x, q.y, q.z, q.w]


def round_list(values, digits: int) -> list[float]:
    return [round(float(v), digits) for v in values]


def build_entry(args, payload: dict, tcp, quat, tcp_source: str) -> dict:
    verdict = payload.get("verdict")
    coverage = payload.get("coverage_median")
    credible = payload.get("credible_groups")
    if verdict != "OK":
        raise SystemExit(f"本次测量 verdict={verdict}，拒绝登记（先修好再拍）")
    if coverage is None:
        raise SystemExit("测量结果里没有 coverage_median，无法登记")
    if coverage < args.min_coverage:
        raise SystemExit(f"覆盖率 {coverage:.4f} < {args.min_coverage:.2f}，拒绝登记")

    group = (payload.get("per_group") or [])[0]
    pose = group.get("pose_base") or {}
    translation = pose.get("translation")
    quaternion = pose.get("quaternion")
    yaw = payload.get("workpiece_yaw_deg")
    if yaw is None:
        raise SystemExit("测量结果里没有 workpiece_yaw_deg，无法登记")
    if abs(float(yaw) - args.yaw) > args.yaw_tol:
        raise SystemExit(
            f"测量出的工件偏角 {float(yaw):.2f}° 与 --yaw {args.yaw:.2f}° 相差超过 "
            f"{args.yaw_tol:.1f}°：工件可能没摆正，拒绝登记")
    if tcp is None or quat is None:
        raise SystemExit("读不到当前 TCP；请用 --tcp/--quat 显式给出相机位姿")

    try:
        relative = pathlib.Path(args.measurement).resolve().relative_to(REPO)
    except ValueError:
        relative = pathlib.Path(args.measurement)
    return {
        "yaw_deg": round(float(yaw), 2),
        "tier": int(args.tier),
        "tcp": round_list(tcp, 5),
        "quat": round_list(quat, 6),
        "empirical_coverage": round(float(coverage), 3),
        "reference_measurement": str(relative),
        "note": (f"{datetime.date.today().isoformat()} 精拍登记：覆盖 {float(coverage):.3f}，"
                 f"可信组 {credible}/{payload.get('groups', '?')}，"
                 f"重复性 {repeat_text(payload.get('repeatability'))}，"
                 f"相机位姿来自 {tcp_source}"),
        "validated": True,
        "workpiece_pose": {
            "translation": round_list(translation, 5),
            "quaternion": round_list(quaternion, 6),
        },
    }


def repeat_text(value) -> str:
    """把 repeatability 字典变成纯文本，避免字典里带冒号破坏 YAML。"""
    if not isinstance(value, dict):
        return str(value)
    translation = value.get("translation_mm")
    rotation = value.get("rotation_deg")
    if translation is None and rotation is None:
        return "未采集"
    return f"{translation} mm / {rotation}°"


def render_block(key: str, entry: dict) -> str:
    lines = [f"  {json.dumps(key, ensure_ascii=False)}:"]
    lines.append(f"    yaw_deg: {entry['yaw_deg']}")
    lines.append(f"    tier: {entry['tier']}")
    lines.append("    tcp:")
    lines.extend(f"    - {v}" for v in entry["tcp"])
    lines.append("    quat:")
    lines.extend(f"    - {v}" for v in entry["quat"])
    lines.append(f"    empirical_coverage: {entry['empirical_coverage']}")
    lines.append(f"    reference_measurement: {entry['reference_measurement']}")
    lines.append(f"    note: {json.dumps(entry['note'], ensure_ascii=False)}")
    lines.append("    validated: true")
    lines.append("    # 该站位精拍时的工件位姿（摆放检查/复核用）")
    lines.append("    workpiece_pose:")
    lines.append("      translation:")
    lines.extend(f"      - {v}" for v in entry["workpiece_pose"]["translation"])
    lines.append("      quaternion:")
    lines.extend(f"      - {v}" for v in entry["workpiece_pose"]["quaternion"])
    return "\n".join(lines) + "\n"


def insert_entry(text: str, block: str) -> str:
    """把条目插到 workpiece_views 映射的开头（保留原文件所有注释）。"""
    marker = "workpiece_views:\n"
    index = text.index(marker)
    head = text[: index + len(marker)]
    tail = text[index + len(marker):]
    # 若第一条就是注释/空行，跳过它们让条目紧随其后仍合法
    return head + block + tail


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--measurement", required=True, help="精拍得到的测量 JSON")
    parser.add_argument("--key", required=True,
                        help="站位键；与已有角度撞名时建议带日期，如 '0@20261120'。"
                             "注意键以负号开头时必须用等号形式：--key=-20@20261120")
    parser.add_argument("--yaw", type=float, required=True,
                        help="本次精拍的工件偏角（度），用于核对测量结果")
    parser.add_argument("--yaw-tol", type=float, default=1.5,
                        help="测得偏角与 --yaw 允许的偏差（度）")
    parser.add_argument("--tier", type=int, default=1,
                        help="1 = 新拍首选（默认）；2 = 既有兜底")
    parser.add_argument("--min-coverage", type=float, default=0.95,
                        help="低于这个覆盖率拒绝登记。示教位要长期复用，标准比测量硬底线"
                             "（0.90）更高：0.95 以上才登记（所有者 2026-11-20 定）")
    parser.add_argument("--tcp", default="", help="相机位姿 x,y,z（默认读当前机器人）")
    parser.add_argument("--quat", default="", help="相机四元数 x,y,z,w")
    parser.add_argument("--write", action="store_true", help="真正写入 common.yaml")
    args = parser.parse_args()

    payload = read_measurement(pathlib.Path(args.measurement))
    if args.tcp and args.quat:
        tcp = [float(v) for v in args.tcp.split(",")]
        quat = [float(v) for v in args.quat.split(",")]
        tcp_source = "命令行参数"
    else:
        read = current_tcp()
        if read is None:
            raise SystemExit("读不到当前 TCP：请用 --tcp/--quat 显式给出")
        tcp, quat = read
        tcp_source = "/tcp_pose_broadcaster/pose"

    entry = build_entry(args, payload, tcp, quat, tcp_source)
    block = render_block(args.key, entry)
    print("将要登记的站位条目：")
    print(block)

    config = yaml.safe_load(COMMON.read_text(encoding="utf-8"))
    existing = (config.get("workpiece_views") or {}).get(args.key)
    if existing is not None:
        print(f"注意：key {args.key} 已存在，写入会与它重复（YAML 后者覆盖前者）")

    if not args.write:
        print("（未写入；确认无误后加 --write）")
        return 0

    text = COMMON.read_text(encoding="utf-8")
    COMMON.write_text(insert_entry(text, block), encoding="utf-8")
    # 写完立即回读校验：YAML 仍可解析，且新条目能被读到
    reloaded = yaml.safe_load(COMMON.read_text(encoding="utf-8"))
    check = (reloaded.get("workpiece_views") or {}).get(args.key)
    if not check or check.get("tcp") != entry["tcp"]:
        raise SystemExit("写入后回读校验失败：条目没有正确落地")
    print(f"已写入 {COMMON}，键 {args.key} 回读校验通过")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
