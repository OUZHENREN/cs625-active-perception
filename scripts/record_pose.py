#!/usr/bin/env python3
"""记录当前机械臂六轴位姿 + TCP 到 reference 文档（只记录，不改任何模式配置）。

为什么单独一个脚本
------------------
`set_observation_pose.py` 会**改写** `config/tasks/common.yaml` 里的
`teaching_observation_poses`，那是给示教/效率模式用的。所有者 2026-11-10 的要求是
"只做一个有效记录，先不替代效率模式里的那些" —— 所以这里只往
`docs/reference/observation_poses_recorded.yaml` 追加一条，不动任何模式配置。

用法（需要机械臂 ROS 驱动栈在跑，它提供 /joint_states 与 /tcp_pose_broadcaster/pose）：

    source scripts/source_dev_env.sh --full
    python3 scripts/record_pose.py --name workpiece_parked_20261110 \
        --note "粗观测位有效记录，不替代效率模式" \
        --dataset runtime/vision_dataset/20261110_1115_orient5

退出码：0 已写入；2 读不到位姿（驱动栈没起）；3 文件写入失败。
"""

from __future__ import annotations

import argparse
import datetime
import pathlib
import sys
import time

sys.stdout.reconfigure(line_buffering=True)

REPO = pathlib.Path(__file__).resolve().parents[1]
RECORD = REPO / "docs" / "reference" / "observation_poses_recorded.yaml"


def read_pose(timeout: float) -> dict | None:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    rclpy.init(args=None)
    node = rclpy.create_node("record_pose")
    state: dict = {}

    def on_joints(message):
        state["joints"] = {"names": list(message.name),
                           "positions": [float(v) for v in message.position]}

    def on_tcp(message):
        p = message.pose
        state["tcp"] = {"xyz": [round(p.position.x, 6), round(p.position.y, 6),
                                round(p.position.z, 6)],
                        "quat_xyzw": [round(p.orientation.x, 6), round(p.orientation.y, 6),
                                      round(p.orientation.z, 6), round(p.orientation.w, 6)],
                        "frame": message.header.frame_id}

    node.create_subscription(JointState, "/joint_states", on_joints, 10)
    node.create_subscription(PoseStamped, "/tcp_pose_broadcaster/pose", on_tcp, 10)
    deadline = time.time() + timeout
    while time.time() < deadline and ("joints" not in state or "tcp" not in state):
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()
    return state if "joints" in state and "tcp" in state else None


def append_record(entry: dict) -> None:
    RECORD.parent.mkdir(parents=True, exist_ok=True)
    if not RECORD.exists():
        RECORD.write_text(
            "# 已记录的机械臂位姿（只做记录，不改任何模式配置）\n"
            "# 由 scripts/record_pose.py 追加；字段：recorded_at/name/source/note/\n"
            "# joints(names,rad)/tcp(xyz,quat_xyzw)/camera/dataset\n"
            "poses: []\n")
    text = RECORD.read_text()
    block = ["", f"  - recorded_at: {entry['recorded_at']}",
             f"    name: {entry['name']}",
             f"    source: {entry['source']}",
             f"    note: {entry['note']!r}",
             "    joints:"]
    for name, value in zip(entry["joints"]["names"], entry["joints"]["positions"]):
        block.append(f"      {name}: {value:.6f}")
    block.append("    tcp:")
    block.append("      xyz: " + "[" + ", ".join(f"{v:.6f}" for v in entry["tcp"]["xyz"]) + "]")
    block.append("      quat_xyzw: " + "[" + ", ".join(f"{v:.6f}" for v in entry["tcp"]["quat_xyzw"]) + "]")
    if entry.get("dataset"):
        block.append(f"    dataset: {entry['dataset']}")
    if entry.get("camera"):
        block.append("    camera: " + "{"
                     + ", ".join(f"{k}: {v}" for k, v in entry["camera"].items()) + "}")
    if "poses: []" in text:
        text = text.replace("poses: []", "poses:" + block[0].lstrip("\n") if False else "poses:")
        text = text.rstrip("\n") + "\n" + "\n".join(block[1:]) + "\n"
    else:
        text = text.rstrip("\n") + "\n" + "\n".join(block[1:]) + "\n"
    RECORD.write_text(text)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--name", required=True, help="记录名，例如 workpiece_parked_20261110")
    parser.add_argument("--note", default="", help="备注（写清楚用途）")
    parser.add_argument("--dataset", default=None, help="该位姿下对应的数据集目录（可选）")
    parser.add_argument("--timeout", type=float, default=15.0)
    args = parser.parse_args()

    state = read_pose(args.timeout)
    if state is None:
        print("读不到位姿：/joint_states 或 /tcp_pose_broadcaster/pose 没有数据。\n"
              "机械臂 ROS 驱动栈没启动——先跑 VS Code 任务「① 开工 · 真机」，"
              "或 ros2 launch cs625_bringup real_base.launch.py ...", file=sys.stderr)
        return 2

    entry = {
        "recorded_at": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
        "name": args.name,
        "source": "ROS: /joint_states + /tcp_pose_broadcaster/pose",
        "note": args.note,
        "joints": state["joints"],
        "tcp": state["tcp"],
        "dataset": args.dataset,
    }
    print("六轴（rad）：")
    for name, value in zip(state["joints"]["names"], state["joints"]["positions"]):
        print(f"  {name:>10} {value:+.6f}")
    print("TCP：", entry["tcp"])
    try:
        append_record(entry)
    except OSError as exc:
        print(f"写入失败：{exc}", file=sys.stderr)
        return 3
    print(f"已追加到 {RECORD.relative_to(REPO)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
