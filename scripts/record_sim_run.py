#!/usr/bin/env python3
"""录制一段仿真任务视频，并且**只录仿真画面**。

为什么需要专门的录制器
----------------------
2026-10-25 的教训：一段"仿真演示视频"里混进了真机相机的画面，原因是 `run_task.py`
的相机模式钩子重启了真机驱动，而它继承了仿真的 `ROS_DOMAIN_ID`。肉眼看视频只能事后
发现，代价很高。所以这个录制器把三件事做成硬约束：

1. **来源自检**：先看 `/camera/color/image_raw` 的发布者节点名。出现 `camera`
   （Percipio 驱动）或 `percipio*` 就拒绝录制，并打印该怎么隔离（换 domain / 停驱动）。
2. **逐帧量化**：Gazebo 世界没有贴图，画面饱和度极低；真机照片饱和度很高。每帧都
   计算 HSV 饱和度，超过阈值（默认 0.10）的帧**直接丢弃**并在结束时报告丢弃数量。
3. **来源烧进画面**：每帧顶部印 `SIM`、阶段名/序号/状态、帧号与饱和度，视频本身就能
   自证来源，不需要再靠记忆或肉眼判断。

用法
----
    # 终端 A：起仿真（务必用独立域）
    ROS_DOMAIN_ID=71 GZ_PARTITION=cs625_rec \\
      ros2 launch cs625_bringup sim_cell.launch.py headless:=true launch_rviz:=false

    # 终端 B：录制（同一域），然后另开一个终端跑任务
    python3 scripts/record_sim_run.py --domain 71 --seconds 900 --out ~/cs625_evidence

    # 跑任务
    ROS_DOMAIN_ID=71 python3 scripts/run_task.py --scope load --mode efficient \\
      --world sim --confirm --allow-release --no-record

    # 录够时间后 Ctrl+C；脚本会自动合成 WebM（VP9，浏览器可直接播放）

输出
----
    <out>/sim_<scope>_<mode>_<ts>.webm     成品视频（带 SIM 标签与阶段字幕）
    <out>/sim_<...>_frames/frames.json     每帧的元数据（时间、饱和度、是否保留）
    <out>/sim_<...>_frames/events.json     阶段时间线（来自 /task_state）
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

SATURATION_LIMIT = 0.10          # > this is a real photo, not a Gazebo render
SIM_PUBLISHER_HINTS = ("alias", "bridge", "gazebo", "sim")
REAL_PUBLISHER_HINTS = ("percipio", "camera_container", "camera")  # node name 'camera'
FPS = 4.0


def classify_publishers(names: list[str]) -> tuple[bool, list[str], list[str]]:
    """(ok, simulated_names, suspicious_names) for the camera topic publishers."""
    simulated, suspicious = [], []
    for name in names:
        leaf = name.rsplit("/", 1)[-1]
        if any(hint in leaf.lower() for hint in SIM_PUBLISHER_HINTS):
            simulated.append(name)
        elif any(leaf.lower().startswith(hint) for hint in REAL_PUBLISHER_HINTS):
            suspicious.append(name)
    return (not suspicious), simulated, suspicious


class Recorder(Node):
    def __init__(self, out_dir: pathlib.Path, tag: str, allow_mixed: bool) -> None:
        super().__init__("cs625_sim_recorder")
        self.out_dir = out_dir
        self.allow_mixed = allow_mixed
        self.frames_dir = out_dir / f"{tag}_frames"
        self.frames_dir.mkdir(parents=True, exist_ok=True)
        for old in self.frames_dir.glob("*.jpg"):
            old.unlink()
        self.frames: list[dict] = []
        self.events: list[dict] = []
        self.bridge = None
        self.real_dropped = 0
        self.refused: str | None = None
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST,
                         durability=QoSDurabilityPolicy.VOLATILE)
        self.create_subscription(Image, "/camera/color/image_raw", self.on_image, qos)
        self.create_subscription(String, "/task_state", self.on_state, 10)
        self.create_timer(2.0, self.check_publishers)

    # ------------------------------------------------------------- provenance
    def check_publishers(self) -> None:
        names = []
        for topic, types in self.get_topic_names_and_types():
            if topic == "/camera/color/image_raw":
                for info in self.get_publishers_info_by_topic(topic):
                    names.append(f"{info.node_namespace.rstrip('/')}/{info.node_name}")
        if not names:
            return
        ok, simulated, suspicious = classify_publishers(names)
        if not ok and not self.allow_mixed:
            self.refused = (f"拒绝录制：/camera/color/image_raw 上有真机相机节点 "
                            f"{suspicious}；仿真与真机必须在不同 ROS_DOMAIN_ID 下。"
                            f" 建议：停掉相机驱动，或用另一个 domain 起仿真。")
            self.get_logger().error(self.refused)
            raise SystemExit(3)

    # ----------------------------------------------------------------- frames
    def on_state(self, msg: String) -> None:
        try:
            state = json.loads(msg.data)
        except ValueError:
            return
        self.events.append({"t": time.time(), **{k: state.get(k) for k in
                            ("flow", "mode", "phase", "phase_index", "phase_total", "status")}})

    def on_image(self, msg: Image) -> None:
        from cv_bridge import CvBridge

        if self.bridge is None:
            self.bridge = CvBridge()
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        saturation = float(cv2.cvtColor(image, cv2.COLOR_BGR2HSV)[:, :, 1].mean()) / 255.0
        index = len(self.frames) + 1
        keep = saturation < SATURATION_LIMIT
        record = {"index": index, "t": time.time(), "saturation": saturation, "keep": keep}
        if keep:
            name = f"{index:05d}.jpg"
            cv2.imwrite(str(self.frames_dir / name), image,
                        [cv2.IMWRITE_JPEG_QUALITY, 90])
            record["file"] = name
        else:
            self.real_dropped += 1
        self.frames.append(record)

    # ------------------------------------------------------------------ video
    def assemble(self, out_file: pathlib.Path) -> None:
        kept = [f for f in self.frames if f["keep"]]
        if not kept:
            print("没有可用的仿真帧，未生成视频")
            return
        sample = cv2.imread(str(self.frames_dir / kept[0]["file"]))
        height, width = sample.shape[:2]
        writer = cv2.VideoWriter(str(out_file), cv2.VideoWriter_fourcc(*"VP90"),
                                 FPS, (width, height))
        phase = ("", 0, 0, "")
        for frame in kept:
            image = cv2.imread(str(self.frames_dir / frame["file"]))
            if image is None:
                continue
            for event in self.events:
                if event["t"] <= frame["t"]:
                    phase = (event.get("phase") or "", event.get("phase_index"),
                             event.get("phase_total"), event.get("status"))
            cv2.rectangle(image, (0, 0), (width, 60), (18, 22, 28), -1)
            cv2.putText(image, f"SIM   {phase[0]}  {phase[1]}/{phase[2]}  {phase[3]}",
                        (14, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.78, (200, 235, 220), 2,
                        cv2.LINE_AA)
            cv2.putText(image, f"CS625 Gazebo   frame {frame['index']}/{len(self.frames)}"
                               f"   sat={frame['saturation']:.3f}",
                        (14, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (130, 150, 165), 1,
                        cv2.LINE_AA)
            writer.write(image)
        writer.release()
        print(f"视频: {out_file}  ({out_file.stat().st_size // 1024} KiB)")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", default="~/cs625_evidence")
    parser.add_argument("--tag", default="", help="文件名标签，默认 sim_<时间戳>")
    parser.add_argument("--seconds", type=float, default=0.0,
                        help="录多少秒后自动停止（0 = 一直录到 Ctrl+C）")
    parser.add_argument("--allow-mixed", action="store_true",
                        help="即使检测到真机相机也继续录（**不建议**，会污染视频）")
    args = parser.parse_args(argv)

    out_dir = pathlib.Path(args.out).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)
    tag = args.tag or time.strftime("sim_%Y%m%d_%H%M%S")
    rclpy.init()
    node = Recorder(out_dir, tag, args.allow_mixed)
    print(f"录制中：domain={__import__('os').environ.get('ROS_DOMAIN_ID', '0')}  "
          f"输出 {node.frames_dir}")
    print(f"规则：饱和度 >= {SATURATION_LIMIT} 的帧判定为真机画面，直接丢弃")
    started = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if args.seconds and time.time() - started >= args.seconds:
                break
    except KeyboardInterrupt:
        pass
    finally:
        video = out_dir / f"{tag}.webm"
        (node.frames_dir / "events.json").write_text(
            json.dumps(node.events, indent=1), encoding="utf-8")
        (node.frames_dir / "frames.json").write_text(
            json.dumps(node.frames, indent=1), encoding="utf-8")
        keept = sum(1 for f in node.frames if f["keep"])
        print(f"\n共收到 {len(node.frames)} 帧：仿真 {keept} 帧，丢弃(真机) {node.real_dropped} 帧")
        node.assemble(video)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
