#!/usr/bin/env python3
"""坤维 KWR75 六轴力/力矩传感器 → ROS 2（2026-11-25 接入）。

协议（KWR75用户手册 V21.6.75.3.3LB，非定制版长数据格式 28 字节 / RS485）
----------------------------------------------------------------------
  串口：460800 8N1（默认；可用 0x50 01/02/04 改，改后必须断电重启）
  命令帧（4 字节）：0x48 0xAA 0x0D 0x0A  → 1 kHz 连续发送，数据帧以 0x48 开头
                    0x49 0xAA 0x0D 0x0A  → 单次转换（第一次只启动刷新，第二次才回）
                    0x43 0xAA 0x0D 0x0A  → 停止发送
  数据帧（28 字节）：[0x48 或 0x49] 0xAA + 6 × float32(小端) + 0x0D 0x0A
     6 个分量顺序：Fx Fy Fz（**kg**，kgf）、Mx My Mz（**kg·m**）
     ROS 用 N / N·m，所以发布前乘 9.80665。
  手册给出的样例帧已在本仓库测试里逐位校验（Fx=-9.77873 vs 手册 -9.7787）。

WSL 侧接线（Windows 的 COM3 不能被 WSL 直接打开）
------------------------------------------------
  Windows（管理员 PowerShell）：
      usbipd list
      usbipd bind     --busid <BUSID>
      usbipd attach   --wsl --busid <BUSID>
  WSL：
      sudo usermod -aG dialout $USER           # 重新登录生效
      echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
      cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer   # 应为 1
  **互斥**：数采软件 SenserLinker 会独占 COM3；跑本节点前请先关闭它。
  （就像 RVS 与 ROS 相机互斥、HITBOT Studio 与夹爪桥互斥一样。）

用法
----
    source scripts/source_dev_env.sh --full

    # 先离线自检：用手册样例帧验证解析与单位换算（不需要硬件）
    python3 scripts/ft_sensor_node.py --self-test

    # 真机采集（默认流模式，1 kHz）
    python3 scripts/ft_sensor_node.py                      # 发布 /ft_sensor/wrench
    python3 scripts/ft_sensor_node.py --port /dev/ttyUSB0 --baud 460800
    python3 scripts/ft_sensor_node.py --poll --rate 100    # 轮询模式（40 Hz 级别）

发布：
    /ft_sensor/wrench        geometry_msgs/WrenchStamped  （N / N·m）
    /ft_sensor/diagnostics   std_msgs/String（JSON：帧数/坏帧/速率/温度无）
退出码：0 正常；2 串口打不开（被占用/未 attach）；3 自检失败；4 收不到数据。
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import struct
import sys
import time

sys.stdout.reconfigure(line_buffering=True)

REPO = pathlib.Path(__file__).resolve().parents[1]
CONFIG = REPO / "config" / "ft_sensor" / "kwr75.yaml"
G = 9.80665                      # kgf → N；kgf·m → N·m
FRAME_LEN = 28
HEADERS = (0x49, 0x48)
MARKER = 0xAA
TRAILER = b"\x0d\x0a"
COMPONENTS = ("fx", "fy", "fz", "mx", "my", "mz")


def parse_frame(frame: bytes, gravity: float = G) -> dict | None:
    """解析一帧 28 字节；不合法返回 None。

    单位换算在这里完成：手册给的是 kg / kg·m，ROS 需要 N / N·m。
    """
    if len(frame) != FRAME_LEN:
        return None
    if frame[0] not in HEADERS or frame[1] != MARKER or frame[-2:] != TRAILER:
        return None
    raw = struct.unpack("<6f", frame[2:26])
    if any(not math.isfinite(v) for v in raw):
        return None
    force = [raw[0] * gravity, raw[1] * gravity, raw[2] * gravity]        # N
    torque = [raw[3] * gravity, raw[4] * gravity, raw[5] * gravity]       # N·m
    return {"force": force, "torque": torque, "raw": list(raw),
            "frame_header": frame[0]}


class FrameParser:
    """从字节流里切出完整帧；容忍丢字节与噪声（靠 0xAA + 尾部 0D0A 双校验）。"""

    def __init__(self, gravity: float = G) -> None:
        self.buffer = bytearray()
        self.gravity = gravity
        self.bad_frames = 0
        self.resyncs = 0

    def feed(self, data: bytes) -> list[dict]:
        self.buffer.extend(data)
        out: list[dict] = []
        while True:
            if len(self.buffer) < FRAME_LEN:
                break
            if self.buffer[0] not in HEADERS or self.buffer[1] != MARKER:
                # 丢到下一个可能的帧头（下一个 0x49/0x48 后面紧跟 0xAA）
                index = self._next_header()
                if index <= 0:
                    self.buffer.clear()
                    self.resyncs += 1
                    break
                del self.buffer[:index]
                self.resyncs += 1
                continue
            frame = bytes(self.buffer[:FRAME_LEN])
            if frame[-2:] != TRAILER:
                del self.buffer[:1]        # 尾部不对：只丢一个字节，继续找
                self.bad_frames += 1
                continue
            parsed = parse_frame(frame, self.gravity)
            del self.buffer[:FRAME_LEN]
            if parsed is None:
                self.bad_frames += 1
                continue
            out.append(parsed)
        return out

    def _next_header(self) -> int:
        for i in range(1, len(self.buffer) - 1):
            if self.buffer[i] in HEADERS and self.buffer[i + 1] == MARKER:
                return i
        return -1


def self_test() -> int:
    """用手册样例帧做离线自检（不需要硬件）。"""
    sample = bytes.fromhex(
        "49AAB0751CC19B3FEDC0AE5CF040D4B52DBE5F55993E7C1AB0BB0D0A")
    parsed = parse_frame(sample)
    if parsed is None:
        print("自检失败：样例帧解析不出来")
        return 3
    expect_raw = [-9.77873, -7.41401, 7.51131, -0.16964, 0.29948, -0.00537]
    for name, got, want in zip(COMPONENTS, parsed["raw"], expect_raw):
        if abs(got - want) > 1e-4:
            print(f"自检失败：{name} 期望 {want} 实得 {got}")
            return 3
    print("自检通过：28 字节帧解析与手册一致")
    print(f"  原始（kg / kg·m）  {[round(v, 5) for v in parsed['raw']]}")
    print(f"  换算后（N / N·m）  force {[round(v, 2) for v in parsed['force']]} "
          f"torque {[round(v, 3) for v in parsed['torque']]}")
    # 容错：前面塞垃圾、中间丢字节，仍应切出正确帧
    parser = FrameParser()
    frames = parser.feed(b"\x00\x11garbage" + sample + b"\x99" + sample)
    print(f"  容错测试：切出 {len(frames)} 帧（应为 2），坏帧 {parser.bad_frames}，重同步 {parser.resyncs}")
    return 0 if len(frames) == 2 else 3


def load_config() -> dict:
    import yaml
    payload = yaml.safe_load(CONFIG.read_text(encoding="utf-8")) or {}
    return payload


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=None, help="串口设备（默认读 config）")
    parser.add_argument("--baud", type=int, default=None, help="波特率（默认 460800）")
    parser.add_argument("--poll", action="store_true",
                        help="轮询模式（发 0x49，适合低速率观察）；默认流模式 0x48")
    parser.add_argument("--rate", type=float, default=0.0,
                        help="轮询模式的目标频率 Hz（默认 20；流模式忽略）")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="采集时长秒（0 = 一直跑到 Ctrl+C）")
    parser.add_argument("--self-test", action="store_true", help="只跑离线自检")
    parser.add_argument("--topic", default=None)
    parser.add_argument("--frame-id", default=None)
    args = parser.parse_args()

    if args.self_test:
        return self_test()

    cfg = load_config()
    sensor = cfg.get("sensor") or {}
    ros = cfg.get("ros") or {}
    port = args.port or str(sensor.get("port", "/dev/ttyUSB0"))
    baud = args.baud or int(sensor.get("baudrate", 460800))
    topic = args.topic or str(ros.get("topic", "/ft_sensor/wrench"))
    frame_id = args.frame_id or str(ros.get("frame_id", "ft_sensor"))
    gravity = float(sensor.get("gravity_m_s2", G))
    commands = sensor.get("command") or {}
    stream_cmd = bytes(commands.get("stream_1khz", [0x48, 0xAA, 0x0D, 0x0A]))
    poll_cmd = bytes(commands.get("poll_once", [0x49, 0xAA, 0x0D, 0x0A]))
    stop_cmd = bytes(commands.get("stop", [0x43, 0xAA, 0x0D, 0x0A]))

    try:
        import serial
    except ImportError:
        print("缺少 pyserial：pip install pyserial")
        return 2
    try:
        link = serial.Serial(port=port, baudrate=baud, bytesize=8, parity="N",
                             stopbits=1, timeout=0.05)
    except Exception as error:      # noqa: BLE001
        print(f"打不开串口 {port} @ {baud}：{error}")
        print("检查：① usbipd attach 是否完成（WSL）；② SenserLinker 是否还占用 COM3；"
              "③ 当前用户是否在 dialout 组")
        return 2

    if args.poll:
        link.write(poll_cmd)        # 手册：第一次只启动内部刷新
        time.sleep(0.05)
        period = 1.0 / max(1.0, args.rate or 20.0)
    else:
        link.write(stream_cmd)
        period = 0.0

    import rclpy
    from geometry_msgs.msg import WrenchStamped
    from rclpy.node import Node
    from std_msgs.msg import String

    rclpy.init(args=None)
    node = Node("ft_sensor_node")
    pub = node.create_publisher(WrenchStamped, topic, 50)
    diag = node.create_publisher(String, str(ros.get("diagnostics_topic", "/ft_sensor/diagnostics")), 10)
    print(f"已连接 {port} @ {baud}，{'轮询' if args.poll else '流模式 1 kHz'} → 发布 {topic}")

    stream = FrameParser(gravity)
    frames = 0
    started = time.time()
    last_report = started
    last_poll = 0.0
    try:
        while rclpy.ok():
            if args.poll and time.time() - last_poll >= period:
                link.write(poll_cmd)
                last_poll = time.time()
            chunk = link.read(link.in_waiting or 1)
            for item in stream.feed(chunk) if chunk else []:
                msg = WrenchStamped()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.header.frame_id = frame_id
                msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = item["force"]
                msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z = item["torque"]
                pub.publish(msg)
                frames += 1
            now = time.time()
            if now - last_report >= 2.0:
                rate = frames / max(1e-6, now - started)
                payload = {"frames": frames, "rate_hz": round(rate, 1),
                           "bad_frames": stream.bad_frames, "resyncs": stream.resyncs,
                           "port": port, "baud": baud}
                diag.publish(String(data=json.dumps(payload, ensure_ascii=False)))
                node.get_logger().info(
                    f"帧 {frames}｜{rate:.1f} Hz｜坏帧 {stream.bad_frames}｜重同步 {stream.resyncs}")
                last_report = now
            rclpy.spin_once(node, timeout_sec=0.0)
            if args.duration and now - started >= args.duration:
                break
            if frames == 0 and now - started > 5.0:
                print("5 s 内没有收到任何数据帧：检查波特率（460800）、接线 A/B、"
                      "传感器是否已上电，或是否被数采软件占用")
                break
    except KeyboardInterrupt:
        pass
    finally:
        try:
            link.write(stop_cmd)
            link.close()
        except Exception:      # noqa: BLE001
            pass
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:      # noqa: BLE001
            pass
    elapsed = time.time() - started
    print(f"结束：{frames} 帧 / {elapsed:.1f} s = {frames / max(1e-6, elapsed):.1f} Hz")
    return 0 if frames else 4


if __name__ == "__main__":
    raise SystemExit(main())
