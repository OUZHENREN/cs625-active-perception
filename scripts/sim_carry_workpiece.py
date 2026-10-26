#!/usr/bin/env python3
"""仿真专用：把"夹住工件"实现成 Gazebo DetachableJoint 的 attach / detach。

为什么必须有这一步
------------------
真机的夹紧靠夹爪摩擦；仿真里没有力闭环。所以 `grasp → lift → transit → insert`
如果不做任何连接，就只是"空跑手臂"，工件一直留在桌上。

这个节点只做一件事（对外契约一个字没变）：
    /cs625/grasp_state_event == grasp_confirmed   → 发一次 Gazebo attach（焊在腕部）
    /cs625/grasp_state_event == release_confirmed → 发一次 Gazebo detach（工件自由落体）
    启动时                                        → 先发一次 detach 并确认
    每次状态变化                                   → 在 ROS 上广播 /cs625/carry_state

为什么不再用"把工件位姿 set_pose 到工具上"
------------------------------------------
实测两次都失败：`gz service` 一次调用 0.43 s，工件在两次刷新之间自由落体 0.31 m；
改成经 ros_gz_bridge 高频调用后，`set_pose` 只改位姿、不重置速度，重力让速度无界
累积（2 s 内工件被甩到 z ≈ -1300 m）。位姿写操作不是约束。DetachableJoint 是物理
引擎里的真 fixed 约束：dartsim 求解，工件重量/接触力/释放都由求解器处理。
attach 按**触发那一刻的当前相对位姿**建关节（gz-sim8 Physics.cc:
poseParent.inverse()*poseChild，两侧都是当前世界位姿），所以不会把工件瞬移到
名义夹持位 —— 但这也意味着 **attach 必须发生在机械臂停在抓取位姿时**。

插件启动就是"已吸附"
--------------------
gz-sim8 的 DetachableJoint `attachRequested{true}`：机器人一 spawn 就会把 19 kg
工件焊到腕上。所以本节点启动第一件事必须是发 detach 并确认 `detached`，否则
第一次观测运动就会把工件从桌上拖走。启动顺序必须是：仿真 → 本节点 → run_task。

依赖的仿真侧改动（见 scripts/sim_robot_description.py）
------------------------------------------------------
robot_description 里注入了一段 model 级插件（parent_link=wrist_3_link，
child=shielding_module/shielding_module_link），话题：
    /cs625/carry/attach   gz.msgs.Empty   （发一次 = 焊住）
    /cs625/carry/detach   gz.msgs.Empty   （发一次 = 松开）
    /cs625/carry/state    gz.msgs.StringMsg（"attached" / "detached"）

用法
----
    source scripts/source_dev_env.sh --full
    export ROS_DOMAIN_ID=79                       # 与仿真 launch 同一个 ROS 域
    python3 scripts/sim_carry_workpiece.py        # 仿真起来之后、run_task 之前

Gazebo 分区名自动取（环境变量 → runtime/gz_partition.txt，后者由 sim_base 写入）。
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

REPO_ROOT = Path(__file__).resolve().parent.parent
PARTITION_FILE = REPO_ROOT / "runtime" / "gz_partition.txt"

ATTACH_TOPIC = "/cs625/carry/attach"
DETACH_TOPIC = "/cs625/carry/detach"
STATE_TOPIC = "/cs625/carry/state"
STATE_PUBLISHER = "/cs625/carry_state"   # ROS 侧（供 run_pick_and_place.py 同步）


def resolve_partition() -> str:
    """本次 Gazebo 会话的分区名：环境变量优先，其次 sim_base 写下的运行时文件。

    sim_base 每次启动都生成唯一分区名，避免连到上一次没退干净的服务器。命令行
    工具必须用同一个分区才看得见这台服务器；找不到时不是报错，而是所有 gz 调用
    都静默超时，所以这里显式解析并在日志里打出来。
    """
    for name in ("IGN_PARTITION", "GZ_PARTITION"):
        value = os.environ.get(name, "").strip()
        if value:
            return value
    try:
        return PARTITION_FILE.read_text(encoding="utf-8").strip()
    except OSError:
        return ""


class Carry(Node):
    def __init__(self, args) -> None:
        super().__init__("sim_carry_workpiece")
        self.args = args
        partition = resolve_partition()
        self.gz_env = dict(os.environ)
        if partition:
            self.gz_env["IGN_PARTITION"] = partition
            self.gz_env["GZ_PARTITION"] = partition
        self.state_pub = self.create_publisher(String, STATE_PUBLISHER, 10)
        self.create_subscription(String, "/cs625/grasp_state_event", self.on_event, 10)
        self.get_logger().info(
            f"carry ready: world={args.world} model={args.model} "
            f"partition={partition or '(none!)'} attach={ATTACH_TOPIC} "
            f"detach={DETACH_TOPIC}")

    # ------------------------------------------------------------ gz 交互
    def publish_empty(self, topic: str) -> bool:
        """在 Gazebo 传输层发一条 gz.msgs.Empty（命令：attach / detach）。"""
        try:
            run = subprocess.run(
                ["gz", "topic", "-t", topic, "-m", "gz.msgs.Empty", "-p", "unused: true"],
                capture_output=True, text=True, timeout=15, env=self.gz_env)
        except (OSError, subprocess.TimeoutExpired) as exc:
            self.get_logger().error(f"{topic} 发布失败: {type(exc).__name__}: {exc}")
            return False
        if run.returncode != 0:
            tail = " ".join(((run.stdout or "") + (run.stderr or "")).split())[:200]
            self.get_logger().error(f"{topic} 发布返回 {run.returncode}: {tail}")
            return False
        return True

    def publish_and_read(self, wanted: str) -> str:
        """先挂监听、再发一条 attach/detach，返回读到的状态字符串（读不到为 ""）。

        为什么必须"先监听、再发命令"：Gazebo 的 /cs625/carry/state 只在状态**变化**
        时发一条（已实测：刚连上时长时间读不到，一旦发命令就立刻收到），发完再听
        就已经错过了。所以每次都先起 `gz topic -e -n 1`，再发命令，最后读那一条。
        """
        topic = ATTACH_TOPIC if wanted == "attached" else DETACH_TOPIC
        try:
            listener = subprocess.Popen(
                ["gz", "topic", "-e", "-t", STATE_TOPIC, "-n", "1"],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                env=self.gz_env)
        except OSError as exc:
            self.get_logger().error(f"监听 {STATE_TOPIC} 失败: {exc}")
            return ""
        time.sleep(0.3)   # 让监听器完成 discovery（本地传输，300 ms 足够）
        sent = self.publish_empty(topic)
        try:
            out, _err = listener.communicate(timeout=self.args.timeout)
        except subprocess.TimeoutExpired:
            listener.kill()
            listener.communicate()
            out = ""
        match = re.search(r'data:\s*"?([a-z_]+)"?', out or "")
        state = match.group(1) if match else ""
        if not sent:
            self.get_logger().warn(f"发 {topic} 失败")
        return state

    def try_state(self, wanted: str) -> bool:
        state = self.publish_and_read(wanted)
        if state == wanted:
            self.get_logger().info(f"{wanted} 已确认")
            return True
        self.get_logger().debug(f"期望 {wanted}，读到 {state!r}")
        return False

    def set_state(self, wanted: str, why: str) -> bool:
        """把工件状态置成 wanted（attached / detached），并确认。

        状态话题只在"变化"时发消息，所以命令没引起变化时无法区分
        "本来就是目标状态" 和 "插件没生效"。处理办法：先切到相反状态——能切成功
        就说明链路活着，再切回来即可；两步都切不动，就是插件/child/分区有问题。
        """
        other = "detached" if wanted == "attached" else "attached"
        if self.try_state(wanted):
            self.broadcast(wanted)
            self.get_logger().info(f"{why}: {wanted} 已确认")
            return True
        # 没有变化：先切到相反状态验证链路，再切回来（启动时的"本来就 detached"
        # 走的就是这条路；抓取时若本来就 attached 也只是短暂松开再焊回，臂是静止的）。
        if self.try_state(other):
            self.get_logger().warn(
                f"{why}: 直接发 {wanted} 没引起变化，已切到 {other}；再切回 {wanted}")
            if self.try_state(wanted):
                self.broadcast(wanted)
                self.get_logger().info(f"{why}: {wanted} 已确认（经 {other} 中转）")
                return True
        self.broadcast("failed")
        self.get_logger().error(
            f"{why}: 无法确认 {wanted}。排查：(1) robot_description 里是否注入了 "
            "gz-sim-detachable-joint-system，(2) 世界里是否有 shielding_module / "
            "shielding_module_link，(3) Gazebo 分区是否一致（runtime/gz_partition.txt）")
        return False

    def broadcast(self, state: str) -> None:
        self.state_pub.publish(String(data=state))
        self.get_logger().info(f"carry_state -> {state}")

    # ------------------------------------------------------------ 动作
    def detach(self, why: str) -> bool:
        self.get_logger().info(f"detach（{why}）")
        return self.set_state("detached", why)

    def attach(self, why: str) -> bool:
        self.get_logger().info(f"attach（{why}）")
        return self.set_state("attached", why)

    def on_event(self, msg: String) -> None:
        data = (msg.data or "").strip()
        if data == "grasp_confirmed":
            self.attach("grasp_confirmed")
        elif data == "release_confirmed":
            self.detach("release_confirmed")


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--world", default="cs625_real_cell")
    parser.add_argument("--model", default="shielding_module")
    parser.add_argument("--timeout", type=float, default=4.0,
                        help="每次 attach/detach 等待状态确认的秒数")
    parser.add_argument("--no-initial-detach", action="store_true",
                        help="跳过启动时的 detach（调试用；正常流程不要加）")
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = Carry(args)

    # 插件启动即"已吸附"，所以第一件事必须是松开；失败就明确退出，不静默继续。
    if not args.no_initial_detach:
        deadline = time.time() + 30.0
        ok = False
        while time.time() < deadline:
            if node.detach("启动：插件默认就是吸附状态"):
                ok = True
                break
            time.sleep(1.0)
        if not ok:
            node.get_logger().error(
                "30 s 内无法确认 detach：检查 (1) sim 是否已起、(2) robot_description 里"
                "是否注入了 gz-sim-detachable-joint-system、(3) 分区是否一致、"
                "(4) 世界里是否有 shielding_module/shielding_module_link")
            node.destroy_node()
            rclpy.shutdown()
            return 1

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
