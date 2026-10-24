#!/usr/bin/env python3
"""P4 自动决策节点（正式版）：视觉失败 → 有界重试 → 只规划。

数据流：
  /box_pre_grasp_tcp_pose（有效位姿） ─┐
  /rvs/pose_status（NG 状态，可选）   ─┴→ 本节点 → /cs625/plan_to_pose_srv（只规划）
                                              → /p4/state（状态 JSON）
                                              → /p4/reobserve_request（重观测请求）
                                              →（可选）RVS 7000 的 T1/T3 命令

实时 TCP 位姿来源（`pose_source` 参数）：
  state  : /cs625/raw_state（cs625_state_monitor/CS625State）的 actual_tcp_* 字段；首选
  tf     : base_link -> tcp_pose（与师兄 moveit_planner_client_node 一致）
  topic  : 任意外部 PoseStamped（m/rad）
  static : 调试用 rvs_pose 字符串（旧行为）

安全边界：
- 只调用规划服务，**不调用** /cs625/execute_last_plan；
- RVS 重观测默认 dry-run（rvs_dry_run=true），不会向真实机器人发命令；
- 无真机时保持 execute=false。
"""

from __future__ import annotations

import json
import socket
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from cs625_task_orchestrator.p4_decision_logic import Action, DecisionState
from cs625_task_orchestrator.p4_pose_source import (
    format_values,
    parse_static_values,
    pose_to_values,
    state_to_flange_values,
    state_to_values,
    transform_to_values,
)


class P4DecisionNode(Node):
    def __init__(self) -> None:
        super().__init__("p4_decision_node")

        self.declare_parameter("pose_topic", "/box_pre_grasp_tcp_pose")
        self.declare_parameter("status_topic", "/rvs/pose_status")
        self.declare_parameter("plan_service", "/cs625/plan_to_pose_srv")
        self.declare_parameter("pose_timeout_sec", 5.0)
        self.declare_parameter("max_retries", 2)
        self.declare_parameter("state_topic", "/p4/state")
        self.declare_parameter("reobserve_topic", "/p4/reobserve_request")
        self.declare_parameter("result_file", "")

        self.declare_parameter("pose_source", "state")
        self.declare_parameter("raw_state_topic", "/cs625/raw_state")
        self.declare_parameter("report_frame", "flange")
        self.declare_parameter("tf_base_frame", "base_link")
        self.declare_parameter("tf_tcp_frame", "tcp_pose")
        self.declare_parameter("tf_timeout_sec", 0.2)
        self.declare_parameter("tcp_pose_topic", "/cs625/current_tcp_pose")

        self.declare_parameter("rvs_host", "")
        self.declare_parameter("rvs_port", 7000)
        self.declare_parameter("rvs_command", "T3")
        self.declare_parameter("rvs_pose", "")
        self.declare_parameter("rvs_timeout_sec", 2.0)
        self.declare_parameter("rvs_dry_run", True)

        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.plan_service = str(self.get_parameter("plan_service").value)
        self.pose_timeout = float(self.get_parameter("pose_timeout_sec").value)
        self.result_file = str(self.get_parameter("result_file").value)
        self.raw_state_topic = str(self.get_parameter("raw_state_topic").value)
        self.report_frame = str(self.get_parameter("report_frame").value)

        self.pose_source = str(self.get_parameter("pose_source").value)
        self.tf_base_frame = str(self.get_parameter("tf_base_frame").value)
        self.tf_tcp_frame = str(self.get_parameter("tf_tcp_frame").value)
        self.tf_timeout = float(self.get_parameter("tf_timeout_sec").value)
        self.tcp_pose_topic = str(self.get_parameter("tcp_pose_topic").value)

        self.rvs_host = str(self.get_parameter("rvs_host").value)
        self.rvs_port = int(self.get_parameter("rvs_port").value)
        self.rvs_command = str(self.get_parameter("rvs_command").value)
        self.rvs_pose = str(self.get_parameter("rvs_pose").value)
        self.rvs_timeout = float(self.get_parameter("rvs_timeout_sec").value)
        self.rvs_dry_run = bool(self.get_parameter("rvs_dry_run").value)

        self.state = DecisionState(max_retries=int(self.get_parameter("max_retries").value))
        self.phase = "WAIT_POSE"
        self.finished = False
        self.pending_future = None
        self.latest_state = None
        self.latest_tcp_pose: PoseStamped | None = None
        self.tf_buffer = None
        self.tf_listener = None

        self.state_pub = self.create_publisher(
            String, str(self.get_parameter("state_topic").value), 10
        )
        self.reobserve_pub = self.create_publisher(
            String, str(self.get_parameter("reobserve_topic").value), 10
        )
        self.create_subscription(PoseStamped, self.pose_topic, self.on_pose, 10)
        self.create_subscription(String, self.status_topic, self.on_status, 10)
        self._setup_pose_source()

        plan_type = self._load_plan_type()
        self.client = self.create_client(plan_type, self.plan_service)
        if not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning(
                f"规划服务 {self.plan_service} 暂不可用；收到位姿时会继续尝试。"
            )

        # 计时必须在服务发现之后开始，否则超时会提前触发。
        self.deadline = time.monotonic() + self.pose_timeout
        self.create_timer(0.2, self.tick)

        self.get_logger().info(
            f"P4 启动: pose={self.pose_topic} status={self.status_topic} "
            f"plan={self.plan_service} timeout={self.pose_timeout}s "
            f"max_retries={self.state.max_retries} "
            f"pose_source={self.pose_source} report_frame={self.report_frame} "
            f"rvs={'%s:%d/%s' % (self.rvs_host, self.rvs_port, self.rvs_command) if self.rvs_host else 'disabled'} "
            f"dry_run={self.rvs_dry_run}"
        )
        self.publish_state("WAIT_POSE")

    def _setup_pose_source(self) -> None:
        if self.pose_source == "state":
            state_type = self._load_state_type()
            self.create_subscription(
                state_type, self.raw_state_topic, self.on_state, 10
            )
        elif self.pose_source == "topic":
            self.create_subscription(
                PoseStamped, self.tcp_pose_topic, self.on_tcp_pose, 10
            )
        elif self.pose_source == "tf":
            import tf2_ros

            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        elif self.pose_source == "static":
            if not self.rvs_pose:
                self.get_logger().warning("pose_source=static 但 rvs_pose 为空")
        else:
            raise ValueError(
                f"未知 pose_source={self.pose_source!r}；可选 state/tf/topic/static"
            )

    # ------------------------- 回调 -------------------------
    def on_pose(self, msg: PoseStamped) -> None:
        if self.finished or self.phase != "WAIT_POSE":
            return
        p = msg.pose.position
        self.get_logger().info(
            f"收到有效位姿: frame={msg.header.frame_id} "
            f"pos=({p.x:.3f},{p.y:.3f},{p.z:.3f})"
        )
        action = self.state.on_pose_valid()
        if action is Action.PLAN:
            self.plan(msg)

    def on_status(self, msg: String) -> None:
        if self.finished:
            return
        data = msg.data.strip()
        if data.upper().startswith("NG"):
            self.get_logger().warning(f"收到视觉失败状态: '{data}' → VISION_NO_MATCH")
            self.handle_failure("VISION_STATUS", self.state.on_vision_no_match())

    def on_state(self, msg) -> None:
        self.latest_state = msg

    def on_tcp_pose(self, msg: PoseStamped) -> None:
        self.latest_tcp_pose = msg

    def tick(self) -> None:
        if self.finished:
            return
        if self.phase == "WAIT_POSE" and time.monotonic() > self.deadline:
            self.get_logger().warning("等待位姿超时 → VISION_NO_MATCH")
            self.handle_failure("POSE_TIMEOUT", self.state.on_timeout())

    # ------------------------- 状态机 -------------------------
    def plan(self, pose: PoseStamped) -> None:
        self.phase = "PLANNING"
        self.state.planning_calls += 1
        self.publish_state("PLANNING")
        request = self._make_plan_request()
        request.target_pose = pose
        self.pending_future = self.client.call_async(request)
        self.pending_future.add_done_callback(self.on_plan_done)

    def on_plan_done(self, future) -> None:
        if self.finished:
            return
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 - 服务异常也要进入失败码
            self.get_logger().error(f"规划服务调用异常: {exc}")
            self.handle_failure("PLAN_EXCEPTION", self.state.on_plan_failed())
            return
        if response.success:
            self.get_logger().info(
                f"规划成功: planning_time={response.planning_time:.4f}s（未执行）"
            )
            self.state.on_plan_ok()
            self.finish("SUCCESS", "PLAN_OK")
        else:
            action = self.state.on_plan_failed()
            self.get_logger().warning(f"规划失败: {response.message} → PLAN_FAILED")
            self.handle_failure("PLAN_FAILED", action)

    def handle_failure(self, event: str, action: Action) -> None:
        if action is Action.RETRY:
            self.phase = "WAIT_POSE"
            self.deadline = time.monotonic() + self.pose_timeout
            retry_payload = {
                "event": event,
                "failure": self.state.failures[-1].value,
                "retries": self.state.retries,
                "max_retries": self.state.max_retries,
            }
            self.reobserve_pub.publish(String(data=json.dumps(retry_payload, ensure_ascii=False)))
            self._start_rvs_reobserve()
            self.get_logger().warning(
                f"重试 {self.state.retries}/{self.state.max_retries}，已请求重新观测"
            )
            self.publish_state("RETRY", event)
        elif action is Action.ABORT:
            self.finish("ABORT", event)
        else:
            self.get_logger().error(f"未预期的失败动作: {action}")

    def finish(self, decision: str, event: str) -> None:
        self.finished = True
        self.phase = "DONE"
        result = self.state.result()
        result.update(
            {
                "decision": decision,
                "final_code": "SUCCESS" if decision == "SUCCESS" else "RETRY_EXHAUSTED",
                "last_event": event,
                "pose_topic": self.pose_topic,
                "plan_service": self.plan_service,
                "status_topic": self.status_topic,
                "pose_source": self.pose_source,
                "executed": False,
            }
        )
        if self.result_file:
            path = Path(self.result_file)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
        self.publish_state(decision, event)
        self.get_logger().info(f"P4 结束: {decision}（未执行任何运动）")

    # ------------------------- 实时 TCP 位姿 -------------------------
    def _current_pose_text(self) -> tuple[str, str] | None:
        """返回 (mm/deg 字符串, 来源描述)；拿不到就返回 None，绝不瞎发。"""
        try:
            if self.pose_source == "state":
                if self.latest_state is None:
                    raise RuntimeError(f"尚未收到 {self.raw_state_topic}")
                if self.report_frame == "flange":
                    values = state_to_flange_values(self.latest_state)
                    return format_values(values), "state/flange"
                if self.report_frame == "tcp":
                    values = state_to_values(self.latest_state)
                    return format_values(values), "state/tcp"
                raise ValueError(f"未知 report_frame={self.report_frame!r}；可选 flange/tcp")
            if self.pose_source == "topic":
                if self.latest_tcp_pose is None:
                    raise RuntimeError(f"尚未收到 {self.tcp_pose_topic}")
                values = pose_to_values(self.latest_tcp_pose.pose)
                return format_values(values), f"topic:{self.tcp_pose_topic}"
            if self.pose_source == "tf":
                if self.tf_buffer is None:
                    raise RuntimeError("TF buffer 未初始化")
                transform = self.tf_buffer.lookup_transform(
                    self.tf_base_frame,
                    self.tf_tcp_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=self.tf_timeout),
                )
                values = transform_to_values(transform)
                return format_values(values), f"tf:{self.tf_base_frame}->{self.tf_tcp_frame}"
            if self.pose_source == "static":
                values = parse_static_values(self.rvs_pose)
                return format_values(values), "static"
        except Exception as exc:  # noqa: BLE001 - 位姿不可用是正常失败路径
            self.get_logger().warning(
                f"无法获取实时 TCP 位姿({self.pose_source})，跳过 RVS 命令: {exc}"
            )
            return None
        raise ValueError(f"未知 pose_source={self.pose_source!r}")

    def _start_rvs_reobserve(self) -> None:
        pose = self._current_pose_text()
        if pose is None:
            return
        text, source = pose
        self.get_logger().info(f"实时 TCP 位姿({source}): {text}")

        if not self.rvs_host:
            self.get_logger().info("RVS 未配置，仅发布 /p4/reobserve_request（dry-run）")
            return
        if self.rvs_dry_run:
            self.get_logger().info("rvs_dry_run=true，不向 RVS 发送命令（安全）")
            return
        threading.Thread(target=self._send_rvs_command, args=(text,), daemon=True).start()

    def _send_rvs_command(self, pose_text: str) -> None:
        payload = f"{self.rvs_command},{pose_text}\r\n"
        try:
            with socket.create_connection(
                (self.rvs_host, self.rvs_port), timeout=self.rvs_timeout
            ) as sock:
                sock.sendall(payload.encode("utf-8"))
            self.get_logger().info(f"已向 RVS 发送重观测命令: {payload.strip()}")
        except OSError as exc:
            self.get_logger().error(f"RVS 重观测命令发送失败: {exc}")

    # ------------------------- 工具 -------------------------
    def publish_state(self, phase: str, event: str | None = None) -> None:
        payload = {
            "phase": phase,
            "event": event,
            "decision": self.state.final,
            "attempts": self.state.attempts,
            "retries": self.state.retries,
            "failures": [code.value for code in self.state.failures],
            "timestamp": time.time(),
        }
        self.state_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    @staticmethod
    def _load_plan_type():
        try:
            from cs625_trajectory_tools.srv import PlanToPose

            return PlanToPose
        except ImportError as exc:  # pragma: no cover - 环境缺失时给出明确提示
            raise RuntimeError(
                "找不到 cs625_trajectory_tools.srv.PlanToPose；请先 source "
                "~/cs625_legacy_underlay_jazzy/install/setup.bash"
            ) from exc

    @staticmethod
    def _load_state_type():
        try:
            from cs625_state_monitor.msg import CS625State

            return CS625State
        except ImportError as exc:  # pragma: no cover - 环境缺失时给出明确提示
            raise RuntimeError(
                "找不到 cs625_state_monitor.msg.CS625State；请先 source "
                "~/cs625_legacy_underlay_jazzy/install/setup.bash"
            ) from exc

    @classmethod
    def _make_plan_request(cls):
        return cls._load_plan_type().Request()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P4DecisionNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
