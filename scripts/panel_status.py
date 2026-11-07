#!/usr/bin/env python3
"""面板用的状态卡：以 RViz 为准的 3D，面板只显示数据源是否就绪。

为什么不再自绘 3D（所有者 2026-11-07 决定）：
  · RViz 才能完整显示机械臂模型、TF、规划场景、点云与 Marker；
  · 自绘视图只能画"模型里已有的几何"，容易让人误以为系统缺东西；
  · Jazzy 没有 rviz_common 的 Python 绑定，无法把 RViz 嵌进面板。

所以面板负责"看得见状态、点得开 RViz"，3D 交给 RViz 本身。

数据源（每一行都对应一个真实的 ROS 检查，不是装饰）：
  /joint_states            关节状态 → 机械臂姿态、力矩条
  /compute_fk              MoveIt 正解服务 → 3D 里连杆位姿
  /sensors/camera/points   归一化点云 → 3D 里的现场点云
  /robot_description       机器人描述 → RViz 的 RobotModel
"""

from __future__ import annotations

import time

from PyQt5 import QtCore, QtGui, QtWidgets

C_BG = "#0b1015"
C_PANEL = "#121b24"
C_BORDER = "#1f2c39"
C_TEXT = "#dbe7f1"
C_MUTED = "#7d93a6"
C_OK = "#3ec98a"
C_WARN = "#d6a52f"
C_BAD = "#e05252"
C_ACCENT = "#2fd6c9"
MONO = "JetBrains Mono, DejaVu Sans Mono, monospace"


class TelemetryWorker(QtCore.QThread):
    """一个轻量 rclpy 节点：读关节/夹爪位姿，并统计各数据源的发布者数量。"""

    frame = QtCore.pyqtSignal(dict)

    def __init__(self, period_s: float = 1.0, parent=None) -> None:
        super().__init__(parent)
        self._period = period_s
        self._stop = False

    def stop(self) -> None:
        self._stop = True

    def run(self) -> None:                      # noqa: C901 - polling loop
        try:
            import rclpy
            from geometry_msgs.msg import PoseStamped
            from sensor_msgs.msg import JointState, PointCloud2
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        except Exception as exc:                # noqa: BLE001
            self.frame.emit({"error": f"ROS 不可用：{type(exc).__name__}"})
            return

        try:
            self._owns_context = not rclpy.ok()
            if self._owns_context:
                rclpy.init(args=None)
            node = rclpy.create_node("panel_status")
            latest: dict = {"cloud_frames": 0}
            node.create_subscription(JointState, "/joint_states",
                                     lambda m: latest.__setitem__("joints", m), 10)
            node.create_subscription(PoseStamped, "/tcp_pose_broadcaster/pose",
                                     lambda m: latest.__setitem__("tcp", m), 10)
            cloud_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                                   reliability=ReliabilityPolicy.BEST_EFFORT)

            def on_cloud(_msg) -> None:
                latest["cloud_frames"] = latest.get("cloud_frames", 0) + 1

            node.create_subscription(PointCloud2, "/sensors/camera/points", on_cloud, cloud_qos)
            from moveit_msgs.srv import GetPositionFK
            fk = node.create_client(GetPositionFK, "/compute_fk")
            last_cloud = 0

            while not self._stop and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=self._period / 2)
                payload: dict = {}
                joints = latest.get("joints")
                if joints is not None:
                    payload["efforts"] = {name: abs(value) for name, value in
                                          zip(joints.name, joints.effort)}
                tcp = latest.get("tcp")
                if tcp is not None:
                    position = tcp.pose.position
                    payload["tcp"] = (position.x, position.y, position.z)
                payload["joints_count"] = node.count_publishers("/joint_states")
                payload["cloud_publishers"] = node.count_publishers("/sensors/camera/points")
                payload["cloud_frames"] = latest.get("cloud_frames", 0) - last_cloud
                last_cloud = latest.get("cloud_frames", 0)
                payload["description_publishers"] = node.count_publishers("/robot_description")
                payload["compute_fk"] = bool(fk.service_is_ready())
                self.frame.emit(payload)
                for _ in range(int(self._period / 0.1)):
                    if self._stop:
                        break
                    time.sleep(0.1)
            node.destroy_node()
            if self._owns_context and rclpy.ok():
                rclpy.shutdown()
        except Exception as exc:                # noqa: BLE001
            self.frame.emit({"error": f"{type(exc).__name__}: {exc}"})


class SourceRow(QtWidgets.QWidget):
    """一行数据源：状态点 + 名称 + 右侧数值。"""

    def __init__(self, name: str, detail: str, parent=None) -> None:
        super().__init__(parent)
        self.setFixedHeight(30)
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(10, 0, 10, 0)
        layout.setSpacing(8)
        self.dot = QtWidgets.QLabel("●")
        self.dot.setStyleSheet(f"color:{C_MUTED}; font-size:12pt;")
        label = QtWidgets.QLabel(name)
        label.setStyleSheet(f"color:{C_TEXT}; font-size:10pt;")
        self.detail = QtWidgets.QLabel(detail)
        self.detail.setStyleSheet(f"color:{C_MUTED}; font-size:8.5pt;")
        self.value = QtWidgets.QLabel("—")
        self.value.setStyleSheet(f"color:{C_MUTED}; font-family:{MONO}; font-size:9pt;")
        layout.addWidget(self.dot, 0)
        layout.addWidget(label, 0)
        layout.addWidget(self.detail, 1)
        layout.addWidget(self.value, 0)

    def set_state(self, level: str, value: str) -> None:
        colour = {"ok": C_OK, "warn": C_WARN, "bad": C_BAD}.get(level, C_MUTED)
        self.dot.setStyleSheet(f"color:{colour}; font-size:12pt;")
        self.value.setStyleSheet(f"color:{colour}; font-family:{MONO}; font-size:9pt;")
        self.value.setText(value)


class RvizStatusCard(QtWidgets.QWidget):
    """RViz 的数据源状态卡：告诉操作者现在能不能看到完整 3D。"""

    open_rviz = QtCore.pyqtSignal()
    diagnose = QtCore.pyqtSignal()

    def __init__(self, probe: bool = True, parent=None) -> None:
        super().__init__(parent)
        self.setStyleSheet(f"background:{C_PANEL};")
        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(0, 6, 0, 6)
        root.setSpacing(0)

        header = QtWidgets.QHBoxLayout()
        header.setContentsMargins(10, 0, 10, 6)
        title = QtWidgets.QLabel("3D 由 RViz 显示")
        title.setStyleSheet(f"color:{C_TEXT}; font-size:10pt; font-weight:700;")
        subtitle = QtWidgets.QLabel("完整模型 / TF / 规划场景 / 点云")
        subtitle.setStyleSheet(f"color:{C_MUTED}; font-size:8.5pt;")
        header.addWidget(title, 0)
        header.addWidget(subtitle, 1)
        self.button_diag = QtWidgets.QPushButton("诊断")
        self.button_diag.setFixedWidth(56)
        self.button_diag.clicked.connect(self.diagnose.emit)
        self.button_rviz = QtWidgets.QPushButton("打开 RViz")
        self.button_rviz.setFixedWidth(88)
        self.button_rviz.setStyleSheet(
            f"background:#10423e; border:1px solid {C_ACCENT}; border-radius:2px;"
            f" color:#eaffff; font-weight:700; font-size:9pt; min-height:24px;")
        self.button_rviz.clicked.connect(self.open_rviz.emit)
        header.addWidget(self.button_diag, 0)
        header.addWidget(self.button_rviz, 0)
        root.addLayout(header)

        self.rows = {
            "joints": SourceRow("/joint_states", "关节状态 · 机械臂姿态与力矩"),
            "fk": SourceRow("/compute_fk", "MoveIt 正解 · 3D 中的连杆位姿"),
            "cloud": SourceRow("/sensors/camera/points", "点云 · 3D 中的现场"),
            "desc": SourceRow("/robot_description", "机器人描述 · RViz 的 RobotModel"),
        }
        for row in self.rows.values():
            root.addWidget(row)

        self.footer = QtWidgets.QLabel("等待 ROS 数据 …")
        self.footer.setContentsMargins(10, 8, 10, 0)
        self.footer.setStyleSheet(f"color:{C_MUTED}; font-size:8.5pt;")
        self.footer.setWordWrap(True)
        root.addWidget(self.footer)
        root.addStretch(1)

        self._worker: TelemetryWorker | None = None
        if probe:
            self.start()

    # ------------------------------------------------------------------
    def start(self) -> None:
        if self._worker is not None:
            return
        self._worker = TelemetryWorker(parent=self)
        self._worker.frame.connect(self.update_state)
        self._worker.start()

    def stop(self) -> None:
        if self._worker is not None:
            self._worker.stop()
            self._worker.wait(1500)
            self._worker = None

    def update_state(self, payload: dict) -> None:
        if "error" in payload:
            for row in self.rows.values():
                row.set_state("bad", "—")
            self.footer.setText(payload["error"])
            return
        joints = payload.get("joints_count", 0)
        self.rows["joints"].set_state("ok" if joints else "bad",
                                      f"{joints} 发布者" if joints else "缺失")
        fk = payload.get("compute_fk", False)
        self.rows["fk"].set_state("ok" if fk else "warn", "可用" if fk else "move_group 未运行")
        cloud = payload.get("cloud_publishers", 0)
        frames = payload.get("cloud_frames", 0)
        if cloud and frames:
            self.rows["cloud"].set_state("ok", f"{frames} 帧/秒")
        elif cloud:
            self.rows["cloud"].set_state("warn", "有发布者，无数据")
        else:
            self.rows["cloud"].set_state("bad", "缺失")
        desc = payload.get("description_publishers", 0)
        self.rows["desc"].set_state("ok" if desc else "bad",
                                    f"{desc} 发布者" if desc else "缺失")
        ready = sum(1 for key in ("joints", "fk", "cloud", "desc")
                    if payload.get({"joints": "joints_count", "fk": "compute_fk",
                                    "cloud": "cloud_publishers",
                                    "desc": "description_publishers"}[key]))
        if ready == 4:
            self.footer.setText("四项就绪：点「打开 RViz」即可看到完整场景")
        else:
            self.footer.setText(f"{ready}/4 就绪；缺失项在 RViz 里会是空白或报错")


def source_summary(payload: dict) -> str:
    """给日志用的一行摘要。"""
    return (f"joints={payload.get('joints_count', 0)} "
            f"fk={payload.get('compute_fk', False)} "
            f"cloud={payload.get('cloud_publishers', 0)}/{payload.get('cloud_frames', 0)}fps "
            f"desc={payload.get('description_publishers', 0)}")
