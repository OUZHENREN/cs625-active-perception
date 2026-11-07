#!/usr/bin/env python3
"""3D 场景视图（自绘，无 RViz 依赖）：碰撞体 + 机械臂 + TCP。

为什么不用 RViz：Jazzy 的 rviz_common 没有 Python 绑定，无法嵌进 PyQt5 面板；
而操作面板需要的只是"看得见碰撞体、看得见臂、看得见 TCP"，不需要完整显示管线。

离线也能用：场景来自 `runtime/environment/latest/model.json`（桌子/插槽/工件/体素），
所以没有 ROS 时仍能画出工位；机械臂与 TCP 由可选的 ROS 工作线程补齐。

坐标：世界系 base_link（X 前、Y 左、Z 上，REP-103）。视图可鼠标左键拖动旋转、滚轮缩放。
"""

from __future__ import annotations

import json
import math
import pathlib
import threading
import time

from PyQt5 import QtCore, QtGui, QtWidgets

REPO = pathlib.Path(__file__).resolve().parents[1]
MODEL = REPO / "runtime" / "environment" / "latest" / "model.json"

# 从 base_link 到末端的链路（厂商 URDF 的固定顺序）
ARM_LINKS = ["base_link", "shoulder_link", "upperarm_link", "forearm_link",
             "wrist_1_link", "wrist_2_link", "wrist_3_link", "flange", "tool0"]

C_TABLE = "#3b4a5a"
C_VOXEL = "#22303d"
C_SLOT = "#2fd6c9"
C_PART = "#d6a52f"
C_ARM = "#7fb8d6"
C_ARM_JOINT = "#cfe6f3"
C_TCP = "#e05252"
C_GRID = "#1d2836"


def _warn(state: int) -> bool:
    return state in (2, 4, 5, 6, 7)


class SceneWorker(QtCore.QThread):
    """Collect arm link poses + TCP pose over ROS without blocking the UI.

    rclpy is imported lazily inside the thread: on a machine without ROS (or in the smoke
    test) the scene simply stays static.
    """

    frame = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)

    def __init__(self, period_s: float = 0.2, parent=None) -> None:
        super().__init__(parent)
        self._period = period_s
        self._stop = threading.Event()

    def stop(self) -> None:
        self._stop.set()

    def run(self) -> None:                      # noqa: C901 - simple polling loop
        try:
            import rclpy
            from geometry_msgs.msg import PoseStamped
            from moveit_msgs.srv import GetPositionFK
            from moveit_msgs.msg import RobotState
            from sensor_msgs.msg import JointState
        except Exception as exc:                # noqa: BLE001
            self.failed.emit(f"ROS 不可用：{type(exc).__name__}")
            return

        node = None
        try:
            rclpy.init(args=None)
            node = rclpy.create_node("panel_scene")
            latest: dict = {}
            node.create_subscription(PoseStamped, "/tcp_pose_broadcaster/pose",
                                     lambda m: latest.__setitem__("tcp", m), 10)
            node.create_subscription(JointState, "/joint_states",
                                     lambda m: latest.__setitem__("joints", m), 10)
            fk = node.create_client(GetPositionFK, "/compute_fk")
            order = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                     "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
            announced = False
            while not self._stop.is_set() and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=self._period)
                payload = {"links": [], "tcp": None, "efforts": {}, "q_deg": {}}
                joints = latest.get("joints")
                if joints is not None:
                    names = list(joints.name)
                    payload["efforts"] = {n: abs(e) for n, e in zip(names, joints.effort)}
                    payload["q_deg"] = {n: math.degrees(q)
                                        for n, q in zip(names, joints.position)}
                if joints is not None and fk.service_is_ready():
                    positions = dict(zip(joints.name, joints.position))
                    if all(k in positions for k in order):
                        state = RobotState()
                        state.joint_state = joints
                        request = GetPositionFK.Request()
                        request.header.frame_id = "world"
                        request.fk_link_names = list(ARM_LINKS)
                        request.robot_state = state
                        future = fk.call_async(request)
                        deadline = time.time() + 1.0
                        while not future.done() and time.time() < deadline:
                            rclpy.spin_once(node, timeout_sec=0.02)
                        result = future.result()
                        if result is not None and result.error_code.val == 1:
                            for name, pose in zip(result.fk_link_names, result.pose_stamped):
                                p = pose.pose.position
                                payload["links"].append((name, (p.x, p.y, p.z)))
                tcp = latest.get("tcp")
                if tcp is not None:
                    p = tcp.pose.position
                    payload["tcp"] = (p.x, p.y, p.z)
                if payload["links"] or payload["tcp"] or payload["efforts"]:
                    self.frame.emit(payload)
                elif not announced:
                    announced = True
                    self.failed.emit("等待 /joint_states 与 /compute_fk")
        except Exception as exc:                # noqa: BLE001
            self.failed.emit(f"场景线程异常：{type(exc).__name__}: {exc}")
        finally:
            try:
                if node is not None:
                    node.destroy_node()
                import rclpy
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:                   # noqa: BLE001
                pass


class SceneView(QtWidgets.QWidget):
    """Orthographic 3D view of the cell: collision bodies, arm chain, TCP."""

    telemetry = QtCore.pyqtSignal(object)

    def __init__(self, parent=None, probe: bool = True) -> None:
        super().__init__(parent)
        self.setMinimumSize(320, 240)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.azimuth = math.radians(-40.0)
        self.elevation = math.radians(24.0)
        self.scale = 210.0
        self.center = (0.55, 0.30, 0.10)
        self._drag: tuple[float, float] | None = None
        self._boxes: list[tuple[tuple[float, float, float], tuple[float, float, float], str]] = []
        self._links: list[tuple[str, tuple[float, float, float]]] = []
        self._tcp: tuple[float, float, float] | None = None
        self._status = "静态场景"
        self._load_model()
        self._worker: SceneWorker | None = None
        if probe:
            self.start_probe()

    # ------------------------------------------------------------------ data
    def _load_model(self) -> None:
        """Read tables / slot / workpiece / voxels from the measured cell model."""
        if not MODEL.exists():
            self._status = "无环境模型"
            return
        try:
            data = json.loads(MODEL.read_text())
        except (OSError, json.JSONDecodeError):
            self._status = "环境模型损坏"
            return
        boxes = self._boxes
        for name, table in (data.get("tables") or {}).items():
            center = table.get("center") or table.get("xyz")
            size = table.get("size")
            if center and size:
                boxes.append((tuple(center), tuple(size), C_TABLE))
        voxels = data.get("collisions") or []
        stride = max(1, len(voxels) // 420)             # cap the draw count
        for index, voxel in enumerate(voxels):
            if index % stride:
                continue
            center = voxel.get("center")
            size = voxel.get("size")
            if center and size:
                boxes.append((tuple(center), tuple(size), C_VOXEL))
        for key, color, extent in (("slot", C_SLOT, 0.11), ("workpiece", C_PART, 0.07)):
            entry = data.get(key) or {}
            xyz = entry.get("xyz") or entry.get("centre")
            if xyz:
                boxes.append((tuple(xyz), (extent, extent, extent), color))
        if boxes:
            xs = [c[0] for c, _, _ in boxes]
            ys = [c[1] for c, _, _ in boxes]
            zs = [c[2] for c, _, _ in boxes]
            self.center = ((min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2,
                           (min(zs) + max(zs)) / 2)
            span = max(max(xs) - min(xs), max(ys) - min(ys), 1e-3)
            self.scale = max(80.0, min(420.0, 320.0 / span))
        self._status = f"{len(boxes)} 个体"

    def start_probe(self) -> None:
        self._worker = SceneWorker(parent=self)
        self._worker.frame.connect(self._on_frame)
        self._worker.failed.connect(self._on_failed)
        self._worker.start()

    def stop_probe(self) -> None:
        if self._worker is not None:
            self._worker.stop()
            self._worker.wait(1500)
            self._worker = None

    def _on_frame(self, payload) -> None:
        links = payload.get("links") or []
        if links:
            self._links = links
        tcp = payload.get("tcp")
        if tcp:
            self._tcp = tcp
        self._status = "在线" if self._links or self._tcp else self._status
        self.telemetry.emit(payload)
        self.update()

    def _on_failed(self, text: str) -> None:
        self._status = text
        self.update()

    # ------------------------------------------------------------- projection
    def _project(self, point: tuple[float, float, float]) -> QtCore.QPointF:
        x, y, z = point
        cx, cy, cz = self.center
        x, y, z = x - cx, y - cy, z - cz
        ca, sa = math.cos(self.azimuth), math.sin(self.azimuth)
        rx = ca * x - sa * y
        ry = sa * x + ca * y
        ce, se = math.cos(self.elevation), math.sin(self.elevation)
        screen_x = rx
        screen_y = -(ce * z - se * ry)
        return QtCore.QPointF(self.width() / 2 + screen_x * self.scale,
                              self.height() / 2 + screen_y * self.scale)

    def _depth(self, point: tuple[float, float, float]) -> float:
        x, y, z = point[0] - self.center[0], point[1] - self.center[1], point[2] - self.center[2]
        ca, sa = math.cos(self.azimuth), math.sin(self.azimuth)
        return sa * x + ca * y + z

    # ----------------------------------------------------------------- painting
    def paintEvent(self, event) -> None:            # noqa: N802
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        painter.fillRect(self.rect(), QtGui.QColor("#0b1015"))
        self._draw_floor(painter)

        items: list[tuple[float, str, object]] = []
        for center, size, color in self._boxes:
            items.append((self._depth(center), "box", (center, size, color)))
        for index in range(len(self._links) - 1):
            p0 = self._links[index][1]
            p1 = self._links[index + 1][1]
            mid = ((p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2, (p0[2] + p1[2]) / 2)
            items.append((self._depth(mid), "link", (p0, p1)))
        if self._tcp is not None:
            items.append((self._depth(self._tcp), "tcp", self._tcp))
        items.sort(key=lambda item: item[0])

        for _, kind, payload in items:
            if kind == "box":
                self._draw_box(painter, *payload)
            elif kind == "link":
                p0, p1 = payload
                painter.setPen(QtGui.QPen(QtGui.QColor(C_ARM), 3))
                painter.drawLine(self._project(p0), self._project(p1))
                painter.setPen(QtCore.Qt.NoPen)
                painter.setBrush(QtGui.QColor(C_ARM_JOINT))
                painter.drawEllipse(self._project(p0), 3, 3)
            else:
                self._draw_tcp(painter, payload)

        painter.setPen(QtGui.QColor("#5f7383"))
        font = painter.font()
        font.setPointSizeF(max(7.0, font.pointSizeF() - 1))
        painter.setFont(font)
        painter.drawText(8, self.height() - 8,
                         f"{self._status} · 拖动旋转 / 滚轮缩放")
        painter.end()

    def _draw_floor(self, painter: QtGui.QPainter) -> None:
        painter.setPen(QtGui.QPen(QtGui.QColor(C_GRID), 1))
        for i in range(-10, 11):
            p0 = self._project((i * 0.1, -0.5, -0.15))
            p1 = self._project((i * 0.1, 0.9, -0.15))
            painter.drawLine(p0, p1)
            p0 = self._project((-0.1, i * 0.07, -0.15))
            p1 = self._project((1.5, i * 0.07, -0.15))
            painter.drawLine(p0, p1)

    def _draw_box(self, painter: QtGui.QPainter, center, size, color: str) -> None:
        """实体盒：顶面 + 两个可见侧面，用明暗区分朝向（比线框更容易看出结构）。"""
        hx, hy, hz = size[0] / 2, size[1] / 2, size[2] / 2
        corners = [(center[0] + sx * hx, center[1] + sy * hy, center[2] + sz * hz)
                   for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
        # 索引：sx(-1,+1) 外层、sy 中层、sz 内层 → 0..7
        top = [4, 5, 7, 6]            # z = +hz 的四个角
        west = [0, 2, 6, 4]           # x = -hx
        south = [0, 1, 5, 4]          # y = -hy
        base = QtGui.QColor(color)
        faces = [(top, 1.25), (west, 0.85), (south, 0.62)]
        for indices, shade in faces:
            polygon = QtGui.QPolygonF([self._project(corners[i]) for i in indices])
            shaded = QtGui.QColor(
                min(255, int(base.red() * shade)),
                min(255, int(base.green() * shade)),
                min(255, int(base.blue() * shade)))
            painter.setBrush(shaded)
            painter.setPen(QtGui.QPen(shaded.darker(140), 0.6))
            painter.drawPolygon(polygon)

    def _draw_tcp(self, painter: QtGui.QPainter, tcp) -> None:
        origin = self._project(tcp)
        painter.setPen(QtGui.QPen(QtGui.QColor(C_TCP), 2))
        for axis, color in (((0.06, 0, 0), "#e05252"), ((0, 0.06, 0), "#4ec27a"),
                            ((0, 0, 0.06), "#4a90d9")):
            end = (tcp[0] + axis[0], tcp[1] + axis[1], tcp[2] + axis[2])
            painter.setPen(QtGui.QPen(QtGui.QColor(color), 2))
            painter.drawLine(origin, self._project(end))
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(QtGui.QColor(C_TCP))
        painter.drawEllipse(origin, 4, 4)

    # ------------------------------------------------------------------ input
    def mousePressEvent(self, event) -> None:       # noqa: N802
        self._drag = (event.x(), event.y())

    def mouseMoveEvent(self, event) -> None:        # noqa: N802
        if self._drag is None:
            return
        dx = event.x() - self._drag[0]
        dy = event.y() - self._drag[1]
        self._drag = (event.x(), event.y())
        self.azimuth += dx * 0.008
        self.elevation = max(math.radians(-80), min(math.radians(80),
                                                    self.elevation + dy * 0.006))
        self.update()

    def mouseReleaseEvent(self, event) -> None:     # noqa: N802
        self._drag = None

    def wheelEvent(self, event) -> None:            # noqa: N802
        factor = 1.1 if event.angleDelta().y() > 0 else 1 / 1.1
        self.scale = max(60.0, min(900.0, self.scale * factor))
        self.update()
