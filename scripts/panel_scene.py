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




def _sample_cloud(msg, limit: int = 2600) -> list[tuple[float, float, float]]:
    """Down-sample a PointCloud2 to a drawable list (skip fields, ignore NaN)."""
    try:
        import struct
        fields = {f.name: f for f in msg.fields}
        if not {"x", "y", "z"} <= set(fields):
            return []
        stride = max(1, (msg.width * msg.height) // limit)
        step = msg.point_step
        data = bytes(msg.data)
        points: list[tuple[float, float, float]] = []
        for index in range(0, msg.width * msg.height, stride):
            offset = index * step
            try:
                x = struct.unpack_from("<f", data, offset + fields["x"].offset)[0]
                y = struct.unpack_from("<f", data, offset + fields["y"].offset)[0]
                z = struct.unpack_from("<f", data, offset + fields["z"].offset)[0]
            except struct.error:
                break
            if x != x or y != y or z != z:      # NaN
                continue
            if abs(x) > 4 or abs(y) > 4 or abs(z) > 3:
                continue
            points.append((x, y, z))
            if len(points) >= limit:
                break
        return points
    except Exception:                            # noqa: BLE001 - display only
        return []


def _path_joint_samples(msg, limit: int = 12):
    """Sample joint positions from a DisplayTrajectory so the worker can FK them.

    Returns a list of sensor_msgs/JointState (at most `limit`), or [] when the message
    carries no trajectory.  A TCP path is then computed from these samples with
    /compute_fk -- the display must never invent coordinates.
    """
    try:
        if not msg.trajectory:
            return []
        joint_trajectory = msg.trajectory[0].joint_trajectory
        points = joint_trajectory.points
        if not points:
            return []
        step = max(1, len(points) // limit)
        return [list(p.positions) for p in points[::step]][:limit]
    except Exception:                            # noqa: BLE001
        return []


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
        self._cloud_at = 0.0
        self._path_seen = None
        self._node = None

    def stop(self) -> None:
        self._stop.set()

    def _tcp_path(self, message, joints, fk, rclpy) -> list:
        """FK the planned joint samples into a TCP polyline (display only, no invention)."""
        samples = _path_joint_samples(message)
        if not samples or joints is None or not fk.service_is_ready():
            return []
        names = list(joints.name)
        line = []
        for positions in samples:
            try:
                state = type(joints)()
                state.name = names
                state.position = [float(v) for v in positions][:len(names)]
                request = type(fk.Request())()
                request.header.frame_id = "world"
                request.fk_link_names = ["my_end_effector_link"]
                request.robot_state.joint_state = state
                future = fk.call_async(request)
                deadline = time.time() + 0.4
                while not future.done() and time.time() < deadline:
                    rclpy.spin_once(self._node, timeout_sec=0.01)
                result = future.result()
                if result is None or result.error_code.val != 1:
                    continue
                pose = result.pose_stamped[0].pose.position
                line.append((pose.x, pose.y, pose.z))
            except Exception:                    # noqa: BLE001
                continue
        return line

    def run(self) -> None:                      # noqa: C901 - simple polling loop
        try:
            import rclpy
            from geometry_msgs.msg import PoseStamped
            from moveit_msgs.srv import GetPositionFK
            from moveit_msgs.msg import DisplayTrajectory, RobotState
            from sensor_msgs.msg import JointState, PointCloud2
        except Exception as exc:                # noqa: BLE001
            self.failed.emit(f"ROS 不可用：{type(exc).__name__}")
            return

        node = None
        try:
            rclpy.init(args=None)
            node = rclpy.create_node("panel_scene")
            self._node = node
            latest: dict = {}
            node.create_subscription(PoseStamped, "/tcp_pose_broadcaster/pose",
                                     lambda m: latest.__setitem__("tcp", m), 10)
            node.create_subscription(JointState, "/joint_states",
                                     lambda m: latest.__setitem__("joints", m), 10)
            node.create_subscription(PointCloud2, "/sensors/camera/points",
                                     lambda m: latest.__setitem__("cloud", m), 2)
            node.create_subscription(DisplayTrajectory,
                                     "/move_group/display_planned_path",
                                     lambda m: latest.__setitem__("path", m), 2)
            fk = node.create_client(GetPositionFK, "/compute_fk")
            order = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                     "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
            announced = False
            while not self._stop.is_set() and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=self._period)
                now_s = time.time()
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
                cloud = latest.get("cloud")
                if cloud is not None and now_s - self._cloud_at > 1.2:
                    self._cloud_at = now_s
                    payload["cloud"] = _sample_cloud(cloud)
                path = latest.get("path")
                if path is not None and path is not self._path_seen:
                    self._path_seen = path
                    payload["path"] = self._tcp_path(path, latest.get("joints"), fk, rclpy)
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
        self._tiles: list[tuple[tuple[float, float, float], tuple[float, float]]] = []
        self._markers: list[tuple[str, tuple[float, float, float], str]] = []
        self._links: list[tuple[str, tuple[float, float, float]]] = []
        self._cloud: list[tuple[float, float, float]] = []
        self._path: list[tuple[float, float, float]] = []
        self._tcp: tuple[float, float, float] | None = None
        self._status = "静态场景"
        self._load_model()
        self._worker: SceneWorker | None = None
        if probe:
            self.start_probe()

    # ------------------------------------------------------------------ data
    def _load_model(self) -> None:
        """Read the measured cell model.

        Only geometry that really exists in ``model.json`` is drawn:
          * tables -- slab from z + xy_min/xy_max (no invented thickness);
          * voxels -- the collision band, drawn as flat tiles on the measured surface;
          * slot / workpiece -- POSE MARKERS only: the model stores their pose, not a size,
            so drawing a sized box would invent dimensions.
        """
        if not MODEL.exists():
            self._status = "无环境模型"
            return
        try:
            data = json.loads(MODEL.read_text())
        except (OSError, json.JSONDecodeError):
            self._status = "环境模型损坏"
            return

        for name, table in (data.get("tables") or {}).items():
            try:
                z = float(table.get("z"))
                x0, y0 = (float(v) for v in table.get("xy_min"))
                x1, y1 = (float(v) for v in table.get("xy_max"))
            except (TypeError, ValueError):
                continue
            thickness = 0.02
            center = ((x0 + x1) / 2, (y0 + y1) / 2, z - thickness / 2)
            self._boxes.append((center, (abs(x1 - x0), abs(y1 - y0), thickness), C_TABLE))

        voxels = data.get("collisions") or []
        stride = max(1, len(voxels) // 900)
        for index, voxel in enumerate(voxels):
            if index % stride:
                continue
            center = voxel.get("center")
            size = voxel.get("size")
            if not center or not size:
                continue
            self._tiles.append(((center[0], center[1], center[2] + size[2] / 2),
                                (size[0], size[1])))

        for key, label, color in (("slot", "slot", C_SLOT), ("workpiece", "part", C_PART)):
            entry = data.get(key) or {}
            xyz = entry.get("xyz") or entry.get("centre")
            if xyz:
                self._markers.append((label, tuple(float(v) for v in xyz), color))

        if self._boxes or self._markers:
            xs = [c[0] for c, _, _ in self._boxes] + [m[1][0] for m in self._markers]
            ys = [c[1] for c, _, _ in self._boxes] + [m[1][1] for m in self._markers]
            zs = [c[2] for c, _, _ in self._boxes] + [m[1][2] for m in self._markers]
            self.center = ((min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2,
                           (min(zs) + max(zs)) / 2)
            span = max(max(xs) - min(xs), max(ys) - min(ys), 0.6)
            self.scale = max(150.0, min(700.0, 620.0 / span))
        self._status = (f"表 {len(self._boxes)} · 体素 {len(self._tiles)} · "
                        f"标记 {len(self._markers)}")

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
        cloud = payload.get("cloud")
        if cloud is not None:
            self._cloud = cloud
        path = payload.get("path")
        if path is not None:
            self._path = path
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
        self._draw_cloud(painter)
        self._draw_path(painter)

        items: list[tuple[float, str, object]] = []
        for center, size, color in self._boxes:
            items.append((self._depth(center), "box", (center, size, color)))
        for center, size in self._tiles:
            items.append((self._depth(center), "tile", (center, size)))
        for label, position, color in self._markers:
            items.append((self._depth(position), "marker", (label, position, color)))
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
            elif kind == "tile":
                self._draw_tile(painter, *payload)
            elif kind == "marker":
                self._draw_marker(painter, *payload)
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

    def _draw_tile(self, painter: QtGui.QPainter, center, size) -> None:
        """碰撞带顶面：平坦瓦片，读起来像桌子表面而不是一片柱子。"""
        hx, hy = size[0] / 2, size[1] / 2
        corners = [(center[0] + sx * hx, center[1] + sy * hy, center[2])
                   for sx in (-1, 1) for sy in (-1, 1)]
        polygon = QtGui.QPolygonF([self._project(corners[i]) for i in (0, 1, 3, 2)])
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(QtGui.QColor(C_VOXEL))
        painter.drawPolygon(polygon)

    def _draw_marker(self, painter: QtGui.QPainter, label: str, position, color: str) -> None:
        """位姿标记：模型只有位姿没有尺寸，画坐标轴而不是假尺寸盒子。"""
        base = QtGui.QColor(color)
        origin = self._project(position)
        for axis in ((0.05, 0, 0), (0, 0.05, 0), (0, 0, 0.05)):
            end = (position[0] + axis[0], position[1] + axis[1], position[2] + axis[2])
            painter.setPen(QtGui.QPen(base, 2))
            painter.drawLine(origin, self._project(end))
        painter.setPen(QtGui.QPen(base, 1))
        font = painter.font()
        font.setPointSizeF(7.5)
        painter.setFont(font)
        painter.drawText(origin + QtCore.QPointF(7, -7), label)

    def _draw_cloud(self, painter: QtGui.QPainter) -> None:
        if not self._cloud:
            return
        painter.setPen(QtGui.QPen(QtGui.QColor("#33475a"), 1))
        for point in self._cloud[::2]:
            painter.drawPoint(self._project(point))

    def _draw_path(self, painter: QtGui.QPainter) -> None:
        if len(self._path) < 2:
            return
        painter.setPen(QtGui.QPen(QtGui.QColor("#c8862f"), 1.6))
        previous = self._project(self._path[0])
        for point in self._path[1:]:
            current = self._project(point)
            painter.drawLine(previous, current)
            previous = current

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
