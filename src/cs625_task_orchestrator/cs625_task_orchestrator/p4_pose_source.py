"""把实时 TCP 位姿转换成 RVS 命令需要的 mm/deg 字符串。

支持三种来源：
- `state`：师兄 `cs625_state_monitor/CS625State` 的 `actual_tcp_x_mm` 等字段（首选）；
- `tf`：`base_link -> tcp_pose`（师兄 `moveit_planner_client_node` 同款约定）；
- `topic`：任意外部 `PoseStamped`（m/rad）；
- `static`：调试用固定字符串。

这里只做纯数学/格式转换，不接触 ROS，便于单元测试。
"""

from __future__ import annotations

import math
from typing import Any, Sequence

from scipy.spatial.transform import Rotation

RVS_POSE_FIELDS = 6


def _values_from_xyzquat(
    x: float, y: float, z: float,
    qx: float, qy: float, qz: float, qw: float,
) -> list[float]:
    roll, pitch, yaw = quaternion_to_rpy_deg(qx, qy, qz, qw)
    values = [x * 1000.0, y * 1000.0, z * 1000.0, roll, pitch, yaw]
    _require_finite(values)
    return values


def quaternion_to_rpy_deg(qx: float, qy: float, qz: float, qw: float) -> tuple[float, float, float]:
    """四元数 -> roll/pitch/yaw（度），与 RVS 的 rx/ry/rz 顺序一致（XYZ 固定轴）。"""
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 0.0:
        raise ValueError("zero quaternion")
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def pose_to_values(pose: Any) -> list[float]:
    """geometry_msgs/Pose（m、四元数）→ [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]。"""
    position = pose.position
    orientation = pose.orientation
    return _values_from_xyzquat(
        position.x, position.y, position.z,
        orientation.x, orientation.y, orientation.z, orientation.w,
    )


def transform_to_values(tf_msg: Any) -> list[float]:
    """geometry_msgs/TransformStamped → RVS 的 mm/deg 列表。"""
    translation = tf_msg.transform.translation
    rotation = tf_msg.transform.rotation
    return _values_from_xyzquat(
        translation.x, translation.y, translation.z,
        rotation.x, rotation.y, rotation.z, rotation.w,
    )


def state_to_values(msg: Any) -> list[float]:
    """CS625State → RVS 的 mm/deg 列表（字段本身已是 mm/deg）。"""
    if not getattr(msg, "has_actual_tool_data", True):
        raise ValueError("CS625State.has_actual_tool_data is False")
    values = [
        float(msg.actual_tcp_x_mm),
        float(msg.actual_tcp_y_mm),
        float(msg.actual_tcp_z_mm),
        float(msg.actual_rot_x_deg),
        float(msg.actual_rot_y_deg),
        float(msg.actual_rot_z_deg),
    ]
    _require_finite(values)
    return values


def parse_static_values(text: str) -> list[float]:
    """解析调试用静态字符串：x,y,z,rx,ry,rz。"""
    parts = [part.strip() for part in text.split(",")]
    if len(parts) != RVS_POSE_FIELDS:
        raise ValueError(f"需要 {RVS_POSE_FIELDS} 个数，实际 {len(parts)} 个")
    try:
        values = [float(part) for part in parts]
    except ValueError as exc:
        raise ValueError(f"无法解析为数字: {text!r}") from exc
    _require_finite(values)
    return values


def format_values(values: Sequence[float]) -> str:
    """统一输出三位小数，RVS 端按逗号解析。"""
    if len(values) != RVS_POSE_FIELDS:
        raise ValueError(f"需要 {RVS_POSE_FIELDS} 个值")
    _require_finite(values)
    return ",".join(f"{value:.3f}" for value in values)


def state_to_flange_values(msg: Any) -> list[float]:
    """CS625State 的工具 TCP → 法兰位姿（RVS 的 T1/T3 输入约定）。

    师兄记录：T1/T3 是法兰中心；TCP = 法兰 + offset，offset 由
    `actual_tcp_offset_*` 给出（本机为 (-46, 0, 353) mm，旋转为零）。
    T_tcp = T_flange * T_offset
    → R_flange = R_tcp * R_offset^-1
    → t_flange = t_tcp - R_flange * t_offset
    """
    if not getattr(msg, "has_actual_tool_data", True):
        raise ValueError("CS625State.has_actual_tool_data is False")

    tcp = [float(msg.actual_tcp_x_mm) / 1000.0,
           float(msg.actual_tcp_y_mm) / 1000.0,
           float(msg.actual_tcp_z_mm) / 1000.0]
    rpy_tcp = [float(msg.actual_rot_x_deg),
               float(msg.actual_rot_y_deg),
               float(msg.actual_rot_z_deg)]
    offset = [float(msg.actual_tcp_offset_x_mm) / 1000.0,
              float(msg.actual_tcp_offset_y_mm) / 1000.0,
              float(msg.actual_tcp_offset_z_mm) / 1000.0]
    rpy_offset = [float(msg.actual_tcp_offset_rx_deg),
                  float(msg.actual_tcp_offset_ry_deg),
                  float(msg.actual_tcp_offset_rz_deg)]

    rot_tcp = Rotation.from_euler("xyz", rpy_tcp, degrees=True)
    rot_offset = Rotation.from_euler("xyz", rpy_offset, degrees=True)
    rot_flange = rot_tcp * rot_offset.inv()
    t_flange = [tcp[i] - rot_flange.apply(offset)[i] for i in range(3)]
    rpy_flange = rot_flange.as_euler("xyz", degrees=True).tolist()

    values = [t_flange[0] * 1000.0, t_flange[1] * 1000.0, t_flange[2] * 1000.0,
              rpy_flange[0], rpy_flange[1], rpy_flange[2]]
    _require_finite(values)
    return values


def _require_finite(values: Sequence[float]) -> None:
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"位姿包含非有限值: {values}")
