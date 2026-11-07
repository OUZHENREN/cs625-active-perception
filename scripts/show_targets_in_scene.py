#!/usr/bin/env python3
"""把已知位姿的工件与插槽以可视化标记画进 RViz（不改动碰撞体）。

背景（所有者 2026-11-07 的提问）：
  "已知工件和插槽位姿，不是可以直接把它们的三维模型匹配到场景中吗？"

可以，而且正是分工应该这样：
  · 插槽/夹具：`scripts/apply_real_cell_scene.py` 把 fixture 的 **mesh** 作为碰撞体发布；
  · 环境桌面：`scripts/apply_environment_collision.py` 把 model.json 的体素盒子发布；
  · 工件：**故意不作为碰撞体**（抓取需要那块空间自由），所以它只适合用可视化标记显示。

本脚本只发 Marker（`/visualization_marker_array`），不往规划场景里塞任何碰撞体，
因此不会影响 MoveIt 的规划结果，也不会出现"自己挡自己"的问题。

尺寸来源：config/cs625_real_cell_scene.yaml 里两个模型 STL 的顶点极值（bbox），
不是估算值。位姿来源：工件用运行时实测 S3（有则用），插槽用 RVS 标定的 slot_origin。

用法：
    python3 scripts/show_targets_in_scene.py            # 持续刷新，Ctrl+C 结束
    python3 scripts/show_targets_in_scene.py --once     # 发一次就退出
    python3 scripts/show_targets_in_scene.py --clear    # 清除标记
"""

from __future__ import annotations

import argparse
import pathlib
import sys

import yaml

sys.stdout.reconfigure(line_buffering=True)

REPO = pathlib.Path(__file__).resolve().parents[2]
SCENE = REPO / "config" / "cs625_real_cell_scene.yaml"
COMMON = REPO / "config" / "tasks" / "common.yaml"


def load_scene() -> dict:
    return yaml.safe_load(SCENE.read_text())


def bbox_size(box: dict) -> tuple[float, float, float]:
    low = box["bbox_min_m"]
    high = box["bbox_max_m"]
    return (abs(high[0] - low[0]), abs(high[1] - low[1]), abs(high[2] - low[2]))


def bbox_centre_local(box: dict) -> tuple[float, float, float]:
    low = box["bbox_min_m"]
    high = box["bbox_max_m"]
    return ((low[0] + high[0]) / 2, (low[1] + high[1]) / 2, (low[2] + high[2]) / 2)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--clear", action="store_true")
    parser.add_argument("--rate", type=float, default=2.0)
    args = parser.parse_args()

    import rclpy
    from geometry_msgs.msg import Point, Pose, Quaternion
    from visualization_msgs.msg import Marker, MarkerArray

    scene = load_scene()
    # 配置是 ROS 参数文件格式：cs625_real_cell_scene.ros__parameters.{fixture,module}
    root = scene.get("cs625_real_cell_scene", scene)
    models = root.get("ros__parameters", root)
    fixture = models["fixture"]
    module = models["module"]

    common = yaml.safe_load(COMMON.read_text())
    slot_xyz = common["slot_origin"]
    slot_quat = common["slot_origin_q"]

    rclpy.init()
    node = rclpy.create_node("show_targets_in_scene")
    publisher = node.create_publisher(MarkerArray, "/visualization_marker_array", 2)

    def make_marker(name: str, pose_xyz, pose_quat, size, colour, marker_id: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = "world"
        marker.ns = name
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.DELETE if args.clear else Marker.ADD
        marker.pose = Pose(
            position=Point(x=float(pose_xyz[0]), y=float(pose_xyz[1]), z=float(pose_xyz[2])),
            orientation=Quaternion(x=float(pose_quat[0]), y=float(pose_quat[1]),
                                   z=float(pose_quat[2]), w=float(pose_quat[3])))
        marker.scale.x, marker.scale.y, marker.scale.z = (float(v) for v in size)
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = colour
        return marker

    array = MarkerArray()
    # 工件：用实测 CAD 位姿（config 里 module 的 pose_world 是 RPY，四元数在 measured_quat_xyzw）
    module_quat = module.get("measured_quat_xyzw") or [0.0, 0.0, 0.0, 1.0]
    module_pose = module.get("bbox_centre_world_m") or module.get("pose_world")[:3]
    array.markers.append(make_marker(
        "workpiece_marker", module_pose, module_quat, bbox_size(module),
        (0.85, 0.65, 0.15, 0.55), 1))
    # 插槽：位姿用 RVS 标定的 slot_origin（config/tasks/common.yaml）
    array.markers.append(make_marker(
        "slot_marker", slot_xyz, slot_quat, bbox_size(fixture),
        (0.18, 0.85, 0.80, 0.25), 2))

    node.get_logger().info(
        f"工件 bbox={tuple(round(v, 3) for v in bbox_size(module))} m @ {[round(v,3) for v in module_pose]}")
    node.get_logger().info(
        f"插槽 bbox={tuple(round(v, 3) for v in bbox_size(fixture))} m @ {[round(v,3) for v in slot_xyz]}")

    if args.clear:
        publisher.publish(array)
        node.get_logger().info("已发送 DELETE（清除标记）")
    elif args.once:
        publisher.publish(array)
        node.get_logger().info("已发送一次标记")
    else:
        node.get_logger().info(f"每 {args.rate:.1f} s 刷新一次；Ctrl+C 结束")
        rate = node.create_rate(args.rate)
        while rclpy.ok():
            array.markers[0].header.stamp = node.get_clock().now().to_msg()
            array.markers[1].header.stamp = node.get_clock().now().to_msg()
            publisher.publish(array)
            rclpy.spin_once(node, timeout_sec=0.1)
            rate.sleep()

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
