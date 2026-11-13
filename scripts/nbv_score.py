#!/usr/bin/env python3
"""NBV 视角评分：用"能看到的面和面积"给候选观测方向打分，再从中挑可达的那个。

为什么需要它（2026-11-12 真机结论）
-------------------------------------
按"当前相机方位绕圈"或"工件坐标系的某一个角"去选视角，在工件转到 −30° 时全部失败：
· 标准观测位：0/4 组可信；
· 按方位推算的视角：1/4 组可信，且两组的位姿相差 111 mm（多解）；
· 角视：2/4 组可信。
原因不是算法不稳，而是**视角信息不足**——看不到两个侧面时，偏航方向没有任何约束。

判据（工件近似长方体）
----------------------
一个面能被看到 ⟺ 该面法向朝向相机。可见面积 = 面积 × |cos(法向, 视线)|。
窄面（厚度方向的两个侧面）权重要高：它们正是约束偏航的关键。

用法
----
    python3 scripts/nbv_score.py --measurements runtime/measurements/workpiece_coarse.json \
        --group g00_m --distance 0.70 --top 5
    python3 scripts/nbv_score.py --measurements ... --plan        # 顺带规划前 5 个
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys

import numpy as np

REPO = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))
import coarse_pose as cp  # noqa: E402

from scipy.spatial.transform import Rotation as R  # noqa: E402

HANDEYE = REPO / "config" / "handeye" / "tcp_to_camera.json"
TEMPLATE = pathlib.Path.home() / "cs625_rvs_archive" / "models_20261023" / "弹仓_装配导出.STL"


def upright_tool_rotation(view_dir: np.ndarray) -> np.ndarray:
    """让相机画面"正过来"的工具姿态。

    手眼关系：相机 z = 工具 +z（光轴），相机 y(画面上) = 工具 +x，相机 x(画面右) = 工具 −y。
    所以要让画面里的"上"是世界 +z，就必须让 **工具 +x 指向世界上方**；
    之前的"从当前姿态做最短弧摆动"会把这个滚转原样带过去，
    结果相机沿法兰 z 转了约 90°，画面是横的（所有者 2026-11-12 指出）。
    """
    z_axis = -np.asarray(view_dir, dtype=float)          # 光轴 = 从相机指向工件
    z_axis /= np.linalg.norm(z_axis)
    # ROS 相机光学系：x 向右、y 向下、z 向前。画面里的"上"是 −y_cam，
    # 而 y_cam = 工具 +x，所以要让画面正过来，必须让【工具 +x 指向世界下方】。
    # （之前取成"指向上方"，画面就整幅转了 180°——所有者 2026-11-12 指出。）
    world_up = np.array([0.0, 0.0, 1.0])
    x_axis = -(world_up - np.dot(world_up, z_axis) * z_axis)
    if np.linalg.norm(x_axis) < 1e-6:                     # 几乎垂直向下看
        x_axis = np.array([-1.0, 0.0, 0.0])
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    return np.column_stack([x_axis, y_axis, z_axis])


def read_joint(name: str):
    """读一个关节角的当前值（弧度）。"""
    import subprocess
    completed = subprocess.run(["ros2", "topic", "echo", "/joint_states", "--once"],
                               capture_output=True, text=True, timeout=20)
    text = completed.stdout or ""
    inside = False
    names = []
    positions = []
    section = None
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith("name:"):
            section = "n"
            names.append(stripped.split(":", 1)[1].strip().strip("[]',\""))
        elif stripped.startswith("position:"):
            section = "p"
            positions.append(stripped.split(":", 1)[1].strip().strip("[]"))
        elif section == "n" and stripped.startswith("- ") or (
                section == "n" and stripped.startswith("'")):
            names.append(stripped.strip("- ").strip("',\""))
        elif section == "p" and stripped and stripped[0].isdigit() or (
                section == "p" and stripped.startswith("-")):
            positions.append(stripped.strip("',\""))
    try:
        return float(positions[names.index(name)])
    except (ValueError, IndexError):
        return None


def read_tcp():
    import subprocess
    completed = subprocess.run(
        ["ros2", "topic", "echo", "/tcp_pose_broadcaster/pose", "--once"],
        capture_output=True, text=True, timeout=20)
    text = completed.stdout
    if "position:" not in text:
        return None
    pose = np.eye(4)
    pose_quat = np.zeros(4)   # 局部变量（原来是模块级全局，多线程调用会互相污染）
    section = None
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith("position:"):
            section = "p"
        elif stripped.startswith("orientation:"):
            section = "q"
        elif section == "p" and stripped[:2] in ("x:", "y:", "z:"):
            pose["xyz".index(stripped[0]), 3] = float(stripped.split(":")[1])
        elif section == "q" and stripped[:2] in ("x:", "y:", "z:", "w:"):
            index = "xyzw".index(stripped[0])
            if index < 3:
                pose_quat[index] = float(stripped.split(":")[1])
            else:
                pose_quat[3] = float(stripped.split(":")[1])
    pose[:3, :3] = R.from_quat(pose_quat).as_matrix()
    return pose


def box_faces(bbox_min: np.ndarray, bbox_max: np.ndarray):
    """返回 (法向, 面积) 列表：长方形工件的六个面。"""
    size = bbox_max - bbox_min
    faces = []
    for axis in range(3):
        others = [i for i in range(3) if i != axis]
        area = float(size[others[0]] * size[others[1]])
        for sign in (+1.0, -1.0):
            normal = np.zeros(3)
            normal[axis] = sign
            faces.append((normal, area, axis))
    return faces


def score_direction(direction: np.ndarray, T_base_obj: np.ndarray,
                    faces, narrow_area: float, narrow_weight: float = 2.5) -> float:
    """相机在 direction 方向（从工件指向相机）时，各可见面的加权可见面积。

    窄面权重要高：偏航只能靠"两个侧面"约束，大面再清楚也定不住偏航。
    """
    rotation = T_base_obj[:3, :3]
    score = 0.0
    for normal_obj, area, _axis in faces:
        normal_base = rotation @ normal_obj
        cosine = float(np.dot(normal_base, direction))
        if cosine <= 0.05:            # 背对相机或过于掠射
            continue
        weight = narrow_weight if area <= narrow_area else 1.0
        score += weight * area * cosine
    return score


def handle_face_visible(direction: np.ndarray, T_base_obj: np.ndarray,
                        bbox_min: np.ndarray, bbox_max: np.ndarray,
                        min_cosine: float = 0.30) -> bool:
    """手柄所在的 −y 面是否朝着相机（偏航判据全靠手柄，看不到它就没有可信解）。"""
    normal_obj = np.zeros(3)
    normal_obj[1] = -1.0                     # CAD 里手柄在 y 最小的一侧
    normal_base = T_base_obj[:3, :3] @ normal_obj
    return float(np.dot(normal_base, direction)) >= min_cosine


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--measurements", required=True)
    parser.add_argument("--group", default="g00_m")
    parser.add_argument("--distance", type=float, default=0.70)
    parser.add_argument("--top", type=int, default=5)
    parser.add_argument("--plan", action="store_true", help="顺带规划前几名，看可达性")
    args = parser.parse_args()

    payload = json.loads(pathlib.Path(args.measurements).read_text(encoding="utf-8"))
    rows = payload.get("per_group") if isinstance(payload, dict) else payload
    row = [item for item in rows if item.get("group") == args.group][0]
    base = np.eye(4)
    base[:3, :3] = R.from_quat(row["pose_base"]["quaternion"]).as_matrix()
    base[:3, 3] = row["pose_base"]["translation"]

    bbox = cp.load_template_bbox(TEMPLATE)
    size = bbox[1] - bbox[0]
    # "窄面"= 面积明显小于大面的那些面（厚度方向的两侧），它们是偏航的唯一约束
    narrow_area = float(min(size[1] * size[2], size[0] * size[1]))
    narrow_area = max(narrow_area, 1e-6)
    centre = base[:3, :3] @ ((bbox[0] + bbox[1]) / 2) + base[:3, 3]
    faces = box_faces(bbox[0], bbox[1])
    print(f"工件尺寸 {np.round(size, 3)} m，窄面面积阈值 {narrow_area:.3f} m²，"
          f"中心 {np.round(centre, 3)}")

    # 相机/电机线缆限制（所有者 2026-11-12）：J6 相对当前角度不允许超过 ±45°；
    # 观测方向同理只允许在当前视线 ±45° 内微调（这个范围已足够看到两个侧面）。
    handeye0 = json.loads(HANDEYE.read_text())
    T_tcp_cam0 = np.array(handeye0["T_tcp_cam"])
    tcp_now = read_tcp()
    if tcp_now is None:
        print("读不到 TCP", file=sys.stderr)
        return 2
    T_base_cam_initial = tcp_now @ T_tcp_cam0
    j6_now = read_joint("wrist_3_joint")
    print(f"当前 J6 = {None if j6_now is None else round(np.degrees(j6_now), 1)}°，"
          f"候选限制在视线 ±45°")
    horizon_cam = T_base_cam_initial[:3, 3]
    centre_dir = centre - horizon_cam
    # direction 是"工件→相机"，所以方位角要取相机相对工件的方向（即 centre_dir 的反向）
    azimuth0 = float(np.degrees(np.arctan2(-centre_dir[1], -centre_dir[0])))
    candidates = []
    for offset in (-45, -30, -15, 0, 15, 30, 45):
      for elevation in (15.0, 30.0, 45.0):
        for azimuth in (azimuth0 + offset,):
            az, el = np.radians(azimuth), np.radians(elevation)
            direction = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az),
                                  np.sin(el)])
            if not handle_face_visible(direction, base, bbox[0], bbox[1]):
                continue                     # 看不到手柄面的方向直接排除
            score = score_direction(direction, base, faces, narrow_area)
            candidates.append((score, azimuth, elevation, direction))
    candidates.sort(key=lambda item: -item[0])
    print("候选（按可见性加权得分）：")
    for score, azimuth, elevation, _direction in candidates[: args.top]:
        print(f"  方位 {azimuth:5.0f}°  俯角 {elevation:4.0f}°  得分 {score:.4f}")

    if args.plan:
        handeye = json.loads(HANDEYE.read_text())
        T_tcp_cam = np.array(handeye["T_tcp_cam"])
        horizon = read_tcp()
        if horizon is None:
            print("读不到 TCP，跳过规划", file=sys.stderr)
            return 2
        T_base_cam = horizon @ T_tcp_cam
        import subprocess
        tried = 0
        for score, azimuth, elevation, direction in candidates:
            if tried >= args.top * 2:
                break
            tried += 1
            camera = centre + direction * args.distance
            axis = (centre - camera) / np.linalg.norm(centre - camera)
            rotation = upright_tool_rotation(-axis)
            tcp = camera - rotation @ T_tcp_cam[:3, 3]
            quat = R.from_matrix(rotation).as_quat()
            command = [sys.executable, str(REPO / "scripts" / "plan_pose_once.py"),
                       *[f"{v:.4f}" for v in tcp], *[f"{v:.5f}" for v in quat],
                       "--output", "/tmp/nbv_candidate.json"]
            completed = subprocess.run(command, capture_output=True, text=True, timeout=180)
            out = completed.stdout or ""
            if "code 1 " in out:
                verdict = "可达"
            elif "code 2 " in out:
                verdict = "超出动程限制"
            else:
                verdict = "不可达"
            reachable = verdict == "可达"
            print(f"  方位 {azimuth:5.0f}° 俯角 {elevation:4.0f}° → {verdict}")
            if reachable:
                print(f"选定：方位 {azimuth:.0f}° 俯角 {elevation:.0f}°，"
                      f"TCP {np.round(tcp, 3)}")
                break
    return 0



if __name__ == "__main__":
    raise SystemExit(main())
