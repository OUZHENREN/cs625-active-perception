#!/usr/bin/env python3
"""用棋盘格标定板验证手眼：单帧测板位姿 + 两姿态一致性检查。

为什么需要标定板
----------------
`verify_handeye.py` 只能证明"链路拓扑对"（另一条链会偏 350 mm），但证明不了
"差几毫米"——因为工件今天被手转过，它的 base 位置已经不是真值。
标定板的格子尺寸是已知的，用它可以把"相机看到的板位姿"和"机械臂 TF 说板在哪"
放在一起比，直接给出残差。

两种用法
--------
1) 单帧：板放在视野里，测一次板位姿（同时给出重投影 RMS，检验内参与角点检测）
       python3 scripts/verify_handeye_board.py --record A --squares 12x9 --square-mm 25

2) 两姿态一致性（推荐，不需要知道板的绝对位置）：
       --record A 拍一次 → 机械臂小幅移动（板仍在视野）→ --record B 再拍一次
       python3 scripts/verify_handeye_board.py --consistency runtime/board_A.npz runtime/board_B.npz
   原理：板在 base 系里不动，所以 T_base_cam(A)·T_cam_board(A) 必须等于
         T_base_cam(B)·T_cam_board(B)。两者之差就是手眼 + TF 的综合残差。

参数约定
--------
``--squares 12x9`` 指**方格数**（不是内角点数）；OpenCV 找的是内角点，所以自动减一：
12×9 格 → 11×8 个内角点。常见规格 Q12-340-25：12×9 格、格距 25 mm、板外形约 340 mm。

**板坐标系的定义**：原点取"第一个内角点（左上）"，x 沿行方向、y 沿列方向、z 垂直板面。
这一点必须与外部真值用同一个约定，否则会出现"正好一个格距对角线（√2×25 ≈ 35 mm）"的假误差
——2026-11-10 的离线自测就踩过这个坑。
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys
import time

import numpy as np

REPO = pathlib.Path(__file__).resolve().parents[1]
HANDEYE = REPO / "config" / "handeye" / "tcp_to_camera.json"
RUNTIME = REPO / "runtime"
K = np.array([[931.4730834960938, 0.0, 648.6952514648438],
              [0.0, 931.5953369140625, 449.3193359375],
              [0.0, 0.0, 1.0]])
D = np.zeros(5)


def board_object_points(cols: int, rows: int, square_m: float) -> np.ndarray:
    points = np.zeros((cols * rows, 3), np.float32)
    grid = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    points[:, :2] = grid * square_m
    return points


def detect_board(image, squares: str, square_m: float):
    import cv2
    try:
        cols, rows = (int(v) for v in squares.lower().split("x"))
    except ValueError:
        raise SystemExit(f"--squares 格式应为 12x9，收到 {squares!r}")
    inner = (cols - 1, rows - 1)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(
        gray, inner, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not found:
        return None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001)
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    object_points = board_object_points(inner[0], inner[1], square_m)
    ok, rvec, tvec = cv2.solvePnP(object_points, corners, K, D,
                                  flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    projected, _ = cv2.projectPoints(object_points, rvec, tvec, K, D)
    residual = float(np.sqrt(np.mean(np.sum((projected - corners) ** 2, axis=2))))
    return {"rvec": rvec.ravel(), "tvec": tvec.ravel(), "reproj_rms_px": residual,
            "inner": inner, "square_m": square_m, "corners_px": len(corners)}


def board_pose_in_camera(board: dict) -> np.ndarray:
    import cv2
    rotation, _ = cv2.Rodrigues(board["rvec"])
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = board["tvec"]
    return matrix


def tcp_matrix(node_messages: dict) -> np.ndarray:
    from scipy.spatial.transform import Rotation as R
    pose = node_messages["tcp"].pose
    matrix = np.eye(4)
    matrix[:3, :3] = R.from_quat([pose.orientation.x, pose.orientation.y,
                                  pose.orientation.z, pose.orientation.w]).as_matrix()
    matrix[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return matrix


def grab(args) -> tuple:
    import cv2
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from sensor_msgs.msg import Image

    rclpy.init()
    node = rclpy.create_node("verify_board")
    packets: dict = {}
    node.create_subscription(PoseStamped, "/tcp_pose_broadcaster/pose",
                             lambda m: packets.__setitem__("tcp", m), 10)
    node.create_subscription(Image, "/camera/color/image_raw",
                             lambda m: packets.__setitem__("img", m), 10)
    deadline = time.time() + args.timeout
    while time.time() < deadline and len(packets) < 2:
        rclpy.spin_once(node, timeout_sec=0.2)
    if len(packets) < 2:
        raise SystemExit("收不到 TCP 或图像：驱动栈在吗？")
    image = np.frombuffer(bytes(packets["img"].data), np.uint8).reshape(
        packets["img"].height, packets["img"].width, 3).copy()
    return image, tcp_matrix(packets)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--squares", default="12x9", help="方格数（不是内角点数），如 12x9")
    parser.add_argument("--square-mm", type=float, default=25.0)
    parser.add_argument("--record", default=None, help="记录标签，例如 A / B")
    parser.add_argument("--consistency", nargs=2, default=None, metavar=("A", "B"),
                        help="比较两个已记录姿态的一致性")
    parser.add_argument("--timeout", type=float, default=12.0)
    args = parser.parse_args()

    if args.consistency:
        first = dict(np.load(args.consistency[0]))
        second = dict(np.load(args.consistency[1]))
        base_board_a = first["T_base_cam"] @ first["T_cam_board"]
        base_board_b = second["T_base_cam"] @ second["T_cam_board"]
        delta = np.linalg.inv(base_board_a) @ base_board_b
        translation_mm = float(np.linalg.norm(delta[:3, 3]) * 1000.0)
        angle_deg = float(np.degrees(np.linalg.norm(
            __import__("scipy").spatial.transform.Rotation.from_matrix(
                delta[:3, :3]).as_rotvec())))
        print("两姿态一致性（板在 base 系应不动）：")
        print(f"  平移残差 {translation_mm:.2f} mm   旋转残差 {angle_deg:.3f}°")
        print("  参考：<5 mm 说明链路可用；>20 mm 说明手眼/口径有问题")
        return 0

    image, T_base_tcp = grab(args)
    result = detect_board(image, args.squares, args.square_mm / 1000.0)
    if result is None:
        print(f"没检出 {args.squares} 棋盘格。请确认：板在视野内、格子数/格距填对、"
              f"画面不过曝。可试 --squares 11x8 或 13x10。", file=sys.stderr)
        return 2
    import cv2
    annotated = image.copy()
    cv2.drawChessboardCorners(annotated, result["inner"],
                              cv2.cornerSubPix(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY),
                                               cv2.findChessboardCorners(
                                                   cv2.cvtColor(image, cv2.COLOR_BGR2GRAY),
                                                   result["inner"])[1], (11, 11), (-1, -1),
                                               (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                                                40, 0.001)),
                              True)
    runtime = RUNTIME
    runtime.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(runtime / f"board_{args.record or 'view'}.png"), annotated)

    T_cam_board = board_pose_in_camera(result)
    handeye = json.loads(HANDEYE.read_text())
    T_base_cam = T_base_tcp @ np.array(handeye["T_tcp_cam"])
    T_base_board = T_base_cam @ T_cam_board
    print(f"棋盘格：{result['inner'][0]}x{result['inner'][1]} 个内角点，格距 "
          f"{result['square_m']*1000:.1f} mm，重投影 RMS {result['reproj_rms_px']:.2f} px")
    print(f"板在相机系：t = {np.round(T_cam_board[:3, 3], 3)} m")
    print(f"板在 base 系：t = {np.round(T_base_board[:3, 3], 3)} m")
    if result["reproj_rms_px"] > 1.0:
        print("  ⚠ 重投影 RMS > 1 px：内参或板参数可能不对，先别用它算精度")
    if args.record:
        path = runtime / f"board_{args.record}.npz"
        np.savez(path, T_base_cam=T_base_cam, T_cam_board=T_cam_board,
                 T_base_board=T_base_board)
        print(f"已记录 {path}（再拍一次后用 --consistency {path} <另一个> 比较）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
