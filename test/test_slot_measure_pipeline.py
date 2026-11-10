"""插槽（夹具）也能用同一套测量管线——用合成点云做离线回归。

为什么要有这个测试
------------------
效率模式需要"插槽粗观测"。夹具没有扶手那种非对称小特征，但它的外形
（200×745×508 mm）本身强非对称，所以用"整体覆盖率"选解就够了。
这里用 CAD 采样 + 已知位姿 + 2 mm 噪声合成一组观测点，验证：
  1. 多起点求解能从这个位姿附近收敛回去；
  2. 误差在 5 mm / 1° 以内（远小于插入所需的间隙）。
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np
import pytest

REPO = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))

o3d = pytest.importorskip("open3d")
from scipy.spatial.transform import Rotation as R

import coarse_pose as cp


SLOT_MESH = REPO / "src/cs625_simulation/assets/cs625_real_cell/slot_fixture/meshes/slot_fixture_collision.stl"


def test_slot_pose_recovers_from_synthetic_cloud() -> None:
    mesh = o3d.io.read_triangle_mesh(str(SLOT_MESH))
    assert len(mesh.triangles) > 1000, "插槽网格缺失或为空"
    cad = cp.sample_mesh(mesh, 40000, seed=7)
    bbox = cp.load_template_bbox(SLOT_MESH)
    centre = (bbox[0] + bbox[1]) / 2

    truth = np.eye(4)
    truth[:3, :3] = R.from_euler("x", 90, degrees=True).as_matrix()
    truth[:3, 3] = np.array([0.0, 0.0, 0.8]) - truth[:3, :3] @ centre

    points = (truth[:3, :3] @ cad.T).T + truth[:3, 3]
    visible = ((points[:, 2] > 0.3) & (points[:, 2] < 1.4)
               & (np.abs(points[:, 0] / points[:, 2]) < 0.75)
               & (np.abs(points[:, 1] / points[:, 2]) < 0.75))
    assert visible.sum() > 10000
    rng = np.random.default_rng(0)
    observed = points[visible] + rng.normal(0, 0.002, (int(visible.sum()), 3))

    inits = []
    for _ in range(3):
        perturb = np.eye(4)
        perturb[:3, :3] = R.from_rotvec(rng.normal(0, 0.05, 3)).as_matrix()
        perturb[:3, 3] = rng.normal(0, 0.02, 3)
        inits.append(truth @ perturb)

    solved = cp.solve_pose_multistart(observed, cad, inits, bbox[0], bbox[1], use_feature=False)
    matrix = np.array(solved["matrix"])
    translation_error = float(np.linalg.norm(matrix[:3, 3] - truth[:3, 3]) * 1000.0)
    rotation_error = float(np.degrees((R.from_matrix(matrix[:3, :3]).inv()
                                       * R.from_matrix(np.array(truth[:3, :3]))).magnitude()))
    assert translation_error < 5.0, f"平移误差 {translation_error:.1f} mm"
    assert rotation_error < 1.0, f"旋转误差 {rotation_error:.2f}°"
    assert solved["overall"] > 0.9
