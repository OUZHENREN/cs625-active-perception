#!/usr/bin/env python3
"""Inspect one frozen P7.1 observation without reading Gazebo ground truth."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def read_ppm(path: Path) -> np.ndarray:
    with path.open("rb") as stream:
        if stream.readline().strip() != b"P6":
            raise ValueError("expected binary P6 PPM")
        width, height = map(int, stream.readline().split())
        if int(stream.readline()) != 255:
            raise ValueError("expected 8-bit PPM")
        return np.frombuffer(stream.read(), dtype=np.uint8).reshape(height, width, 3)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("window", type=Path)
    args = parser.parse_args()
    rgb = read_ppm(args.window / "color_preview.ppm")
    depth = np.load(args.window / "depth_raw.npy", allow_pickle=False)
    points = np.load(args.window / "points_xyz_m.npy", allow_pickle=False).reshape(
        rgb.shape[0], rgb.shape[1], 3
    )
    cy, cx = rgb.shape[0] // 2, rgb.shape[1] // 2
    report = {"schema": "p7_2_input_inspection_v1", "ground_truth_read": False}
    for radius in (5, 8, 12, 20):
        ys = slice(cy - radius, cy + radius + 1)
        xs = slice(cx - radius, cx + radius + 1)
        patch_depth = depth[ys, xs]
        patch_rgb = rgb[ys, xs]
        finite = np.isfinite(patch_depth)
        report[f"radius_{radius}_px"] = {
            "depth_percentiles_m": np.percentile(
                patch_depth[finite], [0, 5, 25, 50, 75, 95, 100]
            ).tolist(),
            "rgb_mean": patch_rgb.mean(axis=(0, 1)).tolist(),
            "rgb_std": patch_rgb.std(axis=(0, 1)).tolist(),
            "finite_points": int(np.all(np.isfinite(points[ys, xs]), axis=2).sum()),
        }
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
