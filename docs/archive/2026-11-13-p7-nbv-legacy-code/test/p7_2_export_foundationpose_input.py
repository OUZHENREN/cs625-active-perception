#!/usr/bin/env python3
"""Export frozen P7.1 RGB-D windows into a GT-free FoundationPose bundle."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.ndimage import binary_closing, binary_fill_holes, label


def read_ppm(path: Path) -> np.ndarray:
    with path.open("rb") as stream:
        if stream.readline().strip() != b"P6":
            raise ValueError("expected binary P6 PPM")
        width, height = map(int, stream.readline().split())
        if int(stream.readline()) != 255:
            raise ValueError("expected 8-bit PPM")
        return np.frombuffer(stream.read(), dtype=np.uint8).reshape(height, width, 3)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def segment(rgb: np.ndarray, depth: np.ndarray, k: list[float], crop_radius: int) -> np.ndarray:
    """Make a visible-object mask using only the registered RGB-D observation."""
    cx, cy = k[2], k[5]
    yy, xx = np.indices(depth.shape)
    crop = (xx - cx) ** 2 + (yy - cy) ** 2 <= crop_radius**2
    intensity = rgb.astype(np.float32).mean(axis=2)
    threshold = float(np.percentile(intensity[crop], 62.0))
    candidate = crop & np.isfinite(depth) & (depth >= 0.001) & (intensity >= threshold)
    components, count = label(candidate)
    best_label, best_distance = None, math.inf
    for component in range(1, count + 1):
        rows, cols = np.nonzero(components == component)
        if len(rows) < 3:
            continue
        distance = float((cols.mean() - cx) ** 2 + (rows.mean() - cy) ** 2)
        if distance < best_distance:
            best_label, best_distance = component, distance
    selected = components == best_label if best_label is not None else candidate
    selected_depth = depth[selected]
    if selected_depth.size:
        bins = np.arange(selected_depth.min(), selected_depth.max() + 0.010, 0.010)
        if bins.size >= 2:
            histogram, edges = np.histogram(selected_depth, bins=bins)
            peak = int(np.argmax(histogram))
            mode = 0.5 * (edges[peak] + edges[peak + 1])
            selected &= np.abs(depth - mode) <= 0.035
    return binary_fill_holes(binary_closing(selected, iterations=1))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--p7-1-evidence", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--mesh", required=True, type=Path)
    parser.add_argument("--object-id", default="005_tomato_soup_can")
    parser.add_argument("--crop-radius-px", type=int, default=12)
    parser.add_argument("--minimum-mask-pixels", type=int, default=12)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(f"refusing to overwrite evidence: {args.output}")
    if not args.mesh.is_file():
        raise FileNotFoundError(args.mesh)
    raw = args.p7_1_evidence / "raw"
    windows = sorted(path for path in raw.glob("window_*") if path.is_dir())
    if not windows:
        raise RuntimeError("no frozen P7.1 windows found")

    args.output.mkdir(parents=True)
    records = []
    for window in windows:
        metadata_path = window / "metadata.json"
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        rgb = read_ppm(window / "color_preview.ppm")
        depth_m = np.load(window / "depth_raw.npy", allow_pickle=False)
        if depth_m.shape != rgb.shape[:2]:
            raise ValueError(f"RGB/depth shape mismatch in {window.name}")
        k = metadata["streams"]["camera_info"]["k"]
        mask = segment(rgb, depth_m, k, args.crop_radius_px)
        mask_pixels = int(mask.sum())
        if mask_pixels < args.minimum_mask_pixels:
            raise RuntimeError(f"mask too sparse in {window.name}: {mask_pixels}")

        target = args.output / window.name
        (target / "rgb").mkdir(parents=True)
        (target / "depth").mkdir()
        (target / "masks").mkdir()
        Image.fromarray(rgb, mode="RGB").save(target / "rgb" / "000000.png")
        depth_mm = np.zeros(depth_m.shape, dtype=np.uint16)
        valid = np.isfinite(depth_m) & (depth_m > 0.0) & (depth_m < 65.535)
        depth_mm[valid] = np.rint(depth_m[valid] * 1000.0).astype(np.uint16)
        Image.fromarray(depth_mm, mode="I;16").save(target / "depth" / "000000.png")
        Image.fromarray((mask * 255).astype(np.uint8), mode="L").save(
            target / "masks" / "000000.png"
        )
        np.savetxt(target / "cam_K.txt", np.asarray(k).reshape(3, 3), fmt="%.12g")
        (target / "source_metadata.json").write_text(
            json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        quantization_error = np.abs(depth_mm[valid].astype(float) / 1000.0 - depth_m[valid])
        files = sorted(path for path in target.rglob("*") if path.is_file())
        records.append(
            {
                "window": window.name,
                "mask_pixels": mask_pixels,
                "depth_valid_pixels": int(valid.sum()),
                "depth_quantization_max_m": float(quantization_error.max(initial=0.0)),
                "source_metadata_sha256": sha256(metadata_path),
                "files": {
                    str(path.relative_to(args.output)).replace("\\", "/"): sha256(path)
                    for path in files
                },
            }
        )

    manifest = {
        "schema": "p7_2_foundationpose_input_v1",
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "gate": "P7.2_POSE_ESTIMATION",
        "object_id": args.object_id,
        "input_evidence": str(args.p7_1_evidence),
        "mesh": str(args.mesh),
        "mesh_sha256": sha256(args.mesh),
        "ground_truth_read": False,
        "coordinate_contract": {
            "depth_unit": "metre after uint16 millimetre decode",
            "camera_frame": "camera_depth_optical_frame",
            "foundationpose_output": "object_model_to_camera_optical SE(3)",
        },
        "mask_source": "registered RGB-D only; central connected component and depth mode",
        "windows": records,
    }
    manifest_path = args.output / "input_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    checksum_lines = []
    for path in sorted(item for item in args.output.rglob("*") if item.is_file()):
        if path.name == "checksums.sha256":
            continue
        checksum_lines.append(f"{sha256(path)}  {path.relative_to(args.output).as_posix()}")
    (args.output / "checksums.sha256").write_text("\n".join(checksum_lines) + "\n", encoding="utf-8")
    print(json.dumps({"windows": len(records), "output": str(args.output)}, sort_keys=True))


if __name__ == "__main__":
    main()
