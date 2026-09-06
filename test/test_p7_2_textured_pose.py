"""Focused regression tests for the P7.2 RGB-D target-centre estimator."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from p7_2_estimate_textured_pose import select_hough_candidate  # noqa: E402


def test_hough_selection_prefers_measured_target_texture_over_component_centroid():
    chroma = np.zeros((180, 340), dtype=float)
    target_seed = np.zeros_like(chroma, dtype=bool)
    target_seed[68:111, 226:254] = True
    chroma[target_seed] = 25.0
    candidates = np.array(
        (
            (259.5, 88.5, 33.0),
            (277.5, 126.5, 36.3),
            (242.5, 149.5, 22.1),
        ),
        dtype=float,
    )

    selected = select_hough_candidate(
        candidates,
        chroma=chroma,
        target_seed_mask=target_seed,
        expected_radius_px=29.0,
        component_centre=np.array((278.6, 104.9)),
    )

    assert np.allclose(selected, (259.5, 88.5, 33.0))


def test_hough_selection_uses_radius_and_centroid_only_after_equal_support():
    chroma = np.zeros((80, 80), dtype=float)
    target_seed = np.zeros_like(chroma, dtype=bool)
    candidates = np.array(((20.0, 20.0, 12.0), (40.0, 40.0, 20.0)), dtype=float)

    selected = select_hough_candidate(
        candidates,
        chroma=chroma,
        target_seed_mask=target_seed,
        expected_radius_px=12.0,
        component_centre=np.array((40.0, 40.0)),
    )

    assert np.allclose(selected, (20.0, 20.0, 12.0))
