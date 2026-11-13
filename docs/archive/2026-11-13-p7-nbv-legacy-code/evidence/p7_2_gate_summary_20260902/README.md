# P7.2 gate summary — 2026-09-02

P7.2 has two distinct outcomes.  Its **full single-view severe-pose result is
FAIL**, while its **safe handoff interface is PASS**.  The frozen estimator
passed all five windows in the unoccluded reference, light-occlusion, and
medium-occlusion strata.  The severe stratum had a reproducible 26.0 percent
visible-point ratio, but its single-view top texture was not sufficiently
observable; the estimator rejected the result instead of substituting Gazebo
ground truth or a fixed yaw.  That rejection is the explicit trigger for P7.3
re-observation, never an authorization to plan a grasp.

The accepted working thresholds are 20 mm translation, 5 degrees rotation,
20 mm ADD-S, at least 200 template pixels, template score at least 0.05, and
score margin at least 0.003.  `summary.json` links every numerical claim to an
immutable evaluation record.

Severe scene versions v1 through v5 are retained.  They document the scene
calibration boundary: v1 was extra-severe (8.2 percent visible), v2 was 25.8
percent but yaw-unobservable, v3 was 26.6 percent but yaw-unobservable, v4 was
48.2 percent and therefore medium rather than severe, and v5 was 26.0 percent
but still yaw-unobservable.  None was relabelled after the fact.
