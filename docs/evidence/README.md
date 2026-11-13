# Runtime evidence

This directory contains machine-written or source-derived evidence for
simulation acceptance.  A receipt records only the interface response that
was observed during the named run; higher-level acceptance is evaluated
separately and must not infer missing perception, contact, or real-robot
evidence.

## Current evidence index

| Directory | Role | May be used as final single-attempt evidence? |
| --- | --- | --- |
| `minimum_showcase_matrix_20260906_r1` | completed 3-scene x 5-seed x 3-baseline P4 simulation matrix | **yes; matrix COMPLETE**, but all episodes ended with `MAX_FAILED_ATTEMPTS` |

`sim_video_provenance_20261025.png` documents the provenance of the recorded
simulation video.

## P7 evidence archived 2026-11-13

All `p7_*` evidence directories (the P7 five-gate simulation pipeline: sensor
gate, pose gates, geometry NBV, MoveIt NBV, attachment-assisted grasp on the
old tomato worlds) were archived together with the P7 code on 2026-11-13 per
the owner's instruction.  They now live read-only under
[`docs/archive/2026-11-13-p7-nbv-legacy-code/evidence/`](../archive/2026-11-13-p7-nbv-legacy-code/README.md),
kept only for provenance.  Do not cite them as current acceptance evidence:
the current real-cell task flows no longer run the P7 pipeline.
