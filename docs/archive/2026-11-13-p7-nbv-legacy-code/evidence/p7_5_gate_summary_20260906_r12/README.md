# P7.5 gate summary 2026-09-06 r12

This directory freezes the first perception-conditioned P7.5 physical-contact grasp pass in the Jazzy simulation.

Accepted scope: one `severe_v5` scene with `005_tomato_soup_can`, actual post-NBV RGB-D pose estimation, MoveIt execution, Gazebo finger contact, physical lift, and simulation-time hold. No real robot connection was used, and no Gazebo attachment command was used to lift the object.

Key result: the object center was lifted by 0.140928 m, exceeding the 0.10 m threshold. The hold window covered 2.599 s in simulation time with 0.000000672 m height drift, while bilateral finger contact remained observed and no unexpected environment collision was reported.

The retained failed vertical lift attempt is intentional evidence: a pure +Z lift from the top-grasp pose timed out in IK, so the accepted lift uses the verified reverse-approach/pregrasp pose, which both retreats from the occluder and raises the object by more than 10 cm.

See `summary.json` for the exact metrics and source files.
