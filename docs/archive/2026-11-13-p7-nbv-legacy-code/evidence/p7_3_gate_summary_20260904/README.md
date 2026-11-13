# P7.3 gate summary — 2026-09-04

P7.3 is accepted for the current severe-occlusion tomato-can simulation case.

The first re-observation exposed a target-centre selection defect. After the
centre selection was changed to use measured colour/depth support, translation
success became 5/5, but yaw remained correctly rejected as unobservable. The
updated pose and covariance were then fed back into the CAD/depth geometry and
Fisher-information NBV ranker. Of 240 candidates, 25 passed the original CS625
static FK, image projection, 120 mm distal-link clearance and pose-residual
checks. The best feasible candidate was `geometry_nbv_e75_v-110_a22`.

At that second view, P7.1 again passed for five synchronized RGB-D windows.
The independent post-estimation scorer measured 1.124 mm translation error,
1.0 degree rotation error and 1.008 mm ADD-S in every window. Measured yaw
standard deviation fell to 2.5 degrees, below the frozen 5-degree stop
threshold, so the post-observation NBV decision was
`P7.3_STOP_CRITERION_MET`.

This does not claim P7.4 or P7.5: the candidate was instantiated as a static
Gazebo initial state and was not reached by a MoveIt trajectory in this gate.
