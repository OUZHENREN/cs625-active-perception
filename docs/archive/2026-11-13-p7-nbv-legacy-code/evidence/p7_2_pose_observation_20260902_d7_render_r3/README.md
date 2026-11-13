# P7.2 640x480 close-observation input

This immutable capture is a P7.2 input fixture, not a pose-estimation pass.

- Existing Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic stack; no environment rebuild.
- CS625 URDF eye-in-hand camera at the floor-safe observation configuration.
- 640x480 is the explicitly tested 2x2-binned equivalent of the PS800-E1
  1280x960 depth mode. The native-resolution d6 attempt is retained separately
  as an insufficient-synchronized-windows failure.
- Three synchronized windows were discarded before capture so the new TF2
  listener contained the exact sensor timestamps. No latest-TF fallback was used.
- `gate.json`: 5/5 windows pass, 4535 target points per window, exact-time TF,
  and no trajectory commands.

Ground truth in this directory is referenced only by the visibility audit in
`gate.json`; it is not an estimator input. P7.3, P7.4 and P7.5 were not run.
