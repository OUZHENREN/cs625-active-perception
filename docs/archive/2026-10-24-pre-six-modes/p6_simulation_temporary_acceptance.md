# P6-SIM temporary acceptance — 2026-08-20

## Acceptance decision

At the user's direction, P6 is **temporarily accepted for simulation and
software readiness only**. This is not an acceptance of real-hardware motion
or a claim that R1--R4 physically passed.

## Accepted scope

| Item | Evidence | Result |
|---|---|---|
| Shared sim/real application contract | Phase 0--6 static contract check | Accepted |
| P5 planning-only integration foundation | 24 generated candidates, 16 reachable, P5 joint score selected `fibonacci_23` in Gazebo | Accepted as `scoring_proxy_only` |
| Reproducible score tooling | Unit tests for joint score and paired experiment | Accepted |
| R0 safety profile | `real_preflight` emitted `R0_PROFILE_SAFE` with `real_motion_occurred=false` | Accepted |
| R1--R3 evidence tooling | Read-only readiness monitor and on-site runbook | Accepted as software readiness |
| R4 execution boundary | Default launch did not start an executor; executor requires explicit authorization and <=0.10 scales | Accepted as software safety boundary |

## Explicit exclusions

The following remain unaccepted until real hardware becomes available:

- Elite driver/SDK connection and fresh physical joint state (R1);
- real RGB-D stream, hand-eye calibration and physical TF validation (R2);
- real collision-checked planning-only record (R3);
- E-stop/workspace review, stop test and approved low-speed trajectory outcome
  (R4).

The temporary acceptance is automatically superseded by the first R1--R4
physical acceptance run. Any real connection or execution attempt must follow
`docs/p6_on_site_runbook.md` and must produce a new work log.

## Reverification record

```text
2026-08-20  Phase 0–6 static contract checks: PASS
2026-08-20  P5 unit/paired-experiment tests: 2 passed
2026-08-20  real_preflight: R0_PROFILE_SAFE; real_motion_occurred=false
2026-08-20  real_execution default launch: no executor process started
```
