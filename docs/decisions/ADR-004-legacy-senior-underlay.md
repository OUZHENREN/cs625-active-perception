# ADR-004: WSL mainline with VM fallback, and the legacy senior underlay

- Status: Accepted
- Date: 2026-10-20
- Supersedes: ADR-001 for the current development/evidence environment (the Humble VM
  described there is retained as the historical Phase-0 baseline and as the real-machine
  fallback reference; it is not the environment used for current builds or thesis data).

## Decision

1. **Development and evidence mainline**: WSL2 Ubuntu 24.04 + ROS 2 Jazzy, which is the
   environment this repository, its tests and its evidence already use.
2. **VM fallback**: the senior's VMware VM (`D:\师兄\...`) and the WSL backup tar remain
   available as a known-good real-machine reference. They are not used to produce thesis
   data and are not migrated wholesale.
3. **Legacy automation stack**: the senior's ROS automation packages are frozen as a
   **separate WSL underlay** at `~/cs625_legacy_underlay_jazzy/` and are not copied into
   this repository's `src/`. Source is the 2026-04-14 backup archive of the senior
   workspace, pinned by SHA-256 and copied to `~/cs625_legacy_snapshot_20261020/`.
4. **RVS boundary**: RobotVisionSuite stays on Windows and exchanges only target poses
   over TCP. RVS is an external tool and is not part of this repository or its evidence.
5. **Motion safety**: the legacy direct-control path (`tcp_bridge` → robot port 60000 →
   `external_control.script`) is not used for automatic motion. Automatic motion goes
   through MoveIt, which keeps collision checking and planning validation.

## Reasons

- The repository baseline, Docker-free environment entry and all accepted evidence are
  Jazzy/Ubuntu 24.04; switching the mainline back to the Humble VM would invalidate them.
- The senior's ROS stack compiled unmodified in Jazzy (14/14 packages, 66 s), so the
  feared porting cost did not materialise; the remaining risk is runtime, not build.
- The 2026-04-14 backup archive is the last coherent workspace: it contains the
  compliant-placement package that the later on-disk worktree lacks, and its state
  message is newer (TCP wrench fields).
- Keeping the legacy stack in an underlay preserves provenance and prevents two
  architectures from sharing one `src/` tree.

## Consequences

- Two underlays must be sourced in order: vendor/official underlay, then the legacy
  underlay, then this repository's overlay. `scripts/source_dev_env.sh` does not yet
  include the legacy underlay; a run that needs it sources it explicitly.
- The legacy packages are unmodified senior code. They must be cited, and must not be
  published or redistributed without the author's permission.
- Any runtime fix to a legacy package happens in the legacy underlay, not by editing
  this repository's source; the fix is recorded in the snapshot provenance.
- Direct-control remains available for manual diagnosis only, with the robot in view and
  the hardware E-stop within reach.
