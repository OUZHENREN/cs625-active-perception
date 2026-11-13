# ADR-001: ROS distribution and VM baseline

- Status: **Superseded by ADR-004** (2026-10-20) for the current environment; retained as the
  historical Phase-0 baseline (Humble/VM notes are archive-only now).
- Date: 2026-08-02

## Decision

Use Ubuntu 22.04 LTS in the approved VM with ROS 2 Humble, MoveIt 2 Humble, Gazebo Fortress/ros_gz, C++17, Python 3, colcon and rosdep.

## Reasons

The three project baseline documents identify Humble as the common baseline for the Elite driver and the active-vision reference. Existing Jazzy work is retained only as legacy/experimental and must not be sourced into the Humble workspace.

## Consequences

The VM is a required P0 input. Until it is available, versions remain `UNVERIFIED`; no host-side build result can be presented as the ROS baseline result.

