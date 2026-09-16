# Work logs

Repository work logs.  This directory is the tracked source of record for the
project's working history.

## Naming convention

The date always comes first so the directory sorts chronologically:

```text
YYYY-MM-DD_<TYPE>_<TOPIC>.md
```

For example:

```text
2026-09-16_WORKLOG_GAZEBO_VISUALS_AND_SIM_REAL_BASES.md
2026-07-21_HANDOFF.md
```

`<TYPE>` is one of `WORKLOG`, `HANDOFF`, `PLAN`, `SPEC` or `REVIEW`; `<TOPIC>`
is a short uppercase identifier.  One file per working day, or one file per
topic when a day has several unrelated threads.

## Required content

Every entry states:

- the changed files;
- the exact commands that were run;
- pass/fail results;
- remaining limitations;
- whether fake hardware, Gazebo or real hardware was used, and whether real
  motion occurred.

When the change touches the simulation/real chain, also state the capability
layer and the critical-chain status (`PASS`, `BLOCKED`, `NOT ACCEPTED` or
`NOT STARTED`) with the concrete evidence, as described in `AGENTS.md`.

## Notes

- The two `WORKLOG_2026-08-02_*` entries are historical bootstrap logs kept
  under their original names; new entries follow the convention above.
- Day-to-day notes may additionally be mirrored into a personal Obsidian vault,
  but this directory is what ships with the repository.
