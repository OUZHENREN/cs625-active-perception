# ADR-002: Package boundaries and staged migration

- Status: Accepted
- Date: 2026-08-02

## Decision

Phase 0–1 creates only `cs625_ap_interfaces`, `cs625_ap_description`, `cs625_sensor_adapter`, `cs625_simulation` and `cs625_bringup`. The later perception, mapping, view, motion, orchestration and experiment packages are introduced one responsibility at a time after the Phase 1 gate.

## Reasons

The former system coupled NBV, simulated sensors, logging, motion and orchestration. A staged boundary prevents another mega-node and provides a clean official simulation baseline before algorithm migration.

## Consequences

No NBV code migration is allowed in Phase 0–1. Legacy assets are inventoried first and assigned to a destination package before any file is moved or rewritten.

