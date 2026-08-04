# ADR-003: One application contract for simulation and real profiles

- Status: Accepted
- Date: 2026-08-02

## Decision

Simulation and real deployments share application-facing TF, topic, timestamp, frame and parameter contracts. Only the driver, sensor adapter implementation and profile configuration change between deployments.

## Safety

Real execution defaults to `execute:=false` and `require_confirmation:=true`. Planning-only validation comes before any low-speed confirmed execution.

## Consequences

Camera-brand topics stay below `cs625_sensor_adapter`; vendor packages stay in the underlay; synthetic truth is labeled and must not be silently exposed as real perception quality.

