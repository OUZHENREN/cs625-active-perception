# Interfaces and parameter contract

This document freezes the application-facing contract before algorithm migration. Phase 1 implements only the minimum status and sensor interfaces; view-planning interfaces are reserved for later phases.

## 1. Phase 1 messages

### `cs625_ap_interfaces/msg/ActiveLocalizationState.msg`

Carries the state-machine state, timestamp, human-readable detail and whether execution is enabled. It does not contain a strategy-specific field.

### `cs625_ap_interfaces/msg/SensorStatus.msg`

Carries status timestamp, source frame, connection state, freshness state, received count and a diagnostic detail. A `fresh=true` value is only valid when every required stream in the configured adapter has a recent valid header.

## 2. Sensor adapter parameters

All topic names are required profile parameters; no camera-brand default is allowed in code.

```yaml
input_color_topic: ""
input_depth_topic: ""
input_camera_info_topic: ""
input_points_topic: ""
output_color_topic: ""
output_depth_topic: ""
output_camera_info_topic: ""
output_points_topic: ""
status_topic: ""
freshness_timeout_sec: 1.0
drop_invalid_messages: true
```

The four normalized output topics are supplied by the profile and must match:

```text
/sensors/camera/color/image
/sensors/camera/depth/image
/sensors/camera/depth/camera_info
/sensors/camera/points
```

## 3. Execution parameters

The following parameters are reserved for bringup and later motion layers:

```text
execute:=false
require_confirmation:=true
max_velocity_scale
max_acceleration_scale
workspace_bounds
```

No Phase 1 node may issue a robot trajectory. `execute` is a safety gate, not a command to bypass MoveIt.

## 4. Later interfaces

The following are planned but intentionally not implemented in Phase 1:

- candidate view generation;
- reachability result and failure codes;
- view score terms;
- target pose and localization quality;
- plan/execute action;
- active-localization task action.

They will be added only after the official baseline and five-package build/test gate passes.

