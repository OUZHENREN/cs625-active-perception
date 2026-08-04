# cs625_bringup

Launch composition only. This package contains one common launch composition and
separate sim/real profile entry points:

- `config/common.yaml`: one normalized RGB-D output/freshness contract;
- `config/sim.yaml`: simulation clock profile;
- `config/real.yaml`: system-clock profile;
- `launch/sensor_adapter.launch.py`: common sensor composition reused by both profiles;
- `launch/sim_base.launch.py`: fixture or explicitly verified official simulation;
- `launch/sim_active_localization.launch.py`: Phase 1 framework entry point;
- `launch/real_base.launch.py`: explicit underlay driver + sensor-adapter composition.

The CS625 driver, simulation/MoveIt packages and real camera driver remain
underlay dependencies. `real_base.launch.py` composes the existing
`eli_cs_robot_driver/elite_control.launch.py`, requires an explicit `robot_ip`,
and keeps `activate_joint_controller:=false` by default. Source camera topics
must also be supplied explicitly; no vendor implementation is duplicated here.

Both profiles preserve `execute:=false` and `require_confirmation:=true` by
default. `real_base.launch.py` does not start a robot driver or sensor adapter
unless the corresponding explicit launch arguments are enabled.
