# Planning + temporary Team1 controller test

`path_control_test.launch` composes the unchanged planning profile with the
temporary Team1 controller adapter owned by `vehicle_control_pkg`.

```bash
roslaunch system_bringup_pkg path_control_test.launch
```

This profile visualizes the same HD map, global path, local path and ego trace
as `planning_stack.launch`. The shared RViz configuration also displays the
controller-only adapter path, Pure Pursuit lookahead point and Stanley
projection point. The remaining controller diagnostics and raw actuator output
are observable under `/control_test/team1`.

It does **not** launch MORAI sensor receivers, localization, world-model data,
Safety Supervisor command output or any UDP sender. The required canonical
odometry and lead-vehicle heartbeat must be supplied by the separately
integrated runtime. The raw Team1 command is deliberately unable to move the
simulated vehicle.

See
[`vehicle_control_pkg/docs/team1_temporary_integration.md`](../../vehicle_control_pkg/docs/team1_temporary_integration.md)
for provenance, limitations, topic checks and removal instructions.
