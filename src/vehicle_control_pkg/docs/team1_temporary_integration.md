# Team1 controller temporary integration

## Scope and provenance

This profile pins
[`Team-Stier/Morai_SIM_2026_Team1_MPC_Controller`](https://github.com/Team-Stier/Morai_SIM_2026_Team1_MPC_Controller)
as the Git submodule `vendor/team1_mpc_controller` at commit
`11c076b2e464697d86c76f968999cec58d0ffd69`.

Despite the repository name, the pinned implementation is not MPC. Its lateral
controller blends Pure Pursuit and Stanley through a two-model IMM, and its
longitudinal controller combines curvature speed planning with PID. This name
and provenance are kept unchanged so the temporary dependency remains
auditable and removable.

## Integration boundary

The complete upstream workspace is outside the catkin `src/` tree. A filtered
runtime snapshot exposes only `morai_path_tracking/path_tracking_controller_node`
and uses compatibility types owned by `common_msgs_pkg`. In particular,
none of the upstream sensor, localization, path-manager, vehicle-status UDP or
control UDP packages can be discovered or started from this workspace.

The filtered node maps upstream `ActuatorCommand` and `ControllerStatus` fields
exactly onto `RawActuatorCommand` and `Team1ControllerStatus`. Its status input
is deliberately narrowed to `ControllerVehicleState(header, velocity_x_mps)`:
the pinned tracker reads only those fields, and this test profile must not
masquerade canonical odometry as MORAI `CompetitionVehicleStatus`. Control mode
and gear remain responsibilities of the future approved command-transport path.

The adapter performs the following fail-closed conversion:

1. Accept only a fresh `STATUS_VALID` `/planning/trajectory` in `map` whose
   `valid_until` is still in the future, whose wheel-to-boundary clearance is
   positive, and whose points are finite and ordered.
2. Accept only fresh finite `map -> base_link` `/localization/odometry`.
3. For every accepted odometry sample, publish a controller-only Path,
   Odometry copy and velocity status with the exact same source stamp.
4. Publish an empty controller Path when the trajectory becomes invalid or
   expires. The upstream controller then emits its safe-brake output.

The authoritative controller input remains `/planning/trajectory`.
`/planning/local_path` remains visualization-only and is never used for
control.

## Safety limit of this profile

The upstream controller does not consume `PlannedTrajectory.valid_until`,
`minimum_boundary_clearance_m` or per-point `target_speed_mps`; the adapter
therefore gates all three fields and configures a conservative fixed
10 km/h test target. If any trajectory point requests less than 10 km/h,
including a planned slowdown or zero-speed stop, the adapter rejects the whole
trajectory and publishes an empty synchronized controller path so the tracker
emits safe brake. This prevents the fixed-speed controller from silently
overriding a lower planner speed, but also means this profile is not suitable
for mission-level longitudinal control. Its built-in wheel corridor is relative
to the supplied path, not to the KATRI HD-map boundaries, so it cannot
independently prove that a tracked vehicle remains off every painted line.

For that reason the result is published only as
`/control_test/team1/raw_actuator_command`. No node in this profile forwards it
to `safety_supervisor_pkg`, `morai_interface_pkg`, a UDP socket or MORAI. It is
for controller geometry, synchronization and tuning observation only.

Do not add the upstream `control_sender_node` to this launch. Closed-loop MORAI
actuation must wait for an approved nominal-command contract, a functioning
Safety Supervisor and verified competition packet/unit/sign conventions.

## Build and run

```bash
git submodule update --init --recursive
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
roslaunch system_bringup_pkg path_control_test.launch
```

The planning stack still requires real producers for
`/localization/odometry` and `/world_model/lead_vehicle`. A fresh finite
`valid=false` lead-vehicle heartbeat is the supported no-lead representation.

Useful checks:

```bash
rostopic hz /control_test/team1/local_path
rostopic echo -n 1 /control_test/team1/controller_status
rostopic echo -n 1 /control_test/team1/raw_actuator_command
rosnode list | grep control_test
```

There must be no upstream `control_sender_node` in `rosnode list` and no
controller consumer on `/planning/local_path`.

## Removal

Remove the Team1 submodule entry, this adapter/config/launch/documentation,
the filtered runtime snapshot, the `path_control_test.launch` composition and
the corresponding temporary central-contract records. The normal
`planning_stack.launch` is unchanged.
