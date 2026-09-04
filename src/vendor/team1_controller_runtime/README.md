# Filtered Team1 controller runtime

This directory exposes only the pinned Team1 path-tracking algorithm to the
parent catkin workspace. Its compatibility messages are owned centrally by
`common_msgs_pkg`. The complete upstream
repository is retained outside `src/` at `vendor/team1_mpc_controller`, so its
sensor receivers, localization, path manager, joystick control and UDP sender
cannot be discovered as runnable packages in this workspace.

Upstream repository:
`https://github.com/Team-Stier/Morai_SIM_2026_Team1_MPC_Controller.git`

Pinned revision: `11c076b2e464697d86c76f968999cec58d0ffd69`

The headers and algorithm sources are a filtered snapshot. Tests compare them
with the pinned submodule. The source-level integration patch replaces the
three upstream ROS message types with central equivalents and adds `base_link`
to both raw-command publication sites. `ControllerVehicleState` intentionally
keeps only the header and longitudinal speed fields actually read by the pinned
tracker; it is not presented as MORAI Competition Vehicle Status. Regression
tests reconstruct this declared patch and compare every retained header and
algorithm source against the pinned submodule. No upstream UDP implementation
or executable is present under this catkin source tree.
