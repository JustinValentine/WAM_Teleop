# Files that live beside the package

The repository root is the `wam_teleop` ROS package, as upstream. This folder
holds the files that sit around it in the Docker workspace on the robot
computer (`<workspace>` below is that checkout, e.g. `wam-ros-docker`):

| Here | Goes to | What it is |
|---|---|---|
| `wam` | `<workspace>/wam` | launcher: starts the leader, the follower and roscore in tmux panes, checks the CAN interfaces, summarises recordings (`wam teleop`, `wam status`, `wam log`) |
| `wam_settings.sh` | `<workspace>/wam_settings.sh` | the launcher's settings: both arms' options, CAN interfaces, CPUs, recording |
| `config/wam8g/` | `<workspace>/config/wam8g/` | the leader's libbarrett configuration and calibration (wam8g: WAM 153 base with the Alberta wrist, as of 2026-11-11) |
| `tools/dyn_feel/` | `<workspace>/tools/dyn_feel/` | `wam8g_dyn_feel`: float the leader and switch an identified J1-J4 model's terms on by key, to feel a fit before it goes into the teleop |
| `tools/gravtool/` | `<workspace>/tools/gravtool/` | evaluates the library's gravity model at given joint positions, without the robot |
| `tools/lbc_examples/`, `tools/lbc_tests/` | same | CMake files that build the CANopen library's own examples and hardware tests against the installed library |

The package itself goes to `<workspace>/wam_ws/src/wam_teleop/`.

Not in this repository: the Docker files and build scripts of the lab's
workspace, `libbarrett-canopen`, the gripper and haptic-wrist libraries,
recordings (`teleop_logs/`, `excitation/`) and build products.

State on 2026-11-11: the leader is the wam8g on WAM 153's base; the follower
is the slax arm with the skid gripper and F/T sensor
(`config/follower-slax-skid-ft/`, `src/lib/follower_beta_slax_skid_hand.h`).
The parameters of the other base arm, used until then, are kept in
`src/lib/iros_base/`.
`wam_settings.sh` is as the last session of that afternoon started (16:37):
4C with force gain 0.4, no integral action, `--reference age` on both arms,
and the follower's force taken from its force/torque sensor (`--ft all`: J1-J4,
and the wrist once the leader's `w` key gives it a gain; the package README's
section on the sensor).
`tools/dyn_feel/params-wam153-20261111/` holds the parameter sets identified
on WAM 153 that night, for `wam8g_dyn_feel` (the cable model from runs 2+3
and from runs 2+3+4, the 30-parameter model with and without its Coulomb
terms) and the tool's check outputs for them.
