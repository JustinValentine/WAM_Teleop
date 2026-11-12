# Note
This version of code is for horizontal configuration of the WAM teleop setup.

# Wam Teleop
This package enables teleoperation between the 4DOF leader with the haptic wrist and the 7DOF follower. While this is a ros package, communication between WAMs does not use ros, and instead uses UDP. Ros is only used to publish the state of arm for easier data collection, and is not intended to receive any incoming messages or services to control the arm. 

## Build Instructions

Place this package in `<catkin_ws>/src/`.

By default, this package builds the leader and follower node. If you only wish to build the follower node you can configure this from the command line:
```bash
catkin_make --cmake-args -DBUILD_LEADER=OFF 
```
or set the option to `OFF` in `CMakeLists.txt`:
```bash
option(BUILD_LEADER "Build leader executable" OFF)
```
To build the leader node, the haptic_wrist library is required as a dependency, build and install instructions can be found [here](https://github.com/dmiller12/libhaptic_wrist).

## Run Instructions

`config/` contains the Barrett configuration files for the leader and follower. 
You can set the correct config file by using `source scripts/setup_leader.sh` and `source scripts/setup_follower.sh`. These will set the env variable `BARRETT_CONFIG_FILE` to the correct path. 
You may need to modify the bus port in `config/leader.conf` and `config/follower.conf` depending on the can interface. Note that these environment variables only persist for the current terminal session.

Each node has the same command line options:
```bash
rosrun wam_teleop leader [remoteHost] [recPort] [sendPort]
rosrun wam_teleop follower [remoteHost] [recPort] [sendPort]
```
Use `-h` or `--help` to see options description.

### Example

Source workspace in each terminal in `amir\catkin_ws`
```bash
source devel/setup.bash
```


In first terminl, source CAN in a terminal in `amir\catkin_ws\src\wam_teleop`
```bash
source scripts\pci_can_init.sh
```
if you are doing this with the pci card or if you are using usb:
```bash
source scripts\usb_can_init.sh
```
NOTE: you might have to tweak the can# based on the order you plugged stuff into the computer.


In a separate terminal session, start the master node with: `roscore`.

In a separate terminal session, start the leader in `amir\catkin_ws\src\wam_teleop`:
```bash
source scripts/setup_leader.sh
rosrun wam_teleop leader_nowrist 127.0.0.1 5555 5554
```
In another separate terminal session, start the follower in `amir\catkin_ws\src\wam_teleop`:
```bash
source scripts/setup_follower.sh
rosrun wam_teleop follower 127.0.0.1 5554 5555
```
Note the matching recPort and sendPort between leader and follower, that is, the leader receives on 5555 and the follower sends on 5555 and vice versa.

Once both nodes have started:

1) On the leader use `l` to go to the sync position.
2) On the follower use `l` to go to the sync position. Ensure both arms have reached the sync position before continuing.
3) Press enter to link leader
4) Press enter to link follower

The arm is now ready for user teleoperation.

To turn off, it is recommended to go through the following procedure to ensure proper thread and socket cleanup.
1) Return both wams to home position
2) On the leader, press `x` to exit the loop.
3) Shift idle the leader
4) Repeat for follower. Press `x` to exit the loop
5) Shift idle the follower.

After `x` each program (leader_wam8g, follower) holds its arm and waits for it
to leave ACTIVE, Shift-Idle or E-Stop, then exits (`src/lib/clean_exit.h`). If
the WAM's safety module stops answering (the CAN link is down), it says so and
exits within a second. Ctrl-C (or SIGTERM, or the end of its input) at the
prompt does what `x` does: the teleop ends, the arm holds, the follower's hand
closes to a fist; Ctrl-C again during the wait that follows exits at once (an
arm still active is then no longer controlled). `wam stop`'s force-stop uses
the same two signals and kills only what still runs after them.


### Dynamics Excitation

To collect data for dynamics we provide canopen and legacy libbarrett executables (`dynamics_excitation_canopen` and `dynamics_excitation_legacy`) :   

```bash
# Run with default output path (rbtlog_4_dof.dat)
rosrun wam_teleop dynamics_excitation_canopen
```

Or provide the output save path:   

```bash
rosrun wam_teleop dynamics_excitation_canopen zeus_rbtlog_4_dof.dat
```

These will save data for only the first 4dof.

J1-J4 follow the trajectory's velocity plus a correction toward its position (2/s on the position error), so they do not drift from the planned path; without it Zeus's J2 drifted back 1 rad in 50 s.

During execution, it will actively monitor the joint position bounds defined in the `src/dynamics_excitation.cpp:FourierTrajectory` constructor.
Note that these bounds do not modify the planned trajectory.
If a joint exceeds this limit, the system executes an abrupt stop by commanding zero velocity.
If this occurs, modify the `factors` vector in `FourierTrajectory` to reduce the amplitude of the velocity.


Likely a good idea to change the `joint_velocity_control` gains in the config file, generally j1, j3, and j4 are cut in half and the rest are unchanged.
```conf
#kp = (  42,   42,   18,   18,    3,    3,    3); //default
kp = (  21,   42,   9,   9,    3,    3,    3);
```

On a 7-DOF WAM the program holds the wrist (J5-J7) at its start angles with the `joint_position_control` gains and zeroes the wrist's velocity gains: under velocity control Zeus's wrist, with its hand, shook at 7 and 13 Hz.

## wam8g leader with a P2 follower

**The controllers as they run now, in one place: [CONTROLLERS.md](CONTROLLERS.md)**
(the torque on each arm, the position coupling and its gains, the law, the
external-torque estimates, the force/torque sensor's path, and what lives in
the Barrett libraries and not in this repository).

`leader_wam8g` drives the eight-joint wam8g (CANopen libbarrett: J1-J4 arm,
passive J5, Alberta wrist J6-J8) as the leader for the seven-joint P2
`follower`. The message between them is unchanged (J1-J4 plus the P2's
J5-J7). Both arms sync to the haptic_saber start pose, the follower with its
wrist straight (`src/lib/wam8g_teleop.h`). The wam8g uses its own configuration,
`config/wam8g` at the top of the workspace repository; set `bus.port` there
first. J1-J4 get the Ares leader's position gains at startup; J5-J8 keep the
configured ones.

### Control laws

Both arms' position controllers track the other arm's joints. `--law` adds a
feedforward torque on J1-J4 (`src/lib/teleop_law.h`); `ext` is each arm's
external torque estimate, the commanded torque minus the model's:

| Law | Torque | |
| --- | --- | --- |
| `u1` | 0 | position-position |
| `u2` | dyn - grav | + dynamics compensation (the model's torque replaces libbarrett's gravity) |
| `u4` | -0.5 ext_other + dyn - grav | 4C: + force channels. The follower's default |
| `u6` | u4 - 0.15 (ext_other + ext_own) | 4C + local force feedback. The leader's default |

Each arm sends its `ext` only while it is linked, and `ext_other` fades in over
2 s once both arms are linked (`teleop_law::forceChannel`): an unlinked arm's
estimate pushed the linked one by about its gravity torque, and closing the 4C
loop onto that offset made both arms shake. `--force-gain g` (both programs,
0 to 1) replaces u4's 0.5; it scales u6's ext_other terms too. Four
comma-separated values set J1-J4 separately (J3 is the first to go unstable as
the gain rises), e.g. `--force-gain 0.4,0.4,0.2,0.4`. `--force-filter
hz` (both programs, default off) low-pass filters the force channel: the
closed 4C loop went unstable at 22-36 Hz, where the round trip between the
arms delays it a quarter cycle or more. `--force-notch hz` (both programs,
default off) notches it instead (Q 0.7): the loop rings at a resonance of the
arms (J3 31-36 Hz, J2 30-42 Hz, J1 37-43 Hz), and a notch at 35 Hz takes that
band out while lagging the arms' slow relative swing, near 4 Hz, 9 degrees
(a 15 Hz low-pass: 15 degrees, which made the swing larger).

For an A/B on the same arms, the mode can be switched during the teleop, on
both arms at once (`src/lib/teleop_mode.h`):

| Mode | Key (either arm's pane) | `wam` | Law |
| --- | --- | --- | --- |
| full | `c` or `p` again | `wam 4c on`, `wam pp off` | as launched (`--law u4`: our system) |
| 4C off | `c` | `wam 4c off` | u2: position-position + dynamics compensation |
| P-P | `p` | `wam pp on` | u1: plain position-position, libbarrett's gravity compensation only |

The key or command publishes the mode on `/teleop/mode`; each arm fades its
law's terms (the force channel, u6's local term, dyn - grav) to it over 1 s
(`teleop_law::ModeMix`), so it changes without a jolt, even in contact. The
position gains, the notch and J6's friction feedforward stay as they are. Both
panes print the new mode. The programs start in full.

Each arm's position reference is the other arm's last position, advanced with
its sent velocity when a cycle gets no new packet (for up to two cycles): the
two 500 Hz loops are not synchronized, and a stalled-then-doubled reference
made the position controller's D term kick in proportion to the speed.

The reference is a system of its own (`src/lib/teleop_reference.h`) that the
arm system pulls first each cycle, so the packet is read before anything else.
The external torque estimate includes the position controller's torque, so
reading the estimate runs the controller. When the reference was an output of
the arm system itself, the controller ran before that cycle's packet was read
and tracked the previous one: 2 ms late on each side.

Both programs, and the dynamics excitation, hold `/dev/cpu_dma_latency` at 0
while they run (`src/lib/cpu_latency.h`), keeping the CPUs out of their deep
idle states: with
them on, a packet took 0.2 ms (median) and up to 3.6 ms between the programs,
and one later than the phase between the two loops misses the other arm's next
cycle. Held at 0: 12 us median.

`u2`, `u4` and `u6` need the wam8g fit in `src/lib/leader_beta_wam8g.h` (see
Excitation on the wam8g); until it is there `leader_wam8g` runs only with
`--law u1` and sends the follower zero external torque.

### Excitation on the wam8g

`dynamics_excitation_canopen` (see Dynamics Excitation above) also runs on the
wam8g; run `source scripts/setup_leader_wam8g.sh` first. Without it libbarrett
reads `/etc/barrett`, the P2 WAM's configuration, and the program refuses the
wam8g. J1-J4 track the trajectory velocity as on the other arms. The wrist
(J6-J8) has no velocity-control gains in `config/wam8g`, so the program holds
it where the start move leaves it (at 0) with its position gains; J5 is
passive. `config/wam8g` has the velocity-gain advice above with J2 halved as
well, `kp = (21, 21, 9, 9, 0, 0, 0, 0)`: at 42, J2 oscillated at about 22 Hz
whenever it swung through vertical with the elbow straight. The wam8g's J4
factor is 0.7.

- The program sets the safety board's VL1/VL2 to 1.5/2.0 rad/s per joint
  (`VL_WARNING`, `VL_FAULT`) before it moves, and prints them back. The
  reference peaks at 1.45 rad/s on J1 and J3, and at the start it jumps straight
  to up to 1.4 rad/s. The board keeps these until its next power-up, when they
  return to 0.5 and 1.0 rad/s.
- At the end the program moves the arm straight up, then home, so leave room
  above the robot. It then waits for Shift-Idle.
- The output is the identification notebook's input
  (`feasible-dynamics-estimation/Feas-Ident-WAM4`): time, q1-q4, tau1-tau4
  every 4 ms, for example
  `rosrun wam_teleop dynamics_excitation_canopen wam8g_rbtlog_cpp_4dof.dat`.
  Copy it to the notebook's `data/recdata/` and change the notebook's three
  `zeus_rbtlog_cpp_4dof.dat` references to it.
- Put the 30 values of `tmp/solutions/beta_star_correct.dat` in
  `src/lib/leader_beta_wam8g.h`, set `LEADER_BETA_WAM8G_FITTED` to 1 and
  rebuild.

### Cable-drive model (`--model cable`)

`leader_wam8g` and `follower` take `--model cable`: the external-torque
estimate and the feedforward then use the cable-drive model
(`src/lib/cable_drive_model.h`, parameters `src/lib/cable_params_wam8g.h` and
`cable_params_zeus_hand.h`) instead of `leader_dynamics.h` /
`follower_dynamics.h`:

- the friction is a bank of Jenkins elements per motor (and on J2/J3), driven
  by encoder increments, so it is in the estimate without the velocity-sign
  chatter that made the old fits drop Coulomb friction;
- the inertia uses the real acceleration at full scale, the rotors and the
  J2/J3 differential included, and the wrist's gravity is corrected from
  libbarrett's calibration;
- the parameters come from `ident/` (`python -m cable_ident fit`), fitted on
  the excitation and on every session's self-driven moves to the sync pose;
  see `ident/README.md` and `ident/METHOD.md`.

The feedforward keeps the legacy terms unless asked: `--ff-friction s` adds a
share s (0-1) of the friction, `--ff-inertia s` sets the inertia's share
(leader 0, follower 0.25 as in legacy), and the key `m` changes both live.
Raise `--ff-friction` in steps: all of the friction can drive a joint. For
example

```bash
rosrun wam_teleop follower --sync wam8g --model cable 127.0.0.1 5554 5555
rosrun wam_teleop leader_wam8g --law u4 --model cable --ff-friction 0.3 127.0.0.1 5555 5554
```

`dynamics_excitation_* --trajectory <file> <log>` plays a trajectory designed
by `python -m cable_ident design` (position control, the wrist held where it
starts) and logs every joint's position, velocity and torque at 500 Hz.

### Low latency and recording (2026-11-09)

Changed in `leader_wam8g` and `follower` so that the delay between the arms is
short and the same from cycle to cycle, and so that data is collected without
ROS. None of it has run on the robots yet; the offline tests below cover it.

**The control threads.** The CANopen library starts the wam8g's 500 Hz thread
real-time (SCHED_FIFO 50, memory locked). The legacy library does not
(outside Xenomai its loop timer ignores the priority): the follower's loop ran
as an ordinary thread, late whenever the computer was busy. Each arm system now
sets up its own thread in its first cycle (`src/lib/rt_thread.h`): SCHED_FIFO
`--rt-priority` (default 50) if the library left it non-real-time, pinned to
`--rt-cpu` if given; both programs lock their memory. Each prints what its
thread was and is, e.g. `Follower control thread: SCHED_FIFO priority 50 (the
library started it as SCHED_OTHER (not real-time)); pinned to CPU 4.`

**The link.** `src/lib/teleop_link.h` replaces `udp_handler.h` in these two
programs: the control loop sends and receives itself, non-blocking, with no
helper thread and no lock (a packet used to wake a normal-priority thread to be
sent, and be handed over by another under a mutex). Packets carry a cycle
counter and the time of the sample they hold, so each arm knows how old the
other's data is and counts lost packets, and they carry the teleop mode, so
`c` and `p` reach the other arm without ROS. `follower` still works with the
older leader programs (it answers a `udp_handler.h` sender in its format). The
loop no longer prints (`lost link` comes from a watcher thread) and no longer
allocates (`RegressorDynamics` replaces `LeaderDynamics` / `FollowerDynamics`,
which built a matrix on the heap every cycle).

**Recording without ROS.** `--record <file or directory>` makes the control
loop write one row per cycle (`src/lib/teleop_recorder.h`): time, cycle
counter and period, q, dq, the commanded torque's three parts (gravity,
position controller, feedforward), the reference used, the external-torque
estimate (own, sent, received, as used), the model's feedforward, the link
(new packet, its age, sequence, lost count), state, mode and the recorder's
own drop count. The loop only copies the row into a ring; a normal thread
writes the file, and a slow disk drops rows (counted, reported) instead of
delaying the loop. `scripts/teleop_log.py` reads it with numpy only:

```bash
python3 scripts/teleop_log.py teleop_logs/teleop_<time>_leader.tlog teleop_logs/teleop_<time>_follower.tlog
python3 scripts/teleop_log.py <file> --npz out.npz     # or --csv out.csv
```

The summary shows the control period (median, 99 %, 99.9 %, worst), how old the
other arm's sample was when used, and cycles without a new packet: the numbers
to look at for "is the lag consistent". `--no-ros-state` leaves out the
`joint_states`/`gravity` publisher (a 500 Hz thread that takes the control
loop's lock three times a cycle; only the ROS recording needs it), and
`--no-ros` runs without ROS at all (no roscore; not with `--hand`). `wam`
does this from `wam_settings.sh`: `RECORD=native` (the default now), `bag`,
`both` or `no`; `wam log` prints the latest session's summary.

**Options that change the control, off by default** (so the first run after
this update uses the law as at IROS); try one at a time:

| Option | What it does |
| --- | --- |
| `--reference age` | The other arm's position is advanced with its sent velocity over each packet's age (both programs on one computer). The two loops are not synchronized, so that age is anywhere in a 2 ms window plus the sender's bus read, different every run; the position coupling feels it as a drag of kp x age per rad/s. `hold` (default): as before. |
| `--estimator matched` | The external-torque estimate with the commanded torque and the model's other terms passed through the same filter as the model's acceleration (`src/lib/estimator_core.h`). The legacy estimate subtracts a 4 Hz-filtered inertial torque from the unfiltered commanded torque, so above about 3 Hz it is the arm's own position-controller torque, which the other arm then applies. `--estimator-filter hp,lp` sets the two corners in rad/s (default 25,50, the existing acceleration filter: the estimate is then limited to about 4 Hz). The feedforward is not changed. |
| `--ki-scale s` | J1-J4's integral gain times s. With the arm at rest the integrator keeps moving the commanded torque inside the stiction band, which the estimate reports as an external torque. |

On the arms (odin, 2026-11-11): `--ki-scale 0` on both arms from 15:15 on;
`--reference age` on both arms from the session started at 16:37. Offline, on
that day's 15:15 recording, the packet's age was steady to 0.01-0.02 ms from
one cycle to the next (2.37 ms at the leader, 1.63 ms at the follower; 1.34
and 0.66 ms in the next start), the aged reference would have removed a drag
of 1.0 / 1.2 / 0.4 / 0.3 Nm on the leader's J1-J4 and 0.7 / 0.8 / 0.3 / 0.2 Nm
on the follower's at 0.5 rad/s, and added 0.02-0.05 Nm rms of torque above
30 Hz to the 0.2-0.3 Nm already there. `--estimator matched` has not run on
the arms.

`low_latency_tests` checks the link, the mode, the aged reference, the
recorder, the estimator's arithmetic and the thread setup without libbarrett;
`wam8g_loopback_tests` (and `_legacy`) also run the mode over the link, the
aged reference, a recording from both arm systems, the estimator systems
against the classes they replace, and the control thread each library starts.

### Calibrating the follower (`wam cal`, 2026-11-11)

The follower's zero and gravity calibration are Barrett's own utilities
(`bt-wam-zerocal`, `bt-wam-gravitycal`, from the follower's libbarrett). They
do not take their files from `BARRETT_CONFIG_FILE` alone: the pose lists
(`calibration.conf`) and the results (`calibration_data/<arm>/zerocal.conf`,
`gravitycal.conf`) are looked up under `~/.barrett` when that folder exists
and under `/etc/barrett` otherwise (`EtcPathRelative` in the library's
`config.h`). In the lab's container `~/.barrett` is the leader's folder, so
run plainly they would read the leader's pose lists and write the follower's
calibration into the leader's folder, where the follower never reads it.
`workspace/wam` therefore runs them with a `HOME` whose `.barrett` is a link
to the follower's configuration folder (`config/follower-slax-skid-ft/`,
which now holds the follower library's `calibration.conf`):

| Command | What runs |
| --- | --- |
| `wam cal check` | prints where the calibrations read and write; starts nothing |
| `wam cal zero` | `bt-wam-zerocal`: the arm moves to two poses (upper arm level with the forearm up; straight up); each joint is turned with the arrow keys until its mark lines up, Enter records the joint's actual angle and its motor angles; writes `zerocal.conf` (`home` and `zeroangle`) |
| `wam cal gravity` | `bt-wam-gravitycal`: nine poses all around the base, the holding torques measured in each; writes `gravitycal.conf` |
| `wam ft-cal` | the gripper beyond the force/torque sensor (below) |
| `wam cal excite NAME` | `dynamics_excitation_legacy`: 60 s on the Fourier trajectory with the wrist held at zero, then straight up and home; writes `excitation/slax_<date>_NAME.dat` |

In that order: the gravity calibration, the gripper's calibration and the
dynamics identification all depend on the joint zeros. The arm moves by
itself in every one of them; none activates it.

When this was written (2026-11-11) the slax follower had no zero calibration
(`zerocal.conf` held the home pose only), so its joint zeros were wherever
the arm sat when it was first started after power-up. The gripper's calibration of that day
put the sensor's axis 10 degrees from the tool axis about the J6 axis, which
a wrist zero of that kind would explain.

Two things about `wam cal zero` with this arm's configuration: the utility
runs without gravity compensation and the position gains are the teleop's
(no integral action), so the arm sags a little in the first pose (an
estimated 2 degrees at J2). The recorded angle is the measured one, so
joints with scribe marks (J1, J4, J5, J7) are not affected; for J6 ("the
tool-plate is horizontal") line the tool plate up square to the forearm
rather than level to the room.

### Force/torque sensor and the wrist (2026-11-09)

The wrist is coupled position to position only (the laws act on J1-J4), so
the leader's wrist feels its position controller pulling it toward the
follower's wrist, which lags by its own friction and inertia: in the IROS
recordings a median 0.1-0.25 Nm against the hand while turning, 0.4-0.6 Nm a
tenth of the time, on top of the wam8g wrist's own friction. A contact reaches
the hand only through the same position error.

With the Barrett force/torque sensor on the follower's tool plate, the contact
is measured instead (`src/lib/ft_core.h` has the arithmetic and the reasons).
Everything is off unless asked for. On the arms so far (odin, 2026-11-11, the
slax follower with its skid gripper): reading the sensor in the loop (every
cycle, at most 0.30 ms of loop time), `wam ft-zero`, the gripper's calibration
(`wam ft-cal`, two runs), a teleop session with `--ft arm` (16:08, 21 minutes
linked) and one with `--ft all` (16:37), in which the leader's wrist force
gain was raised by key from 0 to 0.2 and then 0.5. `--wrist-friction` has not
run on the arms.

In the `--ft arm` session, against the session before it with the model's
estimate as the follower's force (15:15, the same force gain 0.4): the
leader's joint velocity between 5 and 20 Hz was 2.8 / 4.2 / 6.9 / 5.8 mrad/s
rms on J1-J4 against 7.6 / 16.3 / 14.2 / 17.8, the lightly damped swings at 8
and 12 Hz were gone, and the force the leader received no longer had the
follower's friction in it (its mean along the motion about 0). Over the whole
linked session, contacts included, the sensor read 1.6 N (median) after the
gripper's weight was removed, under 2.9 N for nine tenths of the time.

The calmer session cannot be credited to the sensor alone. Between the two
the leader's kd on J1 and J2 went from 8.3 and 6 to 10 and 8, the follower's
integral gains were switched off, and the packet ages, which depend on how
the two loops happened to start, summed to 2 ms instead of 4 ms. On that day
the two sessions that swung (15:07 at force gain 0.5, 15:15 at 0.4) were the
only two with a 4 ms sum, and every session with 2 ms was calm, so the force
gain, the force's source and the delay cannot be told apart from these
recordings. The age-compensated reference (in use from 16:37) removes the
delay's part; the comparison should be repeated with it.

**Running the teleop with the sensor's force (`--ft arm`).** Both arms keep
the 4C law; what changes is where the follower's force comes from: the
external torque on J1-J4 that it sends to the leader is the sensor's wrench
(the gripper's weight removed, through the Jacobian transpose), not the
estimate from the follower's dynamics model. So the follower's own friction
and model error no longer reach the leader as force, and only what touches
the gripper does (a push on the forearm arrives through the position coupling
alone). With the launcher (`workspace/wam`, `workspace/wam_settings.sh`):

1. `wam ft-cal` once per gripper (below): it writes
   `config/follower-slax-skid-ft/ft_tool.conf`.
2. `FOLLOWER_EXTRA="--ft arm --ft-tool $FT_TOOL_IN"` in `wam_settings.sh`.
3. Both arms at home; Shift-Idle the follower before `wam teleop` (the slax
   sensor only answers once the arm is in Idle). The follower's pane must
   print `Force/torque sensor: read every cycle; J1-J4's external torque sent
   to the leader is the sensor's.` and the gripper's mass.
4. Activate and link as usual. The sensor's zero is taken at the link pose:
   keep clear of the gripper between `l` and Enter. `wam ft-zero` takes it
   again at any time (nothing touching the gripper for a second).
5. `wam pp on` drops to position-position from any terminal and `wam pp off`
   goes back; `wam 4c off` and `wam 4c on` switch the force channels alone.

As run that day: force gain 0.4 on both arms, 35 Hz notch, no integral action
(`--ki-scale 0`, and ki 0 in the follower's configuration), the leader on the
cable model with 0.2 of its friction fed forward, the follower on the legacy
model with slax's parameters. `workspace/wam_settings.sh` is the 16:37
session's: `--reference age` on both arms, `--ft all` on the follower, the
leader's wrist force at 0 until `w` raises it; in that session the leader's
friction share was also raised by key (`m`) from 0.2 to 0.5 and the wrist
force gain to 0.5. The operator's note on it: the best the teleop has felt,
with an artifact in the arm's redundant (null-space) motion that is not yet
analysed. What the gripper's calibration leaves (about
1 N, and up to 2 N more near the ends of J7's travel, see below) appears to
the leader as a contact force.

| Option | What it does |
| --- | --- |
| `follower --ft log` | Reads the sensor every cycle and records it (`--record`). The teleop is unchanged. |
| `follower --ft wrist --ft-tool F` | Also sends the wrist torques it measures (wrench minus the gripper's weight, through the Jacobian transpose) in the packet's three wrist slots, flagged. |
| `follower --ft arm --ft-tool F` | J1-J4's external torque sent to the leader is the sensor's instead of the model's estimate: the 4C force channel from the follower no longer depends on the follower's dynamics fit. It only sees what acts beyond the sensor (a contact on the gripper, not on the forearm). `--ft all`: wrist and arm. |
| `leader_wam8g --wrist-force g` | The gain (0 to 1, or three for J6,J7,J8) on those wrist torques, applied to the leader's wrist with the position map's signs. Low-passed (`--wrist-force-filter`, 15 Hz), nothing below `--wrist-force-deadzone` (0.03 Nm), at most 0.8 Nm. |
| `leader_wam8g --wrist-deadband deg` | The leader wrist's position coupling becomes a tether: no pull while the follower's wrist is within `deg` of the leader's, the usual spring on what is beyond. Works without the sensor (to hear whether the coupling is the heaviness); with `--wrist-force` it is slack only while the follower's measured torques arrive. |
| `follower --wrist-friction nm` | A friction feedforward on the follower's J5-J7 in the direction the leader's wrist turns, so it lags less (no sensor needed). Start low (0.1): more than the friction drives the wrist ahead. |

`c` (4C off) turns the wrist's force channel and the tether off with the force
channels, and back on, fading over 1 s: the A/B on the same arms. `w` in the
leader's pane sets gain and deadband live; `z` in the follower's takes the
sensor's zero again, and `wam ft-zero` does the same from any terminal (the
sensor drifts: nothing may touch the gripper for the second it takes).

**The gripper's calibration (`--ft-tool`).** The sensor reads the gripper's
weight too, and that changes as the wrist turns (a 1.2 kg hand: up to 23 N
and about 2 Nm between pointing up and down). `wrist`, `arm` and `all`
therefore refuse to start without the gripper's mass and centre of mass.
`scripts/ft_tool_cal.py` fits them from a recording, together with how the
sensor's axes sit in the tool frame and its sign convention, so nothing about
the mounting is assumed.

`wam ft-cal` makes that recording by itself and fits it: with the teleop
stopped, the follower at home and in Idle (so that the sensor is powered), it
starts the follower alone (`follower --ft-cal`, no leader, no ROS). After
Shift-Activate and Enter the arm goes to the sync pose and turns the wrist
alone through 15 poses (`src/lib/ft_cal_poses.h`), 2.5 s still in each, and
comes back; about a minute and a half. Nothing can touch the gripper on the
way, which a recording made while teleoperating cannot promise (the first
one here had 40 N of contact in it and was refused). Keep clear, give the
gripper's cable slack, and keep the E-stop at hand; Ctrl-C stops the run and
the arm holds.

In the twelve poses that are fitted J5 turns from 0.9 rad one way to 1.5 rad
the other and J6 0.9 rad either way, with J7 at the sync pose's angle. The
first run on slax (2026-11-11, J7 up to a radian either way) showed why: the
sensor read a load that followed J7 and nothing else, 5 N a radian beyond
the sync angle and 1 to 2 N a radian short of it, through the middle of the
sensor and with a twist about the tool axis that grew with the angle, less
on the way back than on the way out. That is something winding up across
J7, not the gripper's weight, and it spoiled the fit (0.95 kg with 2.1 N
unexplained; 0.945 kg with 0.8 N where J7 was at the sync angle). The table
ends with J7 0.35 rad either way (about what the teleop reaches) and the
first pose again: the script fits where J7 is at the sync angle
(`--j7-window 0.1`) and compares the rest with that fit, so the load and the
sensor's drift over the run are printed, not fitted.
The second run, with this table, was accepted: 0.957 kg, centre of mass
54 mm out, 0.74 N rms left (worst pause 1.3 N); that is the file in
`config/follower-slax-skid-ft/`.

The fit prints the mass, the centre of mass, the sensor's rotation, what is
left unexplained and a table of every pause against the fit, and writes the
file only if the fit leaves less than 1 N rms and no pause is more than 2 N
off. On slax it reports the sensor's axis about 10 degrees off the tool
axis, about the J6 axis: a bolted sensor cannot be tilted, so this is more
likely the wrist's zero (slax is zeroed by its home pose only, so J6's zero
is wherever the wrist sat at start-up) or the sensor mixing its axes; the
file keeps the axes as fitted, which is what removes the weight best.
`workspace/tools/ft_cal_check` shows, without the robot, how well the fitted
poses spread gravity over the tool frame for a follower configuration (the
fit needs 0.15 in the weakest direction).

`wam ft-cal FILE` fits a recording made another way (`FOLLOWER_EXTRA="--ft
log"`, a teleop in which nothing touches the gripper while the wrist is
turned through its range and the arm pauses in a few poses).

The follower takes the sensor's zero at the link pose (after `l`, before
Enter: keep clear of the gripper), which also removes its drift since the last
session; only the change in the gripper's weight since that moment is
computed.

**Reading the sensor.** The library's `ForceTorqueSensor::update()` waits for
the two replies with the bus locked, for up to a second, and throws if they do
not come: in the control loop, a late cycle every cycle and a dead loop when
the sensor's cable does. `src/lib/ft_barrett.h` instead takes whatever replies
have arrived and sends the next request, never waiting (a reading is one cycle
old when used). A sensor that stops answering is counted and reported; within
0.1 s its torques fade out, J1-J4 go back to the model's estimate, and the
leader's wrist goes back to the plain position coupling. A reading is limited
(60 N, 6 Nm, 2000 N/s, 200 Nm/s) and low-passed (`--ft-filter`, 40 Hz) before
it becomes a torque.

**What to expect, and what to watch.** The wrist's force channel also carries
the gripper's own inertia as the wrist accelerates (the sensor cannot tell it
from a contact). That is the limit on `--wrist-force`: the gain times the
gripper's inertia about a wrist axis should stay below the leader wrist's own
(with the hand on it), or the two wrists can ring against each other. Start
at 0.2, raise it with `w`, and stop where a tap on the leader's wrist rings.
The deadzone keeps small torques out; the recording has `wrist_ff` (the torque
applied), `wrist_ext_rx`, `ft_tau`, `ft_f`, `ft_n` to look at afterwards, and
`wam log` prints the sensor's reading rate, how long reading it took the loop,
and the wrist coupling's torque with and without the tether.

Tests, without a robot: `low_latency_tests` (the arithmetic; `--ft-samples` /
`--ft-check` run `ft_tool_cal.py` against a made-up gripper),
`wam8g_loopback_tests` and `_legacy` (the library's frames and the Jacobian
transpose against virtual work on the follower's kinematics; a leader and
follower with a stand-in sensor: gains and signs per joint, the tether, 4C
off, the sensor lost and back), `ft_bus_tests` (the read on libbarrett's bus
code and a simulated bus next to the arm's 500 Hz traffic: no waiting, a
silent sensor, duplicate replies).

### Running

Before the fit (checks the sync pose and the joint map with no model):

```bash
source scripts/setup_leader_wam8g.sh
rosrun wam_teleop leader_wam8g --law u1 127.0.0.1 5555 5554
```
```bash
source scripts/setup_follower.sh
rosrun wam_teleop follower --sync wam8g --law u1 127.0.0.1 5554 5555
```

With the fit: `leader_wam8g` (u6) and `follower --sync wam8g` (u4). Link as
above (`l` on each, then Enter).

The wam8g sync pose is `(π/2, −π/2, −π/2, π/2, 0, 0, 0, 0)` rad, matching
the saber demo's start pose (2026-09-25). J5 is passive and keeps its measured
angle during the move; its nominal target is zero. The P2 follower with
`--sync wam8g` uses the same J1–J4 angles and a straight wrist `(0, 0, 0)`.

The follower sets the P2 safety module's velocity fault limit to 3 m/s (the
arm's Cartesian speed; warning at 90%) after startup, since the library reloads
the module's defaults at every start; `--velocity-limit v` sets `v` m/s instead.
`leader_wam8g` sets the wam8g safety board's, per joint, to 4 rad/s (warning at
90%): the board powers up with 1.0 rad/s (warning 0.5) and the library leaves
it, and at 3.0 a fast forearm roll faulted the leader at 3.3 rad/s on J3.
`--velocity-limit v` sets `v` rad/s instead. Both programs print the limits in
effect at startup.

From the first link, each program stops the teleop when its own arm leaves
ACTIVE (a fault, E-stop, Shift-Idle; `src/lib/safety_watch.h` polls the safety
module at 10 Hz). The stopped side sends nothing more, so the other side's link
times out within 20 ms and it holds the stopped arm's last position; the
stopped arm tracks its own position, so it does not jump if it is reactivated.
Restart both programs to link again. Before this, a faulted leader went on
sending its sagging position and the torques it could no longer apply: the
follower shook, then followed it down into its stops. `x` ends the teleop the
same way and holds the arm where it is until Shift-Idle.

`leader_wam8g --j6-friction nm` (default 0, off) adds a friction feedforward on
J6, whose bearing drags (its outer race was squeezed when it was installed): a
torque in the direction J6 turns, `nm * tanh(v / 0.05 rad/s)`, from the first
link until the teleop stops. `f` changes it live. More than the drag would turn
J6 by itself.

`wam8g_j6_friction` (after `source scripts/setup_leader_wam8g.sh`) measures J6's
friction by commanding its motor torque directly. The arm goes straight up
(J2-J4 at 0), where J6's axis is vertical, so gravity puts no torque on J6;
J6's position gains go to zero while the rest of the arm holds the pose. The
torque rises at 0.1 Nm/s, one way and then the other, four times each, until J6
has turned 0.01 rad, and then drops to zero: the breakaway torque. The average
of the two ways is the friction; half their difference is whatever else pushes
J6 one way there (the gravity model, cogging). It stops if J6 turns the wrong
way, turns more than 0.3 rad or faster than 1 rad/s, does not move by 2 Nm, or
the arm leaves ACTIVE; then J6's gains come back and the arm returns to where it
started. It suggests half the breakaway for `--j6-friction`: the drag once J6
turns is below the breakaway.

The wrist is not coupled unless `leader_wam8g` gets `--wrist`, which maps joint
for joint, relative to the two sync poses: wam8g J6 -> P2 J5, J7 -> P2 J6
(reversed: with the same sign the leader's J7 turned the follower's J6 the
wrong way, 2026-09-28), J8 -> P2 J7 (the passive J5 maps to nothing). The follower's sync wrist is J5 -pi/2,
J6 0, J7 +pi/2: with J6 at 0 its J5 and J7 turn about the same axis and cancel,
so the hand points as with the wrist at zero, and the P2's J6 axis lines up
with the wam8g's J7 axis (at J5 = 0 it was 90 degrees off). The follower turns
J5 and J7 there together in its move to the sync pose, and the map is relative
to that pose, so linking does not jump. Check each direction on the robot
(`src/lib/wam8g_wrist_map.h`).

`wam8g_teleop_tests` checks the wrist map and the laws; `wam8g_loopback_tests`
runs the leader and follower code against each other over UDP, checks that each
position controller tracks that cycle's reference inside the Wam's torque graph,
and checks the excitation's wrist hold; `wam8g_loopback_tests_legacy` runs the
same on the follower's library. None needs a robot.

### BarrettHand on the follower (Sony navigation controller)

With `--hand` the follower also drives the BarrettHand (BH8-282, pucks 11-14)
on its CAN bus: the grasp from `/follower/hand/velocity_setpoint`
(`gripper_ros_common/VelocitySetpoint`, its `grasp` in rad/s; its `spread` is
not used) and the spread from `/follower/hand/next_pose` (`std_msgs/Empty`).
`launch/hand_nav_controller.launch` feeds both from the navigation controller:
`joy_node` (`/dev/nav_controller`, from `99-nav_controller.rules`) ->
`joy_teleop` (`config/hand_nav_controller.yaml`) -> `bhand_mux` for the grasp,
and `joy_teleop` straight to `next_pose`. Hold L2 and move the stick up/down to
close/open the fingers, up to 2 rad/s; let go of L2 and they stop within 0.1 s.
L2 + cross steps the spread to its next pose:

| Press | Spread | Fingers | The grasp moves |
|---|---|---|---|
| (start) | 0% (0) | F1 and F2 side by side, opposite F3 | F1, F2, F3 |
| 1 | 50% (90 degrees) | F1 and F2 face each other: a pinch | F1 and F2; F3 stays open |
| 2 | 100% (180 degrees) | all three side by side, closing the same way | F1, F2, F3 |
| 3 | 0% | back to the start | F1, F2, F3 |

A switch drives F1-F3 open (the hand lets go of what it holds), then turns the
spread to the pose, then gives the stick back with the fingers open; open
fingers clear each other at any spread (BH8 manual 8.4). The stick waits
meanwhile. Everything in a switch is a velocity command, what the stick has
always sent and the one thing proven on this hand: each digit is driven toward
its target from its position readings, slowing near it, and stopped there
(V = 0: its puck servos that spot). The hand's own "open" (CMD open) and
trapezoidal moves are not used, so a hand that ignores them still switches
(`hand_teleop_tests`). Nothing but a switch ever commands the spread, so it does
not creep while the fingers work; its 0% and 100% poses are 1 degree inside its
stops, so it holds there without pushing into them. Once open, the fingers are
idled (their worm drives hold them) rather than pushed against their stops. On
the simulated hand a switch takes about 1-2 s with the fingers open and 2.2 s
from a closed fist. One press is one step: holding L2 + cross is one step, and
a press during a switch is ignored (the follower says so), so a visitor
pressing repeatedly gets one switch at a time. `--no-spread-poses`
(`SPREAD_POSES=no` in `wam_settings.sh`) turns the poses off: the grasp only,
the spread held at 0%.
A switch waits on progress, not on a fixed time: the spread turns once all
three fingers read open, whatever the hand's speeds. If a finger stops short of
open (blocked) for 0.3 s, or the spread does not move at all, the switch ends
and the hand keeps its pose. At the end the follower brings the spread back to
0% the same way before it closes the fist.

At the link pose the follower checks that the hand answers the switch's
position requests and that the fingers read closed after the fist (else the
hand's zero is off: HI skipped since it was powered on), and prints
`Hand: spread poses on (L2 + X); ... the hand reads F1 .., spread .. rad` (or
why the poses are off; the grasp works either way). It prints each press it
receives (`Hand: L2 + X received: ...`) and each step of a switch; no such line
means the press did not reach it (check the buttons with `wam joytest`). The
hand is live from the link pose on, before linking: try it at the follower's
"Press [Enter] to link" prompt.

The controller works over USB or Bluetooth: thor's kernel (hid-sony) gives it
the same buttons and axes either way, and `99-nav_controller.rules` links
either to `/dev/nav_controller`.

```bash
roslaunch wam_teleop hand_nav_controller.launch
rosrun wam_teleop follower --sync wam8g --hand 127.0.0.1 5554 5555
```

The hand does not move until the arm is at the link pose. After `l` moves the
arm there (the first time), the follower asks before homing the hand (HI: the
fingers open fully and the spread swings through its range); `s` skips HI when
the hand was homed since it was powered on. Then the hand closes to a fist
(spread at 0%, fingers closed) and teleoperation starts from the fist. At `x`,
with the arm holding still, the hand closes to a fist again and idles: when the
spread is away from 0 the fingers open first, the spread returns to 0 and the
fingers close, so the hand lets go of whatever it holds. If the arm stops (a
fault, E-stop, Shift-Idle; see above), the fingers stop where they are (in a
switch the spread holds where it is) and the hand is sent nothing more.

The follower must be the only program talking to the hand on this bus: do not
run `gripper_ros`'s `bhand_node` (or pyHand) next to it. libbarrett keeps every
frame the hand's pucks send and throws `Buffer overflow` once ten unread ones
pile up; the follower's realtime loop does not catch that, and the program
ends. A program polling the hand does that within 0.2 s
(`hand_teleop_tests`).

The hand thread (`src/lib/hand_teleop.h`) runs at 50 Hz outside the realtime
loop. For the grasp it only sends: V when a value changes, MODE when a finger
starts moving, both again every 0.25 s while it moves, V = 0 when the command
is zero or 0.1 s old. It never waits on the bus, because libbarrett holds the
bus lock while it waits for a reply and a silent puck would stall the arm's
loop for 1 s. In a switch it follows the fingers and the spread with
non-blocking position requests (the hand group's request of libbarrett's
`Hand::update()` in one step, the replies picked up in later ones); a hand
puck that stops answering ends the hand's control after 0.5 s, and the arm's
loop does not notice. The hand's setup and HI at
startup do wait on the bus, as in Barrett's own programs.

`hand_teleop_tests` runs libbarrett's Hand and the hand thread on a simulated
bus next to the arm's 500 Hz traffic, including the poses (the fingers are open
whenever the spread moves, each pose holds against a push), a blocked finger
and a silent puck; `hand_chain_tests` checks the joystick chain end to end,
L2 + cross included (start `roscore` and the launch file with
`joy_dev:=/dev/no_joystick` first). Neither needs a robot.

### Robotiq 2F gripper (Sony navigation controller)

A Robotiq 2F-140 on the follower is worked from the same navigation controller
as the BarrettHand, by a node of its own: `robotiq_teleop` owns the gripper's
serial port (Modbus RTU, through libgripper's
`gripper::robotiq::RobotiqGripper`) and reads the controller; the arm programs
know nothing of it. `launch/robotiq_nav_controller.launch` starts `joy_node`
(as for the hand) and the node. With `GRIPPER=robotiq` in `wam_settings.sh`,
`wam teleop` starts it in a `gripper` pane. It is the hand or the gripper:
with `HAND_CONTROLLER=yes` as well, `wam teleop` says so and starts nothing.

| Controller | The gripper |
|---|---|
| L2 + stick up / down | closes / opens, at a speed proportional to the stick, 50 mm/s at full stick; with L2 let go or the stick centred the opening stays |
| L2 + cross | opens fully: one press, one action; stick up on the way turns it round |
| L2 + circle, held 1 s | restarts the gripper (below); a shorter press does nothing |
| off, or out of range | 0.5 s after the last message the gripper stops where it is; a grip stays |

The gripper takes positions, not velocities: a request holds the opening to go
to and the speed and force to use on the way. So the stick moves a target
opening, and every change of it by one of the gripper's 255 position counts
(0.55 mm) goes out as a request, always with `device_speed` and `force`
(0.3 and 0.5, libgripper's defaults: about 95 mm/s and 70 N on a 2F-140, whose
ranges are 30-250 mm/s and 10-125 N). The gripper's own speed has to stay
above `stick_speed` for the opening to follow the stick; the node says so at
its start if it does not.

While the stick moves the target it never gets more than `lead` (5 mm) ahead
of the measured opening. Closed on an object, or on the fingers' own stop,
the request therefore stays up to 5 mm beyond the contact: the gripper keeps
its force on the object, follows it by at most those 5 mm if it gives way
(the gripper's re-grasp), and opens within a step or two when the stick goes
down, however long it was held up. This goes by the measured opening, not by
the gripper's object flag, which its manual says can miss a grasp. The
millimetres are libgripper's straight line from count 0 (140 mm) to count 255
(0 mm); the real fingers stop a few counts inside both ends (the manual's
example reads 13 at fully open), so they are nominal.

Settings: `GRIPPER`, `GRIPPER_PORT` (the serial device as the container sees
it) and `GRIPPER_ARGS` (more arguments for the launch file) in
`wam_settings.sh`; speeds, force, lead, buttons, the Modbus address in
`config/robotiq_nav_controller.yaml`; `auto_activate` and `auto_restart` as
launch arguments (`GRIPPER_ARGS="auto_restart:=true"`).

**What the pane prints.** One line per event, with the time, also kept in
`pane_logs/<start>_gripper.log` (these are from the simulated gripper):

```
14:19:00 Gripper: Robotiq 2F on /dev/ttyUSB0 (115200 baud, slave 9). L2 + stick up/down: close/open, at up to 50 mm/s. L2 + X: open fully. L2 + circle held 1 s: restart. Each request: speed 0.30, force 0.50.
14:19:00 Gripper: the serial port is open; waiting for the gripper to answer.
14:19:00 Gripper: the gripper is not activated yet (as after it was powered on). A restart activates it: L2 + circle held 1 s, or `wam gripper restart`, once its fingers have room. They run through their travel: whatever they hold falls. (activated NO, ready NO, gOBJ 0, fault 0x00, request echo 0, position 13 (132.9 mm), motor current 0 mA)
14:19:05 Gripper: RESTART (L2 + circle held 1 s): closing and opening the serial port, then reset and activation. The fingers run through their travel: whatever they hold falls.
14:19:07 Gripper: restarted in 1.8 s: ready at 132.9 mm.
14:19:08 Gripper: closing stopped by contact at 59.8 mm: holding.
14:19:35 Gripper: serial: [Serial] Write error: Input/output error
14:19:36 Gripper: the gripper is NOT ANSWERING: no status for 0.5 s. Nothing is sent to it; the serial port is closed and opened again every 1 s. Its last status: activated yes, ready yes, gOBJ 3, fault 0x00, request echo 104, position 104 (82.9 mm), motor current 0 mA.
14:19:40 Gripper: the gripper answers again and is still activated: ready at 82.9 mm. No restart, nothing moved.
14:23:26 Gripper: the gripper reports fault 0x0E, overcurrent triggered. Nothing is sent to it until it is restarted: L2 + circle held 1 s, or `wam gripper restart`. A restart runs the fingers through their travel: whatever they hold falls. (activated yes, ready yes, gOBJ 0, fault 0x0E, request echo 56, position 56 (109.3 mm), motor current 320 mA)
```

The `serial:` lines are libgripper's own error messages, each different one
at once and repeats as a count every 10 s. Every line about a fault or a
silence carries the gripper's status as it was last read. A gripper that
looks healthy and does not do as asked is reported too (a request its echo
does not show after 1 s; "in motion" for 2 s without moving).

**Restarting it.** Three ways, all the same restart: L2 + circle held for a
second; `wam gripper restart` from any terminal; or
`rosservice call /robotiq_teleop/restart` (it answers when the restart is
over, with the pane's line about it). The node closes the serial port, opens
it, resets the gripper (which clears its faults), activates it, waits until
it is ready and at rest (10 s at most), and takes the opening it then reads
as the target. **The activation runs the fingers through their travel, and
libgripper then sends them fully open: whatever the gripper holds falls.**
That is why a restart waits to be asked for. The arm programs are not
involved and keep running.

Not everything needs one:

| The gripper | The node |
|---|---|
| does not answer for 0.5 s (adapter unplugged, cable, power) | sends nothing, closes and opens the port every second, says whether the port opens. When the gripper answers and is still activated, the teleop goes on from the opening it reads: no restart, nothing moves, a grip is kept |
| answers, but not activated (it lost its power) | waits for a restart |
| fault 0x07 (activation bit must be set), 0x0A (under minimum voltage), 0x0B / 0x0F (automatic release in progress / completed), 0x0C (internal fault), 0x0D (activation fault), 0x0E (overcurrent) | prints the code and the manual's words for it, waits for a restart; these need the reset |
| fault 0x05 (action delayed: activation must be completed), 0x08 (over 85 C inside), 0x09 (no communication for 1 s) | prints it and waits for it to clear, which the manual says needs no reset; suggests a restart if it has not after 5 s |
| answers out of step (one answer came after libgripper's 0.2 s time-out and stays on the line, so that every later request is paired with the answer before it) | after three position requests without a proper answer it closes and opens the port, which empties the line; no restart |

At its start the node takes a gripper that is already activated and ready as
it is, so a node that died and was started again by roslaunch does not drop
what is held. One that is not activated, as after it was powered on, waits
for a restart like any other: nothing moves until it is asked for, with the
arm where the fingers have room (as the follower asks before it initializes
the BarrettHand). `auto_activate:=true` activates it at the start without
asking. `auto_restart:=true` restarts without asking whenever a restart is
needed, at most every 10 s; the gripper then lets go by itself.

`wam gripper restart` asks the node. If the node does not answer within 30 s
(hung in the serial library), it stops the node; roslaunch starts a new one
within a second, and the restart goes through that. A node that finds its own
loop stuck for 8 s when a restart is asked for ends itself for the same
purpose. If the launch file itself has ended, it is started again in the
gripper pane. `wam gripper status` prints whether the serial device is there
and the node's state in one line (also
`rosservice call /robotiq_teleop/status`). `wam stop` ends the gripper pane
with the rest; the node sends the gripper nothing at the end, so it stays as
it is, holding what it holds (the fingers are self-locking).

`/robotiq_teleop/joint_states` (`sensor_msgs/JointState`, joint
`finger_joint`) has the opening in m and the motor current in A as the
effort, while the gripper answers; `RECORD=bag` records it.

**On the robot computer, before the first use.**

1. The gripper powered (24 V) and its RS-485 adapter plugged into the robot
   computer. `ls -l /dev/serial/by-id/` names the adapter; that name stays
   the same when it is plugged in again, which `/dev/ttyUSB0` does not (it
   can come back as `ttyUSB1`, and the node would wait for a device that is
   gone). Put it in `GRIPPER_PORT`.
2. The container has to see it: `docker exec <container> ls -l <that name>`.
   The lab's `docker-compose.yml` runs the container privileged with `/dev`
   bound, and then it does, also after a re-plug; a container made otherwise
   has to be made again with the device.
3. libgripper's branch `robotiq` at 2ddb4c7 ("tested and working",
   2026-11-11: the register packing that was tried on the gripper), in the
   workspace's `libgripper` checkout: `git fetch && git checkout robotiq`.
   Then, in the container, the workspace's `build_gripper.sh` (cmake, make,
   `sudo make install` to `/usr/local`, and the links of `gripper_ros` and
   `gripper_ros_common` into `wam_ws/src`) and `build_ws.sh` (`catkin_make`).
   CMake prints `robotiq_teleop: built (libgripper with the Robotiq
   classes)`; with an older libgripper it says why not and builds everything
   else as before. That branch left `main` in August 2025 and has none of
   main's later work (the gecko gripper, the BarrettHand changes of 2026):
   installing it replaces those in `/usr/local` and in `gripper_ros`. This
   package only uses `gripper_ros_common` of it (the follower's `--hand`
   messages), which is the same on both branches.
4. `GRIPPER=robotiq` and `HAND_CONTROLLER=no` in `wam_settings.sh`; then
   `wam teleop`. Without the arms: `roslaunch wam_teleop
   robotiq_nav_controller.launch port:=<that name>`. The pane says that the
   gripper is not activated yet; with the fingers clear, hold L2 + circle
   for a second (or `wam gripper restart`).
5. Do not run `gripper_ros`'s `robotiq_node` on the same port at the same
   time: two programs asking on one serial line get each other's answers.

**Not tried on the gripper.** Everything above ran against a simulated
gripper only (`test/robotiq_sim.py`, whose frames match the manual's Modbus
example byte for byte). Open on the real one: what "bugs out" is (the pane's
log of the first time will say); whether a new request in mid-move takes
effect at once, as the stick needs; how small a change of the request it
acts on; where and how it reports contact, and what `force` 0.5 does to what
it holds; that activation ends with the fingers open, and how long it takes
(the node waits 10 s); whether fault 0x09 appears on this serial line and
clears by itself; whether answers ever do get out of step on the real
adapter, and that closing its port clears them; the rate of the exchanges
through the adapter (with an FTDI adapter's default latency timer, 16 ms
each, the loop runs at about 30 Hz instead of 40; the stick's speed does not
depend on it).

`robotiq_teleop_tests` runs the logic (`src/lib/robotiq_teleop_core.h`) in
simulated time against a made-up gripper; no library, no ROS. With a
workspace built for the gripper, `test/robotiq_e2e.sh` runs the real node and
launch file against `test/robotiq_sim.py` on a pseudo-terminal (activation,
the stick, an object, restarts, faults, the serial device going away and
coming back, the node killed), and `test/robotiq_wam_check.sh` the launcher's
part with stand-ins for Docker and the arm programs.
`test/robotiq_docker.sh <libgripper clone>` does all of it in Docker, with no
libgripper, with its `main` and with its `robotiq` installed.
`python3 test/robotiq_sim.py --link /tmp/robotiq_sim` alone is a gripper to
try the pane against (`port:=/tmp/robotiq_sim`; `help` lists what can be done
to it).
