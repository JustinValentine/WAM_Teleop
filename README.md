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

`low_latency_tests` checks the link, the mode, the aged reference, the
recorder, the estimator's arithmetic and the thread setup without libbarrett;
`wam8g_loopback_tests` (and `_legacy`) also run the mode over the link, the
aged reference, a recording from both arm systems, the estimator systems
against the classes they replace, and the control thread each library starts.

### Force/torque sensor and the wrist (2026-11-09)

The wrist is coupled position to position only (the laws act on J1-J4), so
the leader's wrist feels its position controller pulling it toward the
follower's wrist, which lags by its own friction and inertia: in the IROS
recordings a median 0.1-0.25 Nm against the hand while turning, 0.4-0.6 Nm a
tenth of the time, on top of the wam8g wrist's own friction. A contact reaches
the hand only through the same position error.

With the Barrett force/torque sensor on the follower's tool plate, the contact
is measured instead (`src/lib/ft_core.h` has the arithmetic and the reasons).
None of this has run on the robots; everything is off unless asked for.

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
the mounting is assumed:

1. `FOLLOWER_EXTRA="--ft log"` in `wam_settings.sh`, `wam teleop`, link.
2. With nothing touching the gripper (and the hand as it will be used), turn
   the wrist through its range and move the arm between a few poses, pausing a
   second or two in each; about a minute.
3. `wam stop`, then `wam ft-cal`. It prints the fit (mass, centre of mass,
   the sensor's rotation about the tool axis, what is left unexplained) and
   writes the file only if the poses covered enough directions and the fit is
   clean; otherwise it says what was missing.

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
