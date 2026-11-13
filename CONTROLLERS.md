# The controllers in use

What runs on each arm in the teleop of the wam8g leader (WAM 153's four-joint
base with the Alberta wrist) and the slax follower (7-DOF WAM, skid gripper,
Barrett force/torque sensor), as of the session of 2026-11-11 16:37 on odin.
Everything here is read from the code; file names are relative to `src/`.

Both programs run one control cycle every 2 ms (500 Hz) and exchange one UDP
packet per cycle (`lib/teleop_link.h`): joint positions, joint velocities, the
external torque, the sample time and a few flags.

## 1. The torque on each arm

Every cycle, each arm's motors get the sum of three torques:

    tau = tau_gravity + tau_PID + u

| Term | What it is | Where |
| --- | --- | --- |
| `tau_gravity` | gravity compensation from the arm's gravity calibration | the Barrett library (section 8) |
| `tau_PID` | a joint-space PID that makes this arm's joints follow the other arm's: the position coupling | the Barrett library's `jpController`; its reference and gains are set here (section 2) |
| `u` | the feedforward of the control law: the force channel and the dynamics compensation | `lib/teleop_law.h`, `lib/leader_wam8g.h`, `lib/follower.h` (sections 3 to 6) |

There is no task-space or null-space control: every joint of one arm is
coupled to the matching joint of the other. The only Jacobian is the
follower's, which turns the sensor's wrench into joint torques (section 5).

## 2. The position coupling (`tau_PID`)

The library's PID, one per joint (`systems::PIDController`, the step itself is
in section 8):

    e    = q_ref - q
    I   += ki * T * e_previous            (limited)
    tau  = kp * e + I + kd * (e - e_previous) / T     (limited)

`q_ref` is the other arm's joint position from the latest packet.

* With `--reference age` (in use): `q_ref = q_other + age * dq_other`, where
  `age` is the time since the other arm sampled its joints, at most 6 ms
  (`teleop_law::agedReference`). Without it the reference is held, and the
  coupling drags by `kp * age` per rad/s.
* The wrists are mapped joint to joint relative to the sync pose
  (`lib/wam8g_wrist_map.h`): wam8g J6 to follower J5, J7 to J6 (reversed), J8
  to J7. The wam8g's J5 is a passive pin and is never commanded.
* If no packet arrives for 20 ms the link is dropped and each arm holds.

Gains as run (N m/rad, N m s/rad; the integral gains are zero on both arms,
`--ki-scale 0` and `ki = 0` in the follower's configuration):

| Leader (wam8g) | J1 | J2 | J3 | J4 | J6 | J7 | J8 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| kp | 750 | 1000 | 400 | 200 | 20 | 10 | 10 |
| kd | 10 | 8 | 3.3 | 1.2 | 0.1 | 0.1 | 0.1 |
| limit (N m) | 50 | 80 | 50 | 30 | 5 | 5 | 5 |

| Follower (slax) | J1 | J2 | J3 | J4 | J5 | J6 | J7 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| kp | 750 | 1000 | 400 | 200 | 20 | 10 | 8 |
| kd | 10 | 8 | 3.3 | 0.8 | 0.5 | 0.5 | 0.05 |
| limit (N m) | 50 | 80 | 50 | 30 | 5 | 5 | 5 |

The leader's J1-J4 gains are compiled in (`lib/wam8g_teleop.h`); its wrist
gains come from `workspace/config/wam8g/wam8g.conf`, the follower's from
`config/follower-slax-skid-ft/wam7w.conf`.

## 3. The law (`u`), J1-J4

`teleop_law::compute`, law `u4` (the four-channel architecture: positions and
forces in both directions):

    u = -g * ext_other + dyn - grav           g = --force-gain = 0.4

* `ext_other`: the other arm's external torque (section 4 or 5), received in
  the packet. It fades in over 2 s after both arms are linked and passes a
  notch at 35 Hz, Q 0.7 (`--force-notch 35`; the loop rings at 30-43 Hz
  without it).
* `ext` is defined as commanded torque minus model torque, which is minus the
  torque from outside. So `-g * ext_other` pushes this arm the way the other
  arm is being pushed, scaled by `g`.
* `dyn - grav`: this arm's dynamics compensation (section 6), replacing the
  library's gravity torque by the fitted model's.

The other laws in the file: `u1 = 0` (position-position only), `u2 = dyn -
grav`, `u6 = u4 - 0.15 (ext_other + ext_own)`. Keys or `wam 4c off` / `wam pp
on` switch the force channels or all of `u` off and on while linked, fading
over 1 s (`lib/teleop_mode.h`).

## 4. The external torque from the model

    ext = (u + tau_gravity + tau_PID) - tau_model(q, dq, ddq)

`lib/dynamic_external_torque.h`. The acceleration `ddq` is the joint velocity
through a first-order high-pass (25 rad/s) and a low-pass (50 rad/s), so the
model's inertial torque is limited to a few hertz.

* **Leader** (`--model cable`): `tau_model` is the cable-drive model,
  `lib/cable_drive_model.h` with the parameters in
  `lib/cable_params_wam8g.h`: rigid body (30 base parameters), motor rotor
  inertia, viscous friction at the motors, and cable friction as
  spring-and-slider (Jenkins) elements driven by motor position, six per
  motor and six on J2 and J3. Identified on WAM 153 on 2026-11-11 with
  `ident/cable_ident`.
* **Follower** (legacy model): `tau_model = W(q, dq, 0.25 ddq) beta`
  (`RegressorDynamics` in `lib/estimator_systems.h`, the regressor in
  `lib/regressor_W.h`) with the 30 parameters in
  `lib/follower_beta_slax_20261112.h`: gravity, Coriolis, viscous friction
  and a quarter of the inertial torque; no Coulomb friction. Since
  2026-11-12 these are fitted on the follower's own excitation runs of that
  morning, after its first zero calibration, a new gravity calibration and
  with the gripper mounted that day. On a run that was not fitted they leave
  1.38 / 1.67 / 0.66 / 0.68 Nm rms on J1-J4 (most of it the friction this
  model does not have); the parameters used until then
  (`lib/follower_beta_slax_skid_hand.h`, a lab-mate's fit for the earlier
  zero and gripper) leave 1.44 / 2.44 / 0.79 / 1.00 Nm there and are off
  by up to 1.4 Nm of gravity on J2. A cable-drive model fitted on the same
  runs (`lib/cable_params_slax_20261112.h`: 0.52 / 0.59 / 0.29 / 0.21 Nm) is
  what `follower --model cable` loads; it has not run on the arm (the
  follower runs the legacy model unless that option is given).

The leader sends its `ext` for J1-J4 to the follower. What the follower sends
is in the next section.

## 5. The force/torque sensor (follower)

`lib/ft_barrett.h` reads the sensor every cycle without waiting for it;
`lib/ft_core.h` and `lib/ft_systems.h` turn the reading into joint torques:

    W      = sign * rot * (reading - reading_at_zero) - (weight_now - weight_at_zero)
    tau_ft = J_tool(q)^T W

* `weight` is the gripper's, from `config/follower-slax-skid-ft/ft_tool.conf`
  (0.957 kg, centre of mass 54 mm out; fitted by `wam ft-cal`). The zero is
  taken at the link pose and again by `wam ft-zero`.
* `W` is limited to 60 N and 6 N m, to 2000 N/s and 200 N m/s, and low-passed
  at 40 Hz.
* With `--ft arm` or `--ft all`, the external torque the follower sends for
  J1-J4 is `(1 - use) * ext - tau_ft`, where `use` is 1 while the sensor
  answers (it falls to 0 within 0.1 s if it stops, back to the model's
  estimate). So the leader's force channel carries what touches the gripper,
  not the follower's friction or model error.
* With `--ft all` the packet's three wrist slots carry `-tau_ft` for J5-J7.
  The leader applies them to its wrist (`lib/leader_wam8g.h`):

      tau_wrist = -g_w * deadzone(lowpass_15Hz(sign * ext_wrist), 0.03 N m),  at most 0.8 N m

  with the position map's signs and `g_w` set by `--wrist-force` or the key
  `w` (0 at the start of the 16:37 session, then 0.2 and 0.5).

The follower itself still uses the leader's model-based `ext` in its own `u`.

## 6. The dynamics compensation (`dyn - grav`)

* **Leader**: the cable model's gravity, Coriolis, viscous friction and
  offsets, plus a share of its friction elements (`--ff-friction`, key `m`:
  0.2 at the start of the 16:37 session, raised to 0.5) and a share of its
  inertial torque (`--ff-inertia`, 0). `lib/cable_drive_dynamics.h`. The
  estimate of section 4 always uses the whole model; only these shares are
  fed forward, because feeding all of the friction or a filtered inertia
  forward feeds the arm's own motion back.
* **Follower**: the legacy model's torque as in section 4.
* **Leader's J6**: a friction feedforward `0.20 N m * tanh(dq / 0.05 rad/s)`
  for its bearing drag (`--j6-friction`).

## 7. As run

    leader_wam8g --law u4 --force-gain 0.4 --force-notch 35 --wrist --j6-friction 0.20 \
                 --model cable --ff-friction 0.2 --ki-scale 0 --reference age
    follower     --sync wam8g --law u4 --force-gain 0.4 --force-notch 35 \
                 --ft all --ft-tool config/follower-slax-skid-ft/ft_tool.conf --ki-scale 0 --reference age

and, by key during the session, the leader's friction share at 0.5 and its
wrist force gain at 0.5. `workspace/wam_settings.sh` holds these options;
`workspace/wam` starts both programs.

## 8. What is not in this repository

The two programs link against Barrett's libbarrett, which provides the 500 Hz
loop, the CAN communication, the joint PID of section 2, gravity compensation,
kinematics and the safety system:

* the follower uses the standard libbarrett for the WAM's original pucks;
* the leader uses a CANopen version of libbarrett 3.0 (its README names
  https://git.barrett.com/software/libbarrett as the source) with the lab's
  additions for the wam8g.

The PID step, in short, from the library's
`systems/detail/pid_controller-inl.h`:

    error = reference - feedback
    intError = saturate(intError + ki * T_s * error_1, intErrorLimit)
    controlSignal = saturate(kp * error + intError + kd * (error - error_1) / T_s, controlSignalLimit)
    error_1 = error

The programs set its reference with `wam.trackReferenceSignal(theirJPOutput)`
and add `u` with `connect(wamJPOutput, wam.input)` (`leader_wam8g.cpp`,
`follower.cpp`).

## 9. A known limit of the joint-to-joint force reflection

The follower's joint torques are copied joint for joint onto the leader, and
the two arms differ beyond the elbow (the leader's hand is 15.5 cm past the
forearm on the Alberta wrist, the follower's tool 6 cm past on the WAM wrist).
A force on the gripper therefore does not reach the operator as one force at
the hand: against what a force at the leader's own hand would need, the copy
gave J1 102 %, J2 97 %, J3 73 %, J4 80 %, J6 182 %, J7 143 % and J8 24 % in the
16:37 session. The remainder acts along the arm's elbow swivel, which a hand
on the handle cannot resist.

In that session there were seven downward pushes on a table (16-39 N on the
gripper). Both elbows rose by 6-9 cm within a second or two in three of them
(16, 24 and 17 N) and did not in the other four, the hardest (39 N) among
them, although the remainder pointed the same way in all seven. So the
remainder is there and is a candidate cause, but the recording does not show
that it alone decides whether the elbow lifts: the operator's arm, the
friction state and the pose differ from push to push. In the session before
(16:08) the same kind of push left a comparable torque along the swivel and
the elbow stayed. Three things changed between the two: half of the leader's
friction was compensated (0.2 before), the wrist's force channel was on, and
the age-compensated reference had removed the coupling's drag. Sending the
sensor's force through the leader's own Jacobian would remove the force
channel's part of the remainder; it has not been built.
