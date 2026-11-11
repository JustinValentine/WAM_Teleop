# Cable-drive dynamics identification (`cable_ident`)

Identifies each arm's J1–J4 dynamics for the sensorless 4C teleop and writes
the parameters the controller runs (`src/lib/cable_params_*.h`, used by
`leader_wam8g --model cable` and `follower --model cable`). The method and the
evidence are in [METHOD.md](METHOD.md).

## Setup (host, no ROS needed)

```
python3 -m venv ~/.venvs/cable_ident
~/.venvs/cable_ident/bin/pip install -r requirements.txt
cd wam_ws/src/wam_teleop/ident
~/.venvs/cable_ident/bin/python -m cable_ident --help
```

The tool compiles the controller's own regressor (`src/lib/regressor_W.h`) with
`g++` on first use. Bag extractions are cached in `~/.cache/cable_ident`
(`CABLE_IDENT_CACHE` to move it; the two largest bags take a few minutes once).

## Commands

| command | what it does |
|---|---|
| `fit --arm leader\|follower` | fits the model on the arm's excitation log(s) and the self-driven teleop segments (below); writes `src/lib/<header>` and a `.json` record |
| `cv --arm ...` | leave-one-session-out: fits without one teleop session, scores that session's self-driven segments, for every session |
| `ablate --arm ...` | the `cv` for the model variants in METHOD.md's table |
| `hysteresis --arm ...` | friction after reversals against displacement and against time (rate independence) |
| `design --arm ... --out traj.txt` | the next excitation: D-optimal against the data already fitted, inside the configurations the arm visited in teleop, plus reversal ladders |

`--exc-weight` sets the excitation's total weight relative to the teleop
segments (0.25 for the shipped headers: the teleop region dominates, the
excitation still pins what the teleop does not excite).

### Data

* Excitation logs: `excitation/*.dat` (`dynamics_excitation`). The 4-joint
  format (t, q1–q4, τ1–τ4 at 250 Hz) and the full format written by
  `--trajectory` (t, q1–qN, dq1–dqN, τ1–τN at 500 Hz) are both read.
* Self-driven teleop segments: in every session each arm drives itself to the
  sync pose (and home at the end) before anyone touches it. `data.self_driven`
  finds these moves (smooth, ending at the sync pose or home, followed by a
  rest) in `teleop_logs/*.bag`; they are free motion in the teleop's own
  workspace. The leader's holds are cut 3 s after arrival (the operator may
  take the handle).
* `arms.py` lists, per arm, the logs that share its current calibration.
  **After a zerocal or gravitycal, start a new list**: data from before it
  mixes two joint-angle offsets or gravity models.

## One iteration

1. `python -m cable_ident design --arm follower --out trajectories/follower_roundN.txt`
   (likewise the leader). It prints the information gain and the limits.
2. On the robot, with the E-stop at hand:
   `dynamics_excitation_legacy --trajectory <traj> excitation/zeus_roundN.dat`
   (the leader: `dynamics_excitation_canopen` after `source scripts/setup_leader_wam8g.sh`).
   It moves to the trajectory's start (J1–J4; the wrist stays where it is),
   waits for Enter, plays the file with position control and logs every
   joint at 500 Hz. It stops and holds if a joint leaves its bounds or lags
   the reference by more than 0.1 rad.
3. Add the log to the arm's `excitation` list in `arms.py`, then `cv` (the new
   round should lower the held-out error) and `fit`.
4. Rebuild (`source build_ws.sh`), run the teleop with `--model cable`, and
   record as usual: the new sessions' self-driven segments join the next fit.

Stop when another round no longer lowers the `cv` error.

## In the controller

`--model cable` replaces LeaderDynamics/FollowerDynamics with
`lib/cable_drive_dynamics.h`:

* the external-torque estimate subtracts the full model (inertia from the
  controller's acceleration at full scale, the friction elements, the wrist's
  gravity relative to the identification pose);
* the feedforward (`dyn` in the laws) keeps the old terms by default and adds
  `--ff-friction` of the friction and `--ff-inertia` of the inertia
  (defaults 0 for the leader; the follower's inertia share stays 0.25 as in
  legacy). The key `m` changes both live. Compensating all of the friction
  can drive a joint by itself: raise `--ff-friction` in steps.

`test/test_cable_drive_model.cpp` checks the controller's model against this
package sample by sample (and the friction's passivity and its behaviour at a
one-count toggle); `test/test_arm_gravity.cpp` checks the wrist correction.
After a new `fit`, regenerate the test vectors (see the test's header).
