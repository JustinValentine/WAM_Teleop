# Cable-drive friction as observable hysteresis: convex identification for sensorless 4C teleoperation

Working notes for the RSS contribution (2026-10-05). Numbers are from the
logs in this repository; see "Status" for what is and is not yet validated.

## Claim

In a cable-driven arm the error that dominates a sensorless force estimate is
not the rigid-body model; it is friction. On the WAM that friction is a
**rate-independent hysteresis of motor displacement**: after a reversal it
rises with how far the motor has turned, not how fast. It is observable
through the motor encoders because the cable transmission has no backlash and
multiplies joint motion 18–42 times at the encoder. As a bank of Jenkins
elements (a spring in series with a Coulomb slider) at fixed slip
displacements, it is **linear in its parameters and passive when they are
nonnegative**. The whole inverse-dynamics model therefore becomes **one convex
program**: physically consistent rigid body (pseudo-inertia LMIs) plus passive
presliding friction plus rotor inertia. Identified in the teleop's own
workspace by an iterative, task-aware design, it removes both workarounds the
current sensorless 4C needed: dropping Coulomb friction, which chattered at
rest, and scaling the acceleration to 25 %.

## What is specific to the WAM

* **Zero-backlash cable capstans, motor-side encoders.** One count is
  2π/4096 rad at the motor, 36.5 µrad at J1 (ratio 42) and 85 µrad at J4 (18).
  The friction's transition after a reversal spans about 2–4 mrad at J1,
  roughly 100 counts, so it is resolved. With gear backlash or flexspline
  hysteresis it would be buried.
* **Rate independence (leader J1, excitation 2026-09-25).** Friction torque
  after reversals, binned by displacement since the reversal, collapses for
  slow and fast reversals. Binned by time since the reversal, it does not
  (`python -m cable_ident hysteresis --arm leader`):

  | displacement since reversal (mrad) | <0.5 | 0.5–1 | 1–2 | 2–4 | 4–8 |
  |---|---|---|---|---|---|
  | slow reversals (Nm) | 0.61 | 1.06 | 1.53 | 2.00 | 2.24 |
  | fast reversals (Nm) | 0.63 | 0.89 | 1.54 | 2.04 | 2.07 |

  | time since reversal (s) | <0.02 | 0.02–0.05 | 0.05–0.1 |
  |---|---|---|---|
  | slow reversals (Nm) | −0.88 | −0.22 | 0.94 |
  | fast reversals (Nm) | −0.22 | 0.30 | 1.11 |

* **Physical reading.** The mrad-scale transition is too long for bearing
  presliding (µrad). It looks like cable stretch against friction until the
  slider slips, i.e. Jenkins elements, with stiffness about F/δ ≈ 10³ Nm/rad
  at J1. This should be checked directly; see "Risks".
* **The differential.** Motors 2 and 3 drive J2 and J3 together
  (`m2 = 28.25 q2 − 16.8 q3`, `m3 = −28.25 q2 − 16.8 q3`). Friction lives at
  the motors, so J2's friction depends on J3's motion. The elements are
  therefore placed in motor coordinates (x = TM q), plus a joint-side bank on
  J2/J3.

## Model (`src/lib/cable_drive_model.h`)

```
tau = W(q, dq, ddq) beta                    rigid body, 22 base parameters, physically consistent
    + TM^T diag(I) TM ddq                    motor rotors
    + TM^T f(TM q) + f_J2,J3(q)              Jenkins elements: z <- clamp(z + dx, ±delta_i),
                                              f = sum_i w_i z_i / delta_i, w_i >= 0
    + TM^T diag(v) TM dq                     viscous, at the motors
    + offset                                 per joint
    + g_lb(q) − g_lb(q_wrist = 0)            the wrist's gravity, from libbarrett's calibration
```

Slip displacements: 5–170 counts at the motor (fixed grid; the smallest is 5
counts, so a joint toggling one count at rest moves the estimate by 0.1–0.4 Nm
instead of flipping it). The elements are driven by encoder increments, so
there is no velocity sign to chatter.

## Identification (`ident/cable_ident/ident.py`)

```
min  Σ_j w_j ||G_j θ − y_j||² / N + Σ_i λ_i D_logdet(J_i, J_i^CAD) + s Σ (w_k − w_k+1)²
s.t. J_i(δ_i) ⪰ ε I  (links 1–4),   w ≥ 0, v ≥ 0, I ≥ 0
```

* One SDP (Clarabel; SCS fallback). The pseudo-inertia LMIs (Wensing et al.
  RA-L 2017) make the rigid body realizable. Nonnegative element forces make
  the friction passive. The log-det divergence to the WAM CAD model (Lee,
  Wensing & Park, T-RO 2020) regularizes links 1–3; link 4 carries the wrist,
  so it gets no prior.
* **Fitted through the controller's own signal chain:** libbarrett's
  velocity, the controller's 25/50 rad/s acceleration filter at full scale,
  elements driven by encoder increments, and a 10 Hz causal filter on both
  sides. The identified model is the one that runs at 500 Hz, and the C++ is
  checked against it to 10⁻⁷ Nm (`test/test_cable_drive_model.cpp`).
* **Estimate and feedforward are separate.** The external-torque estimate
  subtracts every term. The feedforward takes a tunable share of the friction
  and of the inertia (`--ff-friction`, `--ff-inertia`, key `m`), so
  transparency can be traded against passivity explicitly.

## Iterative, task-aware data (`ident/cable_ident/design.py`, `data.py`)

* **The excitation misses the teleop.** Teleop runs at J2 ∈ [−1.98, −1.33]
  and J4 ∈ [1.2, 2.7] (5th–95th percentile, wam8g session of 2026-09-29),
  while the excitation covers J2 ∈ ±1.04: 0 % overlap in J2, and a median
  distance of 0.7–0.8 rad to the nearest excitation pose. The teleop is also
  nearly static: the 95th-percentile joint speed is 0.04 rad/s, against a
  median of 0.5 rad/s in the excitation.
* **Free task-region data.** Every session begins with each arm driving
  itself to the sync pose and holding: free motion in the teleop workspace,
  with no operator and no contact. 12 follower sessions give 540 s; 6 leader
  sessions give 62 s. These segments are training data and the held-out test.
* **Sequential design.** Each round's Fourier trajectory maximizes
  log det(F_now + F_new) over everything fitted so far. It is constrained to
  stay within 0.12 rad of configurations the arm visited in teleop (the
  operator already drove it there), with joint speed ≤ 0.6 rad/s. Reversal
  ladders follow: amplitudes from 16 mrad down to 0.3 mrad, then slow sweeps
  with holds. Round 1 is designed (`ident/trajectories/*_round1.txt`): 285 s
  per arm, information gain 1.4 nats (follower) and 4.5 nats (leader). Stop
  when another round no longer lowers the held-out error.

## Evidence so far (offline, held-out)

Leave-one-session-out: fit on the excitation (plus, where stated, the other
sessions), score a session's self-driven segments. RMS of the
external-torque estimate in free motion, J1–J4, Nm, at a 10 Hz bandwidth
(`python -m cable_ident ablate --arm ... --exc-weight 0.25`):

| model | follower (12 sessions) | leader (6 sessions) |
|---|---|---|
| viscous only, inertia ×0.25, OLS (the deployed models; prior WAM work) | 3.07 0.93 2.00 0.63 | 1.96 1.34 2.02 0.73 |
| Coulomb tanh, LMI (Sousa & Cortesão) | 2.84 0.56 1.39 0.56 | 1.62 0.83 1.53 0.61 |
| joint-space Jenkins, LMI | 1.46 1.61 0.95 0.39 | 1.16 1.32 1.23 0.63 |
| motor-space Jenkins, LMI | 1.67 0.97 0.73 0.43 | 1.18 1.28 0.87 0.70 |
| cable (motor + J2/J3 Jenkins), LMI | 1.67 1.22 0.31 0.43 | 1.18 1.31 0.65 0.68 |
| cable, OLS (no LMI, no sign constraints) | 1.73 1.46 0.38 0.44 | 1.18 1.64 0.70 0.64 |
| cable, no inertia in the model | 1.75 1.61 1.29 0.40 | 1.17 1.60 1.49 0.61 |
| **cable + teleop-region data (proposed)** | **0.76 0.50 0.24 0.28** | **1.19 0.96 0.63 0.53** |

* **Follower:** 4.0×, 1.9×, 8.3× and 2.3× lower than the deployed model, and
  the per-session bias drops from (2.6, 0.6, 1.9, 0.4) Nm to 0.11 Nm or less.
* **The ablations agree with the physics:**
  * motor coordinates fix J3 (0.95 → 0.31 Nm);
  * inertia matters (J3 1.29 → 0.31 Nm);
  * the LMIs help extrapolation (J2 1.46 → 1.22 Nm);
  * the teleop-region data removes the extrapolation bias (J2 1.22 → 0.50 Nm).
* **Leader:** improves less (6 short segments). It is the arm the designed
  round must fix.
* **Wrist gravity:** during teleop the follower's wrist pose shifts J2/J3
  gravity by 0.24/0.27 Nm RMS, up to 1.2 Nm. The self-driven segments hold the
  wrist still, so this table cannot show the correction; the contact
  experiments below must.

## Status

Done and tested offline:

* the model and estimator (C++, both arms);
* the identification tool;
* the parameter headers fitted on all data that matches each arm's
  calibration;
* trajectory playback with full logging in `dynamics_excitation`;
* round-1 designs.

Not yet done:

* nothing has run on the robot;
* no closed-loop teleop with `--model cable`;
* no force ground truth.

## Experiments the paper needs

1. **Rounds 1–3 of the designed excitation** on both arms. Report the
   held-out learning curve per round (the iteration's main figure).
2. **Ground truth.** Known payloads hung at the follower's tool, moved through
   the teleop workspace (τ_ext = Jᵀ m g), and an F/T sensor at the follower's
   end effector for contact tasks (door, compression). Report free motion,
   reversals, low speed and static contact separately.
3. **Clamped-link test** for the physics claim. Lock the link and ramp motor
   torque: motor displacement against torque gives the cable stiffness and the
   element law directly. This separates cable compliance from bearing
   presliding (cf. Ruderman, TIE 2009).
4. **4C performance:**
   * transparency, as force tracking leader↔follower against the F/T sensor;
   * stability, as the largest stable force gain and resonance, with the
     legacy and cable models;
   * the feedforward-share sweep (friction 0 → 0.8).
5. **Baselines:**
   * LuGre/GMS fitted nonconvexly;
   * a GP or LSTM residual on the same data, tested out of distribution;
   * the momentum observer. Offline, it is noise-limited above about 3 Hz by
     encoder quantization (ε·K·M), which is why the controller's acceleration
     chain is kept.
6. **Drift:** per-session refit of only the friction and offsets with
   projected RLS, from the session's own self-driven segment.

## Risks

* **Cable compliance vs. friction.** The Jenkins reading is a hypothesis. The
  clamped-link test decides it. Either answer keeps the convex model; it
  changes the story.
* **Stiction band at rest.** When a joint is stuck at its joint side, an
  external torque below breakaway does not reach the motor. No proprioceptive
  method sees it. Quantify it in the payload tests. Dither, exploiting the
  backlash-free drive, is a possible extension.
* **Calibration changes.**
  * Each zerocal or gravitycal starts a new dataset (`arms.py`).
  * The leader's zerocal changed on 2026-09-28, between its excitation
    (09-25) and five of its six sessions.
* **Same trajectory.** All existing excitations are one trajectory. The
  held-out test therefore relies on the self-driven segments, which are slow
  moves and holds, until the designed rounds exist.

## Positioning

From an automated literature search; verify before citing.

* **(a) Rigid body + presliding friction in one convex program: not found.**
  * Wang et al., RA-L 2019 (dVRK, convex physically consistent SDP) state
    that Dahl-type presliding friction cannot be used in their convex
    framework.
  * Zhang et al., MSSP 2025 combine PI-type hysteresis with feasibility
    constraints using iteratively reweighted least squares, not one convex
    program, and on an elastic-joint robot.
  * Maxwell-slip / PI with fixed thresholds being linear in its weights is
    known (Rizos & Fassois 2005; Kuhnen 2003). The combination, its passivity
    by construction, and its observability argument for cable drives are the
    contribution.
* **Task-weighted, log-driven excitation design:** not found for manipulator
  identification. Related: Wilson T-RO 2014; application-oriented design,
  Annergren CSM 2017; Ross ICML 2012.
* **Known, so used as ablations rather than claims:**
  * motor-space friction for coupled joints (Gautier & Venture IROS 2013;
    Wang RA-L 2019);
  * acceleration-free and observer-consistent identification (Gautier's
    energy/power models);
  * per-session friction recalibration (Iskandar ICRA 2019).
* **Sensorless 4C itself:** Yilmaz et al., RA-L 2024 (dVRK, learned dynamics
  plus disturbance observer); Yamane et al., arXiv 2025.
* **The group's own WAM work** (Noohian et al., Frontiers 2026; arXiv
  2607.01201) used viscous-only friction and 25 % acceleration. State the
  difference explicitly (and cite in the third person: RSS is double blind).
