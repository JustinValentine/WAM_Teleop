#pragma once

/* The wam8g (eight joints: J1-J4 WAM arm, passive pinned J5, Alberta wrist
 * J6-J8) as the teleop leader for a seven-joint P2 WAM follower. Shared by
 * leader_wam8g, follower (--sync wam8g) and dynamics_excitation_wam8g. */

namespace wam8g_teleop {

/* Sync pose (rad), J1-J8: the haptic_saber start pose, set by the user
 * 2026-09-25. J5 is passive and never commanded (Wam8g keeps its gains at
 * zero); its nominal target is zero. */
const double SYNC_POSE[8] = {
    1.5707963267948966, -1.5707963267948966, -1.5707963267948966,
    1.5707963267948966, 0.0, 0.0, 0.0, 0.0
};

/* The follower's sync pose: the same J1-J4, and the wrist at J5 -pi/2, J6 0,
 * J7 +pi/2 (2026-09-26). With J6 at 0, J5 and J7 turn about the same axis, so
 * the hand points as with the wrist at zero; J5's quarter turn lines the P2's
 * J6 axis up with the wam8g's J7 axis, which it follows (lib/wam8g_wrist_map.h):
 * at zero they were 90 degrees apart. The follower moves here before linking
 * (J5 and J7 together, so the hand does not turn on the way), and the wrist map
 * works relative to this pose, so linking does not jump. */
const double FOLLOWER_SYNC_POSE[7] = {
    SYNC_POSE[0], SYNC_POSE[1], SYNC_POSE[2], SYNC_POSE[3],
    -1.5707963267948966, 0.0, 1.5707963267948966
};

/* Teleop position gains for J1-J4 (the position controller is the coupling
 * between the arms): the Ares leader's, config/leader-ares-link/wam4.conf,
 * except two kd tuned live on 2026-09-24 in 4C: J4's 1.2 instead of 0.8 (as
 * on the follower), against a 5 Hz swing of the two elbows; J1's 8.3 instead
 * of 10 (the follower's), against a grainy J1.
 * J5-J8 keep the robot's configured gains (config/wam8g/wam8g.conf). */
const double ARES_KP[4] = {750, 1000, 400, 200};
const double ARES_KI[4] = {2.5, 1, 2.5, 0.5};
const double ARES_KD[4] = {8.3, 6, 3.3, 1.2};
const double ARES_CONTROL_SIGNAL_LIMIT[4] = {50, 80, 50, 30};
const double ARES_INTEGRATOR_LIMIT[4] = {20, 10, 20, 60};

}  // namespace wam8g_teleop
