/* The poses of `follower --ft-cal` (`wam ft-cal`): the arm goes to the wam8g
 * sync pose (lib/wam8g_teleop.h, the pose every teleop session links at) and
 * stays there with J1-J4 while the wrist alone turns the gripper through the
 * directions of gravity, pausing in each pose, so that scripts/ft_tool_cal.py
 * can fit the gripper beyond the force/torque sensor from a recording in
 * which nothing can have touched it.
 *
 * J5 rolls the forearm's end, J6 pitches the tool axis, J7 turns the gripper
 * about its own axis. J7 stays at the sync pose's angle in the poses the fit
 * uses (FITTED of them): on slax, 2026-11-11, turning J7 loaded the sensor by
 * itself (whatever crosses J7 to the gripper winds up: 5 N one radian beyond
 * the sync angle, 1 to 2 N one radian short of it), which is no part of the
 * gripper's weight. J5 does what J7 would have done for the fit: with J6 at 0
 * the two turn about the same axis. Then come two poses with J7 0.35 rad
 * either way, about as far as the teleop turns it (the wam8g's J8 between its
 * stops), which the fit script compares with the fit to show that load, and
 * the first pose again (the sensor's drift over the run).
 *
 * The table stays inside what the teleop itself reached on 2026-11-11 (J5
 * -2.21 to -0.05, J6 -0.64 to 1.50, J7 1.09 to 1.98 rad) except J5 -2.47 and
 * J6 -0.9, which the first calibration run visited, and well inside the P2
 * wrist's ranges (J5 -4.76..1.24, J6 -1.57..1.57, J7 -3.0..3.0 rad).
 * workspace/tools/ft_cal_check prints, for a follower configuration, how well
 * the fitted poses spread gravity over the tool frame (0.58 in each of the
 * three directions is ideal; with J7 fixed and the forearm level the weakest
 * is 0.23; the fit needs 0.15). */
#ifndef FT_CAL_POSES_H_
#define FT_CAL_POSES_H_

namespace ft_cal {

const int POSES = 15;
const int FITTED = 12;  // the first of them, all with J7 at the sync pose's angle

/* J5, J6, J7 (rad), in the order they are visited: neighbours differ by one
 * joint where that is possible, so the moves are short. */
const double WRIST[POSES][3] = {
    {-1.571, 0.0, 1.571},   // the sync pose's wrist
    {-2.471, 0.0, 1.571},
    {-2.471, 0.9, 1.571},
    {-1.571, 0.9, 1.571},
    {-0.671, 0.9, 1.571},
    {-0.071, 0.9, 1.571},
    {-0.071, 0.0, 1.571},
    {-0.071, -0.9, 1.571},
    {-0.671, -0.9, 1.571},
    {-1.571, -0.9, 1.571},
    {-2.471, -0.9, 1.571},
    {-0.671, 0.0, 1.571},
    {-1.571, 0.0, 1.221},   // J7 alone, one way
    {-1.571, 0.0, 1.921},   // and the other
    {-1.571, 0.0, 1.571},   // the first pose again
};

const double HOLD_SECONDS = 2.5;   // at rest in each pose (the fit uses what is still)
const double MOVE_VELOCITY = 0.5;  // rad/s
const double MOVE_ACCELERATION = 0.5;  // rad/s^2

}  // namespace ft_cal

#endif
