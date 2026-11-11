/* The poses of `follower --ft-cal` (`wam ft-cal`): the arm goes to the wam8g
 * sync pose (lib/wam8g_teleop.h, the pose every teleop session links at) and
 * stays there with J1-J4 while the wrist alone turns the gripper through the
 * directions of gravity, pausing in each pose, so that scripts/ft_tool_cal.py
 * can fit the gripper beyond the force/torque sensor from a recording in
 * which nothing can have touched it.
 *
 * J5 rolls the forearm's end, J6 pitches the tool axis, J7 turns the gripper
 * about its own axis. The table stays well inside the P2 wrist's ranges
 * (J5 -4.76..1.24, J6 -1.57..1.57, J7 -3.0..3.0 rad): J5 within 0.9 rad of
 * the sync pose's -pi/2, J6 within 0.9 rad of 0, J7 within 1.0 rad of pi/2.
 * workspace/tools/ft_cal_check prints, for a follower configuration, how well
 * these poses spread gravity over the tool frame (0.58 in each of the three
 * directions is ideal; the fit needs 0.15 in the weakest). */
#ifndef FT_CAL_POSES_H_
#define FT_CAL_POSES_H_

namespace ft_cal {

const int POSES = 14;

/* J5, J6, J7 (rad), in the order they are visited: neighbours differ by one
 * joint where that is possible, so the moves are short. */
const double WRIST[POSES][3] = {
    {-1.571, 0.0, 1.571},   // the sync pose's wrist
    {-2.471, 0.0, 1.571},
    {-2.471, 0.9, 1.571},
    {-1.571, 0.9, 1.571},
    {-0.671, 0.9, 1.571},
    {-0.671, 0.0, 1.571},
    {-0.671, -0.9, 1.571},
    {-1.571, -0.9, 1.571},
    {-2.471, -0.9, 1.571},
    {-2.471, -0.9, 0.571},
    {-1.571, 0.0, 0.571},
    {-0.671, 0.9, 0.571},
    {-0.671, 0.9, 2.571},
    {-1.571, 0.0, 2.571},
};

const double HOLD_SECONDS = 2.5;   // at rest in each pose (the fit uses what is still)
const double MOVE_VELOCITY = 0.5;  // rad/s
const double MOVE_ACCELERATION = 0.5;  // rad/s^2

}  // namespace ft_cal

#endif
