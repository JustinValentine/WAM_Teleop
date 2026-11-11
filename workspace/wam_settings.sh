# Settings for ./wam (the teleop and the haptic saber). Change them here; the
# next `wam teleop` or `wam saber` uses them.

# odin: this checkout's own container (docker-compose.yml here), not the
# wam_ros container of /home/serg/projects/wam-ros-docker.
CONTAINER=wam_ros_4c

# J6 friction feedforward on the wam8g, N m, for both demos. wam8g_j6_friction
# measures J6's breakaway; use about half of it (0.59 Nm measured 2026-09-24).
J6_FRICTION=0.20   # 0.30 until 2026-09-28

# Teleop (IROS demo settings, 2026-09-25)
# force gain: 0.3 at IROS and in the first runs on odin (stable); 0.5 on 2026-11-11 15:07 grew a
# 7.6 Hz swing of J2 and J4 on both arms within 24 s until the leader faulted; 0.4 from then on.
LEADER_ARGS="--law u4 --force-gain 0.4 --force-notch 35 --wrist --j6-friction $J6_FRICTION"
# odin, 2026-11-11: the follower is slax with the skid gripper (no BarrettHand, so no --hand)
# and its own configuration, a copy of Serg's (config/follower-slax-skid-ft/ORIGIN.txt).
FOLLOWER_ARGS="--sync wam8g --law u4 --force-gain 0.4 --force-notch 35"
FOLLOWER_CONF=/home/user/wam_ros/wam_ws/src/wam_teleop/config/follower-slax-skid-ft/follower.conf
HAND_CONTROLLER=no    # the Sony navigation controller -> hand chain (yes on thor; no hand on slax)
SPREAD_POSES=yes      # L2 + X steps the spread 0/50/100%; no: the grasp only, spread at 0%
# Recording, in teleop_logs/ (2026-11-09):
#   native  each arm program writes teleop_<time>_<arm>.tlog itself, one row per
#           control cycle, from its control loop: no ROS in the data path, and the
#           500 Hz ROS publisher threads (which take the control loop's lock) are
#           not started. `wam log` summarizes it; scripts/teleop_log.py reads it.
#   bag     the rosbag of /leader|/follower joint_states and gravity, as before
#   both    both;  no: neither
RECORD=native

# The CPU each arm's control thread is pinned to (empty: not pinned). On thor
# and on odin (i9-14900K) CPUs 0-15 are the performance cores, two threads per
# core: 2 and 4 are on different cores.
LEADER_CPU=2
FOLLOWER_CPU=4

# The CAN interface of each arm, as bus.port in its configuration file says
# (config/wam8g/default.conf: 1; .../follower-zeus-hand/follower.conf: 0).
FOLLOWER_CAN=can0
LEADER_CAN=can1
# Frames each interface's transmit queue holds; `wam teleop` raises a shorter
# one (sudo). Linux's default of 10 is too few on a PCI/PCIe card (./wam, check_can).
CAN_TXQUEUELEN=100

# Control options added with the low-latency update, for either arm. Empty: the
# law as at IROS. They have not run on the robots yet: add one at a time, with
# the E-stop at hand (README, "Low latency and recording").
#   --reference age      the other arm's position advanced by each packet's age
#   --estimator matched  the external-torque estimate filtered consistently
#   --ki-scale 0         no integral action in the position coupling (J1-J4)
# The gripper's calibration for the follower's force/torque sensor (below).
FT_TOOL=wam_ws/src/wam_teleop/config/follower-slax-skid-ft/ft_tool.conf   # in this repository; `wam ft-cal` writes it
FT_TOOL_IN=/home/user/wam_ros/$FT_TOOL                                 # the same file inside the container
FT_ORIGIN_Z=""   # m from the tool plate to the sensor's measuring origin, if its drawing gives it
# 2026-11-11: the leader on the cable model (WAM 153's fit of that night), 0.2 of its friction in
# the feedforward; raise it live with the key m. First use in a linked teleop: "" goes back to legacy.
# --ki-scale 0 (2026-11-11, user): no integral action in J1-J4's position coupling, on both arms.
LEADER_EXTRA="--model cable --ff-friction 0.2 --ki-scale 0"
# 2026-11-11: the force the follower sends the leader for J1-J4 is the force/torque sensor's
# (--ft arm), with the gripper's weight removed by the calibration `wam ft-cal` wrote that day.
# First session with it: 16:08 that day. Back to recording the sensor only:
# FOLLOWER_EXTRA="--ft log --ki-scale 0".
FOLLOWER_EXTRA="--ft arm --ft-tool $FT_TOOL_IN --ki-scale 0"

# Force/torque sensor on the follower, and the wrist (wam_teleop/README.md,
# "Force/torque sensor and the wrist"). Not run on the robots yet: one step at
# a time, E-stop at hand, each step set in the two lines above.
#   1  FOLLOWER_EXTRA="--ft log"     read and record the sensor; nothing else changes.
#      Run a teleop with nothing touching the gripper, turn the wrist through
#      its range, pause in a few poses; then `wam ft-cal` fits the gripper.
#   2  LEADER_EXTRA="--wrist-deadband 3"   (no sensor needed) the leader's wrist is
#      not pulled toward the follower's within 3 degrees: is this the heaviness?
#   3  FOLLOWER_EXTRA="--ft wrist --ft-tool $FT_TOOL_IN"
#      LEADER_EXTRA="--wrist-deadband 3 --wrist-force 0.2"   contacts on the wrist
#      from the sensor; raise the gain with 'w' in the leader's pane.
#   4  FOLLOWER_EXTRA="--ft all --ft-tool $FT_TOOL_IN"   J1-J4's force channel from
#      the sensor too, instead of the follower's model estimate.
#   also  FOLLOWER_EXTRA="... --wrist-friction 0.1"   the follower's wrist helped
#      along, so it lags the leader's less (start low).

# Haptic saber on the wam8g (demos/
