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
# Robotiq 2F gripper from the same navigation controller (wam_teleop/README.md, "Robotiq 2F
# gripper"). robotiq: `wam teleop` starts a gripper pane; L2 + stick up/down closes/opens,
# L2 + X opens fully, L2 + circle held 1 s restarts the gripper, as does `wam gripper restart`
# from any terminal (also its first activation after it was powered on); `wam gripper status`
# shows its state. no: nothing changes.
# Not together with HAND_CONTROLLER=yes: there is one controller.
# odin, 2026-11-12: on, first use.
GRIPPER=robotiq
# The gripper's serial adapter, as the container sees it. A name under /dev/serial/by-id/ stays
# the same when the adapter is plugged in again; /dev/ttyUSB0 can come back as /dev/ttyUSB1.
# odin, 2026-11-12: the gripper's adapter by its stable name (it is /dev/ttyUSB0 today).
GRIPPER_PORT=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
# More arguments for launch/robotiq_nav_controller.launch. "auto_activate:=true": activate the
# gripper when the pane starts, without waiting for L2 + circle (its fingers move, with the arm
# wherever it is then). "auto_restart:=true": restart without asking whenever the gripper reports
# a fault that needs it (it then drops what it holds by itself).
# odin, 2026-11-12: the joystick device itself. It is js0 whether the controller is on its USB
# cable or on Bluetooth (the only joystick on this computer), and joy_node opens it again by
# itself when the controller changes over. /dev/nav_controller (the launch file's default) only
# exists over Bluetooth here: on the cable the kernel names the controller differently and the
# installed udev rule does not link it.
GRIPPER_ARGS="joy_dev:=/dev/input/js0"
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
# 2026-11-11, for the run after the first --ft arm session. One new thing at the start, two by key:
#   --reference age (both arms; first use on the arms): each arm advances the other's position by
#     the packet's age, which removes the coupling's drag of kp * age (in the 15:15 session
#     1.0 / 1.2 / 0.4 / 0.3 Nm on the leader's J1-J4 at 0.5 rad/s, 0.7 / 0.8 / 0.3 / 0.2 on the
#     follower's) and its change from one start to the next. Without it: drop it from both lines.
#   the leader's wrist force from the follower's sensor (the follower's --ft all sends it): starts
#     at gain 0; in the leader's pane  w  then  0.2 0  switches it on. First use on the arms.
#   the leader's friction share: in the leader's pane  m  then  0.3 0  (0.4, 0.5), arm at rest.
# The settings of the first --ft arm session: wam_settings.sh.bak-20261111-ft-arm.
LEADER_EXTRA="--model cable --ff-friction 0.2 --ki-scale 0 --reference age"
# 2026-11-11: the force the follower sends the leader for J1-J4 is the force/torque sensor's
# (--ft arm), with the gripper's weight removed by the calibration `wam ft-cal` wrote that day.
# First session with it: 16:08 that day. Back to recording the sensor only:
# FOLLOWER_EXTRA="--ft log --ki-scale 0".
# 2026-11-13: the follower on the cable-drive model fitted on its own excitation runs of
# 2026-11-12 (--model cable, lib/cable_params_slax_20261112.h), feeding forward 0.4 of its cable
# friction, so that pushing it by a link takes less force (the 30-parameter model it ran until
# now compensates no such friction). NOT RUN ON THE ARM YET. In the follower's pane  m  then
# 0.5 0.25  raises the share (arm at rest; not above 0.5),  m  then  0 0.25  switches it off.
# Back to the 30-parameter model: remove the last three options (--model cable and the two
# shares), or restore wam_settings.sh.bak-20261113-pre-follower-cable.
FOLLOWER_EXTRA="--ft all --ft-tool $FT_TOOL_IN --ki-scale 0 --reference age --model cable --ff-friction 0.4 --ff-inertia 0.25"

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
