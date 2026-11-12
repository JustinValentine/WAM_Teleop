#!/bin/bash
# The Robotiq gripper's teleop without a gripper or a controller: first the
# simulated gripper against the manual's own Modbus frames (robotiq_sim.py
# --selftest), then the real node and launch file against it
# (robotiq_e2e.py). Run it in a sourced workspace that has robotiq_teleop
# (built with libgripper's branch robotiq) and the joy package. It starts
# roscore if none runs. Exit status 0: nothing failed.
here=$(cd "$(dirname "$0")" && pwd)

echo "The simulated gripper against the manual's example frames"
python3 "$here/robotiq_sim.py" --selftest || exit 1

if [ -z "$(catkin_find wam_teleop robotiq_teleop 2>/dev/null)" ]; then
	echo "robotiq_teleop is not in this workspace: it is only built when the installed libgripper has the"
	echo "Robotiq classes (branch robotiq), and the workspace has to be sourced (devel/setup.bash)."
	exit 1
fi
if ! rospack find joy >/dev/null 2>&1; then
	echo "The joy package is missing (the launch file's joy_node): apt install ros-noetic-joy."
	exit 1
fi

core=
if ! rostopic list >/dev/null 2>&1; then
	roscore >/tmp/robotiq_e2e_roscore.log 2>&1 &
	core=$!
	for _ in $(seq 60); do
		rostopic list >/dev/null 2>&1 && break
		sleep 0.5
	done
fi
echo "The node and the launch file against the simulated gripper"
python3 "$here/robotiq_e2e.py"
rc=$?
[ -n "$core" ] && kill -INT "$core" && wait "$core" 2>/dev/null
exit $rc
