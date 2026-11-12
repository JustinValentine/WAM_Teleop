#!/bin/bash
# Everything about the Robotiq gripper's teleop that can be checked without a
# gripper, in Docker, in one go:
#
#   test/robotiq_docker.sh /path/to/a/clone/of/libgripper [quick]
#
# The clone (github dmiller12/libgripper) needs its branches main and robotiq;
# nothing is changed in it. In a container of the image robotiq-testenv
# (wam-teleop-testenv, the image the package's other tests run in, plus joy
# and tmux; built here if it is missing) the package is built three times, in
# the workspace path the robot computer's container has:
#   A  no libgripper installed
#   B  libgripper's main installed (no Robotiq classes), gripper_ros_common in
#      the workspace
#   C  libgripper's robotiq installed over it, as on the robot computer
# each time on top of the build before, as catkin_make would be run there
# after installing a library. After each: which programs there are, and the
# package's hardware-free tests. In A and B robotiq_teleop must be missing and
# everything else there; in C it must be there too, and then
# test/robotiq_e2e.sh (the node against the simulated gripper) and
# test/robotiq_wam_check.sh (the launcher) run. "quick" does C only.
#
# The last line is "robotiq_docker: all passed" or what failed; the exit
# status says the same. About half an hour for all three, the builds taking
# most of it (three compilers at a time: JOBS=n changes that; each of the arm
# programs' sources needs about 2 GB to compile).

IMAGE=robotiq-testenv
BASE=wam-teleop-testenv
WS=/home/user/wam_ros/wam_ws
TESTS="low_latency_tests wam8g_teleop_tests cable_model_tests wam8g_loopback_tests wam8g_loopback_tests_legacy hand_teleop_tests ft_bus_tests robotiq_teleop_tests"

if [ "${1:-}" != --inside ]; then
	clone=${1:-}
	mode=${2:-all}
	if [ -z "$clone" ] || ! git -C "$clone" rev-parse --verify -q robotiq >/dev/null || ! git -C "$clone" rev-parse --verify -q main >/dev/null; then
		echo "usage: $0 /path/to/libgripper [quick]     (a clone with the branches main and robotiq)"
		exit 2
	fi
	package=$(cd "$(dirname "$0")/.." && pwd)
	if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
		docker image inspect "$BASE" >/dev/null 2>&1 || { echo "The image $BASE is missing (its Dockerfile: testenv/ of the teleop's test environment)."; exit 2; }
		echo "Building the image $IMAGE ($BASE + ros-noetic-joy, tmux)"
		docker build -t "$IMAGE" - <<-DOCKERFILE || exit 2
			FROM $BASE
			ARG DEBIAN_FRONTEND=noninteractive
			RUN apt-get update && apt-get install -y --no-install-recommends ros-noetic-joy tmux psmisc && rm -rf /var/lib/apt/lists/*
		DOCKERFILE
	fi
	# Copies of the package as it is now and of the two branches: what is
	# edited meanwhile does not reach the run, and nothing here is written to.
	copies=$(mktemp -d)
	mkdir "$copies/wam_teleop" "$copies/main" "$copies/robotiq"
	rsync -a --exclude .git "$package/" "$copies/wam_teleop/"
	git -C "$clone" archive main | tar -x -C "$copies/main"
	git -C "$clone" archive robotiq | tar -x -C "$copies/robotiq"
	echo "libgripper main: $(git -C "$clone" rev-parse --short main)   robotiq: $(git -C "$clone" rev-parse --short robotiq)"
	docker rm -f robotiq-test-run >/dev/null 2>&1
	docker run --rm --name robotiq-test-run -e JOBS="${JOBS:-3}" -v "$copies/wam_teleop":/src/wam_teleop:ro -v "$copies":/src/libgripper:ro \
		"$IMAGE" bash /src/wam_teleop/test/robotiq_docker.sh --inside "$mode"
	rc=$?
	rm -rf "$copies"
	exit $rc
fi

# ---- inside the container ---------------------------------------------------
mode=${2:-all}
failed=""
source /opt/ros/noetic/setup.bash
mkdir -p "$WS/src"

install_libgripper() {  # as the robot computer's build_gripper.sh: cmake, make, make install into /usr/local
	rm -rf /root/libgripper && cp -r "/src/libgripper/$1" /root/libgripper
	(cd /root/libgripper/libgripper && mkdir -p build && cd build && cmake .. >/dev/null && make -j4 gripper 2>&1 | grep -E "error|\*\*\*" ; make install >/dev/null 2>&1) &&
		ldconfig && rm -rf "$WS/src/gripper_ros_common" && cp -r /root/libgripper/gripper_ros_common "$WS/src/gripper_ros_common"
}

build() {  # build NAME: the package as mounted, on top of the build before
	rsync -a --delete --exclude .git /src/wam_teleop/ "$WS/src/wam_teleop/"
	(cd "$WS" && catkin_make -DBUILD_LEADER=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo -j"${JOBS:-3}") > "/tmp/build_$1.log" 2>&1
	local rc=$?
	grep -E "^-- (follower: (--hand|no gripper)|robotiq_teleop:)" "/tmp/build_$1.log" | sed 's/^/  /'
	grep -E "error|Error 1|\*\*\*|warning: " "/tmp/build_$1.log" | head -n 20
	echo "  catkin_make: exit $rc; programs: $(ls "$WS/devel/lib/wam_teleop" | tr '\n' ' ')"
	[ $rc -eq 0 ] || failed="$failed build-$1"
}

tests() {  # tests NAME
	local t
	for t in $TESTS; do
		(cd "$WS/devel/lib/wam_teleop" && "./$t") > "/tmp/${t}_$1.log" 2>&1
		local rc=$?
		echo "  $t: $(grep -E "checks, |all passed|passed" "/tmp/${t}_$1.log" | tail -n 1)   exit $rc"
		[ $rc -eq 0 ] || failed="$failed $t-$1"
	done
}

expect() {  # expect NAME "what" command...
	local name=$1 what=$2
	shift 2
	if "$@"; then
		echo "  ok   $what"
	else
		echo "  FAIL $what"
		failed="$failed $name"
	fi
}

if [ "$mode" != quick ]; then
	echo "A. No libgripper installed"
	build A
	expect A-programs "no robotiq_teleop; follower, leader_wam8g and the core's tests are there" \
		eval '[ ! -e $WS/devel/lib/wam_teleop/robotiq_teleop ] && [ -x $WS/devel/lib/wam_teleop/follower ] && [ -x $WS/devel/lib/wam_teleop/leader_wam8g ] && [ -x $WS/devel/lib/wam_teleop/robotiq_teleop_tests ]'
	tests A

	echo "B. libgripper main installed (no Robotiq classes)"
	install_libgripper main
	build B
	expect B-programs "no robotiq_teleop; the follower has --hand, hand_chain_tests is there" \
		eval '[ ! -e $WS/devel/lib/wam_teleop/robotiq_teleop ] && grep -q "follower: --hand available" /tmp/build_B.log && [ -x $WS/devel/lib/wam_teleop/hand_chain_tests ]'
	tests B
fi

echo "C. libgripper robotiq installed"
install_libgripper robotiq
build C
expect C-programs "robotiq_teleop is there, with everything else" \
	eval '[ -x $WS/devel/lib/wam_teleop/robotiq_teleop ] && [ -x $WS/devel/lib/wam_teleop/follower ] && [ -x $WS/devel/lib/wam_teleop/hand_chain_tests ]'
tests C

source "$WS/devel/setup.bash"
echo "The node against the simulated gripper (test/robotiq_e2e.sh)"
bash "$WS/src/wam_teleop/test/robotiq_e2e.sh" > /tmp/robotiq_e2e.log 2>&1 || failed="$failed robotiq_e2e"
grep -E "^[0-9]+\. |^  FAIL|checks, " /tmp/robotiq_e2e.log | sed 's/^/  /'
echo "The launcher (test/robotiq_wam_check.sh)"
bash "$WS/src/wam_teleop/test/robotiq_wam_check.sh" > /tmp/robotiq_wam_check.log 2>&1 || failed="$failed robotiq_wam_check"
grep -E "^[0-9]+\. |^  FAIL|checks, " /tmp/robotiq_wam_check.log | sed 's/^/  /'

if [ -n "$failed" ]; then
	echo "robotiq_docker: FAILED:$failed"
	for f in /tmp/robotiq_e2e.log /tmp/robotiq_wam_check.log; do
		grep -q "^  FAIL" "$f" 2>/dev/null && { echo "---- $f"; cat "$f"; }
	done
	exit 1
fi
echo "robotiq_docker: all passed"
