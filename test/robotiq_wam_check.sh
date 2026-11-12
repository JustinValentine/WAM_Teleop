#!/bin/bash
# The launcher's gripper support (workspace/wam with GRIPPER=robotiq) without
# robots, a controller or Docker: the real script, its real settings file with
# a few lines appended, real tmux panes, the real launch file and node, and
# the simulated gripper of robotiq_sim.py.
#
# The launcher normally runs on the robot computer and reaches the programs
# with `docker exec` into the workspace's container. Here everything is in one
# place, so stand-ins on the PATH take the place of what is missing:
#   docker  `docker exec [-it] NAME command` runs the command here; the
#           container always counts as running
#   ip, sudo  the CAN interfaces look up and configured
#   tmux    the real one; where it is older than 3.1 (Ubuntu 20.04's), the
#           launcher's pane sizes "-l 25%" are passed on as "-p 25"
# and two idle processes carry the arm programs' names, to see that the
# gripper commands leave them alone and that `wam stop` still ends everything.
#
# Needs: the workspace at /home/user/wam_ros/wam_ws (the path the launcher has
# for it inside the container), built with robotiq_teleop; tmux; the joy
# package. test/robotiq_docker.sh sets that up and runs this.
#
# What it goes through: the refusal of HAND_CONTROLLER=yes with
# GRIPPER=robotiq and of an unknown GRIPPER; GRIPPER=no (no gripper pane);
# the gripper pane with `wam teleop`; wam status; wam gripper status;
# wam gripper restart as the gripper's first activation, with the node well,
# with the serial device gone, with the node hung and with its launch file
# ended; wam stop.

IN=/home/user/wam_ros
BIN=$IN/wam_ws/devel/lib/wam_teleop
here=$(cd "$(dirname "$0")" && pwd)
package=$(dirname "$here")
PORT=/tmp/robotiq_wam_port
work=$(mktemp -d)
checks=0
failures=0

check() {  # check "what" command...: ok when the command succeeds
	local what=$1
	shift
	checks=$((checks + 1))
	if "$@"; then
		echo "  ok   $what"
	else
		failures=$((failures + 1))
		echo "  FAIL $what"
	fi
}
has() { grep -qF -- "$2" <<<"$1"; }            # has "$text" "words"
hasnt() { ! grep -qF -- "$2" <<<"$1"; }
wait_for() {  # wait_for seconds command...
	local n=$(($1 * 5))
	shift
	for _ in $(seq $n); do
		"$@" >/dev/null 2>&1 && return 0
		sleep 0.2
	done
	return 1
}
indent() { sed 's/^/       | /'; }

if [ ! -x "$BIN/robotiq_teleop" ]; then
	echo "No $BIN/robotiq_teleop: build the workspace there, with libgripper's branch robotiq installed."
	exit 1
fi
command -v tmux >/dev/null || { echo "tmux is missing."; exit 1; }

# ---- the stand-ins ----------------------------------------------------------
mkdir -p "$work/bin"
cat > "$work/bin/docker" <<'EOF'
#!/bin/bash
case "$1" in
inspect) echo true ;;
start) ;;
exec)
	shift
	while [[ "$1" == -* ]]; do shift; done
	shift  # the container's name
	exec "$@" ;;
esac
EOF
cat > "$work/bin/ip" <<'EOF'
#!/bin/bash
[ "$1" = -details ] || exit 0
echo "9: $4: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP mode DEFAULT group default qlen 100"
echo "    can state ERROR-ACTIVE restart-ms 0"
echo "	  bitrate 1000000 sample-point 0.750"
EOF
printf '#!/bin/bash\nexec "$@"\n' > "$work/bin/sudo"
real_tmux=$(command -v tmux)
cat > "$work/bin/tmux" <<EOF
#!/bin/bash
args=()
while [ \$# -gt 0 ]; do
	if [ "\$1" = -l ] && [[ "\${2:-}" == *% ]]; then
		args+=(-p "\${2%\\%}")
		shift 2
	else
		args+=("\$1")
		shift
	fi
done
exec $real_tmux "\${args[@]}"
EOF
chmod +x "$work/bin"/*
export PATH=$work/bin:$PATH

settings() {  # settings HAND_CONTROLLER GRIPPER: the repository's file, then this test's lines
	cp "$package/workspace/wam_settings.sh" "$IN/wam_settings.sh"
	cat >> "$IN/wam_settings.sh" <<EOF

# test/robotiq_wam_check.sh
HAND_CONTROLLER=$1
GRIPPER=$2
GRIPPER_PORT=$PORT
RECORD=no
EOF
}
cp "$package/workspace/wam" "$IN/wam"
chmod +x "$IN/wam"
wam() { "$IN/wam" "$@" </dev/null 2>&1; }
panes() { tmux list-panes -s -t wam -F '#{@name}' 2>/dev/null | sort | tr '\n' ' '; }
pane_text() { cat "$IN"/pane_logs/*_gripper.log 2>/dev/null; }
node_pid() { pgrep -f "^$BIN/robotiq_teleop( |$)" | head -n 1; }
activations() { grep -c "activation started" "$work/sim.log"; }
sim() { echo "$*" > "$work/sim.in"; }

tmux kill-server 2>/dev/null
killall -q rosmaster roscore roslaunch robotiq_teleop joy_node 2>/dev/null
rm -rf "$IN/pane_logs"
mkfifo "$work/sim.in"
sleep 100000 > "$work/sim.in" &
keeper=$!
python3 -u "$here/robotiq_sim.py" --link "$PORT" < "$work/sim.in" > "$work/sim.log" 2>&1 &
simulator=$!
sleep 1

echo "1. Settings the launcher refuses"
settings yes robotiq
out=$(wam teleop)
echo "$out" | indent
check "HAND_CONTROLLER=yes with GRIPPER=robotiq: wam teleop says both want the one controller, and how to choose" \
	eval 'has "$out" "HAND_CONTROLLER=yes" && has "$out" "GRIPPER=robotiq" && has "$out" "one navigation controller" && has "$out" "Nothing was started"'
check "... and starts nothing" eval '! tmux has-session -t wam 2>/dev/null && ! pgrep -f rosmaster >/dev/null'
settings no robotic
out=$(wam teleop)
check "GRIPPER=robotic (a typing error): refused, nothing started" eval 'has "$out" "it is no (none) or robotiq" && ! tmux has-session -t wam 2>/dev/null'

echo "2. GRIPPER=no: as before"
settings no no
out=$(wam gripper restart)
check "wam gripper restart: $out" has "$out" "No gripper is set up"
wam teleop >/dev/null
wait_for 15 pgrep -f rosmaster
check "wam teleop: the panes are $(panes)" [ "$(panes)" = "follower leader roscore " ]
out=$(wam status)
check "wam status has no gripper lines" eval 'hasnt "$out" "ripper"'
out=$(wam stop)
check "wam stop: stopped" eval 'has "$out" "Stopped." && ! tmux has-session -t wam 2>/dev/null && ! pgrep -f rosmaster >/dev/null'

echo "3. GRIPPER=robotiq: the gripper pane"
settings no robotiq
out=$(wam teleop)
echo "$out" | grep -v "^$" | indent
check "wam teleop: the panes are $(panes)" [ "$(panes)" = "follower gripper leader roscore " ]
check "... with a note that the controller is off" has "$out" "the gripper holds until it is on"
wait_for 30 eval 'pane_text | grep -q "not activated yet"'
check "the gripper pane: the node connects, finds the gripper not activated and says how to activate it" \
	eval 'pane_text | grep -q "not activated yet" && pane_text | grep -q "wam gripper restart" && [ "$(activations)" = 0 ]'
pane_text | grep -a "Gripper:" | cut -c1-200 | indent
# the arm programs, by name (the launcher knows them by their paths); like them they end on SIGINT
idle='import signal, sys, time; signal.signal(signal.SIGINT, lambda *a: sys.exit(0)); time.sleep(100000)'
bash -c "exec -a $BIN/leader_wam8g python3 -c '$idle'" &
bash -c "exec -a $BIN/follower python3 -c '$idle'" &
sleep 0.5
arms=$(pgrep -f "^$BIN/(leader_wam8g|follower)( |$)" | tr '\n' ' ')
out=$(wam status)
echo "$out" | indent
check "wam status: the serial device, and the gripper among what runs" \
	eval 'has "$out" "serial device $PORT: there" && has "$out" "Running: gripper" && has "$out" "Running: leader" && has "$out" "Running: follower"'
out=$(wam gripper status)
echo "$out" | indent
check "wam gripper status: the device, and the node's own line" eval 'has "$out" "Serial device $PORT: there" && has "$out" "Gripper: NOT USABLE: not activated"'

echo "4. wam gripper restart"
node=$(node_pid)
t0=$(date +%s.%N)
out=$(wam gripper restart)
rc=$?
took=$(python3 -c "print('%.1f' % ($(date +%s.%N) - $t0))")
echo "$out" | indent
check "it says what happens to what is held, and answers with the node's line (exit $rc, $took s)" \
	eval '[ $rc -eq 0 ] && has "$out" "whatever they hold falls" && has "$out" "Gripper: restarted in"'
check "the gripper was activated, by the same node" eval '[ "$(activations)" = 1 ] && [ "$(node_pid)" = "$node" ]'
check "the arm programs were not touched" [ "$(pgrep -f "^$BIN/(leader_wam8g|follower)( |$)" | tr '\n' ' ')" = "$arms" ]
out=$(wam gripper status)
check "wam gripper status afterwards: $(grep -o "Gripper: [^;]*" <<<"$out")" has "$out" "Gripper: ready, opening"
before=$(activations)
out=$(wam gripper restart)
check "wam gripper restart again, on a gripper that is well: activated once more" eval 'has "$out" "Gripper: restarted in" && [ "$(activations)" = $((before + 1)) ]'

echo "5. ... with the serial device gone"
sim unplug
wait_for 10 eval 'pane_text | grep -q "serial port does not open"'
out=$(wam gripper status)
echo "$out" | indent
check "wam gripper status: the device is not there, the gripper not answering" eval 'has "$out" "NOT THERE" && has "$out" "Gripper: NOT ANSWERING"'
out=$(wam gripper restart)
rc=$?
echo "$out" | indent
check "wam gripper restart: fails with the node's reason (exit $rc); the node is left running" \
	eval '[ $rc -eq 1 ] && has "$out" "RESTART FAILED: the serial port does not open" && [ "$(node_pid)" = "$node" ]'
sim plug
wait_for 10 eval 'pane_text | grep -q "answers again and is still activated"'
check "the device back: the pane says the gripper answers again" eval 'pane_text | grep -q "answers again and is still activated"'

echo "6. ... with the node hung"
kill -STOP "$node"
before=$(activations)
out=$(wam gripper restart)
rc=$?
kill -CONT "$node" 2>/dev/null
echo "$out" | indent
check "the node does not answer: it is stopped, its launch file starts a new one, the restart goes through that (exit $rc)" \
	eval '[ $rc -eq 0 ] && has "$out" "does not answer" && has "$out" "Gripper: restarted in" && [ -n "$(node_pid)" ] && [ "$(node_pid)" != "$node" ]'
check "one activation came of it" [ "$(activations)" = $((before + 1)) ]
check "the arm programs were not touched" [ "$(pgrep -f "^$BIN/(leader_wam8g|follower)( |$)" | tr '\n' ' ')" = "$arms" ]

echo "7. ... with the gripper's launch file ended"
tmux send-keys -t "$(tmux list-panes -s -t wam -F '#{pane_id} #{@name}' | awk '$2 == "gripper" { print $1 }')" C-c
wait_for 20 eval '! pgrep -f "roslaunch wam_teleop robotiq_nav_controller.launch"'
out=$(wam gripper status)
check "wam gripper status says the launch file is not running" has "$out" "launch file is not running"
out=$(wam gripper restart)
rc=$?
echo "$out" | indent
check "wam gripper restart starts it again in its pane and restarts the gripper (exit $rc)" \
	eval '[ $rc -eq 0 ] && has "$out" "starting it again in its pane" && has "$out" "Gripper: restarted in" && pgrep -f "roslaunch wam_teleop robotiq_nav_controller.launch" >/dev/null'

echo "8. wam stop"
out=$(wam stop --force)
echo "$out" | tail -n 4 | indent
check "everything ends: the arm programs, the gripper's launch file and node, roscore, the session" \
	eval 'has "$out" "Stopped." && ! tmux has-session -t wam 2>/dev/null && ! pgrep -f "rosmaster|roslaunch|robotiq_teleop|leader_wam8g|/follower" >/dev/null'
check "the gripper pane's log has the node's last line: nothing sent to the gripper at the end" \
	eval 'pane_text | grep -q "stopped. Nothing was sent to the gripper at the end"'
out=$(wam gripper restart)
check "wam gripper restart now: $out" has "$out" "no teleop session to start them in"

kill "$simulator" "$keeper" 2>/dev/null
tmux kill-server 2>/dev/null
rm -rf "$work"
echo "$checks checks, $failures failed"
[ "$failures" -eq 0 ]
