// lib/robotiq_teleop_core.h without a gripper, a controller or ROS.
//
// The core runs in simulated time (a step every 25 ms) against a 2F-140 made
// up here, as it looks through libgripper: travel at the speed of the last
// request, an activation that takes the fingers through their travel, an
// object between the fingers, faults, a power cycle, a serial device that
// goes away, and the library's activate() with its own 4 s wait.
//
//   1. The start: a gripper that is not activated waits to be asked, or is
//      activated at once with auto_activate; one that is already activated
//      is left alone.
//   2. The stick: speed proportional to it, both ways; letting go holds.
//   3. Closing on an object: the request stays just beyond it, opening
//      responds at once, the object giving way; and the fingers' own stops,
//      which a real gripper has a few counts inside the range.
//   4. L2 + cross: fully open, once per press.
//   5. No joystick messages: hold.
//   6. L2 + circle held: a restart, once per hold.
//   7. Faults: the major ones wait for a restart, the passing ones for
//      themselves; a gripper that lost its power; auto_restart.
//   8. Silence: nothing is sent, the port is reopened every second, and the
//      teleop goes on without a restart when the gripper is still activated.
//   9. Restarts that do not work.
//  10. A gripper slower than the stick, a loop held up, a gripper that looks
//      healthy and does not do as asked.
// Throughout: no position request ever goes to a gripper that is not
// activated, ready and free of faults (the library's setWidth() sets the
// activation bit, so such a request would start an activation).

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <string>
#include <vector>

#include "robotiq_teleop_core.h"

using namespace robotiq_teleop;

namespace {

int checks = 0;
int failures = 0;
int requestsToUnreadyGripper = 0;  // over every test

std::string strf(const char* format, ...) {
	char buf[1024];
	va_list ap;
	va_start(ap, format);
	vsnprintf(buf, sizeof(buf), format, ap);
	va_end(ap);
	return buf;
}

void check(bool ok, const std::string& what) {
	++checks;
	if (!ok) {
		++failures;
	}
	std::printf("  %s %s\n", ok ? "ok  " : "FAIL", what.c_str());
}

const double STROKE = 140.0;
const double STEP = 0.025;           // s, the node's loop
const double COUNT = STROKE / 255.0; // mm per position count

// The gripper, and the serial port in front of it.
struct SimGripper {
	// the mechanism
	double width = STROKE;           // mm
	double object = -1.0;            // mm: something this wide between the fingers; < 0: nothing
	double openStop = STROKE;        // mm: the fingers' own stops; a real gripper's lie a few counts
	double closedStop = 0.0;         //     inside the range (the manual's example reads 13 at fully open)
	double activationSpeed = 110.0;  // mm/s through the travel in an activation: closed, then open
	// what can be wrong with it
	bool device = true;              // the serial device exists
	bool answers = true;             // the gripper is powered and on the line
	int stuckFault = 0;              // a fault whose cause stays: back after every reset
	int activationFault = 0;         // the fault an activation ends in
	bool ignoresRequests = false;    // answers a request, does not take it
	bool frozen = false;             // reports moving, does not move
	bool neverArrives = false;       // keeps reporting "in motion" at the request it has reached
	int droppedWrites = 0;           // this many position requests fail on the line
	bool outOfStep = false;          // a stale answer on the line: requests arrive, their answers do not match
	// its state
	bool portOpen = false, stale = false;
	bool act = false, gto = false, mustClear = false;
	int request = 0;                 // counts
	double speed = 0.0, force = 0.0;
	int sta = 0, obj = 0, fault = 0;
	int activating = 0;              // 1 closing, 2 opening
	// the record
	int connects = 0, disconnects = 0, resets = 0, activateCalls = 0, activations = 0, moves = 0;

	bool talks() const { return portOpen && !stale && device && answers; }

	void unplug() {
		device = false;
		stale = portOpen;  // the open port is dead even when the device is back
	}
	void powerCycle() {
		act = gto = false;
		sta = obj = fault = 0;
		activating = 0;
		mustClear = true;  // the manual: after a power loss the activation bit has to be cleared, then set
	}

	void advance(double dt) {
		if (!answers) {
			return;  // no power: the fingers stay (self-locking)
		}
		if (activating == 1) {
			const double stop = (object >= 0.0 && object <= width) ? object : closedStop;
			width = std::max(stop, width - activationSpeed * dt);
			if (width <= stop && activationSpeed > 0.0) {
				activating = 2;
			}
		} else if (activating == 2) {
			width = std::min(openStop, width + activationSpeed * dt);
			if (width >= openStop) {
				activating = 0;
				sta = 3;
				obj = 3;
			}
		} else if (act && sta == 3 && gto && fault == 0) {
			const double goal = widthFromCount(request, STROKE);
			if (frozen) {
				obj = std::fabs(goal - width) > 1e-9 ? 0 : 3;
				return;
			}
			const double v = 30.0 + speed * 220.0;  // 2F-140: 30 to 250 mm/s
			if (goal < width - 1e-9) {
				double stop = std::max(goal, closedStop);  // the fingers touching each other: contact
				if (object >= 0.0 && object <= width + 1e-9) {
					stop = std::max(stop, object);
				}
				width = std::max(stop, width - v * dt);
				obj = width > stop ? 0 : (stop > goal ? 2 : 3);
			} else if (goal > width + 1e-9) {
				const double stop = std::min(goal, openStop);  // the manual's example: "at requested position" there
				width = std::min(stop, width + v * dt);
				obj = width < stop ? 0 : 3;
			} else if (obj == 0) {
				obj = 3;
			}
			if (neverArrives && obj == 3) {
				obj = 0;
			}
		}
	}
	void elapse(double seconds) {
		for (double t = 0.0; t < seconds - 1e-9; t += 0.005) {
			advance(0.005);
		}
	}

	Status status() const {
		Status s;
		if (!talks()) {
			return s;
		}
		s.valid = true;
		s.activated = act;
		s.ready = sta == 3;
		s.object = obj;
		s.moving = gto && obj == 0;
		s.fault = fault;
		s.echo = request;
		s.position = countFromWidth(width, STROKE);
		s.width = widthFromCount(s.position, STROKE);
		s.current = s.moving ? 30 : 0;
		return s;
	}

	// One write of the command registers.
	void command(bool a, bool g, int count, double sp, double fo) {
		if (ignoresRequests) {
			return;
		}
		if (!a) {
			act = gto = false;
			sta = obj = 0;
			activating = 0;
			fault = stuckFault;
			mustClear = false;
			return;
		}
		if (!act) {  // the activation bit rises: the activation routine
			if (mustClear) {
				fault = 0x07;
				return;
			}
			act = true;
			++activations;
			if (activationFault != 0) {
				fault = activationFault;
			} else {
				sta = 1;
				activating = 1;
			}
		}
		gto = g;
		request = count;
		speed = sp;
		force = fo;
	}

	bool connect() {
		++connects;
		portOpen = device;
		stale = false;
		outOfStep = false;  // closing the port emptied the line
		return portOpen;
	}
	void disconnect() {
		++disconnects;
		portOpen = false;
	}
	bool reset() {
		if (!talks()) {
			return false;
		}
		++resets;
		command(false, false, 0, 0.0, 0.0);
		return true;
	}
	// libgripper's activate(): reset, 0.1 s, the activation command, the
	// status every 50 ms for up to 4 s, then the request to go fully open.
	bool activate(double sp, double fo, double* took) {
		++activateCalls;
		*took = 0.0;
		if (!reset()) {
			return false;
		}
		elapse(0.1);
		*took += 0.1;
		command(true, false, 0, sp, fo);
		for (double t = 0.0; t < 4.0; t += 0.05) {
			if (talks() && sta == 3) {
				command(true, true, 0, sp, fo);
				return true;
			}
			elapse(0.05);
			*took += 0.05;
		}
		return false;
	}
	bool move(double w, double sp, double fo) {
		++moves;
		if (!talks()) {
			return false;
		}
		if (droppedWrites > 0) {
			--droppedWrites;
			return false;
		}
		if (!(act && sta == 3 && fault == 0)) {
			++requestsToUnreadyGripper;
		}
		command(true, true, countFromWidth(w, STROKE), sp, fo);
		return !outOfStep;  // out of step the gripper carries it out, and the library reports a failure
	}
};

// The parameters with the activation at the start switched on, as most of
// the tests want a gripper that is ready without more ado.
Params activating() {
	Params p;
	p.autoActivate = true;
	return p;
}

// The node's loop: a status read, a step, the action, the notes.
struct Rig {
	explicit Rig(const Params& p = activating()) : core(p) {}

	SimGripper g;
	Core core;
	Joy joy;
	bool controller = true;  // it sends messages
	double t = 0.0;
	std::vector<std::string> said;
	std::vector<Action::Kind> actions;

	void step() {
		if (controller) {
			joy.valid = true;
			joy.stamp = t;
		}
		const Action a = core.step(t, joy, g.status());
		bool ok = true;
		double took = 0.0;
		switch (a.kind) {
		case Action::MOVE: ok = g.move(a.width, a.speed, a.force); break;
		case Action::DISCONNECT: g.disconnect(); break;
		case Action::CONNECT: ok = g.connect(); break;
		case Action::RESET: ok = g.reset(); break;
		case Action::ACTIVATE: ok = g.activate(a.speed, a.force, &took); break;
		default: break;
		}
		t += took;
		if (a.kind != Action::NONE) {
			actions.push_back(a.kind);
			core.result(ok, t);
		}
		const std::vector<std::string> notes = core.takeNotes();
		for (size_t i = 0; i < notes.size(); ++i) {
			std::printf("       [%7.2f s] Gripper: %s\n", t, notes[i].c_str());
			said.push_back(notes[i]);
		}
		g.elapse(STEP);
		t += STEP;
	}
	void run(double seconds) {
		const double end = t + seconds;
		while (t < end - 1e-9) {
			step();
		}
	}
	// To READY at the fully open position, as after a start.
	void start() {
		run(6.0);
	}

	void hold(double axis) {  // L2 and the stick
		joy.deadman = true;
		joy.axis = axis;
		joy.open = joy.restart = false;
	}
	void letGo() {
		joy.deadman = joy.open = joy.restart = false;
		joy.axis = 0.0;
	}

	int count(Action::Kind k, size_t from = 0) const {
		int n = 0;
		for (size_t i = from; i < actions.size(); ++i) {
			n += actions[i] == k ? 1 : 0;
		}
		return n;
	}
	bool heard(const char* text, size_t from = 0) const {
		for (size_t i = from; i < said.size(); ++i) {
			if (said[i].find(text) != std::string::npos) {
				return true;
			}
		}
		return false;
	}
	int heardTimes(const char* text, size_t from = 0) const {
		int n = 0;
		for (size_t i = from; i < said.size(); ++i) {
			n += said[i].find(text) != std::string::npos ? 1 : 0;
		}
		return n;
	}
	double requested() const { return widthFromCount(g.request, STROKE); }  // mm, what the gripper was last asked for
	bool ready() const { return core.getState() == READY; }
};

// The full stick, and the deflection that is half of it past the deadband.
const double FULL = 1.0;
const double HALF = 0.05 + 0.5 * 0.95;

void testStart() {
	std::printf("1. The start\n");
	{
		Rig r;  // auto_activate on
		r.g.width = 90.0;  // wherever it was when it lost its power
		r.run(6.0);
		const bool order = r.actions.size() >= 5 && r.actions[0] == Action::CONNECT && r.actions[1] == Action::DISCONNECT
				&& r.actions[2] == Action::CONNECT && r.actions[3] == Action::RESET && r.actions[4] == Action::ACTIVATE;
		check(order, "auto_activate, a gripper that is not activated: the port opens, then close, open, reset, activate");
		check(r.ready() && r.g.activations == 1 && r.g.sta == 3, strf("ready after one activation (%d)", r.g.activations));
		check(std::fabs(r.core.getTarget() - r.g.status().width) < 1e-9 && r.count(Action::MOVE) == 0,
				strf("the measured opening is the target (%.1f mm), and nothing was asked of the gripper yet", r.core.getTarget()));
		check(r.heard("not activated") && r.heard("RESTART (activation at the start") && r.heard("restarted in"),
				"the pane says: not activated, the restart and why, restarted");
		check(r.core.restartsFinished() == 1 && r.core.lastRestartWorked(), "one restart finished, and it worked");
	}
	{
		Rig r((Params()));  // as set by default: nothing moves before it is asked for
		r.run(3.0);
		check(r.core.getState() == FAULT && r.g.activations == 0 && r.count(Action::ACTIVATE) == 0 && r.count(Action::RESET) == 0,
				"by default it waits: nothing is reset or activated");
		check(r.heard("not activated yet") && r.heard("L2 + circle") && r.heard("wam gripper restart") && r.heard("whatever they hold falls"),
				"... and says how to activate it, and that this moves the fingers and drops what is held");
		r.hold(FULL);
		r.run(1.0);
		check(r.count(Action::MOVE) == 0, "... and the stick does nothing meanwhile");
		r.letGo();
		check(r.core.requestRestart("the test"), "a restart request is taken");
		r.run(6.0);
		check(r.ready() && r.g.activations == 1, "... and makes it ready");
	}
	{
		Rig r;
		r.g.act = r.g.gto = true;  // activated before this program started, holding at 87 mm
		r.g.sta = 3;
		r.g.obj = 3;
		r.g.request = countFromWidth(87.0, STROKE);
		r.g.width = widthFromCount(r.g.request, STROKE);
		r.run(2.0);
		check(r.ready() && r.g.activations == 0 && r.g.resets == 0 && r.count(Action::MOVE) == 0,
				"a gripper that is already activated and ready is taken as it is: no reset, no activation, no request");
		check(std::fabs(r.core.getTarget() - r.g.width) < 1e-9 && r.heard("not activated again"),
				strf("... at its opening (%.1f mm), and the pane says so", r.core.getTarget()));
	}
}

void testStick() {
	std::printf("2. The stick\n");
	Rig r;
	r.start();
	r.hold(FULL);
	r.run(0.5);
	double w0 = r.g.width;
	r.run(1.0);
	double speed = w0 - r.g.width;
	check(speed > 48.0 && speed < 52.0, strf("L2 + full stick up closes at %.1f mm/s (50 set)", speed));
	check(r.g.speed == 0.3 && r.g.force == 0.5, "every request carries the set device speed and force");

	r.letGo();
	w0 = r.g.width;
	r.run(0.3);
	const double overrun = w0 - r.g.width;
	const double held = r.g.width;
	const size_t actionsThen = r.actions.size();
	r.run(1.0);
	check(overrun >= 0.0 && overrun < 3.0 && r.g.width == held && r.actions.size() == actionsThen,
			strf("L2 released: the opening stays (%.1f mm further, then still), nothing more is sent", overrun));

	r.hold(HALF);
	r.run(0.5);
	w0 = r.g.width;
	r.run(1.0);
	speed = w0 - r.g.width;
	check(speed > 23.5 && speed < 26.5, strf("half stick: %.1f mm/s", speed));

	r.hold(0.0);
	r.run(0.3);
	w0 = r.g.width;
	r.run(1.0);
	check(r.g.width == w0, "L2 with the stick centred: the opening stays");

	r.joy.deadman = false;
	r.joy.axis = FULL;
	const int movesThen = r.g.moves;
	r.run(1.0);
	check(r.g.width == w0 && r.g.moves == movesThen, "the stick without L2: nothing");

	r.hold(-FULL);
	r.run(0.3);
	w0 = r.g.width;
	r.run(0.5);
	speed = (r.g.width - w0) / 0.5;
	check(speed > 48.0 && speed < 52.0, strf("L2 + full stick down opens at %.1f mm/s", speed));

	r.hold(0.1);  // a tenth of the stick: a position count every fifth step
	r.run(0.5);
	w0 = r.g.width;
	r.run(2.0);
	speed = (w0 - r.g.width) / 2.0;
	const double expected = 50.0 * (0.1 - 0.05) / 0.95;
	check(std::fabs(speed - expected) < 0.4, strf("a tenth of the stick: %.2f mm/s (%.2f by proportion)", speed, expected));

	r.hold(FULL);
	r.run(5.0);
	check(r.g.width == 0.0 && r.core.getTarget() >= 0.0, "held up: fully closed, and the target stops at 0");
	r.hold(-FULL);
	r.run(5.0);
	check(r.g.width == STROKE && r.core.getTarget() <= STROKE, "held down: fully open, and the target stops at the stroke");

	r.joy.axis = std::nan("");
	r.run(0.5);
	check(r.g.width == STROKE && r.ready(), "a stick value that is not a number: nothing");
}

void testObject() {
	std::printf("3. Closing on an object\n");
	Rig r;
	r.start();
	r.g.object = 60.0;
	r.hold(FULL);
	r.run(3.0);
	const double past = r.g.width - r.requested();
	check(r.g.width == 60.0 && r.g.obj == 2, "the fingers stop on the object (60 mm), the gripper reports contact while closing");
	check(past > 0.0 && past <= 5.0 + COUNT, strf("the request is %.1f mm beyond the contact: beyond it, by no more than the lead and a position count (5.5 mm)", past));
	check(r.heardTimes("closing stopped by contact") == 1 && r.core.inContact(), "the pane says so, once");
	int moves = r.g.moves;
	r.run(2.0);
	check(r.g.moves == moves && r.g.width - r.requested() == past, "the stick still held up: the request goes no further");

	r.letGo();
	r.run(1.0);
	check(r.g.moves == moves && r.requested() < r.g.width && r.g.obj == 2, "L2 released: the request stays beyond the object (the grip keeps its force)");

	r.hold(-FULL);
	int steps = 0;
	while (r.g.width <= 60.0 && steps < 40) {
		r.step();
		++steps;
	}
	check(steps <= 2, strf("stick down: the fingers open within %d step%s (%d ms)", steps, steps == 1 ? "" : "s",
			static_cast<int>(steps * STEP * 1000)));
	r.run(0.5);
	check(!r.core.inContact(), "... and the contact is over");

	// the same at a tenth of the stick, where a far-off target would take seconds to come back
	r.hold(FULL);
	r.run(2.0);
	r.hold(-0.1);
	steps = 0;
	while (r.g.width <= 60.0 && steps < 80) {
		r.step();
		++steps;
	}
	check(steps <= 12, strf("opening slowly from a grip: the fingers open after %d ms (one position count at that speed)",
			static_cast<int>(steps * STEP * 1000)));

	// what is held gives way
	r.hold(FULL);
	r.run(2.0);
	r.letGo();
	r.run(0.5);
	const size_t saidThen = r.said.size();
	const double requested = r.requested();
	r.g.object = -1.0;
	r.run(1.0);
	check(r.g.width == requested && 60.0 - r.g.width <= 5.0 + COUNT,
			strf("the object slips out: the fingers follow by %.1f mm, to the request, and stop", 60.0 - r.g.width));
	check(r.heard("no contact any more", saidThen) && !r.core.inContact(), "... and the pane says the contact is lost");
}

void testEndStops() {
	std::printf("3b. The fingers' own stops, inside the range of the position counts\n");
	Rig r;
	r.g.openStop = widthFromCount(13, STROKE);     // 132.9 mm
	r.g.closedStop = widthFromCount(230, STROKE);  // 13.7 mm
	r.g.width = r.g.openStop;
	r.start();
	check(r.ready() && r.g.width == r.g.openStop && std::fabs(r.core.getTarget() - r.g.openStop) < 1e-9,
			strf("after the activation the fingers stand at their open stop, %.1f mm by the counts, and that is the target", r.g.width));
	r.hold(-FULL);
	r.run(2.0);
	check(r.core.getTarget() - r.g.width <= 5.0 + COUNT, strf("stick down at the open stop: the target stops %.1f mm beyond it", r.core.getTarget() - r.g.width));
	r.hold(FULL);
	int steps = 0;
	while (r.g.width >= r.g.openStop && steps < 40) {
		r.step();
		++steps;
	}
	check(steps <= 2, strf("stick up from there: the fingers close within %d step%s", steps, steps == 1 ? "" : "s"));
	r.run(4.0);
	check(r.g.width == r.g.closedStop && r.g.obj == 2 && r.g.width - r.requested() <= 5.0 + COUNT,
			strf("held up: closed on their own stop (%.1f mm), the request %.1f mm beyond it", r.g.width, r.g.width - r.requested()));
	r.joy.deadman = r.joy.open = true;
	r.joy.axis = 0.0;
	r.run(0.1);
	r.letGo();
	r.run(3.0);
	check(r.g.width == r.g.openStop && r.g.request == 0, "L2 + cross: to the open stop, the request at count 0");
	r.hold(FULL);
	steps = 0;
	while (r.g.width >= r.g.openStop && steps < 40) {
		r.step();
		++steps;
	}
	check(steps <= 2, strf("stick up after it: the fingers close within %d step%s, although the request was 7 mm beyond the stop", steps,
			steps == 1 ? "" : "s"));
}

void testOpenFully() {
	std::printf("4. L2 + cross\n");
	Rig r;
	r.start();
	r.hold(FULL);
	r.run(2.0);
	r.letGo();
	r.run(0.5);
	const double from = r.g.width;
	size_t saidThen = r.said.size();
	int moves = r.g.moves;

	r.joy.open = true;  // cross alone
	r.run(0.5);
	check(r.g.width == from && r.g.moves == moves, "cross without L2: nothing");
	r.joy.deadman = true;
	r.run(0.1);
	r.letGo();
	r.run(2.0);
	check(r.g.width == STROKE && r.g.moves == moves + 1 && r.heardTimes("opening fully", saidThen) == 1,
			strf("L2 + cross, a short press: from %.0f mm to fully open with one request, although everything was let go", from));

	r.hold(FULL);
	r.run(1.5);
	r.letGo();
	saidThen = r.said.size();
	moves = r.g.moves;
	r.joy.deadman = r.joy.open = true;
	r.run(3.0);
	check(r.g.width == STROKE && r.g.moves == moves + 1 && r.heardTimes("opening fully", saidThen) == 1,
			"held for 3 s: one action");
	r.letGo();
	r.run(0.2);

	r.hold(FULL);
	r.run(2.0);
	r.letGo();
	r.run(0.2);
	r.joy.deadman = r.joy.open = true;
	r.run(0.3);  // on its way open
	const double reversedAt = r.g.width;
	r.hold(FULL);
	double peak = reversedAt;
	for (int i = 0; i < 40; ++i) {
		r.step();
		peak = std::max(peak, r.g.width);
	}
	check(reversedAt < 130.0 && peak - reversedAt < 3.0 && r.g.width < reversedAt - 20.0,
			strf("stick up on the way open (at %.0f mm): it turns round within %.1f mm and closes", reversedAt, peak - reversedAt));
}

void testNoJoystick() {
	std::printf("5. No joystick messages\n");
	Rig r;
	r.start();
	r.hold(FULL);
	r.run(1.0);
	r.controller = false;  // the last message stays as it was: L2 and the stick held
	size_t saidThen = r.said.size();
	const double t0 = r.t;
	double stoppedAt = -1.0, last = r.g.width;
	for (int i = 0; i < 80; ++i) {
		r.step();
		if (stoppedAt < 0.0 && r.g.width == last) {
			stoppedAt = r.t - STEP - t0;
		} else if (r.g.width != last) {
			stoppedAt = -1.0;
		}
		last = r.g.width;
	}
	check(stoppedAt > 0.45 && stoppedAt < 0.65, strf("the controller goes silent with the stick held: the fingers stop %.2f s later (0.5 s set)", stoppedAt));
	check(r.heardTimes("no joystick message", saidThen) == 1, "the pane says so, once");

	r.controller = true;
	r.letGo();
	r.run(0.2);
	check(r.heard("navigation controller connected", saidThen), "the controller back: the pane says so");
	r.hold(FULL);
	r.run(1.5);  // nearly closed, so that the way open is long
	r.letGo();
	r.run(0.2);
	r.joy.deadman = r.joy.open = true;
	r.run(0.3);
	r.controller = false;
	r.run(0.5 + 2 * STEP);
	const double at = r.g.width;
	r.run(2.0);
	check(at < 110.0 && r.g.width - at <= 5.0 + COUNT, strf("silent during a full opening: it stops (at %.0f mm, %.1f mm further)", at, r.g.width - at));

	r.controller = true;
	r.letGo();
	r.g.object = 60.0;
	r.hold(FULL);
	r.run(3.0);
	const int request = r.g.request;
	r.controller = false;
	r.run(3.0);
	check(r.g.request == request && r.requested() < r.g.width, "silent while holding an object: the request stays beyond it");

	Rig none;
	none.controller = false;
	none.run(8.0);
	check(none.ready() && none.heardTimes("no navigation controller yet") == 1, "no controller from the start: the gripper is made ready, the pane says to turn the controller on");
}

void testRestartHold() {
	std::printf("6. L2 + circle\n");
	Rig r;
	r.start();
	r.g.object = 60.0;
	r.hold(FULL);
	r.run(3.0);
	r.letGo();
	r.run(0.2);

	r.joy.restart = true;  // circle alone
	r.run(3.0);
	check(r.g.activations == 1, "circle without L2 for 3 s: no restart");
	r.joy.deadman = true;
	r.run(0.9);
	r.letGo();
	r.run(0.5);
	check(r.g.activations == 1 && r.ready(), "L2 + circle for 0.9 s: no restart");

	size_t from = r.actions.size();
	r.joy.deadman = r.joy.restart = true;
	r.run(1.0 + 2 * STEP);
	check(r.core.getState() == RESTARTING, "held for 1 s: the restart starts");
	r.run(8.0);  // still held
	const bool order = r.actions.size() >= from + 4 && r.actions[from] == Action::DISCONNECT && r.actions[from + 1] == Action::CONNECT
			&& r.actions[from + 2] == Action::RESET && r.actions[from + 3] == Action::ACTIVATE;
	check(order, "it closes the port, opens it, resets, activates");
	check(r.g.activations == 2 && r.ready(), "held on for 9 s: one restart, not more");
	check(r.g.width == STROKE && std::fabs(r.core.getTarget() - STROKE) < 1e-9 && r.count(Action::MOVE, from) == 0,
			"afterwards the fingers are where the activation left them (open: what they held has fallen), and that is the target");
	r.letGo();
	r.run(0.2);
	r.joy.deadman = r.joy.restart = true;
	r.run(8.0);
	check(r.g.activations == 3 && r.ready(), "let go and held again: the next restart");
	r.letGo();

	check(r.core.requestRestart("the test") && !r.core.requestRestart("the test again"), "a second request while one waits is refused");
	r.run(0.5);
	check(!r.core.requestRestart("the test again"), "... and while it runs");
	r.run(8.0);
	check(r.g.activations == 4 && r.ready() && r.core.restartsFinished() == 4, "... and one restart comes of it");
}

void testFaults() {
	std::printf("7. Faults\n");
	bool table = Fault::kind(0) == Fault::NONE;
	const int passing[] = {0x05, 0x08, 0x09};
	const int needReset[] = {0x07, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x03};
	for (size_t i = 0; i < 3; ++i) {
		table = table && Fault::kind(passing[i]) == Fault::PASSES;
	}
	for (size_t i = 0; i < 8; ++i) {
		table = table && Fault::kind(needReset[i]) == Fault::RESET;
	}
	check(table, "the manual's table: 0x05, 0x08, 0x09 need no reset; 0x07 and 0x0A-0x0F do; an unlisted code is treated as these");
	{
		Rig r;
		r.start();
		r.hold(FULL);
		r.run(1.0);
		const size_t saidThen = r.said.size();
		r.g.fault = 0x0E;
		r.step();
		r.step();
		const int moves = r.g.moves;
		check(r.core.getState() == FAULT, "overcurrent (0x0E) while closing: noticed in the next step");
		r.run(3.0);
		check(r.g.moves == moves && r.g.activations == 1, "nothing is sent, with the stick still held; no restart by itself");
		check(r.heard("fault 0x0E", saidThen) && r.heard("overcurrent", saidThen) && r.heard("wam gripper restart", saidThen)
				&& r.heard("position", saidThen),
				"the pane has the code, its meaning, how to restart, and the status");
		r.joy.open = true;
		r.run(0.2);
		check(r.heard("L2 + X ignored", saidThen), "L2 + cross meanwhile: ignored, and the pane says why");
		r.letGo();
		r.core.requestRestart("the test");
		r.run(6.0);
		check(r.ready() && r.g.fault == 0 && r.g.activations == 2, "a restart clears it");
	}
	{
		Rig r;
		r.start();
		r.hold(FULL);
		r.run(1.0);
		const size_t saidThen = r.said.size();
		const int resets = r.g.resets;
		r.g.fault = 0x09;
		r.run(1.0);
		const double at = r.g.status().width;
		check(r.core.getState() == FAULT && r.heard("no communication", saidThen) && r.heard("needs no restart", saidThen),
				"no communication for a second (0x09): waited out, and the pane says it needs no restart");
		r.g.fault = 0;
		r.run(0.1);
		check(r.ready() && r.g.activations == 1 && r.g.resets == resets && r.heard("no fault any more", saidThen),
				"it clears: ready again without a restart");
		r.letGo();
		r.run(0.5);
		check(std::fabs(r.g.width - at) < 5.0 + COUNT, "... from the opening it has");

		r.g.fault = 0x08;
		const size_t then = r.said.size();
		r.run(7.0);
		check(r.core.getState() == FAULT && r.heard("has not cleared", then) && r.g.activations == 1,
				"over temperature (0x08) for 7 s: after 5 s the pane suggests a restart; none by itself");
		r.g.fault = 0;
		r.run(0.1);
	}
	{
		Rig r;
		r.start();
		r.hold(FULL);
		r.run(1.0);
		const size_t saidThen = r.said.size();
		r.g.powerCycle();
		r.run(3.0);
		check(r.core.getState() == FAULT && r.heard("not activated", saidThen) && r.g.activations == 1 && r.g.fault == 0,
				"the gripper loses its power and comes back (in reset): waits for a restart; no request reached it");
		r.letGo();
		r.joy.deadman = r.joy.restart = true;
		r.run(8.0);
		r.letGo();
		check(r.ready() && r.g.activations == 2, "L2 + circle: ready again");
	}
	{
		Params p = activating();
		p.autoRestart = true;
		Rig r(p);
		r.start();
		const size_t saidThen = r.said.size();
		r.g.fault = 0x0E;
		r.run(6.0);
		check(r.ready() && r.g.activations == 2 && r.heard("automatic, auto_restart", saidThen), "auto_restart: it restarts itself");
		r.g.fault = 0x09;
		r.run(2.0);
		check(r.g.activations == 2 && r.core.getState() == FAULT, "... but not for a fault that passes");
		r.g.fault = 0;
		r.run(0.2);
		r.g.stuckFault = r.g.fault = 0x0A;  // the supply stays low
		const int before = r.g.activateCalls;
		r.run(35.0);
		const int tries = r.g.activateCalls - before;
		check(tries >= 3 && tries <= 4, strf("a fault that comes back after every reset: %d tries in 35 s (one every 10 s)", tries));
	}
}

void testSilence() {
	std::printf("8. Silence\n");
	{
		Rig r;
		r.start();
		r.hold(FULL);
		r.run(1.0);
		const size_t saidThen = r.said.size();
		const int moves = r.g.moves;
		r.g.answers = false;
		r.run(0.3);
		check(r.ready() && r.g.moves == moves, "0.3 s without a status: still ready, and nothing was sent on the old reading");
		r.g.answers = true;
		r.run(0.5);
		check(r.ready() && r.g.moves > moves && r.said.size() == saidThen, "the status back: the stick goes on, nothing to report");

		r.g.answers = false;
		const size_t from = r.actions.size();
		r.run(5.0);
		check(r.core.getState() == SILENT && r.heard("NOT ANSWERING", saidThen) && r.heard("Its last status", saidThen),
				"5 s without a status: not answering, with the last status in the pane");
		const int opens = r.count(Action::CONNECT, from);
		check(opens >= 4 && opens <= 5 && r.count(Action::DISCONNECT, from) == opens,
				strf("the port was closed and opened %d times in those 5 s", opens));
		check(r.count(Action::MOVE, from) == 0 && r.count(Action::RESET, from) == 0 && r.count(Action::ACTIVATE, from) == 0,
				"nothing else was tried: no request, no reset, no activation");
		r.g.answers = true;
		for (int i = 0; i < 80 && !r.ready(); ++i) {
			r.step();
		}
		check(r.ready() && r.g.activations == 1 && r.heard("answers again and is still activated", saidThen),
				"it answers again, still activated: ready without a restart");
		const double w0 = r.g.width;
		r.run(0.5);
		check(w0 - r.g.width > 20.0, "... and the stick, held all along, moves it again");
	}
	{
		Rig r;
		r.start();
		const size_t saidThen = r.said.size();
		r.g.unplug();
		r.run(3.0);
		check(r.core.getState() == SILENT && r.heard("serial port does not open", saidThen), "the serial device gone: the pane says the port does not open");
		r.g.device = true;
		r.run(1.5);
		check(r.ready() && r.g.activations == 1, "the device back: ready, no restart");

		r.g.unplug();
		r.g.powerCycle();
		r.run(2.0);
		r.g.device = true;
		const size_t then = r.said.size();
		r.run(2.0);
		check(r.core.getState() == FAULT && r.heard("answers again", then) && r.heard("not activated", then) && r.g.activations == 1,
				"the device back with the gripper in reset: it says so and waits for a restart");
		r.core.requestRestart("the test");
		r.run(6.0);
		check(r.ready() && r.g.activations == 2, "... which makes it ready");

		r.g.unplug();
		r.run(2.0);
		const unsigned finished = r.core.restartsFinished();
		r.core.requestRestart("the test");
		r.run(1.0);
		check(r.core.restartsFinished() == finished + 1 && !r.core.lastRestartWorked()
				&& r.core.lastRestartNote().find("serial port does not open") != std::string::npos && r.core.getState() == SILENT,
				"a restart asked for with the device gone: it fails and says why");
		r.g.device = true;
		r.run(1.5);
		check(r.ready() && r.g.activations == 2, "the device back: ready (the gripper was activated all along)");
	}
	{
		Rig r;
		r.g.device = false;
		r.run(40.0);
		check(r.core.getState() == SILENT && r.heardTimes("trying every") == 1 && r.heard("still no answer"),
				"no serial device from the start: said once, with a reminder later");
		check(r.said.size() <= 4, strf("... in %zu lines over 40 s, not one a second", r.said.size()));
		r.g.device = true;
		r.run(8.0);
		check(r.ready() && r.g.activations == 1, "the device appears: connected and activated");
	}
}

void testRestartsThatFail() {
	std::printf("9. Restarts that do not work\n");
	{
		Rig r;
		r.g.activationSpeed = 280.0 / 6.0;  // 6 s through the travel: longer than the library waits
		r.run(9.0);
		check(r.ready() && r.g.activations == 1 && r.core.lastRestartWorked(),
				"an activation of 6 s (the library's call gives up after 4): the core waits for it, one activation");
	}
	{
		Rig r;
		r.g.activationSpeed = 0.0;  // never finishes
		r.run(9.0);
		check(r.core.getState() == RESTARTING, "an activation that does not finish: still waiting after 9 s");
		r.run(3.0);
		check(!r.ready() && r.heard("RESTART FAILED: not ready and at rest 10 s after") && r.core.restartsFinished() == 1
				&& !r.core.lastRestartWorked(), "... given up 10 s after the activation command, with the status in the pane");
		r.run(8.0);
		check(r.g.activations == 1 && r.core.getState() == FAULT, "... and not tried again by itself");
	}
	{
		Rig r;
		r.g.activationFault = 0x0D;
		r.run(6.0);
		check(r.heard("RESTART FAILED: the gripper reports fault 0x0D") && r.heard("no interference") && r.core.getState() == FAULT,
				"an activation fault (0x0D): the restart fails with the code and its meaning");
		r.g.activationFault = 0;
		r.core.requestRestart("the test");
		r.run(6.0);
		check(r.ready(), "its cause removed, the next restart works");
	}
	{
		Rig r;
		r.start();
		r.g.answers = false;
		r.run(2.0);
		r.core.requestRestart("the test");
		r.run(2.0);
		check(r.heard("did not answer the reset, twice") && r.core.getState() == SILENT,
				"a restart with the port there and the gripper silent: after a second try it says the gripper did not answer the reset");
	}
}

void testOddOnes() {
	std::printf("10. A slow gripper, a loop held up, a gripper that does not do as asked\n");
	{
		Params p = activating();
		p.deviceSpeed = 0.0;  // 30 mm/s, slower than the stick's 50
		Rig r(p);
		r.start();
		r.hold(FULL);
		double most = 0.0;
		for (int i = 0; i < 80; ++i) {
			r.step();
			most = std::max(most, r.g.width - r.core.getTarget());
		}
		r.letGo();
		const double w0 = r.g.width;
		r.run(1.0);
		check(most <= 5.0 + COUNT, strf("a gripper slower than the stick: the target is never more than %.1f mm ahead of it", most));
		check(w0 - r.g.width <= 5.0 + COUNT, strf("... and it travels %.1f mm after the stick is let go", w0 - r.g.width));
	}
	{
		Rig r;
		r.start();
		r.hold(FULL);
		r.run(0.5);
		const double before = r.core.getTarget();
		r.t += 1.0;        // the loop held up for a second
		r.joy.stamp = r.t; // with fresh joystick messages waiting
		r.controller = false;
		r.step();
		check(before - r.core.getTarget() <= 50.0 * Core::MAX_STEP + 1e-9,
				strf("a step 1 s late moves the target %.1f mm, not 50", before - r.core.getTarget()));
	}
	{
		Params p = activating();
		p.lead = 0.3;
		Rig r(p);
		r.start();
		r.hold(FULL);
		r.run(1.0);
		check(r.heard("less than two position counts") && r.g.width < 100.0, "a lead below two position counts is raised to two, and says so");
	}
	{
		Rig r;
		r.start();
		r.g.ignoresRequests = true;
		r.hold(FULL);
		r.run(0.3);
		r.letGo();
		r.run(1.5);
		check(r.heardTimes("did not take the last position request") == 1, "a gripper that answers a request and does not take it: reported, once");
		r.g.ignoresRequests = false;

		Rig f;
		f.start();
		f.g.frozen = true;
		f.hold(FULL);
		f.run(4.0);
		check(f.heardTimes("reports it is moving") == 1, "a gripper that reports moving and stands still: reported after 2 s, once");

		Rig m;
		m.g.neverArrives = true;  // after its activation it is fully open, and goes on reporting "in motion"
		m.run(8.0);
		check(m.ready() && m.g.activations == 1, "a gripper that keeps reporting \"in motion\" after its activation: taken as at rest once its position stands still");

		Rig d;
		d.start();
		d.g.droppedWrites = 2;
		d.hold(FULL);
		d.run(1.0);
		check(d.heardTimes("got no proper answer from the gripper") == 1 && d.g.width < 110.0 && d.g.connects == 2,
				"two requests lost on the line: said once, sent again; the port is left alone");
	}
	{
		Rig r;
		r.start();
		r.hold(FULL);
		r.run(0.5);
		const int opens = r.g.connects, activations = r.g.activations;
		const size_t saidThen = r.said.size();
		r.g.outOfStep = true;
		r.run(1.0);
		check(r.heard("out of step", saidThen) && r.g.connects == opens + 1 && !r.g.outOfStep,
				"a stale answer on the line (every request then seems to fail): after three the port is closed and opened");
		check(r.ready() && r.g.activations == activations && r.heard("open again and the gripper is still activated", saidThen),
				"... and the teleop goes on without a restart");
		const double w0 = r.g.width;
		r.run(0.5);
		check(w0 - r.g.width > 20.0, "... the stick moving the gripper as before");
	}
}

}  // namespace

int main() {
	testStart();
	testStick();
	testObject();
	testEndStops();
	testOpenFully();
	testNoJoystick();
	testRestartHold();
	testFaults();
	testSilence();
	testRestartsThatFail();
	testOddOnes();
	check(requestsToUnreadyGripper == 0, strf("in all of the above, %d position requests went to a gripper that was not activated, ready and free of faults",
			requestsToUnreadyGripper));
	std::printf("%d checks, %d failed\n", checks, failures);
	return failures == 0 ? 0 : 1;
}
