#pragma once

/* A Robotiq 2F gripper teleoperated from the navigation controller: what to
 * ask of the gripper next, decided from the joystick, the gripper's status
 * and the time. No ROS and no gripper library in here (tested offline:
 * test/test_robotiq_teleop.cpp); src/robotiq_teleop.cpp is the node around
 * it, with libgripper's RobotiqGripper on the serial port.
 *
 * Controls, as for the BarrettHand (hand_teleop.h): hold L2 (the deadman)
 * and push the stick up to close, down to open, at a speed proportional to
 * the stick; let go and the opening stays. L2 + cross opens fully, once per
 * press. L2 + circle held for a second restarts the gripper.
 *
 * The gripper takes positions, not velocities: one request holds the opening
 * to go to and the speed and force to use on the way. So the stick moves a
 * target opening, and every change of it by one of the gripper's 255
 * position counts goes out as a request with a fixed speed and force. The
 * gripper's own speed has to be above the stick's for the opening to follow.
 *
 * While the stick moves the target, the target never gets more than `lead`
 * ahead of the measured opening:
 *   - Closed on an object (or on the fingers' own stop) the target stops
 *     `lead` past the contact. The request stays beyond the object, so the
 *     gripper keeps its force on it, and follows it by at most `lead` if it
 *     gives way (the gripper's re-grasp); and the target is never far away
 *     when the operator opens.
 *   - A gripper slower than the stick is not sent far ahead: when the stick
 *     is let go it has at most `lead` left to travel.
 * This goes by the measured opening, not by the gripper's object flag (gOBJ),
 * which the manual says can miss a grasp. Reversing the stick starts from the
 * measured opening, so opening responds at once.
 *
 * A request goes out only in a step whose status read arrived and showed the
 * gripper activated, ready and without a fault. Its health otherwise:
 *   SILENT      no status for silentAfter: not connected yet, the serial
 *               adapter gone, the gripper without power. Nothing is sent;
 *               the port is closed and opened again every reconnectEvery.
 *               When it answers and is still activated, the teleop goes on
 *               from the opening it reads, without a restart.
 *   FAULT       it answers, but reports a fault or is not activated. Faults
 *               that need no reset (over temperature, no communication for a
 *               second) are waited out. The others, and a gripper that is
 *               not activated, need a restart, which waits for a request
 *               unless autoRestart is set: activation runs the fingers
 *               through their travel, and whatever they hold falls.
 *   RESTARTING  close the port, open it, reset, activate, wait until the
 *               gripper is ready and at rest; then its measured opening is
 *               the target.
 * A gripper found activated and ready when the program starts is taken as it
 * is, for the same reason. One that is not (as after it was powered on) waits
 * for a restart like any other, until its fingers have room, unless
 * autoActivate is set.
 *
 * One more thing can go wrong on the serial line without the gripper being
 * at fault. libgripper reads each answer as a fixed number of bytes and never
 * empties the line, so one answer that comes after its 0.2 s time-out stays
 * there, and from then on every request is paired with the answer to the one
 * before: status reads still look right (they are one read old), but every
 * position request seems to fail, although the gripper carries it out. Three
 * of those in a row with the status still arriving are taken as that, and the
 * port is closed and opened (closing a serial port empties it); the teleop
 * then goes on as after a silence, without a restart. */

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <string>
#include <vector>

namespace robotiq_teleop {

/* gFLT, the fault code in the gripper's status, and what the manual says of
 * each (2F-85 & 2F-140 instruction manual, 4.4.3 "Fault status"). */
struct Fault {
	enum Kind {
		NONE,
		PASSES,  // priority and minor faults: no reset needed, they clear with their cause
		RESET,   // major faults, and "not activated": a reset and a new activation
	};
	static Kind kind(int code) {
		switch (code) {
		case 0x00: return NONE;
		case 0x05: case 0x08: case 0x09: return PASSES;
		default: return RESET;  // 0x07, 0x0A-0x0F; a code the manual does not list is not waited out
		}
	}
	static const char* meaning(int code) {
		switch (code) {
		case 0x00: return "no fault";
		case 0x05: return "action delayed: the activation must be completed first";
		case 0x07: return "the activation bit must be set first (the gripper is not activated)";
		case 0x08: return "maximum operating temperature exceeded (85 C inside): let it cool down below 80 C";
		case 0x09: return "no communication for at least 1 second";
		case 0x0A: return "under minimum operating voltage";
		case 0x0B: return "automatic release in progress";
		case 0x0C: return "internal fault (the manual: contact support@robotiq.com)";
		case 0x0D: return "activation fault: verify that no interference or other error occurred";
		case 0x0E: return "overcurrent triggered";
		case 0x0F: return "automatic release completed";
		default: return "a code the 2F manual does not list";
		}
	}
};

struct Params {
	double stroke = 140.0;           // mm, the opening at position count 0 (2F-140)
	double stickSpeed = 50.0;        // mm/s the target opening moves at full stick
	double deviceSpeed = 0.3;        // 0-1, the gripper's own speed in every request (2F-140: 30 to 250 mm/s)
	double force = 0.5;              // 0-1, its force in every request (2F-140: 10 to 125 N)
	double lead = 5.0;               // mm the stick moves the target ahead of the measured opening at most
	double deadband = 0.05;          // stick deflection that counts as centred
	double joyTimeout = 0.5;         // s without a joystick message: hold
	double restartHold = 1.0;        // s L2 + circle has to be held for a restart
	double silentAfter = 0.5;        // s without a status before the gripper counts as not answering
	double reconnectEvery = 1.0;     // s between attempts to open the serial port again
	double activationTimeout = 10.0; // s from the activation command until ready and at rest
	double passesWithin = 5.0;       // s a fault that needs no reset may last before a restart is suggested
	double autoRestartEvery = 10.0;  // s between automatic restarts at least
	double stillAfter = 2.0;         // s "moving" without moving before it is reported
	bool autoActivate = false;       // at the start, activate a gripper that is not activated without asking
	bool autoRestart = false;        // restart without asking whenever a restart is needed
};

/* The latest joystick message. */
struct Joy {
	bool valid = false;    // one has arrived
	double stamp = 0.0;    // s, when (the clock of Core::step())
	bool deadman = false;  // L2
	bool open = false;     // cross
	bool restart = false;  // circle
	double axis = 0.0;     // stick y: +1 up (close), -1 down (open)
};

/* One read of the gripper's status registers. */
struct Status {
	bool valid = false;      // the read arrived
	bool activated = false;  // gACT
	bool ready = false;      // gSTA = 3, activation completed
	bool moving = false;     // going to the request and not stopped (gGTO = 1, gOBJ = 0)
	int object = 0;          // gOBJ: 0 in motion, 1 stopped by contact opening, 2 closing, 3 at the request
	int fault = 0;           // gFLT
	int echo = 0;            // gPR, the request as the gripper has it, counts
	int position = 0;        // gPO, counts: 0 open, 255 closed
	int current = 0;         // gCU, about 10 mA each
	double width = 0.0;      // mm, the opening for gPO
};

/* What step() wants done before the next step; result() takes how it went. */
struct Action {
	enum Kind {
		NONE,
		MOVE,        // a position request: width, speed, force
		DISCONNECT,  // close the serial port
		CONNECT,     // open it
		RESET,       // clear the activation bit: also clears the faults
		ACTIVATE,    // set it and wait for the activation (libgripper's activate() waits up to 4 s)
	};
	Kind kind = NONE;
	double width = 0.0;  // mm
	double speed = 0.0;  // 0-1
	double force = 0.0;  // 0-1
};

enum State { SILENT, FAULT, RESTARTING, READY };

inline const char* stateName(State s) {
	switch (s) {
	case SILENT: return "silent";
	case FAULT: return "fault";
	case RESTARTING: return "restarting";
	default: return "ready";
	}
}

/* The gripper's position count for an opening, and back: 0 is fully open,
 * 255 fully closed, the same straight line as libgripper's rawFromWidth(). */
inline int countFromWidth(double width, double stroke) {
	const double w = std::max(0.0, std::min(stroke, width));
	return static_cast<int>(std::lround((1.0 - w / stroke) * 255.0));
}
inline double widthFromCount(int count, double stroke) {
	return (1.0 - count / 255.0) * stroke;
}

class Core {
public:
	explicit Core(const Params& p = Params()) : params(p) {
		const double twoCounts = 2.0 * params.stroke / 255.0;
		if (params.lead < twoCounts) {
			// With less, the target could not get a whole count ahead of the
			// opening, and nothing would ever be sent.
			say("lead %.2f mm is less than two position counts: using %.2f mm.", params.lead, twoCounts);
			params.lead = twoCounts;
		}
		params.deviceSpeed = clamp(params.deviceSpeed, 0.0, 1.0);
		params.force = clamp(params.force, 0.0, 1.0);
		params.stickSpeed = std::max(0.0, params.stickSpeed);
	}

	/* A restart asked for from outside the joystick (the node's service).
	 * False when one is running or waiting already. It starts in the next
	 * step(), from any state. */
	bool requestRestart(const std::string& who) {
		if (state == RESTARTING || restartWanted) {
			return false;
		}
		restartWanted = true;
		restartWho = who;
		return true;
	}

	/* One step, every 20 to 35 ms: now in seconds on any steady clock, the
	 * latest joystick message, and a status read made for this step. At most
	 * one action comes back; call result() once it is done. */
	Action step(double now, const Joy& joy, const Status& status) {
		const double dt = stepped ? clamp(now - lastStep, 0.0, MAX_STEP) : 0.0;
		if (!stepped) {
			firstStep = now;
			lastValid = now;
		}
		lastStep = now;
		stepped = true;
		pending = Action();

		if (status.valid) {
			last = status;
			lastValid = now;
			answered = true;
		}
		const Input in = readJoystick(now, joy);
		if (restartWanted && state != RESTARTING) {
			beginRestart(now);
		}
		switch (state) {
		case SILENT: silentStep(now, status); break;
		case FAULT: faultStep(now, status, in); break;
		case RESTARTING: restartStep(now, status); break;
		case READY: readyStep(now, dt, status, joy, in); break;
		}
		return pending;
	}

	/* How the action of the last step() went. */
	void result(bool ok, double now) {
		const Action done = pending;
		pending = Action();
		switch (done.kind) {
		case Action::MOVE:
			if (ok) {
				sentCount = pendingCount;
				lastMove = now;
				++moves;
				moveFailed = false;
				moveFailures = 0;
			} else {
				++moveFailures;  // the next step tries again: the count still differs
				if (!moveFailed) {
					moveFailed = true;
					say("a position request got no proper answer from the gripper; trying again.");
				}
			}
			break;
		case Action::DISCONNECT:
			if (state == RESTARTING) {
				phase = OPEN;
			}
			break;
		case Action::CONNECT:
			if (state == RESTARTING) {
				if (ok) {
					phase = CLEAR;
				} else {
					failRestart(now, "the serial port does not open (the adapter unplugged? the port's name?)");
				}
			} else {
				reopen = WAITING;
				nextReopen = now + params.reconnectEvery;
				++attempts;
				if (!portKnown || ok != portOpens) {
					say(ok ? "the serial port is open; waiting for the gripper to answer."
							: "the serial port does not open (the adapter unplugged? the port's name?); trying every %g s.",
							params.reconnectEvery);
				}
				portKnown = true;
				portOpens = ok;
			}
			break;
		case Action::RESET:
			if (ok) {
				phase = ACTIVATE;
			} else if (!resetRetried) {
				resetRetried = true;  // once more through the port, in case the line was not clean
				phase = CLOSE;
			} else {
				failRestart(now, "the serial port opens, but the gripper did not answer the reset, twice "
						"(is it powered? the right port and slave id?)");
			}
			break;
		case Action::ACTIVATE:
			// Whatever the library's call returned, the status decides: its
			// wait is 4 s long, and an activation may take longer.
			phase = WAIT;
			waitFrom = now;
			restPosition = -1;
			break;
		default:
			break;
		}
	}

	/* What happened since the last call, one plain sentence each: the
	 * operator's pane. */
	std::vector<std::string> takeNotes() {
		std::vector<std::string> out;
		out.swap(notes);
		return out;
	}

	/* Where things stand, in one line (the node's status service). */
	std::string describe(double now) const {
		std::string s;
		switch (state) {
		case READY:
			s = strf("ready, opening %.1f mm, target %.1f mm%s", last.width, target, holding ? ", in contact" : "");
			break;
		case FAULT:
			s = "NOT USABLE: " + troubleText(last);
			break;
		case RESTARTING:
			s = strf("restarting (%.1f s so far)", now - restartBegan);
			break;
		default:
			s = answered ? strf("NOT ANSWERING for %.1f s", now - lastValid) : std::string("NOT CONNECTED yet");
			if (portKnown) {
				s += portOpens ? " (the serial port opens)" : " (the serial port does not open)";
			}
			break;
		}
		s += joyFresh ? "; controller connected" : "; controller not connected";
		s += strf("; %u restart%s", restartsDone, restartsDone == 1 ? "" : "s");
		if (restartsDone > 0) {
			s += lastRestartOk ? " (the last one worked)" : " (the last one failed)";
		}
		if (answered) {
			s += strf("; last status %.1f s ago: %s", now - lastValid, statusText(last).c_str());
		}
		return s;
	}

	State getState() const { return state; }
	double getTarget() const { return target; }      // mm; meaningful when READY
	bool inContact() const { return holding; }
	bool joystickFresh() const { return joyFresh; }
	const Status& lastStatus() const { return last; }  // the last one that arrived
	const Params& getParams() const { return params; }
	/* Restarts finished, whichever way, and how the last one ended. */
	unsigned restartsFinished() const { return restartsDone; }
	bool lastRestartWorked() const { return lastRestartOk; }
	const std::string& lastRestartNote() const { return restartNote; }

	static std::string statusText(const Status& s) {
		return strf("activated %s, ready %s, gOBJ %d, fault 0x%02X, request echo %d, position %d (%.1f mm), motor current %d mA",
				s.activated ? "yes" : "NO", s.ready ? "yes" : "NO", s.object, s.fault, s.echo, s.position, s.width,
				s.current * 10);
	}

	// A step counts this much time at most, so that a loop held up (by the
	// library's wait in an activation, a serial timeout) does not jump the target.
	static constexpr double MAX_STEP = 0.1;       // s
	// A request the gripper's echo does not show after this long was not taken.
	static constexpr double ECHO_WITHIN = 1.0;    // s
	// Without a controller from the start, say so once after this long.
	static constexpr double CONTROLLER_WITHIN = 3.0;  // s
	// While not answering: a reminder this often.
	static constexpr double REMIND_EVERY = 30.0;  // s
	// After an activation, a position that has stood this long is at rest.
	static constexpr double REST_AFTER = 0.5;     // s
	// This many position requests in a row without a proper answer, with the
	// status still arriving: the line is out of step. The port is closed and
	// opened for it at most every REOPEN_EVERY.
	static constexpr int OUT_OF_STEP = 3;
	static constexpr double REOPEN_EVERY = 10.0;  // s

private:
	enum Trouble { FINE, WAITS, NEEDS_RESTART };
	enum Phase { CLOSE, OPEN, CLEAR, ACTIVATE, WAIT };
	enum Reopen { WAITING, CLOSE_NEXT, OPEN_NEXT };
	struct Input {
		bool fresh;      // a joystick message within joyTimeout
		bool openPress;  // L2 + cross went down in this step
	};

	static double clamp(double x, double lo, double hi) {
		return x < lo ? lo : (x > hi ? hi : x);
	}
	static std::string strf(const char* format, ...) {
		char buf[640];
		va_list ap;
		va_start(ap, format);
		vsnprintf(buf, sizeof(buf), format, ap);
		va_end(ap);
		return buf;
	}
	void say(const char* format, ...) {
		char buf[640];
		va_list ap;
		va_start(ap, format);
		vsnprintf(buf, sizeof(buf), format, ap);
		va_end(ap);
		notes.push_back(buf);
	}

	static Trouble trouble(const Status& s) {
		if (Fault::kind(s.fault) == Fault::RESET || !s.activated) {
			return NEEDS_RESTART;
		}
		if (s.fault != 0 || !s.ready) {
			return WAITS;  // a fault that passes, or an activation still running
		}
		return FINE;
	}
	static std::string troubleText(const Status& s) {
		std::string t;
		if (s.fault != 0) {
			t = strf("fault 0x%02X, %s", s.fault, Fault::meaning(s.fault));
		}
		if (!s.activated) {
			t += t.empty() ? "" : "; ";
			t += "not activated (in reset: it lost its power, or something reset it)";
		} else if (!s.ready && s.fault == 0) {
			t += "its activation is still running";
		}
		return t;
	}
	// What a status would make a FAULT note say: a change of it is a new note.
	static int signature(const Status& s) {
		return s.fault | (s.activated ? 0x100 : 0) | (s.ready ? 0x200 : 0);
	}

	// The stick's deflection beyond the deadband, -1 to 1.
	double stick(double axis) const {
		if (!std::isfinite(axis)) {
			return 0.0;
		}
		const double a = clamp(axis, -1.0, 1.0);
		if (std::fabs(a) <= params.deadband) {
			return 0.0;
		}
		return (a > 0.0 ? 1.0 : -1.0) * (std::fabs(a) - params.deadband) / (1.0 - params.deadband);
	}

	Input readJoystick(double now, const Joy& joy) {
		Input in;
		in.fresh = joy.valid && now - joy.stamp <= params.joyTimeout;
		in.openPress = false;
		if (in.fresh && !joyFresh) {
			say("navigation controller connected: L2 + stick up/down closes/opens, L2 + X opens fully, "
					"L2 + circle held %g s restarts the gripper.", params.restartHold);
		} else if (!in.fresh && joyFresh) {
			say("no joystick message for %g s (the controller off or out of range): holding.", params.joyTimeout);
		} else if (!in.fresh && !joySeen && !joyMissed && now - firstStep > CONTROLLER_WITHIN) {
			joyMissed = true;
			say("no navigation controller yet (no joystick messages): turn it on (PS); the gripper holds meanwhile.");
		}
		joyFresh = in.fresh;
		joySeen = joySeen || in.fresh;
		if (!in.fresh) {
			// What was held when the controller went away is not held any more.
			openHeld = false;
			circleSince = -1.0;
			circleFired = false;
			return in;
		}
		const bool open = joy.deadman && joy.open;
		in.openPress = open && !openHeld;  // once per press, however long it is held
		openHeld = open;
		if (!(joy.deadman && joy.restart)) {
			circleSince = -1.0;
			circleFired = false;
		} else if (circleSince < 0.0) {
			circleSince = now;
		} else if (!circleFired && now - circleSince >= params.restartHold) {
			circleFired = true;  // one restart per hold
			if (!requestRestart(strf("L2 + circle held %g s", params.restartHold))) {
				say("L2 + circle: a restart is running already.");
			}
		}
		return in;
	}

	// ---- READY -----------------------------------------------------------

	void readyStep(double now, double dt, const Status& status, const Joy& joy, const Input& in) {
		if (!status.valid) {
			if (now - lastValid > params.silentAfter) {
				goSilent(now);
			}
			return;  // nothing goes out on an old reading
		}
		if (trouble(status) != FINE) {
			enterFault(now, status);
			return;
		}
		if (moveFailures >= OUT_OF_STEP && now - lastReopen >= REOPEN_EVERY) {
			reopenLine(now);
			return;
		}
		const double actual = status.width;
		const double count = params.stroke / 255.0;
		noteContact(status);

		if (in.openPress) {
			target = params.stroke;
			say("L2 + X: opening fully.");
		} else if (in.fresh && joy.deadman) {
			const double a = stick(joy.axis);
			if (a > 0.0) {  // up: close
				if (target > actual + count) {
					target = actual;  // from where the gripper is, not from where the target was left
				}
				target = std::max(target - a * params.stickSpeed * dt, std::min(target, actual - params.lead));
			} else if (a < 0.0) {  // down: open
				if (target < actual - count) {
					target = actual;
				}
				target = std::min(target - a * params.stickSpeed * dt, std::max(target, actual + params.lead));
			}
		} else if (!in.fresh) {
			// The controller is gone: stop what is under way (a full opening).
			// A grip stays: its target is within the lead already.
			target = clamp(target, actual - params.lead, actual + params.lead);
		}
		target = clamp(target, 0.0, params.stroke);

		const int c = countFromWidth(target, params.stroke);
		if (c != sentCount) {
			pending.kind = Action::MOVE;
			pending.width = widthFromCount(c, params.stroke);
			pending.speed = params.deviceSpeed;
			pending.force = params.force;
			pendingCount = c;
			if (holding && (heldBy == 2 ? pending.width >= heldAt : pending.width <= heldAt)) {
				holding = false;  // the operator let go
			}
			return;
		}
		watch(now, status);
	}

	// Contact, as the gripper reports it, once per grasp: where it closed on
	// something, and when that gave way.
	void noteContact(const Status& s) {
		const bool contact = s.object == 1 || s.object == 2;
		if (contact && !holding) {
			holding = true;
			heldBy = s.object;
			heldAt = s.width;
			say(s.object == 2 ? "closing stopped by contact at %.1f mm: holding." : "opening stopped by contact at %.1f mm.",
					s.width);
		} else if (holding && s.object == 3) {
			// At the request, which was beyond the contact: what was held is not there any more.
			holding = false;
			say("no contact any more: the fingers reached the requested opening (%.1f mm), "
					"so what they held gave way or slipped out.", s.width);
		}
	}

	// A gripper that looks healthy and does not do as asked: say so once,
	// with the numbers, each time it starts.
	void watch(double now, const Status& s) {
		if (moves > 0 && s.echo != sentCount && now - lastMove > ECHO_WITHIN) {
			if (!echoNoted) {
				echoNoted = true;
				say("the gripper did not take the last position request: %d was sent %.1f s ago, its echo says %d "
						"(%s). If it stays so, restart it.", sentCount, now - lastMove, s.echo, statusText(s).c_str());
			}
		} else {
			echoNoted = false;
		}
		if (s.position != watched || !s.moving) {
			watched = s.position;
			watchedSince = now;
			stillNoted = false;
		} else if (!stillNoted && now - watchedSince > params.stillAfter) {
			stillNoted = true;
			say("the gripper reports it is moving to count %d but has stayed at %d (%.1f mm) for %g s (%s). "
					"If it stays so, restart it.", s.echo, s.position, s.width, params.stillAfter, statusText(s).c_str());
		}
	}

	// From here the teleop goes on: the measured opening is the target, and
	// nothing is sent until the stick moves it.
	void adopt(const Status& s) {
		state = READY;
		atStart = false;
		target = s.width;
		sentCount = countFromWidth(target, params.stroke);
		moves = 0;
		moveFailed = false;
		moveFailures = 0;
		echoNoted = false;
		stillNoted = false;
		watched = s.position;
		holding = false;
	}

	// ---- SILENT ----------------------------------------------------------

	// The status arrives and the position requests get no proper answer: the
	// answers are out of step with the requests (the note at the top). The
	// port is closed and opened; then it goes on as after a silence.
	void reopenLine(double now) {
		say("%d position requests in a row got no proper answer while the status still arrives: the answers on the "
				"serial line are out of step with the requests (an answer came late, or broken). Closing and opening "
				"the port to clear the line.", moveFailures);
		moveFailures = 0;
		lastReopen = now;
		state = SILENT;
		afterRestart = false;
		lineCleared = true;
		reopen = CLOSE_NEXT;  // whatever the next status says
		reminded = now;
		portKnown = true;     // the port opens: no word about it this time
		portOpens = true;
	}

	void goSilent(double now) {
		say("the gripper is NOT ANSWERING: no status for %.1f s. Nothing is sent to it; the serial port is closed and "
				"opened again every %g s. Its last status: %s.", now - lastValid, params.reconnectEvery,
				statusText(last).c_str());
		state = SILENT;
		afterRestart = false;
		lineCleared = false;
		reopen = WAITING;
		nextReopen = now;
		reminded = now;
		portKnown = false;
	}

	void silentStep(double now, const Status& status) {
		if (reopen == OPEN_NEXT) {
			pending.kind = Action::CONNECT;
			return;
		}
		if (reopen == CLOSE_NEXT) {
			pending.kind = Action::DISCONNECT;
			reopen = OPEN_NEXT;
			return;
		}
		if (status.valid) {
			const bool again = everReady;
			if (trouble(status) == FINE) {
				adopt(status);
				everReady = true;
				say(lineCleared ? "the serial port is open again and the gripper is still activated: ready at %.1f mm. No restart, nothing moved."
						: again ? "the gripper answers again and is still activated: ready at %.1f mm. No restart, nothing moved."
						: "connected: the gripper is activated and ready, at %.1f mm. It was not activated again, so nothing moved.",
						status.width);
				lineCleared = false;
			} else {
				if (again && !afterRestart && !lineCleared) {
					say("the gripper answers again.");
				}
				lineCleared = false;
				enterFault(now, status);
			}
			return;
		}
		if (now >= nextReopen) {
			pending.kind = Action::DISCONNECT;
			reopen = OPEN_NEXT;
			return;
		}
		if (now - reminded > REMIND_EVERY) {
			reminded = now;
			say("still no answer from the gripper after %.0f s (%d tries; the serial port %s).", now - lastValid, attempts,
					!portKnown ? "was not tried yet" : portOpens ? "opens: is the gripper powered?" : "does not open");
		}
	}

	// ---- FAULT -----------------------------------------------------------

	void enterFault(double now, const Status& s) {
		state = FAULT;
		faultSignature = signature(s);
		faultSince = now;
		overdueNoted = false;
		const Trouble t = trouble(s);
		const std::string what = troubleText(s);
		if (t == WAITS) {
			say("the gripper reports %s. This needs no restart: waiting for it to clear; nothing is sent meanwhile. (%s)",
					what.c_str(), statusText(s).c_str());
		} else if (atStart && params.autoActivate) {
			say("the gripper reports %s. (%s)", what.c_str(), statusText(s).c_str());
		} else if (atStart && !s.activated && s.fault == 0 && !params.autoRestart) {
			say("the gripper is not activated yet (as after it was powered on). A restart activates it: L2 + circle held "
					"%g s, or `wam gripper restart`, once its fingers have room. They run through their travel: whatever "
					"they hold falls. (%s)", params.restartHold, statusText(s).c_str());
		} else if (params.autoRestart) {
			say("the gripper reports %s. Nothing is sent to it; it is restarted automatically (auto_restart). (%s)",
					what.c_str(), statusText(s).c_str());
		} else {
			say("the gripper reports %s. Nothing is sent to it until it is restarted: L2 + circle held %g s, or "
					"`wam gripper restart`. A restart runs the fingers through their travel: whatever they hold falls. (%s)",
					what.c_str(), params.restartHold, statusText(s).c_str());
		}
	}

	void faultStep(double now, const Status& status, const Input& in) {
		if (!status.valid) {
			if (now - lastValid > params.silentAfter) {
				goSilent(now);
			}
			return;
		}
		const Trouble t = trouble(status);
		if (t == FINE) {
			adopt(status);
			everReady = true;
			say("the gripper reports no fault any more: ready at %.1f mm.", status.width);
			return;
		}
		if (signature(status) != faultSignature) {
			enterFault(now, status);
		}
		if (in.openPress) {
			say("L2 + X ignored: the gripper is not ready (%s).", troubleText(status).c_str());
		}
		bool restart = t == NEEDS_RESTART;
		if (t == WAITS && now - faultSince > params.passesWithin) {
			restart = true;
			if (!overdueNoted) {
				overdueNoted = true;
				say("it has not cleared in %g s (%s)%s", params.passesWithin, troubleText(status).c_str(),
						params.autoRestart || (atStart && params.autoActivate) ? "."
								: ": restart the gripper with L2 + circle or `wam gripper restart` if it stays.");
			}
		}
		if (!restart) {
			return;
		}
		if (atStart && params.autoActivate) {
			atStart = false;  // once: a start that fails is not tried again by itself
			requestRestart("activation at the start, auto_activate");
		} else if (params.autoRestart && now - lastAuto >= params.autoRestartEvery) {
			lastAuto = now;
			requestRestart("automatic, auto_restart");
		}
	}

	// ---- RESTARTING ------------------------------------------------------

	void beginRestart(double now) {
		restartWanted = false;
		atStart = false;
		state = RESTARTING;
		phase = CLOSE;
		resetRetried = false;
		restartBegan = now;
		say("RESTART (%s): closing and opening the serial port, then reset and activation. The fingers run through "
				"their travel: whatever they hold falls.", restartWho.c_str());
	}

	void restartStep(double now, const Status& status) {
		switch (phase) {
		case CLOSE:
			pending.kind = Action::DISCONNECT;
			break;
		case OPEN:
			pending.kind = Action::CONNECT;
			break;
		case CLEAR:
			pending.kind = Action::RESET;
			break;
		case ACTIVATE:
			pending.kind = Action::ACTIVATE;
			pending.speed = params.deviceSpeed;
			pending.force = params.force;
			activationStart = now;
			break;
		case WAIT:
			waitUntilReady(now, status);
			break;
		}
	}

	void waitUntilReady(double now, const Status& status) {
		if (!status.valid) {
			if (now - std::max(lastValid, waitFrom) > params.silentAfter) {
				failRestart(now, "the gripper stopped answering after the activation command");
			}
			return;
		}
		if (Fault::kind(status.fault) == Fault::RESET) {
			failRestart(now, strf("the gripper reports fault 0x%02X in its activation, %s", status.fault,
					Fault::meaning(status.fault)));
		} else if (!status.activated) {
			failRestart(now, "the activation command did not take: the gripper is still in reset");
		} else if (trouble(status) == FINE && atRest(now, status)) {
			adopt(status);
			everReady = true;
			++restartsDone;
			lastRestartOk = true;
			restartNote = strf("restarted in %.1f s: ready at %.1f mm.", now - restartBegan, status.width);
			notes.push_back(restartNote);
		} else if (now - activationStart > params.activationTimeout) {
			failRestart(now, strf("not ready and at rest %g s after the activation command (%s)", params.activationTimeout,
					statusText(status).c_str()));
		}
	}

	// After its activation the library sends the gripper fully open; the
	// target is taken once the fingers are there. At rest: the gripper says
	// so, or its position has not changed for REST_AFTER (in case it keeps
	// reporting "in motion" at a request it has already reached).
	bool atRest(double now, const Status& s) {
		if (s.position != restPosition) {
			restPosition = s.position;
			restSince = now;
		}
		return !s.moving || now - restSince >= REST_AFTER;
	}

	// The restart is over without a usable gripper. From here it is watched
	// like one that does not answer: the next status says what it is.
	void failRestart(double now, const std::string& why) {
		++restartsDone;
		lastRestartOk = false;
		restartNote = "RESTART FAILED: " + why + ".";
		notes.push_back(restartNote);
		state = SILENT;
		afterRestart = true;
		lineCleared = false;
		reopen = WAITING;
		nextReopen = now + params.reconnectEvery;
		reminded = now;
		portKnown = false;
	}

	Params params;
	State state = SILENT;
	Action pending;
	std::vector<std::string> notes;

	bool stepped = false;
	double firstStep = 0.0, lastStep = 0.0;
	Status last;              // the last status that arrived
	double lastValid = 0.0;   // when
	bool answered = false;    // one has arrived at all
	bool everReady = false;   // the teleop has run: a silence since is "again"
	bool atStart = true;      // until the first READY or the first restart

	// the joystick
	bool joyFresh = false, joySeen = false, joyMissed = false;
	bool openHeld = false;
	double circleSince = -1.0;
	bool circleFired = false;

	// READY
	double target = 0.0;      // mm
	int sentCount = 0;        // the request the gripper has, as far as known
	int pendingCount = 0;
	unsigned moves = 0;       // requests sent since READY began
	double lastMove = 0.0;
	bool moveFailed = false;
	int moveFailures = 0;     // in a row
	double lastReopen = -1e9; // when the port was last reopened for them
	bool holding = false;     // the gripper reported contact, and it was not let go since
	int heldBy = 0;           // gOBJ then: 1 opening, 2 closing
	double heldAt = 0.0;      // mm
	bool echoNoted = false, stillNoted = false;
	int watched = -1;
	double watchedSince = 0.0;

	// SILENT
	Reopen reopen = OPEN_NEXT;  // the first step opens the port
	double nextReopen = 0.0, reminded = 0.0;
	int attempts = 0;
	bool portKnown = false, portOpens = false;
	bool afterRestart = false;  // silent since a restart failed, not since the gripper stopped answering
	bool lineCleared = false;   // ... or since the port was reopened for answers out of step

	// FAULT
	int faultSignature = 0;
	double faultSince = 0.0;
	bool overdueNoted = false;
	double lastAuto = -1e9;

	// RESTARTING
	bool restartWanted = false;
	std::string restartWho;
	Phase phase = CLOSE;
	bool resetRetried = false;
	double restartBegan = 0.0, activationStart = 0.0, waitFrom = 0.0;
	int restPosition = -1;
	double restSince = 0.0;
	unsigned restartsDone = 0;
	bool lastRestartOk = false;
	std::string restartNote;
};

}  // namespace robotiq_teleop
