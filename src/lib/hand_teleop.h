#pragma once

/* Teleoperation of a BarrettHand (BH8-280/282) on the follower WAM's CAN bus
 * (follower --hand): the grasp by velocity, the spread in three poses.
 *
 * The hand's pucks (11-14) share the WAM's bus, so the follower has to be the
 * only program talking to them. libbarrett keeps every frame a puck sends
 * until it is read and throws "Buffer overflow" once ten unread frames with
 * one id pile up (BusManager::storeMessage); in the follower that exception
 * leaves the 500 Hz loop and ends the program. The hand's replies to another
 * program's requests (bhand_node polling the hand) are exactly such frames.
 *
 * Everything that moves the hand here is a velocity command (V, and MODE =
 * velocity), what the stick has always sent: update() runs at a fixed rate
 * outside the realtime loop and sends
 *   - V to a digit when its value changes,
 *   - MODE = velocity when a digit starts moving,
 *   - both again every refresh period while it moves, so a digit that
 *     something else idled resumes,
 *   - V = 0 to stop it; in velocity mode the puck then servos where it is,
 *     which is what holds a finger, and the spread (it is backdrivable).
 *
 * Grasp: the velocity in rad/s at the inner finger joints, e.g. from joy ->
 * joy_teleop -> bhand_mux -> follower/hand/velocity_setpoint.
 *
 * Spread: three poses, one step on per nextPose() (follower/hand/next_pose,
 * L2 + cross on the nav controller): 0% -> 50% -> 100% -> 0%.
 *   0%   (0)     F1 and F2 side by side, opposite F3; the grasp moves all three.
 *   50%  (pi/2)  F1 and F2 face each other, a pinch; the grasp moves F1 and
 *                F2, F3 stays open.
 *   100% (pi)    F1 and F2 turned round beside F3, all three closing the same
 *                way; the grasp moves all three.
 * A switch drives F1-F3 open, slowing near open (open fingers clear each
 * other at any spread; closed ones may not, manual 8.4); once all three read
 * open it drives the spread to the pose, slowing near it, and stops it there
 * (V = 0: held). Nothing else ever commands the spread, so it does not creep.
 * The fingers, once open, are idled (their worm drives hold them) rather than
 * pushed against their stops. The grasp waits meanwhile and comes back with
 * the fingers open, and the hand lets go of what it holds. One press is one
 * step: presses during a switch are ignored. A finger that stops short of
 * open (blocked), or a spread that does not move, ends the switch; the pose
 * stays.
 *
 * It never waits on the bus: libbarrett's receive() holds the bus lock while
 * it waits for a reply, so a hand puck that does not answer would stall the
 * arm's loop for up to a second. In a switch it follows the fingers and the
 * spread with non-blocking position requests: the hand group's request of
 * Hand::update() in one step, the four replies picked up in later ones, one
 * request out at a time. Setup checks once that the hand answers them; if it
 * does not, the poses are off and the grasp works as before.
 *
 * makeFist() gives the hand its start and stop pose (returnToStart() first
 * brings the spread back to 0% the same way as a switch); it waits on the bus,
 * so it runs only while the hand is not teleoperated. */

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <barrett/products/hand.h>
#include <barrett/products/motor_puck.h>
#include <barrett/products/puck.h>
#include <barrett/products/puck_group.h>

class HandTeleop {
public:
	typedef std::chrono::steady_clock Clock;

	struct Params {
		double maxGrasp = 2.0;       // rad/s at the inner finger joint
		double timeout = 0.1;        // s without a new command: stop
		double refresh = 0.25;       // s between V + MODE repeats while moving
		double poseGap = 0.12;       // s without a pose request that ends a press
		double switchSpeed = 2.0;    // rad/s at most in a switch, fingers and spread
		// A switch goes by progress: the fingers are open when they read open,
		// the spread there when it reads there, and a digit that stops short
		// (STALL_TIME) ends it. These only bound it.
		double openTimeout = 10.0;   // s for the fingers to open
		double spreadTimeout = 5.0;  // s for the spread to get to the pose
		double replyTimeout = 0.5;   // s for a hand puck to answer a position request
		// Set up right after makeFist() (startHandTeleop()): the fingers must
		// then read closed, else the hand's zero is off (HI skipped after a
		// power cycle: "open" would be the fist) and the poses stay off.
		bool afterFist = false;
		bool poses = true;  // false: the spread stays where it is (grasp only)
	};

	// What became of a pose request (nextPose()).
	enum Press {
		REPEAT,    // the buttons still held from the last press
		ACCEPTED,  // a switch to the next pose starts
		BUSY,      // a switch runs: ignored
		OFF,       // the poses are off
	};

	enum Event {
		NOTHING,
		TIMED_OUT,       // the command timed out with the fingers moving
		SWITCH_STARTED,  // the fingers open for a new pose (note())
		SPREAD_STARTED,  // they are open; the spread turns (note())
		SWITCH_DONE,     // the spread is at the pose, the grasp is back (note())
		SWITCH_FAILED,   // a finger did not open, or the spread did not move: the pose stays (note())
	};

	static const int POSES = 3;
	// The spread's target for a pose: 0, pi/2, pi, the ends 1 degree inside
	// the spread's stops (it holds there without pushing into them).
	static double poseAngle(int pose) {
		return pose == 0 ? END_MARGIN : pose == 1 ? M_PI / 2.0 : M_PI - END_MARGIN;
	}
	static int posePercent(int pose) { return pose * 50; }
	// The fingers the grasp moves (Hand::F1, ...).
	static unsigned int poseFingers(int pose) {
		return pose == 1 ? (barrett::Hand::F1 | barrett::Hand::F2) : barrett::Hand::GRASP;
	}
	static const char* poseName(int pose) {
		switch (pose) {
		case 0: return "0% (F1 and F2 opposite F3)";
		case 1: return "50% (pinch: F1 and F2, F3 stays open)";
		default: return "100% (all three fingers side by side)";
		}
	}

	// Reads the pucks' counts per revolution and the spread's HOLD and
	// position: setup only, not while the arm is teleoperated. The pose is the
	// one nearest the spread (0% after makeFist()); the spread is held there.
	HandTeleop(barrett::Hand& hand, const Params& params) :
		hand(hand), params(params), grasp(0.0), haveCommand(false), requested(0), phase(HOLDING), pose(0),
		target(0), spreadFrom(0.0), serial(0), holdWasOff(false), posesOn(false), framesSent(0)
	{
		const std::vector<barrett::Puck*>& handPucks = hand.getPucks();
		for (size_t i = 0; i < DIGITS; ++i) {
			pucks[i] = handPucks[i];
			// Hand::j2pp, motor counts per joint radian (Hand::J2_RATIO and
			// Hand::SPREAD_RATIO are protected).
			countsPerRad[i] = barrett::MotorPuck(pucks[i]).getCountsPerRad()
					* (i == SPREAD ? 17.5 : 125.0);
			positionId[i] = pucks[i]->getPropertyId(barrett::Puck::P);
		}
		sent.fill(0);
		pending.fill(false);
		have.fill(false);
		settled.fill(false);
		position.fill(0);

		// HOLD = 1 (the firmware default for the spread): makeFist()'s move of
		// the spread ends held, not let go.
		if (pucks[SPREAD]->getProperty(barrett::Puck::HOLD) == 0) {
			pucks[SPREAD]->setProperty(barrett::Puck::HOLD, 1);
			holdWasOff = true;
		}
		hand.update(barrett::Hand::S_POSITION);
		const double spread = hand.getInnerLinkPosition()[barrett::Hand::SPREAD_INDEX];
		pose = std::max(0, std::min(POSES - 1, static_cast<int>(std::lround(spread / (M_PI / 2.0)))));
		requested = pose;
		send(SPREAD, barrett::Puck::V, 0);  // held where it is
		send(SPREAD, barrett::Puck::MODE, barrett::MotorPuck::MODE_VELOCITY);

		// The switches follow the hand with non-blocking position requests:
		// check here, where waiting is allowed, that it answers them. If not,
		// the poses are off and the grasp works as before.
		posesOn = params.poses && pollOnce();
		if (!params.poses) {
			setupNote = "spread poses off (--no-spread-poses): the spread stays where it is, the grasp works";
		} else if (posesOn && params.afterFist && std::min(rad(0), std::min(rad(1), rad(2))) < FIST_LEAST) {
			posesOn = false;
			setupNote = strf("the fingers read %.2f, %.2f, %.2f rad right after closing to a fist: the hand's zero is "
					"off (HI skipped since it was powered on?), so the spread poses are off (the grasp works); "
					"restart the follower and let it initialize the hand", rad(0), rad(1), rad(2));
		} else if (posesOn) {
			setupNote = strf("spread poses on (L2 + X); at %s; the hand reads F1 %.2f, F2 %.2f, F3 %.2f, spread %.2f rad",
					poseName(pose), rad(0), rad(1), rad(2), rad(SPREAD));
		} else {
			setupNote = "the hand did not answer a position request within 0.5 s: the spread poses are off "
					"(the grasp works); restart the follower to try again";
		}
	}

	// The latest grasp command; any thread.
	void setCommand(double graspVelocity) {
		std::lock_guard<std::mutex> lock(mutex);
		grasp = std::isfinite(graspVelocity) ? graspVelocity : 0.0;
		stamp = Clock::now();
		haveCommand = true;
	}

	// One step on to the next pose; any thread. Held buttons repeat the
	// request with every joystick message (joy_teleop, 50 Hz), so a request
	// counts only after poseGap without one: once per press. A press during a
	// switch is ignored: one press, one step. The pose it asks for in *asked.
	Press nextPose(Clock::time_point now = Clock::now(), int* asked = NULL) {
		std::lock_guard<std::mutex> lock(mutex);
		const bool counted = now - lastRequest > seconds(params.poseGap);
		lastRequest = now;
		Press r = REPEAT;
		if (counted) {
			if (!posesOn) {
				r = OFF;
			} else if (phase != HOLDING || requested != pose) {
				r = BUSY;
			} else {
				requested = (requested + 1) % POSES;
				r = ACCEPTED;
			}
		}
		if (asked != NULL) {
			*asked = requested;
		}
		return r;
	}

	// One step (the hand thread, every 20 ms). Throws if a hand puck does not
	// answer a position request within replyTimeout.
	Event update(Clock::time_point now = Clock::now()) {
		double g = 0.0;
		bool fresh;
		int want;
		{
			std::lock_guard<std::mutex> lock(mutex);
			fresh = haveCommand && now - stamp <= seconds(params.timeout);
			if (fresh) {
				g = grasp;
			}
			want = requested;
		}
		if (posesOn) {
			collect(now);
		}
		if (phase == HOLDING && want != pose) {
			startSwitch(want, now);
			return SWITCH_STARTED;
		}
		if (phase != HOLDING) {
			return switchStep(want, now);
		}

		const bool timedOut = !fresh && moving();
		g = std::max(-params.maxGrasp, std::min(params.maxGrasp, g));
		const unsigned int fingers = poseFingers(pose);
		for (size_t i = 0; i < FINGERS; ++i) {
			apply(i, (fingers & (1u << i)) ? counts(i, g) : 0, now);
		}
		return timedOut ? TIMED_OUT : NOTHING;
	}

	// Back to 0% (the stop pose, before makeFist()): a switch, stepped here
	// until it ends. Waits; not while teleoperating. True when at 0%.
	bool returnToStart(double timeout_s = 10.0) {
		if (!posesOn || (phase == HOLDING && pose == 0)) {
			return pose == 0;
		}
		{
			std::lock_guard<std::mutex> lock(mutex);
			requested = 0;
			haveCommand = false;  // no grasp meanwhile
		}
		const Clock::time_point end = Clock::now() + seconds(timeout_s);
		while (Clock::now() < end) {
			update();
			if (phase == HOLDING) {
				int want;
				{
					std::lock_guard<std::mutex> lock(mutex);
					want = requested;
				}
				if (pose == 0 || want != 0) {  // there, or the switch gave up
					return pose == 0;
				}
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		}
		return false;
	}

	// Stops the hand where it is: V = 0 to what moves (velocity mode: each
	// holds there), also in a switch. Then collects the replies still due (up
	// to replyTimeout), so that no later reader of the hand (makeFist()) gets
	// them. Does not throw on a silent puck.
	void stop() {
		for (size_t i = 0; i < DIGITS; ++i) {
			if (sent[i] != 0) {
				send(i, barrett::Puck::V, 0);
				sent[i] = 0;
			}
		}
		if (phase != HOLDING) {
			phase = HOLDING;
			std::lock_guard<std::mutex> lock(mutex);
			requested = pose;
		}
		const Clock::time_point end = Clock::now() + seconds(params.replyTimeout);
		while (posesOn && awaitingReplies() && Clock::now() < end) {
			std::this_thread::sleep_for(std::chrono::milliseconds(2));
			try {
				collect(Clock::now());
			} catch (const std::exception&) {
				break;
			}
		}
	}

	bool moving() const {
		for (size_t i = 0; i < FINGERS; ++i) {
			if (sent[i] != 0) {
				return true;
			}
		}
		return false;
	}
	bool switching() const { return phase != HOLDING; }
	int getPose() const { return pose; }  // while switching, the one it left
	const std::string& note() const { return lastNote; }
	bool spreadHoldWasOff() const { return holdWasOff; }  // HOLD was 0: set to 1
	bool posesAvailable() const { return posesOn; }
	const std::string& setupReport() const { return setupNote; }  // for the log
	int sentCounts(size_t digit) const { return sent[digit]; }  // counts/ms
	long getFramesSent() const { return framesSent; }

	static const size_t DIGITS = 4;
	static const size_t FINGERS = 3;
	static const size_t SPREAD = 3;

	// A finger this close to open (rad) reads open: the spread may turn. The
	// fingers go on to FINAL_OPEN, or until they stop there.
	static constexpr double OPEN_TOLERANCE = 0.05;
	static constexpr double FINAL_OPEN = 0.01;
	// The spread stops this close to a pose; further than SPREAD_TOLERANCE
	// (stopped short: a stop, an object) is reported.
	static constexpr double SPREAD_STOP = 0.01;
	static constexpr double SPREAD_TOLERANCE = 0.05;
	// Speed toward a target, rad/s per rad left (so it slows near it), and the
	// least speed until it is there.
	static constexpr double FINGER_GAIN = 10.0;
	static constexpr double SPREAD_GAIN = 10.0;
	static constexpr double MIN_SPEED = 0.1;
	// A digit that moved less than STILL rad between two readings over
	// STALL_TIME s has stopped: a finger short of open is blocked, the spread
	// short of the pose is at a stop or held back.
	static constexpr double STILL = 0.002;
	static constexpr double STALL_TIME = 0.3;
	// The least a finger reads (rad, motor) in a fist when the zero is right.
	static constexpr double FIST_LEAST = 0.8;
	// How far inside its stops the spread's end poses are (rad).
	static constexpr double END_MARGIN = 0.02;

private:
	enum Phase { HOLDING, OPENING, SPREADING };

	static Clock::duration seconds(double s) {
		return std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(s));
	}
	static std::string strf(const char* format, ...) {
		char buf[256];
		va_list ap;
		va_start(ap, format);
		vsnprintf(buf, sizeof(buf), format, ap);
		va_end(ap);
		return buf;
	}

	// Puck velocity, counts per millisecond (Hand::velocityMove's units).
	int counts(size_t i, double velocity) const {
		return static_cast<int>(std::lround(countsPerRad[i] * velocity / 1000.0));
	}
	// The last reading, rad (inner finger joint; spread).
	double rad(size_t i) const { return position[i] / countsPerRad[i]; }
	// The speed for a distance left (rad): proportional, bounded.
	double approach(size_t i, double left) const {
		const double k = i == SPREAD ? SPREAD_GAIN : FINGER_GAIN;
		return std::max(MIN_SPEED, std::min(params.switchSpeed, k * left));
	}

	void apply(size_t i, int c, Clock::time_point now) {
		if (c == sent[i]) {
			if (c == 0 || now - lastSend[i] < seconds(params.refresh)) {
				return;
			}
			send(i, barrett::Puck::V, c);  // refresh
			send(i, barrett::Puck::MODE, barrett::MotorPuck::MODE_VELOCITY);
		} else {
			send(i, barrett::Puck::V, c);
			if (sent[i] == 0) {  // starting
				send(i, barrett::Puck::MODE, barrett::MotorPuck::MODE_VELOCITY);
			}
		}
		sent[i] = c;
		lastSend[i] = now;
	}

	void send(size_t i, enum barrett::Puck::Property prop, int value) {
		pucks[i]->setProperty(prop, value);  // one frame, no reply
		++framesSent;
	}

	// An open finger stops and is idled: no current, its worm drive holds it
	// (as after the fist); the next grasp command starts it again.
	void release(size_t i) {
		if (sent[i] != 0) {
			send(i, barrett::Puck::V, 0);
			sent[i] = 0;
		}
		send(i, barrett::Puck::MODE, barrett::MotorPuck::MODE_IDLE);
	}

	// The grasp stops; the fingers are driven open from the next reading on.
	void startSwitch(int want, Clock::time_point now) {
		for (size_t i = 0; i < FINGERS; ++i) {
			apply(i, 0, now);
		}
		settled.fill(false);
		++serial;  // readings from before are dropped
		have.fill(false);
		newPhase(OPENING, now);
		switchStart = now;
		ask(now);
		lastNote = strf("spread to %s; opening the fingers first", poseName(want));
	}

	Event switchStep(int want, Clock::time_point now) {
		Event e = NOTHING;
		if (have[0] && have[1] && have[2] && have[SPREAD]) {
			// The fingers: toward open until FINAL_OPEN, or until they stop
			// once open (the stop), then held.
			bool open = true;
			for (size_t i = 0; i < FINGERS; ++i) {
				const double p = rad(i);
				open = open && p <= OPEN_TOLERANCE;
				if (!settled[i] && (p <= FINAL_OPEN || (p <= OPEN_TOLERANCE && stalled(i)))) {
					settled[i] = true;
					release(i);
				}
				if (!settled[i]) {
					apply(i, -counts(i, approach(i, p)), now);
				}
			}
			if (phase == OPENING) {
				if (open) {
					if (want == pose) {  // pressed round to where it started
						return finish(SWITCH_DONE, strf("spread back at %s", poseName(pose)), now);
					}
					lastNote = strf("fingers open (%s rad) after %.1f s; turning the spread", fingerReadings().c_str(),
							std::chrono::duration<double>(now - switchStart).count());
					target = want;
					newPhase(SPREADING, now);
					spreadFrom = rad(SPREAD);
					e = SPREAD_STARTED;
				} else {
					for (size_t i = 0; i < FINGERS; ++i) {
						if (rad(i) > OPEN_TOLERANCE && stalled(i)) {
							return abandon(strf("F%zu stopped %.2f rad short of open", i + 1, rad(i)), now);
						}
					}
				}
			}
			if (phase == SPREADING) {
				// The spread: toward the pose until SPREAD_STOP, or until it
				// stops short, then held.
				const double left = poseAngle(target) - rad(SPREAD);
				if (std::fabs(left) > SPREAD_TOLERANCE && stalled(SPREAD)
						&& std::fabs(rad(SPREAD) - spreadFrom) < SPREAD_TOLERANCE) {
					apply(SPREAD, 0, now);  // it did not move at all: the pose stays
					return finish(SWITCH_FAILED, strf("the spread did not move (it reads %.0f degrees); it stays at %s",
							rad(SPREAD) * 180.0 / M_PI, poseName(pose)), now, true);
				}
				if (std::fabs(left) <= SPREAD_STOP || stalled(SPREAD)) {
					apply(SPREAD, 0, now);
					pose = target;
					if (want != pose) {  // pressed again meanwhile; the fingers are still open
						target = want;
						newPhase(SPREADING, now);
						spreadFrom = rad(SPREAD);
					} else {
						const double took = std::chrono::duration<double>(now - switchStart).count();
						if (std::fabs(left) <= SPREAD_TOLERANCE) {
							return finish(SWITCH_DONE, strf("spread at %s, %.1f s", poseName(pose), took), now);
						}
						return finish(SWITCH_DONE, strf("spread at %s, %.1f s; it stopped %.0f degrees %s of it",
								poseName(pose), took, std::fabs(left) * 180.0 / M_PI, left > 0.0 ? "short" : "past"),
								now);
					}
				} else {
					const int c = counts(SPREAD, approach(SPREAD, std::fabs(left)));
					apply(SPREAD, left > 0.0 ? c : -c, now);
				}
			}
		}
		if (phase == OPENING && now - phaseStart > seconds(params.openTimeout)) {
			return abandon(strf("the fingers did not open within %.0f s", params.openTimeout), now);
		}
		if (phase == SPREADING && now - phaseStart > seconds(params.spreadTimeout)) {
			apply(SPREAD, 0, now);
			pose = target;
			return finish(SWITCH_DONE, strf("spread at %s? it did not get there within %.0f s (at %.0f degrees)",
					poseName(pose), params.spreadTimeout, rad(SPREAD) * 180.0 / M_PI), now, true);
		}
		ask(now);
		return e;
	}

	// Back to the grasp; the fingers stop where they are (a held stick moves
	// them again next step). cancel: forget the presses since (after a
	// failure); otherwise one that came meanwhile starts the next switch.
	Event finish(Event e, const std::string& what, Clock::time_point now, bool cancel = false) {
		for (size_t i = 0; i < FINGERS; ++i) {
			apply(i, 0, now);
		}
		phase = HOLDING;
		lastNote = what;
		if (cancel) {
			std::lock_guard<std::mutex> lock(mutex);
			requested = pose;
		}
		return e;
	}
	// The fingers did not open: they stop, the spread stays, the grasp is back.
	Event abandon(const std::string& why, Clock::time_point now) {
		apply(SPREAD, 0, now);
		return finish(SWITCH_FAILED, strf("%s (%s rad); the spread stays at %s", why.c_str(), fingerReadings().c_str(),
				poseName(pose)), now, true);
	}
	std::string fingerReadings() const {
		std::string r;
		for (size_t i = 0; i < FINGERS; ++i) {
			r += (i ? " " : "") + (have[i] ? strf("F%zu %.2f", i + 1, rad(i)) : strf("F%zu ?", i + 1));
		}
		return r;
	}

	// One round of the non-blocking requests, waiting for it (setup only).
	bool pollOnce() {
		++serial;
		have.fill(false);
		ask(Clock::now());
		while (awaitingReplies()) {
			std::this_thread::sleep_for(std::chrono::milliseconds(2));
			try {
				collect(Clock::now());
			} catch (const std::exception&) {
				return false;  // no answer within replyTimeout, or a bad one
			}
		}
		return have[0] && have[1] && have[2] && have[SPREAD];
	}

	// Stall detection restarts with each phase.
	void newPhase(Phase p, Clock::time_point now) {
		phase = p;
		phaseStart = now;
		movedAt.fill(now);
	}

	// The hand group's position request (as Hand::update()): one frame, a
	// reply from each puck. Only when every reply to the last one is in.
	void ask(Clock::time_point now) {
		if (awaitingReplies()) {
			return;
		}
		hand.getPuckGroup().sendGetPropertyRequest(positionId[0]);  // throws if it cannot send
		++framesSent;
		for (size_t i = 0; i < DIGITS; ++i) {
			pending[i] = true;
			askedAt[i] = now;
			askedIn[i] = serial;
		}
	}

	// Picks up the replies that have arrived, without waiting. A reply to a
	// request from before the switch is read and dropped.
	void collect(Clock::time_point now) {
		for (size_t i = 0; i < DIGITS; ++i) {
			if (!pending[i]) {
				continue;
			}
			int p = 0;
			const int r = barrett::Puck::receiveGetPropertyReply<barrett::MotorPuck::MotorPositionParser<int> >(
					pucks[i]->getBus(), pucks[i]->getId(), positionId[i], &p, false, false);
			if (r == 1) {  // not yet
				if (now - askedAt[i] > seconds(params.replyTimeout)) {
					throw std::runtime_error(strf("hand puck %d did not answer within %.1f s", pucks[i]->getId(),
							params.replyTimeout));
				}
				continue;
			}
			pending[i] = false;
			if (r != 0) {
				throw std::runtime_error(strf("hand puck %d: bad position reply (%d)", pucks[i]->getId(), r));
			}
			if (askedIn[i] != serial) {
				continue;
			}
			if (have[i] && std::abs(p - position[i]) > STILL * countsPerRad[i]) {
				movedAt[i] = now;
			}
			position[i] = p;
			have[i] = true;
			answeredAt[i] = now;
		}
	}

	bool awaitingReplies() const {
		return pending[0] || pending[1] || pending[2] || pending[3];
	}
	bool stalled(size_t i) const {
		return answeredAt[i] - movedAt[i] >= seconds(STALL_TIME);
	}

	barrett::Hand& hand;
	const Params params;
	std::array<barrett::Puck*, DIGITS> pucks;
	std::array<double, DIGITS> countsPerRad;
	std::array<int, DIGITS> positionId;
	std::array<int, DIGITS> sent;
	std::array<Clock::time_point, DIGITS> lastSend;

	std::mutex mutex;
	double grasp;
	Clock::time_point stamp;
	bool haveCommand;
	int requested;
	Clock::time_point lastRequest;

	std::atomic<Phase> phase;  // atomic: switching() and getPose() from other threads
	std::atomic<int> pose;
	int target;
	double spreadFrom;  // rad, where the spread's move began
	Clock::time_point phaseStart, switchStart;
	long serial;
	std::array<bool, DIGITS> pending, have, settled;
	std::array<long, DIGITS> askedIn;
	std::array<int, DIGITS> position;
	std::array<Clock::time_point, DIGITS> askedAt, answeredAt, movedAt;

	bool holdWasOff;
	bool posesOn;
	std::string lastNote, setupNote;
	long framesSent;
};

// Whether the digits (Hand::GRASP, ...) have finished their moves: idle, or
// holding the end of one (MODE_PID: HOLD = 1). Hand::doneMoving() takes
// MODE_PID as done only if HOLD was 1 when the Hand was made, and HandTeleop
// sets the spread's HOLD to 1 if it was 0.
inline bool movesDone(barrett::Hand& hand, unsigned int digits) {
	int modes[barrett::Hand::DOF];
	hand.getPuckGroup().getProperty(barrett::Puck::MODE, modes);
	for (size_t i = 0; i < barrett::Hand::DOF; ++i) {
		if ((digits & (1u << i)) && modes[i] != barrett::MotorPuck::MODE_IDLE
				&& modes[i] != barrett::MotorPuck::MODE_PID) {
			return false;
		}
	}
	return true;
}

// Waits until the digits (Hand::GRASP, ...) stop moving, reading their modes
// every 0.1 s. False after timeout_s.
inline bool waitUntilDone(barrett::Hand& hand, unsigned int digits, double timeout_s) {
	const HandTeleop::Clock::time_point end = HandTeleop::Clock::now()
			+ std::chrono::duration_cast<HandTeleop::Clock::duration>(std::chrono::duration<double>(timeout_s));
	while (!movesDone(hand, digits)) {
		if (HandTeleop::Clock::now() > end) {
			return false;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	return true;
}

// The hand's start and stop pose: a fist, the spread at 0 (fingers 1 and 2 side
// by side) and the fingers closed. When the spread has to move, the fingers
// open first so they cannot sweep into each other (the hand lets go of what it
// holds). Blocking, and it reads the bus: not while teleoperating. False if a
// move did not finish within timeout_s.
inline bool makeFist(barrett::Hand& hand, double timeout_s = 10.0) {
	hand.update(barrett::Hand::S_POSITION);
	if (std::fabs(hand.getInnerLinkPosition()[barrett::Hand::SPREAD_INDEX]) > 0.05) {
		hand.open(barrett::Hand::GRASP, false);
		if (!waitUntilDone(hand, barrett::Hand::GRASP, timeout_s)) {
			return false;
		}
		hand.trapezoidalMove(barrett::Hand::jp_type(0.0), barrett::Hand::SPREAD, false);
		if (!waitUntilDone(hand, barrett::Hand::SPREAD, timeout_s)) {
			return false;
		}
	}
	hand.close(barrett::Hand::GRASP, false);
	return waitUntilDone(hand, barrett::Hand::GRASP, timeout_s);
}
