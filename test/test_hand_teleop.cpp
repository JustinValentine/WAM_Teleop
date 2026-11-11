// follower --hand without a robot.
//
// A simulated CAN bus carries a 7-DOF WAM (pucks 1-7) and a BarrettHand
// (pucks 11-14) speaking the Barrett protocol with libbarrett's property ids.
// The follower's side is libbarrett itself (BusManager, Puck, PuckGroup, Hand)
// over a CommunicationsBus to the simulator, plus a stand-in for the
// follower's 500 Hz loop doing on the bus what LowLevelWam does every cycle:
// a group position read and two packed-torque frames.
//
//   1. libbarrett's Hand setup and HI next to the arm's loop.
//   2. lib/hand_teleop.h, the grasp: speed, stop, command timeout, resume
//      after an external idle, traffic, no reads; the spread holds meanwhile.
//   3. The spread's three poses, a press each (L2 + cross): 0% -> 50% ->
//      100% -> 0%. The fingers are open whenever the spread moves; at 50%
//      the grasp moves F1 and F2 only; each pose holds against a push; held
//      buttons step once; a second press during a switch goes on from it;
//      the arm's loop never waits.
//   4. Switches that cannot finish: a blocked finger (the spread stays), a
//      hand puck that stops answering (the hand stops, the arm's loop does
//      not wait), the spread's HOLD found at 0; a hand slower and less exact
//      than the manual's (the waits go by progress); a hand that ignores CMD
//      open and trapezoidal moves (the switches use neither); the fingers
//      reading open right after the fist (the zero is off: the poses stay
//      off); setup's check of the position requests failing (the poses off,
//      the grasp on).
//   5. makeFist(), the start and stop pose: the fingers open before the
//      spread moves; a finger that does not finish ends it after its timeout.
//   6. Firmware 114-118, where V is property 106.
//   7. Why HandTeleop never waits on the bus: a hand puck that does not
//      answer holds the bus lock, and the arm's loop, for libbarrett's 1 s
//      timeout.
//   8. Why nothing else may poll the hand on this bus (gripper_ros's
//      bhand_node): the replies pile up in the follower's bus manager, which
//      throws after ten; the follower's realtime loop does not catch that.
//
// With -DWITH_LIBGRIPPER, test 6 also runs libgripper's BarrettHandCanDriver
// as the second program.

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "hand_teleop.h"
#include "sim_barrett_bus.h"

#ifdef WITH_LIBGRIPPER
#include <gripper/barrett/barrett_hand_can_driver.h>
#endif

using namespace sim;

namespace {

// The follower's hand thread (HandTeleopThread in hand_teleop_ros.h) without
// ROS.
class HandRunner {
public:
	explicit HandRunner(HandTeleop& teleop) :
		timeouts(0), started(0), done(0), failed(0), teleop(teleop), stopping(false)
	{
		thread = std::thread(&HandRunner::run, this);
	}
	~HandRunner() {
		stopping = true;
		thread.join();
	}
	std::atomic<int> timeouts, started, done, failed;
	std::string error() {
		std::lock_guard<std::mutex> lock(mutex);
		return err;
	}
	std::string lastNote() {
		std::lock_guard<std::mutex> lock(mutex);
		return note;
	}
	Clock::time_point errorAt;

private:
	void run() {
		const std::chrono::milliseconds period(20);
		Clock::time_point next = Clock::now();
		while (!stopping) {
			next += period;
			try {
				const HandTeleop::Event e = teleop.update();
				if (e == HandTeleop::SWITCH_STARTED || e == HandTeleop::SPREAD_STARTED || e == HandTeleop::SWITCH_DONE
						|| e == HandTeleop::SWITCH_FAILED) {
					std::lock_guard<std::mutex> lock(mutex);
					note = teleop.note();
					std::printf("       [hand] %s\n", note.c_str());
				}
				if (e == HandTeleop::TIMED_OUT) {
					++timeouts;
				} else if (e == HandTeleop::SWITCH_STARTED) {
					++started;
				} else if (e == HandTeleop::SWITCH_DONE) {
					++done;
				} else if (e == HandTeleop::SWITCH_FAILED) {
					++failed;
				}
			} catch (const std::exception& e) {
				std::lock_guard<std::mutex> lock(mutex);
				errorAt = Clock::now();
				err = e.what();
				return;
			}
			const Clock::time_point now = Clock::now();
			if (now < next) {
				std::this_thread::sleep_until(next);
			} else {
				next = now;
			}
		}
	}
	HandTeleop& teleop;
	std::atomic<bool> stopping;
	std::mutex mutex;
	std::string err, note;
	std::thread thread;
};

// The joystick side: a grasp command every 20 ms (joy_node's autorepeat,
// 50 Hz).
void stream(HandTeleop& teleop, double seconds, std::function<double(double)> grasp) {
	const Clock::time_point t0 = Clock::now();
	for (double t = 0.0; t < seconds; t = secondsSince(t0)) {
		teleop.setCommand(grasp(t));
		sleepMs(20);
	}
}
std::function<double(double)> constant(double grasp) {
	return [grasp](double) { return grasp; };
}

// L2 + cross held for the given time: joy_teleop publishes the request with
// every joystick message, 50 Hz; the grasp at 0 alongside it.
void press(HandTeleop& teleop, double seconds) {
	const Clock::time_point t0 = Clock::now();
	while (secondsSince(t0) < seconds) {
		teleop.nextPose();
		teleop.setCommand(0.0);
		sleepMs(20);
	}
}

// Waits up to timeout for f(); its time in s, or -1.
double waitFor(std::function<bool()> f, double timeout) {
	const Clock::time_point t0 = Clock::now();
	while (!f()) {
		if (secondsSince(t0) > timeout) {
			return -1.0;
		}
		sleepMs(2);
	}
	return secondsSince(t0);
}

// Frames to the spread's puck (V and MODE = velocity).
long spreadFrames(SimCan& sim) {
	return sim.with([](Pucks& p) { return p[14].vSets + p[14].modeVelocitySets; });
}
// Held by its puck: velocity mode at V = 0.
bool held(SimCan& sim, int id) {
	return sim.with([&](Pucks& p) { return p[id].mode == MotorPuck::MODE_VELOCITY && p[id].v == 0; });
}

void testSetupAndHi() {
	std::printf("1. libbarrett's Hand on the follower's bus, HI next to the arm's loop\n");
	Rig rig(175);
	rig.sim.with([](Pucks& p) {
		p[11].pos = 50000;  // not homed
		p[12].pos = 30000;
		p[13].pos = 1000;
		p[14].pos = 10000;
		return 0;
	});
	rig.startArm();
	sleepMs(300);
	rig.arm->mark();
	Clock::time_point t0 = Clock::now();
	try {
		rig.makeHand();
	} catch (const std::exception& e) {
		check(false, strf("Hand setup threw: %s", e.what()));
		return;
	}
	const double setup = secondsSince(t0);
	check(rig.hand->hasFingertipTorqueSensors(), strf("Hand set up in %.0f ms, fingertip torque sensors found", setup * 1000));
	t0 = Clock::now();
	std::string err;
	try {
		rig.hand->initialize();
	} catch (const std::exception& e) {
		err = e.what();
	}
	const double hi = secondsSince(t0);
	const bool homed = rig.sim.with([](Pucks& p) {
		for (int id = 11; id <= 14; ++id) {
			// at the end the fingers let go, the spread holds (HOLD 0 and 1)
			if (p[id].pos != 0.0 || p[id].mode != (id == 14 ? MotorPuck::MODE_PID : MotorPuck::MODE_IDLE)) {
				return false;
			}
		}
		return true;
	});
	check(err.empty() && homed, strf("Hand::initialize() (HI) homed all four in %.1f s, the spread held%s", hi,
			err.empty() ? "" : (": " + err).c_str()));
	const std::vector<double> t = rig.arm->snapshot();
	check(!rig.arm->failed, "the arm's loop ran through the setup and HI" + (rig.arm->failed ? ": " + rig.arm->error : std::string()));
	std::printf("       arm loop bus time meanwhile: %s\n", timing(t).c_str());
}

void testTeleop() {
	std::printf("2. HandTeleop (follower --hand), the grasp, next to the arm's 500 Hz loop\n");
	Rig rig(175);
	rig.startArm();
	rig.makeHand();
	sleepMs(200);
	rig.arm->mark();
	sleepMs(1000);
	const std::vector<double> baseline = rig.arm->snapshot();

	HandTeleop::Params params;
	HandTeleop teleop(*rig.hand, params);  // reads the pucks' CTS, HOLD, P: setup
	const bool spreadHeld = waitFor([&] { return held(rig.sim, 14); }, 0.1) >= 0.0;
	check(teleop.getPose() == 0 && spreadHeld, "setup: at spread 0%, which the spread's puck holds (velocity mode, V = 0)");
	const long spreadFrames0 = spreadFrames(rig.sim);
	const long repliesBefore = rig.sim.handReplies;
	const long violationsBefore = rig.sim.violations;
	const long handFramesBefore = rig.sim.handFrames;
	rig.arm->mark();
	int timeoutsInD = 0;
	{
		HandRunner runner(teleop);

		// a) grasp 1 rad/s for 1 s; the speed over the middle of it
		std::vector<std::pair<double, double> > samples;  // (s, counts)
		stream(teleop, 1.0, [&](double t) {
			samples.push_back(std::make_pair(t, rig.sim.pos(11)));
			return 1.0;
		});
		const double f1 = rig.sim.pos(11), f2 = rig.sim.pos(12), f3 = rig.sim.pos(13), sp = rig.sim.pos(14);
		size_t k0 = 0;
		while (k0 + 1 < samples.size() && samples[k0].first < 0.2) {
			++k0;
		}
		const double speed = (samples.back().second - samples[k0].second)
				/ (samples.back().first - samples[k0].first) / FINGER_COUNTS_PER_RAD;
		check(speed > 0.97 && speed < 1.01 && std::fabs(f2 - f1) < 0.01 * f1 && std::fabs(f3 - f1) < 0.01 * f1
				&& sp == 0.0,
				strf("grasp 1 rad/s: the fingers move at %.3f rad/s (V = 81 counts/ms = 0.994 rad/s), "
						"%.3f %.3f %.3f rad after 1 s, spread %.3f",
						speed, f1 / FINGER_COUNTS_PER_RAD, f2 / FINGER_COUNTS_PER_RAD, f3 / FINGER_COUNTS_PER_RAD,
						sp / SPREAD_COUNTS_PER_RAD));

		// b) zero stops them
		stream(teleop, 0.1, constant(0.0));
		const double z1 = rig.sim.pos(11);
		stream(teleop, 0.3, constant(0.0));
		check(rig.sim.pos(11) == z1 && rig.sim.mode(11) == MotorPuck::MODE_VELOCITY,
				"zero command: the fingers stop (V = 0, still in velocity mode)");

		// c) the spread: pushed (the object, the fingers) while the grasp moves
		rig.sim.with([](Pucks& p) {
			p[14].load = 5.0;  // counts/ms, 0.44 rad/s when nothing holds it
			return 0;
		});
		stream(teleop, 0.5, [](double t) { return t < 0.25 ? 1.0 : -1.0; });
		const double pushed = rig.sim.pos(14);
		rig.sim.with([](Pucks& p) {
			p[14].load = 0.0;
			return 0;
		});
		check(pushed == 0.0 && spreadFrames(rig.sim) == spreadFrames0,
				strf("the spread, pushed at 0.44 rad/s for 0.5 s while the fingers move: held at %.4f rad; "
						"nothing was sent to it (%ld frames)", pushed / SPREAD_COUNTS_PER_RAD,
						spreadFrames(rig.sim) - spreadFrames0));

		// d) the commands stop (deadman released, joystick or mux gone)
		const int timeoutsBefore = runner.timeouts;
		stream(teleop, 0.4, constant(-1.0));
		teleop.setCommand(-1.0);  // the last one
		const double atLast = rig.sim.pos(11);
		sleepMs(500);
		const double after = rig.sim.pos(11);
		const double movedMs = (atLast - after) / 81.0;  // V = 81 counts/ms at 1 rad/s
		timeoutsInD = runner.timeouts - timeoutsBefore;
		check(movedMs <= 150.0, strf("no more commands: the fingers stop %.0f ms after the last one (timeout 100 ms + one 20 ms step)", movedMs));
		check(timeoutsInD == 1, strf("the timeout is reported once (%d)", timeoutsInD));

		// e) a finger idled by something else resumes
		stream(teleop, 0.3, constant(0.5));
		rig.sim.with([](Pucks& p) {
			p[11].mode = MotorPuck::MODE_IDLE;
			return 0;
		});
		const Clock::time_point tIdle = Clock::now();
		double resumed = -1.0;
		for (int i = 0; i < 40 && resumed < 0.0; ++i) {
			teleop.setCommand(0.5);
			sleepMs(20);
			if (rig.sim.mode(11) == MotorPuck::MODE_VELOCITY) {
				resumed = secondsSince(tIdle);
			}
		}
		check(resumed >= 0.0 && resumed <= 0.3, strf("an idled finger resumes at the next refresh: %.0f ms", resumed * 1000));

		// f) traffic of a sweeping command
		const long sent0 = teleop.getFramesSent();
		const Clock::time_point tt = Clock::now();
		stream(teleop, 2.0, [](double t) { return 1.5 * std::sin(M_PI * t); });
		const double dur = secondsSince(tt);
		const double fps = (teleop.getFramesSent() - sent0) / dur;
		// ~110 bits per 6-byte frame with stuffing, 1 Mbit/s
		check(fps < 250.0, strf("sweeping grasp: %.0f frames/s from the hand thread, about %.1f%% of the bus",
				fps, fps * 110e-6 * 100));
		stream(teleop, 0.1, constant(0.0));
		const std::string error = runner.error();
		check(error.empty(), "the hand thread ran without errors" + (error.empty() ? std::string() : ": " + error));
	}
	teleop.stop();
	sleepMs(50);
	const double held = rig.sim.pos(11);
	sleepMs(100);
	const bool still = rig.sim.with([](Pucks& p) {
		for (int id = 11; id <= 14; ++id) {
			if (p[id].v != 0) {
				return false;
			}
		}
		return true;
	});
	check(still && !teleop.moving() && rig.sim.pos(11) == held, "stop(): V = 0 to the moving pucks, which hold still");
	check(rig.sim.handReplies == repliesBefore,
			strf("the hand sent nothing back while teleoperated (%ld frames to it, 0 reads)",
					rig.sim.handFrames - handFramesBefore));
	check(rig.sim.violations == violationsBefore, "every frame went to the hand (11-14, group 5) or the arm's ids");
	const std::vector<double> during = rig.arm->snapshot();
	check(!rig.arm->failed, "the arm's loop kept running" + (rig.arm->failed ? ": " + rig.arm->error : std::string()));
	std::printf("       arm loop bus time alone:     %s\n", timing(baseline).c_str());
	std::printf("       arm loop bus time with hand: %s\n", timing(during).c_str());
}

// The pucks' positions every 2 ms while it runs.
class Recorder {
public:
	explicit Recorder(SimCan& sim) : sim(sim), running(true) {
		thread = std::thread([this] {
			while (running) {
				samples.push_back(this->sim.with([](Pucks& p) {
					return std::vector<double>{p[11].pos, p[12].pos, p[13].pos, p[14].pos};
				}));
				sleepMs(2);
			}
		});
	}
	std::vector<std::vector<double> > stop() {
		running = false;
		thread.join();
		return samples;
	}

private:
	SimCan& sim;
	std::atomic<bool> running;
	std::thread thread;
	std::vector<std::vector<double> > samples;
};

// The most any finger was closed while the spread moved (rad).
double fingersWhileSpreadMoved(const std::vector<std::vector<double> >& s) {
	double most = 0.0;
	for (size_t k = 1; k < s.size(); ++k) {
		if (s[k][3] != s[k - 1][3]) {
			most = std::max(most, std::max(s[k][0], std::max(s[k][1], s[k][2])) / FINGER_COUNTS_PER_RAD);
		}
	}
	return most;
}

// The fingers closed, the spread at 0 (or where a switch to 0% stops it).
bool isFist(SimCan& sim) {
	return sim.with([](Pucks& p) {
		bool fist = std::fabs(p[14].pos) <= (HandTeleop::END_MARGIN + HandTeleop::SPREAD_STOP) * SPREAD_COUNTS_PER_RAD + 1.0;
		for (int id = 11; id <= 13; ++id) {
			fist = fist && std::fabs(p[id].pos - p[id].maxPos) < 1.0;
		}
		return fist;
	});
}

double spreadCounts(SimCan& sim) {
	return sim.pos(14);
}
double fingerRad(SimCan& sim, int id) {
	return sim.pos(id) / FINGER_COUNTS_PER_RAD;
}
// The fingers open: at most FINAL_OPEN from it, as a switch leaves them.
bool fingersOpen(SimCan& sim) {
	const double most = HandTeleop::FINAL_OPEN * FINGER_COUNTS_PER_RAD + 1.0;
	return sim.pos(11) <= most && sim.pos(12) <= most && sim.pos(13) <= most;
}

// The spread's targets, counts: 0% and 100% 1 degree inside its stops.
const double ZERO = HandTeleop::poseAngle(0) * SPREAD_COUNTS_PER_RAD;
const double PINCH = HandTeleop::poseAngle(1) * SPREAD_COUNTS_PER_RAD;
const double HOOK = HandTeleop::poseAngle(2) * SPREAD_COUNTS_PER_RAD;
// The spread at a pose: SPREAD_STOP (0.01 rad) from it at most.
bool spreadAt(SimCan& sim, double counts) {
	return std::fabs(sim.pos(14) - counts) <= HandTeleop::SPREAD_STOP * SPREAD_COUNTS_PER_RAD + 1.0;
}
double spreadDegrees(SimCan& sim) {
	return sim.pos(14) / SPREAD_COUNTS_PER_RAD * 180.0 / M_PI;
}

void testPoses() {
	std::printf("3. The spread's three poses, a press each (L2 + cross): 0%% -> 50%% -> 100%% -> 0%%\n");
	Rig rig(175);
	rig.startArm();
	rig.makeHand();
	makeFist(*rig.hand, 10.0);  // as the follower starts: the fingers closed, the spread at 0
	HandTeleop teleop(*rig.hand, HandTeleop::Params());
	const long repliesBefore = rig.sim.handReplies;
	rig.arm->mark();
	{
		HandRunner runner(teleop);

		// a) from the fist, one press held for 0.3 s
		Recorder r1(rig.sim);
		Clock::time_point t0 = Clock::now();
		press(teleop, 0.3);
		bool arrived = waitFor([&] { return runner.done >= 1; }, 5.0) >= 0.0;
		double took = secondsSince(t0);
		sleepMs(100);
		double closed = fingersWhileSpreadMoved(r1.stop());
		check(arrived && runner.started == 1 && runner.done == 1 && teleop.getPose() == 1,
				strf("from the fist, a press held for 0.3 s (15 requests): one switch, to 50%% in %.2f s "
						"(the fingers open from the fist at up to 2 rad/s, then the spread turns)", took));
		check(closed <= HandTeleop::OPEN_TOLERANCE, strf("the fingers were open whenever the spread moved (at most %.3f rad)", closed));
		check(spreadAt(rig.sim, PINCH) && held(rig.sim, 14) && fingersOpen(rig.sim),
				strf("at 50%%: the spread at %.2f degrees, held (velocity mode, V = 0); the fingers open",
						spreadDegrees(rig.sim)));

		// b) at 50% the grasp moves F1 and F2; F3 stays open
		const double f3Before = fingerRad(rig.sim, 13);
		stream(teleop, 0.5, constant(1.0));
		stream(teleop, 0.1, constant(0.0));
		const double a1 = fingerRad(rig.sim, 11), a2 = fingerRad(rig.sim, 12), a3 = fingerRad(rig.sim, 13);
		check(a1 > 0.4 && a2 > 0.4 && std::fabs(a1 - a2) < 0.01 && a3 == f3Before && a3 <= HandTeleop::FINAL_OPEN,
				strf("at 50%%, grasp 1 rad/s for 0.5 s: F1 %.2f rad, F2 %.2f rad, F3 %.2f rad (stays open): a pinch", a1, a2, a3));

		// c) the pose holds, pushed both ways while the fingers move
		const double pinchAt = spreadCounts(rig.sim);
		const long framesAt50 = spreadFrames(rig.sim);
		const double loads[2] = {5.0, -5.0};
		for (int k = 0; k < 2; ++k) {
			rig.sim.with([&](Pucks& p) {
				p[14].load = loads[k];
				return 0;
			});
			stream(teleop, 0.4, [](double t) { return t < 0.2 ? -0.5 : 0.5; });
		}
		rig.sim.with([](Pucks& p) {
			p[14].load = 0.0;
			return 0;
		});
		check(spreadCounts(rig.sim) == pinchAt && spreadFrames(rig.sim) == framesAt50,
				strf("at 50%%, pushed at 0.44 rad/s each way for 0.8 s while F1 and F2 move: the spread stays at "
						"%.2f degrees; nothing was sent to it", spreadDegrees(rig.sim)));

		// d) 50% -> 100% with the stick held at close all along
		const double before = fingerRad(rig.sim, 11);
		Recorder r2(rig.sim);
		t0 = Clock::now();
		for (int i = 0; i < 5; ++i) {
			teleop.nextPose();
			teleop.setCommand(1.0);
			sleepMs(20);
		}
		while (runner.done < 2 && secondsSince(t0) < 5.0) {
			teleop.setCommand(1.0);
			sleepMs(20);
		}
		took = secondsSince(t0);
		stream(teleop, 0.3, constant(1.0));
		stream(teleop, 0.1, constant(0.0));
		closed = fingersWhileSpreadMoved(r2.stop());
		check(teleop.getPose() == 2 && spreadAt(rig.sim, HOOK) && held(rig.sim, 14),
				strf("50%% -> 100%% with the stick held at close: there in %.2f s, held", took));
		check(closed <= HandTeleop::OPEN_TOLERANCE,
				strf("F1 and F2 (at %.2f rad) opened first, and the held close waited: the fingers were at most "
						"%.3f rad closed while the spread moved", before, closed));
		const double b1 = fingerRad(rig.sim, 11), b2 = fingerRad(rig.sim, 12), b3 = fingerRad(rig.sim, 13);
		check(b1 > 0.2 && std::fabs(b1 - b2) < 0.01 && std::fabs(b1 - b3) < 0.01,
				strf("at 100%% the grasp is back and moves all three: %.2f %.2f %.2f rad", b1, b2, b3));

		// e) 100% -> 0%: the spread through 180 degrees
		Recorder r3(rig.sim);
		t0 = Clock::now();
		press(teleop, 0.1);
		arrived = waitFor([&] { return runner.done >= 3; }, 5.0) >= 0.0;
		took = secondsSince(t0);
		closed = fingersWhileSpreadMoved(r3.stop());
		check(arrived && teleop.getPose() == 0 && spreadAt(rig.sim, ZERO) && closed <= HandTeleop::OPEN_TOLERANCE,
				strf("100%% -> 0%%: the spread through 180 degrees in %.2f s, the fingers open throughout (at most %.3f rad)",
						took, closed));

		// f) two quick presses: one press, one step
		Recorder r4(rig.sim);
		t0 = Clock::now();
		const HandTeleop::Press first = teleop.nextPose();
		sleepMs(250);
		const HandTeleop::Press second = teleop.nextPose();
		arrived = waitFor([&] { return !teleop.switching(); }, 5.0) >= 0.0;
		took = secondsSince(t0);
		closed = fingersWhileSpreadMoved(r4.stop());
		check(first == HandTeleop::ACCEPTED && second == HandTeleop::BUSY && arrived && teleop.getPose() == 1
				&& spreadAt(rig.sim, PINCH) && closed <= HandTeleop::OPEN_TOLERANCE,
				strf("two quick presses (0.25 s apart) from 0%%: the second, during the switch, is ignored; at 50%% "
						"in %.2f s", took));

		// g) a press held through the whole switch: one step
		const int doneBefore = runner.done;
		press(teleop, 1.5);
		waitFor([&] { return !teleop.switching(); }, 5.0);
		sleepMs(100);
		check(teleop.getPose() == 2 && runner.done == doneBefore + 1,
				strf("L2 + cross held for 1.5 s: one step (50%% -> 100%%), %d switch", runner.done - doneBefore));

		const std::string error = runner.error();
		check(error.empty(), "the hand thread ran without errors" + (error.empty() ? std::string() : ": " + error));
	}
	const std::vector<double> during = rig.arm->snapshot();
	check(!rig.arm->failed && percentile(during, 1.0) < 20.0,
			strf("the arm's loop never waited on the hand: its bus time %s, with %ld position replies polled",
					timing(during).c_str(), rig.sim.handReplies - repliesBefore));
	teleop.stop();
	const bool fist = makeFist(*rig.hand, 10.0);
	check(fist && isFist(rig.sim), "at the end, makeFist() from 0%: a fist");
}

void testPoseFailures() {
	std::printf("4. Switches that cannot finish\n");
	{
		Rig rig(175);
		rig.startArm();
		rig.makeHand();
		rig.sim.with([](Pucks& p) {  // the fingers part closed, F2 on something that blocks it
			p[11].pos = p[12].pos = p[13].pos = 60000;
			p[12].stuck = true;
			return 0;
		});
		HandTeleop teleop(*rig.hand, HandTeleop::Params());
		HandRunner runner(teleop);
		const Clock::time_point t0 = Clock::now();
		press(teleop, 0.1);
		const bool gaveUp = waitFor([&] { return runner.failed >= 1; }, 5.0) >= 0.0;
		const double took = secondsSince(t0);
		sleepMs(400);
		check(gaveUp && teleop.getPose() == 0 && spreadCounts(rig.sim) == 0.0 && runner.done == 0 && held(rig.sim, 11)
				&& held(rig.sim, 13),
				strf("F2 blocked short of open: the switch gives up after %.2f s (F2 still for %.2f s); the spread "
						"never moved; F1 and F3 stopped where they were", took, HandTeleop::STALL_TIME));
		rig.sim.with([](Pucks& p) {
			p[12].stuck = false;
			return 0;
		});
		stream(teleop, 0.3, constant(-1.0));
		check(rig.sim.pos(12) < 40000.0, strf("then the grasp is back: F2 opens (%.2f rad)", fingerRad(rig.sim, 12)));
	}
	{
		Rig rig(175);
		rig.startArm();
		rig.makeHand();
		HandTeleop teleop(*rig.hand, HandTeleop::Params());
		rig.arm->mark();
		{
			HandRunner runner(teleop);
			rig.sim.with([](Pucks& p) {
				p[13].mute = true;
				return 0;
			});
			const Clock::time_point t0 = Clock::now();
			press(teleop, 0.1);
			waitFor([&] { return !runner.error().empty(); }, 3.0);
			const std::string error = runner.error();
			const double took = std::chrono::duration<double>(runner.errorAt - t0).count();
			const std::vector<double> during = rig.arm->snapshot();
			check(error.find("did not answer") != std::string::npos && took > 0.45 && took < 0.8
					&& spreadCounts(rig.sim) == 0.0,
					strf("F3 silent in a switch: the hand thread stops after %.2f s (%s); the spread never moved",
							took, error.c_str()));
			check(!rig.arm->failed && percentile(during, 1.0) < 20.0,
					strf("meanwhile the arm's loop never waited: its longest bus time %.2f ms (a blocking read: "
							"about 1000 ms, test 7)", percentile(during, 1.0)));
		}
		teleop.stop();  // gives up on F3's reply after 0.5 s
	}
	{
		Rig rig(175);
		rig.sim.with([](Pucks& p) {
			p[14].props[pid(Puck::HOLD, 175)] = 0;
			return 0;
		});
		rig.startArm();
		rig.makeHand();  // libbarrett's Hand notes HOLD = 0
		HandTeleop teleop(*rig.hand, HandTeleop::Params());
		const int hold = rig.sim.with([](Pucks& p) { return p[14].props[pid(Puck::HOLD, 175)]; });
		check(teleop.spreadHoldWasOff() && hold == 1, "the spread's HOLD found at 0 (it would let go after each move): set to 1");
		{
			HandRunner runner(teleop);
			press(teleop, 0.1);
			waitFor([&] { return runner.done >= 1; }, 5.0);
			rig.sim.with([](Pucks& p) {
				p[14].load = 5.0;
				return 0;
			});
			sleepMs(500);
			rig.sim.with([](Pucks& p) {
				p[14].load = 0.0;
				return 0;
			});
		}
		check(teleop.getPose() == 1 && spreadAt(rig.sim, PINCH) && held(rig.sim, 14),
				strf("then at 50%% and pushed for 0.5 s: held at %.2f degrees", spreadDegrees(rig.sim)));
		teleop.stop();
		check(!rig.hand->doneMoving(barrett::Hand::SPREAD),
				"libbarrett's Hand::doneMoving() (HOLD 0 when the Hand was made) takes the held spread as moving");
		const Clock::time_point t0 = Clock::now();
		const bool fist = makeFist(*rig.hand, 3.0);
		check(fist && isFist(rig.sim),
				strf("makeFist() (movesDone(): held counts as done) still finishes, from 50%%: %.1f s", secondsSince(t0)));
	}
}

// A hand slower and less exact than the manual's: velocity mode tops out at
// 0.6 rad/s for the fingers and 0.7 rad/s for the spread, trapezoidal moves
// (the fist) ramp and end short, and the fingers' position replies carry the
// inner link encoder.
void testSlowHand() {
	std::printf("   a slower, less exact hand\n");
	Rig rig(175);
	rig.sim.with([](Pucks& p) {
		for (int id = 11; id <= 13; ++id) {
			p[id].maxVel = 50.0;  // counts/ms in velocity mode: 0.6 rad/s
			p[id].speed = 60.0;
			p[id].accel = 0.5;
			p[id].stopShort = 300.0;
			p[id].secondary = true;
		}
		p[14].maxVel = 8.0;  // 0.7 rad/s
		return 0;
	});
	rig.startArm();
	rig.makeHand();
	const bool fist = makeFist(*rig.hand, 10.0);
	HandTeleop::Params params;
	params.afterFist = true;
	HandTeleop teleop(*rig.hand, params);
	check(fist && teleop.posesAvailable() && teleop.setupReport().find("F1 2.44") != std::string::npos,
			"setup: " + teleop.setupReport());
	HandRunner runner(teleop);
	Recorder r(rig.sim);
	const Clock::time_point t0 = Clock::now();
	press(teleop, 0.1);
	const bool arrived = waitFor([&] { return runner.done >= 1; }, 15.0) >= 0.0;
	const double took = secondsSince(t0);
	const double closed = fingersWhileSpreadMoved(r.stop());
	check(arrived && teleop.getPose() == 1 && spreadAt(rig.sim, PINCH) && closed <= HandTeleop::OPEN_TOLERANCE,
			strf("from the fist to 50%% in %.1f s, the spread at %.2f degrees, the fingers at most %.3f rad closed "
					"while it moved", took, spreadDegrees(rig.sim), closed));
	const std::string error = runner.error();
	check(error.empty(), "no errors" + (error.empty() ? std::string() : ": " + error));
}

// A hand that does nothing on CMD open or a trapezoidal move: the switches
// and the way back to the fist use neither (velocity commands, what the stick
// sends, and CMD close, the startup fist's).
void testOnlyVelocity() {
	std::printf("   a hand that ignores CMD open and trapezoidal moves\n");
	Rig rig(175);
	rig.startArm();
	rig.makeHand();
	const bool fist = makeFist(*rig.hand, 10.0);
	rig.sim.with([](Pucks& p) {
		for (int id = 11; id <= 14; ++id) {
			p[id].ignoresOpen = true;
			p[id].ignoresTrapezoidal = true;
		}
		return 0;
	});
	HandTeleop::Params params;
	params.afterFist = true;
	std::unique_ptr<HandTeleop> teleop(new HandTeleop(*rig.hand, params));
	bool cycle = fist && teleop->posesAvailable();
	std::string where;
	{
		HandRunner runner(*teleop);
		Recorder r(rig.sim);
		const double poses[3] = {PINCH, HOOK, ZERO};
		for (int k = 0; k < 3; ++k) {
			press(*teleop, 0.1);
			const bool ok = waitFor([&] { return runner.done >= k + 1 && !teleop->switching(); }, 10.0) >= 0.0;
			cycle = cycle && ok && spreadAt(rig.sim, poses[k]);
			where += strf(" %.0f", spreadDegrees(rig.sim));
		}
		press(*teleop, 0.1);  // to 50% again, where the follower's end finds it
		waitFor([&] { return runner.done >= 4 && !teleop->switching(); }, 10.0);
		const double closed = fingersWhileSpreadMoved(r.stop());
		check(cycle && closed <= HandTeleop::OPEN_TOLERANCE && runner.error().empty(),
				strf("0%% -> 50%% -> 100%% -> 0%%: the spread at%s degrees; the fingers at most %.3f rad closed while it "
						"moved", where.c_str(), closed));
	}
	// the follower's end (~HandTeleopThread): back to 0%, the fist, idle
	teleop->stop();
	const bool back = teleop->returnToStart();
	teleop->stop();
	const bool fist2 = makeFist(*rig.hand, 10.0);
	check(back && fist2 && isFist(rig.sim),
			strf("at the end, from 50%%: the spread back to 0%% (%.2f degrees), then the fist (CMD close)",
					spreadDegrees(rig.sim)));
}

// A visitor: L2 + X pressed ten times in 2 s while working the stick the whole
// time. One switch at a time, the fingers open whenever the spread moves, the
// grasp back after each switch, no errors, the arm's loop never waits.
void testVisitor() {
	std::printf("   a visitor mashing L2 + X while working the stick\n");
	Rig rig(175);
	rig.startArm();
	rig.makeHand();
	makeFist(*rig.hand, 10.0);
	HandTeleop::Params params;
	params.afterFist = true;
	HandTeleop teleop(*rig.hand, params);
	rig.arm->mark();
	int accepted = 0, busy = 0;
	{
		HandRunner runner(teleop);
		Recorder r(rig.sim);
		const Clock::time_point t0 = Clock::now();
		int step = 0;
		while (secondsSince(t0) < 12.0) {
			const double t = secondsSince(t0);
			// the stick: up and down, 2 rad/s either way, all along
			teleop.setCommand(std::sin(2.0 * M_PI * 0.7 * t) > 0.0 ? 2.0 : -2.0);
			// L2 + X: a 60 ms tap every 200 ms for the first 2 s, then at 6 s
			const bool tap = (t < 2.0 && step % 10 < 3) || (t > 6.0 && t < 6.06);
			if (tap) {
				const HandTeleop::Press p = teleop.nextPose();
				accepted += p == HandTeleop::ACCEPTED;
				busy += p == HandTeleop::BUSY;
			}
			++step;
			sleepMs(20);
		}
		teleop.setCommand(0.0);
		waitFor([&] { return !teleop.switching(); }, 10.0);
		const double closed = fingersWhileSpreadMoved(r.stop());
		const std::string error = runner.error();
		const int pose = teleop.getPose();
		const double at = pose == 0 ? ZERO : pose == 1 ? PINCH : HOOK;
		check(accepted == runner.done + runner.failed && runner.failed == 0 && accepted >= 2 && busy >= 5
				&& spreadAt(rig.sim, at) && closed <= HandTeleop::OPEN_TOLERANCE && error.empty(),
				strf("%d presses taken, %d ignored during switches; %d switches, one at a time; at %d%%; the fingers "
						"at most %.3f rad closed while the spread moved%s", accepted, busy, runner.done.load(),
						HandTeleop::posePercent(pose), closed, error.empty() ? "" : (": " + error).c_str()));
	}
	const std::vector<double> t = rig.arm->snapshot();
	check(!rig.arm->failed && percentile(t, 1.0) < 20.0,
			"the arm's loop never waited: " + timing(t));
}

// The stick held at close through a press: the switch opens the fingers
// anyway (the grasp waits), and once it is done the held stick closes them in
// the new pose (at 50%: F1 and F2).
void testCloseThroughSwitch() {
	std::printf("   the stick held at close through a press\n");
	Rig rig(175);
	rig.startArm();
	rig.makeHand();
	makeFist(*rig.hand, 10.0);
	HandTeleop::Params params;
	params.afterFist = true;
	HandTeleop teleop(*rig.hand, params);
	HandRunner runner(teleop);
	Recorder r(rig.sim);
	teleop.nextPose();
	const Clock::time_point t0 = Clock::now();
	while (runner.done < 1 && secondsSince(t0) < 8.0) {
		teleop.setCommand(2.0);
		sleepMs(20);
	}
	const double atDone = fingerRad(rig.sim, 11);
	stream(teleop, 0.3, constant(2.0));
	stream(teleop, 0.1, constant(0.0));
	const double closed = fingersWhileSpreadMoved(r.stop());
	const double f1 = fingerRad(rig.sim, 11), f2 = fingerRad(rig.sim, 12), f3 = fingerRad(rig.sim, 13);
	check(runner.done == 1 && closed <= HandTeleop::OPEN_TOLERANCE && atDone <= HandTeleop::OPEN_TOLERANCE
			&& f1 > 0.4 && f2 > 0.4 && f3 <= HandTeleop::FINAL_OPEN,
			strf("the fingers opened (%.2f rad at the end of the switch) and stayed open while the spread moved (at "
					"most %.3f rad); then the held stick closed F1 %.2f, F2 %.2f; F3 %.2f stays open", atDone, closed,
					f1, f2, f3));
}

// The spread does not move at all (stuck, or deaf to its commands): the
// switch fails and the pose stays, so the grasp keeps its fingers.
void testSpreadStuck() {
	std::printf("   the spread does not move\n");
	Rig rig(175);
	rig.startArm();
	rig.makeHand();
	HandTeleop teleop(*rig.hand, HandTeleop::Params());
	rig.sim.with([](Pucks& p) {
		p[14].stuck = true;
		return 0;
	});
	HandRunner runner(teleop);
	press(teleop, 0.1);
	const bool gaveUp = waitFor([&] { return runner.failed >= 1; }, 5.0) >= 0.0;
	stream(teleop, 0.3, constant(1.0));
	stream(teleop, 0.1, constant(0.0));
	check(gaveUp && teleop.getPose() == 0 && runner.done == 0 && fingerRad(rig.sim, 13) > 0.2 && held(rig.sim, 14)
			&& runner.lastNote().find("did not move") != std::string::npos,
			strf("the switch fails, the pose stays 0%% (the grasp still moves F3: %.2f rad), the spread held: %s",
					fingerRad(rig.sim, 13), runner.lastNote().c_str()));
}

// Set up right after the fist, but the fingers read open: the hand's zero is
// off (HI skipped after a power cycle, when it was left in a fist). Driving
// the fingers "open" would leave them closed, so the poses stay off.
void testWrongZero() {
	std::printf("   the fingers read open right after the fist (the hand's zero is off)\n");
	Rig rig(175);
	rig.startArm();
	rig.makeHand();  // the fingers read 0
	HandTeleop::Params params;
	params.afterFist = true;
	HandTeleop teleop(*rig.hand, params);
	check(!teleop.posesAvailable() && teleop.setupReport().find("zero is off") != std::string::npos,
			"setup: " + teleop.setupReport());
}

// Setup finds that the hand does not answer the non-blocking position
// requests: the poses are off, and the grasp works.
void testPosesOff() {
	std::printf("   setup's check of the position requests fails\n");
	Rig rig(175);
	rig.startArm();
	rig.makeHand();
	rig.sim.with([](Pucks& p) {
		p[13].positionAnswersLeft = 1;  // the setup's Hand::update(), then no more
		return 0;
	});
	HandTeleop teleop(*rig.hand, HandTeleop::Params());
	check(!teleop.posesAvailable() && teleop.setupReport().find("poses are off") != std::string::npos,
			"setup: " + teleop.setupReport());
	HandRunner runner(teleop);
	press(teleop, 0.1);
	sleepMs(300);
	stream(teleop, 0.3, constant(1.0));
	stream(teleop, 0.1, constant(0.0));
	const std::string error = runner.error();
	check(runner.started == 0 && spreadCounts(rig.sim) == 0.0 && fingerRad(rig.sim, 13) > 0.2 && error.empty(),
			strf("L2 + X then does nothing, the grasp still works (F3 %.2f rad), no errors%s", fingerRad(rig.sim, 13),
					error.empty() ? "" : (": " + error).c_str()));
}

void testFist() {
	std::printf("5. makeFist(): the hand's start and stop pose (spread 0, fingers closed)\n");
	Rig rig(175);
	rig.startArm();
	rig.makeHand();

	// after HI: fingers open, spread at 0
	Recorder r1(rig.sim);
	Clock::time_point t0 = Clock::now();
	bool ok = makeFist(*rig.hand, 10.0);
	double took = secondsSince(t0);
	std::vector<std::vector<double> > s1 = r1.stop();
	bool spreadMoved = false;
	for (size_t k = 1; k < s1.size(); ++k) {
		spreadMoved = spreadMoved || s1[k][3] != s1[k - 1][3];
	}
	check(ok && isFist(rig.sim) && !spreadMoved,
			strf("after HI (fingers open, spread 0): only the fingers close, %.1f s", took));

	// from teleop: fingers at 0.8 rad, spread at 1.0 rad, velocity mode at V = 0
	rig.sim.with([](Pucks& p) {
		for (int id = 11; id <= 13; ++id) {
			p[id].pos = 0.8 * FINGER_COUNTS_PER_RAD;
			p[id].mode = MotorPuck::MODE_VELOCITY;
			p[id].v = 0;
		}
		p[14].pos = 1.0 * SPREAD_COUNTS_PER_RAD;
		p[14].mode = MotorPuck::MODE_VELOCITY;
		p[14].v = 0;
		return 0;
	});
	Recorder r2(rig.sim);
	t0 = Clock::now();
	ok = makeFist(*rig.hand, 10.0);
	took = secondsSince(t0);
	const double closedMeanwhile = fingersWhileSpreadMoved(r2.stop());
	check(ok && isFist(rig.sim) && closedMeanwhile < 0.02,
			strf("from fingers 0.8 rad, spread 1.0 rad: fingers open, spread to 0 (fingers at most %.3f rad "
					"meanwhile), fingers close; %.1f s", closedMeanwhile, took));

	// a finger that never gets there
	rig.sim.with([](Pucks& p) {
		for (int id = 11; id <= 13; ++id) {
			p[id].pos = 0.0;
			p[id].mode = MotorPuck::MODE_IDLE;
		}
		p[12].stuck = true;
		return 0;
	});
	t0 = Clock::now();
	ok = makeFist(*rig.hand, 1.0);
	took = secondsSince(t0);
	rig.hand->idle();
	check(!ok && took >= 1.0 && took < 1.5, strf("a stuck finger: makeFist() gives up after its 1 s timeout (%.2f s)", took));
	check(!rig.arm->failed, "the arm's loop kept running" + (rig.arm->failed ? ": " + rig.arm->error : std::string()));
}

void testFirmware116() {
	std::printf("6. Hand pucks with firmware 116 (V is property 106 on 114-118)\n");
	Rig rig(116);
	rig.startArm();
	rig.makeHand();
	HandTeleop teleop(*rig.hand, HandTeleop::Params());
	{
		HandRunner runner(teleop);
		stream(teleop, 0.3, constant(1.0));
	}
	teleop.stop();
	const int vProp = rig.sim.with([](Pucks& p) { return p[11].lastVProperty; });
	const long other = rig.sim.with([](Pucks& p) { return p[11].otherVSets + p[12].otherVSets + p[13].otherVSets; });
	const double moved = rig.sim.pos(11) / FINGER_COUNTS_PER_RAD;
	check(vProp == 106 && other == 0 && moved > 0.2, strf("V went to property %d (never 44); finger moved %.2f rad", vProp, moved));
}

void testSilentPuck() {
	std::printf("7. Why HandTeleop never waits on the bus: a hand puck that does not answer\n");
	Rig rig(175);
	rig.startArm();
	rig.makeHand();
	sleepMs(300);
	rig.arm->mark();
	sleepMs(500);
	const std::vector<double> base = rig.arm->snapshot();
	rig.sim.with([](Pucks& p) {
		p[12].mute = true;
		return 0;
	});
	rig.arm->mark();
	const Clock::time_point t0 = Clock::now();
	std::string err;
	try {
		rig.hand->doneMoving();  // a group read, as in Hand::initialize() and update()
	} catch (const std::exception& e) {
		err = e.what();
	}
	const double took = secondsSince(t0);
	sleepMs(50);
	const std::vector<double> during = rig.arm->snapshot();
	rig.sim.with([](Pucks& p) {
		p[12].mute = false;
		return 0;
	});
	std::printf("       Hand::doneMoving() with puck 12 silent %s after %.2f s\n", err.empty() ? "returned" : "threw", took);
	check(percentile(during, 1.0) > 500.0,
			strf("meanwhile the arm's loop waited %.0f ms for the bus lock (longest cycle alone: %.2f ms)",
					percentile(during, 1.0), percentile(base, 1.0)));
}

void testSecondProgram() {
	std::printf("8. Why nothing else may poll the hand on this bus (gripper_ros's bhand_node)\n");
	{
		Rig rig(175);
		const int fdB = rig.sim.addHost(handOnlyFilter);  // libgripper's CAN filter
		rig.startArm();
		sleepMs(300);
		check(!rig.arm->failed, "the arm's loop runs");
		// what libgripper's realtime loop sends: P from each hand puck every 20 ms
		std::atomic<bool> polling(true);
		const Clock::time_point t0 = Clock::now();
		std::thread poller([&] {
			while (polling) {
				for (int id = 11; id <= 14; ++id) {
					can_frame f{};
					f.can_id = canId(false, 0, id);
					f.can_dlc = 1;
					f.data[0] = static_cast<uint8_t>(pid(Puck::P, 175));
					(void)::send(fdB, &f, sizeof(f), 0);
				}
				can_frame r;
				while (::recv(fdB, &r, sizeof(r), MSG_DONTWAIT) > 0) {
				}
				sleepMs(20);
			}
		});
		while (!rig.arm->failed && secondsSince(t0) < 3.0) {
			sleepMs(5);
		}
		polling = false;
		poller.join();
		::close(fdB);
		check(rig.arm->failed && rig.arm->error.find("Buffer overflow") != std::string::npos,
				strf("a program polling the hand's positions every 20 ms: the follower's bus manager threw %.2f s later: %s",
						std::chrono::duration<double>(rig.arm->failedAt - t0).count(), rig.arm->error.c_str()));
	}
#ifdef WITH_LIBGRIPPER
	{
		Rig rig(175);
		const int fdB = rig.sim.addHost(handOnlyFilter);
		rig.startArm();
		sleepMs(300);
		gripper::barrett::BarrettHandCanDriver driver;
		const Clock::time_point t0 = Clock::now();
		const bool connected = driver.connect("fd:" + std::to_string(fdB), 0);
		std::string started = "not connected";
		if (connected) {
			auto r = driver.startRealtimeControl(
					[](const gripper::barrett::RealtimeFeedback&) {
						return boost::optional<gripper::barrett::RealtimeControlSetpoint>();
					},
					gripper::barrett::MotorGroup::All);
			started = r ? "polling" : r.error().message();
		}
		while (!rig.arm->failed && secondsSince(t0) < 3.0) {
			sleepMs(5);
		}
		driver.disconnect();
		check(rig.arm->failed && rig.arm->error.find("Buffer overflow") != std::string::npos,
				strf("libgripper's BarrettHandCanDriver (%s): the follower's bus manager threw %.2f s after connect: %s",
						started.c_str(), std::chrono::duration<double>(rig.arm->failedAt - t0).count(),
						rig.arm->error.c_str()));
	}
#endif
	std::printf("       (the follower's RealTimeExecutionManager catches only ExecutionManagerException;\n"
			"        this std::runtime_error ends the program)\n");
}

}  // namespace

int main() {
	testSetupAndHi();
	testTeleop();
	testPoses();
	testPoseFailures();
	testSlowHand();
	testOnlyVelocity();
	testVisitor();
	testCloseThroughSwitch();
	testSpreadStuck();
	testWrongZero();
	testPosesOff();
	testFist();
	testFirmware116();
	testSilentPuck();
	testSecondProgram();
	std::printf("%d checks, %d failed\n", checks, failures);
	return failures == 0 ? 0 : 1;
}
