/*
 * wam8g_j6_friction.cpp
 *
 * Measures the wam8g's J6 friction by commanding J6's motor torque directly,
 * for leader_wam8g --j6-friction (J6's bearing drags: its outer race was
 * squeezed when it was installed).
 *
 * The arm goes straight up (J2-J4 at 0), where J6's axis is vertical: J6 turns
 * parallel to the ground and gravity puts no torque on it. J6's position gains
 * go to zero, so J6 gets only the torque commanded here, while the rest of the
 * arm holds the pose. The torque rises slowly, one way and then the other,
 * until J6 has turned 0.01 rad, and then drops to zero at once: the breakaway
 * torque. The average of the two ways is J6's friction; half their difference
 * is whatever else pushes J6 one way there (the gravity model, the motor's
 * cogging). Then J6's gains come back and the arm returns to where it started.
 *
 * Usage: wam8g_j6_friction   (source scripts/setup_leader_wam8g.sh first)
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>
#include <vector>

#include <barrett/os.h>
#include <barrett/products/product_manager.h>
#include <barrett/systems.h>
#include <barrett/units.h>

#define BARRETT_SMF_VALIDATE_ARGS
#define BARRETT_SMF_WAM8G
#define BARRETT_SMF_CONFIGURE_PM
#include <barrett/standard_main_function.h>

#include "lib/wam8g_config.h"

using namespace barrett;

namespace {

const char* const PROGRAM = "wam8g_j6_friction";
const size_t J6 = 5;
const double RAMP = 0.1;         // Nm/s
const double MOVED = 0.01;       // rad: J6 has broken away
const double MAX_TORQUE = 2.0;   // Nm: give up
const double MAX_SPEED = 1.0;    // rad/s: stop at once
const double MAX_TRAVEL = 0.3;   // rad from where the tests start: stop
const int PAIRS = 4;             // trials each way
const double MOVE_SPEED = 0.3;   // rad/s, to and from the pose

typedef units::JointPositions<8>::type jp_type;
typedef units::JointTorques<8>::type jt_type;
typedef math::Vector<8>::type v_type;

enum Result { BROKE_AWAY, NO_MOTION, WRONG_WAY, TOO_FAR, NOT_ACTIVE };

/* One trial: J6's torque rises at RAMP in direction dir (+1, -1) until J6 has
 * turned MOVED that way, then drops to zero; *nm is the torque it took. */
Result breakaway(systems::Wam<8>& wam, SafetyModule* safety, systems::ExposedOutput<jt_type>& torque, int dir,
		double origin, double* nm) {
	if (safety->pollMode() != SafetyModule::ACTIVE) {
		return NOT_ACTIVE;
	}
	const double start = wam.getJointPositions()[J6];
	const std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
	jt_type t(0.0);
	Result result = NO_MOTION;
	for (;;) {
		const double ramp = RAMP * std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
		if (ramp >= MAX_TORQUE) {
			break;
		}
		const double q = wam.getJointPositions()[J6];
		const double v = wam.getJointVelocities()[J6];
		if (std::fabs(q - origin) > MAX_TRAVEL || std::fabs(v) > MAX_SPEED) {
			result = TOO_FAR;
			break;
		}
		if (std::fabs(q - start) >= MOVED) {
			result = dir * (q - start) > 0.0 ? BROKE_AWAY : WRONG_WAY;
			break;
		}
		*nm = ramp;
		t[J6] = dir * ramp;
		torque.setValue(t);
		btsleep(0.002);
	}
	t[J6] = 0.0;
	torque.setValue(t);
	btsleep(1.0);  // J6 comes to rest
	return result;
}

void setJ6Gains(systems::Wam<8>& wam, const v_type& kp, const v_type& ki, const v_type& kd) {
	BARRETT_SCOPED_LOCK(wam.getEmMutex());
	wam.jpController.setKp(kp);
	wam.jpController.setKi(ki);
	wam.jpController.setKd(kd);
}

double mean(const std::vector<double>& x) {
	double s = 0.0;
	for (size_t i = 0; i < x.size(); ++i) s += x[i];
	return x.empty() ? 0.0 : s / x.size();
}

}  // namespace

bool validate_args(int argc, char** argv) {
	if (argc > 1) {
		printf("Usage: %s   (no options; source scripts/setup_leader_wam8g.sh first)\n", PROGRAM);
		return false;
	}
	(void) argv;
	return true;
}

bool configure_pm(int, char** argv, ProductManager& pm) {
	// Refuse an unflagged calibration before wake/zero/ACTIVE.
	return wam8g_config::configReady(pm.getConfig(), argv[0]);
}

int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam8g& wam8g) {
	(void) argc;
	(void) argv;
	wam8g.gravityCompensate();
	systems::Wam<8>& wam = wam8g;
	SafetyModule* safety = pm.getSafetyModule();

	const jp_type home = wam.getJointPositions();
	jp_type pose = home;
	pose[1] = pose[2] = pose[3] = 0.0;  // straight up: J6's axis vertical
	printf("The arm goes straight up (J2-J4 to 0; J1 and the wrist stay) at %.1f rad/s. Clear the\n"
			"space above it, and keep hands off the wrist during the test. [Enter] to go, 'q' [Enter] to quit: ",
			MOVE_SPEED);
	std::string line;
	std::getline(std::cin, line);
	if (line != "q") {
		wam.moveTo(pose, true, MOVE_SPEED, 0.5);

		// J6 free: its position gains to zero; its torque from here.
		v_type kp, ki, kd;
		{
			BARRETT_SCOPED_LOCK(wam.getEmMutex());
			kp = wam.jpController.getKp();
			ki = wam.jpController.getKi();
			kd = wam.jpController.getKd();
		}
		v_type kp0 = kp, ki0 = ki, kd0 = kd;
		kp0[J6] = ki0[J6] = kd0[J6] = 0.0;
		systems::ExposedOutput<jt_type> torque{jt_type(0.0)};
		systems::connect(torque.output, wam.input);
		setJ6Gains(wam, kp0, ki0, kd0);
		btsleep(0.5);

		printf("J6 torque ramps at %.2f Nm/s until J6 turns %.3f rad, %d times each way.\n", RAMP, MOVED, PAIRS);
		const double origin = wam.getJointPositions()[J6];
		std::vector<double> up, down;
		Result r = BROKE_AWAY;
		for (int i = 0; i < PAIRS && r == BROKE_AWAY; ++i) {
			for (int dir = 1; dir >= -1 && r == BROKE_AWAY; dir -= 2) {
				double nm = 0.0;
				r = breakaway(wam, safety, torque, dir, origin, &nm);
				if (r == BROKE_AWAY) {
					(dir > 0 ? up : down).push_back(nm);
					printf("  turning %c: %.3f Nm\n", dir > 0 ? '+' : '-', nm);
				}
			}
		}
		switch (r) {
			case BROKE_AWAY: break;
			case NO_MOTION: printf("J6 did not turn up to %.1f Nm; stopped.\n", MAX_TORQUE); break;
			case WRONG_WAY: printf("J6 turned against the torque; stopped (check J6's torque sign).\n"); break;
			case TOO_FAR: printf("J6 turned too far or too fast; stopped.\n"); break;
			case NOT_ACTIVE: printf("The arm is not active; stopped.\n"); break;
		}

		// J6 held where it is now, then its gains back, then home.
		systems::disconnect(wam.input);
		if (safety->pollMode() == SafetyModule::ACTIVE) {
			wam.moveTo(wam.getJointPositions(), true, MOVE_SPEED, 0.5);
			setJ6Gains(wam, kp, ki, kd);
			wam.moveTo(home, true, MOVE_SPEED, 0.5);
		} else {
			setJ6Gains(wam, kp, ki, kd);
		}

		if (!up.empty() && !down.empty()) {
			const double friction = 0.5 * (mean(up) + mean(down));
			printf("\nJ6 breakaway: %.3f Nm (turning + %.3f, - %.3f); %.3f Nm pushes it + there\n"
					"(the gravity model, cogging).\n", friction, mean(up), mean(down), 0.5 * (mean(down) - mean(up)));
			printf("Start leader_wam8g at half of it, --j6-friction %.2f, and tune it with 'f': the drag\n"
					"once J6 turns is below the breakaway, and more than the drag turns J6 by itself.\n",
					0.5 * friction);
		}
	}

	printf("Shift-Idle the WAM to finish.\n");
	safety->waitForMode(SafetyModule::IDLE);
	return 0;
}
