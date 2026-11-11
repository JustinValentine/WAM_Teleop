/*
 * wam8g_dyn_feel.cpp
 *
 * Feel an identified J1-J4 dynamics model on the wam8g by hand, the way
 * ex04_display_basic_info lets you feel a gravity calibration: the arm floats
 * under libbarrett's gravity compensation, the screen shows the joints, and
 * keys add the model's terms on top, so each can be felt and compared with
 * libbarrett's gravity compensation alone.
 *
 * The parameters are read from a file at start (lib/dyn_feel_core.h): the 30
 * numbers the Feas-Ident-WAM4 notebook writes (tmp/solutions/beta_star_correct.dat),
 * or a cable_params_*.h from ident/ (python -m cable_ident fit).
 *
 * What is added to J1-J4 (J5-J8 get nothing: the model has no wrist):
 *
 *   added = clamp( level gate ( [g] model gravity - libbarrett's gravity
 *                               [k] Coriolis and centrifugal
 *                               [o] the cable model's offsets
 *                               share ( [v] viscous + [c] dry friction ) ) )
 *
 * - It starts with nothing: the model off, every term off, level 0 %.
 * - Every key ramps (2 s from 0 to 1); none steps a torque.
 * - level 0 % is libbarrett alone; with [g] on, level 100 % has swapped
 *   libbarrett's J1-J4 gravity for the model's (the teleop laws' dyn - grav).
 *   The wrist's own gravity stays libbarrett's (lib/arm_gravity.h).
 * - The added torque is clamped per joint (--clamp), fades out as a joint
 *   nears the speed limit (--speed-limit, never above 80 % of the safety
 *   board's velocity warning, which this program does not change) and near
 *   the ends of its range, and the model switches itself off when a joint
 *   passes the speed limit, when a signal is missing, or when the display
 *   stops. When the WAM leaves ACTIVE the program adds nothing and ends.
 * - The inertial term is shown and never applied (lib/dyn_feel_core.h says why).
 *
 * Usage: wam8g_dyn_feel --params FILE [--clamp nm[,nm,nm,nm]] [--speed-limit v]
 *                       [--coulomb-v0 v] [--no-wrist-gravity] [--check]
 *        (source scripts/setup_leader_wam8g.sh first)
 *   --check  read the parameters and the configuration, print what would be
 *            added at a few poses, and exit without opening the CAN bus.
 *
 * Keys: SPACE or m  the model on/off (after a trip: acknowledge it)
 *       g k v c o   gravity, Coriolis, viscous, dry friction, offsets
 *       + -         level, 10 % steps; 0: level 0 %
 *       ] [         friction share, 10 % steps
 *       q           ramp down and quit
 *
 * The display needs a terminal about 100 columns by 30 lines.
 *
 * State, 2026-11-10: written from the sources on a Mac. There, this file passed
 * a syntax check against the libbarrett-canopen headers (clang), and
 * test/test_dyn_feel_core.cpp and test/test_dyn_feel_system.cpp ran. It has
 * never been compiled with the robot computer's compiler, never linked against
 * libbarrett, and never run on a robot: the display and everything in
 * wam_main() are untested.
 */

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include <barrett/os.h>
#include <barrett/products/product_manager.h>
#include <barrett/systems.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/units.h>

#define BARRETT_SMF_VALIDATE_ARGS
#define BARRETT_SMF_WAM8G
#define BARRETT_SMF_CONFIGURE_PM
#include <barrett/standard_main_function.h>

#include "lib/arm_gravity.h"
#include "lib/clean_exit.h"
#include "lib/dyn_feel_core.h"
#include "lib/dyn_feel_system.h"
#include "lib/wam8g_config.h"

// Last: curses.h's macros (timeout, clear, refresh, ...) collide with Boost and the standard library.
#define NCURSES_NOMACROS
#include <curses.h>

using namespace barrett;

namespace {

const char* const PROGRAM = "wam8g_dyn_feel";
const double CLAMP_MAX = 5.0;        // Nm: the most --clamp accepts
const double UI_STALL = 1.0;         // s without the display loop: the model switches off
const double BOARD_SHARE = 0.8;      // of the safety board's velocity warning: the speed limit's ceiling

typedef DynFeel::jp_type jp_type;
typedef DynFeel::jv_type jv_type;
typedef DynFeel::jt_type jt_type;

struct Options {
	std::string params;
	double clamp[4];
	double speedLimit;
	double coulombV0;
	bool wristGravity;
	bool check;
	Options() : speedLimit(1.0), coulombV0(0.05), wristGravity(true), check(false) {
		const dyn_feel::Limits d;
		for (int j = 0; j < 4; ++j) clamp[j] = d.clamp[j];
	}
};

void printUsage() {
	printf("Usage: %s --params FILE [--clamp nm[,nm,nm,nm]] [--speed-limit v] [--coulomb-v0 v]\n"
	       "       [--no-wrist-gravity] [--check]\n"
	       "  --params  the identified J1-J4 parameters: 30 numbers (the Feas-Ident-WAM4 notebook's\n"
	       "            tmp/solutions/beta_star_correct.dat), or a cable_params_*.h from ident/\n"
	       "  --clamp   the most that is added to J1-J4, Nm: one value or four comma-separated,\n"
	       "            each 0 to %.0f; default 2.5,2.5,1.5,1.5\n"
	       "  --speed-limit  rad/s (0.2 to 2): the added torque fades out from 60 %% of it and the model\n"
	       "            switches off beyond it; default 1.0, and never above %.0f %% of the safety board's\n"
	       "            velocity warning (which this program does not change)\n"
	       "  --coulomb-v0  rad/s (0.005 to 0.5): Coulomb friction is fc tanh(dq / v0); default 0.05\n"
	       "  --no-wrist-gravity  leave out the wrist's gravity on J1-J4 relative to the wrist at 0\n"
	       "  --check   print what the parameters would add at a few poses; no CAN bus\n"
	       "Source scripts/setup_leader_wam8g.sh first (BARRETT_CONFIG_FILE).\n",
	       PROGRAM, CLAMP_MAX, 100.0 * BOARD_SHARE);
}

/* count values (one given: for all), each within [lo, hi]. */
bool parseList(const char* text, int count, double lo, double hi, double* out) {
	if (text == NULL) {
		return false;
	}
	std::vector<double> v;
	const char* p = text;
	for (;;) {
		char* end = NULL;
		const double x = std::strtod(p, &end);
		if (end == p || !(x >= lo && x <= hi)) {
			return false;
		}
		v.push_back(x);
		if (*end == '\0') {
			break;
		}
		if (*end != ',') {
			return false;
		}
		p = end + 1;
	}
	if (v.size() != 1 && (int) v.size() != count) {
		return false;
	}
	for (int i = 0; i < count; ++i) {
		out[i] = v.size() == 1 ? v[0] : v[i];
	}
	return true;
}

bool parseOptions(int argc, char** argv, Options* o) {
	for (int i = 1; i < argc; ++i) {
		const std::string a(argv[i]);
		const char* next = i + 1 < argc ? argv[i + 1] : NULL;
		if (a == "--params" && next != NULL) {
			o->params = next;
			++i;
		} else if (a == "--clamp") {
			if (!parseList(next, 4, 0.0, CLAMP_MAX, o->clamp)) {
				printf("%s: --clamp takes Nm from 0 to %.0f, one value or four comma-separated.\n", PROGRAM, CLAMP_MAX);
				return false;
			}
			++i;
		} else if (a == "--speed-limit") {
			if (!parseList(next, 1, 0.2, 2.0, &o->speedLimit)) {
				printf("%s: --speed-limit takes rad/s from 0.2 to 2.\n", PROGRAM);
				return false;
			}
			++i;
		} else if (a == "--coulomb-v0") {
			if (!parseList(next, 1, 0.005, 0.5, &o->coulombV0)) {
				printf("%s: --coulomb-v0 takes rad/s from 0.005 to 0.5.\n", PROGRAM);
				return false;
			}
			++i;
		} else if (a == "--no-wrist-gravity") {
			o->wristGravity = false;
		} else if (a == "--check") {
			o->check = true;
		} else {
			return false;
		}
	}
	return !o->params.empty();
}

/* The gravity term at level 100 % at one pose: the model's gravity minus the
 * calibrated model's with the wrist at 0 (lib/arm_gravity.h; NULL: not shown). */
void printGravityAt(const dyn_feel::Params& p, const arm_gravity::Model* ag, const double* q8, const double* clamp) {
	double model[4];
	dyn_feel::modelGravity(p, q8, model);
	printf("  J1-J4 at %6.2f %6.2f %6.2f %6.2f rad: model gravity %7.2f %7.2f %7.2f %7.2f Nm", q8[0], q8[1], q8[2],
	       q8[3], model[0], model[1], model[2], model[3]);
	if (ag == NULL || ag->getDof() != 8) {
		printf("\n");
		return;
	}
	double qRef[8], lib[8];
	for (int j = 0; j < 8; ++j) {
		qRef[j] = j < 4 ? q8[j] : 0.0;
	}
	ag->torque(qRef, lib);
	bool beyond = false;
	printf("\n      minus the calibration's (wrist at 0):          %7.2f %7.2f %7.2f %7.2f Nm", model[0] - lib[0],
	       model[1] - lib[1], model[2] - lib[2], model[3] - lib[3]);
	for (int j = 0; j < 4; ++j) {
		beyond = beyond || std::fabs(model[j] - lib[j]) > clamp[j];
	}
	printf("%s\n", beyond ? "  <-- beyond the clamp" : "");
}

void printLimits(const dyn_feel::Limits& lim) {
	printf("Added torque: at most %.2f %.2f %.2f %.2f Nm on J1-J4; fading from %.2f rad/s, the model off beyond\n"
	       "  %.2f rad/s; nothing beyond J1 %.2f..%.2f, J2 %.2f..%.2f, J3 %.2f..%.2f, J4 %.2f..%.2f rad, full\n"
	       "  %.2f rad inside; every change ramps over %.1f s.\n",
	       lim.clamp[0], lim.clamp[1], lim.clamp[2], lim.clamp[3], lim.speedFade, lim.speedTrip, lim.lower[0],
	       lim.upper[0], lim.lower[1], lim.upper[1], lim.lower[2], lim.upper[2], lim.lower[3], lim.upper[3], lim.margin,
	       lim.rampTime);
}

dyn_feel::Limits makeLimits(const Options& o, double boardWarning) {
	dyn_feel::Limits lim;
	for (int j = 0; j < 4; ++j) {
		lim.clamp[j] = o.clamp[j];
	}
	lim.speedTrip = o.speedLimit;
	if (boardWarning > 0.0) {
		lim.speedTrip = std::min(lim.speedTrip, BOARD_SHARE * boardWarning);
	}
	lim.speedFade = 0.6 * lim.speedTrip;
	return lim;
}

/* --check: no CAN bus. */
int preflight(const Options& o) {
	dyn_feel::Params p;
	std::string error;
	if (!dyn_feel::loadParams(o.params, &p, &error)) {
		printf("%s: %s\n", PROGRAM, error.c_str());
		return 2;
	}
	printf("%s", dyn_feel::describe(p).c_str());
	printLimits(makeLimits(o, 0.0));
	libconfig::Config config;
	arm_gravity::Model ag;
	bool haveGravity = false;
	const bool ready = wam8g_config::loadAndCheck(config, PROGRAM);
	if (config.exists("wam8g")) {
		haveGravity = ag.load(config.lookup("wam8g"), PROGRAM) && ag.getDof() == 8;
	}
	printf("What the gravity term would add at level 100 %% (the wrist at 0)%s:\n",
	       haveGravity ? "" : "; no calibrated gravity model to compare with");
	const double poses[6][4] = {{0.0, 0.0, 0.0, 0.0},    {0.0, -1.57, 0.0, 1.57}, {0.0, -1.95, 0.0, 3.0},
	                            {0.5, -1.0, 0.5, 2.0}, {0.0, 1.0, 0.0, 1.0},   {1.0, -1.6, -1.0, 2.0}};
	for (int k = 0; k < 6; ++k) {
		double q8[8] = {poses[k][0], poses[k][1], poses[k][2], poses[k][3], 0.0, 0.0, 0.0, 0.0};
		printGravityAt(p, haveGravity ? &ag : NULL, q8, o.clamp);
	}
	printf(ready ? "Ready. Nothing was sent to the robot.\n" : "Not ready (see above). Nothing was sent to the robot.\n");
	return ready ? 0 : 2;
}

// --- the display ------------------------------------------------------------

bool cursesOn = false;
void endCurses() {
	if (cursesOn) {
		endwin();
		cursesOn = false;
	}
}

double shown(double x) {
	return !std::isfinite(x) ? 999.0 : std::max(-999.0, std::min(999.0, x));
}

void row(int y, const char* label, const double* v, int n, bool fine) {
	mvprintw(y, 0, "%-30s", label);
	for (int i = 0; i < n; ++i) {
		printw(fine ? " %7.3f" : " %7.2f", shown(v[i]));
	}
	clrtoeol();
}

void termRow(int y, char key, int term, const char* label, const double* v, const DynFeel::Snapshot& s, bool friction) {
	char text[40];
	snprintf(text, sizeof(text), "[%c] %s", key, label);
	row(y, text, v, 4, false);
	const double w = s.mixer.weight[term] * (friction ? s.mixer.frictionShare : 1.0);
	printw("   %s %3.0f %%", s.targets.term[term] ? "ON " : "off", 100.0 * w);
	clrtoeol();
}

/* Until 'q', a stop signal, or the WAM leaves ACTIVE. Returns why. */
const char* display(ProductManager& pm, systems::Wam8g& wam, DynFeel& feel, const dyn_feel::Params& params,
                    const dyn_feel::Limits& lim) {
	SafetyModule* safety = pm.getSafetyModule();
	initscr();
	cursesOn = true;
	std::atexit(endCurses);
	curs_set(0);
	noecho();
	cbreak();
	timeout(100);  // getch() waits this long: the loop runs at about 10 Hz

	const char* why = "quit";
	int pollFailures = 0;
	bool going = true;
	while (going) {
		feel.heartbeat();
		const DynFeel::Snapshot s = feel.snapshot();
		const jp_type jp = wam.getJointPositions();
		const jv_type jv = wam.getJointVelocities();
		const jt_type jt = wam.getJointTorques();
		double q[8], dq[8], tau[8];
		for (int j = 0; j < 8; ++j) {
			q[j] = jp[j];
			dq[j] = jv[j];
			tau[j] = jt[j];
		}

		int y = 0;
		mvprintw(y++, 0, "%s   %s (%s)", PROGRAM, params.source.c_str(),
		         params.kind == dyn_feel::Params::CABLE ? "cable model" : "30 base parameters");
		clrtoeol();
		if (s.mixer.trip != dyn_feel::NO_TRIP) {
			mvprintw(y++, 0, "MODEL OFF: %s", dyn_feel::tripText(s.mixer.trip));
			if (s.mixer.tripJoint >= 0) {
				printw(" (J%d)", s.mixer.tripJoint + 1);
			}
			printw(". SPACE acknowledges; SPACE again switches it on.");
		} else if (s.targets.model || s.mixer.level > 0.0) {
			mvprintw(y++, 0, "MODEL %s   level %3.0f %% (asked %3.0f %%)   friction share %3.0f %%",
			         s.targets.model ? "ON " : "going off", 100.0 * s.mixer.level, 100.0 * s.targets.level,
			         100.0 * s.targets.frictionShare);
		} else {
			mvprintw(y++, 0, "MODEL OFF: libbarrett's gravity compensation alone   (level asked %3.0f %%, friction share %3.0f %%)",
			         100.0 * s.targets.level, 100.0 * s.targets.frictionShare);
		}
		clrtoeol();
		++y;
		mvprintw(y++, 0, "%-30s %7s %7s %7s %7s %7s %7s %7s %7s", "", "J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8");
		row(y++, "Joint positions (rad)", q, 8, true);
		row(y++, "Joint velocities (rad/s)", dq, 8, true);
		row(y++, "Joint torques, commanded (Nm)", tau, 8, false);
		row(y++, "libbarrett gravity (Nm)", s.libGravity, 8, false);
		++y;
		mvprintw(y++, 0, "The model, J1-J4 (Nm)%45s", "asked, applied");
		clrtoeol();
		double gravity[4], difference[4];
		for (int j = 0; j < 4; ++j) {
			gravity[j] = s.terms.gravity[j] + s.wrist[j];
			difference[j] = gravity[j] - s.libGravity[j];
		}
		row(y++, "    gravity (with the wrist's)", gravity, 4, false);
		termRow(y++, 'g', dyn_feel::GRAVITY, "... minus libbarrett's", difference, s, false);
		termRow(y++, 'k', dyn_feel::CORIOLIS, "Coriolis, centrifugal", s.terms.coriolis, s, false);
		termRow(y++, 'v', dyn_feel::VISCOUS, "viscous friction", s.terms.viscous, s, true);
		termRow(y++, 'c', dyn_feel::DRY, "dry friction", s.terms.dry, s, true);
		termRow(y++, 'o', dyn_feel::OFFSET, "offsets", s.terms.offset, s, false);
		row(y++, "    inertia (never applied)", s.terms.inertia, 4, false);
		++y;
		row(y++, "ADDED to J1-J4 (Nm)", s.mixer.added, 4, false);
		printw("   clamp %.1f %.1f %.1f %.1f", lim.clamp[0], lim.clamp[1], lim.clamp[2], lim.clamp[3]);
		for (int j = 0; j < 4; ++j) {
			if (s.mixer.clamped[j]) {
				printw("  J%d CLAMPED", j + 1);
			}
		}
		clrtoeol();
		row(y++, "    fade (speed, joint range)", s.mixer.gate, 4, false);
		printw("   1: none; speed limit %.2f rad/s", lim.speedTrip);
		clrtoeol();
		++y;
		try {
			const units::CartesianPosition::type cp = wam.getToolPosition();
			const Eigen::Quaterniond to = wam.getToolOrientation();
			const Eigen::Matrix3d R = to.toRotationMatrix();
			mvprintw(y++, 0, "%-30s [%6.3f, %6.3f, %6.3f]", "Tool position (m)", shown(cp[0]), shown(cp[1]), shown(cp[2]));
			clrtoeol();
			mvprintw(y++, 0, "%-30s %+7.4f %+7.4fi %+7.4fj %+7.4fk", "Tool orientation (quat)", to.w(), to.x(), to.y(),
			         to.z());
			clrtoeol();
			for (int r = 0; r < 3; ++r) {
				mvprintw(y++, 0, "%-30s [%+7.4f %+7.4f %+7.4f]", r == 0 ? "Tool orientation (mat)" : "", R(r, 0), R(r, 1),
				         R(r, 2));
				clrtoeol();
			}
		} catch (const std::exception&) {
			y += 5;  // no calibrated kinematics: no tool pose
		}
		++y;
		mvprintw(y++, 0, "SPACE/m model on/off   g k v c o terms   +/- level (0: none)   ]/[ friction share   q quit");
		clrtoeol();
		if (s.cycles == 0) {
			mvprintw(y++, 0, "(waiting for the control loop)");
			clrtoeol();
		}
		refresh();

		switch (getch()) {
		case ' ':
		case 'm': feel.toggleModel(); break;
		case 'g': feel.toggleTerm(dyn_feel::GRAVITY); break;
		case 'k': feel.toggleTerm(dyn_feel::CORIOLIS); break;
		case 'v': feel.toggleTerm(dyn_feel::VISCOUS); break;
		case 'c': feel.toggleTerm(dyn_feel::DRY); break;
		case 'o': feel.toggleTerm(dyn_feel::OFFSET); break;
		case '+':
		case '=': feel.changeLevel(0.1); break;
		case '-':
		case '_': feel.changeLevel(-0.1); break;
		case '0': feel.setLevel(0.0); break;
		case ']': feel.changeFrictionShare(0.1); break;
		case '[': feel.changeFrictionShare(-0.1); break;
		case 'q':
		case 'x': going = false; break;
		default: break;  // no key within 100 ms, or another key
		}
		if (clean_exit::requested()) {
			why = "a stop signal";
			going = false;
		}
		try {
			if (safety->getMode() != SafetyModule::ACTIVE) {
				why = "the WAM is not active";
				going = false;
			}
			pollFailures = 0;
		} catch (const std::exception&) {  // a lost reply: poll again
			if (++pollFailures >= 10) {
				why = "the safety module does not answer";
				going = false;
			}
		}
	}
	return why;
}

}  // namespace

bool validate_args(int argc, char** argv) {
	Options o;
	if (!parseOptions(argc, argv, &o)) {
		printUsage();
		return false;
	}
	if (o.check) {
		std::exit(preflight(o));
	}
	dyn_feel::Params p;
	std::string error;
	if (!dyn_feel::loadParams(o.params, &p, &error)) {  // before any puck is woken
		printf("%s: %s\n", PROGRAM, error.c_str());
		return false;
	}
	return true;
}

bool configure_pm(int, char** argv, ProductManager& pm) {
	// Refuse an unflagged calibration before wake/zero/ACTIVE.
	return wam8g_config::configReady(pm.getConfig(), argv[0]);
}

int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam8g& wam8g) {
	Options o;
	parseOptions(argc, argv, &o);  // checked in validate_args()
	dyn_feel::Params params;
	std::string error;
	if (!dyn_feel::loadParams(o.params, &params, &error)) {
		printf("%s: %s\n", PROGRAM, error.c_str());
		return 1;
	}
	printf("%s", dyn_feel::describe(params).c_str());

	// The arm floats, as in ex04; Wam8g re-checks both calibration flags.
	wam8g.gravityCompensate();
	systems::Wam<8>& wam = wam8g;
	SafetyModule* safety = pm.getSafetyModule();

	// The safety board's velocity limits are read, not changed.
	double boardWarning = 0.0, boardFault = 0.0;
	try {
		boardWarning = safety->getPuck()->getProperty(Puck::VL1) / 4096.0;
		boardFault = safety->getPuck()->getProperty(Puck::VL2) / 4096.0;
		printf("Safety board velocity limits (rad/s per joint): warning %.2f, fault %.2f (not changed here).\n", boardWarning,
		       boardFault);
	} catch (const std::exception& e) {
		printf("The safety board's velocity limits could not be read (%s).\n", e.what());
	}
	const dyn_feel::Limits limits = makeLimits(o, boardWarning);
	printLimits(limits);

	arm_gravity::Model armGravity;
	bool useWrist = false;
	if (o.wristGravity) {
		useWrist = armGravity.load(pm.getConfig().lookup("wam8g"), PROGRAM) && armGravity.getDof() == 8;
		if (!useWrist) {
			printf("%s: continuing without the wrist's gravity correction.\n", PROGRAM);
		}
	}
	printf("Wrist gravity on J1-J4, relative to the wrist at 0: %s.\n", useWrist ? "on" : "off");
	{
		const jp_type jp = wam.getJointPositions();
		double q8[8];
		for (int j = 0; j < 8; ++j) q8[j] = jp[j];
		printf("Here, the gravity term at level 100 %% would add:\n");
		printGravityAt(params, useWrist ? &armGravity : NULL, q8, limits.clamp);
	}

	DynFeel feel(params, limits, o.coulombV0, useWrist ? &armGravity : NULL, pm.getExecutionManager()->getPeriod(),
	             UI_STALL);
	systems::connect(wam.jpOutput, feel.jpInput);
	systems::connect(wam.jvOutput, feel.jvInput);
	systems::connect(wam.gravity.output, feel.gravityInput);
	systems::connect(feel.output, wam.input);  // zero until the keys ask for more

	clean_exit::install();  // Ctrl-C ends it as 'q' does
	printf("\nThe arm floats under libbarrett's gravity compensation; nothing else is applied until you\n"
	       "switch the model on. Keep the E-stop at hand. The display needs about 100 columns by 30 lines.\n"
	       "[Enter] opens it, 'q' [Enter] quits: ");
	fflush(stdout);
	std::string line;
	const char* why = "quit";
	if (clean_exit::readLine(&line) && line != "q") {
		try {
			why = display(pm, wam8g, feel, params, limits);
		} catch (const std::exception& e) {
			endCurses();
			printf("%s: the display failed: %s\n", PROGRAM, e.what());
			why = "an error";
		}
		// Down by its ramp, then off for good.
		feel.setModel(false);
		for (int i = 0; i < 30 && feel.snapshot().mixer.level > 0.0; ++i) {
			btsleep(0.1);
		}
		endCurses();
		printf("Ended: %s.\n", why);
	}
	feel.stop();
	btsleep(0.5);  // the level's last fall, if the ramp above was cut short
	const DynFeel::Snapshot last = feel.snapshot();
	systems::disconnect(wam.input);
	printf("The model is off (last added torque %.3f %.3f %.3f %.3f Nm); the arm still floats under libbarrett's\n"
	       "gravity compensation. Support it before you idle it.\n",
	       last.mixer.added[0], last.mixer.added[1], last.mixer.added[2], last.mixer.added[3]);

	clean_exit::waitUntilNotActive(safety);
	return 0;
}
