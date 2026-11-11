/*
 * leader_wam8g.cpp
 *
 * Teleop leader on the eight-joint wam8g (CANopen libbarrett: J1-J4 arm,
 * passive J5, Alberta wrist J6-J8), paired with a seven-joint P2 follower
 * started with --sync wam8g. Same control structure as leader.cpp: both arms'
 * position controllers track each other, and lib/teleop_law.h adds the
 * feedforward on J1-J4.
 *
 * Usage: leader_wam8g [--law u1|u2|u4|u6] [--model legacy|cable] [--wrist] [--check]
 *                     [remoteHost] [recPort] [sendPort]
 *   --law    feedforward on J1-J4 (lib/teleop_law.h), default u6. u2, u4 and
 *            u6 need the wam8g fit in lib/leader_beta_wam8g.h; before it, run
 *            with --law u1.
 *   --wrist  couple the wrist (lib/wam8g_wrist_map.h). Without it both
 *            wrists hold their sync angles and only J1-J4 are coupled.
 *   --model  the dynamics model behind the external-torque estimate and the
 *            feedforward: legacy (lib/leader_dynamics.h, the default) or
 *            cable (lib/cable_drive_dynamics.h: rigid body, inertia, the cable
 *            drive's friction elements, the wrist's gravity; ident/).
 *   --wrist-force, --wrist-deadband  the wrist with the follower's
 *            force/torque sensor (lib/ft_core.h; README, "Force/torque sensor").
 *   --check  check the configuration and print the settings, without
 *            opening the CAN bus.
 *   Defaults: remoteHost 127.0.0.1, recPort 5555, sendPort 5554.
 *
 * The position gains for J1-J4 are the Ares leader's (lib/wam8g_teleop.h);
 * J5-J8 keep the robot's configured gains.
 */

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <barrett/detail/stl_utils.h>
#include <barrett/os.h>
#include <barrett/products/product_manager.h>
#include <barrett/systems.h>
#include <barrett/units.h>

#define BARRETT_SMF_VALIDATE_ARGS
// The eight-joint wam8g only, through the guarded Wam8g view; configure_pm
// checks its calibration flags before any puck is woken.
#define BARRETT_SMF_WAM8G
#define BARRETT_SMF_CONFIGURE_PM
#include <barrett/standard_main_function.h>

#include "lib/background_state_publisher.h"
#include "lib/cable_drive_dynamics.h"
#include "lib/cable_params_wam8g.h"
#include "lib/clean_exit.h"
#include "lib/cpu_latency.h"
#include "lib/teleop_mode.h"
#include "lib/safety_watch.h"
#include "lib/dynamic_external_torque.h"
#include "lib/estimator_systems.h"
#include "lib/leader_beta_wam8g.h"
#include "lib/leader_wam8g.h"
#include "lib/teleop_app.h"
#include "lib/teleop_recorder.h"
#include "lib/wam8g_config.h"
#include "lib/wam8g_teleop.h"

using namespace barrett;
using detail::waitForEnter;

namespace {

const char* const PROGRAM = "leader_wam8g";

/* The safety board's velocity fault limit, rad/s per joint (warning at 90%).
 * The board powers up with 1.0 (warning 0.5) and the library leaves it, so it
 * is set at every start. At 3.0 (Barrett's advice) a fast forearm roll faulted
 * the leader at 3.3 rad/s on J3 (2026-09-24). */
const double VELOCITY_LIMIT = 4.0;

struct Options {
	teleop_law::Law law;
	teleop_law::ForceGains forceGain;  // 0.5 on J1-J4
	double forceFilter;
	double forceNotch;
	double velocityLimit;
	double j6Friction;
	bool cableModel;   // --model cable
	double ffFriction;  // --ff-friction: share of the friction elements in the feedforward
	double ffInertia;   // --ff-inertia: share of the inertial torque in the feedforward
	bool wristGravity;  // --no-wrist-gravity clears it
	bool wrist;
	double wristForce[3];     // --wrist-force: J6-J8
	double wristForceFilter;  // --wrist-force-filter, Hz
	double wristForceDeadzone;  // --wrist-force-deadzone, Nm
	double wristDeadbandDeg;  // --wrist-deadband
	bool check;
	teleop_app::Options app;  // the low-latency options (lib/teleop_app.h)
	std::string remoteHost;
	int recPort;
	int sendPort;
	Options()
		: law(teleop_law::U6), forceFilter(0.0), forceNotch(0.0), velocityLimit(VELOCITY_LIMIT),
		  j6Friction(0.0), cableModel(false), ffFriction(0.0), ffInertia(0.0), wristGravity(true),
		  wrist(false), wristForceFilter(LeaderWam8g::wristForceFilterDefault()),
		  wristForceDeadzone(LeaderWam8g::wristForceDeadzoneDefault()), wristDeadbandDeg(0.0),
		  check(false), remoteHost("127.0.0.1"),
		  recPort(5555), sendPort(5554) {
		wristForce[0] = wristForce[1] = wristForce[2] = 0.0;
	}
	bool wristForceOn() const { return wristForce[0] > 0.0 || wristForce[1] > 0.0 || wristForce[2] > 0.0; }
};

void printUsage() {
	printf("Usage: %s [--law u1|u2|u4|u6] [--force-gain g] [--force-filter hz] [--force-notch hz]\n"
			"       [--velocity-limit v] [--j6-friction nm] [--model legacy|cable]\n"
			"       [--ff-friction s] [--ff-inertia s] [--no-wrist-gravity] [--wrist] [--check]\n"
			"       [--wrist-force g] [--wrist-deadband deg] [--wrist-force-filter hz] [--wrist-force-deadzone nm]\n"
			"       [remoteHost] [recPort] [sendPort]\n"
			"  --law    feedforward on J1-J4, default u6:\n"
			"             u1  position-position\n"
			"             u2  + dynamics compensation\n"
			"             u4  4C: + force channels\n"
			"             u6  4C + local force feedback\n"
			"           u2, u4 and u6 need the wam8g fit in lib/leader_beta_wam8g.h.\n"
			"  --force-gain  the force channel's gain in u4 and u6 (0 to 1), one for J1-J4\n"
			"           or four comma-separated (e.g. 0.4,0.4,0.2,0.4), default 0.5\n"
			"  --force-filter  a low-pass on the force channel in Hz, default 0 (off)\n"
			"  --force-notch  a notch on the force channel in Hz (Q 0.7), default 0 (off)\n"
			"  --velocity-limit  the safety board's velocity fault limit in rad/s per joint\n"
			"           (0 to 5], default 4 (warning at 90%%)\n"
			"  --j6-friction  J6's friction feedforward in Nm (0 to 2), default 0 (off);\n"
			"           wam8g_j6_friction measures J6's friction, 'f' changes it live\n"
			"  --model  the dynamics model of the external-torque estimate and the feedforward:\n"
			"           legacy (lib/leader_dynamics.h, default) or cable (the cable-drive model,\n"
			"           lib/cable_params_wam8g.h from ident/)\n"
			"  --ff-friction  with --model cable: the share (0 to 1) of the cable friction in the\n"
			"           feedforward, default 0 (the estimate always uses all of it); 'm' changes it live\n"
			"  --ff-inertia  with --model cable: the share (0 to 1) of the inertial torque in the\n"
			"           feedforward, default 0\n"
			"  --no-wrist-gravity  with --model cable: leave out the wrist's gravity correction\n"
			"  --wrist  couple the wrist (wam8g J6, J7, J8 -> P2 J5, J6, J7)\n"
			"  --wrist-force  with --wrist and the follower's --ft wrist|all: the gain (0 to 1) on the wrist\n"
			"           torques its force/torque sensor measured, one for J6-J8 or three comma-separated;\n"
			"           default 0 (off). 'w' changes it live\n"
			"  --wrist-deadband  with --wrist: the wrist's position coupling pulls only beyond this many\n"
			"           degrees (0 to 10), default 0 (off: the plain coupling). With --wrist-force it is in\n"
			"           effect only while the follower's measured torques arrive. 'w' changes it live\n"
			"  --wrist-force-filter  the wrist force channel's low-pass in Hz (0: off), default 15\n"
			"  --wrist-force-deadzone  the measured wrist torque below which it stays out, Nm, default 0.03\n"
			"  --check  check the configuration without opening the CAN bus\n"
			"%s"
			"  Defaults: remoteHost=127.0.0.1, recPort=5555, sendPort=5554\n", PROGRAM, teleop_app::usage());
}

bool parseOptions(int argc, char** argv, Options* o) {
	std::vector<std::string> positional;
	for (int i = 1; i < argc; ++i) {
		const std::string a(argv[i]);
		const int app = teleop_app::parse(argc, argv, &i, &o->app, PROGRAM);
		if (app < 0) {
			return false;
		} else if (app > 0) {
			continue;
		}
		if (a == "--law") {
			if (i + 1 >= argc || !teleop_law::parse(argv[i + 1], &o->law)) {
				printf("%s: --law takes u1, u2, u4 or u6.\n", PROGRAM);
				return false;
			}
			++i;
		} else if (a == "--force-gain") {
			if (i + 1 >= argc || !teleop_law::parseForceGain(argv[i + 1], &o->forceGain)) {
				printf("%s: --force-gain takes one number from 0 to 1, or four comma-separated.\n", PROGRAM);
				return false;
			}
			++i;
		} else if (a == "--force-filter") {
			if (i + 1 >= argc || !teleop_law::parseForceFilter(argv[i + 1], &o->forceFilter)) {
				printf("%s: --force-filter takes Hz from 0 (off) to 100.\n", PROGRAM);
				return false;
			}
			++i;
		} else if (a == "--force-notch") {
			if (i + 1 >= argc || !teleop_law::parseForceFilter(argv[i + 1], &o->forceNotch)) {
				printf("%s: --force-notch takes Hz from 0 (off) to 100.\n", PROGRAM);
				return false;
			}
			++i;
		} else if (a == "--velocity-limit") {
			char* end = NULL;
			const double v = i + 1 < argc ? std::strtod(argv[i + 1], &end) : 0.0;
			if (i + 1 >= argc || end == argv[i + 1] || *end != '\0' || !(v > 0.0 && v <= 5.0)) {
				printf("%s: --velocity-limit takes rad/s per joint, above 0 and up to 5.\n", PROGRAM);
				return false;
			}
			o->velocityLimit = v;
			++i;
		} else if (a == "--j6-friction") {
			char* end = NULL;
			const double v = i + 1 < argc ? std::strtod(argv[i + 1], &end) : -1.0;
			if (i + 1 >= argc || end == argv[i + 1] || *end != '\0' || !(v >= 0.0 && v <= 2.0)) {
				printf("%s: --j6-friction takes Nm from 0 (off) to 2.\n", PROGRAM);
				return false;
			}
			o->j6Friction = v;
			++i;
		} else if (a == "--model") {
			const std::string m = i + 1 < argc ? argv[i + 1] : "";
			if (m != "legacy" && m != "cable") {
				printf("%s: --model takes legacy or cable.\n", PROGRAM);
				return false;
			}
			o->cableModel = (m == "cable");
			++i;
		} else if (a == "--ff-friction" || a == "--ff-inertia") {
			char* end = NULL;
			const double v = i + 1 < argc ? std::strtod(argv[i + 1], &end) : -1.0;
			if (i + 1 >= argc || end == argv[i + 1] || *end != '\0' || !(v >= 0.0 && v <= 1.0)) {
				printf("%s: %s takes a share from 0 to 1.\n", PROGRAM, a.c_str());
				return false;
			}
			(a == "--ff-friction" ? o->ffFriction : o->ffInertia) = v;
			++i;
		} else if (a == "--no-wrist-gravity") {
			o->wristGravity = false;
		} else if (a == "--wrist") {
			o->wrist = true;
		} else if (a == "--wrist-force") {
			if (!teleop_app::parseList(i + 1 < argc ? argv[i + 1] : NULL, 3, 0.0, 1.0, o->wristForce)) {
				printf("%s: --wrist-force takes one gain from 0 to 1, or three comma-separated (J6,J7,J8).\n", PROGRAM);
				return false;
			}
			++i;
		} else if (a == "--wrist-deadband") {
			if (!teleop_app::parseList(i + 1 < argc ? argv[i + 1] : NULL, 1, 0.0, 10.0, &o->wristDeadbandDeg)) {
				printf("%s: --wrist-deadband takes degrees from 0 (off) to 10.\n", PROGRAM);
				return false;
			}
			++i;
		} else if (a == "--wrist-force-filter") {
			if (!teleop_app::parseList(i + 1 < argc ? argv[i + 1] : NULL, 1, 0.0, 100.0, &o->wristForceFilter)) {
				printf("%s: --wrist-force-filter takes Hz from 0 (off) to 100.\n", PROGRAM);
				return false;
			}
			++i;
		} else if (a == "--wrist-force-deadzone") {
			if (!teleop_app::parseList(i + 1 < argc ? argv[i + 1] : NULL, 1, 0.0, 0.5, &o->wristForceDeadzone)) {
				printf("%s: --wrist-force-deadzone takes Nm from 0 to 0.5.\n", PROGRAM);
				return false;
			}
			++i;
		} else if (a == "--check") {
			o->check = true;
		} else if (a.find(":=") != std::string::npos) {
			// a ROS remapping argument; ros::init() takes it
		} else if (!a.empty() && a[0] == '-') {
			return false;
		} else {
			positional.push_back(a);
		}
	}
	if (positional.size() > 3) {
		return false;
	}
	if (!o->wrist && (o->wristForceOn() || o->wristDeadbandDeg > 0.0)) {
		printf("%s: --wrist-force and --wrist-deadband act on the coupled wrist: add --wrist.\n", PROGRAM);
		return false;
	}
	if (positional.size() >= 1) o->remoteHost = positional[0];
	if (positional.size() >= 2) o->recPort = std::atoi(positional[1].c_str());
	if (positional.size() >= 3) o->sendPort = std::atoi(positional[2].c_str());
	return true;
}

void printSettings(const Options& o) {
	printf("Law: %s, force gain %s, force filter ", teleop_law::name(o.law), teleop_law::forceGainText(o.forceGain).c_str());
	if (o.forceFilter > 0.0) {
		printf("%.1f Hz", o.forceFilter);
	} else {
		printf("off");
	}
	if (o.forceNotch > 0.0) {
		printf(", force notch %.1f Hz\n", o.forceNotch);
	} else {
		printf(", force notch off\n");
	}
	printf("Safety board velocity limits to set (rad/s per joint): warning %.2f, fault %.2f\n",
			0.9 * o.velocityLimit, o.velocityLimit);
	if (o.j6Friction > 0.0) {
		printf("J6 friction feedforward: %.2f Nm (from the first link)\n", o.j6Friction);
	} else {
		printf("J6 friction feedforward: off\n");
	}
	printf("wam8g dynamics fit (lib/leader_beta_wam8g.h): %s\n",
			LEADER_BETA_WAM8G_FITTED ? "yes" : "not yet (the follower is sent zero external torque)");
	if (o.cableModel) {
		printf("Model: cable (lib/cable_params_wam8g.h); feedforward shares: friction %.2f, inertia %.2f;\n"
				"       wrist gravity correction %s\n", o.ffFriction, o.ffInertia, o.wristGravity ? "on" : "off");
	} else {
		printf("Model: legacy (lib/leader_beta_wam8g.h)\n");
	}
	if (o.wrist) {
		const wam8g_teleop::WristMap m = wam8g_teleop::WristMap::coupled();
		printf("Wrist: coupled, wam8g J6 -> P2 J5%s, J7 -> P2 J6%s, J8 -> P2 J7%s\n",
				m.sign[0] < 0 ? " (reversed)" : "", m.sign[1] < 0 ? " (reversed)" : "",
				m.sign[2] < 0 ? " (reversed)" : "");
		if (o.wristForceOn()) {
			printf("Wrist force channel: gain %.2f, %.2f, %.2f (J6-J8) on the follower's measured wrist torques\n"
					"       (its --ft wrist|all), low-pass %.0f Hz, nothing below %.2f Nm, at most %.1f Nm\n",
					o.wristForce[0], o.wristForce[1], o.wristForce[2], o.wristForceFilter, o.wristForceDeadzone,
					LeaderWam8g::wristForceLimit());
		} else {
			printf("Wrist force channel: off\n");
		}
		if (o.wristDeadbandDeg > 0.0) {
			printf("Wrist position coupling: a tether, no pull within %.1f degrees%s\n", o.wristDeadbandDeg,
					o.wristForceOn() ? " while the follower's measured torques arrive" : " (always: no wrist force channel)");
		} else {
			printf("Wrist position coupling: as before (no deadband)\n");
		}
	} else {
		printf("Wrist: not coupled; both wrists hold their sync angles\n");
	}
	teleop_app::print(o.app);
	if (!o.app.record.empty()) {
		printf("Recording: %s (one row per control cycle)\n", o.app.record.c_str());
	}
	printf("Sync pose J1-J8 (rad):");
	for (int i = 0; i < 8; ++i) {
		printf(" %.3f", wam8g_teleop::SYNC_POSE[i]);
	}
	printf(" (J5 passive)\nFollower: %s, sending to port %d, receiving on %d\n",
			o.remoteHost.c_str(), o.sendPort, o.recPort);
}

void printGains(const char* label, const double* v) {
	printf("  %-6s", label);
	for (int i = 0; i < 8; ++i) {
		printf(" %7.4g", v[i]);
	}
	printf("\n");
}

/* A libconfig number, integer or float. */
double number(const libconfig::Setting& s) {
	switch (s.getType()) {
	case libconfig::Setting::TypeInt: return (int) s;
	case libconfig::Setting::TypeInt64: return (double) (long long) s;
	default: return s;
	}
}

/* The configured J1-J8 values of wam8g.joint_position_control.<name>, J1-J4
 * replaced by the Ares values. False if the setting is missing. */
bool teleopGains(const libconfig::Config& config, const char* name, const double* ares, double* out) {
	const std::string path = std::string("wam8g.joint_position_control.") + name;
	if (!config.exists(path)) {
		return false;
	}
	const libconfig::Setting& s = config.lookup(path);
	if (s.getLength() != 8) {
		return false;
	}
	for (int i = 0; i < 8; ++i) {
		out[i] = i < 4 ? ares[i] : number(s[i]);
	}
	return true;
}

int preflight(const Options& o) {
	libconfig::Config config;
	const bool ready = wam8g_config::loadAndCheck(config, PROGRAM);
	printSettings(o);
	printf("Teleop position gains J1-J8 (J1-J4 Ares leader, J5-J8 configured):\n");
	const char* names[4] = {"kp", "ki", "kd", "control_signal_limit"};
	const char* labels[4] = {"kp", "ki", "kd", "limit"};
	const double* ares[4] = {wam8g_teleop::ARES_KP, wam8g_teleop::ARES_KI, wam8g_teleop::ARES_KD,
			wam8g_teleop::ARES_CONTROL_SIGNAL_LIMIT};
	for (int g = 0; g < 4; ++g) {
		double v[8];
		try {
			if (teleopGains(config, names[g], ares[g], v)) {
				printGains(labels[g], v);
			} else {
				printf("  %-6s (wam8g.joint_position_control.%s missing)\n", labels[g], names[g]);
			}
		} catch (const libconfig::SettingException& e) {
			printf("  %-6s unreadable: %s\n", labels[g], e.what());
		}
	}
	printf(ready ? "Ready. Nothing was sent to the robot.\n"
			: "Not ready (see above). Nothing was sent to the robot.\n");
	return ready ? 0 : 2;
}

/* J1-J4 get the Ares leader's position gains; J5-J8 keep the configured ones. */
template <size_t DOF>
void applyTeleopGains(systems::Wam<DOF>& wam, systems::ExecutionManager* em, double kiScale) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	BARRETT_SCOPED_LOCK(em->getMutex());
	v_type kp = wam.jpController.getKp();
	v_type ki = wam.jpController.getKi();
	v_type kd = wam.jpController.getKd();
	v_type integratorLimit = wam.jpController.getIntegratorLimit();
	jt_type controlSignalLimit = wam.jpController.getControlSignalLimit();
	for (size_t i = 0; i < 4; ++i) {
		kp[i] = wam8g_teleop::ARES_KP[i];
		ki[i] = kiScale * wam8g_teleop::ARES_KI[i];
		kd[i] = wam8g_teleop::ARES_KD[i];
		integratorLimit[i] = wam8g_teleop::ARES_INTEGRATOR_LIMIT[i];
		controlSignalLimit[i] = wam8g_teleop::ARES_CONTROL_SIGNAL_LIMIT[i];
	}
	wam.jpController.setKp(kp);
	wam.jpController.setKi(ki);
	wam.jpController.setKd(kd);
	wam.jpController.setIntegratorLimit(integratorLimit);
	wam.jpController.setControlSignalLimit(controlSignalLimit);
}

template <size_t DOF>
void printControllerGains(systems::Wam<DOF>& wam) {
	double v[4][8];
	for (size_t i = 0; i < DOF && i < 8; ++i) {
		v[0][i] = wam.jpController.getKp()[i];
		v[1][i] = wam.jpController.getKi()[i];
		v[2][i] = wam.jpController.getKd()[i];
		v[3][i] = wam.jpController.getControlSignalLimit()[i];
	}
	printf("Teleop position gains J1-J8:\n");
	printGains("kp", v[0]);
	printGains("ki", v[1]);
	printGains("kd", v[2]);
	printGains("limit", v[3]);
}

}  // namespace

bool validate_args(int argc, char** argv) {
	Options o;
	if (!parseOptions(argc, argv, &o)) {
		printUsage();
		return false;
	}
	if (teleop_law::usesModel(o.law) && !LEADER_BETA_WAM8G_FITTED) {
		printf("%s: %s needs the wam8g dynamics fit in lib/leader_beta_wam8g.h, which\n"
				"is not filled in yet. Until then run with --law u1.\n", PROGRAM, teleop_law::name(o.law));
		return false;
	}
	if (o.app.matched && !LEADER_BETA_WAM8G_FITTED) {
		printf("%s: --estimator matched needs the wam8g dynamics fit (lib/leader_beta_wam8g.h).\n", PROGRAM);
		return false;
	}
	if (o.check) {
		std::exit(preflight(o));
	}
	return true;
}

bool configure_pm(int, char** argv, ProductManager& pm) {
	// Refuse an unflagged calibration before wake/zero/ACTIVE.
	return wam8g_config::configReady(pm.getConfig(), argv[0]);
}

template <size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {
	static_assert(DOF == 8, "leader_wam8g drives the eight-joint wam8g");
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	systems::ExecutionManager* em = pm.getExecutionManager();
	CpuLatencyHold cpuLatency;  // until the program ends

	Options o;
	parseOptions(argc, argv, &o);  // checked in validate_args()
	printSettings(o);
	SafetyModule* safety = pm.getSafetyModule();
	safety->setVelocityLimit(o.velocityLimit);  // warning at 90%
	printf("Safety board velocity limits (rad/s per joint): warning %.2f, fault %.2f\n",
			safety->getPuck()->getProperty(Puck::VL1) / 4096.0, safety->getPuck()->getProperty(Puck::VL2) / 4096.0);
	applyTeleopGains(wam, em, o.app.kiScale);
	printControllerGains(wam);

	// The sync pose; J5 is passive, so its measured angle is kept.
	jp_type syncPos = wam.getJointPositions();
	for (size_t i = 0; i < DOF; ++i) {
		if (i != 4) {
			syncPos[i] = wam8g_teleop::SYNC_POSE[i];
		}
	}

	// The loop's pages stay in memory (the CANopen library locks them too).
	if (!rt_thread::lockMemory()) {
		printf("Warning: cannot lock the program's memory (mlockall): page faults can delay the loop.\n");
	}

	// ROS: the joint_states/gravity publisher (the source of the ROS
	// recording; a 500 Hz thread that takes the control loop's lock) and the
	// mode topic. --no-ros-state drops the publisher, --no-ros everything.
	teleop_app::Lazy<BackgroundStatePublisher<DOF> > state_publisher;
	if (o.app.ros) {
		ros::init(argc, argv, "leader");
		if (o.app.rosState) {
			state_publisher.emplace(em, wam);
		}
	}

	systems::Summer<jt_type, 3> customjtSum;
	em->startManaging(customjtSum);

	// The fitted model without the acceleration (as before: LeaderDynamics'
	// acceleration input was zero), on fixed arrays.
	RegressorDynamics<DOF> leaderDynamics(initialize_leader_beta(), 0.0);
	DynamicExternalTorque<DOF> dynamicExternalTorque(em);

	// Joint acceleration for the dynamics model: high-passed velocity, then
	// low-passed (as in leader.cpp).
	double h_omega_p = 25.0;
	systems::FirstOrderFilter<jv_type> hp1;
	hp1.setHighPass(jv_type(h_omega_p), jv_type(h_omega_p));
	systems::Gain<jv_type, double, ja_type> jaWAM(1.0);
	em->startManaging(hp1);

	systems::FirstOrderFilter<ja_type> jaFilter;
	ja_type l_omega_p = ja_type::Constant(50.0);
	jaFilter.setLowPass(l_omega_p);
	em->startManaging(jaFilter);

	LeaderWam8g leader(em, o.remoteHost, o.recPort, o.sendPort, o.law,
			o.wrist ? wam8g_teleop::WristMap::coupled() : wam8g_teleop::WristMap::held(),
			LEADER_BETA_WAM8G_FITTED != 0);
	leader.setForceGain(o.forceGain);
	leader.setForceFilter(o.forceFilter);
	leader.setForceNotch(o.forceNotch);
	leader.setJ6Friction(o.j6Friction);
	leader.setWristForce(o.wristForce);
	leader.setWristForceFilter(o.wristForceFilter, o.wristForceDeadzone);
	leader.setWristDeadband(o.wristDeadbandDeg * M_PI / 180.0);
	leader.setRealtime(o.app.rtPriority, o.app.rtCpu);
	leader.setAgeReference(o.app.ageReference);
	// 'c' (4C off/on) or 'p' (P-P on/off) here or in the other arm's program,
	// or `wam 4c|pp on|off`: the mode on both arms (lib/teleop_mode.h). The
	// message ends with this program's prompt, where `wam stop` looks for it,
	// unless the key was typed here (the loop prints it then).
	std::atomic<bool> ownMode(false);
	std::unique_ptr<TeleopModeSwitch> modeSwitch;
	if (o.app.ros) modeSwitch.reset(new TeleopModeSwitch([&leader, &o, &ownMode](teleop_law::Mode m) {
		if (m == leader.getMode()) {
			printf("\nMode already %s.\n", teleop_law::modeText(m, o.law).c_str());
		} else {
			printf("\nMode %s; the change fades in over %.0f s.\n", teleop_law::modeText(m, o.law).c_str(),
			       teleop_law::MODE_FADE_TIME);
		}
		if (!ownMode.exchange(false)) {
			printf(">>> ");
		}
		fflush(stdout);
		leader.setMode(m);  // after the message: the keys wait for this
	}));

	systems::connect(wam.jvOutput, hp1.input);
	systems::connect(hp1.output, jaWAM.input);
	systems::connect(jaWAM.output, jaFilter.input);

	systems::connect(wam.jpOutput, leader.wamJPIn);
	systems::connect(wam.jvOutput, leader.wamJVIn);
	systems::connect(dynamicExternalTorque.wamExternalTorqueOut, leader.extTorqueIn);
	systems::connect(wam.supervisoryController.output, leader.pidIn);  // for the recording

	systems::connect(wam.jpOutput, leaderDynamics.jpInput);
	systems::connect(wam.jvOutput, leaderDynamics.jvInput);

	systems::connect(leader.wamJPOutput, customjtSum.getInput(0));
	systems::connect(wam.gravity.output, customjtSum.getInput(1));
	systems::connect(wam.supervisoryController.output, customjtSum.getInput(2));

	systems::connect(customjtSum.output, dynamicExternalTorque.wamTorqueSumIn);
	systems::connect(leaderDynamics.dynamicsFeedFWD, dynamicExternalTorque.wamDynamicsIn);

	systems::connect(wam.gravity.output, leader.wamGravIn);
	systems::connect(leaderDynamics.dynamicsFeedFWD, leader.wamDynIn);

	// --model cable: the cable-drive model (lib/cable_drive_dynamics.h) in
	// place of LeaderDynamics, for the estimate and the feedforward. Its
	// inertia uses the real acceleration (jaFilter); the feedforward's share
	// of it is --ff-inertia (0: none, as with the legacy model above).
	arm_gravity::Model armGravity;
	std::unique_ptr<CableDriveDynamics<DOF> > cable;
	if (o.cableModel) {
		if (o.wristGravity && !armGravity.load(pm.getConfig().lookup("wam8g"), PROGRAM)) {
			printf("%s: continuing without the wrist gravity correction.\n", PROGRAM);
		}
		const bool useGravity = o.wristGravity && armGravity.getDof() == (int) DOF;
		cable.reset(new CableDriveDynamics<DOF>(em, cable_params_wam8g::params(), useGravity ? &armGravity : NULL, 4,
				o.ffInertia, o.ffFriction));
		systems::connect(wam.jpOutput, cable->jpInput);
		systems::connect(wam.jvOutput, cable->jvInput);
		systems::connect(jaFilter.output, cable->jaInput);
		systems::forceConnect(cable->modelOutput, dynamicExternalTorque.wamDynamicsIn);
		systems::forceConnect(cable->feedforwardOutput, leader.wamDynIn);
		printf("Model: cable; wrist gravity correction %s.\n", useGravity ? "on" : "off");
	}

	// --estimator matched: the commanded torque and the model's other terms
	// through the same filter as the model's acceleration, so the arm's own
	// inertial torque cancels at every frequency (lib/estimator_core.h). The
	// feedforward is not touched.
	MatchedAcceleration<DOF> matchedAcceleration(o.app.estimatorHp, o.app.estimatorLp, em->getPeriod());
	MatchedExternalTorque<DOF> matchedExternalTorque(o.app.estimatorHp, o.app.estimatorLp, em->getPeriod());
	if (o.app.matched) {
		systems::connect(wam.jvOutput, matchedAcceleration.jvInput);
		systems::connect(customjtSum.output, matchedExternalTorque.wamTorqueSumIn);
		if (cable) {
			systems::connect(matchedAcceleration.output, cable->jaEstimatorInput);
			systems::connect(cable->staticOutput, matchedExternalTorque.staticIn);
			systems::connect(cable->estimatorInertiaOutput, matchedExternalTorque.inertiaIn);
		} else {
			systems::connect(matchedAcceleration.output, leaderDynamics.jaEstimatorInput);
			systems::connect(leaderDynamics.staticOutput, matchedExternalTorque.staticIn);
			systems::connect(leaderDynamics.estimatorInertiaOutput, matchedExternalTorque.inertiaIn);
		}
		systems::forceConnect(matchedExternalTorque.wamExternalTorqueOut, leader.extTorqueIn);
	}

	// --record: one row per control cycle, written by the loop itself
	// (lib/teleop_recorder.h); scripts/teleop_log.py reads it.
	std::unique_ptr<TeleopRecorder> recorder;
	if (!o.app.record.empty()) {
		const std::string file = teleop_app::recordFile(o.app.record, "leader");
		std::string note = "leader_wam8g";
		for (int i = 1; i < argc; ++i) {
			note += std::string(" ") + argv[i];
		}
		try {
			recorder.reset(new TeleopRecorder(file, LeaderWam8g::recordColumns(), note));
			leader.setRecorder(recorder.get());
			printf("Recording to %s\n", file.c_str());
		} catch (const std::exception& e) {
			printf("NOT recording: %s\n", e.what());
		}
	}

	// What the loop no longer prints itself (lib/teleop_app.h).
	// Messages end with the prompt, where `wam stop` looks for it.
	bool recorderWarned = false;
	unsigned int lostSeen = 0, adoptedSeen = 0;
	teleop_app::Watch watch([&]() {
		const unsigned int lost = leader.getLinkLostEvents();
		if (lost != lostSeen) {
			lostSeen = lost;
			printf("\nlost link\n>>> ");
			fflush(stdout);
		}
		const unsigned int adopted = leader.getModeAdoptions();
		if (adopted != adoptedSeen) {
			adoptedSeen = adopted;
			printf("\nMode %s (from the follower); the change fades in over %.0f s.\n>>> ",
					teleop_law::modeText(leader.getMode(), o.law).c_str(), teleop_law::MODE_FADE_TIME);
			fflush(stdout);
		}
		if (recorder && !recorderWarned && (recorder->writeFailed() || recorder->dropped() > 0)) {
			recorderWarned = true;
			printf("\nRECORDING PROBLEM: %s (%lu rows dropped so far). The teleop is not affected.\n>>> ",
					recorder->writeFailed() ? "the file cannot be written (disk full?)" : "the disk is too slow",
					recorder->dropped());
			fflush(stdout);
		}
	});

	wam.gravityCompensate();

	// From the first link: when this arm leaves ACTIVE, stop the teleop
	// (lib/safety_watch.h).
	std::unique_ptr<SafetyWatch> safetyWatch;

	std::string line;
	v_type gainTmp;
	bool going = true;

	// Ctrl-C, SIGTERM or the end of the input end the teleop as 'x' does
	// (lib/clean_exit.h).
	clean_exit::install();

	// What the control thread is now (lib/rt_thread.h), once it has run a cycle.
	for (int i = 0; i < 100 && !leader.getRealtimeReport().done; ++i) {
		btsleep(0.01);
	}
	rt_thread::print("Leader", leader.getRealtimeReport());

	while (going) {
		printf(">>> ");
		if (!clean_exit::readLine(&line)) {
			going = false;
			continue;
		}
		if (line.empty()) {
			printf("\n");
			continue;
		}

		switch (line[0]) {
		case 'l':
			if (leader.isHalted()) {
				printf("The teleop has stopped (above); restart both programs to link again.\n");
			} else if (leader.isLinked()) {
				leader.unlink();
				printf("Unlinked: no feedforward; the arm still follows the other WAM's position.\n");
			} else {
				wam.moveTo(syncPos, true);
				printf("Press [Enter] to link with the other WAM.");
				waitForEnter();
				if (clean_exit::requested()) {  // a stop, not Enter: no link
					going = false;
					break;
				}
				leader.tryLink();
				wam.trackReferenceSignal(leader.theirJPOutput);
				systems::forceConnect(leader.wamJPOutput, wam.input);

				btsleep(0.1);  // wait an execution cycle or two
				if (leader.isLinked()) {
					printf("Linked with remote WAM.\n");
					if (!safetyWatch) {
						safetyWatch.reset(new SafetyWatch(safety, [&leader]() {
							leader.halt();
							printf("\nThe leader is not active (a fault, E-stop or idle): teleop stopped. It\n"
									"sends nothing more, so the follower holds. Restart both programs to link again.\n");
							fflush(stdout);
						}));
					}
				} else {
					printf("WARNING: Linking was unsuccessful.\n");
				}
			}
			break;

		case 't': {
			size_t jointNumber;
			std::cout << "\tJoint: ";
			std::cin >> jointNumber;
			const size_t jointIndex = jointNumber - 1;
			if (jointIndex >= DOF || jointIndex == 4) {
				std::cout << "\tBad joint number (J5 is passive): " << jointNumber << std::endl;
				break;
			}

			std::cout << "\tGain identifier (p, i, or d): ";
			std::cin >> line;
			const char gainId = line[0];
			if (gainId != 'p' && gainId != 'i' && gainId != 'd') {
				std::cout << "\tBad gain identifier." << std::endl;
				break;
			}
			{
				BARRETT_SCOPED_LOCK(em->getMutex());
				gainTmp = gainId == 'p' ? wam.jpController.getKp()
						: gainId == 'i' ? wam.jpController.getKi() : wam.jpController.getKd();
			}
			std::cout << "\tCurrent value: " << gainTmp[jointIndex] << std::endl;
			std::cout << "\tNew value: ";
			std::cin >> gainTmp[jointIndex];
			{
				BARRETT_SCOPED_LOCK(em->getMutex());
				switch (gainId) {
				case 'p': wam.jpController.setKp(gainTmp); break;
				case 'i': wam.jpController.setKi(gainTmp); break;
				case 'd': wam.jpController.setKd(gainTmp); break;
				}
			}
			} break;

		case 'f': {
			std::cout << "\tJ6 friction feedforward (Nm), now " << leader.getJ6Friction() << "; new value: ";
			double nm = -1.0;
			std::cin >> nm;
			if (!std::cin || nm < 0.0 || nm > 2.0) {
				std::cin.clear();
				std::cout << "\tKept it: 0 (off) to 2 Nm." << std::endl;
			} else {
				leader.setJ6Friction(nm);
			}
			std::getline(std::cin, line);  // the rest of the line
			} break;

		case 'w': {
			if (!o.wrist) {
				std::cout << "\tThe wrist is not coupled (--wrist)." << std::endl;
				break;
			}
			double gains[3];
			leader.getWristForce(gains);
			printf("\tWrist force gain now %.2f, %.2f, %.2f (J6-J8), deadband %.1f degrees; the follower's measured\n"
					"\twrist torques are %s. New gain (0 to 1, all three) and deadband (0 to 10 degrees): ",
					gains[0], gains[1], gains[2], leader.getWristDeadband() * 180.0 / M_PI,
					leader.getWristUse() > 0.99 ? "in use" : leader.getWristUse() > 0.0 ? "fading" : "not arriving");
			double g = -1.0, deg = -1.0;
			std::cin >> g >> deg;
			if (!std::cin || g < 0.0 || g > 1.0 || deg < 0.0 || deg > 10.0) {
				std::cin.clear();
				std::cout << "\tKept them." << std::endl;
			} else {
				gains[0] = gains[1] = gains[2] = g;
				leader.setWristForce(gains);
				leader.setWristDeadband(deg * M_PI / 180.0);
			}
			std::getline(std::cin, line);  // the rest of the line
			} break;

		case 'm': {
			if (!cable) {
				std::cout << "\tThe feedforward shares need --model cable." << std::endl;
				break;
			}
			std::cout << "\tFeedforward shares now: friction " << cable->getFeedforwardFriction() << ", inertia "
			          << cable->getFeedforwardInertia() << "; new friction and inertia shares (0 to 1): ";
			double fs = -1.0, is = -1.0;
			std::cin >> fs >> is;
			if (!std::cin || fs < 0.0 || fs > 1.0 || is < 0.0 || is > 1.0) {
				std::cin.clear();
				std::cout << "\tKept them." << std::endl;
			} else {
				cable->setFeedforward(is, fs);
			}
			std::getline(std::cin, line);  // the rest of the line
			} break;

		case 'c':
		case 'p': {
			teleop_law::Mode target = line[0] == 'c'
					? (leader.getMode() == teleop_law::FULL ? teleop_law::NO_4C : teleop_law::FULL)
					: (leader.getMode() == teleop_law::PP ? teleop_law::FULL : teleop_law::PP);
			if (modeSwitch) {
				ownMode = true;
				modeSwitch->publish(target);
				for (int i = 0; i < 50 && leader.getMode() != target; ++i) {
					btsleep(0.01);  // its message first, then the prompt
				}
			} else {  // no ROS: the follower takes it from this arm's packets
				leader.setMode(target);
				printf("Mode %s; the change fades in over %.0f s.\n", teleop_law::modeText(target, o.law).c_str(),
						teleop_law::MODE_FADE_TIME);
			}
			} break;

		case 'x':
			going = false;
			break;

		default:
			printf("\n");
			printf("    'l' to go to the sync pose and link with the other WAM (again: unlink)\n");
			printf("    'c' 4C off/on, 'p' P-P on/off: on both arms, fading over 1 s\n");
			printf("    't' to tune the position gains\n");
			printf("    'f' to set J6's friction feedforward\n");
			printf("    'w' to set the wrist's force gain and position deadband\n");
			printf("    'm' to set the cable model's feedforward shares (friction, inertia)\n");
			printf("    'x' to exit\n");
			break;
		}
	}

	// The teleop ends here: nothing more is sent (the follower holds) and the
	// arm holds where it is.
	if (safetyWatch) {
		safetyWatch->stop();
	}
	leader.halt();
	wam.moveTo(wam.getJointPositions());

	// The session's link and recording, for the pane's log.
	watch.stop();
	const teleop_link::Link<7>::Stats link = leader.getLinkStats();
	printf("Link: %llu packets sent (%llu refused), %llu received, %llu lost on the way, %llu arrived two in one "
			"cycle, %llu rejected; the link dropped %u time(s).\n",
			(unsigned long long) link.sent, (unsigned long long) link.sendErrors, (unsigned long long) link.received,
			(unsigned long long) link.lost, (unsigned long long) link.skipped, (unsigned long long) link.rejected,
			leader.getLinkLostEvents());
	if (recorder) {
		leader.setRecorder(NULL);
		btsleep(0.01);  // the cycle in progress finishes its row
		recorder->close();
		printf("Recording: %lu rows in %s (%lu dropped%s).\n", recorder->written(), recorder->getPath().c_str(),
				recorder->dropped(), recorder->writeFailed() ? "; THE FILE COULD NOT BE WRITTEN" : "");
	}

	clean_exit::waitUntilNotActive(pm.getSafetyModule());

	return 0;
}

int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam8g& wam) {
	// Wam8g's own gravityCompensate() re-checks both calibration flags; the
	// teleop then runs on the eight-joint Wam<8> view (J5 is an inert slot).
	wam.gravityCompensate();
	return wam_main<8>(argc, argv, pm, wam);
}
