/* Software-in-the-loop check of the wam8g leader (lib/leader_wam8g.h) and the
 * P2 follower (lib/follower.h) talking over UDP on this computer. Fixed joint
 * states stand in for the two robots; each side runs on its own manual
 * execution manager. No robot or CAN bus.
 * Run: devel/lib/wam_teleop/wam8g_loopback_tests */

#include <sched.h>
#include <unistd.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <libconfig.h++>

#include <barrett/math/kinematics.h>
#include <barrett/systems.h>
#include <barrett/systems/kinematics_base.h>
#include <barrett/systems/manual_execution_manager.h>
#include <barrett/units.h>

#include "dynamic_external_torque.h"
#include "estimator_systems.h"
#include "follower.h"
#include "ft_systems.h"
#include "follower_dynamics.h"  // the class RegressorDynamics replaces in follower.cpp
#include "leader_wam8g.h"
#include "teleop_recorder.h"
#include "wrist_hold.h"

using namespace barrett;

namespace {

int failures = 0;
int checks = 0;

void check(bool ok, const char* what) {
	++checks;
	if (!ok) {
		++failures;
		printf("FAIL: %s\n", what);
	}
}

bool near(double a, double b) { return std::fabs(a - b) < 1e-9; }
/* A reference to the moving leader may be advanced with its sent velocity for
 * up to two cycles between packets (teleop_law::ReferenceTracker). */
bool nearRef(double a, double b) { return std::fabs(a - b) < 3e-3; }

/* Reads a signal each execution cycle. */
template <typename T>
class Probe : public systems::System, public systems::SingleInput<T> {
public:
	explicit Probe(systems::ExecutionManager* em)
		: System("Probe"), systems::SingleInput<T>(this), defined(false) {
		em->startManaging(*this);
	}
	virtual ~Probe() { this->mandatoryCleanUp(); }
	T value;
	bool defined;

protected:
	virtual void operate() {
		value = this->input.getValue();
		defined = true;
	}
};

/* A joint position whose J1 moves every execution cycle. */
template <typename T>
class Moving : public systems::System, public systems::SingleOutput<T> {
public:
	Moving(systems::ExecutionManager* em, const T& start, double step)
		: System("Moving"), systems::SingleOutput<T>(this), value(start), step(step) {
		em->startManaging(*this);
	}
	virtual ~Moving() { this->mandatoryCleanUp(); }
	T value;

protected:
	virtual void operate() {
		value[0] += step;
		this->outputValue->setData(&value);
	}
	double step;
};

/* The torque graph around an arm system in follower.cpp and leader_wam8g.cpp.
 * The Wam's low-level sink, managed first, pulls the Wam's torque sum each
 * cycle: the arm system's feedforward (JT_INPUT, pulled first), gravity, and
 * the position controller tracking the arm system's reference. customjtSum -
 * dynamics is the external torque estimate into the arm system. The
 * controller here is P only, kp 1: its torque is the reference it used minus
 * the measured position. */
template <size_t DOF>
struct WamStandIn {
	typedef typename units::JointPositions<DOF>::type jp_type;
	typedef typename units::JointTorques<DOF>::type jt_type;

	Probe<jt_type> sink;
	systems::Summer<jt_type, 3> jtSum;
	systems::PIDController<jp_type, jt_type> controller;
	systems::Summer<jt_type, 3> customjtSum;
	DynamicExternalTorque<DOF> extEstimate;
	systems::Constant<jt_type> grav, dyn;

	explicit WamStandIn(systems::ExecutionManager* em)
		: sink(em), jtSum(true), extEstimate(em), grav(jt_type(1.0)), dyn(jt_type(2.0)) {
		em->startManaging(customjtSum);
		controller.setKp(typename math::Vector<DOF>::type(1.0));
	}

	template <typename Arm>
	void attach(Arm* arm, systems::System::Output<jp_type>& jp) {
		systems::connect(arm->wamJPOutput, jtSum.getInput(0));
		systems::connect(grav.output, jtSum.getInput(1));
		systems::connect(controller.controlOutput, jtSum.getInput(2));
		systems::connect(jtSum.output, sink.input);
		systems::connect(arm->theirJPOutput, controller.referenceInput);
		systems::connect(jp, controller.feedbackInput);
		systems::connect(arm->wamJPOutput, customjtSum.getInput(0));
		systems::connect(grav.output, customjtSum.getInput(1));
		systems::connect(controller.controlOutput, customjtSum.getInput(2));
		systems::connect(customjtSum.output, extEstimate.wamTorqueSumIn);
		systems::connect(dyn.output, extEstimate.wamDynamicsIn);
		systems::connect(extEstimate.wamExternalTorqueOut, arm->extTorqueIn);
		systems::connect(jp, arm->wamJPIn);
		systems::connect(grav.output, arm->wamGravIn);
		systems::connect(dyn.output, arm->wamDynIn);
	}
};

typedef units::JointPositions<8>::type jp8;
typedef units::JointVelocities<8>::type jv8;
typedef units::JointTorques<8>::type jt8;
typedef units::JointPositions<7>::type jp7;
typedef units::JointVelocities<7>::type jv7;
typedef units::JointTorques<7>::type jt7;

void cycles(systems::ManualExecutionManager* a, systems::ManualExecutionManager* b, int n) {
	for (int i = 0; i < n; ++i) {
		a->runExecutionCycle();
		if (b != NULL) {
			b->runExecutionCycle();
		}
		usleep(2000);
	}
}

/* One leader-follower pair on its own ports; the follower runs u4. */
void runPair(teleop_law::Law leaderLaw, bool sendExtTorque, int port) {
	using wam8g_teleop::SYNC_POSE;
	printf("pair on ports %d/%d: leader %s, %s external torque\n", port, port + 1,
			teleop_law::name(leaderLaw), sendExtTorque ? "sends" : "does not send");
	systems::ManualExecutionManager lem(0.002), fem(0.002);

	// The wam8g near its sync pose.
	jp8 ljp;
	for (int i = 0; i < 8; ++i) ljp[i] = SYNC_POSE[i];
	ljp[0] += 0.1;
	ljp[3] -= 0.05;
	ljp[4] = 0.004;  // passive J5 as measured
	ljp[5] += 0.2;   // J6
	ljp[6] += 0.15;  // J7
	ljp[7] -= 0.3;   // J8
	jv8 ljv(0.0);
	ljv[0] = 0.1;
	ljv[5] = 0.5;
	ljv[6] = 0.7;
	ljv[7] = -0.4;
	jt8 lext, lgrav, ldyn;
	for (int i = 0; i < 8; ++i) {
		lext[i] = 1.0 + i;
		lgrav[i] = 10.0 + i;
		ldyn[i] = 12.0 + 2 * i;
	}
	systems::Constant<jp8> lJp(ljp);
	systems::Constant<jv8> lJv(ljv);
	systems::Constant<jt8> lExt(lext), lGrav(lgrav), lDyn(ldyn);
	LeaderWam8g leader(&lem, "127.0.0.1", port, port + 1, leaderLaw,
			wam8g_teleop::WristMap::coupled(), sendExtTorque);
	systems::connect(lJp.output, leader.wamJPIn);
	systems::connect(lJv.output, leader.wamJVIn);
	systems::connect(lExt.output, leader.extTorqueIn);
	systems::connect(lGrav.output, leader.wamGravIn);
	systems::connect(lDyn.output, leader.wamDynIn);
	Probe<jp8> leaderRef(&lem);
	Probe<jt8> leaderTorque(&lem);
	systems::connect(leader.theirJPOutput, leaderRef.input);
	systems::connect(leader.wamJPOutput, leaderTorque.input);

	// The P2 WAM near its sync pose.
	jp7 fjp;
	for (int i = 0; i < 7; ++i) fjp[i] = wam8g_teleop::FOLLOWER_SYNC_POSE[i];
	fjp[1] += 0.05;
	fjp[4] += 0.25;  // J5, from its sync angle
	fjp[5] -= 0.1;   // J6
	fjp[6] += 0.6;   // J7
	jv7 fjv(0.0);
	jt7 fext, fgrav, fdyn;
	for (int i = 0; i < 7; ++i) {
		fext[i] = 2.0 + i;
		fgrav[i] = 20.0 + i;
		fdyn[i] = 21.0 + 3 * i;
	}
	systems::Constant<jp7> fJp(fjp);
	systems::Constant<jv7> fJv(fjv);
	systems::Constant<jt7> fExt(fext), fGrav(fgrav), fDyn(fdyn);
	Follower<7> follower(&fem, "127.0.0.1", port + 1, port);
	follower.setLaw(4);
	systems::connect(fJp.output, follower.wamJPIn);
	systems::connect(fJv.output, follower.wamJVIn);
	systems::connect(fExt.output, follower.extTorqueIn);
	systems::connect(fGrav.output, follower.wamGravIn);
	systems::connect(fDyn.output, follower.wamDynIn);
	Probe<jp7> followerRef(&fem);
	Probe<jt7> followerTorque(&fem);
	systems::connect(follower.theirJPOutput, followerRef.input);
	systems::connect(follower.wamJPOutput, followerTorque.input);

	cycles(&lem, &fem, 100);
	check(leaderRef.defined && followerRef.defined, "both sides receive");

	// Unlinked: references flow, no feedforward.
	for (int i = 0; i < 8; ++i) check(leaderTorque.value[i] == 0.0, "unlinked leader: no feedforward");
	for (int i = 0; i < 7; ++i) check(followerTorque.value[i] == 0.0, "unlinked follower: no feedforward");

	// What the follower tracks: leader J1-J4, then the mapped wrist.
	for (int i = 0; i < 4; ++i) check(nearRef(followerRef.value[i], ljp[i]), "follower tracks leader J1-J4");
	using wam8g_teleop::FOLLOWER_SYNC_POSE;
	check(nearRef(followerRef.value[4], FOLLOWER_SYNC_POSE[4] + 0.2), "follower J5 = sync + wam8g J6 offset (+0.2)");
	check(nearRef(followerRef.value[5], FOLLOWER_SYNC_POSE[5] - 0.15), "follower J6 = sync - wam8g J7 offset (reversed)");
	check(nearRef(followerRef.value[6], FOLLOWER_SYNC_POSE[6] - 0.3), "follower J7 = sync + wam8g J8 offset (-0.3)");

	// What the leader tracks: follower J1-J4, its own J5, the wrist mapped back.
	for (int i = 0; i < 4; ++i) check(near(leaderRef.value[i], fjp[i]), "leader tracks follower J1-J4");
	check(near(leaderRef.value[4], ljp[4]), "leader J5 reference = measured J5");
	check(near(leaderRef.value[5], SYNC_POSE[5] + 0.25), "leader J6 = sync + follower J5");
	check(near(leaderRef.value[6], SYNC_POSE[6] + 0.1), "leader J7 = sync - follower J6 (reversed)");
	check(near(leaderRef.value[7], SYNC_POSE[7] + 0.6), "leader J8 = sync + follower J7");

	// The leader linked alone: the unlinked follower sends no external torque,
	// so the leader's law has no force channel yet.
	const double u6Own = leaderLaw == teleop_law::U6 ? -0.15 : 0.0;
	leader.tryLink();
	cycles(&lem, &fem, 50);
	for (int i = 0; i < 4; ++i) {
		check(near(leaderTorque.value[i], u6Own * lext[i] + ldyn[i] - lgrav[i]),
				"unlinked follower sends no external torque");
	}

	// Both linked: the force channels fade in over FORCE_FADE_TIME.
	follower.tryLink();
	cycles(&lem, &fem, 50);
	check(leader.isLinked() && follower.isLinked(), "both linked");
	const double leaderFade = (leaderTorque.value[0] - (u6Own * lext[0] + ldyn[0] - lgrav[0]))
			/ ((leaderLaw == teleop_law::U6 ? -0.65 : -0.5) * fext[0]);
	check(leaderFade > 0.0 && leaderFade < 0.1, "leader force channel fading in after 0.1 s");
	if (sendExtTorque) {
		const double followerFade = (followerTorque.value[0] - (fdyn[0] - fgrav[0])) / (-0.5 * lext[0]);
		check(followerFade > 0.0 && followerFade < 0.1, "follower force channel fading in after 0.1 s");
	}

	// Past the fade: the laws act on J1-J4, nothing on the wrists.
	cycles(&lem, &fem, 1050);
	check(leader.isLinked() && follower.isLinked(), "still linked");
	for (int i = 0; i < 8; ++i) {
		double want = 0.0;
		if (i < 4 && leaderLaw == teleop_law::U4) {
			want = -0.5 * fext[i] + ldyn[i] - lgrav[i];
		} else if (i < 4 && leaderLaw == teleop_law::U6) {
			want = -0.5 * fext[i] - 0.15 * (fext[i] + lext[i]) + ldyn[i] - lgrav[i];
		}
		check(near(leaderTorque.value[i], want), "leader law on J1-J4, zero on J5-J8");
	}
	for (int i = 0; i < 7; ++i) {
		const double refExt = sendExtTorque && i < 4 ? lext[i] : 0.0;
		const double want = i < 4 ? -0.5 * refExt + fdyn[i] - fgrav[i] : 0.0;
		check(near(followerTorque.value[i], want),
				sendExtTorque ? "follower u4 uses the leader's external torque"
						: "follower gets zero external torque from an unfitted leader");
	}

	// The mode on both mid-teleop ('c', 'p', `wam 4c|pp`): full -> 4C off
	// (u2) -> P-P (u1) -> full, each term fading over MODE_FADE_TIME.
	if (sendExtTorque) {
		const double lChannel = leaderLaw == teleop_law::U6 ? -0.65 : -0.5;  // on fext
		// the leader's and follower's J1 torque with the terms at these shares
		auto lWant = [&](int i, double force, double own, double dyn) {
			return force * lChannel * fext[i] + own * u6Own * lext[i] + dyn * (ldyn[i] - lgrav[i]);
		};
		auto fWant = [&](int i, double force, double dyn) {
			return force * -0.5 * lext[i] + dyn * (fdyn[i] - fgrav[i]);
		};
		auto near5 = [](double a, double b) { return std::fabs(a - b) <= 0.05 * std::fabs(b) + 1e-6; };

		leader.setMode(teleop_law::NO_4C);
		follower.setMode(teleop_law::NO_4C);
		cycles(&lem, &fem, 250);  // 0.5 s
		check(near5(leaderTorque.value[0], lWant(0, 0.5, 0.5, 1.0)) && near5(followerTorque.value[0], fWant(0, 0.5, 1.0)),
				"4C off: the force channels halfway out after 0.5 s");
		cycles(&lem, &fem, 300);
		for (int i = 0; i < 4; ++i) {
			check(near(leaderTorque.value[i], ldyn[i] - lgrav[i]) && near(followerTorque.value[i], fdyn[i] - fgrav[i]),
					"4C off: u2 on both after 1 s");
		}
		check(leader.isLinked() && follower.isLinked(), "4C off: still linked");

		leader.setMode(teleop_law::PP);
		follower.setMode(teleop_law::PP);
		cycles(&lem, &fem, 250);
		check(near5(leaderTorque.value[0], lWant(0, 0.0, 0.0, 0.5)) && near5(followerTorque.value[0], fWant(0, 0.0, 0.5)),
				"P-P: the dynamics compensation halfway out after 0.5 s");
		cycles(&lem, &fem, 300);
		for (int i = 0; i < 8; ++i) {
			check(near(leaderTorque.value[i], 0.0), "P-P: u1 (no feedforward) on the leader after 1 s");
		}
		for (int i = 0; i < 7; ++i) {
			check(near(followerTorque.value[i], 0.0), "P-P: u1 (no feedforward) on the follower after 1 s");
		}
		check(leader.isLinked() && follower.isLinked(), "P-P: still linked");

		leader.setMode(teleop_law::FULL);
		follower.setMode(teleop_law::FULL);
		cycles(&lem, &fem, 250);
		check(near5(leaderTorque.value[0], lWant(0, 0.5, 0.5, 0.5)) && near5(followerTorque.value[0], fWant(0, 0.5, 0.5)),
				"full again: every term halfway in after 0.5 s");
		cycles(&lem, &fem, 300);
		for (int i = 0; i < 4; ++i) {
			check(near(leaderTorque.value[i], lWant(i, 1.0, 1.0, 1.0)) && near(followerTorque.value[i], fWant(i, 1.0, 1.0)),
					"full again: the laws as launched on both after 1 s");
		}
	}

	// The follower stops: the leader drops the link and its feedforward.
	cycles(&lem, NULL, 30);
	check(!leader.isLinked(), "leader unlinks after 20 ms without messages");
	for (int i = 0; i < 8; ++i) check(leaderTorque.value[i] == 0.0, "no feedforward after the link is lost");
}

/* Both arms moving, each inside the Wam's torque graph: every cycle, the
 * position controller must track the reference its arm system made from the
 * newest packet that cycle, not the previous cycle's. */
void testFreshReference(int port) {
	printf("reference timing on ports %d/%d\n", port, port + 1);
	systems::ManualExecutionManager lem(0.002), fem(0.002);
	const double step = 1e-4;  // rad per cycle on J1: 0.05 rad/s

	WamStandIn<8> lWam(&lem);
	jp8 ljp;
	for (int i = 0; i < 8; ++i) ljp[i] = wam8g_teleop::SYNC_POSE[i];
	Moving<jp8> lJp(&lem, ljp, step);
	systems::Constant<jv8> lJv(jv8(0.0));
	LeaderWam8g leader(&lem, "127.0.0.1", port, port + 1, teleop_law::U4,
			wam8g_teleop::WristMap::coupled(), true);
	systems::connect(lJv.output, leader.wamJVIn);
	lWam.attach(&leader, lJp.output);
	Probe<jp8> leaderRef(&lem), leaderJp(&lem);
	Probe<jt8> leaderControl(&lem);
	systems::connect(leader.theirJPOutput, leaderRef.input);
	systems::connect(lJp.output, leaderJp.input);
	systems::connect(lWam.controller.controlOutput, leaderControl.input);

	WamStandIn<7> fWam(&fem);
	jp7 fjp;
	for (int i = 0; i < 7; ++i) fjp[i] = wam8g_teleop::FOLLOWER_SYNC_POSE[i];
	Moving<jp7> fJp(&fem, fjp, -step);
	systems::Constant<jv7> fJv(jv7(0.0));
	Follower<7> follower(&fem, "127.0.0.1", port + 1, port);
	follower.setLaw(4);
	systems::connect(fJv.output, follower.wamJVIn);
	fWam.attach(&follower, fJp.output);
	Probe<jp7> followerRef(&fem), followerJp(&fem);
	Probe<jt7> followerControl(&fem);
	systems::connect(follower.theirJPOutput, followerRef.input);
	systems::connect(fJp.output, followerJp.input);
	systems::connect(fWam.controller.controlOutput, followerControl.input);

	cycles(&lem, &fem, 100);
	leader.tryLink();
	follower.tryLink();
	const int n = 300;
	int leaderLate = 0, followerLate = 0;
	for (int k = 0; k < n; ++k) {
		cycles(&lem, &fem, 1);
		// kp 1: the controller's J1 torque plus the measured J1 is the reference it used.
		if (std::fabs(leaderControl.value[0] + leaderJp.value[0] - leaderRef.value[0]) > 1e-9) ++leaderLate;
		if (std::fabs(followerControl.value[0] + followerJp.value[0] - followerRef.value[0]) > 1e-9) ++followerLate;
	}
	printf("  cycles the controller used the previous reference: leader %d, follower %d of %d\n", leaderLate,
			followerLate, n);
	check(leader.isLinked() && follower.isLinked(), "reference timing: both linked");
	check(leaderLate == 0, "leader: the position controller tracks this cycle's reference");
	check(followerLate == 0, "follower: the position controller tracks this cycle's reference");
}

/* One arm stops (SafetyWatch -> halt()): it unlinks and sends nothing more,
 * so the other arm's link times out and it holds the stopped arm's last
 * position; the stopped arm tracks its own position and cannot link again. */
void testHalt(bool haltLeader, int port) {
	printf("halt the %s on ports %d/%d\n", haltLeader ? "leader" : "follower", port, port + 1);
	systems::ManualExecutionManager lem(0.002), fem(0.002);
	jp8 ljp;
	for (int i = 0; i < 8; ++i) ljp[i] = wam8g_teleop::SYNC_POSE[i] + 0.01 * i;
	systems::Constant<jp8> lJp(ljp);
	systems::Constant<jv8> lJv(jv8(0.0));
	systems::Constant<jt8> lZero(jt8(0.0));
	LeaderWam8g leader(&lem, "127.0.0.1", port, port + 1, teleop_law::U4, wam8g_teleop::WristMap::coupled(), true);
	systems::connect(lJp.output, leader.wamJPIn);
	systems::connect(lJv.output, leader.wamJVIn);
	systems::connect(lZero.output, leader.extTorqueIn);
	systems::connect(lZero.output, leader.wamGravIn);
	systems::connect(lZero.output, leader.wamDynIn);
	Probe<jp8> leaderRef(&lem);
	systems::connect(leader.theirJPOutput, leaderRef.input);

	jp7 fjp;
	for (int i = 0; i < 7; ++i) fjp[i] = wam8g_teleop::FOLLOWER_SYNC_POSE[i] - 0.02 * i;
	systems::Constant<jp7> fJp(fjp);
	systems::Constant<jv7> fJv(jv7(0.0));
	systems::Constant<jt7> fZero(jt7(0.0));
	Follower<7> follower(&fem, "127.0.0.1", port + 1, port);
	systems::connect(fJp.output, follower.wamJPIn);
	systems::connect(fJv.output, follower.wamJVIn);
	systems::connect(fZero.output, follower.extTorqueIn);
	systems::connect(fZero.output, follower.wamGravIn);
	systems::connect(fZero.output, follower.wamDynIn);
	Probe<jp7> followerRef(&fem);
	systems::connect(follower.theirJPOutput, followerRef.input);

	cycles(&lem, &fem, 100);
	leader.tryLink();
	follower.tryLink();
	cycles(&lem, &fem, 50);
	check(leader.isLinked() && follower.isLinked(), "halt: both linked first");

	if (haltLeader) {
		leader.halt();
	} else {
		follower.halt();
	}
	cycles(&lem, &fem, 30);
	check(!leader.isLinked() && !follower.isLinked(), "halt: both unlinked 60 ms later");
	if (haltLeader) {
		for (int i = 0; i < 8; ++i) check(near(leaderRef.value[i], ljp[i]), "halted leader: tracks its own position");
		for (int i = 0; i < 4; ++i) check(near(followerRef.value[i], ljp[i]), "follower: holds the leader's last position");
		leader.tryLink();
	} else {
		for (int i = 0; i < 7; ++i) check(near(followerRef.value[i], fjp[i]), "halted follower: tracks its own position");
		for (int i = 0; i < 4; ++i) check(near(leaderRef.value[i], fjp[i]), "leader: holds the follower's last position");
		follower.tryLink();
	}
	cycles(&lem, &fem, 5);
	check(!(haltLeader ? leader.isLinked() : follower.isLinked()), "halt: the stopped arm does not link again");
}

/* The leader's J6 friction feedforward: on J6 only, linked or not, off once
 * the teleop has stopped. */
void testJ6Friction(int port) {
	printf("J6 friction feedforward on port %d\n", port);
	systems::ManualExecutionManager lem(0.002);
	jp8 ljp;
	for (int i = 0; i < 8; ++i) ljp[i] = wam8g_teleop::SYNC_POSE[i];
	jv8 ljv(0.0);
	ljv[5] = 0.2;  // J6 turning
	ljv[6] = -0.3;
	systems::Constant<jp8> lJp(ljp);
	systems::Constant<jv8> lJv(ljv);
	systems::Constant<jt8> lZero(jt8(0.0));
	LeaderWam8g leader(&lem, "127.0.0.1", port, port + 1, teleop_law::U4, wam8g_teleop::WristMap::held(), true);
	systems::connect(lJp.output, leader.wamJPIn);
	systems::connect(lJv.output, leader.wamJVIn);
	systems::connect(lZero.output, leader.extTorqueIn);
	systems::connect(lZero.output, leader.wamGravIn);
	systems::connect(lZero.output, leader.wamDynIn);
	Probe<jt8> torque(&lem);
	systems::connect(leader.wamJPOutput, torque.input);

	cycles(&lem, NULL, 5);
	check(torque.value[5] == 0.0, "J6 friction: off by default");
	leader.setJ6Friction(0.25);
	cycles(&lem, NULL, 5);
	const double want = teleop_law::frictionFeedforward(0.25, 0.2);
	check(near(torque.value[5], want), "J6 friction: unlinked, with J6's direction");
	for (int i = 0; i < 8; ++i) {
		if (i != 5) check(torque.value[i] == 0.0, "J6 friction: nothing on the other joints");
	}
	leader.tryLink();
	cycles(&lem, NULL, 5);
	check(near(torque.value[5], want), "J6 friction: linked");
	leader.halt();
	cycles(&lem, NULL, 5);
	check(torque.value[5] == 0.0, "J6 friction: off once the teleop stopped");
}

/* dynamics_excitation's wrist hold: PD on J6-J8 only, clamped. */
void testWristHold() {
	systems::ManualExecutionManager em(0.002);
	jp8 hold(0.0), jp(0.0);
	jv8 jv(0.0);
	for (int i = 0; i < 8; ++i) {
		hold[i] = 0.1 * i;
		jp[i] = 0.1 * i;
	}
	jp[5] += 0.1;   // J6 0.1 rad past its hold angle
	jp[6] -= 1.0;   // J7 far off: clamped
	jv[7] = 2.0;    // J8 moving
	jp[1] += 0.5;   // an arm joint off: not held
	math::Vector<8>::type kp(0.0), kd(0.0);
	jt8 limit(0.0);
	for (int i = 0; i < 8; ++i) {
		kp[i] = 20.0 - i;
		kd[i] = 0.1;
		limit[i] = 5.0;
	}
	systems::Constant<jp8> cJp(jp);
	systems::Constant<jv8> cJv(jv);
	WristHold<8> wristHold(5, hold, kp, kd, limit);
	Probe<jt8> probe(&em);
	systems::connect(cJp.output, wristHold.jpInput);
	systems::connect(cJv.output, wristHold.jvInput);
	systems::connect(wristHold.output, probe.input);
	em.runExecutionCycle();
	for (int i = 0; i < 5; ++i) check(probe.value[i] == 0.0, "wrist hold: nothing on J1-J5");
	check(near(probe.value[5], -(20.0 - 5) * 0.1), "wrist hold: J6 pulled back");
	check(near(probe.value[6], 5.0), "wrist hold: J7 clamped to its limit");
	check(near(probe.value[7], -0.1 * 2.0), "wrist hold: J8 damped");
}


/* The two arm systems on fixed joint states, for the checks below. */
struct QuietPair {
	systems::ManualExecutionManager lem, fem;
	jp8 ljp;
	jp7 fjp;
	systems::Constant<jp8> lJp;
	systems::Constant<jv8> lJv;
	systems::Constant<jt8> lGrav, lZero;
	systems::Constant<jp7> fJp;
	systems::Constant<jv7> fJv;
	systems::Constant<jt7> fGrav, fZero;
	LeaderWam8g leader;
	Follower<7> follower;
	Probe<jp8> leaderRef;
	Probe<jp7> followerRef;

	static jp8 leaderPose() {
		jp8 q;
		for (int i = 0; i < 8; ++i) q[i] = wam8g_teleop::SYNC_POSE[i] + 0.01 * i;
		return q;
	}
	static jp7 followerPose() {
		jp7 q;
		for (int i = 0; i < 7; ++i) q[i] = wam8g_teleop::FOLLOWER_SYNC_POSE[i] - 0.02 * i;
		return q;
	}
	static jv8 leaderVelocity(double j1) {
		jv8 v(0.0);
		v[0] = j1;
		return v;
	}

	QuietPair(int port, double leaderJ1Velocity)
		: lem(0.002), fem(0.002), ljp(leaderPose()), fjp(followerPose()), lJp(ljp),
		  lJv(leaderVelocity(leaderJ1Velocity)), lGrav(jt8(3.0)), lZero(jt8(0.0)), fJp(fjp), fJv(jv7(0.0)),
		  fGrav(jt7(4.0)), fZero(jt7(0.0)),
		  leader(&lem, "127.0.0.1", port, port + 1, teleop_law::U4, wam8g_teleop::WristMap::coupled(), true),
		  follower(&fem, "127.0.0.1", port + 1, port), leaderRef(&lem), followerRef(&fem) {
		systems::connect(lJp.output, leader.wamJPIn);
		systems::connect(lJv.output, leader.wamJVIn);
		systems::connect(lZero.output, leader.extTorqueIn);
		systems::connect(lGrav.output, leader.wamGravIn);
		systems::connect(lZero.output, leader.wamDynIn);
		systems::connect(leader.theirJPOutput, leaderRef.input);
		systems::connect(fJp.output, follower.wamJPIn);
		systems::connect(fJv.output, follower.wamJVIn);
		systems::connect(fZero.output, follower.extTorqueIn);
		systems::connect(fGrav.output, follower.wamGravIn);
		systems::connect(fZero.output, follower.wamDynIn);
		systems::connect(follower.theirJPOutput, followerRef.input);
	}
};

/* A mode change on one arm reaches the other in its packets (no ROS). */
void testModeOverLink(int port) {
	printf("mode over the link on ports %d/%d\n", port, port + 1);
	QuietPair p(port, 0.0);
	cycles(&p.lem, &p.fem, 50);
	p.leader.tryLink();
	p.follower.tryLink();
	cycles(&p.lem, &p.fem, 20);
	check(p.leader.getMode() == teleop_law::FULL && p.follower.getMode() == teleop_law::FULL &&
			p.follower.getModeAdoptions() == 0, "mode over the link: both start in full");
	p.leader.setMode(teleop_law::PP);
	cycles(&p.lem, &p.fem, 5);
	check(p.follower.getMode() == teleop_law::PP && p.follower.getModeAdoptions() == 1,
			"mode over the link: the follower takes the leader's P-P");
	check(p.leader.getMode() == teleop_law::PP && p.leader.getModeAdoptions() == 0,
			"mode over the link: the leader keeps it (no echo)");
	p.follower.setMode(teleop_law::FULL);
	cycles(&p.lem, &p.fem, 5);
	check(p.leader.getMode() == teleop_law::FULL && p.leader.getModeAdoptions() == 1,
			"mode over the link: the leader takes the follower's full");
	check(p.leader.isLinked() && p.follower.isLinked(), "mode over the link: still linked");
}

/* --reference age: the leader's position advanced by the packet's age. */
void testAgeReference(int port) {
	printf("aged reference on ports %d/%d\n", port, port + 1);
	const double speed = 0.5;  // rad/s on the leader's J1
	QuietPair p(port, speed);
	p.follower.setAgeReference(true);
	double lead = 0.0;
	for (int k = 0; k < 100; ++k) {  // the follower's cycle starts 1.5 ms after the leader's
		p.lem.runExecutionCycle();
		usleep(1500);
		p.fem.runExecutionCycle();
		usleep(500);
		lead = p.followerRef.value[0] - p.ljp[0];
	}
	printf("  follower reference leads the leader's sample by %.2f ms of its velocity\n", 1e3 * lead / speed);
	check(lead > speed * 0.0012 && lead < speed * 0.0040, "aged reference: advanced by the packet's age (about 1.5 ms)");
	for (int i = 1; i < 4; ++i) check(near(p.followerRef.value[i], p.ljp[i]), "aged reference: a joint at rest is not moved");

	p.follower.setAgeReference(false);
	for (int k = 0; k < 5; ++k) {
		p.lem.runExecutionCycle();
		usleep(1500);
		p.fem.runExecutionCycle();
		usleep(500);
	}
	check(near(p.followerRef.value[0], p.ljp[0]), "hold reference: a new packet is used as sent");

	// The leader stops sending: the age grows, the advance is limited.
	p.follower.setAgeReference(true);
	p.lem.runExecutionCycle();
	for (int k = 0; k < 5; ++k) {
		usleep(2000);
		p.fem.runExecutionCycle();
	}
	check(near(p.followerRef.value[0], p.ljp[0] + speed * teleop_law::REFERENCE_AGE_LIMIT),
			"aged reference: limited when packets stop");
}

/* --record: one row per cycle from each arm system. */
void testRecording(int port) {
	printf("recording on ports %d/%d\n", port, port + 1);
	const char* lPath = "/tmp/wam8g_loopback_leader.tlog";
	const char* fPath = "/tmp/wam8g_loopback_follower.tlog";
	const int n = 150;
	{
		QuietPair p(port, 0.0);
		TeleopRecorder lRec(lPath, LeaderWam8g::recordColumns(), "loopback test");
		TeleopRecorder fRec(fPath, Follower<7>::recordColumns(), "loopback test");
		p.leader.setRecorder(&lRec);
		p.follower.setRecorder(&fRec);
		cycles(&p.lem, &p.fem, 100);
		p.leader.tryLink();
		p.follower.tryLink();
		cycles(&p.lem, &p.fem, n - 100);
		p.leader.setRecorder(NULL);
		p.follower.setRecorder(NULL);
		lRec.close();
		fRec.close();
		check(lRec.written() == (unsigned long) n && fRec.written() == (unsigned long) n, "recording: one row per cycle");
		check(lRec.dropped() == 0 && fRec.dropped() == 0 && !lRec.writeFailed(), "recording: nothing dropped");
	}
	std::ifstream in(lPath, std::ios::binary);
	std::string first, names, note;
	std::getline(in, first);
	std::getline(in, names);
	std::getline(in, note);
	std::vector<std::string> columns;
	std::stringstream ss(names);
	for (std::string c; std::getline(ss, c, ',');) columns.push_back(c);
	check(columns.size() == LeaderWam8g::recordColumns().size(), "recording: the header lists the columns");
	auto col = [&](const char* name) {
		for (size_t i = 0; i < columns.size(); ++i) {
			if (columns[i] == name) return (int) i;
		}
		return -1;
	};
	const int cCycle = col("cycle"), cQ1 = col("q_1"), cGrav8 = col("tau_grav_8"), cPid1 = col("tau_pid_1"),
			cRef1 = col("ref_1"), cAge = col("rx_age"), cAlive = col("link_alive"), cState = col("state"),
			cDt = col("dt"), cSeq = col("rx_seq");
	check(cCycle >= 0 && cQ1 >= 0 && cGrav8 >= 0 && cPid1 >= 0 && cRef1 >= 0 && cAge >= 0 && cAlive >= 0 && cState >= 0 &&
			cDt >= 0 && cSeq >= 0, "recording: the expected columns exist");
	std::vector<double> row(columns.size());
	int rows = 0;
	bool counting = true, values = true, ages = true, sequence = true;
	double lastCycle = 0.0, lastSeq = 0.0, lastState = -1.0;
	const jp8 ljp = QuietPair::leaderPose();
	const jp7 fjp = QuietPair::followerPose();
	while (in.read(reinterpret_cast<char*>(&row[0]), sizeof(double) * row.size())) {
		if (rows > 0) {
			counting = counting && row[cCycle] == lastCycle + 1.0 && row[cDt] > 0.0015 && row[cDt] < 0.05;
		}
		values = values && row[cQ1] == ljp[0] && row[cGrav8] == 3.0 && row[cPid1] != row[cPid1];  // pid not connected: NaN
		if (row[cAlive] == 1.0) {
			ages = ages && row[cAge] > -0.001 && row[cAge] < 0.02 && std::fabs(row[cRef1] - fjp[0]) < 1e-9;
			sequence = sequence && row[cSeq] >= lastSeq;
			lastSeq = row[cSeq];
		}
		lastCycle = row[cCycle];
		lastState = row[cState];
		++rows;
	}
	check(rows == n && counting, "recording: consecutive cycles with their period");
	check(values, "recording: the joints, gravity and (unconnected) controller torque of the cycle");
	check(ages && sequence && lastSeq > 100.0, "recording: the link's age, sequence and the reference used");
	check(lastState == 1.0, "recording: the last rows are linked");
	if (std::getenv("KEEP_TEST_RECORDINGS") == NULL) {  // set it to try scripts/teleop_log.py on them
		std::remove(lPath);
		std::remove(fPath);
	}
}

/* RegressorDynamics against the class it replaces, and the matched estimate
 * at rest against the legacy one (they must agree when nothing moves). */
void testEstimatorSystems() {
	printf("estimator systems\n");
	typedef units::JointAccelerations<7>::type ja7;
	systems::ManualExecutionManager em(0.002);
	jp7 q;
	jv7 dq;
	ja7 a;
	jt7 torque;
	for (int i = 0; i < 7; ++i) {
		q[i] = 0.3 * i - 0.8;
		dq[i] = 0.25 - 0.1 * i;
		a[i] = 1.5 - 0.6 * i;
		torque[i] = 5.0 + i;
	}
	systems::Constant<jp7> cQ(q);
	systems::Constant<jv7> cDq(dq), cRest(jv7(0.0));
	systems::Constant<ja7> cA(a);
	systems::Constant<jt7> cTorque(torque);

	FollowerDynamics<7> old(&em);
	systems::connect(cQ.output, old.jpInputDynamics);
	systems::connect(cDq.output, old.jvInputDynamics);
	systems::connect(cA.output, old.jaInputDynamics);
	RegressorDynamics<7> now(initialize_follower_beta(), 0.25);
	systems::connect(cQ.output, now.jpInput);
	systems::connect(cDq.output, now.jvInput);
	systems::connect(cA.output, now.jaInput);
	systems::connect(cA.output, now.jaEstimatorInput);
	Probe<jt7> pOld(&em), pNow(&em), pStatic(&em), pInertia(&em);
	systems::connect(old.dynamicsFeedFWD, pOld.input);
	systems::connect(now.dynamicsFeedFWD, pNow.input);
	systems::connect(now.staticOutput, pStatic.input);
	systems::connect(now.estimatorInertiaOutput, pInertia.input);

	// The arm at rest in the estimator's view (zero velocity into its
	// acceleration): matched = commanded torque - static model, as legacy.
	RegressorDynamics<7> resting(initialize_follower_beta(), 0.25);
	MatchedAcceleration<7> acceleration(25.0, 50.0, 0.002);
	MatchedExternalTorque<7> matched(25.0, 50.0, 0.002);
	DynamicExternalTorque<7> legacy(&em);
	systems::connect(cQ.output, resting.jpInput);
	systems::connect(cRest.output, resting.jvInput);
	systems::connect(cRest.output, acceleration.jvInput);
	systems::connect(acceleration.output, resting.jaEstimatorInput);
	systems::connect(cTorque.output, matched.wamTorqueSumIn);
	systems::connect(resting.staticOutput, matched.staticIn);
	systems::connect(resting.estimatorInertiaOutput, matched.inertiaIn);
	systems::connect(cTorque.output, legacy.wamTorqueSumIn);
	systems::connect(resting.dynamicsFeedFWD, legacy.wamDynamicsIn);
	Probe<jt7> pMatched(&em), pLegacy(&em);
	systems::connect(matched.wamExternalTorqueOut, pMatched.input);
	systems::connect(legacy.wamExternalTorqueOut, pLegacy.input);

	for (int k = 0; k < 5; ++k) em.runExecutionCycle();
	for (int i = 0; i < 4; ++i) {
		check(std::fabs(pOld.value[i] - pNow.value[i]) < 1e-9, "RegressorDynamics: FollowerDynamics' feedforward");
		check(std::fabs(pStatic.value[i] + 0.25 * pInertia.value[i] - pNow.value[i]) < 1e-9,
				"RegressorDynamics: feedforward = static + a quarter of the inertial torque");
		check(std::fabs(pInertia.value[i]) > 1e-3, "RegressorDynamics: an inertial torque for the estimator");
		check(std::fabs(pMatched.value[i] - pLegacy.value[i]) < 1e-9, "matched estimate: equals the legacy one at rest");
	}
	for (int i = 4; i < 7; ++i) {
		check(pNow.value[i] == 0.0 && pMatched.value[i] == 0.0, "estimator systems: nothing on the wrist");
	}
}

/* The control thread the library starts, and what the arm system makes of it
 * (lib/rt_thread.h): real-time on both libraries. */
void testRealtimeThread(int port) {
	printf("control thread on port %d\n", port);
	try {
		systems::RealTimeExecutionManager em(0.002, 50);
		Follower<7> follower(&em, "127.0.0.1", port + 1, port);
		follower.setRealtime(50, -1);
		em.start();
		usleep(100000);
		const rt_thread::Report& r = follower.getRealtimeReport();
		check(r.done != 0, "control thread: the arm system configured it in its first cycle");
		printf("  this library starts its control thread as %s; ", rt_thread::policyName(r.policyBefore));
		rt_thread::print("the arm system's", r);
		if (r.schedError == 0) {
			check(r.policyAfter == SCHED_FIFO || r.policyAfter == SCHED_RR, "control thread: real-time once the arm system runs");
		} else {
			printf("  (no real-time permission here: run in the wam_ros container)\n");
		}
		em.stop();
	} catch (const std::exception& e) {
		printf("  could not start a real-time execution manager here (%s)\n", e.what());
	}
}

}  // namespace

/* The follower's kinematics (config/follower-zeus-hand/wam7w.conf). */
const char* const FOLLOWER_KINEMATICS =
	"kinematics: {\n"
	"  world_to_base = ((1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1));\n"
	"  moving: (\n"
	"    { alpha_pi = -0.5; a =      0; d =      0; },\n"
	"    { alpha_pi =  0.5; a =      0; d =      0; },\n"
	"    { alpha_pi = -0.5; a =  0.045; d = 0.5500; },\n"
	"    { alpha_pi =  0.5; a = -0.045; d =      0; },\n"
	"    { alpha_pi = -0.5; a =      0; d = 0.3000; },\n"
	"    { alpha_pi =  0.5; a =      0; d =      0; },\n"
	"    { alpha_pi =    0; a =      0; d = 0.0609; }\n"
	"  );\n"
	"  toolplate = { alpha_pi = 0; theta_pi = 0; a = 0; d = 0; };\n"
	"};\n";

/* A force/torque sensor that reads what the test says. */
struct FakeFt : public FtSource {
	double force[3], torque[3];
	bool answering;
	long polls;
	FakeFt() : answering(true), polls(0) {
		for (int i = 0; i < 3; ++i) force[i] = torque[i] = 0.0;
	}
	virtual bool poll(double* f, double* n) {
		++polls;
		if (!answering) {
			return false;
		}
		for (int i = 0; i < 3; ++i) {
			f[i] = force[i];
			n[i] = torque[i];
		}
		return true;
	}
};

/* The tool's position and rotation (tool axes as columns, in the world) at q. */
struct ToolPose {
	double p[3], r[9];
};
ToolPose toolPose(math::Kinematics<7>& kin, const jp7& q) {
	kin.eval(q, jv7(0.0));
	ToolPose t;
	for (int i = 0; i < 3; ++i) {
		t.p[i] = gsl_vector_get(kin.impl->tool->origin_pos, i);
		for (int j = 0; j < 3; ++j) t.r[3 * i + j] = gsl_matrix_get(kin.impl->tool->rot_to_world, i, j);
	}
	return t;
}

/* The joint torques a wrench at the tool (tool axes, about its origin) causes,
 * from virtual work with numerical derivatives of the tool's pose: no
 * Jacobian of the library's. */
jt7 wrenchTorques(math::Kinematics<7>& kin, const jp7& q, const double* toolForce, const double* toolTorque) {
	const ToolPose at = toolPose(kin, q);
	double fw[3], nw[3];
	ft_core::rotate(at.r, toolForce, fw);
	ft_core::rotate(at.r, toolTorque, nw);
	jt7 tau;
	const double h = 1e-6;
	for (int j = 0; j < 7; ++j) {
		jp7 a(q), b(q);
		a[j] += h;
		b[j] -= h;
		const ToolPose plus = toolPose(kin, a), minus = toolPose(kin, b);
		double dp[3], dr[9], w[9];
		for (int i = 0; i < 3; ++i) dp[i] = (plus.p[i] - minus.p[i]) / (2 * h);
		for (int i = 0; i < 9; ++i) dr[i] = (plus.r[i] - minus.r[i]) / (2 * h);
		for (int i = 0; i < 3; ++i) {  // (dR/dq) R^T = [omega]x
			for (int k = 0; k < 3; ++k) {
				w[3 * i + k] = dr[3 * i] * at.r[3 * k] + dr[3 * i + 1] * at.r[3 * k + 1] + dr[3 * i + 2] * at.r[3 * k + 2];
			}
		}
		const double omega[3] = {w[7], w[2], w[3]};
		tau[j] = fw[0] * dp[0] + fw[1] * dp[1] + fw[2] * dp[2] + nw[0] * omega[0] + nw[1] * omega[1] + nw[2] * omega[2];
	}
	return tau;
}

/* The library's frames as lib/ft_systems.h takes them, and FtTorques' joint
 * torques against virtual work. */
void testFtTorques() {
	printf("force/torque sensor: frames and joint torques\n");
	libconfig::Config config;
	config.readString(FOLLOWER_KINEMATICS);
	math::Kinematics<7> kin(config.lookup("kinematics"));

	// rot_to_world's columns are the tool's axes in the world: its third one
	// is the direction from J7's frame... the last link's offset (d = 0.0609
	// along the tool's z) from the wrist centre to the tool plate.
	jp7 q;
	q << 0.3, -0.7, 0.4, 1.9, -0.6, 0.5, 0.8;
	const ToolPose at = toolPose(kin, q);
	double wrist[3];
	for (int i = 0; i < 3; ++i) wrist[i] = gsl_vector_get(kin.impl->link[5]->origin_pos, i);
	bool column = true, row = true;
	for (int i = 0; i < 3; ++i) {
		const double along = (at.p[i] - wrist[i]) / 0.0609;
		column = column && std::fabs(at.r[3 * i + 2] - along) < 1e-9;
		row = row && std::fabs(at.r[6 + i] - along) < 1e-9;
	}
	check(column && !row, "ft: rot_to_world turns tool-frame vectors into the world's (its columns are the tool's axes)");
	q.setZero();
	const ToolPose up = toolPose(kin, q);
	double g[3];
	ft_core::gravityInTool(up.r, g);
	check(near(up.p[2], 0.55 + 0.30 + 0.0609) && near(g[2], -1.0), "ft: the arm straight up has gravity along the tool's -z");

	// FtTorques on that kinematics, a sensor with its axes as the tool's.
	systems::ManualExecutionManager em(0.002);
	q << 0.3, -0.7, 0.4, 1.9, -0.6, 0.5, 0.8;
	systems::Constant<jp7> jp(q);
	systems::Constant<jv7> jv(jv7(0.0));
	systems::KinematicsBase<7> kinBase(config.lookup("kinematics"));
	systems::connect(jp.output, kinBase.jpInput);
	systems::connect(jv.output, kinBase.jvInput);
	FakeFt sensor;
	ft_core::Limits limits;
	FtTorques<7> ft(&sensor, ft_core::Tool(), limits, 0.002);
	systems::connect(kinBase.kinOutput, ft.kinInput);
	Probe<jt7> tau(&em);
	systems::connect(ft.output, tau.input);

	const double bias[3] = {2.0, -3.0, 5.0};
	for (int i = 0; i < 3; ++i) sensor.force[i] = bias[i];
	cycles(&em, NULL, 20);
	for (int i = 0; i < 7; ++i) check(tau.value[i] == 0.0, "ft: no joint torques before the zero");
	ft.startZero(0.1);
	cycles(&em, NULL, 60);
	check(ft.zerosDone() == 1, "ft: the zero is taken in the loop");
	const double pushF[3] = {4.0, -7.5, 11.0}, pushN[3] = {0.3, 0.45, -0.2};
	for (int i = 0; i < 3; ++i) {
		sensor.force[i] = bias[i] + pushF[i];
		sensor.torque[i] = pushN[i];
	}
	cycles(&em, NULL, 500);
	const jt7 want = wrenchTorques(kin, q, pushF, pushN);
	double largest = 0.0;
	for (int i = 0; i < 7; ++i) {
		check(std::fabs(tau.value[i] - want[i]) < 1e-5, "ft: joint torques = J^T wrench (virtual work, every joint)");
		check(std::fabs(ft.report().jointTorque[i] - want[i]) < 1e-5, "ft: ... and in the report");
		largest = std::max(largest, std::fabs(want[i]));
	}
	check(largest > 3.0 && std::fabs(want[6]) > 0.05, "ft: (the wrench used turns every part of the arm)");
	check(ft.report().use == 1.0 && ft.report().fresh == 1.0 && sensor.polls == 580, "ft: one poll per cycle");
	sensor.answering = false;
	cycles(&em, NULL, 80);
	for (int i = 0; i < 7; ++i) check(tau.value[i] == 0.0, "ft: no joint torques once the sensor stops answering");
	check(std::fabs(ft.report().jointTorque[1] - want[1]) < 1e-5 && ft.report().use == 0.0, "ft: (its last wrench held, out of use)");
}

/* A leader-follower pair with the follower's sensor: its torques reach the
 * leader's J1-J4 (ft arm) and wrist (ft wrist, wrist force gain), the tether,
 * and the way back to the plain coupling when the sensor stops or 4C is off. */
void testFtPair(int port) {
	printf("force/torque sensor: pair on ports %d/%d\n", port, port + 1);
	using wam8g_teleop::SYNC_POSE;
	libconfig::Config config;
	config.readString(FOLLOWER_KINEMATICS);
	math::Kinematics<7> kin(config.lookup("kinematics"));

	QuietPair p(port, 0.0);
	// The follower's model estimate, to tell it from the sensor's.
	jt7 fext;
	for (int i = 0; i < 7; ++i) fext[i] = 2.0 + i;
	systems::Constant<jt7> fExt(fext);
	systems::forceConnect(fExt.output, p.follower.extTorqueIn);
	Probe<jt8> leaderTorque(&p.lem);
	systems::connect(p.leader.wamJPOutput, leaderTorque.input);
	Probe<jt7> followerTorque(&p.fem);
	systems::connect(p.follower.wamJPOutput, followerTorque.input);

	systems::KinematicsBase<7> kinBase(config.lookup("kinematics"));
	systems::connect(p.fJp.output, kinBase.jpInput);
	systems::connect(p.fJv.output, kinBase.jvInput);
	FakeFt sensor;
	ft_core::Limits limits;
	FtTorques<7> ft(&sensor, ft_core::Tool(), limits, 0.002);
	systems::connect(kinBase.kinOutput, ft.kinInput);
	systems::connect(ft.output, p.follower.ftTorqueIn);
	p.follower.setFt(&ft.report(), true, true);

	const char* lPath = "/tmp/wam8g_loopback_ft_leader.tlog";
	const char* fPath = "/tmp/wam8g_loopback_ft_follower.tlog";
	TeleopRecorder lRec(lPath, LeaderWam8g::recordColumns(), "loopback test: force/torque sensor");
	TeleopRecorder fRec(fPath, Follower<7>::recordColumns(), "loopback test: force/torque sensor");
	p.leader.setRecorder(&lRec);
	p.follower.setRecorder(&fRec);

	const double gains[3] = {1.0, 0.5, 0.25};
	const double deadband = 3.0 * M_PI / 180.0;
	p.leader.setWristForce(gains);
	p.leader.setWristForceFilter(15.0, 0.0);
	p.leader.setWristDeadband(deadband);

	// The wrist's position errors as the leader sees them: J6 and J8 outside
	// the 3 degree tether, J7 inside.
	const double ref[3] = {SYNC_POSE[5] + (p.fjp[4] - wam8g_teleop::FOLLOWER_SYNC_POSE[4]),
	                       SYNC_POSE[6] - (p.fjp[5] - wam8g_teleop::FOLLOWER_SYNC_POSE[5]),
	                       SYNC_POSE[7] + (p.fjp[6] - wam8g_teleop::FOLLOWER_SYNC_POSE[6])};
	const double error[3] = {ref[0] - p.ljp[5], ref[1] - p.ljp[6], ref[2] - p.ljp[7]};
	check(error[0] < -deadband && std::fabs(error[1]) < deadband && error[2] < -deadband, "ft pair: (errors on both sides of the tether)");

	const double pushF[3] = {1.0, 0.5, -0.8}, pushN[3] = {0.10, -0.20, 0.15};
	const jt7 tauEnv = wrenchTorques(kin, p.fjp, pushF, pushN);
	const double wristSign[3] = {1.0, -1.0, 1.0};
	// The leader's u4 on J1-J4 with this pair's model (dyn 0, grav 3): -0.5 ext_follower - 3.
	const double own = -3.0;

	// Linked, the sensor not zeroed yet: everything as without it.
	cycles(&p.lem, &p.fem, 50);
	p.leader.tryLink();
	p.follower.tryLink();
	cycles(&p.lem, &p.fem, 1100);
	check(p.leader.isLinked() && p.follower.isLinked(), "ft pair: linked");
	for (int i = 0; i < 4; ++i) check(near(leaderTorque.value[i], -0.5 * fext[i] + own), "ft pair: not zeroed: J1-J4 from the model's estimate");
	for (int k = 0; k < 3; ++k) {
		check(leaderTorque.value[5 + k] == 0.0, "ft pair: not zeroed: no wrist torque");
		check(near(p.leaderRef.value[5 + k], ref[k]), "ft pair: not zeroed: the plain wrist coupling");
	}
	check(p.leader.getWristUse() == 0.0, "ft pair: not zeroed: the leader sees no measured wrist torques");

	// Zeroed, in free space: J1-J4 get the sensor's nothing instead of the
	// model's estimate; the wrist is slack within the tether.
	ft.startZero(0.1);
	cycles(&p.lem, &p.fem, 500);
	check(ft.zerosDone() == 1 && ft.report().use == 1.0 && p.leader.getWristUse() == 1.0, "ft pair: zeroed and in use on both sides");
	for (int i = 0; i < 4; ++i) check(std::fabs(leaderTorque.value[i] - own) < 1e-9, "ft pair: free space: no force channel on J1-J4");
	for (int k = 0; k < 3; ++k) check(std::fabs(leaderTorque.value[5 + k]) < 1e-9, "ft pair: free space: nothing on the wrist");
	check(near(p.leaderRef.value[5], p.ljp[5] + error[0] + deadband), "ft pair: J6 beyond the tether: pulled by what is beyond");
	check(near(p.leaderRef.value[6], p.ljp[6]), "ft pair: J7 within the tether: no pull");
	check(near(p.leaderRef.value[7], p.ljp[7] + error[2] + deadband), "ft pair: J8 beyond the tether: pulled by what is beyond");
	for (int i = 0; i < 4; ++i) check(near(p.leaderRef.value[i], p.fjp[i]), "ft pair: J1-J4's coupling untouched");

	// A push on the gripper: J1-J4 get half its joint torques (u4), the wrist
	// its wrist torques times the gain, with the position map's signs.
	for (int i = 0; i < 3; ++i) {
		sensor.force[i] = pushF[i];
		sensor.torque[i] = pushN[i];
	}
	cycles(&p.lem, &p.fem, 500);
	for (int i = 0; i < 4; ++i) {
		check(std::fabs(leaderTorque.value[i] - (0.5 * tauEnv[i] + own)) < 1e-5, "ft pair: a push: J1-J4 pushed the way the follower is");
	}
	bool wristFelt = true;
	for (int k = 0; k < 3; ++k) {
		const double want = gains[k] * wristSign[k] * tauEnv[4 + k];
		check(std::fabs(leaderTorque.value[5 + k] - want) < 1e-5, "ft pair: a push: the wrist torque, gain and sign per joint");
		wristFelt = wristFelt && std::fabs(want) > 0.02 && std::fabs(want) < LeaderWam8g::wristForceLimit();
	}
	check(wristFelt, "ft pair: (the push turns every wrist joint, under the limit)");
	for (int i = 4; i < 7; ++i) check(followerTorque.value[i] == 0.0, "ft pair: the follower's own wrist gets nothing");

	// A small wrist torque stays out (the gripper's inertia as the wrist turns).
	p.leader.setWristForceFilter(15.0, 0.5);
	cycles(&p.lem, &p.fem, 100);
	for (int k = 0; k < 3; ++k) check(leaderTorque.value[5 + k] == 0.0, "ft pair: wrist torques under the deadzone stay out");
	p.leader.setWristForceFilter(15.0, 0.0);
	cycles(&p.lem, &p.fem, 300);

	// 4C off: the wrist's force channel and tether go with J1-J4's.
	p.leader.setMode(teleop_law::NO_4C);
	cycles(&p.lem, &p.fem, 600);
	for (int k = 0; k < 3; ++k) {
		check(leaderTorque.value[5 + k] == 0.0, "ft pair: 4C off: no wrist torque");
		check(near(p.leaderRef.value[5 + k], ref[k]), "ft pair: 4C off: the plain wrist coupling");
	}
	p.leader.setMode(teleop_law::FULL);
	cycles(&p.lem, &p.fem, 600);
	check(std::fabs(leaderTorque.value[5] - gains[0] * tauEnv[4]) < 1e-5 && near(p.leaderRef.value[6], p.ljp[6]),
			"ft pair: full again: both back");

	// The sensor stops answering: within half a second J1-J4 are back on the
	// model's estimate and the wrist on the plain coupling, without a step.
	sensor.answering = false;
	double largestStep = 0.0, last = leaderTorque.value[5];
	for (int n = 0; n < 250; ++n) {
		cycles(&p.lem, &p.fem, 1);
		largestStep = std::max(largestStep, std::fabs(leaderTorque.value[5] - last));
		last = leaderTorque.value[5];
	}
	check(largestStep < 0.05 * std::fabs(gains[0] * tauEnv[4]) + 1e-9, "ft pair: sensor lost: the wrist torque fades, no step");
	for (int i = 0; i < 4; ++i) check(near(leaderTorque.value[i], -0.5 * fext[i] + own), "ft pair: sensor lost: J1-J4 back on the model's estimate");
	for (int k = 0; k < 3; ++k) {
		check(leaderTorque.value[5 + k] == 0.0, "ft pair: sensor lost: no wrist torque");
		check(near(p.leaderRef.value[5 + k], ref[k]), "ft pair: sensor lost: the plain wrist coupling");
	}
	check(p.leader.getWristUse() == 0.0 && p.leader.isLinked() && p.follower.isLinked(), "ft pair: sensor lost: still linked");

	// It answers again: back in use by itself.
	sensor.answering = true;
	cycles(&p.lem, &p.fem, 600);
	check(std::fabs(leaderTorque.value[5] - gains[0] * tauEnv[4]) < 1e-5 && near(p.leaderRef.value[6], p.ljp[6]),
			"ft pair: sensor back: in use again");

	// The tether without a force gain is always slack while linked (a test of the coupling alone).
	const double none[3] = {0.0, 0.0, 0.0};
	p.leader.setWristForce(none);
	sensor.answering = false;
	cycles(&p.lem, &p.fem, 300);
	check(near(p.leaderRef.value[6], p.ljp[6]) && leaderTorque.value[5] == 0.0, "ft pair: the tether alone: slack without the sensor");

	p.leader.setRecorder(NULL);
	p.follower.setRecorder(NULL);
	lRec.close();
	fRec.close();
	check(lRec.dropped() == 0 && fRec.dropped() == 0, "ft pair: recorded");
	if (std::getenv("KEEP_TEST_RECORDINGS") == NULL) {
		std::remove(lPath);
		std::remove(fPath);
	}
}

/* follower --wrist-friction: a torque on its J5-J7 in the direction the
 * leader's wrist turns, while linked. */
void testWristFriction(int port) {
	printf("follower wrist friction feedforward on ports %d/%d\n", port, port + 1);
	QuietPair p(port, 0.0);
	jv8 ljv(0.0);
	ljv[5] = 0.5;   // J6 -> follower J5
	ljv[6] = 0.7;   // J7 -> follower J6, reversed
	ljv[7] = -0.4;  // J8 -> follower J7
	systems::Constant<jv8> lJv(ljv);
	systems::forceConnect(lJv.output, p.leader.wamJVIn);
	Probe<jt7> followerTorque(&p.fem);
	systems::connect(p.follower.wamJPOutput, followerTorque.input);
	const double nm[3] = {0.2, 0.1, 0.05};
	p.follower.setWristFriction(nm);
	cycles(&p.lem, &p.fem, 50);
	for (int i = 4; i < 7; ++i) check(followerTorque.value[i] == 0.0, "wrist friction: nothing unlinked");
	p.leader.tryLink();
	p.follower.tryLink();
	cycles(&p.lem, &p.fem, 50);
	check(near(followerTorque.value[4], teleop_law::frictionFeedforward(0.2, 0.5)) &&
			near(followerTorque.value[5], teleop_law::frictionFeedforward(0.1, -0.7)) &&
			near(followerTorque.value[6], teleop_law::frictionFeedforward(0.05, -0.4)),
			"wrist friction: on J5-J7, the way the leader's wrist turns");
	check(followerTorque.value[4] > 0.19 && followerTorque.value[5] < -0.09, "wrist friction: (nearly all of it at these speeds)");
}

int main() {
	testFtTorques();
	testFtPair(25654);
	testWristFriction(25664);
	testWristHold();
	runPair(teleop_law::U4, false, 25554);
	runPair(teleop_law::U6, true, 25564);
	testFreshReference(25574);
	testHalt(true, 25584);
	testHalt(false, 25594);
	testJ6Friction(25604);
	testModeOverLink(25614);
	testAgeReference(25624);
	testRecording(25634);
	testEstimatorSystems();
	testRealtimeThread(25644);
	printf("%d checks, %d failed\n", checks, failures);
	return failures == 0 ? 0 : 1;
}
