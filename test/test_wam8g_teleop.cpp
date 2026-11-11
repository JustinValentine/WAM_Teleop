/* Offline checks for the wam8g teleop pieces that do not need a robot:
 * the wrist map and the feedforward laws.
 * Run: devel/lib/wam_teleop/wam8g_teleop_tests */

#include <cmath>
#include <cstdio>

#include <Eigen/Core>

#include "teleop_law.h"
#include "wam8g_teleop.h"
#include "wam8g_wrist_map.h"

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

bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

void testWristMap() {
	using wam8g_teleop::SYNC_POSE;
	using wam8g_teleop::FOLLOWER_SYNC_POSE;
	typedef Eigen::Matrix<double, 8, 1> Leader;
	typedef Eigen::Matrix<double, 3, 1> Wrist;

	for (int j = 0; j < 4; ++j) {
		check(SYNC_POSE[j] == FOLLOWER_SYNC_POSE[j], "both arms sync J1-J4 to the same angles");
	}
	// The follower's sync wrist (J5 -pi/2, J6 0, J7 +pi/2): J6 at 0 makes J5
	// and J7 coaxial, and their turns cancel, so the hand points as at zero.
	check(FOLLOWER_SYNC_POSE[5] == 0.0 && FOLLOWER_SYNC_POSE[4] + FOLLOWER_SYNC_POSE[6] == 0.0 &&
			near(FOLLOWER_SYNC_POSE[4], -M_PI / 2, 1e-15),
			"follower sync wrist: J5 -pi/2, J6 0, J7 +pi/2 (the hand as at zero)");

	Leader sync;
	for (int j = 0; j < 8; ++j) sync[j] = SYNC_POSE[j];
	Leader moved = sync;
	moved[5] += 0.3;   // J6
	moved[6] -= 0.2;   // J7
	moved[7] += 0.25;  // J8

	// held(): both wrists stay at their sync angles.
	const wam8g_teleop::WristMap held = wam8g_teleop::WristMap::held();
	Wrist f;
	held.toFollower(moved, &f);
	for (int k = 0; k < 3; ++k) {
		check(f[k] == FOLLOWER_SYNC_POSE[4 + k], "held: the follower wrist stays at its sync angles");
	}
	Leader back = moved;
	Wrist anything(0.4, -0.3, 0.2);
	held.toLeader(anything, &back);
	for (int j = 5; j < 8; ++j) check(back[j] == SYNC_POSE[j], "held: the leader wrist reference is its sync pose");
	for (int j = 0; j < 5; ++j) check(back[j] == moved[j], "toLeader leaves J1-J5 alone");
	check(!held.couplesAnything(), "held couples nothing");

	// coupled(): J6 -> J5, J7 -> J6 (reversed), J8 -> J7.
	const wam8g_teleop::WristMap coupled = wam8g_teleop::WristMap::coupled();
	// Linking does not jump: each arm at its sync pose is the other's reference.
	coupled.toFollower(sync, &f);
	for (int k = 0; k < 3; ++k) {
		check(near(f[k], FOLLOWER_SYNC_POSE[4 + k], 1e-15), "coupled: the leader at sync -> the follower's sync wrist");
	}
	Wrist fsync(FOLLOWER_SYNC_POSE[4], FOLLOWER_SYNC_POSE[5], FOLLOWER_SYNC_POSE[6]);
	Leader lback = moved;
	coupled.toLeader(fsync, &lback);
	for (int j = 5; j < 8; ++j) {
		check(near(lback[j], SYNC_POSE[j], 1e-15), "coupled: the follower at sync -> the leader's sync wrist");
	}
	coupled.toFollower(moved, &f);
	check(near(f[0] - FOLLOWER_SYNC_POSE[4], 0.3, 1e-12), "coupled: wam8g J6 +0.3 -> P2 J5 +0.3");
	check(near(f[1] - FOLLOWER_SYNC_POSE[5], 0.2, 1e-12), "coupled: wam8g J7 -0.2 -> P2 J6 +0.2 (reversed)");
	check(near(f[2] - FOLLOWER_SYNC_POSE[6], 0.25, 1e-12), "coupled: wam8g J8 +0.25 -> P2 J7 +0.25");
	back = sync;
	coupled.toLeader(f, &back);
	for (int j = 5; j < 8; ++j) check(near(back[j], moved[j], 1e-12), "coupled: round trip J6-J8");
	Leader v;
	v.setZero();
	v[5] = 1.0;
	v[6] = 2.0;
	v[7] = 3.0;
	coupled.velocityToFollower(v, &f);
	check(f[0] == 1.0 && f[1] == -2.0 && f[2] == 3.0, "coupled: velocities (J7 -> J6 reversed)");
}

void testLaws() {
	typedef Eigen::Matrix<double, 8, 1> JT;
	JT other, own, grav, dyn;
	for (int i = 0; i < 8; ++i) {
		other[i] = 1.0 + i;
		own[i] = 0.5 * i;
		grav[i] = 2.0 + i;
		dyn[i] = 3.0 + 2 * i;
	}
	const JT u1 = teleop_law::compute(teleop_law::U1, other, own, grav, dyn);
	const JT u2 = teleop_law::compute(teleop_law::U2, other, own, grav, dyn);
	const JT u4 = teleop_law::compute(teleop_law::U4, other, own, grav, dyn);
	const JT u6 = teleop_law::compute(teleop_law::U6, other, own, grav, dyn);
	for (int i = 0; i < 8; ++i) {
		const bool arm = i < 4;
		check(u1[i] == 0.0, "u1 is zero");
		check(u2[i] == (arm ? dyn[i] - grav[i] : 0.0), "u2 = dyn - grav on J1-J4 only");
		check(u4[i] == (arm ? -0.5 * other[i] + dyn[i] - grav[i] : 0.0), "u4 on J1-J4 only");
		check(near(u6[i], arm ? -0.5 * other[i] - 0.15 * (other[i] + own[i]) + dyn[i] - grav[i] : 0.0, 1e-12),
				"u6 on J1-J4 only");
	}
	teleop_law::Law law;
	check(teleop_law::parse("u4", &law) && law == teleop_law::U4, "parse u4");
	check(!teleop_law::parse("u3", &law), "u3 is not offered");
	check(!teleop_law::usesModel(teleop_law::U1) && teleop_law::usesModel(teleop_law::U4), "which laws use the model");
}

}  // namespace

/* The force channel: zero until both arms are linked, then faded in. */
void testForceChannel() {
	typedef Eigen::Matrix<double, 8, 1> JT;
	const double T = 0.002;
	JT ext;
	ext << 4, -2, 1, 3, 0, 0, 0, 0;
	double t = 0.0;

	JT u = teleop_law::forceChannel(ext, false, T, &t);
	check(u.norm() == 0.0 && t == 0.0, "force channel: nothing while this arm is unlinked");
	u = teleop_law::forceChannel(JT(JT::Zero()), true, T, &t);
	check(u.norm() == 0.0 && t == 0.0, "force channel: nothing while the other arm sends zero");

	u = teleop_law::forceChannel(ext, true, T, &t);
	check(near(u[0], (T / teleop_law::FORCE_FADE_TIME) * ext[0], 1e-12), "force channel: first step faded");
	for (int n = 1; n < 1500; ++n) u = teleop_law::forceChannel(ext, true, T, &t);
	check((u - ext).norm() < 1e-9, "force channel: the full ext_other after the fade");

	u = teleop_law::forceChannel(ext, false, T, &t);
	check(u.norm() == 0.0 && t == 0.0, "force channel: reset when the link drops");

	teleop_law::ForceGains gains;
	check(gains.g[2] == teleop_law::FORCE_GAIN, "--force-gain default 0.5");
	check(teleop_law::parseForceGain("0.25", &gains) && gains.g[0] == 0.25 && gains.g[3] == 0.25,
			"--force-gain 0.25 on J1-J4");
	check(teleop_law::parseForceGain("0.4,0.4,0.2,0.4", &gains) && gains.g[2] == 0.2 && gains.g[3] == 0.4,
			"--force-gain per joint");
	check(!teleop_law::parseForceGain("1.5", &gains) && !teleop_law::parseForceGain("x", &gains) &&
			!teleop_law::parseForceGain("0.4,0.2", &gains) && !teleop_law::parseForceGain("0.4,0.4,0.2,0.4,0.1", &gains) &&
			!teleop_law::parseForceGain("0.4,", &gains) && gains.g[2] == 0.2,
			"--force-gain rejects out-of-range, non-numbers, and two or five values");
}

/* The other arm's position as the reference: advanced between packets, then held. */
void testReferenceTracker() {
	typedef Eigen::Matrix<double, 2, 1> V;
	teleop_law::ReferenceTracker<V, int> r;
	const double T = 0.002;
	V jp(1.0, -1.0), jv(0.5, -2.0);
	r.update(jp, jv, 7, T);
	check(r.position == jp, "reference: a new packet sets the position");
	r.update(jp, jv, 7, T);
	check(near(r.position[0], 1.0 + 0.5 * T, 1e-15) && near(r.position[1], -1.0 - 2.0 * T, 1e-15),
			"reference: no new packet, advanced one cycle");
	r.update(jp, jv, 7, T);
	r.update(jp, jv, 7, T);
	check(near(r.position[0], 1.0 + 2 * 0.5 * T, 1e-15), "reference: held after two advanced cycles");
	V jp2(1.1, -1.1);
	r.update(jp2, jv, 8, T);
	check(r.position == jp2, "reference: the next packet replaces the advanced position");

	typedef Eigen::Matrix<double, 8, 1> JT;
	JT x = JT::Constant(2.0), s = JT::Zero();
	check(teleop_law::lowPass(x, 0.0, T, &s) == x, "low-pass off passes through");
	s.setZero();
	JT y = teleop_law::lowPass(x, 15.0, T, &s);
	check(near(y[0], 2.0 * (1.0 - std::exp(-2.0 * M_PI * 15.0 * T)), 1e-12), "low-pass 15 Hz first step");
	for (int n = 0; n < 500; ++n) y = teleop_law::lowPass(x, 15.0, T, &s);
	check(near(y[0], 2.0, 1e-9), "low-pass 15 Hz settles");
	double hz = 0.0;
	check(teleop_law::parseForceFilter("15", &hz) && hz == 15.0 && !teleop_law::parseForceFilter("-1", &hz),
			"--force-filter 15, rejects -1");
}

/* The notch's steady-state gain and phase lag (degrees) at f Hz, at 35 Hz. */
void notchResponse(double f, double* gain, double* lagDeg) {
	typedef Eigen::Matrix<double, 8, 1> JT;
	const double T = 0.002;
	teleop_law::Notch<JT> notch;
	double s = 0.0, c = 0.0;
	const int n = 4000, settle = 2000;
	for (int k = 0; k < n; ++k) {
		const double y = notch(JT::Constant(std::sin(2.0 * M_PI * f * k * T)), 35.0, T)[0];
		if (k >= settle) {
			s += y * std::sin(2.0 * M_PI * f * k * T);
			c += y * std::cos(2.0 * M_PI * f * k * T);
		}
	}
	*gain = 2.0 * std::sqrt(s * s + c * c) / (n - settle);
	*lagDeg = -std::atan2(c, s) * 180.0 / M_PI;
}

void testNotch() {
	typedef Eigen::Matrix<double, 8, 1> JT;
	const double T = 0.002;
	teleop_law::Notch<JT> off;
	const JT x = JT::Constant(1.5);
	check(off(x, 0.0, T) == x, "notch off passes through");
	teleop_law::Notch<JT> dc;
	JT y;
	for (int k = 0; k < 2000; ++k) y = dc(x, 35.0, T);
	check(near(y[0], 1.5, 1e-9), "notch passes a constant");
	double gain, lag;
	notchResponse(35.0, &gain, &lag);
	check(gain < 0.01, "notch at 35 Hz: rejects 35 Hz");
	notchResponse(31.0, &gain, &lag);
	check(gain < 0.25, "notch at 35 Hz: passes under a quarter of 31 Hz");
	notchResponse(43.0, &gain, &lag);
	check(gain < 0.35, "notch at 35 Hz: passes under 0.35 of 43 Hz");
	notchResponse(4.0, &gain, &lag);
	check(gain > 0.98 && lag > 5.0 && lag < 12.0, "notch at 35 Hz: 4 Hz passes, lagging about 9 degrees");
	printf("notch at 35 Hz, 4 Hz: gain %.3f, lag %.1f deg\n", gain, lag);
}

/* J6's friction feedforward: with the direction of turning, fading out at rest. */
void testFrictionFeedforward() {
	check(teleop_law::frictionFeedforward(0.3, 0.0) == 0.0, "friction feedforward: nothing at rest");
	check(near(teleop_law::frictionFeedforward(0.3, 1.0), 0.3, 1e-6), "friction feedforward: full turning +");
	check(near(teleop_law::frictionFeedforward(0.3, -1.0), -0.3, 1e-6), "friction feedforward: full turning -");
	check(near(teleop_law::frictionFeedforward(0.3, teleop_law::FRICTION_V0), 0.3 * std::tanh(1.0), 1e-12),
			"friction feedforward: 76% at FRICTION_V0");
	check(teleop_law::frictionFeedforward(0.0, 1.0) == 0.0, "friction feedforward: off at 0 Nm");
}

int main() {
	testWristMap();
	testLaws();
	testForceChannel();
	testReferenceTracker();
	testNotch();
	testFrictionFeedforward();
	printf("%d checks, %d failed\n", checks, failures);
	return failures == 0 ? 0 : 1;
}
