#pragma once

/* The external-torque estimate's arithmetic, without libbarrett (tested
 * offline: test/test_low_latency.cpp).
 *
 * The estimate is the commanded torque minus the model's torque. The model's
 * inertial term needs an acceleration, which only exists low-pass filtered
 * (the velocity high-passed at hp, low-passed at lp: 25 and 50 rad/s in
 * leader_wam8g.cpp and follower.cpp). The estimate so far ("legacy")
 * subtracted that slow inertial torque from the unfiltered commanded torque,
 * so the two only cancel for slow motion: at 1 Hz 41 % of the arm's own
 * inertial torque stays in the estimate, at 2 Hz 74 %, above 3 Hz all of it.
 * Above a few hertz the "external torque" each arm sent the other was mostly
 * its own position-controller torque.
 *
 * The "matched" estimate passes the commanded torque and the rest of the
 * model through the same two low-passes as the acceleration:
 *
 *     ext = LP_hp(LP_lp(tau - static)) - M(q) a,     a = LP_lp(d/dt LP_hp(v))
 *
 * For a constant inertia the two sides then cancel exactly at every
 * frequency (a = LP_hp LP_lp of the true acceleration), and what is left is
 * the low-passed external torque: a momentum observer with two real poles.
 * The filters are libbarrett's FirstOrderFilter (backward Euler), so the
 * acceleration is the same number the existing chain produces at the same
 * corner frequencies. */

#include <cmath>

#include "regressor_W.h"

namespace estimator {

/* libbarrett's FirstOrderFilter as a low-pass, on four joints:
 * y = (y1 + w T x) / (1 + w T). */
struct LowPass4 {
	double y[4];
	double gain;
	LowPass4() : gain(1.0) { reset(0.0); }
	void set(double omega, double period) { gain = omega * period / (1.0 + omega * period); }
	void reset(double v) {
		for (int i = 0; i < 4; ++i) y[i] = v;
	}
	void reset(const double* v) {
		for (int i = 0; i < 4; ++i) y[i] = v[i];
	}
	const double* step(const double* x) {
		for (int i = 0; i < 4; ++i) y[i] += gain * (x[i] - y[i]);
		return y;
	}
};

/* a = LP_lp(HP_hp(v)), HP_hp = hp s / (s + hp): what hp1 and jaFilter compute
 * in leader_wam8g.cpp / follower.cpp. Backward Euler makes the high-pass the
 * backward difference of the low-passed velocity. */
struct Acceleration {
	LowPass4 velocity, accel;
	double previous[4];
	double period;
	bool started;
	Acceleration() : period(0.002), started(false) {}
	void configure(double hpOmega, double lpOmega, double T) {
		period = T;
		velocity.set(hpOmega, T);
		accel.set(lpOmega, T);
		started = false;
	}
	const double* step(const double* v) {
		if (!started) {  // at rest in the filter's view: no start-up spike
			velocity.reset(v);
			accel.reset(0.0);
			for (int i = 0; i < 4; ++i) previous[i] = v[i];
			started = true;
		}
		const double* l = velocity.step(v);
		double d[4];
		for (int i = 0; i < 4; ++i) {
			d[i] = (l[i] - previous[i]) / period;
			previous[i] = l[i];
		}
		return accel.step(d);
	}
};

/* ext = LP_hp(LP_lp(tau - static)) - inertia, J1-J4. inertia: M(q) times the
 * Acceleration above at the same corner frequencies. */
struct MatchedTorque {
	LowPass4 first, second;
	bool started;
	MatchedTorque() : started(false) {}
	void configure(double hpOmega, double lpOmega, double T) {
		first.set(hpOmega, T);
		second.set(lpOmega, T);
		started = false;
	}
	void step(const double* tau, const double* staticModel, const double* inertia, double* ext) {
		double x[4];
		for (int i = 0; i < 4; ++i) x[i] = tau[i] - staticModel[i];
		if (!started) {
			first.reset(x);
			second.reset(x);
			started = true;
		}
		const double* y = second.step(first.step(x));
		for (int i = 0; i < 4; ++i) ext[i] = y[i] - inertia[i];
	}
};

/* The fitted rigid-body model (regressor_W.h with a 30-value beta) on fixed
 * arrays: nothing is allocated in the control loop (LeaderDynamics and
 * FollowerDynamics built an Eigen::MatrixXd every cycle). */
struct RegressorModel {
	double beta[30];
	double W0[120], W1[120];
	double staticTorque[4];  // W(q, dq, 0) beta: gravity, Coriolis, friction

	template <typename B>
	void setBeta(const B& b) {
		for (int i = 0; i < 30; ++i) beta[i] = b[i];
	}
	/* Once per cycle, before inertia(). */
	void update(const double* q, const double* dq) {
		const double zero[4] = {0.0, 0.0, 0.0, 0.0};
		calculate_W(q, dq, zero, W0);
		for (int r = 0; r < 4; ++r) {
			double s = 0.0;
			for (int c = 0; c < 30; ++c) s += W0[r * 30 + c] * beta[c];
			staticTorque[r] = s;
		}
	}
	/* M(q) ddq: the regressor is linear in the acceleration. */
	void inertia(const double* q, const double* dq, const double* ddq, double* out) {
		calculate_W(q, dq, ddq, W1);
		for (int r = 0; r < 4; ++r) {
			double s = 0.0;
			for (int c = 0; c < 30; ++c) s += (W1[r * 30 + c] - W0[r * 30 + c]) * beta[c];
			out[r] = s;
		}
	}
};

/* Checks the corner frequencies of --estimator-filter (rad/s). */
inline bool validCorners(double hpOmega, double lpOmega) {
	return hpOmega >= 5.0 && hpOmega <= 400.0 && lpOmega >= 5.0 && lpOmega <= 400.0;
}

}  // namespace estimator
