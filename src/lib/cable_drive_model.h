#pragma once

/* The WAM's J1-J4 inverse dynamics with cable-drive friction, for the
 * external-torque estimate of the sensorless teleop (no libbarrett; Eigen and
 * regressor_W.h only, so it is tested offline against the identification).
 *
 *   tau = W(q, dq, ddq) beta                   rigid body (physically consistent)
 *       + TM^T diag(I) TM ddq                  motor rotors
 *       + TM^T f(x)  +  f_J2,J3(q)             cable-drive friction (below)
 *       + TM^T diag(v) TM dq                   viscous, at the motors
 *       + offset
 *
 * x = TM q is the motor angle (TM: j2mp's J1-J4 block; motors 2 and 3 drive
 * J2 and J3 through the cable differential). The WAM's cables have no
 * backlash, so a motor encoder sees the joint's micro-motion amplified 18-42
 * times, and the friction after a reversal rises with how far the motor has
 * turned, not how fast: a cable stretching against friction until it slips.
 * Each element is that: a spring of slip displacement delta in series with a
 * Coulomb slider of force w (a Jenkins / Maxwell-slip element),
 *
 *     z <- clamp(z + dx, -delta, delta),    f = sum_i w_i z_i / delta_i,
 *
 * driven by position increments, so it does not chatter with velocity noise
 * at rest the way w tanh(100 dq) did. With w >= 0 every element is passive.
 * The identification (ident/) fits beta, w, I, v and the offsets in one convex
 * program; its parameter headers are cable_params_*.h. */

#include <cmath>

#include <Eigen/Core>

#include "regressor_W.h"

namespace cable_model {

const int KM = 6;  // elements per motor
const int KJ = 6;  // elements on J2 and J3 (joint side of the differential)

struct Params {
	double beta[30];            // regressor_W.h base parameters; its fv/fc slots are 0
	double TM[4][4];            // motor angle = TM q (rad)
	double motorDelta[KM];      // slip displacements, motor rad
	double motorW[4][KM];       // slip forces, Nm at the motor
	double jointDelta[KJ];      // slip displacements, joint rad (J2, J3)
	double jointW[2][KJ];       // slip forces, Nm at J2 and J3
	double motorViscous[4];     // Nm s/rad at the motor
	double rotor[4];            // kg m^2 at the motor
	double offset[4];           // Nm at the joint
};

typedef Eigen::Matrix<double, 4, 1> Vec4;

class Model {
public:
	explicit Model(const Params& p) : p(p), started(false) {
		for (int r = 0; r < 4; ++r) {
			for (int c = 0; c < 4; ++c) {
				TM(r, c) = p.TM[r][c];
			}
		}
		for (int i = 0; i < 30; ++i) {
			beta[i] = p.beta[i];
		}
		zero();
	}

	// Zero the friction states (the next update() starts from q).
	void reset() {
		started = false;
	}

	/* One control cycle. q, dq: libbarrett's joint position and velocity;
	 * ddq: the controller's acceleration (the same filter as in the
	 * identification: dq high-passed at 25 rad/s, low-passed at 50 rad/s). */
	void update(const Vec4& q, const Vec4& dq, const Vec4& ddq) {
		if (!started) {
			zero();
			qPrev = q;
			started = true;
		}
		// friction elements, from the position increment
		const Vec4 dqStep = q - qPrev;
		qPrev = q;
		const Vec4 dx = TM * dqStep;
		Vec4 fm;
		for (int m = 0; m < 4; ++m) {
			double f = 0.0;
			for (int i = 0; i < KM; ++i) {
				zm[m][i] = clamp(zm[m][i] + dx[m], p.motorDelta[i]);
				f += p.motorW[m][i] * zm[m][i] / p.motorDelta[i];
			}
			fm[m] = f;
		}
		hyst = TM.transpose() * fm;
		for (int j = 0; j < 2; ++j) {
			double f = 0.0;
			for (int i = 0; i < KJ; ++i) {
				zj[j][i] = clamp(zj[j][i] + dqStep[1 + j], p.jointDelta[i]);
				f += p.jointW[j][i] * zj[j][i] / p.jointDelta[i];
			}
			hyst[1 + j] += f;
		}

		// rigid body: W is linear in ddq, so inertia = (W(q,dq,ddq) - W(q,dq,0)) beta
		double qa[4], dqa[4], ddqa[4], zero4[4] = {0.0, 0.0, 0.0, 0.0};
		for (int i = 0; i < 4; ++i) {
			qa[i] = q[i];
			dqa[i] = dq[i];
			ddqa[i] = ddq[i];
			qLast[i] = q[i];
			dqLast[i] = dq[i];
		}
		calculate_W(qa, dqa, zero4, W0);
		calculate_W(qa, dqa, ddqa, W1);
		for (int r = 0; r < 4; ++r) {
			double a = 0.0, b = 0.0;
			for (int c = 0; c < 30; ++c) {
				a += W0[r * 30 + c] * beta[c];
				b += W1[r * 30 + c] * beta[c];
			}
			rbNoAcc[r] = a;
			inert[r] = b - a;
		}
		Vec4 xdd = TM * ddq, xd = TM * dq, rot, visc;
		for (int m = 0; m < 4; ++m) {
			rot[m] = p.rotor[m] * xdd[m];
			visc[m] = p.motorViscous[m] * xd[m];
		}
		inert += TM.transpose() * rot;
		viscous = TM.transpose() * visc;
		for (int j = 0; j < 4; ++j) {
			off[j] = p.offset[j];
		}
	}

	// The components of the last update(), joint torques J1-J4 (Nm).
	const Vec4& gravityCoriolis() const { return rbNoAcc; }  // W(q, dq, 0) beta
	const Vec4& inertia() const { return inert; }             // links and rotors
	const Vec4& hysteresis() const { return hyst; }           // the friction elements
	const Vec4& viscousFriction() const { return viscous; }
	const Vec4& offset() const { return off; }
	Vec4 full() const { return rbNoAcc + inert + hyst + viscous + off; }
	// Everything but the inertial torque: what the matched estimate
	// (estimator_core.h) low-passes together with the commanded torque.
	Vec4 withoutInertia() const { return rbNoAcc + hyst + viscous + off; }

	/* The inertial torque (links and rotors) of another acceleration at the
	 * last update()'s q and dq: the regressor is linear in the acceleration. */
	Vec4 inertiaFor(const Vec4& ddq) {
		double ddqa[4];
		for (int i = 0; i < 4; ++i) {
			ddqa[i] = ddq[i];
		}
		calculate_W(qLast, dqLast, ddqa, W2);
		Vec4 out;
		for (int r = 0; r < 4; ++r) {
			double s = 0.0;
			for (int c = 0; c < 30; ++c) {
				s += (W2[r * 30 + c] - W0[r * 30 + c]) * beta[c];
			}
			out[r] = s;
		}
		const Vec4 xdd = TM * ddq;
		Vec4 rot;
		for (int m = 0; m < 4; ++m) {
			rot[m] = p.rotor[m] * xdd[m];
		}
		return out + TM.transpose() * rot;
	}

private:
	static double clamp(double z, double d) {
		return z > d ? d : (z < -d ? -d : z);
	}
	void zero() {
		for (int m = 0; m < 4; ++m) {
			for (int i = 0; i < KM; ++i) {
				zm[m][i] = 0.0;
			}
		}
		for (int j = 0; j < 2; ++j) {
			for (int i = 0; i < KJ; ++i) {
				zj[j][i] = 0.0;
			}
		}
		for (int i = 0; i < 4; ++i) {
			qLast[i] = dqLast[i] = 0.0;
		}
		for (int i = 0; i < 120; ++i) {
			W0[i] = 0.0;
		}
		rbNoAcc.setZero();
		inert.setZero();
		hyst.setZero();
		viscous.setZero();
		off.setZero();
	}

	const Params p;
	Eigen::Matrix<double, 4, 4> TM;
	double beta[30];
	double W0[120], W1[120], W2[120];
	double qLast[4], dqLast[4];
	double zm[4][KM], zj[2][KJ];
	Vec4 qPrev;
	bool started;
	Vec4 rbNoAcc, inert, hyst, viscous, off;
};

}  // namespace cable_model
