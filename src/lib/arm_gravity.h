#pragma once

/* libbarrett's calibrated gravity model (cdlbt/calgrav.c: T_j = g_j x mu_j,
 * accumulated toward the base; standard DH with theta = q + theta_pi * pi),
 * evaluated for any joint vector. The cable-drive model of J1-J4 is
 * identified with the wrist at its reference angles (0); the wrist's pose
 * shifts the gravity torque on J1-J4 (the follower with its hand: up to about
 * 1.2 Nm on J2/J3 in the teleop logs), so the estimate adds
 *     g(q) - g(q with the wrist at its reference)
 * from this calibrated model. No libbarrett types: the arm's DH and mus are
 * read once from its configuration group (wam7w, wam8g, ...). */

#include <cmath>
#include <cstdio>
#include <string>

#include <Eigen/Core>
#include <libconfig.h++>

namespace arm_gravity {

const int MAX_DOF = 8;
const double G_WORLD_Z = -9.805;  // calgrav.c

class Model {
public:
	Model() : dof(0) {}

	/* From the arm's configuration group. False (and a message) if it has no
	 * kinematics.moving or gravity_compensation.mus of the same length. */
	bool load(const libconfig::Setting& arm, const char* who) {
		try {
			const libconfig::Setting& moving = arm["kinematics"]["moving"];
			const libconfig::Setting& mus = arm["gravity_compensation"]["mus"];
			dof = moving.getLength();
			if (dof > MAX_DOF || mus.getLength() != dof) {
				printf("%s: kinematics.moving (%d) and gravity_compensation.mus (%d) do not match.\n", who, dof,
				       mus.getLength());
				dof = 0;
				return false;
			}
			for (int j = 0; j < dof; ++j) {
				const libconfig::Setting& l = moving[j];
				alpha[j] = number(l["alpha_pi"]) * M_PI;
				thetaOffset[j] = l.exists("theta_pi") ? number(l["theta_pi"]) * M_PI : 0.0;
				for (int k = 0; k < 3; ++k) {
					mu[j][k] = number(mus[j][k]);
				}
			}
		} catch (const libconfig::SettingException& e) {
			printf("%s: gravity model: %s\n", who, e.what());
			dof = 0;
			return false;
		}
		return true;
	}

	int getDof() const { return dof; }

	// Gravity torque of every joint at q (q has at least getDof() entries).
	template <typename V>
	void torque(const V& q, double* tau) const {
		Eigen::Matrix3d Rp[MAX_DOF], Rw[MAX_DOF];
		Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
		for (int j = 0; j < dof; ++j) {
			const double th = q[j] + thetaOffset[j];
			const double ct = std::cos(th), st = std::sin(th);
			const double ca = std::cos(alpha[j]), sa = std::sin(alpha[j]);
			Rp[j] << ct, -st * ca, st * sa,
			         st, ct * ca, -ct * sa,
			         0.0, sa, ca;
			R = R * Rp[j];
			Rw[j] = R;
		}
		const Eigen::Vector3d gWorld(0.0, 0.0, G_WORLD_Z);
		Eigen::Vector3d tNext = Eigen::Vector3d::Zero();
		for (int j = dof - 1; j >= 0; --j) {
			const Eigen::Vector3d g = Rw[j].transpose() * gWorld;
			Eigen::Vector3d t = g.cross(Eigen::Vector3d(mu[j][0], mu[j][1], mu[j][2]));
			if (j < dof - 1) {
				t += Rp[j + 1] * tNext;
			}
			tau[j] = (Rp[j] * t)[2];
			tNext = t;
		}
	}

	/* g(q) - g(q with joints first.. at ref) on J1-J4: the wrist's gravity
	 * relative to the pose the J1-J4 model was identified at. */
	template <typename V>
	Eigen::Matrix<double, 4, 1> wristCorrection(const V& q, int first, const double* ref) const {
		double qa[MAX_DOF], qr[MAX_DOF], ta[MAX_DOF], tr[MAX_DOF];
		for (int j = 0; j < dof; ++j) {
			qa[j] = q[j];
			qr[j] = j >= first ? ref[j - first] : q[j];
		}
		torque(qa, ta);
		torque(qr, tr);
		Eigen::Matrix<double, 4, 1> dg;
		for (int j = 0; j < 4; ++j) {
			dg[j] = ta[j] - tr[j];
		}
		return dg;
	}

private:
	static double number(const libconfig::Setting& s) {
		switch (s.getType()) {
		case libconfig::Setting::TypeInt: return (int) s;
		case libconfig::Setting::TypeInt64: return (double) (long long) s;
		default: return s;
		}
	}

	int dof;
	double alpha[MAX_DOF], thetaOffset[MAX_DOF];  // calgrav needs only the rotations
	double mu[MAX_DOF][3];
};

}  // namespace arm_gravity
