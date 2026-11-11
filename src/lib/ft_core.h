#pragma once

/* The follower's force/torque sensor in the teleop: the arithmetic, without
 * libbarrett (tested offline: test/test_low_latency.cpp).
 *
 * Why. The wrist is coupled position-position: the leader's wrist feels its
 * position controller pulling it toward the follower's wrist, and the
 * follower's wrist lags by its own friction and inertia, so the operator's
 * hand drags the follower's wrist around (a median 0.1-0.25 Nm against the
 * hand while turning in the IROS recordings, 0.4-0.6 Nm a tenth of the time).
 * A contact is felt only through the same position error.
 *
 * With the sensor on the follower's tool plate the contact is measured, so
 * the leader no longer needs the position error to feel it:
 *
 *   follower   reading - the weight of what hangs beyond the sensor
 *              -> joint torques (Jacobian transpose), sent to the leader as
 *                 the external torque: J1-J4 in place of the model's estimate
 *                 (follower --ft arm), the wrist in the three slots that were
 *                 zero (--ft wrist);
 *   leader     applies the wrist's (--wrist-force) and ignores small wrist
 *              position errors (--wrist-deadband): the position coupling
 *              becomes a tether that only pulls when the follower's wrist is
 *              really left behind.
 *
 * Frames. T: libbarrett's tool frame (the tool plate; follower's
 * kinematics). S: the sensor's own axes; v_T = rot * v_S. The sensor's
 * measuring origin is at `origin` in T. gravityDirection is the unit vector
 * of gravity in T: R_world_tool^T (0, 0, -1).
 *
 * Zero. The library tares the sensor when the program opens it, in whatever
 * pose the arm is then, and strain gauges drift; so the readings are taken
 * relative to an average at a known moment with nothing touching the gripper
 * (Zero: at the sync pose, before linking). The gripper's weight to subtract
 * is then its weight now minus its weight at that moment, which needs only its
 * mass and centre of mass: scripts/ft_tool_cal.py fits them, the sensor's
 * axes and its sign convention from a recording. */

#include <cmath>
#include <fstream>
#include <sstream>
#include <string>

namespace ft_core {

const double GRAVITY = 9.81;  // m/s^2

inline void cross(const double* a, const double* b, double* out) {
	out[0] = a[1] * b[2] - a[2] * b[1];
	out[1] = a[2] * b[0] - a[0] * b[2];
	out[2] = a[0] * b[1] - a[1] * b[0];
}
/* out = M v, M row-major 3x3. */
inline void rotate(const double* m, const double* v, double* out) {
	for (int i = 0; i < 3; ++i) {
		out[i] = m[3 * i] * v[0] + m[3 * i + 1] * v[1] + m[3 * i + 2] * v[2];
	}
}
/* out = M^T v. */
inline void rotateBack(const double* m, const double* v, double* out) {
	for (int i = 0; i < 3; ++i) {
		out[i] = m[i] * v[0] + m[3 + i] * v[1] + m[6 + i] * v[2];
	}
}
inline double norm(const double* v) {
	return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}
inline double clamp(double x, double limit) {
	return x > limit ? limit : (x < -limit ? -limit : x);
}

/* Gravity's direction in the tool frame from the tool's rotation
 * (v_world = R v_tool, row-major): R^T (0, 0, -1). */
inline void gravityInTool(const double* worldFromTool, double* g) {
	g[0] = -worldFromTool[6];
	g[1] = -worldFromTool[7];
	g[2] = -worldFromTool[8];
}

/* What hangs beyond the sensor (the gripper), and how the sensor reports. */
struct Tool {
	double mass;        // kg
	double com[3];      // m, centre of mass from the sensor's measuring origin, in T
	double sign;        // +1: the reading is the wrench the gripper side applies to the arm; -1: the opposite
	double rot[9];      // v_T = rot v_S, row-major
	double origin[3];   // m, the sensor's measuring origin in T
	bool calibrated;    // from a file (ft_tool_cal.py), not these defaults
	Tool() : mass(0.0), sign(1.0), calibrated(false) {
		for (int i = 0; i < 3; ++i) {
			com[i] = origin[i] = 0.0;
		}
		for (int i = 0; i < 9; ++i) {
			rot[i] = (i % 4 == 0) ? 1.0 : 0.0;
		}
	}

	/* A file of "key = value" lines ('#' starts a comment): mass, com_x,
	 * com_y, com_z, sign, rot_xx ... rot_zz, origin_x, origin_y, origin_z. */
	bool load(const std::string& path, std::string* error) {
		std::ifstream in(path.c_str());
		if (!in) {
			*error = "cannot open " + path;
			return false;
		}
		static const char* const axes = "xyz";
		bool haveMass = false;
		std::string line;
		while (std::getline(in, line)) {
			const size_t hash = line.find('#');
			if (hash != std::string::npos) line.erase(hash);
			const size_t eq = line.find('=');
			if (eq == std::string::npos) {
				if (line.find_first_not_of(" \t\r") != std::string::npos) {
					*error = path + ": cannot read \"" + line + "\"";
					return false;
				}
				continue;
			}
			std::istringstream k(line.substr(0, eq)), v(line.substr(eq + 1));
			std::string key, rest;
			double value;
			if (!(k >> key) || !(v >> value) || (v >> rest) || !std::isfinite(value)) {
				*error = path + ": cannot read \"" + line + "\"";
				return false;
			}
			bool known = true;
			if (key == "mass") {
				mass = value;
				haveMass = true;
			} else if (key == "sign") {
				sign = value;
			} else if (key.size() == 5 && key.compare(0, 4, "com_") == 0 && std::string(axes).find(key[4]) != std::string::npos) {
				com[std::string(axes).find(key[4])] = value;
			} else if (key.size() == 8 && key.compare(0, 7, "origin_") == 0 && std::string(axes).find(key[7]) != std::string::npos) {
				origin[std::string(axes).find(key[7])] = value;
			} else if (key.size() == 6 && key.compare(0, 4, "rot_") == 0 && std::string(axes).find(key[4]) != std::string::npos &&
			           std::string(axes).find(key[5]) != std::string::npos) {
				rot[3 * std::string(axes).find(key[4]) + std::string(axes).find(key[5])] = value;
			} else {
				known = false;
			}
			if (!known) {
				*error = path + ": unknown key \"" + key + "\"";
				return false;
			}
		}
		if (!haveMass || !(mass >= 0.0 && mass < 20.0) || (sign != 1.0 && sign != -1.0)) {
			*error = path + ": needs mass (0 to 20 kg) and sign +1 or -1";
			return false;
		}
		for (int i = 0; i < 3; ++i) {
			if (std::fabs(com[i]) > 0.5 || std::fabs(origin[i]) > 0.5) {
				*error = path + ": a centre of mass or origin beyond 0.5 m";
				return false;
			}
		}
		if (!properRotation()) {
			*error = path + ": rot_* is not a rotation (orthonormal, determinant +1)";
			return false;
		}
		calibrated = true;
		return true;
	}

	bool properRotation() const {
		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {
				double dot = 0.0;
				for (int k = 0; k < 3; ++k) dot += rot[3 * i + k] * rot[3 * j + k];
				if (std::fabs(dot - (i == j ? 1.0 : 0.0)) > 1e-3) return false;
			}
		}
		const double det = rot[0] * (rot[4] * rot[8] - rot[5] * rot[7]) - rot[1] * (rot[3] * rot[8] - rot[5] * rot[6]) +
		                   rot[2] * (rot[3] * rot[7] - rot[4] * rot[6]);
		return det > 0.0;
	}

	/* The gripper's weight on the arm, in T about the sensor's origin. */
	void weight(const double* gravityDirection, double* force, double* torque) const {
		for (int i = 0; i < 3; ++i) force[i] = mass * GRAVITY * gravityDirection[i];
		cross(com, force, torque);
	}
};

/* The wrench the surroundings apply to the arm through the gripper, in T
 * about T's origin:
 *   sign rot (reading - reading at the zero) - (gripper weight now - at the zero),
 * the torque then moved from the sensor's origin to T's. */
inline void environmentWrench(const Tool& tool, const double* rawForce, const double* rawTorque,
                              const double* zeroForce, const double* zeroTorque, const double* gravityNow,
                              const double* gravityAtZero, double* force, double* torque) {
	double df[3], dn[3], f[3], n[3], fNow[3], nNow[3], fZero[3], nZero[3], shift[3];
	for (int i = 0; i < 3; ++i) {
		df[i] = rawForce[i] - zeroForce[i];
		dn[i] = rawTorque[i] - zeroTorque[i];
	}
	rotate(tool.rot, df, f);
	rotate(tool.rot, dn, n);
	tool.weight(gravityNow, fNow, nNow);
	tool.weight(gravityAtZero, fZero, nZero);
	for (int i = 0; i < 3; ++i) {
		force[i] = tool.sign * f[i] - (fNow[i] - fZero[i]);
		torque[i] = tool.sign * n[i] - (nNow[i] - nZero[i]);
	}
	cross(tool.origin, force, shift);
	for (int i = 0; i < 3; ++i) torque[i] += shift[i];
}

/* The sensor's zero: the average reading and gravity direction over a
 * stretch with nothing touching the gripper. */
struct Zero {
	double force[3], torque[3], gravity[3];
	bool valid;
	int count;       // zeros completed
	Zero() : valid(false), count(0), left(0), total(0) {
		for (int i = 0; i < 3; ++i) force[i] = torque[i] = gravity[i] = sumF[i] = sumN[i] = sumG[i] = 0.0;
	}
	void begin(int samples) {
		left = total = samples > 0 ? samples : 1;
		for (int i = 0; i < 3; ++i) sumF[i] = sumN[i] = sumG[i] = 0.0;
	}
	bool running() const { return left > 0; }
	/* One reading while running(); true when this one completed the zero. */
	bool add(const double* rawForce, const double* rawTorque, const double* gravityDirection) {
		if (left <= 0) return false;
		for (int i = 0; i < 3; ++i) {
			sumF[i] += rawForce[i];
			sumN[i] += rawTorque[i];
			sumG[i] += gravityDirection[i];
		}
		if (--left > 0) return false;
		const double g = norm(sumG);
		for (int i = 0; i < 3; ++i) {
			force[i] = sumF[i] / total;
			torque[i] = sumN[i] / total;
			gravity[i] = g > 0.0 ? sumG[i] / g : 0.0;
		}
		valid = g > 0.0;
		++count;
		return true;
	}

private:
	double sumF[3], sumN[3], sumG[3];
	int left, total;
};

/* Whether the readings can be used: the sensor must have answered recently,
 * and after a gap it must answer for a while before it is trusted again.
 * use() is 0..1 and moves gradually, so a sensor that drops out, or comes
 * back, never steps a torque. */
struct Health {
	int missed;       // cycles since the last reading
	double trust;     // 0..1
	Health() : missed(1000000), trust(0.0) {}
	static int maxMissed() { return 10; }           // 20 ms without a reading: stale
	static double riseTime() { return 0.5; }        // s of readings to full use
	static double fallTime() { return 0.1; }        // s to no use once stale
	void step(bool gotReading, bool usable, double period) {
		missed = gotReading ? 0 : (missed < 1000000 ? missed + 1 : missed);
		if (usable && missed <= maxMissed()) {
			trust += period / riseTime();
			if (trust > 1.0) trust = 1.0;
		} else {
			trust -= period / fallTime();
			if (trust < 0.0) trust = 0.0;
		}
	}
	bool stale() const { return missed > maxMissed(); }
	double use() const { return trust; }
};

/* The limits on what the sensor may ask of the arms: a reading beyond them
 * is a fault or a crash, not something to pass to the operator's hand. */
struct Limits {
	double force, torque;          // N, Nm: the wrench's magnitude
	double forceRate, torqueRate;  // N/s, Nm/s: how fast it may change
	double filterHz;               // first-order low-pass, 0: none
	Limits() : force(60.0), torque(6.0), forceRate(2000.0), torqueRate(200.0), filterHz(40.0) {}
};

/* Reading by reading: zero, the gripper's weight, the limits, the low-pass.
 * force/torque: the wrench the surroundings apply to the arm, in T about T's
 * origin, to be used with weight use() (0 until a zero was taken and the
 * sensor has answered for a while). */
struct Processor {
	Tool tool;
	Limits limits;
	Zero zero;
	Health health;
	double rawForce[3], rawTorque[3];   // the last reading (S)
	double gravity[3];                  // gravity's direction in T, this cycle
	double force[3], torque[3];         // the result
	bool fresh;                         // a reading arrived this cycle

	Processor() : fresh(false), haveReading(false), started(false) {
		for (int i = 0; i < 3; ++i) {
			rawForce[i] = rawTorque[i] = gravity[i] = force[i] = torque[i] = limited[i] = limited[3 + i] = 0.0;
		}
	}

	/* The average of the next `samples` readings becomes the zero. */
	void startZero(int samples) { zero.begin(samples); }

	/* Once per control cycle. gotReading: reading[F, N] is new. */
	void step(bool gotReading, const double* readingForce, const double* readingTorque, const double* worldFromTool,
	          double period) {
		fresh = gotReading;
		gravityInTool(worldFromTool, gravity);
		if (gotReading) {
			for (int i = 0; i < 3; ++i) {
				rawForce[i] = readingForce[i];
				rawTorque[i] = readingTorque[i];
			}
			haveReading = true;
			if (zero.running() && zero.add(rawForce, rawTorque, gravity)) {
				started = false;  // the result restarts from the new zero, through use()
				health.trust = 0.0;
			}
		}
		health.step(gotReading, zero.valid && !zero.running(), period);
		if (!haveReading || !zero.valid) {
			for (int i = 0; i < 3; ++i) force[i] = torque[i] = 0.0;
			return;
		}
		double w[6];
		environmentWrench(tool, rawForce, rawTorque, zero.force, zero.torque, gravity, zero.gravity, w, w + 3);
		limitNorm(w, limits.force);
		limitNorm(w + 3, limits.torque);
		if (!started) {
			for (int i = 0; i < 6; ++i) limited[i] = w[i];
			for (int i = 0; i < 3; ++i) {
				force[i] = w[i];
				torque[i] = w[3 + i];
			}
			started = true;
			return;
		}
		const double k = limits.filterHz > 0.0 ? 1.0 - std::exp(-2.0 * M_PI * limits.filterHz * period) : 1.0;
		for (int i = 0; i < 6; ++i) {
			const double step = (i < 3 ? limits.forceRate : limits.torqueRate) * period;
			limited[i] += clamp(w[i] - limited[i], step);
		}
		for (int i = 0; i < 3; ++i) {
			force[i] += k * (limited[i] - force[i]);
			torque[i] += k * (limited[3 + i] - torque[i]);
		}
	}

	double use() const { return health.use(); }

private:
	static void limitNorm(double* v, double limit) {
		const double n = norm(v);
		if (n > limit && n > 0.0) {
			for (int i = 0; i < 3; ++i) v[i] *= limit / n;
		}
	}
	bool haveReading, started;
	double limited[6];
};

/* What the follower's loop and its recording take of the sensor each cycle
 * (written by FtTorques::operate(), lib/ft_systems.h; read by
 * Follower::operate() after it has pulled FtTorques' output: the same thread,
 * the same cycle). */
struct Report {
	double rawForce[3], rawTorque[3];  // the last reading, sensor axes
	double gravity[3];                 // gravity's direction in the tool frame
	double force[3], torque[3];        // from the surroundings, tool frame, at its origin
	double jointTorque[7];             // the same as joint torques, Nm (not yet times use)
	double use;                        // 0..1 (Health)
	double fresh;                      // 1: a reading arrived this cycle
	double zeros;                      // zeros taken so far
	double pollSeconds;                // time the cycle spent reading the sensor
	Report() : use(0.0), fresh(0.0), zeros(0.0), pollSeconds(0.0) {
		for (int i = 0; i < 3; ++i) rawForce[i] = rawTorque[i] = gravity[i] = force[i] = torque[i] = 0.0;
		for (int i = 0; i < 7; ++i) jointTorque[i] = 0.0;
	}
};

/* The wrist's position coupling as a tether: no pull while the follower's
 * wrist is within `width` of the leader's, the usual spring beyond it. The
 * same shape keeps a small measured torque (the gripper's own inertia as the
 * wrist turns) out of the wrist's force channel. */
inline double softDeadband(double x, double width) {
	if (width <= 0.0) return x;
	if (x > width) return x - width;
	if (x < -width) return x + width;
	return 0.0;
}

/* x moved toward target by at most step. */
inline double toward(double x, double target, double step) {
	return x < target ? (target - x < step ? target : x + step) : (x - target < step ? target : x - step);
}

}  // namespace ft_core
