/* The controller's cable-drive model (lib/cable_drive_model.h) against the
 * identification (ident/: python -m cable_ident), sample by sample on 12 s of
 * each arm's excitation log, plus the friction elements' properties:
 *   - the same torque as the Python model (test/data/cable_model_*.txt);
 *   - a joint toggling one encoder count at rest moves the friction by a
 *     fraction of an element's force, never by a sign flip;
 *   - over a closed cycle of motion the elements dissipate (passivity).
 * After a new fit: python -m cable_ident testvectors (ident/) rewrites test/data.
 * Build: catkin target cable_model_tests, or
 *   g++ -std=c++11 -I/usr/include/eigen3 -Isrc/lib test/test_cable_drive_model.cpp */

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "cable_drive_model.h"
#include "cable_params_slax_20261112.h"
#include "cable_params_wam8g.h"
#include "cable_params_zeus_hand.h"

#ifndef TEST_DATA_DIR
#define TEST_DATA_DIR "test/data"
#endif

static int failures = 0;
#define CHECK(cond, ...)                         \
	do {                                         \
		if (!(cond)) {                           \
			++failures;                          \
			printf("FAIL %s:%d: ", __FILE__, __LINE__); \
			printf(__VA_ARGS__);                 \
			printf("\n");                        \
		}                                        \
	} while (0)

static bool agreesWithPython(const char* name, const cable_model::Params& p) {
	const std::string path = std::string(TEST_DATA_DIR) + "/cable_model_" + name + ".txt";
	std::ifstream in(path.c_str());
	if (!in) {
		printf("SKIP %s: no %s (python -m cable_ident writes it)\n", name, path.c_str());
		return true;
	}
	cable_model::Model m(p);
	std::string line;
	double worst = 0.0;
	int rows = 0;
	while (std::getline(in, line)) {
		if (line.empty() || line[0] == '#') {
			continue;
		}
		std::istringstream s(line);
		double v[16];
		for (int i = 0; i < 16; ++i) {
			s >> v[i];
		}
		cable_model::Vec4 q, dq, ddq;
		for (int i = 0; i < 4; ++i) {
			q[i] = v[i];
			dq[i] = v[4 + i];
			ddq[i] = v[8 + i];
		}
		m.update(q, dq, ddq);
		const cable_model::Vec4 tau = m.full();
		for (int i = 0; i < 4; ++i) {
			worst = std::max(worst, std::fabs(tau[i] - v[12 + i]));
		}
		++rows;
	}
	printf("%s: %d samples, largest difference from the identification %.2e Nm\n", name, rows, worst);
	CHECK(rows > 1000, "%s: only %d samples", name, rows);
	CHECK(worst < 1e-6, "%s: the controller's model differs from the identification by %g Nm", name, worst);
	return worst < 1e-6;
}

// A joint dithering one encoder count at rest (2 pi / 4096 at the motor).
static void countToggle(const char* name, const cable_model::Params& p) {
	cable_model::Model m(p);
	cable_model::Vec4 q(1.57, -1.57, -1.57, 1.57), z = cable_model::Vec4::Zero();
	// approach from below so the elements are loaded in +
	for (int k = 0; k <= 200; ++k) {
		cable_model::Vec4 qk = q;
		qk[0] = q[0] - 0.01 + 0.01 * k / 200.0;
		m.update(qk, z, z);
	}
	const double loaded = m.hysteresis()[0];
	const double count = 2.0 * M_PI / 4096.0 / 42.0;  // J1, joint rad
	double lo = loaded, hi = loaded;
	for (int k = 0; k < 100; ++k) {
		cable_model::Vec4 qk = q;
		qk[0] += (k % 2) ? -count : 0.0;
		m.update(qk, z, z);
		lo = std::min(lo, m.hysteresis()[0]);
		hi = std::max(hi, m.hysteresis()[0]);
	}
	double total = 0.0;
	for (int i = 0; i < cable_model::KM; ++i) {
		total += std::fabs(42.0 * p.motorW[0][i]);
	}
	printf("%s: J1 friction %.2f Nm after a move; a one-count toggle moves it by %.2f Nm (of %.2f)\n", name,
	       loaded, hi - lo, 2 * total);
	CHECK(hi - lo < 0.5 * total, "%s: a one-count toggle swings the friction by %g of %g Nm", name, hi - lo, 2 * total);
}

// Passivity: over closed cycles the friction elements take energy out.
static void dissipates(const char* name, const cable_model::Params& p) {
	cable_model::Model m(p);
	cable_model::Vec4 q0(1.57, -1.57, -1.57, 1.57), z = cable_model::Vec4::Zero();
	double work = 0.0;
	cable_model::Vec4 qPrev = q0;
	m.update(q0, z, z);
	for (int k = 1; k <= 20000; ++k) {
		const double t = k * 0.002;
		cable_model::Vec4 q = q0;
		q[0] += 0.004 * std::sin(2.1 * t) + 0.001 * std::sin(13.0 * t);
		q[1] += 0.003 * std::sin(1.3 * t + 0.4);
		q[2] += 0.005 * std::sin(1.7 * t + 1.0) + 0.0005 * std::sin(9.0 * t);
		q[3] += 0.002 * std::sin(3.1 * t);
		m.update(q, z, z);
		const cable_model::Vec4 f = m.hysteresis();
		work += f.dot(q - qPrev);  // energy taken from the motion by the friction torque
		qPrev = q;
	}
	printf("%s: friction elements over 40 s of mixed cycles: %.3f J taken from the motion\n", name, work);
	CHECK(work > 0.0, "%s: the friction elements returned %g J", name, -work);
}

int main() {
	const cable_model::Params pl = cable_params_wam8g::params();
	const cable_model::Params pf = cable_params_zeus_hand::params();
	const cable_model::Params ps = cable_params_slax_20261112::params();  // what follower --model cable loads
	agreesWithPython("leader", pl);
	agreesWithPython("follower", pf);
	agreesWithPython("slax", ps);
	countToggle("leader", pl);
	countToggle("follower", pf);
	countToggle("slax", ps);
	dissipates("leader", pl);
	dissipates("follower", pf);
	dissipates("slax", ps);
	printf(failures ? "%d FAILED\n" : "all passed\n", failures);
	return failures ? 1 : 0;
}
