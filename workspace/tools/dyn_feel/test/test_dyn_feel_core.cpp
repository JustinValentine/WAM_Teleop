/* lib/dyn_feel_core.h without a robot:
 *   - a 30-number file (the Feas-Ident-WAM4 notebook's beta_star_correct.dat):
 *     the terms sum to regressor_W.h's W(q, dq, ddq) beta, and to the
 *     notebook's own regressor on rows of its saved data (test/data);
 *   - cable_params_wam8g.h read as text gives the numbers the compiler reads
 *     from it, and the terms sum to cable_model::Model's;
 *   - files that are not parameters are refused;
 *   - the mixer: starts at nothing, ramps, clamps, fades with speed and at the
 *     ends of the joints' ranges, trips, and gives nothing on a bad signal.
 * Build (no libbarrett):
 *   g++ -std=c++14 -I/usr/include/eigen3 -I<wam_teleop>/src/lib -Isrc/lib test/test_dyn_feel_core.cpp
 * Run: a.out <beta30.dat> <notebook rows.txt> <wam_teleop>/src/lib/cable_params_wam8g.h */

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "dyn_feel_core.h"
#include "cable_params_wam8g.h"

static int failures = 0, checks = 0;
#define CHECK(cond, ...)                                \
	do {                                                \
		++checks;                                       \
		if (!(cond)) {                                  \
			++failures;                                 \
			printf("FAIL %s:%d: ", __FILE__, __LINE__); \
			printf(__VA_ARGS__);                        \
			printf("\n");                               \
		}                                               \
	} while (0)

static std::string writeTemp(const char* name, const std::string& text) {
	const char* dir = getenv("TMPDIR");
	const std::string path = std::string(dir != NULL ? dir : "/tmp") + "/" + name;
	std::ofstream out(path.c_str());
	out << text;
	return path;
}

static double total(const dyn_feel::Terms& t, int j, bool inertia) {
	return t.gravity[j] + t.coriolis[j] + t.viscous[j] + t.dry[j] + t.offset[j] + (inertia ? t.inertia[j] : 0.0);
}

static void testBeta30(const std::string& betaFile, const std::string& rowsFile) {
	dyn_feel::Params p;
	std::string error;
	CHECK(dyn_feel::loadParams(betaFile, &p, &error), "%s", error.c_str());
	CHECK(p.kind == dyn_feel::Params::BETA30, "kind");
	std::vector<double> beta;
	{
		std::ifstream in(betaFile.c_str());
		double v;
		while (in >> v) beta.push_back(v);
	}
	CHECK(beta.size() == 30, "the test's own read of the file: %zu numbers", beta.size());
	if (beta.size() != 30) return;
	for (int j = 0; j < 4; ++j) {
		CHECK(p.jointViscous[j] == beta[dyn_feel::FV_SLOT[j]], "fv%d", j + 1);
		CHECK(p.jointCoulomb[j] == beta[dyn_feel::FC_SLOT[j]], "fc%d", j + 1);
		CHECK(p.cable.beta[dyn_feel::FV_SLOT[j]] == 0.0 && p.cable.beta[dyn_feel::FC_SLOT[j]] == 0.0, "slots");
	}

	// against regressor_W.h with the whole beta (its Coulomb column is tanh(100 dq): v0 = 0.01)
	{
		dyn_feel::Evaluator e(p, 0.01);
		double worst = 0.0, worstSplit = 0.0;
		unsigned s = 12345;
		for (int n = 0; n < 500; ++n) {
			double q[4], dq[4], ddq[4], W[120], W0[120], zero[4] = {0, 0, 0, 0};
			for (int j = 0; j < 4; ++j) {
				s = s * 1664525u + 1013904223u; q[j] = -2.0 + 4.0 * (s >> 8) / 16777216.0;
				s = s * 1664525u + 1013904223u; dq[j] = -1.0 + 2.0 * (s >> 8) / 16777216.0;
				s = s * 1664525u + 1013904223u; ddq[j] = -3.0 + 6.0 * (s >> 8) / 16777216.0;
			}
			e.update(q, dq, ddq);
			calculate_W(q, dq, ddq, W);
			calculate_W(q, zero, zero, W0);
			for (int j = 0; j < 4; ++j) {
				double ref = 0.0, grav = 0.0;
				for (int c = 0; c < 30; ++c) {
					ref += W[j * 30 + c] * beta[c];
					grav += W0[j * 30 + c] * beta[c];
				}
				worst = std::max(worst, std::fabs(total(e.terms(), j, true) - ref));
				worstSplit = std::max(worstSplit, std::fabs(e.terms().gravity[j] - grav));
			}
		}
		CHECK(worst < 1e-9, "terms against W beta: %.3g Nm", worst);
		CHECK(worstSplit < 1e-12, "gravity term: %.3g Nm", worstSplit);
		printf("30-number file: terms sum to regressor_W.h's W beta within %.1e Nm (500 random states)\n", worst);
	}

	// against the notebook's own regressor (sign(dq): a steep tanh on rows with every |dq| > 0.1)
	{
		std::ifstream in(rowsFile.c_str());
		CHECK(bool(in), "cannot read %s", rowsFile.c_str());
		dyn_feel::Evaluator e(p, 1e-4);
		std::string line;
		double worst = 0.0;
		int rows = 0;
		while (std::getline(in, line)) {
			if (line.empty() || line[0] == '#') continue;
			std::istringstream ss(line);
			double v[16];
			bool ok = true;
			for (int i = 0; i < 16; ++i) ok = ok && bool(ss >> v[i]);
			CHECK(ok, "a short row");
			if (!ok) break;
			e.update(v, v + 4, v + 8);
			for (int j = 0; j < 4; ++j) {
				worst = std::max(worst, std::fabs(total(e.terms(), j, true) - v[12 + j]));
			}
			++rows;
		}
		CHECK(rows == 200, "%d rows", rows);
		CHECK(worst < 1e-6, "against the notebook's regressor: %.3g Nm", worst);
		printf("30-number file: within %.1e Nm of the notebook's regressor on %d rows of its data\n", worst, rows);
	}
}

static void testCableHeader(const std::string& header) {
	dyn_feel::Params p;
	std::string error;
	CHECK(dyn_feel::loadParams(header, &p, &error), "%s", error.c_str());
	CHECK(p.kind == dyn_feel::Params::CABLE, "kind");
	const cable_model::Params ref = cable_params_wam8g::params();
	bool same = true;
	for (int i = 0; i < 30; ++i) same = same && p.cable.beta[i] == ref.beta[i];
	for (int r = 0; r < 4; ++r) {
		for (int k = 0; k < 4; ++k) same = same && p.cable.TM[r][k] == ref.TM[r][k];
		for (int i = 0; i < cable_model::KM; ++i) same = same && p.cable.motorW[r][i] == ref.motorW[r][i];
		same = same && p.cable.motorViscous[r] == ref.motorViscous[r] && p.cable.rotor[r] == ref.rotor[r] &&
		       p.cable.offset[r] == ref.offset[r];
	}
	for (int i = 0; i < cable_model::KM; ++i) same = same && p.cable.motorDelta[i] == ref.motorDelta[i];
	for (int i = 0; i < cable_model::KJ; ++i) {
		same = same && p.cable.jointDelta[i] == ref.jointDelta[i] && p.cable.jointW[0][i] == ref.jointW[0][i] &&
		       p.cable.jointW[1][i] == ref.jointW[1][i];
	}
	CHECK(same, "the header read as text differs from the header compiled");
	for (int j = 0; j < 4; ++j) CHECK(p.jointViscous[j] == 0.0 && p.jointCoulomb[j] == 0.0, "joint friction");

	// the terms against cable_model::Model on a motion with reversals
	dyn_feel::Evaluator e(p, 0.05);
	cable_model::Model m(ref);
	double worst = 0.0, worstInertia = 0.0;
	for (int n = 0; n < 3000; ++n) {
		const double t = 0.002 * n;
		double q[4], dq[4], ddq[4];
		cable_model::Vec4 qv, dqv, ddqv;
		for (int j = 0; j < 4; ++j) {
			const double w = 1.0 + 0.7 * j, a = 0.4 / (1 + j);
			q[j] = 0.3 * j - 0.5 + a * std::sin(w * t) + 0.002 * std::sin(40.0 * t + j);
			dq[j] = a * w * std::cos(w * t) + 0.08 * std::cos(40.0 * t + j);
			ddq[j] = -a * w * w * std::sin(w * t);
			qv[j] = q[j];
			dqv[j] = dq[j];
			ddqv[j] = ddq[j];
		}
		e.update(q, dq, ddq);
		m.update(qv, dqv, ddqv);
		const cable_model::Vec4 without = m.withoutInertia();
		for (int j = 0; j < 4; ++j) {
			worst = std::max(worst, std::fabs(total(e.terms(), j, false) - without[j]));
			worstInertia = std::max(worstInertia, std::fabs(e.terms().inertia[j] - m.inertia()[j]));
			CHECK(std::fabs(e.terms().dry[j] - m.hysteresis()[j]) < 1e-12, "dry friction is the elements'");
		}
	}
	CHECK(worst < 1e-9, "terms against cable_model::Model: %.3g Nm", worst);
	CHECK(worstInertia < 1e-12, "inertia: %.3g Nm", worstInertia);
	printf("cable header: read as text = compiled; terms within %.1e Nm of cable_model::Model (3000 cycles)\n", worst);
}

static void testRefusals(const std::string& betaFile) {
	dyn_feel::Params p;
	std::string error;
	CHECK(!dyn_feel::loadParams("/nonexistent/file", &p, &error), "a missing file");
	CHECK(!dyn_feel::loadParams(writeTemp("dyn_feel_29.dat", "1 2 3\n"), &p, &error), "3 numbers");
	std::string thirty, thirtyNan, thirtyBig;
	for (int i = 0; i < 30; ++i) {
		thirty += "0.1\n";
		thirtyNan += i == 7 ? "nan\n" : "0.1\n";
		thirtyBig += i == 7 ? "250\n" : "0.1\n";
	}
	CHECK(dyn_feel::loadParams(writeTemp("dyn_feel_30.dat", "# a comment\n" + thirty), &p, &error), "%s", error.c_str());
	CHECK(!dyn_feel::loadParams(writeTemp("dyn_feel_31.dat", thirty + "0.1\n"), &p, &error), "31 numbers");
	CHECK(!dyn_feel::loadParams(writeTemp("dyn_feel_nan.dat", thirtyNan), &p, &error), "nan");
	CHECK(!dyn_feel::loadParams(writeTemp("dyn_feel_big.dat", thirtyBig), &p, &error), "250");
	CHECK(!dyn_feel::loadParams(writeTemp("dyn_feel_text.dat", "law: \"u2\"\ndof: 4\n"), &p, &error), "a yaml file");
	CHECK(!dyn_feel::loadParams(writeTemp("dyn_feel_inf.dat", thirty.substr(0, thirty.size() - 4) + "-inf\n"), &p, &error),
	      "-inf");
	// a cable header with one number missing, and one with a negative slip force
	CHECK(!dyn_feel::loadParams(writeTemp("dyn_feel_short.h", "cable_model::Params p = { {1, 2, 3}, {4} };"), &p, &error),
	      "a short header");
	CHECK(!dyn_feel::loadParams(writeTemp("dyn_feel_none.h", "inline cable_model::Params params();"), &p, &error),
	      "a header without the initializer");
	// a 30-number fit with a negative viscous coefficient loads with a warning
	std::string negative;
	for (int i = 0; i < 30; ++i) negative += i == 1 ? "-0.2\n" : "0.1\n";
	CHECK(dyn_feel::loadParams(writeTemp("dyn_feel_neg.dat", negative), &p, &error) && p.warnings.size() == 1, "warning");
	(void) betaFile;
	printf("refusals: ok\n");
}

static void testMixer() {
	using namespace dyn_feel;
	const double T = 0.002;
	Limits lim;
	Terms t;
	for (int j = 0; j < 4; ++j) {
		t.gravity[j] = 5.0 + j;
		t.coriolis[j] = 0.1;
		t.viscous[j] = 0.4;
		t.dry[j] = 0.6;
		t.offset[j] = 0.2;
		t.inertia[j] = 9.0;  // never applied
	}
	const double lib[4] = {4.0, 5.0, 6.0, 7.0}, wrist[4] = {0.3, 0.3, 0.3, 0.3};
	double q[4] = {0.0, -1.0, 0.0, 1.5}, dq[4] = {0.0, 0.0, 0.0, 0.0};

	// nothing at the start, whatever the model says
	{
		Mixer m(lim);
		Targets none;
		for (int n = 0; n < 2000; ++n) {
			const double* u = m.step(none, t, lib, wrist, q, dq, true, T);
			for (int j = 0; j < 4; ++j) CHECK(u[j] == 0.0, "torque with nothing asked");
		}
		Targets termsOnly = none;
		for (int i = 0; i < N_TERMS; ++i) termsOnly.term[i] = true;
		termsOnly.level = 1.0;  // the model itself is still off
		for (int n = 0; n < 2000; ++n) {
			const double* u = m.step(termsOnly, t, lib, wrist, q, dq, true, T);
			for (int j = 0; j < 4; ++j) CHECK(u[j] == 0.0, "torque with the model off");
		}
	}

	// ramps: no cycle changes a joint's torque by more than its ramp allows
	{
		Mixer m(lim);
		Targets on;
		on.model = true;
		on.level = 1.0;
		on.frictionShare = 1.0;
		for (int i = 0; i < N_TERMS; ++i) on.term[i] = true;
		double prev[4] = {0, 0, 0, 0}, worstStep = 0.0;
		for (int n = 0; n < 3000; ++n) {
			const double* u = m.step(on, t, lib, wrist, q, dq, true, T);
			for (int j = 0; j < 4; ++j) {
				worstStep = std::max(worstStep, std::fabs(u[j] - prev[j]));
				prev[j] = u[j];
			}
		}
		// full: gravity + wrist - lib + coriolis + offset + viscous + dry = 1.3 + 0.1 + 0.2 + 1.0 = 2.6 on J1
		const double full[4] = {2.6, 2.6, 2.6, 2.6};
		for (int j = 0; j < 4; ++j) {
			const double want = std::min(full[j], lim.clamp[j]);
			CHECK(std::fabs(prev[j] - want) < 1e-9, "J%d settles at %.3f, wanted %.3f", j + 1, prev[j], want);
			CHECK(m.state().clamped[j] == (full[j] > lim.clamp[j]), "clamp flag J%d", j + 1);
		}
		// level, a weight and the share each move 1/rampTime per second: three factors rising together
		CHECK(worstStep < 3.0 * 2.6 * T / lim.rampTime + 1e-12, "a step of %.4f Nm in one cycle", worstStep);
		// off again: back to nothing, by a ramp
		Targets off = on;
		off.model = false;
		double first = 0.0;
		for (int n = 0; n < 1500; ++n) {
			const double* u = m.step(off, t, lib, wrist, q, dq, true, T);
			if (n == 0) first = u[0];
		}
		CHECK(first > 2.0, "switching off does not step to zero (%.3f)", first);
		CHECK(m.state().added[0] == 0.0 && m.state().level == 0.0, "off is nothing");
	}

	// the level scales, the friction share scales the friction only
	{
		Mixer m(lim);
		Targets half;
		half.model = true;
		half.level = 0.5;
		half.frictionShare = 0.25;
		half.term[VISCOUS] = half.term[DRY] = half.term[CORIOLIS] = true;
		for (int n = 0; n < 3000; ++n) m.step(half, t, lib, wrist, q, dq, true, T);
		CHECK(std::fabs(m.state().added[0] - 0.5 * (0.1 + 0.25 * 1.0)) < 1e-12, "level and share: %.4f", m.state().added[0]);
	}

	// speed: fades, then trips; the trip holds until it is acknowledged
	{
		Mixer m(lim);
		Targets on;
		on.model = true;
		on.level = 1.0;
		on.term[OFFSET] = true;
		for (int n = 0; n < 3000; ++n) m.step(on, t, lib, wrist, q, dq, true, T);
		CHECK(std::fabs(m.state().added[1] - 0.2) < 1e-12, "offset on");
		dq[1] = 0.5 * (lim.speedFade + lim.speedTrip);
		m.step(on, t, lib, wrist, q, dq, true, T);
		CHECK(std::fabs(m.state().added[1] - 0.1) < 1e-12 && m.state().trip == NO_TRIP, "half way: %.4f", m.state().added[1]);
		CHECK(std::fabs(m.state().added[0] - 0.2) < 1e-12, "the other joints are not faded");
		dq[1] = lim.speedTrip + 0.01;
		m.step(on, t, lib, wrist, q, dq, true, T);
		CHECK(m.state().trip == TRIP_SPEED && m.state().tripJoint == 1, "trip");
		CHECK(m.state().added[1] == 0.0, "nothing on the fast joint");
		dq[1] = 0.0;
		for (int n = 0; n < 1000; ++n) m.step(on, t, lib, wrist, q, dq, true, T);
		CHECK(m.state().level == 0.0 && m.state().added[0] == 0.0 && m.state().trip == TRIP_SPEED, "the trip holds");
		m.acknowledge();
		for (int n = 0; n < 3000; ++n) m.step(on, t, lib, wrist, q, dq, true, T);
		CHECK(std::fabs(m.state().added[1] - 0.2) < 1e-12, "on again after the acknowledgement");
	}

	// joint range: nothing at the bound, half at half the margin
	{
		Mixer m(lim);
		Targets on;
		on.model = true;
		on.level = 1.0;
		on.term[OFFSET] = true;
		double qq[4] = {0.0, lim.lower[1] + 0.5 * lim.margin, 0.0, lim.upper[3] + 0.02};
		for (int n = 0; n < 3000; ++n) m.step(on, t, lib, wrist, qq, dq, true, T);
		CHECK(std::fabs(m.state().added[1] - 0.1) < 1e-9, "half the margin: %.4f", m.state().added[1]);
		CHECK(m.state().added[3] == 0.0, "beyond the bound");
		CHECK(std::fabs(m.state().added[0] - 0.2) < 1e-12, "inside");
	}

	// a bad cycle: nothing, at once, and the model stays off
	{
		Mixer m(lim);
		Targets on;
		on.model = true;
		on.level = 1.0;
		on.term[OFFSET] = true;
		for (int n = 0; n < 3000; ++n) m.step(on, t, lib, wrist, q, dq, true, T);
		const double* u = m.step(on, t, lib, wrist, q, dq, false, T);
		for (int j = 0; j < 4; ++j) CHECK(u[j] == 0.0, "a bad cycle");
		for (int n = 0; n < 3000; ++n) m.step(on, t, lib, wrist, q, dq, true, T);
		CHECK(m.state().added[0] == 0.0 && m.state().trip == TRIP_INVALID, "stays off");
		// a term that is not finite although the cycle was called valid
		Mixer m2(lim);
		Terms bad = t;
		bad.offset[2] = std::nan("");
		for (int n = 0; n < 3000; ++n) m2.step(on, bad, lib, wrist, q, dq, true, T);
		for (int j = 0; j < 4; ++j) CHECK(m2.state().added[j] == 0.0, "a nan term");
		CHECK(m2.state().trip == TRIP_INVALID, "a nan term trips");
	}

	// the clamp holds for any model
	{
		Mixer m(lim);
		Targets on;
		on.model = true;
		on.level = 1.0;
		on.frictionShare = 1.0;
		for (int i = 0; i < N_TERMS; ++i) on.term[i] = true;
		Terms huge = t;
		for (int j = 0; j < 4; ++j) huge.gravity[j] = (j % 2 ? -1.0 : 1.0) * 500.0;
		for (int n = 0; n < 3000; ++n) {
			const double* u = m.step(on, huge, lib, wrist, q, dq, true, T);
			for (int j = 0; j < 4; ++j) CHECK(std::fabs(u[j]) <= lim.clamp[j], "clamp");
		}
		CHECK(m.state().clamped[0] && m.state().clamped[1], "clamp flags");
	}
	printf("mixer: ok\n");
}

int main(int argc, char** argv) {
	if (argc != 4) {
		printf("usage: %s <beta30.dat> <notebook rows.txt> <cable_params_wam8g.h>\n", argv[0]);
		return 2;
	}
	testBeta30(argv[1], argv[2]);
	testCableHeader(argv[3]);
	testRefusals(argv[1]);
	testMixer();
	printf("%d checks, %d failed\n", checks, failures);
	return failures == 0 ? 0 : 1;
}
