#pragma once

/* wam8g_dyn_feel's arithmetic, without libbarrett (Eigen, regressor_W.h and
 * cable_drive_model.h only), so it is tested offline
 * (test/test_dyn_feel_core.cpp):
 *
 *   Params, loadParams()   the identified J1-J4 parameters, read from a file;
 *   Evaluator              the model's torque term by term, each control cycle;
 *   Mixer                  what is added to libbarrett's gravity compensation:
 *                          the chosen terms, ramped, gated and clamped.
 *
 * Parameter files (told apart by their content):
 *
 *   30 numbers             the base parameters of regressor_W.h, as the
 *                          Feas-Ident-WAM4 notebook writes them
 *                          (tmp/solutions/beta_star_correct.dat; a
 *                          leader_beta_*.h holds the same 30 values, but that
 *                          C++ header is not read). Their viscous slots (1, 10,
 *                          19, 28) and Coulomb slots (2, 11, 20, 29) become the
 *                          joint friction terms here;
 *   cable_params_*.h       the header python -m cable_ident fit writes (ident/):
 *                          the numbers of its cable_model::Params initializer,
 *                          read as text. Its .json record is not read: it has
 *                          no TM and no slip displacements.
 *
 * Terms, J1-J4, Nm (tau = the torque the motors must give for the motion):
 *
 *   gravity    W(q, 0, 0) beta
 *   coriolis   W(q, dq, 0) beta - W(q, 0, 0) beta   (beta's friction slots are 0)
 *   viscous    TM^T diag(v) TM dq  +  fv dq
 *   dry        the cable model's Jenkins elements  +  fc tanh(dq / v0)
 *   offset     the cable model's constant torques
 *   inertia    M(q) ddq and the rotors: shown, never applied (see Mixer). */

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "cable_drive_model.h"

namespace dyn_feel {

const int FV_SLOT[4] = {1, 10, 19, 28};  // regressor_W.h: fv_1..fv_4
const int FC_SLOT[4] = {2, 11, 20, 29};  // regressor_W.h: fc_1..fc_4

// cable_ident's defaults (ident/cable_ident/model.py), for a 30-number file,
// which has no elements: only so that nothing divides by zero.
const double STOCK_TM[4][4] = {{-42.0, 0.0, 0.0, 0.0},
                               {0.0, 28.25, -16.8155, 0.0},
                               {0.0, -28.25, -16.8155, 0.0},
                               {0.0, 0.0, 0.0, -18.0}};
const double STOCK_MOTOR_DELTA[6] = {7.7e-3, 1.6e-2, 3.2e-2, 6.4e-2, 0.13, 0.26};

struct Params {
	enum Kind { BETA30, CABLE };
	Kind kind;
	cable_model::Params cable;  // beta with its friction slots 0; a BETA30 file leaves the elements 0
	double jointViscous[4];     // Nm s/rad at the joint (beta's fv slots)
	double jointCoulomb[4];     // Nm at the joint (beta's fc slots)
	std::string source;
	std::vector<std::string> warnings;
};

inline const char* kindName(Params::Kind k) {
	return k == Params::CABLE ? "cable-drive model (cable_params_*.h)" : "30 base parameters (regressor_W.h)";
}

namespace detail {

inline bool readFile(const std::string& path, std::string* text) {
	std::ifstream in(path.c_str(), std::ios::in | std::ios::binary);
	if (!in) {
		return false;
	}
	std::ostringstream s;
	s << in.rdbuf();
	*text = s.str();
	return true;
}

// Removes //, # and /* */ comments.
inline std::string stripComments(const std::string& t, bool hash) {
	std::string out;
	out.reserve(t.size());
	size_t i = 0;
	while (i < t.size()) {
		if (t.compare(i, 2, "//") == 0 || (hash && t[i] == '#')) {
			while (i < t.size() && t[i] != '\n') ++i;
		} else if (t.compare(i, 2, "/*") == 0) {
			const size_t end = t.find("*/", i + 2);
			i = end == std::string::npos ? t.size() : end + 2;
			out += ' ';
		} else {
			out += t[i++];
		}
	}
	return out;
}

/* Every number of text, which may hold only numbers and the characters of
 * allowed between them. False at anything else (a name, "nan", "inf"). */
inline bool numbers(const std::string& text, const char* allowed, std::vector<double>* out, std::string* error) {
	const char* p = text.c_str();
	const char* const end = p + text.size();
	while (p < end) {
		const char c = *p;
		if (c == ' ' || c == '\t' || c == '\r' || c == '\n' || std::strchr(allowed, c) != NULL) {
			++p;
			continue;
		}
		if ((c >= '0' && c <= '9') || c == '-' || c == '+' || c == '.') {
			char* stop = NULL;
			const double v = std::strtod(p, &stop);
			if (stop == p) {
				*error = std::string("cannot read a number at \"") + std::string(p, std::min<size_t>(12, end - p)) + "\"";
				return false;
			}
			out->push_back(v);
			p = stop;
			continue;
		}
		*error = std::string("unexpected text \"") + std::string(p, std::min<size_t>(12, end - p)) + "\"";
		return false;
	}
	return true;
}

inline void zeroElements(cable_model::Params* c) {
	for (int r = 0; r < 4; ++r) {
		for (int k = 0; k < 4; ++k) c->TM[r][k] = STOCK_TM[r][k];
		for (int i = 0; i < cable_model::KM; ++i) c->motorW[r][i] = 0.0;
		c->motorViscous[r] = 0.0;
		c->rotor[r] = 0.0;
		c->offset[r] = 0.0;
	}
	for (int i = 0; i < cable_model::KM; ++i) c->motorDelta[i] = STOCK_MOTOR_DELTA[i % 6];
	for (int i = 0; i < cable_model::KJ; ++i) {
		c->jointDelta[i] = STOCK_MOTOR_DELTA[i % 6] / 28.25;
		c->jointW[0][i] = c->jointW[1][i] = 0.0;
	}
}

inline std::string joined(const char* what, const double* v, int n) {
	char buf[64];
	std::string s(what);
	for (int i = 0; i < n; ++i) {
		std::snprintf(buf, sizeof(buf), " %.4g", v[i]);
		s += buf;
	}
	return s;
}

/* Refuses what cannot be a WAM's J1-J4 (a wrong file, a torn file); warns
 * about what a fit can produce but should not. */
inline bool validate(Params* p, std::string* error) {
	const cable_model::Params& c = p->cable;
	bool finite = true;
	for (int i = 0; i < 30; ++i) finite = finite && std::isfinite(c.beta[i]);
	for (int r = 0; r < 4; ++r) {
		for (int k = 0; k < 4; ++k) finite = finite && std::isfinite(c.TM[r][k]);
		for (int i = 0; i < cable_model::KM; ++i) finite = finite && std::isfinite(c.motorW[r][i]);
		finite = finite && std::isfinite(c.motorViscous[r]) && std::isfinite(c.rotor[r]) && std::isfinite(c.offset[r]) &&
		         std::isfinite(p->jointViscous[r]) && std::isfinite(p->jointCoulomb[r]);
	}
	for (int i = 0; i < cable_model::KM; ++i) finite = finite && std::isfinite(c.motorDelta[i]);
	for (int i = 0; i < cable_model::KJ; ++i) {
		finite = finite && std::isfinite(c.jointDelta[i]) && std::isfinite(c.jointW[0][i]) && std::isfinite(c.jointW[1][i]);
	}
	if (!finite) {
		*error = "a parameter is not finite";
		return false;
	}
	for (int i = 0; i < 30; ++i) {
		if (std::fabs(c.beta[i]) > 20.0) {  // the WAM's largest are near 3 (kg m, kg m^2)
			*error = "a base parameter is beyond 20: not a WAM's J1-J4";
			return false;
		}
	}
	for (int i = 0; i < cable_model::KM; ++i) {
		if (!(c.motorDelta[i] > 0.0)) {
			*error = "a motor slip displacement is not positive";
			return false;
		}
	}
	for (int i = 0; i < cable_model::KJ; ++i) {
		if (!(c.jointDelta[i] > 0.0)) {
			*error = "a joint slip displacement is not positive";
			return false;
		}
	}
	for (int r = 0; r < 4; ++r) {
		double tmRow = 0.0;
		for (int k = 0; k < 4; ++k) tmRow = std::max(tmRow, std::fabs(c.TM[r][k]));
		if (tmRow > 100.0) {
			*error = "a transmission ratio is beyond 100";
			return false;
		}
		for (int i = 0; i < cable_model::KM; ++i) {
			if (c.motorW[r][i] < 0.0 || c.motorW[r][i] > 1.0) {
				*error = "a motor slip force is negative or beyond 1 Nm at the motor";
				return false;
			}
		}
		if (c.motorViscous[r] < 0.0 || c.motorViscous[r] > 0.05 || c.rotor[r] < 0.0 || c.rotor[r] > 0.01) {
			*error = "a motor viscous coefficient or rotor inertia is negative or far too large";
			return false;
		}
		if (std::fabs(c.offset[r]) > 5.0) {
			*error = "an offset is beyond 5 Nm";
			return false;
		}
		if (std::fabs(p->jointViscous[r]) > 10.0 || std::fabs(p->jointCoulomb[r]) > 10.0) {
			*error = "a joint friction parameter is beyond 10";
			return false;
		}
	}
	for (int j = 0; j < 2; ++j) {
		for (int i = 0; i < cable_model::KJ; ++i) {
			if (c.jointW[j][i] < 0.0 || c.jointW[j][i] > 10.0) {
				*error = "a J2/J3 slip force is negative or beyond 10 Nm";
				return false;
			}
		}
	}
	bool negV = false, negC = false;
	for (int r = 0; r < 4; ++r) {
		negV = negV || p->jointViscous[r] < 0.0;
		negC = negC || p->jointCoulomb[r] < 0.0;
	}
	if (negV) p->warnings.push_back("a viscous coefficient is negative (an unconstrained fit?): its term adds damping");
	if (negC) p->warnings.push_back("a Coulomb coefficient is negative (an unconstrained fit?): its term adds drag");
	return true;
}

}  // namespace detail

/* Reads path. False with *error set when the file is not one of the two
 * formats or its numbers cannot be a WAM's. */
inline bool loadParams(const std::string& path, Params* p, std::string* error) {
	std::string text;
	if (!detail::readFile(path, &text)) {
		*error = "cannot read " + path;
		return false;
	}
	p->source = path;
	p->warnings.clear();
	for (int r = 0; r < 4; ++r) p->jointViscous[r] = p->jointCoulomb[r] = 0.0;
	std::vector<double> v;
	const size_t marker = text.find("cable_model::Params");
	if (marker != std::string::npos) {
		// cable_params_*.h: the initializer of "cable_model::Params p = { ... };"
		const std::string code = detail::stripComments(text, false);
		const std::string name = "cable_model::Params";
		size_t open = std::string::npos;
		for (size_t at = code.find(name); at != std::string::npos && open == std::string::npos;
		     at = code.find(name, at + 1)) {
			// "cable_model::Params <identifier> = {" (not the function's return type)
			size_t i = at + name.size();
			while (i < code.size() && std::isspace((unsigned char) code[i])) ++i;
			const size_t id = i;
			while (i < code.size() && (std::isalnum((unsigned char) code[i]) || code[i] == '_')) ++i;
			if (i == id) continue;
			while (i < code.size() && std::isspace((unsigned char) code[i])) ++i;
			if (i >= code.size() || code[i] != '=') continue;
			++i;
			while (i < code.size() && std::isspace((unsigned char) code[i])) ++i;
			if (i < code.size() && code[i] == '{') open = i;
		}
		if (open == std::string::npos) {
			*error = path + ": no \"cable_model::Params p = {\" initializer";
			return false;
		}
		size_t close = open;
		int depth = 0;
		for (; close < code.size(); ++close) {
			if (code[close] == '{') ++depth;
			if (code[close] == '}' && --depth == 0) break;
		}
		if (close >= code.size()) {
			*error = path + ": the initializer's braces do not close";
			return false;
		}
		std::string why;
		if (!detail::numbers(code.substr(open, close - open + 1), "{},", &v, &why)) {
			*error = path + ": " + why;
			return false;
		}
		const int KM = cable_model::KM, KJ = cable_model::KJ;
		const size_t want = 30 + 16 + KM + 4 * KM + KJ + 2 * KJ + 12;
		if (v.size() != want) {
			std::ostringstream s;
			s << path << ": " << v.size() << " numbers in the initializer, " << want << " expected (" << KM
			  << " elements per motor, " << KJ << " on J2/J3)";
			*error = s.str();
			return false;
		}
		// in cable_model::Params' order (ident/cable_ident/export.py writes it)
		cable_model::Params& c = p->cable;
		size_t n = 0;
		for (int i = 0; i < 30; ++i) c.beta[i] = v[n++];
		for (int r = 0; r < 4; ++r) for (int k = 0; k < 4; ++k) c.TM[r][k] = v[n++];
		for (int i = 0; i < KM; ++i) c.motorDelta[i] = v[n++];
		for (int r = 0; r < 4; ++r) for (int i = 0; i < KM; ++i) c.motorW[r][i] = v[n++];
		for (int i = 0; i < KJ; ++i) c.jointDelta[i] = v[n++];
		for (int j = 0; j < 2; ++j) for (int i = 0; i < KJ; ++i) c.jointW[j][i] = v[n++];
		for (int r = 0; r < 4; ++r) c.motorViscous[r] = v[n++];
		for (int r = 0; r < 4; ++r) c.rotor[r] = v[n++];
		for (int r = 0; r < 4; ++r) c.offset[r] = v[n++];
		p->kind = Params::CABLE;
	} else {
		std::string why;
		if (!detail::numbers(detail::stripComments(text, true), ",;", &v, &why)) {
			*error = path + ": " + why + " (30 numbers, or a cable_params_*.h, expected)";
			return false;
		}
		if (v.size() != 30) {
			std::ostringstream s;
			s << path << ": " << v.size() << " numbers; the base parameters of regressor_W.h are 30";
			*error = s.str();
			return false;
		}
		detail::zeroElements(&p->cable);
		for (int i = 0; i < 30; ++i) p->cable.beta[i] = v[i];
		p->kind = Params::BETA30;
	}
	// beta's friction slots become the joint friction terms (a cable header's are 0)
	for (int r = 0; r < 4; ++r) {
		p->jointViscous[r] = p->cable.beta[FV_SLOT[r]];
		p->jointCoulomb[r] = p->cable.beta[FC_SLOT[r]];
		p->cable.beta[FV_SLOT[r]] = 0.0;
		p->cable.beta[FC_SLOT[r]] = 0.0;
	}
	std::string why;
	if (!detail::validate(p, &why)) {
		*error = path + ": " + why;
		return false;
	}
	return true;
}

/* A few lines for the operator: what was read. */
inline std::string describe(const Params& p) {
	std::string s = "Parameters: " + p.source + "\n  " + kindName(p.kind) + "\n";
	s += detail::joined("  joint viscous fv (Nm s/rad):", p.jointViscous, 4) + "\n";
	s += detail::joined("  joint Coulomb fc (Nm):      ", p.jointCoulomb, 4) + "\n";
	if (p.kind == Params::CABLE) {
		double motorSum[4], jointSum[2] = {0.0, 0.0};
		for (int r = 0; r < 4; ++r) {
			motorSum[r] = 0.0;
			for (int i = 0; i < cable_model::KM; ++i) motorSum[r] += p.cable.motorW[r][i];
			double ratio = 0.0;
			for (int k = 0; k < 4; ++k) ratio = std::max(ratio, std::fabs(p.cable.TM[r][k]));
			motorSum[r] *= ratio;  // at the joint it drives hardest
		}
		for (int j = 0; j < 2; ++j) for (int i = 0; i < cable_model::KJ; ++i) jointSum[j] += p.cable.jointW[j][i];
		s += detail::joined("  cable friction when slipping, motors 1-4 at the joint (Nm):", motorSum, 4) + "\n";
		s += detail::joined("  ... and the J2, J3 elements (Nm):", jointSum, 2) + "\n";
		s += detail::joined("  motor viscous (Nm s/rad at the motor):", p.cable.motorViscous, 4) + "\n";
		s += detail::joined("  offsets (Nm):", p.cable.offset, 4) + "\n";
	}
	for (size_t i = 0; i < p.warnings.size(); ++i) s += "  WARNING: " + p.warnings[i] + "\n";
	return s;
}

// The model's gravity torque on J1-J4 at q: W(q, 0, 0) beta (no state).
inline void modelGravity(const Params& p, const double* q, double* g) {
	double W[120];
	const double zero[4] = {0.0, 0.0, 0.0, 0.0};
	calculate_W(q, zero, zero, W);
	for (int j = 0; j < 4; ++j) {
		double s = 0.0;
		for (int c = 0; c < 30; ++c) s += W[j * 30 + c] * p.cable.beta[c];
		g[j] = s;
	}
}

struct Terms {
	double gravity[4], coriolis[4], viscous[4], dry[4], offset[4], inertia[4];
	Terms() {
		for (int j = 0; j < 4; ++j) gravity[j] = coriolis[j] = viscous[j] = dry[j] = offset[j] = inertia[j] = 0.0;
	}
};

/* The model term by term. update() every control cycle, from the first one,
 * whether or not anything is applied: the friction elements follow the
 * joints' motion. Nothing is allocated after construction. */
class Evaluator {
public:
	/* coulombV0: fc tanh(dq / v0), rad/s. The fits use the velocity's sign
	 * (the notebook) or tanh(100 dq) (regressor_W.h, v0 = 0.01), which flips
	 * with the velocity noise of a joint at rest; lib/teleop_law.h uses 0.05
	 * for J6's friction feedforward. */
	Evaluator(const Params& p, double coulombV0) : model(p.cable), v0(coulombV0) {
		for (int i = 0; i < 30; ++i) beta[i] = p.cable.beta[i];
		for (int j = 0; j < 4; ++j) {
			fv[j] = p.jointViscous[j];
			fc[j] = p.jointCoulomb[j];
		}
		for (int i = 0; i < 120; ++i) Wg[i] = 0.0;
	}

	void reset() { model.reset(); }

	// ddq only feeds the displayed inertial term.
	void update(const double* q, const double* dq, const double* ddq) {
		cable_model::Vec4 qv, dqv, ddqv;
		for (int j = 0; j < 4; ++j) {
			qv[j] = q[j];
			dqv[j] = dq[j];
			ddqv[j] = ddq[j];
		}
		model.update(qv, dqv, ddqv);
		const double zero[4] = {0.0, 0.0, 0.0, 0.0};
		calculate_W(q, zero, zero, Wg);
		const cable_model::Vec4& gc = model.gravityCoriolis();
		const cable_model::Vec4& visc = model.viscousFriction();
		const cable_model::Vec4& hyst = model.hysteresis();
		const cable_model::Vec4& off = model.offset();
		const cable_model::Vec4& inert = model.inertia();
		for (int j = 0; j < 4; ++j) {
			double g = 0.0;
			for (int c = 0; c < 30; ++c) g += Wg[j * 30 + c] * beta[c];
			t.gravity[j] = g;
			t.coriolis[j] = gc[j] - g;
			t.viscous[j] = visc[j] + fv[j] * dq[j];
			t.dry[j] = hyst[j] + fc[j] * std::tanh(dq[j] / v0);
			t.offset[j] = off[j];
			t.inertia[j] = inert[j];
		}
	}

	const Terms& terms() const { return t; }

private:
	cable_model::Model model;
	double beta[30], fv[4], fc[4];
	const double v0;
	double Wg[120];
	Terms t;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

enum Term { GRAVITY = 0, CORIOLIS, VISCOUS, DRY, OFFSET, N_TERMS };

inline const char* termName(int term) {
	switch (term) {
	case GRAVITY: return "gravity";
	case CORIOLIS: return "Coriolis";
	case VISCOUS: return "viscous";
	case DRY: return "dry friction";
	case OFFSET: return "offsets";
	}
	return "?";
}

/* What the operator asks for. The start: nothing. */
struct Targets {
	bool model;             // false: libbarrett's gravity compensation alone
	bool term[N_TERMS];
	double level;           // 0 to 1: the share of everything below that is applied
	double frictionShare;   // 0 to 1: on VISCOUS and DRY, within level
	Targets() : model(false), level(0.0), frictionShare(0.5) {
		for (int i = 0; i < N_TERMS; ++i) term[i] = false;
	}
};

struct Limits {
	double clamp[4];        // Nm: the most that is added on each of J1-J4
	double speedFade;       // rad/s: above it the added torque fades ...
	double speedTrip;       // ... to nothing here, and the model switches off
	double lower[4], upper[4];  // rad: nothing is added beyond these ...
	double margin;          // ... and it fades in over this much inside them
	double rampTime;        // s for a weight to go from 0 to 1
	double tripRampTime;    // s for the level to fall from 1 to 0 after a trip
	Limits() : speedFade(0.6), speedTrip(1.0), margin(0.10), rampTime(2.0), tripRampTime(0.25) {
		// 0.05 rad inside the WAM's nominal joint ranges (J1 +-2.6, J2 +-2.0,
		// J3 +-2.8, J4 -0.9 to 3.1 rad): full within 0.15 rad of a stop,
		// nothing within 0.05 rad.
		const double lo[4] = {-2.55, -1.95, -2.75, -0.85}, hi[4] = {2.55, 1.95, 2.75, 3.05};
		const double c[4] = {2.5, 2.5, 1.5, 1.5};
		for (int j = 0; j < 4; ++j) {
			lower[j] = lo[j];
			upper[j] = hi[j];
			clamp[j] = c[j];
		}
	}
};

enum Trip { NO_TRIP = 0, TRIP_SPEED, TRIP_INVALID, TRIP_UI_STALL, TRIP_STOPPED };

inline const char* tripText(int trip) {
	switch (trip) {
	case NO_TRIP: return "";
	case TRIP_SPEED: return "a joint passed the speed limit";
	case TRIP_INVALID: return "a signal was missing or not finite";
	case TRIP_UI_STALL: return "the display stopped answering";
	case TRIP_STOPPED: return "stopped";
	}
	return "?";
}

struct MixerState {
	double level;             // as applied now (ramped)
	double weight[N_TERMS];   // as applied now (ramped)
	double frictionShare;     // as applied now (ramped)
	double raw[4];            // the chosen terms at full level, before gates and clamp
	double added[4];          // what is added to J1-J4 this cycle
	double gate[4];           // 0 to 1: the speed and joint-range fade
	bool clamped[4];
	int trip, tripJoint;
	MixerState() : level(0.0), frictionShare(0.0), trip(NO_TRIP), tripJoint(-1) {
		for (int i = 0; i < N_TERMS; ++i) weight[i] = 0.0;
		for (int j = 0; j < 4; ++j) {
			raw[j] = added[j] = 0.0;
			gate[j] = 1.0;
			clamped[j] = false;
		}
	}
};

/* What is added to J1-J4 on top of libbarrett's gravity compensation:
 *
 *   added = clamp( level gate ( wG (gravity + wrist - library gravity)
 *                               + wK coriolis + wO offset
 *                               + share (wV viscous + wD dry) ) )
 *
 * - level and every weight move toward their targets at 1 / rampTime per
 *   second: no key steps a torque. level 0 is libbarrett alone; with the
 *   gravity term on, level 1 swaps libbarrett's J1-J4 gravity for the
 *   model's (the laws' dyn - grav, lib/teleop_law.h).
 * - gate fades each joint's added torque out between speedFade and speedTrip
 *   and toward the ends of its range, so the compensation never pushes a
 *   joint into the safety board's velocity fault or into a stop. Past
 *   speedTrip the model switches off (trip) until the operator switches it on
 *   again: a friction term set too high drives its joint.
 * - clamp bounds what a wrong parameter file can do.
 * - The inertial term is not here. From the measured velocity it exists only
 *   low-passed (25 and 50 rad/s), so feeding it forward is acceleration
 *   feedback with a phase lag: at 4 Hz a share k of an inertia M acts as a
 *   NEGATIVE damper of about 15 k M Nm s/rad (k = 0.25, M = 1 kg m^2: 3.8,
 *   more than J1's identified viscous friction). It is shown, not applied. */
class Mixer {
public:
	explicit Mixer(const Limits& limits) : lim(limits) {}

	const Limits& limits() const { return lim; }
	const MixerState& state() const { return st; }

	// Latches the model off (the level falls over tripRampTime).
	void trip(int reason, int joint = -1) {
		if (st.trip == NO_TRIP) {
			st.trip = reason;
			st.tripJoint = joint;
		}
	}
	// The operator has seen the trip; the next "model on" counts again.
	void acknowledge() {
		st.trip = NO_TRIP;
		st.tripJoint = -1;
	}

	/* One control cycle. libGravity: libbarrett's gravity torque on J1-J4,
	 * already applied. wrist: the wrist's gravity on J1-J4 relative to the
	 * pose the model was identified at (arm_gravity.h; zeros: none).
	 * valid: every signal of this cycle is defined and finite. */
	const double* step(const Targets& target, const Terms& t, const double* libGravity, const double* wrist,
	                   const double* q, const double* dq, bool valid, double period) {
		if (!valid) {
			// nothing can be computed: nothing is added, at once
			trip(TRIP_INVALID);
			st.level = 0.0;
			for (int j = 0; j < 4; ++j) {
				st.raw[j] = st.added[j] = 0.0;
				st.gate[j] = 0.0;
				st.clamped[j] = false;
			}
			return st.added;
		}
		for (int j = 0; j < 4; ++j) {
			if (std::fabs(dq[j]) > lim.speedTrip) {
				trip(TRIP_SPEED, j);
			}
		}
		const double step = period / lim.rampTime;
		if (st.trip != NO_TRIP) {
			st.level = slew(st.level, 0.0, period / lim.tripRampTime);
		} else {
			st.level = slew(st.level, target.model ? unit(target.level) : 0.0, step);
		}
		for (int i = 0; i < N_TERMS; ++i) {
			st.weight[i] = slew(st.weight[i], target.term[i] ? 1.0 : 0.0, step);
		}
		st.frictionShare = slew(st.frictionShare, unit(target.frictionShare), step);
		for (int j = 0; j < 4; ++j) {
			const double raw = st.weight[GRAVITY] * (t.gravity[j] + wrist[j] - libGravity[j]) +
			                   st.weight[CORIOLIS] * t.coriolis[j] + st.weight[OFFSET] * t.offset[j] +
			                   st.frictionShare * (st.weight[VISCOUS] * t.viscous[j] + st.weight[DRY] * t.dry[j]);
			const double speed = std::fabs(dq[j]);
			const double gs = unit((lim.speedTrip - speed) / std::max(1e-6, lim.speedTrip - lim.speedFade));
			const double gp = unit(std::min(q[j] - lim.lower[j], lim.upper[j] - q[j]) / std::max(1e-6, lim.margin));
			double u = st.level * gs * gp * raw;
			st.clamped[j] = std::fabs(u) > lim.clamp[j];
			if (u > lim.clamp[j]) u = lim.clamp[j];
			if (u < -lim.clamp[j]) u = -lim.clamp[j];
			if (!std::isfinite(u)) {  // a term went wrong although its inputs were finite
				trip(TRIP_INVALID, j);
				u = 0.0;
				st.level = 0.0;
			}
			st.raw[j] = raw;
			st.gate[j] = gs * gp;
			st.added[j] = u;
		}
		if (st.trip == TRIP_INVALID) {
			for (int j = 0; j < 4; ++j) st.added[j] = 0.0;
		}
		return st.added;
	}

private:
	static double unit(double x) { return x < 0.0 ? 0.0 : (x > 1.0 ? 1.0 : x); }
	static double slew(double now, double target, double step) {
		if (target > now + step) return now + step;
		if (target < now - step) return now - step;
		return target;
	}

	const Limits lim;
	MixerState st;
};

}  // namespace dyn_feel
