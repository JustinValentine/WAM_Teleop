#pragma once

/* The feedforward torque each arm adds to the position coupling (both arms'
 * position controllers already track the other arm's joints). Same laws as
 * compute_control() in leader.h / follower.h:
 *
 *   u1 = 0                                  position-position only
 *   u2 = dyn - grav                         + dynamics compensation
 *   u4 = -0.5 ext_other + dyn - grav        4C: + force channels (Lawrence)
 *   u6 = u4 - 0.15 (ext_other + ext_own)    4C + local force feedback
 *
 * ext = commanded torque - model torque, the external torque estimate (what a
 * hand or the environment pushes with). dyn = the fitted model's torque;
 * grav = libbarrett's gravity torque, already applied by gravity
 * compensation, so dyn - grav swaps it for the model's. J1-J4 only (the model
 * has no wrist); every other joint gets zero. */

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace teleop_law {

enum Law { U1 = 1, U2 = 2, U4 = 4, U6 = 6 };

inline bool parse(const std::string& s, Law* law) {
	if (s == "u1") { *law = U1; return true; }
	if (s == "u2") { *law = U2; return true; }
	if (s == "u4") { *law = U4; return true; }
	if (s == "u6") { *law = U6; return true; }
	return false;
}

inline const char* name(Law law) {
	switch (law) {
	case U1: return "u1 (position-position)";
	case U2: return "u2 (position-position + dynamics compensation)";
	case U4: return "u4 (4C + dynamics compensation)";
	case U6: return "u6 (4C + dynamics compensation + local force feedback)";
	}
	return "?";
}

/* True when the law uses the dynamics model (dyn or ext). */
inline bool usesModel(Law law) { return law != U1; }

/* The force channel (ext_other) as the laws use it. An arm sends its external
 * torque only while it is linked (unlinked, its estimate pushed the other arm
 * by about its gravity torque), so a nonzero ext_other means both are linked;
 * from then on it fades in over FORCE_FADE_TIME, so the loop never closes with
 * a step. *t is the time both have been linked. */
const double FORCE_FADE_TIME = 2.0;  // s
/* --force-filter: a first-order low-pass on the force channel, in Hz (0: off).
 * The closed 4C loop went unstable at 22-36 Hz, where the round trip between
 * the arms delays it by a quarter cycle or more; a hand's forces are slower. */
inline bool parseForceFilter(const std::string& s, double* hz) {
	char* end = NULL;
	const double v = std::strtod(s.c_str(), &end);
	if (end == s.c_str() || *end != '\0' || !(v >= 0.0 && v <= 100.0)) {
		return false;
	}
	*hz = v;
	return true;
}
template <typename JT>
JT lowPass(const JT& x, double cutoffHz, double period, JT* state) {
	if (cutoffHz <= 0.0) {
		*state = x;
	} else {
		*state += (1.0 - std::exp(-2.0 * M_PI * cutoffHz * period)) * (x - *state);
	}
	return *state;
}

/* --force-notch: a notch on the force channel at hz (0: off; parseForceFilter's
 * range). The 4C loop rings at a resonance of the arms, whatever the force
 * filter: J3 at 31-36 Hz in every run, J2 at 30-42 Hz, J1 at 37-43 Hz. At
 * 35 Hz with Q 0.7 the notch passes 0.12-0.3 of 30-43 Hz (a 15 Hz low-pass,
 * enough to quiet the ring: 0.33-0.44), and lags the two arms' slow relative
 * swing, near 4 Hz, 9 degrees (the low-pass: 15). */
const double FORCE_NOTCH_Q = 0.7;
template <typename JT>
struct Notch {
	JT s1, s2;  // direct form II transposed
	Notch() {
		s1.setZero();
		s2.setZero();
	}
	JT operator()(const JT& x, double hz, double period) {
		if (hz <= 0.0) {
			return x;
		}
		const double w = 2.0 * M_PI * hz * period;
		const double alpha = std::sin(w) / (2.0 * FORCE_NOTCH_Q);
		const double b = 1.0 / (1.0 + alpha);             // b0 = b2
		const double c = -2.0 * std::cos(w) / (1.0 + alpha);  // b1 = a1
		const double a2 = (1.0 - alpha) / (1.0 + alpha);
		const JT y = b * x + s1;
		s1 = c * x - c * y + s2;
		s2 = b * x - a2 * y;
		return y;
	}
};

/* --j6-friction: a friction feedforward on the wam8g's J6 (leader_wam8g.h), a
 * torque in the direction the joint turns, fading out below FRICTION_V0 so the
 * velocity noise of a joint at rest does not flip it. For J6's bearing drag
 * (its outer race was squeezed when it was installed): set it under the drag
 * (wam8g_j6_friction measures the breakaway), since more than the drag drives
 * the joint by itself. */
const double FRICTION_V0 = 0.05;  // rad/s
inline double frictionFeedforward(double nm, double velocity) {
	return nm * std::tanh(velocity / FRICTION_V0);
}

/* The other arm's position as this arm's reference. Each arm sends a packet
 * every cycle, but the two loops are not synchronized: a cycle can find no new
 * packet and the next one two, so the raw reference stalls and then jumps a
 * double step, and the position controller's D term turns that into a torque
 * spike proportional to the speed. Between packets the reference advances with
 * the other arm's sent velocity, for at most REFERENCE_EXTRAPOLATION cycles
 * (then it holds, and the link times out after 20 ms as before). */
const int REFERENCE_EXTRAPOLATION = 2;
template <typename V, typename Stamp>
struct ReferenceTracker {
	V position;
	Stamp stamp;
	int missed;
	bool valid;
	ReferenceTracker() : missed(0), valid(false) {}
	void update(const V& jp, const V& jv, const Stamp& s, double period) {
		if (!valid || s != stamp) {
			position = jp;
			stamp = s;
			missed = 0;
			valid = true;
		} else if (missed < REFERENCE_EXTRAPOLATION) {
			position += period * jv;
			++missed;
		}
	}
};

/* The same reference from the packet's age instead (--reference age): each
 * packet carries the time its joints were sampled (teleop_link.h), so the
 * other arm's position is advanced with its sent velocity over the packet's
 * actual age at this cycle's own sample, every cycle. The two 500 Hz loops
 * are not synchronized, so that age is anywhere in a 2 ms window (plus the
 * sender's bus read) and differs from run to run; without this the position
 * coupling sees it as a lag, a drag of kp * age per rad/s (1.5-2.5 Nm s/rad
 * on J1). The age used is limited, so a stalled sender is not extrapolated
 * far (the link times out after 20 ms as before). */
const double REFERENCE_AGE_LIMIT = 0.006;  // s: three cycles
template <typename V>
V agedReference(const V& jp, const V& jv, double age) {
	const double a = (age >= 0.0) ? std::min(age, REFERENCE_AGE_LIMIT) : 0.0;  // NaN or negative: none
	V out = jp + a * jv;
	return out;
}

/* The mode from the other arm's packets (teleop_link.h): each arm counts its
 * mode changes, and the larger counter wins; on a tie the more conservative
 * mode (the larger Mode value) does, so two keys pressed at once still end
 * with both arms in one mode. mode, epoch and adoptions are atomics. */
template <typename AtomicMode, typename AtomicCount>
void adoptMode(unsigned int theirMode, unsigned int theirEpoch, AtomicMode* mode, AtomicCount* epoch,
               AtomicCount* adoptions) {
	if (theirMode > 2u) {  // not a Mode
		return;
	}
	const unsigned int mine = epoch->load();
	if (theirEpoch > mine ||
	    (theirEpoch == mine && theirEpoch != 0 && static_cast<int>(theirMode) > static_cast<int>(mode->load()))) {
		mode->store(static_cast<int>(theirMode));
		epoch->store(theirEpoch);
		adoptions->fetch_add(1);
	}
}

/* --force-gain: the gain on ext_other in u4 (0.5 as written above), one value
 * for J1-J4 or four comma-separated ones (J3 is the first joint to go unstable
 * as the gain rises). The force channel is scaled by gain / FORCE_GAIN, u6's
 * ext_other terms too. */
const double FORCE_GAIN = 0.5;
struct ForceGains {
	double g[4];
	ForceGains() {
		for (int i = 0; i < 4; ++i) {
			g[i] = FORCE_GAIN;
		}
	}
};
inline bool parseForceGain(const std::string& s, ForceGains* gains) {
	double v[4];
	int n = 0;
	const char* p = s.c_str();
	for (;;) {
		char* end = NULL;
		const double x = std::strtod(p, &end);
		if (end == p || !(x >= 0.0 && x <= 1.0) || n == 4) {
			return false;
		}
		v[n++] = x;
		if (*end == '\0') {
			break;
		}
		if (*end != ',') {
			return false;
		}
		p = end + 1;
	}
	if (n != 1 && n != 4) {
		return false;
	}
	for (int i = 0; i < 4; ++i) {
		gains->g[i] = v[n == 1 ? 0 : i];
	}
	return true;
}
inline std::string forceGainText(const ForceGains& gains) {
	char buf[64];
	if (gains.g[1] == gains.g[0] && gains.g[2] == gains.g[0] && gains.g[3] == gains.g[0]) {
		std::snprintf(buf, sizeof(buf), "%.2f", gains.g[0]);
	} else {
		std::snprintf(buf, sizeof(buf), "%.2f,%.2f,%.2f,%.2f (J1-J4)", gains.g[0], gains.g[1], gains.g[2], gains.g[3]);
	}
	return buf;
}

/* The teleop's mode, switched during the teleop on both arms at once
 * (lib/teleop_mode.h: 'c' and 'p' in either program, or `wam 4c on|off`,
 * `wam pp on|off`), for an A/B on the same arms:
 *   FULL   the law as launched (--law; u4 for the demo)
 *   NO_4C  u2: the force channels off (position coupling + dynamics comp.)
 *   PP     u1: position-position, libbarrett's gravity compensation only
 * Each term fades to its new share over MODE_FADE_TIME, so the change is
 * felt, not a jolt, even in contact. The arms apply it as scale factors on the
 * law's inputs: the other arm's external torque (force), this arm's (own: u6's
 * local term), and dyn - grav (dyn). */
enum Mode { FULL = 0, NO_4C = 1, PP = 2 };
const double MODE_FADE_TIME = 1.0;  // s
struct ModeMix {
	double force, own, dyn;  // 1: as the law has it, 0: off
	ModeMix() : force(1.0), own(1.0), dyn(1.0) {}
	void fade(Mode mode, double period) {
		const double step = period / MODE_FADE_TIME;
		force = toward(force, mode == FULL ? 1.0 : 0.0, step);
		own = toward(own, mode == FULL ? 1.0 : 0.0, step);
		dyn = toward(dyn, mode == PP ? 0.0 : 1.0, step);
	}
	static double toward(double x, double target, double step) {
		return x < target ? std::min(target, x + step) : std::max(target, x - step);
	}
};
// The message on /teleop/mode.
inline const char* modeKey(Mode m) {
	return m == FULL ? "full" : m == NO_4C ? "no4c" : "pp";
}
inline bool parseMode(const std::string& s, Mode* m) {
	if (s == "full") { *m = FULL; return true; }
	if (s == "no4c") { *m = NO_4C; return true; }
	if (s == "pp") { *m = PP; return true; }
	return false;
}
inline std::string modeText(Mode m, Law launched) {
	if (m == FULL) {
		return std::string("full: ") + name(launched);
	}
	return m == NO_4C ? "4C off: u2 (position-position + dynamics compensation)"
	                  : "P-P: u1 (position-position, libbarrett's gravity compensation only)";
}

template <typename JT>
bool sendsExternalTorque(const JT& extOther) {
	for (int i = 0; i < 4; ++i) {
		if (extOther[i] != 0.0) {
			return true;
		}
	}
	return false;
}
/* otherLinked: the other arm says it is linked (teleop_link.h's flag; a
 * program without it: sendsExternalTorque()). */
template <typename JT>
JT forceChannel(const JT& extOther, bool linked, bool otherLinked, double period, double* t) {
	*t = (linked && otherLinked) ? std::min(*t + period, FORCE_FADE_TIME) : 0.0;
	return (*t / FORCE_FADE_TIME) * extOther;
}
template <typename JT>
JT forceChannel(const JT& extOther, bool linked, double period, double* t) {
	return forceChannel(extOther, linked, sendsExternalTorque(extOther), period, t);
}

template <typename JT>
JT compute(Law law, const JT& extOther, const JT& extOwn, const JT& grav, const JT& dyn) {
	JT u;
	u.setZero();
	for (int i = 0; i < 4; ++i) {
		switch (law) {
		case U1:
			break;
		case U2:
			u[i] = dyn[i] - grav[i];
			break;
		case U4:
			u[i] = -0.5 * extOther[i] + dyn[i] - grav[i];
			break;
		case U6:
			u[i] = -0.5 * extOther[i] - 0.15 * (extOther[i] + extOwn[i]) + dyn[i] - grav[i];
			break;
		}
	}
	return u;
}

}  // namespace teleop_law
