/* lib/dyn_feel_system.h in a libbarrett control graph without a robot: a
 * manual execution manager runs a four-input torque sum (as Wam's jtSum, where
 * undefined counts as zero) fed by DynFeel, whose inputs are three exposed
 * outputs standing for wam.jpOutput, wam.jvOutput and wam.gravity.output.
 *   - nothing at the start; J5-J8 never get anything;
 *   - the model on: the gravity term ramps to the model's gravity minus the
 *     library's, clamped;
 *   - an undefined or non-finite input: zero in that same cycle, latched;
 *   - the display's heartbeat missing: the model switches off;
 *   - a trip is acknowledged by the first key and stop() is for good.
 * Build against the installed library (as wam8g_loopback_tests), or with the
 * library's three graph sources and no robot code:
 *   g++ -std=c++14 -I<libbarrett-canopen>/include -I/usr/include/eigen3 -I<wam_teleop>/src/lib -Isrc/lib \
 *       test/test_dyn_feel_system.cpp <libbarrett-canopen>/src/systems/{system,execution_manager}.cpp \
 *       <libbarrett-canopen>/src/thread/null_mutex.cpp
 * Run: a.out <beta30.dat> */

#include <cmath>
#include <cstdio>
#include <string>

#include <barrett/systems/abstract/system.h>
#include <barrett/systems/exposed_output.h>
#include <barrett/systems/helpers.h>
#include <barrett/systems/manual_execution_manager.h>
#include <barrett/systems/summer.h>
#include <barrett/units.h>

#include "dyn_feel_system.h"

using namespace barrett;

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

typedef DynFeel::jp_type jp_type;
typedef DynFeel::jv_type jv_type;
typedef DynFeel::jt_type jt_type;

static double fakeNow = 100.0;
static double fakeClock() { return fakeNow; }

// What reaches the motors: the torque sum's output, read each cycle.
class Probe : public systems::System {
public:
	Input<jt_type> input;
	jt_type last;
	bool defined;
	Probe() : systems::System("Probe"), input(this), defined(false) { last.setZero(); }
	virtual ~Probe() { this->mandatoryCleanUp(); }

protected:
	virtual bool inputsValid() { return true; }
	virtual void operate() {
		defined = input.valueDefined();
		if (defined) {
			last = input.getValue();
		}
	}
};

struct Rig {
	systems::ManualExecutionManager mem;
	systems::ExposedOutput<jp_type> jp;
	systems::ExposedOutput<jv_type> jv;
	systems::ExposedOutput<jt_type> grav;
	systems::Summer<jt_type, 4> sum;
	Probe probe;
	DynFeel feel;
	const double T;

	Rig(const dyn_feel::Params& p, const dyn_feel::Limits& lim)
		: mem(0.002), sum(true), feel(p, lim, 0.05, NULL, 0.002, 1.0, &fakeClock), T(0.002) {
		systems::connect(jp.output, feel.jpInput);
		systems::connect(jv.output, feel.jvInput);
		systems::connect(grav.output, feel.gravityInput);
		systems::connect(feel.output, sum.getInput(0));  // wam.input
		systems::connect(sum.output, probe.input);
		mem.startManaging(probe);
	}
	void run(int cycles, bool heartbeat = true) {
		for (int n = 0; n < cycles; ++n) {
			fakeNow += T;
			if (heartbeat) {
				feel.heartbeat();
			}
			mem.runExecutionCycle();
		}
	}
};

int main(int argc, char** argv) {
	if (argc != 2) {
		printf("usage: %s <beta30.dat>\n", argv[0]);
		return 2;
	}
	dyn_feel::Params p;
	std::string error;
	if (!dyn_feel::loadParams(argv[1], &p, &error)) {
		printf("%s\n", error.c_str());
		return 2;
	}
	dyn_feel::Limits lim;
	const double q4[4] = {0.3, -1.2, 0.4, 1.6};
	jp_type q(0.0);
	for (int j = 0; j < 4; ++j) q[j] = q4[j];
	q[5] = 0.2;
	jv_type still(0.0);
	double model[4];
	dyn_feel::modelGravity(p, q4, model);
	jt_type lib(0.0);
	for (int j = 0; j < 4; ++j) lib[j] = model[j] - (j == 1 ? 9.0 : 0.7 * (j + 1));  // J2 far off: clamped
	lib[5] = 0.4;

	// nothing at the start
	{
		Rig r(p, lim);
		r.run(5);
		CHECK(!r.probe.defined || r.probe.last.isZero(0.0), "torque before any input is defined");
		CHECK(r.feel.snapshot().mixer.trip == dyn_feel::TRIP_INVALID, "missing inputs are a trip");
		r.jp.setValue(q);
		r.jv.setValue(still);
		r.grav.setValue(lib);
		r.feel.toggleModel();  // acknowledges the start's trip
		CHECK(r.feel.snapshot().cycles == 5, "five cycles ran (%lu)", r.feel.snapshot().cycles);
		r.run(500);
		CHECK(r.probe.defined && r.probe.last.isZero(0.0), "torque with nothing asked");
		CHECK(r.feel.snapshot().valid && r.feel.snapshot().mixer.trip == dyn_feel::NO_TRIP, "valid, no trip");
		CHECK(std::fabs(r.feel.snapshot().libGravity[5] - 0.4) < 1e-15, "the library's gravity is shown");
		for (int j = 0; j < 4; ++j) {
			CHECK(std::fabs(r.feel.snapshot().terms.gravity[j] - model[j]) < 1e-12, "the gravity term J%d", j + 1);
		}

		// the gravity term on, the model on, level 100 %
		r.feel.toggleTerm(dyn_feel::GRAVITY);
		r.feel.setLevel(1.0);
		r.run(500);
		CHECK(r.probe.last.isZero(0.0), "a term and a level without the model");
		r.feel.toggleModel();
		double prev = 0.0, worstStep = 0.0;
		for (int n = 0; n < 2000; ++n) {
			r.run(1);
			worstStep = std::max(worstStep, std::fabs(r.probe.last[0] - prev));
			prev = r.probe.last[0];
		}
		CHECK(worstStep < 2.0 * 0.7 * r.T / lim.rampTime + 1e-12, "J1 stepped %.5f Nm in a cycle", worstStep);
		CHECK(std::fabs(r.probe.last[0] - 0.7) < 1e-9, "J1 %.4f", r.probe.last[0]);
		CHECK(std::fabs(r.probe.last[1] - lim.clamp[1]) < 1e-12, "J2 clamped %.4f", r.probe.last[1]);
		CHECK(r.feel.snapshot().mixer.clamped[1] && !r.feel.snapshot().mixer.clamped[0], "clamp flags");
		CHECK(std::fabs(r.probe.last[2] - std::min(2.1, lim.clamp[2])) < 1e-9, "J3 %.4f", r.probe.last[2]);
		for (int j = 4; j < 8; ++j) CHECK(r.probe.last[j] == 0.0, "J%d gets nothing", j + 1);

		// the library's gravity goes undefined: nothing in that same cycle, and it stays off
		r.grav.setValueUndefined();
		r.run(1);
		CHECK(r.probe.last.isZero(0.0), "torque with an undefined input");
		CHECK(r.feel.snapshot().mixer.trip == dyn_feel::TRIP_INVALID && !r.feel.snapshot().valid, "trip");
		r.grav.setValue(lib);
		r.run(2000);
		CHECK(r.probe.last.isZero(0.0) && !r.feel.snapshot().targets.model, "it stays off when the input returns");
		r.feel.toggleModel();  // acknowledges only
		r.run(200);
		CHECK(r.probe.last.isZero(0.0) && r.feel.snapshot().mixer.trip == dyn_feel::NO_TRIP, "acknowledged, still off");
		r.feel.toggleModel();
		r.run(2000);
		CHECK(std::fabs(r.probe.last[0] - 0.7) < 1e-9, "on again: %.4f", r.probe.last[0]);

		// a position that is not a number
		jp_type bad = q;
		bad[6] = std::nan("");
		r.jp.setValue(bad);
		r.run(1);
		CHECK(r.probe.last.isZero(0.0) && r.feel.snapshot().mixer.trip == dyn_feel::TRIP_INVALID, "a nan position");
		r.jp.setValue(q);
		r.feel.toggleModel();
		r.feel.toggleModel();
		r.run(2000);
		CHECK(std::fabs(r.probe.last[0] - 0.7) < 1e-9, "on again");

		// the display stops: off within the stall time plus the trip's ramp
		r.run(400, false);  // 0.8 s without a heartbeat: still on
		CHECK(std::fabs(r.probe.last[0] - 0.7) < 1e-9, "0.8 s without the display");
		r.run(250, false);  // 1.3 s
		CHECK(r.feel.snapshot().mixer.trip == dyn_feel::TRIP_UI_STALL, "the stall trips");
		CHECK(r.probe.last.isZero(0.0), "off 0.3 s after the stall time (%.4f)", r.probe.last[0]);
		r.feel.toggleModel();
		r.feel.toggleModel();
		r.run(2000);
		CHECK(std::fabs(r.probe.last[0] - 0.7) < 1e-9, "on again");

		// a joint too fast: the model switches off
		jv_type fast(0.0);
		fast[2] = lim.speedTrip + 0.05;
		r.jv.setValue(fast);
		r.run(1);
		CHECK(r.feel.snapshot().mixer.trip == dyn_feel::TRIP_SPEED && r.feel.snapshot().mixer.tripJoint == 2, "speed trip");
		CHECK(r.probe.last[2] == 0.0, "nothing on the fast joint at once");
		r.jv.setValue(still);
		r.run(200);
		CHECK(r.probe.last.isZero(0.0), "off after the speed trip");

		// stop() is for good
		r.feel.toggleModel();
		r.feel.toggleModel();
		r.run(2000);
		CHECK(std::fabs(r.probe.last[0] - 0.7) < 1e-9, "on again");
		r.feel.stop();
		r.run(200);
		CHECK(r.probe.last.isZero(0.0), "stopped");
		r.feel.toggleModel();
		r.feel.toggleModel();
		r.feel.setModel(true);
		r.run(2000);
		CHECK(r.probe.last.isZero(0.0) && r.feel.snapshot().stopped, "stopped for good");
	}
	printf("%d checks, %d failed\n", checks, failures);
	return failures == 0 ? 0 : 1;
}
