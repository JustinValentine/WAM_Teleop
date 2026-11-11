#pragma once

/* dyn_feel_core.h as a libbarrett system, for wam8g_dyn_feel.cpp: the model's
 * terms from the wam8g's measured joints, and what the keys ask for, mixed
 * into a torque for wam.input (J1-J4; zero on J5-J8).
 *
 * It is not managed by the execution manager itself: it runs when the WAM's
 * torque sum asks for its output, once a cycle. operate() allocates nothing
 * and does no I/O. A missing or non-finite input gives a zero torque and a
 * latched trip, never a skipped cycle that leaves the last torque standing.
 * The keys' methods and snapshot() are for another thread (the display's);
 * they take the execution manager's lock. */

#include <cmath>
#include <string>

#include <barrett/detail/ca_macro.h>
#include <barrett/os.h>
#include <barrett/systems/abstract/single_io.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/units.h>

#include "arm_gravity.h"
#include "dyn_feel_core.h"
#include "estimator_core.h"

class DynFeel : public barrett::systems::System,
                public barrett::systems::SingleOutput<barrett::units::JointTorques<8>::type> {
public:
	typedef barrett::units::JointPositions<8>::type jp_type;
	typedef barrett::units::JointVelocities<8>::type jv_type;
	typedef barrett::units::JointTorques<8>::type jt_type;
	typedef double (*Clock)();  // seconds, monotonic

	Input<jp_type> jpInput;
	Input<jv_type> jvInput;
	Input<jt_type> gravityInput;  // libbarrett's gravity torque (wam.gravity.output)

	struct Snapshot {
		dyn_feel::Terms terms;
		dyn_feel::MixerState mixer;
		dyn_feel::Targets targets;
		double libGravity[8];
		double wrist[4];
		unsigned long cycles;
		bool valid, stopped;
		Snapshot() : cycles(0), valid(false), stopped(false) {
			for (int j = 0; j < 8; ++j) libGravity[j] = 0.0;
			for (int j = 0; j < 4; ++j) wrist[j] = 0.0;
		}
	};

	/* gravity: the arm's calibrated gravity model, for the wrist's gravity on
	 * J1-J4 relative to the wrist at 0, where the model was identified (NULL:
	 * none; the model's gravity then stands for whatever pose the wrist has).
	 * uiStall: seconds without heartbeat() after which a model that is on
	 * switches off. */
	DynFeel(const dyn_feel::Params& params, const dyn_feel::Limits& limits, double coulombV0,
	        const arm_gravity::Model* gravity, double period, double uiStall,
	        Clock clock = &barrett::highResolutionSystemTime, const std::string& sysName = "DynFeel")
		: barrett::systems::System(sysName), barrett::systems::SingleOutput<jt_type>(this), jpInput(this),
		  jvInput(this), gravityInput(this), evaluator(params, coulombV0), mixer(limits), gravity(gravity),
		  period(period), uiStall(uiStall), clock(clock), lastHeartbeat(0.0) {
		acceleration.configure(25.0, 50.0, period);  // as leader_wam8g.cpp's hp1 and jaFilter
		for (int i = 0; i < arm_gravity::MAX_DOF; ++i) wristRef[i] = 0.0;
		torque.setZero();
		this->outputValue->setData(&torque);
	}
	virtual ~DynFeel() { this->mandatoryCleanUp(); }

	/* The model on/off. After a trip the first call only acknowledges it (the
	 * operator has seen it); the next one switches the model on. */
	void toggleModel() {
		BARRETT_SCOPED_LOCK(this->getEmMutex());
		if (snap.stopped) {
			return;
		}
		if (mixer.state().trip != dyn_feel::NO_TRIP) {
			mixer.acknowledge();
			targets.model = false;
			return;
		}
		targets.model = !targets.model;
		lastHeartbeat = clock();
	}
	// Off by its ramp (on: only when nothing has tripped).
	void setModel(bool on) {
		BARRETT_SCOPED_LOCK(this->getEmMutex());
		targets.model = on && !snap.stopped && mixer.state().trip == dyn_feel::NO_TRIP;
		lastHeartbeat = clock();
	}
	void toggleTerm(int term) {
		BARRETT_SCOPED_LOCK(this->getEmMutex());
		if (term >= 0 && term < dyn_feel::N_TERMS) {
			targets.term[term] = !targets.term[term];
		}
	}
	void changeLevel(double by) {
		BARRETT_SCOPED_LOCK(this->getEmMutex());
		targets.level = tenth(targets.level + by);
	}
	void setLevel(double level) {
		BARRETT_SCOPED_LOCK(this->getEmMutex());
		targets.level = tenth(level);
	}
	void changeFrictionShare(double by) {
		BARRETT_SCOPED_LOCK(this->getEmMutex());
		targets.frictionShare = tenth(targets.frictionShare + by);
	}
	// The display loop is alive.
	void heartbeat() {
		BARRETT_SCOPED_LOCK(this->getEmMutex());
		lastHeartbeat = clock();
	}
	// For good: the level falls to nothing and stays there.
	void stop() {
		BARRETT_SCOPED_LOCK(this->getEmMutex());
		snap.stopped = true;
		targets.model = false;
		mixer.trip(dyn_feel::TRIP_STOPPED);
	}
	Snapshot snapshot() const {
		BARRETT_SCOPED_LOCK(this->getEmMutex());
		return snap;
	}

protected:
	virtual bool inputsValid() { return true; }

	virtual void operate() {
		double q[4] = {0.0, 0.0, 0.0, 0.0}, dq[4] = {0.0, 0.0, 0.0, 0.0}, lib[4] = {0.0, 0.0, 0.0, 0.0};
		double wrist[4] = {0.0, 0.0, 0.0, 0.0};
		bool valid = jpInput.valueDefined() && jvInput.valueDefined() && gravityInput.valueDefined();
		if (valid) {
			const jp_type& jp = jpInput.getValue();
			const jv_type& jv = jvInput.getValue();
			const jt_type& g = gravityInput.getValue();
			valid = jp.allFinite() && jv.allFinite() && g.allFinite();
			if (valid) {
				for (int j = 0; j < 4; ++j) {
					q[j] = jp[j];
					dq[j] = jv[j];
					lib[j] = g[j];
				}
				for (int j = 0; j < 8; ++j) {
					snap.libGravity[j] = g[j];
				}
				const double* ddq = acceleration.step(dq);
				evaluator.update(q, dq, ddq);
				if (gravity != NULL) {
					const cable_model::Vec4 dg = gravity->wristCorrection(jp, 4, wristRef);
					for (int j = 0; j < 4; ++j) {
						wrist[j] = dg[j];
					}
				}
			}
		}
		if (targets.model && clock() - lastHeartbeat > uiStall) {
			mixer.trip(dyn_feel::TRIP_UI_STALL);
		}
		const double* added = mixer.step(targets, evaluator.terms(), lib, wrist, q, dq, valid, period);
		if (mixer.state().trip != dyn_feel::NO_TRIP) {
			targets.model = false;  // on again only by the operator's key
		}
		torque.setZero();
		for (int j = 0; j < 4; ++j) {
			torque[j] = added[j];
		}
		snap.terms = evaluator.terms();
		snap.mixer = mixer.state();
		snap.targets = targets;
		for (int j = 0; j < 4; ++j) {
			snap.wrist[j] = wrist[j];
		}
		snap.valid = valid;
		++snap.cycles;
		this->outputValue->setData(&torque);
	}

private:
	static double tenth(double x) {
		const double r = std::floor(x * 10.0 + 0.5) / 10.0;
		return r < 0.0 ? 0.0 : (r > 1.0 ? 1.0 : r);
	}

	dyn_feel::Evaluator evaluator;
	dyn_feel::Mixer mixer;
	estimator::Acceleration acceleration;  // for the displayed inertial term only
	const arm_gravity::Model* const gravity;
	const double period, uiStall;
	const Clock clock;
	double wristRef[arm_gravity::MAX_DOF];
	dyn_feel::Targets targets;
	double lastHeartbeat;
	jt_type torque;
	Snapshot snap;

	DISALLOW_COPY_AND_ASSIGN(DynFeel);

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
