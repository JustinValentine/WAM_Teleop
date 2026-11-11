#pragma once

/* estimator_core.h as libbarrett systems, for leader_wam8g.cpp and
 * follower.cpp:
 *
 *   RegressorDynamics       the fitted rigid-body model (--model legacy), in
 *                           place of LeaderDynamics / FollowerDynamics: the
 *                           same feedforward output, without allocating, plus
 *                           the two outputs the matched estimate needs;
 *   MatchedAcceleration     the estimator's acceleration from the velocity;
 *   MatchedExternalTorque   the matched estimate, in place of
 *                           DynamicExternalTorque (--estimator matched). */

#include <string>

#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/units.h>

#include "estimator_core.h"

template <size_t DOF>
class RegressorDynamics : public barrett::systems::System {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
	Input<jp_type> jpInput;
	Input<jv_type> jvInput;
	Input<ja_type> jaInput;           // the feedforward's acceleration (may be unconnected)
	Input<ja_type> jaEstimatorInput;  // the matched estimate's acceleration (may be unconnected)

	/* W(q, dq, accelScale * ja) beta: what LeaderDynamics (accelScale 0: its
	 * acceleration input was zero) and FollowerDynamics (0.25) put out; the
	 * law's "dyn" and the legacy estimate's model. */
	Output<jt_type> dynamicsFeedFWD;
	Output<jt_type> staticOutput;            // W(q, dq, 0) beta
	Output<jt_type> estimatorInertiaOutput;  // M(q) jaEstimator, at full scale

	template <typename Beta>
	RegressorDynamics(const Beta& beta, double accelScale, const std::string& sysName = "RegressorDynamics")
		: System(sysName), jpInput(this), jvInput(this), jaInput(this), jaEstimatorInput(this),
		  dynamicsFeedFWD(this, &feedforwardValue), staticOutput(this, &staticValue),
		  estimatorInertiaOutput(this, &inertiaValue), accelScale(accelScale) {
		model.setBeta(beta);
		feedforward.setZero();
		staticTorque.setZero();
		inertia.setZero();
	}
	virtual ~RegressorDynamics() { this->mandatoryCleanUp(); }

	virtual bool inputsValid() { return jpInput.valueDefined() && jvInput.valueDefined(); }

protected:
	typename Output<jt_type>::Value* feedforwardValue;
	typename Output<jt_type>::Value* staticValue;
	typename Output<jt_type>::Value* inertiaValue;

	virtual void operate() {
		const jp_type& jp = jpInput.getValue();
		const jv_type& jv = jvInput.getValue();
		double q[4], dq[4], a[4], m[4];
		for (int i = 0; i < 4; ++i) {
			q[i] = jp[i];
			dq[i] = jv[i];
		}
		model.update(q, dq);
		for (int i = 0; i < 4; ++i) {
			staticTorque[i] = model.staticTorque[i];
			feedforward[i] = model.staticTorque[i];
		}
		if (accelScale != 0.0 && jaInput.valueDefined()) {
			const ja_type& ja = jaInput.getValue();
			for (int i = 0; i < 4; ++i) a[i] = accelScale * ja[i];
			model.inertia(q, dq, a, m);
			for (int i = 0; i < 4; ++i) feedforward[i] += m[i];
		}
		if (jaEstimatorInput.valueDefined()) {
			const ja_type& ja = jaEstimatorInput.getValue();
			for (int i = 0; i < 4; ++i) a[i] = ja[i];
			model.inertia(q, dq, a, m);
			for (int i = 0; i < 4; ++i) inertia[i] = m[i];
		}
		feedforwardValue->setData(&feedforward);
		staticValue->setData(&staticTorque);
		inertiaValue->setData(&inertia);
	}

private:
	estimator::RegressorModel model;
	const double accelScale;
	jt_type feedforward, staticTorque, inertia;

	DISALLOW_COPY_AND_ASSIGN(RegressorDynamics);
};

template <size_t DOF>
class MatchedAcceleration : public barrett::systems::System {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
	Input<jv_type> jvInput;
	Output<ja_type> output;

	MatchedAcceleration(double hpOmega, double lpOmega, double period,
	                    const std::string& sysName = "MatchedAcceleration")
		: System(sysName), jvInput(this), output(this, &outputValue) {
		chain.configure(hpOmega, lpOmega, period);
		ja.setZero();
	}
	virtual ~MatchedAcceleration() { this->mandatoryCleanUp(); }

protected:
	typename Output<ja_type>::Value* outputValue;

	virtual void operate() {
		const jv_type& jv = jvInput.getValue();
		double v[4];
		for (int i = 0; i < 4; ++i) v[i] = jv[i];
		const double* a = chain.step(v);
		for (int i = 0; i < 4; ++i) ja[i] = a[i];
		outputValue->setData(&ja);
	}

private:
	estimator::Acceleration chain;
	ja_type ja;

	DISALLOW_COPY_AND_ASSIGN(MatchedAcceleration);
};

template <size_t DOF>
class MatchedExternalTorque : public barrett::systems::System {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
	Input<jt_type> wamTorqueSumIn;   // the commanded torque (as DynamicExternalTorque's)
	Input<jt_type> staticIn;         // the model without its inertial term
	Input<jt_type> inertiaIn;        // M(q) times MatchedAcceleration's output
	Output<jt_type> wamExternalTorqueOut;  // J1-J4; zero on the other joints

	MatchedExternalTorque(double hpOmega, double lpOmega, double period,
	                      const std::string& sysName = "MatchedExternalTorque")
		: System(sysName), wamTorqueSumIn(this), staticIn(this), inertiaIn(this),
		  wamExternalTorqueOut(this, &outputValue) {
		filter.configure(hpOmega, lpOmega, period);
		ext.setZero();
	}
	virtual ~MatchedExternalTorque() { this->mandatoryCleanUp(); }

protected:
	typename Output<jt_type>::Value* outputValue;

	virtual void operate() {
		const jt_type& tau = wamTorqueSumIn.getValue();
		const jt_type& n = staticIn.getValue();
		const jt_type& m = inertiaIn.getValue();
		double t[4], s[4], i4[4], e[4];
		for (int i = 0; i < 4; ++i) {
			t[i] = tau[i];
			s[i] = n[i];
			i4[i] = m[i];
		}
		filter.step(t, s, i4, e);
		for (int i = 0; i < 4; ++i) ext[i] = e[i];
		outputValue->setData(&ext);
	}

private:
	estimator::MatchedTorque filter;
	jt_type ext;

	DISALLOW_COPY_AND_ASSIGN(MatchedExternalTorque);
};
