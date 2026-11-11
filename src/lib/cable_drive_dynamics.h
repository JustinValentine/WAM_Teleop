#pragma once

/* The cable-drive model (cable_drive_model.h) as a libbarrett system, in
 * place of LeaderDynamics / FollowerDynamics (--model cable). Two outputs:
 *
 *   modelOutput        the full model, J1-J4: what DynamicExternalTorque
 *                      subtracts from the commanded torque, so the external
 *                      torque estimate is the commanded torque minus every
 *                      modelled term (inertia at full scale, the friction
 *                      elements, the wrist's gravity relative to the
 *                      identification pose);
 *   feedforwardOutput  the law's "dyn" (teleop_law.h: u2/u4/u6 add dyn - grav):
 *                      gravity, Coriolis, viscous friction and the offsets of
 *                      the same model, plus the shares --ff-inertia of the
 *                      inertial torque and --ff-friction of the friction
 *                      elements. The defaults keep the old feedforward's terms
 *                      (no Coulomb friction; inertia as before on each arm).
 *
 * The estimate and the feedforward are separate on purpose: compensating
 * inertia from a filtered acceleration, or all of the friction, feeds the
 * arm's own motion back; the estimate needs every term. */

#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/units.h>

#include "arm_gravity.h"
#include "cable_drive_model.h"

template <size_t DOF>
class CableDriveDynamics : public barrett::systems::System {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
	Input<jp_type> jpInput;
	Input<jv_type> jvInput;
	Input<ja_type> jaInput;
	Input<ja_type> jaEstimatorInput;  // the matched estimate's acceleration (may be unconnected)
	Output<jt_type> modelOutput;
	Output<jt_type> feedforwardOutput;
	// For the matched estimate (estimator_systems.h): the model without its
	// inertial torque, and the inertial torque of jaEstimatorInput.
	Output<jt_type> staticOutput;
	Output<jt_type> estimatorInertiaOutput;

	/* gravity: the arm's calibrated gravity model for the wrist correction
	 * (NULL: none); wristFirst: the first joint after J4 that the model
	 * holds at 0 (4: P2 J5-J7; 4 for the wam8g too: passive J5 and J6-J8). */
	CableDriveDynamics(barrett::systems::ExecutionManager* em, const cable_model::Params& params,
	                   const arm_gravity::Model* gravity, int wristFirst, double ffInertia, double ffFriction,
	                   const std::string& sysName = "CableDriveDynamics")
		: System(sysName), jpInput(this), jvInput(this), jaInput(this), jaEstimatorInput(this),
		  modelOutput(this, &modelValue), feedforwardOutput(this, &ffValue),
		  staticOutput(this, &staticValue), estimatorInertiaOutput(this, &inertiaValue), model(params),
		  gravity(gravity), wristFirst(wristFirst), ffInertia(ffInertia), ffFriction(ffFriction) {
		for (int i = 0; i < arm_gravity::MAX_DOF; ++i) {
			wristRef[i] = 0.0;
		}
		full.setZero();
		ff.setZero();
		staticTorque.setZero();
		estimatorInertia.setZero();
		terms.gravityCoriolis.setZero();
		terms.inertia.setZero();
		terms.hysteresis.setZero();
		terms.viscous.setZero();
		terms.offset.setZero();
		terms.wrist.setZero();
		if (em != NULL) {
			em->startManaging(*this);
		}
	}

	virtual ~CableDriveDynamics() { this->mandatoryCleanUp(); }

	virtual bool inputsValid() { return jpInput.valueDefined() && jvInput.valueDefined() && jaInput.valueDefined(); }

	// The feedforward's shares (0 to 1) of the inertial torque and of the
	// friction elements.
	void setFeedforward(double inertiaShare, double frictionShare) {
		BARRETT_SCOPED_LOCK(this->getEmMutex());
		ffInertia = inertiaShare;
		ffFriction = frictionShare;
	}
	double getFeedforwardInertia() const { return ffInertia; }
	double getFeedforwardFriction() const { return ffFriction; }

	// The last cycle's terms (J1-J4, Nm), for logging.
	struct Terms {
		cable_model::Vec4 gravityCoriolis, inertia, hysteresis, viscous, offset, wrist;
	};
	Terms getTerms() const {
		BARRETT_SCOPED_LOCK(this->getEmMutex());
		return terms;
	}

protected:
	typename Output<jt_type>::Value* modelValue;
	typename Output<jt_type>::Value* ffValue;
	typename Output<jt_type>::Value* staticValue;
	typename Output<jt_type>::Value* inertiaValue;

	virtual void operate() {
		const jp_type& jp = jpInput.getValue();
		const jv_type& jv = jvInput.getValue();
		const ja_type& ja = jaInput.getValue();
		cable_model::Vec4 q, dq, ddq;
		for (int i = 0; i < 4; ++i) {
			q[i] = jp[i];
			dq[i] = jv[i];
			ddq[i] = ja[i];
		}
		model.update(q, dq, ddq);
		cable_model::Vec4 dg = cable_model::Vec4::Zero();
		if (gravity != NULL && gravity->getDof() == (int) DOF) {
			dg = gravity->wristCorrection(jp, wristFirst, wristRef);
		}
		const cable_model::Vec4 m = model.full() + dg;
		const cable_model::Vec4 f = model.gravityCoriolis() + ffInertia * model.inertia() +
		                            ffFriction * model.hysteresis() + model.viscousFriction() + model.offset() + dg;
		const cable_model::Vec4 n = model.withoutInertia() + dg;
		cable_model::Vec4 mEst = cable_model::Vec4::Zero();
		if (jaEstimatorInput.valueDefined()) {
			const ja_type& jaEst = jaEstimatorInput.getValue();
			cable_model::Vec4 a;
			for (int i = 0; i < 4; ++i) {
				a[i] = jaEst[i];
			}
			mEst = model.inertiaFor(a);
		}
		full.setZero();
		ff.setZero();
		for (int i = 0; i < 4; ++i) {
			full[i] = m[i];
			ff[i] = f[i];
			staticTorque[i] = n[i];
			estimatorInertia[i] = mEst[i];
		}
		terms.gravityCoriolis = model.gravityCoriolis();
		terms.inertia = model.inertia();
		terms.hysteresis = model.hysteresis();
		terms.viscous = model.viscousFriction();
		terms.offset = model.offset();
		terms.wrist = dg;
		modelValue->setData(&full);
		ffValue->setData(&ff);
		staticValue->setData(&staticTorque);
		inertiaValue->setData(&estimatorInertia);
	}

private:
	cable_model::Model model;
	const arm_gravity::Model* gravity;
	const int wristFirst;
	double wristRef[arm_gravity::MAX_DOF];
	double ffInertia, ffFriction;
	jt_type full, ff, staticTorque, estimatorInertia;
	Terms terms;

	DISALLOW_COPY_AND_ASSIGN(CableDriveDynamics);

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
