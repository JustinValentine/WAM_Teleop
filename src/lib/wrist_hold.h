#pragma once

/* Holds joints first..DOF-1 at fixed angles with a PD torque, clamped per
 * joint; zero on the other joints. dynamics_excitation uses it for the
 * wam8g's wrist (J6-J8), which has no velocity-control gains, while J1-J4
 * track the excitation velocity. */

#include <algorithm>
#include <string>

#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/units.h>

template <size_t DOF>
class WristHold : public barrett::systems::System {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
	Input<jp_type> jpInput;
	Input<jv_type> jvInput;
	Output<jt_type> output;

	WristHold(size_t first, const jp_type& hold, const v_type& kp, const v_type& kd,
			const jt_type& limit, const std::string& sysName = "WristHold")
		: System(sysName), jpInput(this), jvInput(this), output(this, &outputValue),
		  first(first), hold(hold), kp(kp), kd(kd), limit(limit), torque(0.0) {}
	virtual ~WristHold() { this->mandatoryCleanUp(); }

protected:
	virtual void operate() {
		const jp_type& jp = jpInput.getValue();
		const jv_type& jv = jvInput.getValue();
		torque.setZero();
		for (size_t i = first; i < DOF; ++i) {
			const double t = kp[i] * (hold[i] - jp[i]) - kd[i] * jv[i];
			torque[i] = std::max(-limit[i], std::min(limit[i], t));
		}
		outputValue->setData(&torque);
	}

	typename Output<jt_type>::Value* outputValue;
	const size_t first;
	const jp_type hold;
	const v_type kp, kd;
	const jt_type limit;
	jt_type torque;

private:
	DISALLOW_COPY_AND_ASSIGN(WristHold);
};
