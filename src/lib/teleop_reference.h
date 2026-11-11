#pragma once

/* The other arm's position, the position controller's reference, as a system
 * of its own that the arm system (Follower, LeaderWam8g) pulls first each
 * cycle. The arm system's external torque estimate sums the position
 * controller's torque, so pulling that input runs the controller; were the
 * reference an output of the arm system itself, the controller would read it
 * before the arm system had received this cycle's packet and track the
 * previous cycle's, 2 ms late on each side. Owner::receive() reads the newest
 * packet and returns the reference, or NULL to keep the last one. */

#include <string>

#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/system.h>

template <typename Owner, typename JP>
class TeleopReference : public barrett::systems::System {
public:
	Output<JP> output;

	TeleopReference(Owner* owner, const std::string& sysName)
		: System(sysName), output(this, &outputValue), owner(owner) {}
	virtual ~TeleopReference() { this->mandatoryCleanUp(); }

	// Disconnects it: the owner calls this first when it goes, so that the
	// controller cannot pull a reference whose owner is gone.
	void detach() { this->mandatoryCleanUp(); }

protected:
	typename Output<JP>::Value* outputValue;

	virtual void operate() {
		const JP* reference = owner->receive();
		if (reference != NULL) {
			outputValue->setData(reference);
		}
	}

private:
	Owner* owner;

	DISALLOW_COPY_AND_ASSIGN(TeleopReference);
};
