#pragma once

/* The follower's force/torque sensor as a system in its control loop
 * (lib/ft_core.h has the arithmetic and the reasons): each cycle it takes the
 * newest reading, removes the zero and the gripper's weight, and turns the
 * rest, the wrench the surroundings apply to the arm, into the joint torques
 * that wrench causes (Jacobian transpose at libbarrett's tool frame).
 *
 * FtSource is where the readings come from: the Barrett sensor on the
 * follower's bus (lib/ft_barrett.h), or a stand-in in the tests. */

#include <atomic>
#include <string>

#include <gsl/gsl_matrix.h>

#include <barrett/detail/ca_macro.h>
#include <barrett/math/kinematics.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/kinematics_base.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/units.h>

#include "ft_core.h"
#include "teleop_link.h"

struct FtSource {
	virtual ~FtSource() {}
	/* Once per control cycle, from the control thread; it must not wait. True
	 * when force (N) and torque (Nm), in the sensor's axes, are a new reading. */
	virtual bool poll(double* force, double* torque) = 0;
};

template <size_t DOF>
class FtTorques : public barrett::systems::System, public barrett::systems::KinematicsInput<DOF> {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
	/* The joint torques the surroundings cause through the gripper, times
	 * use(): zero until the sensor is zeroed and answering. */
	Output<jt_type> output;

	FtTorques(FtSource* source, const ft_core::Tool& tool, const ft_core::Limits& limits, double period,
	          const std::string& sysName = "FtTorques")
		: System(sysName), barrett::systems::KinematicsInput<DOF>(this), output(this, &outputValue),
		  source(source), period(period), zeroRequest(0), zeroCount(0), readings(0), worstPoll(0.0) {
		processor.tool = tool;
		processor.limits = limits;
		data.setZero();
	}
	virtual ~FtTorques() { this->mandatoryCleanUp(); }

	/* Take the zero over the next `seconds` of readings: nothing may touch
	 * the gripper meanwhile. From any thread; zerosDone() counts up when it
	 * is done. */
	void startZero(double seconds) {
		const int n = static_cast<int>(seconds / period + 0.5);
		zeroRequest = n > 0 ? n : 1;
	}
	int zerosDone() const { return zeroCount; }
	unsigned long readingCount() const { return readings; }
	/* The longest the loop has spent reading the sensor in one cycle, s. */
	double worstPollSeconds() const { return worstPoll; }
	const ft_core::Report& report() const { return rep; }

protected:
	typename Output<jt_type>::Value* outputValue;

	virtual void operate() {
		const barrett::math::Kinematics<DOF>& kin = this->kinInput.getValue();
		const int request = zeroRequest.exchange(0);
		if (request > 0) {
			processor.startZero(request);
		}

		double f[3] = {0.0, 0.0, 0.0}, n[3] = {0.0, 0.0, 0.0};
		const double before = teleop_link::now();
		const bool fresh = source->poll(f, n);
		const double took = teleop_link::now() - before;
		if (took > worstPoll) {
			worstPoll = took;
		}
		if (fresh) {
			readings = readings + 1;
		}

		double worldFromTool[9];
		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {
				worldFromTool[3 * i + j] = gsl_matrix_get(kin.impl->tool->rot_to_world, i, j);
			}
		}
		processor.step(fresh, f, n, worldFromTool, period);
		zeroCount = processor.zero.count;

		// The wrench in the world's axes, then J^T: the tool Jacobian's linear
		// rows on the force, its angular rows on the torque.
		double fw[3], nw[3];
		ft_core::rotate(worldFromTool, processor.force, fw);
		ft_core::rotate(worldFromTool, processor.torque, nw);
		const double use = processor.use();
		for (size_t j = 0; j < DOF; ++j) {
			double tau = 0.0;
			for (int i = 0; i < 3; ++i) {
				tau += gsl_matrix_get(kin.impl->tool_jacobian_linear, i, j) * fw[i] +
				       gsl_matrix_get(kin.impl->tool_jacobian_angular, i, j) * nw[i];
			}
			if (j < 7) {
				rep.jointTorque[j] = tau;
			}
			data[j] = use * tau;
		}
		for (int i = 0; i < 3; ++i) {
			rep.rawForce[i] = processor.rawForce[i];
			rep.rawTorque[i] = processor.rawTorque[i];
			rep.gravity[i] = processor.gravity[i];
			rep.force[i] = processor.force[i];
			rep.torque[i] = processor.torque[i];
		}
		rep.use = use;
		rep.fresh = fresh ? 1.0 : 0.0;
		rep.zeros = processor.zero.count;
		rep.pollSeconds = took;
		outputValue->setData(&data);
	}

private:
	FtSource* source;
	const double period;
	ft_core::Processor processor;
	ft_core::Report rep;
	jt_type data;
	std::atomic<int> zeroRequest;
	std::atomic<int> zeroCount;
	std::atomic<unsigned long> readings;
	std::atomic<double> worstPoll;

	DISALLOW_COPY_AND_ASSIGN(FtTorques);

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
