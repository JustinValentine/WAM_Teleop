#pragma once

/* The Barrett force/torque sensor on the follower's CAN bus (puck 8, the
 * legacy library), read from the control loop without ever waiting.
 *
 * ForceTorqueSensor::update() is not used: it sends the request and then
 * spins on the bus until both replies are in, holding the bus lock, for up to
 * a second, and throws if they do not come. In the control loop that is a
 * late cycle every cycle, and a loop that dies (the arm drops out of ACTIVE)
 * the day the sensor's connector does.
 *
 * Here each cycle takes whatever replies have arrived (the library's
 * non-blocking receive) and, once the last request is answered, sends the
 * next one. The replies come back while the loop sleeps, so a reading is one
 * cycle old when it is used and the loop never waits for the sensor. A
 * request left unanswered for TIMEOUT_CYCLES is counted and sent again; the
 * teleop meanwhile does without the sensor (ft_core::Health).
 *
 * The library keeps at most ten unread messages per CAN id and throws, from
 * whoever reads the bus next (the arm's own position read), when an eleventh
 * arrives. So a request is only sent after the replies to the last one were
 * taken, and late replies are always drained. */

#include <exception>

#include <barrett/bus/abstract/communications_bus.h>
#include <barrett/products/force_torque_sensor.h>
#include <barrett/products/puck.h>
#include <barrett/thread/abstract/mutex.h>

#include "ft_systems.h"

class BarrettFtSource : public FtSource {
public:
	static const int TIMEOUT_CYCLES = 10;  // 20 ms

	explicit BarrettFtSource(barrett::ForceTorqueSensor* fts)
		: bus(fts->getPuck()->getBus()), id(fts->getPuck()->getId()),
		  propId(barrett::Puck::getPropertyId(barrett::Puck::FT, barrett::Puck::PT_ForceTorque, 0)),
		  pending(false), haveForce(false), haveTorque(false), waited(0), timeouts(0), sendErrors(0),
		  busErrors(0) {}
	/* The sensor at puck `id` on `bus`, already awake (test/test_ft_bus.cpp). */
	BarrettFtSource(const barrett::bus::CommunicationsBus& bus, int id)
		: bus(bus), id(id),
		  propId(barrett::Puck::getPropertyId(barrett::Puck::FT, barrett::Puck::PT_ForceTorque, 0)),
		  pending(false), haveForce(false), haveTorque(false), waited(0), timeouts(0), sendErrors(0),
		  busErrors(0) {}

	virtual bool poll(double* force, double* torque) {
		bool fresh = false;
		try {
			BARRETT_SCOPED_LOCK(bus.getMutex());
			barrett::ForceTorqueSensor::ForceParser::result_type f;
			barrett::ForceTorqueSensor::TorqueParser::result_type t;
			// Every reply that has arrived; the newest of each is kept.
			for (int i = 0; i < DRAIN_LIMIT; ++i) {
				const int ret = barrett::Puck::receiveGetPropertyReply<barrett::ForceTorqueSensor::ForceParser>(
						bus, id, propId, &f, false, true);
				if (ret == 0) {
					for (int k = 0; k < 3; ++k) lastForce[k] = f[k];
					haveForce = true;
				} else if (ret > 0) {  // 1: none waiting; else a bus error
					if (ret != 1) busErrors = busErrors + 1;
					break;
				}  // < 0: a malformed reply, dropped
			}
			for (int i = 0; i < DRAIN_LIMIT; ++i) {
				const int ret = barrett::Puck::receiveGetPropertyReply<barrett::ForceTorqueSensor::TorqueParser>(
						bus, id, propId, &t, false, true);
				if (ret == 0) {
					for (int k = 0; k < 3; ++k) lastTorque[k] = t[k];
					haveTorque = true;
				} else if (ret > 0) {
					if (ret != 1) busErrors = busErrors + 1;
					break;
				}
			}
			if (pending && haveForce && haveTorque) {
				for (int k = 0; k < 3; ++k) {
					force[k] = lastForce[k];
					torque[k] = lastTorque[k];
				}
				fresh = true;
				pending = false;
			} else if (pending && ++waited > TIMEOUT_CYCLES) {
				timeouts = timeouts + 1;
				pending = false;
			}
			if (!pending) {
				haveForce = haveTorque = false;
				waited = 0;
				if (barrett::Puck::sendGetPropertyRequest(bus, id, propId) == 0) {
					pending = true;
				} else {
					sendErrors = sendErrors + 1;
				}
			}
		} catch (const std::exception&) {
			busErrors = busErrors + 1;  // the loop goes on without the sensor
		}
		return fresh;
	}

	unsigned long getTimeouts() const { return timeouts; }
	unsigned long getSendErrors() const { return sendErrors; }
	unsigned long getBusErrors() const { return busErrors; }

private:
	static const int DRAIN_LIMIT = 12;

	const barrett::bus::CommunicationsBus& bus;
	const int id, propId;
	bool pending, haveForce, haveTorque;
	int waited;
	double lastForce[3], lastTorque[3];
	std::atomic<unsigned long> timeouts, sendErrors, busErrors;
};
