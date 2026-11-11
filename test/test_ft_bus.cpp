/* The force/torque sensor's read (lib/ft_barrett.h) on libbarrett's own bus
 * code, next to the arm's 500 Hz traffic, on the simulated CAN bus
 * (sim_barrett_bus.h: WAM pucks 1-7, the sensor at puck 8). No robot.
 * Run: devel/lib/wam_teleop/ft_bus_tests */

#include "sim_barrett_bus.h"

#include "ft_barrett.h"

using namespace sim;

namespace {

/* The follower's cycle as the bus sees it (LowLevelWam::update and
 * setTorques), then the sensor's poll, as FtTorques does it. */
struct Loop {
	const barrett::bus::CommunicationsBus& bus;
	std::vector<Puck*> pucks;
	std::unique_ptr<PuckGroup> group, lower, upper;
	int torqueProp;
	BarrettFtSource source;
	double force[3], torque[3];
	long fresh, cyclesRun;
	std::vector<double> pollMs, cycleMs;
	std::string error;

	explicit Loop(const barrett::bus::CommunicationsBus& bus) : bus(bus), source(bus, 8), fresh(0), cyclesRun(0) {
		for (int id = 1; id <= 7; ++id) {
			pucks.push_back(new Puck(bus, id));
		}
		group.reset(new PuckGroup(PuckGroup::BGRP_WAM, pucks));
		lower.reset(new PuckGroup(PuckGroup::BGRP_LOWER_WAM, std::vector<Puck*>(pucks.begin(), pucks.begin() + 4)));
		upper.reset(new PuckGroup(PuckGroup::BGRP_UPPER_WAM, std::vector<Puck*>(pucks.begin() + 4, pucks.end())));
		torqueProp = pucks[0]->getPropertyId(Puck::T);
		for (int i = 0; i < 3; ++i) force[i] = torque[i] = 0.0;
	}
	~Loop() {
		for (size_t i = 0; i < pucks.size(); ++i) {
			delete pucks[i];
		}
	}

	/* n cycles at 500 Hz; false if the arm's bus work threw. */
	bool run(int n) {
		MotorPuck::CombinedPositionParser<double>::result_type positions[7];
		const double pt[7] = {0, 0, 0, 0, 0, 0, 0};
		fresh = 0;
		cyclesRun = 0;
		pollMs.clear();
		cycleMs.clear();
		Clock::time_point next = Clock::now();
		for (int c = 0; c < n; ++c) {
			next += std::chrono::milliseconds(2);
			const Clock::time_point a = Clock::now();
			try {
				group->getProperty<MotorPuck::CombinedPositionParser<double> >(Puck::P, positions, false);
				BARRETT_SCOPED_LOCK(bus.getMutex());
				MotorPuck::sendPackedTorques(bus, lower->getId(), torqueProp, pt, 4);
				MotorPuck::sendPackedTorques(bus, upper->getId(), torqueProp, pt + 4, 3);
			} catch (const std::exception& e) {
				error = e.what();
				return false;
			}
			const Clock::time_point b = Clock::now();
			if (source.poll(force, torque)) {
				++fresh;
			}
			const Clock::time_point d = Clock::now();
			pollMs.push_back(std::chrono::duration<double, std::milli>(d - b).count());
			cycleMs.push_back(std::chrono::duration<double, std::milli>(d - a).count());
			++cyclesRun;
			if (d < next) {
				std::this_thread::sleep_until(next);
			} else {
				next = d;
			}
		}
		return true;
	}
};

}  // namespace

int main() {
	SimCan can(175);
	const int fd = can.addHost(followerFilter);
	SimBus simBus(fd);
	barrett::bus::BusManager bm(&simBus);
	can.start();
	Loop loop(bm);

	std::printf("the sensor answering\n");
	const int forceCounts[3] = {384, -512, 100};   // 1.5, -2, 0.390625 N
	const int torqueCounts[3] = {1024, -2048, 1};  // 0.25, -0.5, 1/4096 Nm
	for (int i = 0; i < 3; ++i) {
		can.ftForceCounts[i] = forceCounts[i];
		can.ftTorqueCounts[i] = torqueCounts[i];
	}
	check(loop.run(1000), "the arm's loop runs with the sensor polled: " + loop.error);
	check(loop.fresh >= 990, strf("a reading in nearly every cycle (%ld of 1000)", loop.fresh));
	bool decoded = true;
	for (int i = 0; i < 3; ++i) {
		decoded = decoded && loop.force[i] == forceCounts[i] / 256.0 && loop.torque[i] == torqueCounts[i] / 4096.0;
	}
	check(decoded, strf("force and torque decoded (%.4f %.4f %.4f N, %.6f %.6f %.6f Nm)", loop.force[0], loop.force[1],
			loop.force[2], loop.torque[0], loop.torque[1], loop.torque[2]));
	check(loop.source.getTimeouts() == 0 && loop.source.getBusErrors() == 0 && loop.source.getSendErrors() == 0,
			"no timeouts or errors");
	check(std::labs(can.ftRequests - 1000) <= 2, strf("one request per cycle (%ld)", can.ftRequests.load()));
	std::printf("  the poll: %s\n  the whole cycle's bus work: %s\n", timing(loop.pollMs).c_str(), timing(loop.cycleMs).c_str());
	check(percentile(loop.pollMs, 0.99) < 0.5, "the poll takes under 0.5 ms in 99 % of the cycles (it never waits)");

	std::printf("the sensor not answering\n");
	can.ftAnswers = false;
	const long requestsBefore = can.ftRequests;
	check(loop.run(250), "the arm's loop is not disturbed: " + loop.error);
	const long asked = can.ftRequests - requestsBefore;
	check(loop.fresh <= 1, strf("no readings (%ld)", loop.fresh));
	check(loop.source.getTimeouts() >= 20 && loop.source.getTimeouts() <= 25,
			strf("unanswered requests counted, one per 22 ms (%lu)", loop.source.getTimeouts()));
	check(asked >= 20 && asked <= 26, strf("and asked again only then (%ld requests in 0.5 s)", asked));
	check(percentile(loop.pollMs, 1.0) < 2.0 && percentile(loop.cycleMs, 0.5) < 2.0,
			strf("the loop does not wait for it (poll max %.3f ms)", percentile(loop.pollMs, 1.0)));

	std::printf("the sensor answering again\n");
	can.ftAnswers = true;
	check(loop.run(250) && loop.fresh >= 235, strf("readings resume (%ld of 250)", loop.fresh));

	std::printf("more replies than requests\n");
	// The library keeps ten unread messages per id and throws at the next:
	// every reply must be taken each cycle.
	can.ftCopies = 4;
	check(loop.run(500), "four replies to every request for 1 s: the arm's loop survives: " + loop.error);
	check(loop.fresh >= 490 && loop.source.getBusErrors() == 0, strf("and the readings keep coming (%ld of 500)", loop.fresh));
	can.ftCopies = 1;

	can.stop();
	std::printf("%d checks, %d failed\n", checks, failures);
	return failures == 0 ? 0 : 1;
}
