// follower --hand end to end without a robot: joystick messages ->
// joy_teleop (config/hand_nav_controller.yaml) -> bhand_mux ->
// /follower/hand/velocity_setpoint and joy_teleop -> /follower/hand/next_pose
// -> HandTeleopThread (lib/hand_teleop_ros.h, started as the follower does at
// the link pose: HI, fist) -> libbarrett's Hand -> the simulated bus of
// sim_barrett_bus.h, next to the arm's 500 Hz loop; at the end the fist again.
//
// Start roscore and launch/hand_nav_controller.launch first, with a joy_dev
// that does not exist (joy_dev:=/dev/no_joystick: its joy_node then finds no
// joystick and publishes nothing); this node plays the joystick on
// /nav_controller_teleop/joy.

#include <atomic>
#include <memory>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include "hand_teleop_ros.h"
#include "sim_barrett_bus.h"

using namespace sim;

namespace {

const int DEADMAN_BUTTON = 5;  // L2, config/hand_nav_controller.yaml
const int POSE_BUTTON = 0;     // cross, with L2

sensor_msgs::Joy joy(bool deadman, double axis0, double axis1, bool cross = false) {
	sensor_msgs::Joy j;
	j.header.stamp = ros::Time::now();
	j.axes.assign(3, 0.0f);
	j.buttons.assign(12, 0);  // the nav controller has 12
	j.axes[0] = axis0;
	j.axes[1] = axis1;
	j.buttons[DEADMAN_BUTTON] = deadman ? 1 : 0;
	j.buttons[POSE_BUTTON] = cross ? 1 : 0;
	return j;
}

// The same joystick state every 20 ms (joy_node's autorepeat).
void play(ros::Publisher& pub, double seconds, bool deadman, double axis0, double axis1, bool cross = false) {
	const Clock::time_point t0 = Clock::now();
	while (secondsSince(t0) < seconds) {
		pub.publish(joy(deadman, axis0, axis1, cross));
		sleepMs(20);
	}
}

// The most any finger was closed (rad) while the spread moved, sampled every
// 2 ms until stop().
class SpreadWatch {
public:
	explicit SpreadWatch(SimCan& sim) : most(0.0), running(true) {
		thread = std::thread([this, &sim] {
			double last = sim.pos(14);
			while (running) {
				sleepMs(2);
				const std::vector<double> p = sim.with([](Pucks& q) {
					return std::vector<double>{q[11].pos, q[12].pos, q[13].pos, q[14].pos};
				});
				if (p[3] != last) {
					most = std::max(most, std::max(p[0], std::max(p[1], p[2])) / FINGER_COUNTS_PER_RAD);
				}
				last = p[3];
			}
		});
	}
	double stop() {
		running = false;
		thread.join();
		return most;
	}

private:
	double most;
	std::atomic<bool> running;
	std::thread thread;
};

long vSets(SimCan& sim, int id) {
	return sim.with([&](Pucks& p) { return p[id].vSets; });
}

// The fingers closed, the spread at 0 (as close as a switch stops it).
bool isFist(SimCan& sim) {
	return sim.with([](Pucks& p) {
		bool fist = std::fabs(p[14].pos) <= (HandTeleop::END_MARGIN + HandTeleop::SPREAD_STOP) * SPREAD_COUNTS_PER_RAD + 1.0;
		for (int id = 11; id <= 13; ++id) {
			fist = fist && std::fabs(p[id].pos - p[id].maxPos) < 1.0;
		}
		return fist;
	});
}

}  // namespace

int main(int argc, char** argv) {
	ros::init(argc, argv, "hand_chain_test");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<sensor_msgs::Joy>("/nav_controller_teleop/joy", 1);

	std::printf("follower --hand, joystick to fingers\n");
	Rig rig(175);
	rig.sim.with([](Pucks& p) {  // wherever the last session left it
		p[11].pos = p[12].pos = p[13].pos = 60000;
		p[14].pos = 20000;
		return 0;
	});
	rig.startArm();
	rig.makeHand();
	std::unique_ptr<HandTeleopThread> hand = startHandTeleop(*rig.hand, true);
	check(isFist(rig.sim), "at the link pose: HI, then a fist (spread 0, fingers closed)");

	const Clock::time_point w0 = Clock::now();
	while (pub.getNumSubscribers() == 0 && secondsSince(w0) < 15.0) {
		sleepMs(50);
	}
	check(pub.getNumSubscribers() > 0, "joy_teleop subscribes to /nav_controller_teleop/joy");
	ros::master::V_TopicInfo topics;
	ros::master::getTopics(topics);
	bool setpointTopic = false;
	for (size_t i = 0; i < topics.size(); ++i) {
		if (topics[i].name == "/follower/hand/velocity_setpoint"
				&& topics[i].datatype == "gripper_ros_common/VelocitySetpoint") {
			setpointTopic = true;
		}
	}
	check(setpointTopic, "bhand_mux publishes /follower/hand/velocity_setpoint (gripper_ros_common/VelocitySetpoint)");
	sleepMs(1000);

	// 1) stick without the deadman button
	const double fist = rig.sim.pos(11);
	play(pub, 0.5, false, 0.0, -0.5);
	check(rig.sim.pos(11) == fist, "stick without the deadman button: the hand does not move");

	// 2) deadman + stick down: grasp = 2.0 x -0.5 = -1 rad/s (open) for 1 s
	const long v0 = vSets(rig.sim, 11);
	const Clock::time_point t0 = Clock::now();
	double firstCommand = -1.0;
	while (secondsSince(t0) < 1.0) {
		pub.publish(joy(true, 0.0, -0.5));
		for (int k = 0; k < 10; ++k) {
			sleepMs(2);
			if (firstCommand < 0.0 && vSets(rig.sim, 11) > v0) {
				firstCommand = secondsSince(t0);
			}
		}
	}
	const double opened = (fist - rig.sim.pos(11)) / FINGER_COUNTS_PER_RAD;
	check(firstCommand >= 0.0 && firstCommand < 0.1,
			strf("deadman + stick: the hand got its first command %.0f ms after the first joystick message", firstCommand * 1000));
	check(opened > 0.85 && opened < 1.02, strf("stick down 0.5 x scale 2.0 for 1 s: fingers opened %.3f rad", opened));

	// 3) deadman released, stick still pushed
	const double atRelease = rig.sim.pos(11);
	play(pub, 0.5, false, 0.0, -0.5);
	const double movedMs = (atRelease - rig.sim.pos(11)) / 81.0;  // V = 81 counts/ms
	check(movedMs < 160.0, strf("deadman released: the fingers stopped %.0f ms later", movedMs));

	// 4) the stick sideways: the spread goes by poses now
	const double s0 = rig.sim.pos(14);
	play(pub, 0.5, true, 0.5, 0.0);
	check(rig.sim.pos(14) == s0, "deadman + stick sideways (the old spread axis): the spread stays at 0%");
	play(pub, 0.2, true, 0.0, 0.0);

	// 5) L2 + cross held for 0.5 s: one pose on, to 50%
	SpreadWatch watch(rig.sim);
	const Clock::time_point p0 = Clock::now();
	play(pub, 0.5, true, 0.0, 0.0, true);
	play(pub, 0.1, true, 0.0, 0.0);
	const double near = HandTeleop::SPREAD_STOP * SPREAD_COUNTS_PER_RAD + 1.0;
	while (!(std::fabs(rig.sim.pos(14) - 17920.0) <= near && rig.sim.with([](Pucks& p) { return p[14].v == 0; }))
			&& secondsSince(p0) < 8.0) {
		pub.publish(joy(false, 0.0, 0.0));
		sleepMs(20);
	}
	const double took = secondsSince(p0);
	sleepMs(200);
	const double closed = watch.stop();
	const bool heldThere = rig.sim.with([&](Pucks& p) {
		return std::fabs(p[14].pos - 17920.0) <= near && p[14].mode == MotorPuck::MODE_VELOCITY && p[14].v == 0;
	});
	check(heldThere, strf("L2 + cross held for 0.5 s: the spread to 50%% (%.2f degrees), held, %.2f s after the press",
			rig.sim.pos(14) / SPREAD_COUNTS_PER_RAD * 180.0 / M_PI, took));
	check(closed <= 0.05, strf("the fingers were open whenever the spread moved (at most %.3f rad)", closed));

	// 6) at 50%, L2 + stick up: F1 and F2 close, F3 stays open
	const double f3 = rig.sim.pos(13);
	play(pub, 0.4, true, 0.0, 0.5);
	play(pub, 0.1, false, 0.0, 0.0);
	const double c1 = rig.sim.pos(11) / FINGER_COUNTS_PER_RAD, c2 = rig.sim.pos(12) / FINGER_COUNTS_PER_RAD;
	check(c1 > 0.3 && c2 > 0.3 && rig.sim.pos(13) == f3,
			strf("at 50%%, deadman + stick up for 0.4 s: F1 %.2f rad, F2 %.2f rad, F3 stays open", c1, c2));

	hand.reset();  // the follower on exit
	sleepMs(50);
	const bool idle = rig.sim.with([](Pucks& p) {
		for (int id = 11; id <= 14; ++id) {
			if (p[id].mode != MotorPuck::MODE_IDLE) {
				return false;
			}
		}
		return true;
	});
	check(isFist(rig.sim) && idle, "follower exit from 50%: the spread back to 0% (the fingers open), the hand closes to a fist and idles");
	check(!rig.arm->failed && rig.sim.violations == 0, "the arm's loop kept running; every frame went to the hand or the arm");
	std::printf("%d checks, %d failed\n", checks, failures);
	return failures == 0 ? 0 : 1;
}
