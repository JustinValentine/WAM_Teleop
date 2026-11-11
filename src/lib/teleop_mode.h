#pragma once

/* The teleop's mode (teleop_law::Mode: full, 4C off, P-P) during the teleop,
 * both arms at once: 'c' or 'p' in either program (leader_wam8g, follower),
 * or `wam 4c on|off` / `wam pp on|off`, publishes the mode on /teleop/mode
 * (std_msgs/String: full, no4c, pp); both programs subscribe, and each arm
 * fades its law's terms to it. The mode goes out, not a toggle, so after any
 * change the two arms agree. */

#include <cstdio>
#include <functional>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/String.h>

#include "teleop_law.h"

class TeleopModeSwitch {
public:
	// apply(mode): called on this program's own spinner thread for every
	// message, this program's included.
	explicit TeleopModeSwitch(const std::function<void(teleop_law::Mode)>& apply) : apply(apply), spinner(1, &queue) {
		nh.setCallbackQueue(&queue);
		sub = nh.subscribe(topic(), 10, &TeleopModeSwitch::onMessage, this);
		pub = nh.advertise<std_msgs::String>(topic(), 10);
		spinner.start();
	}
	~TeleopModeSwitch() { spinner.stop(); }

	// Both arms to the mode (this one too, through its own subscription).
	void publish(teleop_law::Mode mode) {
		std_msgs::String m;
		m.data = teleop_law::modeKey(mode);
		pub.publish(m);
	}

	static const char* topic() { return "/teleop/mode"; }

private:
	void onMessage(const std_msgs::String::ConstPtr& m) {
		teleop_law::Mode mode;
		if (teleop_law::parseMode(m->data, &mode)) {
			apply(mode);
		} else {
			std::printf("\n%s: \"%s\" is not a mode (full, no4c, pp); ignored.\n", topic(), m->data.c_str());
			std::fflush(stdout);
		}
	}

	std::function<void(teleop_law::Mode)> apply;
	ros::NodeHandle nh;
	ros::CallbackQueue queue;
	ros::AsyncSpinner spinner;
	ros::Subscriber sub;
	ros::Publisher pub;
};
