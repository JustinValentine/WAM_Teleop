// lib/teleop_mode.h across programs, without robots: run as two processes
// (see CMakeLists.txt). "listen s": print each mode received for s seconds.
// "send full|no4c|pp": publish that mode once (as the 'c' and 'p' keys do),
// and print what this program itself receives.

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

#include <ros/ros.h>

#include "teleop_mode.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "teleop_mode_test", ros::init_options::AnonymousName);
	teleop_law::Mode mode = teleop_law::FULL;
	if (argc < 3 || (std::string(argv[1]) == "send" && !teleop_law::parseMode(argv[2], &mode))) {
		std::printf("usage: teleop_mode_tests listen <seconds> | send full|no4c|pp\n");
		return 2;
	}
	const std::string role = argv[1];
	TeleopModeSwitch sw([&role](teleop_law::Mode m) {
		std::printf("%s got %s\n", role.c_str(), teleop_law::modeKey(m));
		std::fflush(stdout);
	});
	if (role == "listen") {
		std::this_thread::sleep_for(std::chrono::duration<double>(std::atof(argv[2])));
	} else {
		std::this_thread::sleep_for(std::chrono::milliseconds(1500));  // the connections
		sw.publish(mode);
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}
	return 0;
}
