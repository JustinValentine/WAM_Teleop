/*
 * robotiq_teleop.cpp
 *
 * A Robotiq 2F gripper (2F-140) on its serial adapter, teleoperated from the
 * Sony navigation controller, with a restart that works in the middle of a
 * run (launch/robotiq_nav_controller.launch; `wam teleop` with GRIPPER=robotiq
 * starts it in the gripper pane).
 *
 * lib/robotiq_teleop_core.h decides everything: what the stick does, when the
 * gripper counts as not answering or faulted, how a restart goes. This file
 * is the node around it: it reads the joystick (sensor_msgs/Joy), talks to
 * the gripper through libgripper's RobotiqGripper (Modbus RTU), runs the core
 * 40 times a second, and prints what the core reports, which is what the
 * operator sees in the pane and what pane_logs/ keeps for afterwards.
 *
 *   L2 + stick up/down      close/open, at up to ~stick_speed mm/s
 *   L2 + cross              open fully
 *   L2 + circle, held 1 s   restart the gripper: the port is closed and
 *                           opened, the gripper reset and activated. Its
 *                           fingers run through their travel, so whatever
 *                           they hold falls. Also what activates it the
 *                           first time after it was powered on.
 *
 * Topics and services (in the node's name, /robotiq_teleop in the launch file):
 *   joy            in, sensor_msgs/Joy (remapped to /nav_controller_teleop/joy)
 *   ~joint_states  out, sensor_msgs/JointState: the opening in m, the motor
 *                  current in A as the effort; only while the gripper answers
 *   ~restart       std_srvs/Trigger: the same restart; answers when it is over
 *   ~status        std_srvs/Trigger: where things stand, in one line
 *
 * Parameters: config/robotiq_nav_controller.yaml.
 *
 * Nothing is sent to the gripper when the node ends: the fingers stay where
 * they are (they are self-locking), and so does what they hold.
 */

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <exception>
#include <iostream>
#include <map>
#include <mutex>
#include <streambuf>
#include <string>
#include <thread>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <std_srvs/Trigger.h>

#include <gripper/robotiq/robotiq_gripper.h>

#include "lib/robotiq_teleop_core.h"

namespace {

typedef std::chrono::steady_clock Clock;

std::string strf(const char* format, ...) {
	char buf[1024];
	va_list ap;
	va_start(ap, format);
	vsnprintf(buf, sizeof(buf), format, ap);
	va_end(ap);
	return buf;
}

// One line in the pane, with the time of day: the pane's log is what is left
// of a failure afterwards, next to the arms' recordings.
void say(const std::string& line) {
	char stamp[16];
	const std::time_t now = std::time(NULL);
	struct tm local;
	localtime_r(&now, &local);
	std::strftime(stamp, sizeof(stamp), "%H:%M:%S", &local);
	std::printf("%s Gripper: %s\n", stamp, line.c_str());
	std::fflush(stdout);
}

// libgripper reports its serial and Modbus errors on std::cerr ("[Serial]
// Write error: ...", "[Robotiq] Timed out waiting for Modbus response."),
// some of them from its own I/O thread. They are the only word on what the
// serial line did, so they go to the pane; but one comes with every failed
// exchange, several a second while the gripper is away. This takes std::cerr
// over and hands the lines out for SerialReport to thin.
class CerrLines : public std::streambuf {
public:
	CerrLines() : old(std::cerr.rdbuf(this)) {}
	~CerrLines() { std::cerr.rdbuf(old); }

	std::vector<std::string> take() {
		std::lock_guard<std::mutex> lock(mutex);
		std::vector<std::string> out;
		out.swap(lines);
		return out;
	}

protected:
	int overflow(int c) override {
		if (c != traits_type::eof()) {
			std::lock_guard<std::mutex> lock(mutex);
			put(static_cast<char>(c));
		}
		return c;
	}
	std::streamsize xsputn(const char* s, std::streamsize n) override {
		std::lock_guard<std::mutex> lock(mutex);
		for (std::streamsize i = 0; i < n; ++i) {
			put(s[i]);
		}
		return n;
	}

private:
	// A line is put together per thread: the library's I/O thread and the
	// caller it has just woken write theirs at the same time.
	void put(char c) {
		std::string& partial = partials[std::this_thread::get_id()];
		if (c != '\n') {
			partial += c;
		} else if (!partial.empty()) {
			lines.push_back(partial);
			partial.clear();
		}
	}
	std::streambuf* old;
	std::mutex mutex;
	std::map<std::thread::id, std::string> partials;
	std::vector<std::string> lines;
};

// Each different line from the library at once, the same line again at most
// every QUIET seconds, as the number of times it came meanwhile. When the
// gripper is ready again the count so far is printed and everything is
// forgotten (startOver()), so that the next silence has the serial line's
// own words next to it from its first moment.
class SerialReport {
public:
	void add(const std::string& line, double now) {
		Entry& e = seen[line];
		if (!e.printed || now - e.at >= QUIET) {
			print(line, e, now);
		} else {
			++e.more;
		}
	}
	// The ones still owed, once their line has been quiet for long enough.
	void flush(double now) {
		for (std::map<std::string, Entry>::iterator i = seen.begin(); i != seen.end(); ++i) {
			if (i->second.more > 0 && now - i->second.at >= QUIET) {
				print(i->first, i->second, now);
			}
		}
	}
	void startOver(double now) {
		for (std::map<std::string, Entry>::iterator i = seen.begin(); i != seen.end(); ++i) {
			if (i->second.more > 0) {
				print(i->first, i->second, now);
			}
		}
		seen.clear();
	}
	static constexpr double QUIET = 10.0;  // s

private:
	struct Entry {
		bool printed = false;
		double at = 0.0;
		int more = 0;
	};
	void print(const std::string& line, Entry& e, double now) {
		say(e.more > 0 ? strf("serial: %d more in the last %.0f s of: %s", e.more, now - e.at, line.c_str()) : "serial: " + line);
		e.printed = true;
		e.at = now;
		e.more = 0;
	}
	std::map<std::string, Entry> seen;
};

class RobotiqTeleop {
public:
	RobotiqTeleop() : pnh("~"), core(readParams()), start(Clock::now()) {
		say(strf("Robotiq 2F on %s (%d baud, slave %d). L2 + stick up/down: %s, at up to %g mm/s. L2 + X: open fully. "
				"L2 + circle held %g s: restart. Each request: speed %.2f, force %.2f.", port.c_str(), baud, slave,
				upCloses ? "close/open" : "open/close", core.getParams().stickSpeed, core.getParams().restartHold,
				core.getParams().deviceSpeed, core.getParams().force));
		// 2F-140: 30 mm/s at speed 0 to 250 mm/s at speed 1 (its manual, 5.2).
		const double own = 30.0 + 220.0 * core.getParams().deviceSpeed;
		if (core.getParams().stickSpeed > own) {
			say(strf("Note: stick_speed %.0f mm/s is more than the gripper's own speed at device_speed %.2f (about %.0f mm/s "
					"on a 2F-140): the opening will follow the stick at the gripper's speed.", core.getParams().stickSpeed,
					core.getParams().deviceSpeed, own));
		}
		printNotes();

		joySub = nh.subscribe("joy", 1, &RobotiqTeleop::onJoy, this);
		statePub = pnh.advertise<sensor_msgs::JointState>("joint_states", 1);
		restartSrv = pnh.advertiseService("restart", &RobotiqTeleop::onRestart, this);
		statusSrv = pnh.advertiseService("status", &RobotiqTeleop::onStatus, this);
	}

	void run() {
		const Clock::duration period = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / rate));
		Clock::time_point next = Clock::now();
		while (ros::ok()) {
			next += period;
			step();
			const Clock::time_point t = Clock::now();
			if (t < next) {
				std::this_thread::sleep_until(next);
			} else {
				next = t;  // a slow exchange: no catching up
			}
		}
		say("stopped. Nothing was sent to the gripper at the end: it stays as it is.");
	}

private:
	robotiq_teleop::Params readParams() {
		robotiq_teleop::Params p;
		// The opening is converted to the gripper's counts by the library, for
		// the stroke it was written for.
		p.stroke = gripper::robotiq::kRobotiqMaxWidthMm;
		pnh.param<std::string>("port", port, "/dev/ttyUSB0");
		pnh.param("baud_rate", baud, 115200);
		pnh.param("slave_id", slave, 9);
		pnh.param("rate", rate, 40.0);
		rate = std::max(5.0, std::min(100.0, rate));
		pnh.param("deadman_button", deadmanButton, 5);
		pnh.param("open_button", openButton, 0);
		pnh.param("restart_button", restartButton, 1);
		pnh.param("axis", axis, 1);
		pnh.param("up_closes", upCloses, true);
		pnh.param<std::string>("joint_name", jointName, "finger_joint");
		pnh.param("stick_speed", p.stickSpeed, p.stickSpeed);
		pnh.param("device_speed", p.deviceSpeed, p.deviceSpeed);
		pnh.param("force", p.force, p.force);
		pnh.param("lead", p.lead, p.lead);
		pnh.param("deadband", p.deadband, p.deadband);
		pnh.param("joy_timeout", p.joyTimeout, p.joyTimeout);
		pnh.param("restart_hold", p.restartHold, p.restartHold);
		pnh.param("activation_timeout", p.activationTimeout, p.activationTimeout);
		pnh.param("auto_activate", p.autoActivate, p.autoActivate);
		pnh.param("auto_restart", p.autoRestart, p.autoRestart);
		return p;
	}

	double seconds() const {
		return std::chrono::duration<double>(Clock::now() - start).count();
	}

	void onJoy(const sensor_msgs::Joy::ConstPtr& m) {
		robotiq_teleop::Joy j;
		j.valid = true;
		j.stamp = seconds();
		j.deadman = pressed(*m, deadmanButton);
		j.open = pressed(*m, openButton);
		j.restart = pressed(*m, restartButton);
		if (axis >= 0 && axis < static_cast<int>(m->axes.size())) {
			j.axis = (upCloses ? 1.0 : -1.0) * m->axes[axis];
		}
		std::lock_guard<std::mutex> lock(mutex);
		if (!joyChecked) {
			joyChecked = true;
			const int most = std::max(deadmanButton, std::max(openButton, restartButton));
			if (most >= static_cast<int>(m->buttons.size()) || axis >= static_cast<int>(m->axes.size())) {
				joyNote = strf("the joystick messages have %zu buttons and %zu axes, fewer than the buttons (%d, %d, %d) and the "
						"axis (%d) set for the gripper: is this the navigation controller?", m->buttons.size(), m->axes.size(),
						deadmanButton, openButton, restartButton, axis);
			}
		}
		joy = j;
	}
	static bool pressed(const sensor_msgs::Joy& m, int button) {
		return button >= 0 && button < static_cast<int>(m.buttons.size()) && m.buttons[button] != 0;
	}

	// The same restart as L2 + circle. Answers when it is over, with the
	// pane's line about it. A restart that is running already is waited for
	// like one asked for here.
	bool onRestart(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
		std::unique_lock<std::mutex> lock(mutex);
		const unsigned before = core.restartsFinished();
		core.requestRestart("the restart service: wam gripper restart");
		const double limit = seconds() + core.getParams().activationTimeout + 5.0;
		while (ros::ok() && core.restartsFinished() == before) {
			// The loop stands still for the 4 s of the library's wait in an
			// activation. For much longer than that it is stuck inside the
			// library, where nothing here can reach it: the node ends, without
			// the library's clean-up (which may be what hangs), and its launch
			// file starts a new one; `wam gripper restart` then asks that one.
			if (seconds() - lastStep > STUCK_AFTER) {
				say(strf("a restart was asked for, and the node's loop has not run for %.0f s: it is stuck in the gripper "
						"library. Ending the node; its launch file starts a new one.", seconds() - lastStep));
				std::_Exit(1);
			}
			if (seconds() > limit) {
				res.success = false;
				res.message = "the restart is not over yet: " + core.describe(seconds());
				return true;
			}
			changed.wait_for(lock, std::chrono::milliseconds(200));
		}
		res.success = core.restartsFinished() != before && core.lastRestartWorked();
		res.message = core.restartsFinished() != before ? core.lastRestartNote() : "the node is stopping";
		return true;
	}

	bool onStatus(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
		std::lock_guard<std::mutex> lock(mutex);
		res.success = core.getState() == robotiq_teleop::READY;
		res.message = core.describe(seconds()) + strf("; port %s", port.c_str());
		if (seconds() - lastStep > 1.0) {
			res.message += strf("; the node's loop last ran %.1f s ago (an activation holds it for up to 4 s; longer: it is stuck)",
					seconds() - lastStep);
		}
		return true;
	}

	// The manual: at most 200 exchanges a second, 5 ms between two.
	void pace() {
		const Clock::time_point earliest = lastExchange + std::chrono::milliseconds(5);
		if (Clock::now() < earliest) {
			std::this_thread::sleep_until(earliest);
		}
	}

	robotiq_teleop::Status readStatus() {
		robotiq_teleop::Status s;
		if (!gripper.isConnected()) {
			return s;
		}
		pace();
		const gripper::robotiq::GripperStatus g = gripper.getStatus();
		lastExchange = Clock::now();
		s.valid = g.valid;
		s.activated = g.activated;
		s.ready = g.ready;
		s.moving = g.moving;
		s.object = g.object_status;
		s.fault = g.fault_code;
		s.echo = g.requested_position;
		s.position = g.actual_position;
		s.current = g.motor_current;
		s.width = g.width_mm;
		return s;
	}

	bool perform(const robotiq_teleop::Action& a) {
		using robotiq_teleop::Action;
		bool ok = true;
		if (a.kind != Action::DISCONNECT && a.kind != Action::CONNECT) {
			pace();
		}
		switch (a.kind) {
		case Action::MOVE:
			ok = gripper.setWidth(a.width, a.speed, a.force);
			break;
		case Action::DISCONNECT:
			gripper.disconnect();
			break;
		case Action::CONNECT:
			ok = gripper.connect(port, static_cast<unsigned int>(baud), static_cast<uint8_t>(slave));
			break;
		case Action::RESET:
			ok = gripper.reset();
			break;
		case Action::ACTIVATE:
			// Waits in the library until the gripper is ready, 4 s at most.
			ok = gripper.activate(false, a.speed, a.force);
			break;
		default:
			break;
		}
		lastExchange = Clock::now();
		return ok;
	}

	void step() {
		robotiq_teleop::Status status;
		robotiq_teleop::Action action;
		bool ok = false;
		std::string trouble;
		robotiq_teleop::State state = lastState;
		try {
			status = readStatus();
			{
				std::lock_guard<std::mutex> lock(mutex);
				action = core.step(seconds(), joy, status);
			}
			if (action.kind != robotiq_teleop::Action::NONE) {
				ok = perform(action);
			}
		} catch (const std::exception& e) {
			// Nothing in the library is known to throw here; if something
			// does, it is a failed exchange like any other, and the pane has it.
			trouble = e.what();
			ok = false;
		}
		{
			std::lock_guard<std::mutex> lock(mutex);
			if (action.kind != robotiq_teleop::Action::NONE) {
				core.result(ok, seconds());
			}
			lastStep = seconds();
			state = core.getState();
			if (!joyNote.empty()) {
				say(joyNote);
				joyNote.clear();
			}
		}
		changed.notify_all();
		if (!trouble.empty()) {
			serial.add("exception from the gripper library: " + trouble, seconds());
		}
		const std::vector<std::string> lines = cerrLines.take();
		for (size_t i = 0; i < lines.size(); ++i) {
			serial.add(lines[i], seconds());
		}
		if (state == robotiq_teleop::READY && lastState != robotiq_teleop::READY) {
			serial.startOver(seconds());
		} else {
			serial.flush(seconds());
		}
		lastState = state;
		printNotes();

		if (status.valid) {
			sensor_msgs::JointState msg;
			msg.header.stamp = ros::Time::now();
			msg.name.push_back(jointName);
			msg.position.push_back(status.width / 1000.0);
			msg.effort.push_back(status.current * 0.01);  // the manual: about 10 mA a count
			statePub.publish(msg);
		}
	}

	void printNotes() {
		std::vector<std::string> notes;
		{
			std::lock_guard<std::mutex> lock(mutex);
			notes = core.takeNotes();
		}
		for (size_t i = 0; i < notes.size(); ++i) {
			say(notes[i]);
		}
	}

	// The loop not having run for this long is not an activation any more.
	static constexpr double STUCK_AFTER = 8.0;  // s

	ros::NodeHandle nh, pnh;
	std::string port, jointName;
	int baud, slave, deadmanButton, openButton, restartButton, axis;
	double rate;
	bool upCloses;

	std::mutex mutex;  // the core, the joystick message, lastStep
	std::condition_variable changed;
	robotiq_teleop::Core core;
	robotiq_teleop::Joy joy;
	bool joyChecked = false;
	std::string joyNote;
	double lastStep = 0.0;
	robotiq_teleop::State lastState = robotiq_teleop::SILENT;

	CerrLines cerrLines;
	SerialReport serial;
	gripper::robotiq::RobotiqGripper gripper;
	Clock::time_point start;
	Clock::time_point lastExchange;

	ros::Subscriber joySub;
	ros::Publisher statePub;
	ros::ServiceServer restartSrv, statusSrv;
};

}  // namespace

int main(int argc, char** argv) {
	ros::init(argc, argv, "robotiq_teleop");
	RobotiqTeleop node;
	ros::AsyncSpinner spinner(2);  // the joystick and the services, next to the loop
	spinner.start();
	node.run();
	return 0;
}
