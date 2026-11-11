#pragma once

/* Stops the teleop when this arm's safety module leaves ACTIVE: a velocity or
 * torque fault, an E-stop, Shift-Idle. Without it a faulted arm's program kept
 * running: a faulted leader went on sending its sagging position and the
 * torques it could no longer apply, and the follower shook with them, then
 * followed the leader down into its stops, pushing with 63 Nm (J2) and 50 Nm
 * (J4) (2026-09-24, twice). Polls the mode at 10 Hz outside the realtime loop
 * and calls onInactive() once: the first time it finds the arm not active. */

#include <atomic>
#include <chrono>
#include <cstdio>
#include <functional>
#include <stdexcept>
#include <thread>

#include <barrett/products/safety_module.h>

class SafetyWatch {
public:
	SafetyWatch(barrett::SafetyModule* safety, const std::function<void()>& onInactive)
		: safety(safety), onInactive(onInactive), stopping(false), thread(&SafetyWatch::run, this) {}
	~SafetyWatch() { stop(); }

	// Before a normal exit, whose Shift-Idle is not a fault.
	void stop() {
		stopping = true;
		if (thread.joinable()) {
			thread.join();
		}
	}

private:
	void run() {
		bool warned = false;
		while (!stopping) {
			try {
				if (safety->getMode() != barrett::SafetyModule::ACTIVE) {
					onInactive();
					return;
				}
				warned = false;
			} catch (const std::exception& e) {  // a lost reply: poll again
				if (!warned) {
					printf("Safety mode poll failed (%s); polling again.\n", e.what());
					warned = true;
				}
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	}

	barrett::SafetyModule* const safety;
	const std::function<void()> onInactive;
	std::atomic<bool> stopping;
	std::thread thread;  // last: starts once the rest is set up
};
