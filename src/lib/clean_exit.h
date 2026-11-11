#pragma once

/* A clean end for the arm programs (leader_wam8g, follower).
 *
 * Ctrl-C, SIGTERM, SIGHUP (wam stop's force-stop sends SIGINT), or the end of
 * the program's input (its terminal went away) end the teleop at the prompt as
 * 'x' does: the teleop stops, the arm holds where it is, the follower's hand
 * closes to a fist. Then waitUntilNotActive() waits for the WAM to leave
 * ACTIVE: Shift-Idle or E-Stop. It also gives up when the WAM's safety module
 * stops answering (the CAN link is down), or on a second signal, and the
 * program exits.
 *
 * The handler only counts the signal. Linux gives a process-directed signal
 * to the main thread when it can take it (it is not blocked there), so the
 * prompt's read returns early (no SA_RESTART) and the realtime threads are
 * left alone. */

#include <signal.h>
#include <unistd.h>

#include <atomic>
#include <cstdio>
#include <exception>
#include <iostream>
#include <string>

namespace clean_exit {

inline std::atomic<int>& signalCount() {
	static std::atomic<int> n(0);
	return n;
}

static void onSignal(int) { ++signalCount(); }

inline void install() {
	struct sigaction sa = {};
	sa.sa_handler = onSignal;
	sigemptyset(&sa.sa_mask);
	sa.sa_flags = 0;  // no SA_RESTART: a read at the prompt returns
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);
	sigaction(SIGHUP, &sa, NULL);
}

inline bool requested() { return signalCount() > 0; }

// The prompt's next line. False: end the teleop (a stop signal, or the input
// ended).
inline bool readLine(std::string* line) {
	if (requested()) {
		return false;
	}
	if (std::getline(std::cin, *line)) {
		return true;
	}
	if (!requested()) {
		std::printf("\n(no more input: ending the teleop)\n");
	}
	return false;
}

// After the teleop: waits for the WAM to leave ACTIVE (Shift-Idle or E-Stop),
// polling its safety module. Also returns when the module stops answering
// (four polls in a row, about a second) or on a second stop signal.
template <typename SafetyModule>
void waitUntilNotActive(SafetyModule* sm) {
	std::printf("Shift-Idle (or E-Stop) the WAM to finish.\n");
	std::fflush(stdout);
	const int signalsBefore = signalCount();
	int failures = 0;
	for (;;) {
		if (signalCount() > signalsBefore || signalCount() >= 2) {
			std::printf("Stopped by a signal: exiting now (a WAM still active stops being controlled).\n");
			return;
		}
		try {
			if (sm->getMode() != SafetyModule::ACTIVE) {
				std::printf("The WAM is not active: done.\n");
				return;
			}
			failures = 0;
		} catch (const std::exception& e) {
			if (++failures >= 4) {
				std::printf("The WAM's safety module does not answer (%s): exiting.\n", e.what());
				return;
			}
		}
		usleep(250000);
	}
}

}  // namespace clean_exit
