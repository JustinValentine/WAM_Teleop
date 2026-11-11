// lib/clean_exit.h without a robot. Run by CMake's clean_exit_tests target as
//   clean_exit_tests prompt   (stdin kept open; send it SIGINT: the prompt ends)
//   clean_exit_tests eof      (stdin closed: the prompt ends)
//   clean_exit_tests wait <idle|estop|deaf|active>  (the final wait; for active,
//                             send two SIGINTs)
// A worker thread blocked in a read, as the realtime loop is on its CAN
// socket, must not be interrupted by the signal: it reports how its read
// ended when the program exits.

#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>

#include "clean_exit.h"

namespace {

struct FakeSafety {
	enum SafetyMode { ESTOP, IDLE, ACTIVE };
	std::string kind;
	std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
	SafetyMode getMode() {
		const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
		if (kind == "deaf") {
			throw std::runtime_error("Puck::getProperty(): Failed to receive reply");
		}
		if (kind == "idle" && t > 1.0) {
			return IDLE;
		}
		if (kind == "estop" && t > 1.0) {
			return ESTOP;
		}
		return ACTIVE;
	}
};

}  // namespace

int main(int argc, char** argv) {
	if (argc < 2) {
		return 2;
	}
	// the "realtime loop": blocked reading a pipe nobody writes
	int fds[2];
	if (pipe(fds) != 0) {
		return 2;
	}
	std::atomic<int> workerErrno(-1);
	std::thread worker([&] {
		char c;
		const ssize_t r = read(fds[0], &c, 1);
		workerErrno = r < 0 ? errno : 0;
	});

	clean_exit::install();
	const auto t0 = std::chrono::steady_clock::now();
	const std::string mode = argv[1];
	if (mode == "prompt" || mode == "eof") {
		std::string line;
		int lines = 0;
		while (clean_exit::readLine(&line)) {
			++lines;
		}
		std::printf("prompt ended after %d lines, %s\n", lines, clean_exit::requested() ? "by a signal" : "at the end of input");
	} else if (mode == "wait" && argc > 2) {
		FakeSafety sm;
		sm.kind = argv[2];
		clean_exit::waitUntilNotActive(&sm);
	}
	const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
	// the worker should still be blocked: its read was not interrupted
	const bool workerUntouched = workerErrno == -1;
	close(fds[1]);  // now let it finish
	worker.join();
	std::printf("done after %.1f s; the worker's read was %s\n", t, workerUntouched ? "not interrupted" : "interrupted");
	return workerUntouched ? 0 : 1;
}
