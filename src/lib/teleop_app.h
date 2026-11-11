#pragma once

/* What leader_wam8g.cpp and follower.cpp share outside the control loop: the
 * options added with the low-latency update (2026-11-09), the recording's
 * file, and the thread that prints what the loop no longer prints itself. */

#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <functional>
#include <new>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>

#include "estimator_core.h"

namespace teleop_app {

/* The options both programs take. */
struct Options {
	std::string record;     // --record <file or directory>: the loop's own recording ("" none)
	bool ros;               // --no-ros clears it: no ROS at all (no roscore needed)
	bool rosState;          // --no-ros-state clears it: no joint_states/gravity publisher thread
	int rtPriority;         // --rt-priority n: SCHED_FIFO priority if the loop is not real-time (0: leave)
	int rtCpu;              // --rt-cpu n: pin the control thread (-1: no)
	bool ageReference;      // --reference age|hold
	bool matched;           // --estimator matched|legacy
	double estimatorHp, estimatorLp;  // --estimator-filter hp,lp (rad/s)
	double kiScale;         // --ki-scale s: J1-J4 integral gain times s
	Options()
		: ros(true), rosState(true), rtPriority(50), rtCpu(-1), ageReference(false), matched(false),
		  estimatorHp(25.0), estimatorLp(50.0), kiScale(1.0) {}
};

inline const char* usage() {
	return "  --record path  write one row per control cycle to path (a directory: a new\n"
	       "           teleop_<time>_<arm>.tlog in it), from the control loop, without ROS;\n"
	       "           scripts/teleop_log.py reads it\n"
	       "  --no-ros-state  do not publish joint_states and gravity (the ROS recording's source:\n"
	       "           a 500 Hz thread that takes the control loop's lock); use with --record\n"
	       "  --no-ros  no ROS at all, no roscore needed ('c' and 'p' still reach the other arm,\n"
	       "           through the link); not with --hand\n"
	       "  --rt-priority n  SCHED_FIFO priority for the control thread when the library left it\n"
	       "           non-real-time (the legacy library does), default 50; 0 leaves it alone\n"
	       "  --rt-cpu n  pin the control thread to CPU n (default: not pinned)\n"
	       "  --reference age|hold  age: advance the other arm's position by each packet's age\n"
	       "           (both programs on one computer); hold (default): as before\n"
	       "  --estimator matched|legacy  matched: the external-torque estimate with the commanded\n"
	       "           torque filtered like the model's acceleration; legacy (default): as before\n"
	       "  --estimator-filter hp,lp  the matched estimate's two corner frequencies in rad/s\n"
	       "           (default 25,50: the acceleration filter the legacy estimate uses)\n"
	       "  --ki-scale s  the position controller's integral gain on J1-J4 times s (0 to 1),\n"
	       "           default 1\n";
}

inline bool parseNumber(const char* s, double* v) {
	char* end = NULL;
	const double x = std::strtod(s, &end);
	if (end == s || *end != '\0' || x != x) {
		return false;
	}
	*v = x;
	return true;
}

/* One number, used for all n values, or n comma-separated ones, each from lo
 * to hi. */
inline bool parseList(const char* s, int n, double lo, double hi, double* out) {
	if (s == NULL) {
		return false;
	}
	double v[8];
	int count = 0;
	const char* p = s;
	for (;;) {
		char* end = NULL;
		const double x = std::strtod(p, &end);
		if (end == p || !(x >= lo && x <= hi) || count == n || count == 8) {
			return false;
		}
		v[count++] = x;
		if (*end == '\0') {
			break;
		}
		if (*end != ',') {
			return false;
		}
		p = end + 1;
	}
	if (count != 1 && count != n) {
		return false;
	}
	for (int i = 0; i < n; ++i) {
		out[i] = v[count == 1 ? 0 : i];
	}
	return true;
}

/* 1: argv[*i] was one of these options and is consumed (with its value);
 * 0: it is not one of them; -1: it is, with a bad or missing value (a message
 * is printed). */
inline int parse(int argc, char** argv, int* i, Options* o, const char* program) {
	const std::string a(argv[*i]);
	const char* value = (*i + 1 < argc) ? argv[*i + 1] : NULL;
	double x = 0.0;
	if (a == "--no-ros") {
		o->ros = false;
		o->rosState = false;
		return 1;
	}
	if (a == "--no-ros-state") {
		o->rosState = false;
		return 1;
	}
	if (a == "--record") {
		if (value == NULL || value[0] == '\0' || value[0] == '-') {
			printf("%s: --record takes a file or a directory.\n", program);
			return -1;
		}
		o->record = value;
	} else if (a == "--rt-priority") {
		if (value == NULL || !parseNumber(value, &x) || x < 0 || x > 90 || x != static_cast<int>(x)) {
			printf("%s: --rt-priority takes 0 (leave the thread alone) to 90.\n", program);
			return -1;
		}
		o->rtPriority = static_cast<int>(x);
	} else if (a == "--rt-cpu") {
		const long cpus = sysconf(_SC_NPROCESSORS_CONF);
		if (value == NULL || !parseNumber(value, &x) || x < -1 || x >= cpus || x != static_cast<int>(x)) {
			printf("%s: --rt-cpu takes a CPU number from 0 to %ld (-1: not pinned).\n", program, cpus - 1);
			return -1;
		}
		o->rtCpu = static_cast<int>(x);
	} else if (a == "--reference") {
		const std::string v(value ? value : "");
		if (v != "age" && v != "hold") {
			printf("%s: --reference takes age or hold.\n", program);
			return -1;
		}
		o->ageReference = (v == "age");
	} else if (a == "--estimator") {
		const std::string v(value ? value : "");
		if (v != "matched" && v != "legacy") {
			printf("%s: --estimator takes matched or legacy.\n", program);
			return -1;
		}
		o->matched = (v == "matched");
	} else if (a == "--estimator-filter") {
		const std::string v(value ? value : "");
		const size_t comma = v.find(',');
		double hp = 0.0, lp = 0.0;
		if (comma == std::string::npos || !parseNumber(v.substr(0, comma).c_str(), &hp) ||
		    !parseNumber(v.substr(comma + 1).c_str(), &lp) || !estimator::validCorners(hp, lp)) {
			printf("%s: --estimator-filter takes two corner frequencies in rad/s, 5 to 400, e.g. 25,50.\n", program);
			return -1;
		}
		o->estimatorHp = hp;
		o->estimatorLp = lp;
	} else if (a == "--ki-scale") {
		if (value == NULL || !parseNumber(value, &x) || x < 0.0 || x > 1.0) {
			printf("%s: --ki-scale takes 0 to 1.\n", program);
			return -1;
		}
		o->kiScale = x;
	} else {
		return 0;
	}
	++*i;
	return 1;
}

inline void print(const Options& o) {
	printf("Control thread: SCHED_FIFO %d if the library left it non-real-time%s",
	       o.rtPriority, o.rtPriority == 0 ? " (0: left alone)" : "");
	if (o.rtCpu >= 0) {
		printf(", pinned to CPU %d\n", o.rtCpu);
	} else {
		printf(", not pinned\n");
	}
	printf("Reference: %s\n", o.ageReference ? "age (the other arm's position advanced by each packet's age)"
	                                         : "hold (advanced only in cycles without a new packet)");
	if (o.matched) {
		printf("Estimator: matched (commanded torque and model through the acceleration's filter, %.0f and %.0f rad/s)\n",
		       o.estimatorHp, o.estimatorLp);
	} else {
		printf("Estimator: legacy (commanded torque minus model, unfiltered)\n");
	}
	if (o.kiScale != 1.0) {
		printf("Integral gain on J1-J4: times %.2f\n", o.kiScale);
	}
	printf("ROS: %s\n", !o.ros ? "off" : (o.rosState ? "on (joint_states and gravity published at 500 Hz)"
	                                                : "on for the mode and the hand only (no state publisher)"));
}

/* The recording's file: path itself, or, when path is a directory (or ends
 * with '/'), a new file named after the time and the arm in it. */
inline std::string recordFile(const std::string& path, const char* arm) {
	struct stat st;
	const bool directory = (!path.empty() && path[path.size() - 1] == '/') ||
	                       (stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode));
	if (!directory) {
		return path;
	}
	char stamp[32];
	const std::time_t now = std::time(NULL);
	struct tm local;
	localtime_r(&now, &local);
	std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &local);
	std::string out(path);
	if (out[out.size() - 1] != '/') {
		out += '/';
	}
	return out + "teleop_" + stamp + "_" + arm + ".tlog";
}

/* An object built later, or never, in place (no heap: the systems hold
 * fixed-size Eigen vectors, which a plain new may misalign). */
template <typename T>
class Lazy {
public:
	Lazy() : p(NULL) {}
	~Lazy() { reset(); }
	template <typename... A>
	T* emplace(A&&... a) {
		reset();
		p = new (static_cast<void*>(&storage)) T(std::forward<A>(a)...);
		return p;
	}
	void reset() {
		if (p != NULL) {
			p->~T();
			p = NULL;
		}
	}
	T* operator->() { return p; }
	T* get() { return p; }
	explicit operator bool() const { return p != NULL; }

private:
	typename std::aligned_storage<sizeof(T), 64>::type storage;
	T* p;
	Lazy(const Lazy&);
	Lazy& operator=(const Lazy&);
};

/* Calls poll() ten times a second on a normal thread: what the control loop
 * used to print itself (a lost link), and what it now reports through
 * counters (a mode taken from the other arm, the recording's state). */
class Watch {
public:
	explicit Watch(const std::function<void()>& poll) : poll(poll), stopping(false), thread(&Watch::run, this) {}
	~Watch() { stop(); }
	void stop() {
		stopping = true;
		if (thread.joinable()) {
			thread.join();
		}
	}

private:
	void run() {
		while (!stopping) {
			poll();
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	}
	const std::function<void()> poll;
	std::atomic<bool> stopping;
	std::thread thread;
};

}  // namespace teleop_app
