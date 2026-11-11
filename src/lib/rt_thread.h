#pragma once

/* Real-time setup of the control thread, done by the control loop itself.
 *
 * libbarrett's execution manager owns the 500 Hz thread. The CANopen library
 * (the wam8g) starts it SCHED_FIFO 50 with its memory locked; the legacy
 * library the follower uses does neither outside Xenomai (its
 * PeriodicLoopTimer ignores the priority), so the follower's loop ran as an
 * ordinary thread, competing with roscore, rosbag, the desktop and its own
 * helper threads: late whenever the machine was busy. The arm systems call
 * configureThisThread() first thing in their first cycle, from the control
 * thread: it becomes SCHED_FIFO if it is not real-time already, and is pinned
 * to one CPU if asked (thor has performance and efficiency cores; a control
 * thread that the scheduler moves between them changes speed).
 *
 * The report is read later by a normal thread: nothing here prints. */

#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>

#include <atomic>
#include <cstdio>

namespace rt_thread {

struct Report {
	std::atomic<int> done;
	int policyBefore, priorityBefore, policyAfter, priorityAfter;
	int cpuAsked, cpuNow, schedError, affinityError;
	Report() : done(0), policyBefore(0), priorityBefore(0), policyAfter(0), priorityAfter(0),
	           cpuAsked(-1), cpuNow(-1), schedError(0), affinityError(0) {}
};

inline const char* policyName(int policy) {
	switch (policy) {
	case SCHED_FIFO: return "SCHED_FIFO";
	case SCHED_RR: return "SCHED_RR";
	case SCHED_OTHER: return "SCHED_OTHER (not real-time)";
	default: return "another policy";
	}
}

/* fifoPriority: the priority to take if the thread is not real-time yet
 * (0: leave the scheduling alone). cpu: the CPU to pin to (-1: none). */
inline void configureThisThread(int fifoPriority, int cpu, Report* r) {
	struct sched_param sp = {};
	int policy = SCHED_OTHER;
	pthread_getschedparam(pthread_self(), &policy, &sp);
	r->policyBefore = policy;
	r->priorityBefore = sp.sched_priority;
	if (fifoPriority > 0 && policy != SCHED_FIFO && policy != SCHED_RR) {
		struct sched_param want = {};
		want.sched_priority = fifoPriority;
		r->schedError = pthread_setschedparam(pthread_self(), SCHED_FIFO, &want);
	}
	r->cpuAsked = cpu;
	if (cpu >= 0) {
		cpu_set_t set;
		CPU_ZERO(&set);
		CPU_SET(cpu, &set);
		r->affinityError = pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
	}
	pthread_getschedparam(pthread_self(), &policy, &sp);
	r->policyAfter = policy;
	r->priorityAfter = sp.sched_priority;
	r->cpuNow = sched_getcpu();
	r->done = 1;
}

/* Keeps the program's pages in memory (no page fault in the loop). From the
 * main thread, before the teleop. False if the system refuses. */
inline bool lockMemory() {
	return mlockall(MCL_CURRENT | MCL_FUTURE) == 0;
}

inline void print(const char* who, const Report& r) {
	if (!r.done) {
		printf("%s control thread: not configured yet (the loop has not run).\n", who);
		return;
	}
	printf("%s control thread: %s priority %d", who, policyName(r.policyAfter), r.priorityAfter);
	if (r.policyBefore != r.policyAfter) {
		printf(" (the library started it as %s)", policyName(r.policyBefore));
	}
	if (r.schedError != 0) {
		printf("; COULD NOT make it real-time (error %d: needs rtprio, e.g. the wam_ros container)", r.schedError);
	}
	if (r.cpuAsked >= 0) {
		if (r.affinityError != 0) {
			printf("; could not pin it to CPU %d (error %d)", r.cpuAsked, r.affinityError);
		} else {
			printf("; pinned to CPU %d", r.cpuAsked);
		}
	} else {
		printf("; not pinned (was on CPU %d)", r.cpuNow);
	}
	printf(".\n");
}

}  // namespace rt_thread
