#pragma once

/* Joint map between the wam8g wrist (J6-J8) and a P2 WAM wrist (J5-J7),
 * anchored at the two sync poses:
 *   follower = follower_sync + sign * (leader - leader_sync).
 *
 * coupled() maps joint for joint, as chosen on the robot: wam8g J6 -> P2 J5,
 * J7 -> P2 J6, J8 -> P2 J7; the passive wam8g J5 maps to nothing. At the P2's
 * sync wrist (J5 -pi/2, J6 0, J7 +pi/2; wam8g_teleop.h) its J5 and J7 turn
 * about the same axis, the forearm, and its J6 axis is parallel to the wam8g's
 * J7 axis (at J5 = 0 it was 90 degrees off). J7 -> P2 J6 is reversed: with
 * +1 the leader's J7 drove the follower's J6 the wrong way (the user,
 * 2026-09-28). J6 -> J5 and J8 -> J7 stay +1. The same sign maps positions
 * and velocities to the follower and the follower's wrist back to the
 * leader. */

#include "wam8g_teleop.h"

namespace wam8g_teleop {

struct WristMap {
	/* For follower wrist joint k (J5, J6, J7): the wam8g joint feeding it as
	 * an index into J1-J8 (5 = J6, 6 = J7, 7 = J8), or -1: held at its sync
	 * angle. */
	int source[3];
	double sign[3];

	/* Both wrists hold their sync angles; only J1-J4 are coupled. */
	static WristMap held() {
		WristMap m = {{-1, -1, -1}, {1.0, 1.0, 1.0}};
		return m;
	}
	/* wam8g J6 -> P2 J5, J7 -> P2 J6 (reversed), J8 -> P2 J7. */
	static WristMap coupled() {
		WristMap m = {{5, 6, 7}, {1.0, -1.0, 1.0}};
		return m;
	}

	bool couplesAnything() const {
		return source[0] >= 0 || source[1] >= 0 || source[2] >= 0;
	}

	/* Leader positions (J1-J8) -> follower wrist positions (J5-J7). */
	template <typename Leader, typename Wrist>
	void toFollower(const Leader& leader, Wrist* follower) const {
		for (int k = 0; k < 3; ++k) {
			const int s = source[k];
			(*follower)[k] = FOLLOWER_SYNC_POSE[4 + k] + (s < 0 ? 0.0 : sign[k] * (leader[s] - SYNC_POSE[s]));
		}
	}

	/* Leader velocities (J1-J8) -> follower wrist velocities (J5-J7). */
	template <typename Leader, typename Wrist>
	void velocityToFollower(const Leader& leader, Wrist* follower) const {
		for (int k = 0; k < 3; ++k) {
			const int s = source[k];
			(*follower)[k] = s < 0 ? 0.0 : sign[k] * leader[s];
		}
	}

	/* Follower wrist positions (J5-J7) -> the leader's J6-J8 reference
	 * (entries 5-7 of a J1-J8 vector; the rest is left alone). */
	template <typename Wrist, typename Leader>
	void toLeader(const Wrist& follower, Leader* leader) const {
		for (int j = 5; j < 8; ++j) {
			(*leader)[j] = SYNC_POSE[j];
		}
		for (int k = 0; k < 3; ++k) {
			const int s = source[k];
			if (s >= 0) {
				(*leader)[s] = SYNC_POSE[s] + sign[k] * (follower[k] - FOLLOWER_SYNC_POSE[4 + k]);
			}
		}
	}
};

}  // namespace wam8g_teleop
