#pragma once

/* The UDP link between the two arm programs (leader_wam8g, follower), read
 * and written by the control loop itself.
 *
 * udp_handler.h (which the older leader programs still use) passes every
 * packet through a helper thread and a mutex in each direction: the loop
 * wakes a normal-priority thread to send, and takes a lock that a
 * normal-priority receive thread may hold. Both helpers are scheduled like
 * any desktop program, so a packet usually took 12 us and now and then 0.3-1
 * ms, and a preempted helper could make the loop wait. Here the loop calls
 * sendto() and recvfrom() itself, non-blocking: no helper thread, no lock.
 *
 * Each packet carries the sender's cycle counter and the CLOCK_MONOTONIC time
 * of the sample it holds. On one computer (the usual 127.0.0.1) both
 * programs read the same clock, so the receiver knows how old the sample is
 * (Received::timed) and can count lost packets. It also carries the teleop
 * mode (teleop_law::Mode) and a change counter, so a mode change reaches the
 * other arm without ROS, and flags: whether the sender is linked, and whether
 * its external torques are measured (the follower's force/torque sensor).
 *
 * A program that still uses udp_handler.h sends only the three vectors; such
 * packets are accepted (not timed), and this side then answers in that format
 * too, so `follower` keeps working with the older leaders. */

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

#include <Eigen/Core>

namespace teleop_link {

const uint32_t MAGIC = 0x324B4C54;  // "TLK2" (TLK1 had no flags: both programs must be the same build)

/* Header::flags */
const uint32_t FLAG_LINKED = 1u;          // the sender is linked (its external torques are meant)
const uint32_t FLAG_MEASURED_ARM = 2u;    // its J1-J4 external torques are from a force/torque sensor
const uint32_t FLAG_MEASURED_WRIST = 4u;  // its wrist slots hold external torques, from that sensor

/* Seconds on CLOCK_MONOTONIC: the same clock in every program (and container)
 * on one computer. */
inline double now() {
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return static_cast<double>(ts.tv_sec) + 1e-9 * ts.tv_nsec;
}

#pragma pack(push, 1)
struct Header {
	uint32_t magic;
	uint32_t dof;
	uint64_t sequence;    // +1 every control cycle of the sender
	double sampleTime;    // sender's CLOCK_MONOTONIC when it sampled its joints, s
	uint32_t mode;        // teleop_law::Mode
	uint32_t modeEpoch;   // +1 at every mode change; the larger one wins
	uint32_t flags;       // FLAG_*
	uint32_t reserved;
};
#pragma pack(pop)

template <size_t DOF>
class Link {
public:
	typedef Eigen::Matrix<double, DOF, 1> vec;

	struct Received {
		vec jp, jv, extTorque;
		double sampleTime;  // the sender's clock (meaningful only when timed)
		double arrival;     // this program's clock when poll() read it
		uint64_t sequence;  // 0 from a legacy sender
		uint32_t mode, modeEpoch;
		uint32_t flags;     // FLAG_* (0 from a legacy sender)
		bool timed;         // sampleTime is on this computer's clock
	};

	/* Counters, for the log and the summary at exit. */
	struct Stats {
		uint64_t sent, sendErrors, received, rejected, skipped, lost;
		Stats() : sent(0), sendErrors(0), received(0), rejected(0), skipped(0), lost(0) {}
	};

	Link(const std::string& remoteHost, int sendPort, int recvPort)
		: sendFd(-1), recvFd(-1), txSequence(0), haveLatest(false), peerLegacy(false), sameClock(false) {
		std::memset(&remote, 0, sizeof(remote));
		remote.sin_family = AF_INET;
		remote.sin_port = htons(static_cast<uint16_t>(sendPort));
		if (inet_pton(AF_INET, remoteHost.c_str(), &remote.sin_addr) != 1) {
			throw std::runtime_error("teleop link: \"" + remoteHost + "\" is not an IPv4 address");
		}
		sameClock = (ntohl(remote.sin_addr.s_addr) >> 24) == 127;

		recvFd = openSocket();
		struct sockaddr_in local;
		std::memset(&local, 0, sizeof(local));
		local.sin_family = AF_INET;
		local.sin_addr.s_addr = htonl(INADDR_ANY);
		local.sin_port = htons(static_cast<uint16_t>(recvPort));
		if (bind(recvFd, reinterpret_cast<struct sockaddr*>(&local), sizeof(local)) != 0) {
			const int e = errno;
			close(recvFd);
			throw std::runtime_error("teleop link: cannot listen on UDP port " + std::to_string(recvPort) + ": " +
			                         std::strerror(e) + " (is the other copy of this program still running?)");
		}
		sendFd = openSocket();
		latestRx.jp.setZero();
		latestRx.jv.setZero();
		latestRx.extTorque.setZero();
		latestRx.sampleTime = latestRx.arrival = 0.0;
		latestRx.sequence = 0;
		latestRx.mode = latestRx.modeEpoch = 0;
		latestRx.flags = 0;
		latestRx.timed = false;
	}

	~Link() {
		if (sendFd >= 0) close(sendFd);
		if (recvFd >= 0) close(recvFd);
	}

	/* One packet: this cycle's joints. Never blocks. False if the kernel did
	 * not take it (counted). */
	bool send(const vec& jp, const vec& jv, const vec& extTorque, double sampleTime, uint32_t mode,
	          uint32_t modeEpoch, uint32_t flags = 0) {
		unsigned char buffer[sizeof(Header) + 3 * DOF * sizeof(double)];
		size_t offset = 0;
		if (!peerLegacy) {
			Header h;
			h.magic = MAGIC;
			h.dof = static_cast<uint32_t>(DOF);
			h.sequence = ++txSequence;
			h.sampleTime = sampleTime;
			h.mode = mode;
			h.modeEpoch = modeEpoch;
			h.flags = flags;
			h.reserved = 0;
			std::memcpy(buffer, &h, sizeof(h));
			offset = sizeof(h);
		}
		std::memcpy(buffer + offset, jp.data(), DOF * sizeof(double));
		std::memcpy(buffer + offset + DOF * sizeof(double), jv.data(), DOF * sizeof(double));
		std::memcpy(buffer + offset + 2 * DOF * sizeof(double), extTorque.data(), DOF * sizeof(double));
		const size_t length = offset + 3 * DOF * sizeof(double);
		const ssize_t n = sendto(sendFd, buffer, length, MSG_DONTWAIT,
		                         reinterpret_cast<const struct sockaddr*>(&remote), sizeof(remote));
		if (n != static_cast<ssize_t>(length)) {
			++stats.sendErrors;
			return false;
		}
		++stats.sent;
		return true;
	}

	/* Reads every packet that has arrived and keeps the newest. True when at
	 * least one valid packet arrived since the last call. Never blocks. */
	bool poll(double timeNow) {
		unsigned char buffer[sizeof(Header) + 3 * DOF * sizeof(double) + 8];
		bool arrived = false;
		for (int i = 0; i < MAX_PER_POLL; ++i) {
			const ssize_t n = recv(recvFd, buffer, sizeof(buffer), MSG_DONTWAIT);
			if (n < 0) {
				break;  // nothing more (EAGAIN), or an error: try again next cycle
			}
			const size_t vectors = 3 * DOF * sizeof(double);
			size_t offset;
			Received rx;
			if (static_cast<size_t>(n) == sizeof(Header) + vectors) {
				Header h;
				std::memcpy(&h, buffer, sizeof(h));
				if (h.magic != MAGIC || h.dof != DOF) {
					++stats.rejected;
					continue;
				}
				offset = sizeof(Header);
				rx.sequence = h.sequence;
				rx.sampleTime = h.sampleTime;
				rx.mode = h.mode;
				rx.modeEpoch = h.modeEpoch;
				rx.flags = h.flags;
				rx.timed = sameClock;
				peerLegacy = false;
			} else if (static_cast<size_t>(n) == vectors) {  // a udp_handler.h sender
				offset = 0;
				rx.sequence = 0;
				rx.sampleTime = 0.0;
				rx.mode = 0;
				rx.modeEpoch = 0;
				rx.flags = 0;
				rx.timed = false;
				peerLegacy = true;
			} else {
				++stats.rejected;
				continue;
			}
			std::memcpy(rx.jp.data(), buffer + offset, DOF * sizeof(double));
			std::memcpy(rx.jv.data(), buffer + offset + DOF * sizeof(double), DOF * sizeof(double));
			std::memcpy(rx.extTorque.data(), buffer + offset + 2 * DOF * sizeof(double), DOF * sizeof(double));
			bool finite = true;
			for (size_t k = 0; k < DOF; ++k) {
				finite = finite && rx.jp[k] == rx.jp[k] && rx.jv[k] == rx.jv[k] && rx.extTorque[k] == rx.extTorque[k];
			}
			if (!finite) {  // a NaN must never become a position reference
				++stats.rejected;
				continue;
			}
			rx.arrival = timeNow;
			if (arrived) {
				++stats.skipped;  // two in one cycle: the older one is not used
			}
			if (haveLatest && rx.sequence > latestRx.sequence + 1 && latestRx.sequence != 0) {
				stats.lost += rx.sequence - latestRx.sequence - 1;
			}
			latestRx = rx;
			haveLatest = true;
			arrived = true;
			++stats.received;
		}
		return arrived;
	}

	bool valid() const { return haveLatest; }
	const Received& latest() const { return latestRx; }
	const Stats& getStats() const { return stats; }
	/* The other program is on this computer (its sample times are on this clock). */
	bool sharedClock() const { return sameClock; }
	/* The other program speaks the older, untimed format. */
	bool legacyPeer() const { return peerLegacy; }

private:
	static const int MAX_PER_POLL = 16;

	static int openSocket() {
		const int fd = socket(AF_INET, SOCK_DGRAM, 0);
		if (fd < 0) {
			throw std::runtime_error(std::string("teleop link: socket(): ") + std::strerror(errno));
		}
		const int flags = fcntl(fd, F_GETFL, 0);
		fcntl(fd, F_SETFL, flags | O_NONBLOCK);
		fcntl(fd, F_SETFD, FD_CLOEXEC);
		return fd;
	}

	int sendFd, recvFd;
	struct sockaddr_in remote;
	uint64_t txSequence;
	Received latestRx;
	bool haveLatest;
	bool peerLegacy;
	bool sameClock;
	Stats stats;

	Link(const Link&);
	Link& operator=(const Link&);
};

}  // namespace teleop_link
