#pragma once

// The simulated CAN bus of test_hand_teleop.cpp and test_hand_chain.cpp: a
// 7-DOF WAM (pucks 1-7) and a BarrettHand (pucks 11-14) speaking the Barrett
// protocol with libbarrett's property ids; libbarrett's CommunicationsBus on
// it; a stand-in for the follower's 500 Hz loop; checks.

#include <linux/can.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <barrett/bus/abstract/communications_bus.h>
#include <barrett/bus/bus_manager.h>
#include <barrett/products/hand.h>
#include <barrett/products/motor_puck.h>
#include <barrett/products/puck.h>
#include <barrett/products/puck_group.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/thread/real_time_mutex.h>

using barrett::MotorPuck;
using barrett::Puck;
using barrett::PuckGroup;
typedef std::chrono::steady_clock Clock;

namespace sim {

int checks = 0;
int failures = 0;

std::string strf(const char* format, ...) {
	char buf[1024];
	va_list ap;
	va_start(ap, format);
	vsnprintf(buf, sizeof(buf), format, ap);
	va_end(ap);
	return buf;
}

void check(bool ok, const std::string& what) {
	++checks;
	if (!ok) {
		++failures;
	}
	std::printf("  %s %s\n", ok ? "ok  " : "FAIL", what.c_str());
}

double secondsSince(Clock::time_point t0) {
	return std::chrono::duration<double>(Clock::now() - t0).count();
}

void sleepMs(double ms) {
	std::this_thread::sleep_for(std::chrono::microseconds(static_cast<long>(ms * 1000)));
}

double percentile(std::vector<double> v, double q) {
	if (v.empty()) {
		return 0.0;
	}
	std::sort(v.begin(), v.end());
	return v[std::min(v.size() - 1, static_cast<size_t>(q * (v.size() - 1) + 0.5))];
}

// A motor puck's property id, from libbarrett's tables.
int pid(enum Puck::Property p, int vers) {
	return Puck::getPropertyId(p, Puck::PT_Motor, vers);
}

const double FINGER_COUNTS_PER_RAD = 4096 / (2 * M_PI) * 125.0;  // 81487
const double SPREAD_COUNTS_PER_RAD = 4096 / (2 * M_PI) * 17.5;   // 11408

struct SimPuck {
	int id = 0;
	int vers = 175;
	int role = 0;
	int stat = 2;  // awake
	int mode = MotorPuck::MODE_IDLE;
	int v = 0;           // counts per ms
	double pos = 0.0;    // counts
	double target = 0.0;
	bool homing = false;
	double maxPos = 1e9;
	double speed = 300.0;  // counts per ms in a trapezoidal move
	double accel = 0.0;    // counts per ms^2 up to that speed and down at the end (0: none)
	double stopShort = 0.0;  // counts a move ends short of its target (inside MPE)
	double vel = 0.0;      // the move's speed now
	bool secondary = false;  // position replies carry an inner link encoder (6 bytes)
	int positionAnswersLeft = -1;  // position (P) gets it still answers (-1: all)
	double maxVel = 1e9;   // counts per ms it can go in velocity mode
	bool ignoresOpen = false;        // CMD open does nothing
	bool ignoresTrapezoidal = false;  // MODE trapezoidal does nothing
	double load = 0.0;     // counts per ms something pushes it while nothing holds it
	                       // (idle or torque mode): the backdrivable spread
	std::map<int, int32_t> props;
	Clock::time_point muteUntil;
	bool mute = false;
	bool stuck = false;  // does not move (a blocked finger)
	long vSets = 0;
	long modeVelocitySets = 0;
	long otherVSets = 0;  // sets of the other firmware's V id
	int lastVProperty = -1;
	long positionReads = 0;
	long cmdOpens = 0;
	long trapezoidalSets = 0;
	long pidSets = 0;
};
typedef std::map<int, SimPuck> Pucks;

canid_t canId(bool group, int from, int to) {
	return static_cast<canid_t>((group ? 0x400 : 0) | (from << 5) | to);
}

// The bus: pucks 1-7 (a WAM, firmware 200) and 11-14 (the hand), and any
// number of programs, each behind its CAN socket's receive filter. Replies go
// to every program whose filter passes them, as on a real bus.
class SimCan {
public:
	typedef std::function<bool(canid_t)> Filter;

	explicit SimCan(int handVers) : running(false) {
		for (int id = 1; id <= 7; ++id) {
			add(id, 200, 0 /* ROLE_TATER */, 1e9);
		}
		for (int id = 11; id <= 14; ++id) {
			// ROLE_BHAND; the fingers have strain gauges (RO_Strain)
			add(id, handVers, 5 | (id < 14 ? 0x0800 : 0), id < 14 ? 199111 : 35840);
			SimPuck& p = pucks[id];
			// the BH8 manual: HOLD 1 for the spread, 0 for the fingers; a
			// finger fully open to closed in 1.0 s, the spread through 180
			// degrees in 0.3 s
			p.props[pid(Puck::HOLD, handVers)] = id == 14 ? 1 : 0;
			p.speed = id == 14 ? 120.0 : 200.0;
		}
	}
	~SimCan() {
		stop();
		for (size_t k = 0; k < hosts.size(); ++k) {
			::close(hosts[k]->simFd);
		}
	}

	// A program on the bus; returns its socket.
	int addHost(Filter filter) {
		int sv[2];
		if (socketpair(AF_UNIX, SOCK_SEQPACKET, 0, sv) != 0) {
			std::perror("socketpair");
			std::abort();
		}
		std::unique_ptr<Host> h(new Host);
		h->simFd = sv[1];
		h->filter = filter;
		std::lock_guard<std::mutex> lock(mutex);
		hosts.push_back(std::move(h));
		return sv[0];
	}

	void start() {
		running = true;
		thread = std::thread(&SimCan::run, this);
	}
	void stop() {
		running = false;
		if (thread.joinable()) {
			thread.join();
		}
	}

	template <typename F> auto with(F f) {
		std::lock_guard<std::mutex> lock(mutex);
		return f(pucks);
	}
	double pos(int id) {
		return with([&](Pucks& p) { return p[id].pos; });
	}
	int mode(int id) {
		return with([&](Pucks& p) { return p[id].mode; });
	}

	// The force/torque sensor (puck 8): a get of its FT property is answered
	// with a force frame and a torque frame, three int16 each.
	std::atomic<long> ftRequests{0};
	std::atomic<bool> ftAnswers{true};
	std::atomic<int> ftCopies{1};  // replies per request
	std::atomic<int> ftForceCounts[3] = {{0}, {0}, {0}};   // N * 256
	std::atomic<int> ftTorqueCounts[3] = {{0}, {0}, {0}};  // Nm * 4096

	std::atomic<long> handFrames{0};   // host frames to the hand
	std::atomic<long> armFrames{0};    // host frames to the WAM
	std::atomic<long> handReplies{0};
	std::atomic<long> armReplies{0};
	std::atomic<long> violations{0};   // host frames to anything else
	std::atomic<long> dropped{0};

private:
	struct Host {
		int simFd;
		Filter filter;
	};

	void add(int id, int vers, int role, double maxPos) {
		SimPuck& p = pucks[id];
		p.id = id;
		p.vers = vers;
		p.role = role;
		p.maxPos = maxPos;
	}

	void run() {
		Clock::time_point last = Clock::now();
		std::vector<pollfd> pfds;
		while (running) {
			{
				std::lock_guard<std::mutex> lock(mutex);
				pfds.clear();
				for (size_t k = 0; k < hosts.size(); ++k) {
					pfds.push_back(pollfd{hosts[k]->simFd, POLLIN, 0});
				}
			}
			if (poll(pfds.data(), pfds.size(), 1) > 0) {
				for (size_t k = 0; k < pfds.size(); ++k) {
					if (!(pfds[k].revents & POLLIN)) {
						continue;
					}
					can_frame f;
					while (recv(pfds[k].fd, &f, sizeof(f), MSG_DONTWAIT) == static_cast<ssize_t>(sizeof(f))) {
						std::lock_guard<std::mutex> lock(mutex);
						handle(k, f);
					}
				}
			}
			const Clock::time_point now = Clock::now();
			const double dtMs = std::chrono::duration<double, std::milli>(now - last).count();
			if (dtMs >= 1.0) {
				std::lock_guard<std::mutex> lock(mutex);
				tick(dtMs);
				last = now;
			}
		}
	}

	void handle(size_t fromHost, const can_frame& f) {
		const canid_t id = f.can_id & CAN_SFF_MASK;
		// A frame reaches the other programs too (SocketCAN loopback), through
		// their filters.
		for (size_t k = 0; k < hosts.size(); ++k) {
			if (k != fromHost && hosts[k]->filter(id)) {
				deliver(k, f);
			}
		}
		if (((id >> 5) & 0x1f) != 0) {
			return;
		}
		const bool group = (id & 0x400) != 0;
		const int to = id & 0x1f;
		std::vector<int> targets;
		if (group && to == 5) {
			targets = {11, 12, 13, 14};
		} else if (group && to == 4) {
			targets = {1, 2, 3, 4, 5, 6, 7};
		} else if (group && (to == 1 || to == 2)) {
			++armFrames;  // packed torques
			return;
		} else if (!group && ((to >= 1 && to <= 7) || (to >= 11 && to <= 14))) {
			targets = {to};
		} else if (!group && to == 8) {
			handleFt(f);
			return;
		} else {
			++violations;
			return;
		}
		if (targets[0] >= 11) {
			++handFrames;
		} else {
			++armFrames;
		}
		for (size_t i = 0; i < targets.size(); ++i) {
			SimPuck& p = pucks[targets[i]];
			if (f.data[0] & 0x80) {
				set(p, f);
			} else {
				get(p, f);
			}
		}
	}

	void handleFt(const can_frame& f) {
		static const int FT = Puck::getPropertyId(Puck::FT, Puck::PT_ForceTorque, 0);
		if ((f.data[0] & 0x80) != 0 || (f.data[0] & 0x7f) != FT) {
			return;  // a set (the tare), or another property
		}
		++ftRequests;
		if (!ftAnswers) {
			return;
		}
		for (int c = 0; c < ftCopies; ++c) {
			for (int which = 0; which < 2; ++which) {
				can_frame r{};
				r.can_id = canId(true, 8, which == 0 ? 10 : 11);  // FGRP_FT_FORCE, FGRP_FT_TORQUE
				r.can_dlc = 6;
				for (int i = 0; i < 3; ++i) {
					const int v = which == 0 ? ftForceCounts[i].load() : ftTorqueCounts[i].load();
					r.data[2 * i] = static_cast<uint8_t>(v & 0xff);
					r.data[2 * i + 1] = static_cast<uint8_t>((v >> 8) & 0xff);
				}
				for (size_t k = 0; k < hosts.size(); ++k) {
					if (hosts[k]->filter(r.can_id)) {
						deliver(k, r);
					}
				}
			}
		}
	}

	void set(SimPuck& p, const can_frame& f) {
		const int prop = f.data[0] & 0x7f;
		const int32_t value = f.can_dlc >= 6
				? static_cast<int32_t>(f.data[2] | (f.data[3] << 8) | (f.data[4] << 16)
						| (static_cast<uint32_t>(f.data[5]) << 24))
				: static_cast<int16_t>(f.data[2] | (f.data[3] << 8));
		if (prop == pid(Puck::STAT, p.vers)) {
			p.stat = value;
			return;
		}
		if (p.stat != 2) {
			return;  // asleep: STAT only
		}
		if (prop == pid(Puck::MODE, p.vers)) {
			if (value == MotorPuck::MODE_TRAPEZOIDAL && p.ignoresTrapezoidal) {
				return;
			}
			p.mode = value;
			p.homing = false;
			if (value == MotorPuck::MODE_VELOCITY) {
				++p.modeVelocitySets;
			} else if (value == MotorPuck::MODE_TRAPEZOIDAL) {
				++p.trapezoidalSets;
				p.vel = 0.0;
			} else if (value == MotorPuck::MODE_PID) {
				++p.pidSets;
			}
		} else if (prop == pid(Puck::V, p.vers)) {
			p.v = value;
			++p.vSets;
			p.lastVProperty = prop;
		} else if (prop == 44 || prop == 106) {
			++p.otherVSets;  // another firmware's V
			p.lastVProperty = prop;
			p.props[prop] = value;
		} else if (prop == pid(Puck::E, p.vers)) {
			p.target = value;
		} else if (prop == pid(Puck::CMD, p.vers)) {
			if (value == 13) {  // HI: drive open, zero; no CAN for 0.5 s
				p.mode = MotorPuck::MODE_TRAPEZOIDAL;
				p.homing = true;
				p.muteUntil = Clock::now() + std::chrono::milliseconds(500);
			} else if (value == 20 && !p.ignoresOpen) {  // open
				p.mode = MotorPuck::MODE_TRAPEZOIDAL;
				p.target = 0;
				p.vel = 0.0;
				++p.cmdOpens;
			} else if (value == 18) {  // close
				p.mode = MotorPuck::MODE_TRAPEZOIDAL;
				p.target = p.maxPos;
				p.vel = 0.0;
			}
		} else {
			p.props[prop] = value;
		}
	}

	void get(SimPuck& p, const can_frame& f) {
		const int prop = f.data[0] & 0x7f;
		if (p.mute || Clock::now() < p.muteUntil) {
			return;
		}
		if (p.stat != 2 && prop != pid(Puck::STAT, p.vers)) {
			return;
		}
		if (prop == pid(Puck::P, p.vers)) {
			if (p.positionAnswersLeft == 0) {
				return;
			}
			if (p.positionAnswersLeft > 0) {
				--p.positionAnswersLeft;
			}
			++p.positionReads;
			const int32_t c = static_cast<int32_t>(std::lround(p.pos));
			can_frame r{};
			r.can_id = canId(true, p.id, 3);
			r.can_dlc = p.secondary ? 6 : 3;
			r.data[0] = static_cast<uint8_t>(0x80 | ((c >> 16) & 0x3f));
			r.data[1] = static_cast<uint8_t>((c >> 8) & 0xff);
			r.data[2] = static_cast<uint8_t>(c & 0xff);
			if (p.secondary) {  // the inner link: Hand::J2_ENCODER_RATIO 50 of J2_RATIO 125
				const int32_t e = static_cast<int32_t>(std::lround(p.pos * 50.0 / 125.0));
				r.data[3] = static_cast<uint8_t>((e >> 16) & 0x3f);
				r.data[4] = static_cast<uint8_t>((e >> 8) & 0xff);
				r.data[5] = static_cast<uint8_t>(e & 0xff);
			}
			reply(p, r);
			return;
		}
		int32_t value = 0;
		if (prop == pid(Puck::STAT, p.vers)) {
			value = p.stat;
		} else if (prop == pid(Puck::MODE, p.vers)) {
			value = p.mode;
		} else if (prop == pid(Puck::V, p.vers)) {
			value = p.v;
		} else if (prop == pid(Puck::ROLE, p.vers)) {
			value = p.role;
		} else if (prop == pid(Puck::VERS, p.vers)) {
			value = p.vers;
		} else if (prop == pid(Puck::CTS, p.vers)) {
			value = 4096;
		} else if (prop == pid(Puck::IPNM, p.vers)) {
			value = 2700;
		} else {
			std::map<int, int32_t>::const_iterator it = p.props.find(prop);
			value = it == p.props.end() ? 0 : it->second;
		}
		can_frame r{};
		r.can_id = canId(true, p.id, 6);
		r.can_dlc = 6;
		r.data[0] = static_cast<uint8_t>(0x80 | prop);
		r.data[1] = 0;
		for (int b = 0; b < 4; ++b) {
			r.data[2 + b] = static_cast<uint8_t>((static_cast<uint32_t>(value) >> (8 * b)) & 0xff);
		}
		reply(p, r);
	}

	void reply(const SimPuck& p, const can_frame& r) {
		if (p.id >= 11) {
			++handReplies;
		} else {
			++armReplies;
		}
		for (size_t k = 0; k < hosts.size(); ++k) {
			if (hosts[k]->filter(r.can_id)) {
				deliver(k, r);
			}
		}
	}

	void deliver(size_t k, const can_frame& f) {
		// a full CAN socket queue drops frames, it does not block the bus
		if (::send(hosts[k]->simFd, &f, sizeof(f), MSG_DONTWAIT) < 0) {
			++dropped;
		}
	}

	// The end of a move: the puck servos where it is (MODE_PID) if HOLD is
	// set, else it lets go (idle).
	void endMove(SimPuck& p) {
		std::map<int, int32_t>::const_iterator h = p.props.find(pid(Puck::HOLD, p.vers));
		p.mode = h != p.props.end() && h->second != 0 ? MotorPuck::MODE_PID : MotorPuck::MODE_IDLE;
	}

	void tick(double dtMs) {
		for (Pucks::iterator it = pucks.begin(); it != pucks.end(); ++it) {
			SimPuck& p = it->second;
			if (p.stat != 2 || p.stuck) {
				continue;
			}
			if (p.mode == MotorPuck::MODE_VELOCITY) {
				const double v = std::max(-p.maxVel, std::min(p.maxVel, static_cast<double>(p.v)));
				p.pos = std::max(0.0, std::min(p.maxPos, p.pos + v * dtMs));
			} else if (p.mode == MotorPuck::MODE_TRAPEZOIDAL) {
				if (p.homing) {
					p.pos = std::max(0.0, p.pos - 100.0 * dtMs);
					if (p.pos <= 0.0) {
						p.homing = false;
						endMove(p);
					}
				} else {
					const double d = p.target - p.pos;
					const double left = std::max(0.0, std::fabs(d) - p.stopShort);
					double v = p.speed;
					if (p.accel > 0.0) {  // up at accel, and down to stop there
						v = std::min(v, std::min(p.vel + p.accel * dtMs, std::sqrt(2.0 * p.accel * left) + p.accel * dtMs));
					}
					p.vel = v;
					p.pos += std::copysign(std::min(v * dtMs, left), d);
					if (std::fabs(p.target - p.pos) - p.stopShort < 1.0) {
						endMove(p);
						p.vel = 0.0;
					}
				}
			} else if (p.mode == MotorPuck::MODE_IDLE || p.mode == MotorPuck::MODE_TORQUE) {
				p.pos = std::max(0.0, std::min(p.maxPos, p.pos + p.load * dtMs));
			}  // MODE_PID: held
		}
	}

	Pucks pucks;
	std::vector<std::unique_ptr<Host>> hosts;
	std::mutex mutex;
	std::thread thread;
	std::atomic<bool> running;
};

// The follower's CAN socket filter (libbarrett CANSocket): everything not
// sent by a host.
bool followerFilter(canid_t id) {
	return ((id >> 5) & 0x1f) != 0;
}
// libgripper's BarrettHandCanDriver filter: only the hand's pucks.
bool handOnlyFilter(canid_t id) {
	const int from = (id >> 5) & 0x1f;
	return from >= 11 && from <= 14;
}

// libbarrett's CommunicationsBus on a simulator socket, like CANSocket.
class SimBus : public barrett::bus::CommunicationsBus {
public:
	explicit SimBus(int fd) : fd(fd) {}
	virtual ~SimBus() { ::close(fd); }

	virtual barrett::thread::RealTimeMutex& getMutex() const { return mutex; }
	virtual void open(int) {}
	virtual void close() {}
	virtual bool isOpen() const { return true; }

	virtual int send(int busId, const unsigned char* data, size_t len) const {
		BARRETT_SCOPED_LOCK(mutex);
		can_frame f{};
		f.can_id = busId;
		f.can_dlc = len;
		std::memcpy(f.data, data, len);
		return ::send(fd, &f, sizeof(f), 0) == static_cast<ssize_t>(sizeof(f)) ? 0 : 2;
	}
	virtual int receiveRaw(int& busId, unsigned char* data, size_t& len, bool blocking) const {
		BARRETT_SCOPED_LOCK(mutex);
		can_frame f;
		const ssize_t r = ::recv(fd, &f, sizeof(f), blocking ? 0 : MSG_DONTWAIT);
		if (r < 0) {
			return (errno == EAGAIN || errno == EWOULDBLOCK) ? 1 : 2;
		}
		if (r != static_cast<ssize_t>(sizeof(f))) {
			return 2;
		}
		busId = f.can_id;
		len = f.can_dlc;
		std::memcpy(data, f.data, len);
		return 0;
	}

private:
	int fd;
	mutable barrett::thread::RealTimeMutex mutex;
};

// The follower's 500 Hz loop as the bus sees it (LowLevelWam::update and
// setTorques): a position read of group 4, then packed torques to groups 1
// and 2. Records how long each cycle's bus work takes.
class ArmLoop {
public:
	explicit ArmLoop(const barrett::bus::CommunicationsBus& bus) : failed(false), bus(bus), running(false) {
		for (int id = 1; id <= 7; ++id) {
			pucks.push_back(new Puck(bus, id));
		}
		group.reset(new PuckGroup(PuckGroup::BGRP_WAM, pucks));
		lower.reset(new PuckGroup(PuckGroup::BGRP_LOWER_WAM, std::vector<Puck*>(pucks.begin(), pucks.begin() + 4)));
		upper.reset(new PuckGroup(PuckGroup::BGRP_UPPER_WAM, std::vector<Puck*>(pucks.begin() + 4, pucks.end())));
		torqueProp = pucks[0]->getPropertyId(Puck::T);
	}
	~ArmLoop() {
		stop();
		for (size_t i = 0; i < pucks.size(); ++i) {
			delete pucks[i];
		}
	}

	void start() {
		running = true;
		thread = std::thread(&ArmLoop::run, this);
	}
	void stop() {
		running = false;
		if (thread.joinable()) {
			thread.join();
		}
	}
	void mark() {
		std::lock_guard<std::mutex> lock(mutex);
		times.clear();
	}
	std::vector<double> snapshot() {
		std::lock_guard<std::mutex> lock(mutex);
		return times;
	}

	std::atomic<bool> failed;
	std::string error;
	Clock::time_point failedAt;

private:
	void run() {
		MotorPuck::CombinedPositionParser<double>::result_type positions[7];
		const double pt[7] = {0, 0, 0, 0, 0, 0, 0};
		Clock::time_point next = Clock::now();
		while (running) {
			next += std::chrono::milliseconds(2);
			const Clock::time_point a = Clock::now();
			try {
				group->getProperty<MotorPuck::CombinedPositionParser<double> >(Puck::P, positions, false);
				BARRETT_SCOPED_LOCK(bus.getMutex());
				MotorPuck::sendPackedTorques(bus, lower->getId(), torqueProp, pt, 4);
				MotorPuck::sendPackedTorques(bus, upper->getId(), torqueProp, pt + 4, 3);
			} catch (const std::exception& e) {
				error = e.what();
				failedAt = Clock::now();
				failed = true;
				return;
			}
			{
				std::lock_guard<std::mutex> lock(mutex);
				times.push_back(std::chrono::duration<double, std::milli>(Clock::now() - a).count());
			}
			const Clock::time_point now = Clock::now();
			if (now < next) {
				std::this_thread::sleep_until(next);
			} else {
				next = now;
			}
		}
	}

	const barrett::bus::CommunicationsBus& bus;
	std::vector<Puck*> pucks;
	std::unique_ptr<PuckGroup> group, lower, upper;
	int torqueProp;
	std::atomic<bool> running;
	std::thread thread;
	std::mutex mutex;
	std::vector<double> times;
};

std::string timing(const std::vector<double>& t) {
	return strf("p50 %.2f ms, p99 %.2f ms, max %.2f ms over %zu cycles", percentile(t, 0.5), percentile(t, 0.99),
			percentile(t, 1.0), t.size());
}

// The follower's side: its bus manager, and optionally the arm's loop and the
// Hand, as in follower --hand.
struct Rig {
	explicit Rig(int handVers) : sim(handVers), fd(sim.addHost(followerFilter)), bus(fd), bm(&bus) {
		sim.start();
	}
	~Rig() {
		arm.reset();
		hand.reset();
		for (size_t i = 0; i < handPucks.size(); ++i) {
			delete handPucks[i];
		}
		sim.stop();
	}
	void startArm() {
		arm.reset(new ArmLoop(bm));
		arm->start();
	}
	void makeHand() {
		for (int id = 11; id <= 14; ++id) {
			handPucks.push_back(new Puck(bm, id));
		}
		hand.reset(new barrett::Hand(handPucks));
	}

	SimCan sim;
	int fd;
	SimBus bus;
	barrett::bus::BusManager bm;
	std::vector<Puck*> handPucks;
	std::unique_ptr<barrett::Hand> hand;
	std::unique_ptr<ArmLoop> arm;
};

}  // namespace sim
