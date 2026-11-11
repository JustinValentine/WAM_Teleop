/* Offline checks of the low-latency pieces (2026-11-09), without libbarrett or
 * a robot: the link the control loops use (lib/teleop_link.h), the mode and
 * the aged reference it carries (lib/teleop_law.h), the recording
 * (lib/teleop_recorder.h), the estimator's arithmetic (lib/estimator_core.h)
 * and the control thread's setup (lib/rt_thread.h); and the force/torque
 * sensor's arithmetic (lib/ft_core.h).
 * Run: devel/lib/wam_teleop/low_latency_tests
 *      low_latency_tests --ft-samples FILE   write a made-up gripper's free-space
 *                                            readings as a recording, for ft_tool_cal.py
 *      low_latency_tests --ft-check TOOLFILE  check the file it fitted from them */

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include "estimator_core.h"
#include "follower_beta_zeus_hand.h"
#include "ft_core.h"
#ifdef __linux__
#include "rt_thread.h"
#endif
#include "teleop_law.h"
#include "teleop_link.h"
#include "teleop_recorder.h"

namespace {

int failures = 0;
int checks = 0;

void check(bool ok, const char* what) {
	++checks;
	if (!ok) {
		++failures;
		printf("FAIL: %s\n", what);
	}
}

typedef teleop_link::Link<7> Link;
typedef Link::vec vec7;

void testLink(int port) {
	printf("link on ports %d/%d\n", port, port + 1);
	Link a("127.0.0.1", port, port + 1), b("127.0.0.1", port + 1, port);
	check(a.sharedClock() && b.sharedClock(), "link: 127.0.0.1 shares this computer's clock");
	check(!b.poll(teleop_link::now()) && !b.valid(), "link: nothing before the first packet");

	vec7 jp, jv, ext;
	for (int i = 0; i < 7; ++i) {
		jp[i] = 0.1 * i;
		jv[i] = -0.2 * i;
		ext[i] = 3.0 + i;
	}
	const double t0 = teleop_link::now();
	check(a.send(jp, jv, ext, t0, 2, 7, teleop_link::FLAG_LINKED | teleop_link::FLAG_MEASURED_WRIST), "link: send");
	usleep(2000);
	const double t1 = teleop_link::now();
	check(b.poll(t1) && b.valid(), "link: the packet arrives");
	const Link::Received& rx = b.latest();
	bool same = true;
	for (int i = 0; i < 7; ++i) {
		same = same && rx.jp[i] == jp[i] && rx.jv[i] == jv[i] && rx.extTorque[i] == ext[i];
	}
	check(same, "link: the three vectors arrive unchanged");
	check(rx.timed && rx.sampleTime == t0 && rx.arrival == t1 && rx.sequence == 1, "link: sample time, arrival, sequence");
	check(rx.flags == (teleop_link::FLAG_LINKED | teleop_link::FLAG_MEASURED_WRIST), "link: the flags arrive");
	check(rx.mode == 2 && rx.modeEpoch == 7, "link: the mode and its counter");
	check(!b.poll(teleop_link::now()), "link: no new packet, none reported");

	// Two packets before one poll: the newest is kept, the other counted.
	a.send(jp, jv, ext, t0, 0, 0);
	jp[0] = 9.0;
	a.send(jp, jv, ext, t0, 0, 0);
	usleep(2000);
	check(b.poll(teleop_link::now()) && b.latest().jp[0] == 9.0 && b.latest().sequence == 3, "link: the newest of two");
	check(b.getStats().skipped == 1 && b.getStats().received == 3 && b.getStats().lost == 0, "link: skipped is counted");

	// A NaN never becomes a reference.
	vec7 bad(jp);
	bad[3] = std::numeric_limits<double>::quiet_NaN();
	a.send(bad, jv, ext, t0, 0, 0);
	usleep(2000);
	check(!b.poll(teleop_link::now()) && b.getStats().rejected == 1 && b.latest().jp[0] == 9.0, "link: a NaN packet is rejected");
	// ... and its sequence number (4) then counts as lost when the next arrives.
	a.send(jp, jv, ext, t0, 0, 0);
	usleep(2000);
	check(b.poll(teleop_link::now()) && b.getStats().lost == 1, "link: a missing sequence number is counted");

	// A udp_handler.h program on the other end: three vectors only.
	const int raw = socket(AF_INET, SOCK_DGRAM, 0);
	struct sockaddr_in to;
	std::memset(&to, 0, sizeof(to));
	to.sin_family = AF_INET;
	to.sin_port = htons(port);  // b listens here
	inet_pton(AF_INET, "127.0.0.1", &to.sin_addr);
	double legacy[21];
	for (int i = 0; i < 21; ++i) legacy[i] = 100.0 + i;
	sendto(raw, legacy, sizeof(legacy), 0, reinterpret_cast<struct sockaddr*>(&to), sizeof(to));
	usleep(2000);
	check(b.poll(teleop_link::now()) && b.legacyPeer() && !b.latest().timed, "link: a legacy packet is accepted, untimed");
	check(b.latest().jp[0] == 100.0 && b.latest().jv[0] == 107.0 && b.latest().extTorque[6] == 120.0,
			"link: the legacy packet's vectors");
	// b now answers in the legacy format: a receives 168 bytes, as a legacy packet.
	b.send(jp, jv, ext, t0, 1, 5);
	usleep(2000);
	check(a.poll(teleop_link::now()) && a.legacyPeer() && a.latest().modeEpoch == 0, "link: answers a legacy peer in its format");
	close(raw);
}

void testModeAndReference() {
	std::atomic<int> mode(teleop_law::FULL);
	std::atomic<unsigned int> epoch(0), adoptions(0);
	teleop_law::adoptMode(teleop_law::PP, 0, &mode, &epoch, &adoptions);
	check(mode == teleop_law::FULL && adoptions == 0, "mode: nothing adopted before any change");
	teleop_law::adoptMode(teleop_law::NO_4C, 1, &mode, &epoch, &adoptions);
	check(mode == teleop_law::NO_4C && epoch == 1 && adoptions == 1, "mode: a newer change is adopted");
	teleop_law::adoptMode(teleop_law::FULL, 1, &mode, &epoch, &adoptions);
	check(mode == teleop_law::NO_4C && adoptions == 1, "mode: a tie keeps the more conservative mode");
	teleop_law::adoptMode(teleop_law::PP, 1, &mode, &epoch, &adoptions);
	check(mode == teleop_law::PP && adoptions == 2, "mode: a tie takes the other arm's more conservative mode");
	teleop_law::adoptMode(teleop_law::FULL, 0, &mode, &epoch, &adoptions);
	check(mode == teleop_law::PP, "mode: an older change is ignored");
	teleop_law::adoptMode(9, 50, &mode, &epoch, &adoptions);
	check(mode == teleop_law::PP && epoch == 1, "mode: a value that is no mode is ignored");

	vec7 jp, jv;
	jp.setConstant(1.0);
	jv.setConstant(2.0);
	const double nan = std::numeric_limits<double>::quiet_NaN();
	check(std::fabs(teleop_law::agedReference(jp, jv, 0.002)[0] - 1.004) < 1e-12, "aged reference: position + age * velocity");
	check(std::fabs(teleop_law::agedReference(jp, jv, 0.5)[3] - (1.0 + 2.0 * teleop_law::REFERENCE_AGE_LIMIT)) < 1e-12,
			"aged reference: the age is limited");
	check(teleop_law::agedReference(jp, jv, -0.01)[0] == 1.0 && teleop_law::agedReference(jp, jv, nan)[0] == 1.0,
			"aged reference: a negative or undefined age is not used");
}

void testRecorder() {
	const char* path = "/tmp/low_latency_tests.tlog";
	std::vector<std::string> columns;
	columns.push_back("count");
	columns.push_back("twice");
	columns.push_back("fixed");
	unsigned long pushed = 0, refused = 0;
	{
		TeleopRecorder recorder(path, columns, "a note\nwith a second line", 64);
		check(recorder.columns() == 3, "recorder: columns");
		// Faster than the writer drains a ring of 64: some rows are refused.
		for (int burst = 0; burst < 20; ++burst) {
			for (int i = 0; i < 200; ++i) {
				double* row = recorder.claim();
				if (row == NULL) {
					++refused;
					continue;
				}
				RecorderRow r(row, recorder.columns());
				r.put(static_cast<double>(pushed));
				r.put(2.0 * pushed);
				r.put(7.5);
				check(r.complete(), "recorder: the row is complete");
				recorder.commit();
				++pushed;
			}
			usleep(8000);
		}
		recorder.close();
		check(recorder.dropped() == refused && refused > 0, "recorder: refused rows are counted, never waited for");
		check(recorder.written() == pushed && !recorder.writeFailed(), "recorder: every accepted row is written");
	}
	std::ifstream in(path, std::ios::binary);
	std::string first, names, note;
	std::getline(in, first);
	std::getline(in, names);
	std::getline(in, note);
	check(first == "TELEOPLOG 1 3" && names == "count,twice,fixed" && note == "a note with a second line", "recorder: header");
	unsigned long rows = 0;
	bool ok = true;
	double row[3];
	while (in.read(reinterpret_cast<char*>(row), sizeof(row))) {
		ok = ok && row[0] == static_cast<double>(rows) && row[1] == 2.0 * rows && row[2] == 7.5;
		++rows;
	}
	check(ok && rows == pushed, "recorder: the file holds the accepted rows, in order");
	std::remove(path);
}

/* A joint of constant inertia in free motion: tau = M alpha + n. */
void testEstimator() {
	const double T = 0.002, hp = 25.0, lp = 50.0;
	const double M[4] = {2.0, 3.5, 0.6, 0.4}, n[4] = {1.0, -20.0, 4.0, -2.0};
	estimator::Acceleration accel;
	estimator::MatchedTorque matched;
	accel.configure(hp, lp, T);
	matched.configure(hp, lp, T);
	// libbarrett's own filters (math::FirstOrderFilter, backward Euler), from rest
	double hpY[4] = {0, 0, 0, 0}, hpX[4] = {0, 0, 0, 0}, lpY[4] = {0, 0, 0, 0};

	double v[4] = {0, 0, 0, 0};
	double worstMatched = 0.0, worstLegacy = 0.0, worstChain = 0.0, afterStep = 0.0;
	unsigned int seed = 12345;
	for (int k = 0; k < 6000; ++k) {
		double alpha[4], tau[4], inertia[4], ext[4], staticModel[4];
		for (int i = 0; i < 4; ++i) {
			seed = seed * 1103515245u + 12345u;
			const double noise = ((seed >> 8) & 0xFFFF) / 65535.0 - 0.5;
			// a hand-like motion at 0.5 to 8 Hz plus noise
			alpha[i] = 3.0 * std::sin(2 * M_PI * (0.5 + 2.5 * i) * k * T) + 20.0 * noise;
			v[i] += alpha[i] * T;
			const double external = (k >= 4000) ? 1.5 : 0.0;  // then something pushes
			tau[i] = M[i] * alpha[i] + n[i] + external;
			staticModel[i] = n[i];
		}
		const double* a = accel.step(v);
		for (int i = 0; i < 4; ++i) inertia[i] = M[i] * a[i];
		matched.step(tau, staticModel, inertia, ext);
		for (int i = 0; i < 4; ++i) {
			// the existing chain: hp1 = 25 s / (s + 25) on the velocity, then a 50 rad/s low-pass
			const double y = (hpY[i] + hp * (v[i] - hpX[i])) / (1.0 + hp * T);
			hpY[i] = y;
			hpX[i] = v[i];
			lpY[i] = (lpY[i] + lp * T * y) / (1.0 + lp * T);
			if (k > 1000) {  // once both have forgotten how they started
				worstChain = std::fmax(worstChain, std::fabs(lpY[i] - a[i]));
			}
			if (k > 1000 && k < 4000) {
				worstMatched = std::fmax(worstMatched, std::fabs(ext[i]));
				worstLegacy = std::fmax(worstLegacy, std::fabs(tau[i] - n[i] - inertia[i]));
			}
			if (k == 5999) {
				afterStep = std::fmax(afterStep, std::fabs(ext[i] - 1.5));
			}
		}
	}
	printf("  free motion, constant inertia: matched estimate at most %.2e Nm, legacy at most %.1f Nm\n", worstMatched,
			worstLegacy);
	check(worstChain < 1e-7, "estimator: its acceleration is the existing filter chain's");
	check(worstMatched < 1e-7, "estimator: matched cancels the arm's own inertial torque at every frequency");
	check(worstLegacy > 5.0, "estimator: legacy leaves it in the estimate (the test moves fast enough)");
	check(afterStep < 1e-6, "estimator: matched settles on a constant external torque");
}

void testRegressorModel() {
	estimator::RegressorModel model;
	const Eigen::Matrix<double, 30, 1> beta = initialize_follower_beta();
	model.setBeta(beta);
	double worst = 0.0, worstLinear = 0.0;
	unsigned int seed = 99;
	for (int k = 0; k < 200; ++k) {
		Eigen::Vector4d q, dq, ddq;
		for (int i = 0; i < 4; ++i) {
			seed = seed * 1103515245u + 12345u;
			q[i] = ((seed >> 8) & 0xFFFF) / 65535.0 * 4.0 - 2.0;
			seed = seed * 1103515245u + 12345u;
			dq[i] = ((seed >> 8) & 0xFFFF) / 65535.0 * 3.0 - 1.5;
			seed = seed * 1103515245u + 12345u;
			ddq[i] = ((seed >> 8) & 0xFFFF) / 65535.0 * 10.0 - 5.0;
		}
		// as FollowerDynamics computed it
		const Eigen::Vector4d quarter = 0.25 * ddq;
		const Eigen::Matrix<double, 4, 30> W = calculate_W_eigen(q, dq, quarter);
		const Eigen::Vector4d old = W * beta;
		double inertia[4], scaled[4];
		for (int i = 0; i < 4; ++i) scaled[i] = quarter[i];
		model.update(q.data(), dq.data());
		model.inertia(q.data(), dq.data(), scaled, inertia);
		for (int i = 0; i < 4; ++i) {
			worst = std::fmax(worst, std::fabs(model.staticTorque[i] + inertia[i] - old[i]));
		}
		// linear in the acceleration: M(q) (2 a) = 2 M(q) a
		double twice[4], inertia2[4];
		for (int i = 0; i < 4; ++i) twice[i] = 2.0 * scaled[i];
		model.inertia(q.data(), dq.data(), twice, inertia2);
		for (int i = 0; i < 4; ++i) {
			worstLinear = std::fmax(worstLinear, std::fabs(inertia2[i] - 2.0 * inertia[i]));
		}
	}
	check(worst < 1e-9, "regressor model: static + inertia is FollowerDynamics' torque");
	check(worstLinear < 1e-9, "regressor model: the inertial torque is linear in the acceleration");
}

#ifdef __linux__
void testRealtimeThread() {
	rt_thread::Report before, after;
	std::thread t([&]() {
		rt_thread::configureThisThread(0, -1, &before);   // only looks
		rt_thread::configureThisThread(50, 0, &after);    // SCHED_FIFO 50 on CPU 0
	});
	t.join();
	check(before.done && before.policyAfter == before.policyBefore, "rt thread: priority 0 leaves the thread alone");
	if (after.schedError != 0) {
		printf("  (no real-time permission here, error %d: run in the wam_ros container to check the rest)\n",
				after.schedError);
	} else {
		check(after.policyBefore == SCHED_OTHER && after.policyAfter == SCHED_FIFO && after.priorityAfter == 50,
				"rt thread: an ordinary thread becomes SCHED_FIFO 50");
	}
	check(after.affinityError == 0 && after.cpuNow == 0, "rt thread: pinned to the CPU asked for");
	rt_thread::print("  test", after);
}
#endif

}  // namespace

/* A made-up gripper on a made-up sensor: turned axes, reversed sign, offsets. */
struct FakeSensor {
	ft_core::Tool tool;
	double biasF[3], biasN[3];
	FakeSensor() {
		tool.mass = 1.23;
		tool.com[0] = 0.004;
		tool.com[1] = -0.006;
		tool.com[2] = 0.071;
		tool.sign = -1.0;
		// about z by 110 degrees, then about y by 5
		const double cz = std::cos(110.0 * M_PI / 180.0), sz = std::sin(110.0 * M_PI / 180.0);
		const double cy = std::cos(5.0 * M_PI / 180.0), sy = std::sin(5.0 * M_PI / 180.0);
		const double r[9] = {cz * cy, -sz, cz * sy, sz * cy, cz, sz * sy, -sy, 0.0, cy};
		for (int i = 0; i < 9; ++i) tool.rot[i] = r[i];
		tool.origin[2] = 0.012;
		const double bf[3] = {3.1, -2.2, 7.7}, bn[3] = {0.11, -0.07, 0.03};
		for (int i = 0; i < 3; ++i) {
			biasF[i] = bf[i];
			biasN[i] = bn[i];
		}
	}
	/* The reading with the tool turned as worldFromTool and the surroundings
	 * applying (envF, envN), in the tool frame about its origin. */
	void reading(const double* worldFromTool, const double* envF, const double* envN, double* rawF, double* rawN) const {
		double g[3], f[3], n[3], shift[3], sf[3], sn[3];
		ft_core::gravityInTool(worldFromTool, g);
		tool.weight(g, f, n);
		ft_core::cross(tool.origin, envF, shift);
		for (int i = 0; i < 3; ++i) {
			f[i] += envF[i];
			n[i] += envN[i] - shift[i];  // about the sensor's origin
		}
		ft_core::rotateBack(tool.rot, f, sf);
		ft_core::rotateBack(tool.rot, n, sn);
		for (int i = 0; i < 3; ++i) {
			rawF[i] = tool.sign * sf[i] + biasF[i];
			rawN[i] = tool.sign * sn[i] + biasN[i];
		}
	}
};

/* A rotation from three angles (row-major), different for every k. */
void someRotation(int k, double* r) {
	const double a = 0.7 * k + 0.3, b = 1.3 * k - 0.4, c = 0.45 * k + 1.1;
	const double ca = std::cos(a), sa = std::sin(a), cb = std::cos(b), sb = std::sin(b), cc = std::cos(c), sc = std::sin(c);
	const double m[9] = {ca * cb, ca * sb * sc - sa * cc, ca * sb * cc + sa * sc,
	                     sa * cb, sa * sb * sc + ca * cc, sa * sb * cc - ca * sc,
	                     -sb, cb * sc, cb * cc};
	for (int i = 0; i < 9; ++i) r[i] = m[i];
}

bool nearTo(double a, double b, double tolerance) { return std::fabs(a - b) <= tolerance; }

void testFtCore() {
	printf("force/torque sensor arithmetic\n");
	const FakeSensor sensor;
	const double none[3] = {0.0, 0.0, 0.0};

	// Gravity in the tool frame: the tool upright sees it along -z, upside down along +z.
	double g[3];
	const double upright[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}, flipped[9] = {1, 0, 0, 0, -1, 0, 0, 0, -1};
	ft_core::gravityInTool(upright, g);
	check(g[0] == 0.0 && g[1] == 0.0 && g[2] == -1.0, "ft: gravity along -z of an upright tool");
	ft_core::gravityInTool(flipped, g);
	check(g[2] == 1.0, "ft: gravity along +z of a tool pointing down");

	// Free space: whatever the orientation, the result is zero once zeroed at any other.
	double zeroR[9], zeroF[3], zeroN[3], zeroG[3];
	someRotation(0, zeroR);
	sensor.reading(zeroR, none, none, zeroF, zeroN);
	ft_core::gravityInTool(zeroR, zeroG);
	double worst = 0.0, worstUncalibrated = 0.0;
	for (int k = 1; k < 40; ++k) {
		double r[9], rawF[3], rawN[3], f[3], n[3];
		someRotation(k, r);
		sensor.reading(r, none, none, rawF, rawN);
		ft_core::gravityInTool(r, g);
		ft_core::environmentWrench(sensor.tool, rawF, rawN, zeroF, zeroN, g, zeroG, f, n);
		for (int i = 0; i < 3; ++i) worst = std::max(worst, std::max(std::fabs(f[i]), std::fabs(n[i])));
		ft_core::environmentWrench(ft_core::Tool(), rawF, rawN, zeroF, zeroN, g, zeroG, f, n);
		worstUncalibrated = std::max(worstUncalibrated, ft_core::norm(f));
	}
	check(worst < 1e-9, "ft: free space reads zero at every orientation with the gripper's calibration");
	check(worstUncalibrated > 15.0, "ft: without it the gripper's weight shows (why wrist/arm need --ft-tool)");

	// A push and a twist come back as applied, about the tool frame's origin.
	const double pushF[3] = {4.0, -7.5, 11.0}, pushN[3] = {0.3, 0.45, -0.2};
	for (int k = 1; k < 10; ++k) {
		double r[9], rawF[3], rawN[3], f[3], n[3];
		someRotation(k, r);
		sensor.reading(r, pushF, pushN, rawF, rawN);
		ft_core::gravityInTool(r, g);
		ft_core::environmentWrench(sensor.tool, rawF, rawN, zeroF, zeroN, g, zeroG, f, n);
		for (int i = 0; i < 3; ++i) {
			check(nearTo(f[i], pushF[i], 1e-9) && nearTo(n[i], pushN[i], 1e-9), "ft: an applied wrench is recovered");
		}
	}

	// Reading by reading: nothing until zeroed, then in gradually; held when the sensor stops.
	ft_core::Processor p;
	p.tool = sensor.tool;
	const double T = 0.002;
	double r1[9], rawF[3], rawN[3];
	someRotation(3, r1);
	sensor.reading(zeroR, none, none, rawF, rawN);
	for (int k = 0; k < 500; ++k) p.step(true, rawF, rawN, zeroR, T);
	check(p.use() == 0.0 && ft_core::norm(p.force) == 0.0, "ft: no use and no wrench before a zero");
	p.startZero(250);
	for (int k = 0; k < 249; ++k) p.step(true, rawF, rawN, zeroR, T);
	check(!p.zero.valid && p.zero.running(), "ft: the zero takes its 250 readings");
	p.step(true, rawF, rawN, zeroR, T);
	check(p.zero.valid && !p.zero.running() && p.zero.count == 1, "ft: zero taken");
	check(nearTo(p.zero.force[0], rawF[0], 1e-9) && nearTo(p.zero.gravity[2], zeroG[2], 1e-12), "ft: the zero is the average");
	for (int k = 0; k < 125; ++k) p.step(true, rawF, rawN, zeroR, T);
	check(nearTo(p.use(), 0.5, 0.01), "ft: half in use 0.25 s after the zero");
	for (int k = 0; k < 130; ++k) p.step(true, rawF, rawN, zeroR, T);
	check(p.use() == 1.0, "ft: in full use 0.5 s after the zero");
	sensor.reading(r1, pushF, pushN, rawF, rawN);
	for (int k = 0; k < 500; ++k) p.step(true, rawF, rawN, r1, T);
	for (int i = 0; i < 3; ++i) {
		check(nearTo(p.force[i], pushF[i], 1e-6) && nearTo(p.torque[i], pushN[i], 1e-6), "ft: the filtered wrench settles on the applied one");
	}
	// The sensor answers only every other cycle: still in use.
	for (int k = 0; k <= 500; ++k) p.step(k % 2 == 0, rawF, rawN, r1, T);  // the last one a reading
	check(p.use() == 1.0 && !p.health.stale(), "ft: a reading every other cycle is enough");
	// It stops: stale after 20 ms, out of use 0.1 s later; and back in gradually.
	for (int k = 0; k < 10; ++k) p.step(false, rawF, rawN, r1, T);
	check(p.use() == 1.0, "ft: 20 ms without a reading is tolerated");
	for (int k = 0; k < 26; ++k) p.step(false, rawF, rawN, r1, T);
	check(nearTo(p.use(), 0.5, 0.03), "ft: fading out once stale");
	for (int k = 0; k < 30; ++k) p.step(false, rawF, rawN, r1, T);
	check(p.use() == 0.0 && p.health.stale(), "ft: out of use 0.1 s after going stale");
	p.step(true, rawF, rawN, r1, T);
	check(p.use() > 0.0 && p.use() < 0.01, "ft: back in gradually when it answers again");
	for (int k = 0; k < 260; ++k) p.step(true, rawF, rawN, r1, T);

	// A wild reading (a corrupted frame) moves the result by the rate limit only; a
	// force beyond the limit is cut to it.
	double wildF[3] = {rawF[0] + 120.0, rawF[1], rawF[2]};
	const double before = p.force[0];
	p.step(true, wildF, rawN, r1, T);
	check(std::fabs(p.force[0] - before) < 2.0 && std::fabs(p.force[1] - pushF[1]) < 2.0, "ft: one wild reading moves the force by under 2 N");
	for (int k = 0; k < 2000; ++k) p.step(true, wildF, rawN, r1, T);
	check(nearTo(ft_core::norm(p.force), p.limits.force, 1e-6), "ft: a force beyond the limit is cut to it");

	// A new zero mid-run: out of use, then in again from the new zero.
	for (int k = 0; k < 2000; ++k) p.step(true, rawF, rawN, r1, T);
	p.startZero(100);
	for (int k = 0; k < 100; ++k) p.step(true, rawF, rawN, r1, T);
	check(p.zero.count == 2 && p.use() < 0.01, "ft: a new zero starts from no use");
	for (int k = 0; k < 300; ++k) p.step(true, rawF, rawN, r1, T);
	check(p.use() == 1.0 && ft_core::norm(p.force) < 1e-9 && ft_core::norm(p.torque) < 1e-9,
	      "ft: after it, what was there at the zero reads as nothing");

	// The tether and the small-torque cut.
	check(ft_core::softDeadband(0.02, 0.05) == 0.0 && ft_core::softDeadband(-0.05, 0.05) == 0.0, "ft: no pull within the deadband");
	check(nearTo(ft_core::softDeadband(0.08, 0.05), 0.03, 1e-12) && nearTo(ft_core::softDeadband(-0.08, 0.05), -0.03, 1e-12),
	      "ft: beyond it, what is beyond");
	check(ft_core::softDeadband(0.3, 0.0) == 0.3, "ft: no deadband at width 0");
	check(ft_core::toward(0.2, 1.0, 0.5) == 0.7 && ft_core::toward(0.8, 1.0, 0.5) == 1.0 && ft_core::toward(0.3, 0.0, 0.5) == 0.0,
	      "ft: toward() stops at its target");

	// The gripper's file.
	const std::string path = "/tmp/ft_tool_test.conf";
	{
		std::ofstream out(path.c_str());
		out << "# a comment\nmass = 1.23\ncom_x = 0.004\ncom_y=-0.006\ncom_z = 0.071  # m\nsign = -1\n"
		    << "rot_xx = 0\nrot_xy = -1\nrot_yx = 1\nrot_yy = 0\nrot_zz = 1\norigin_z = 0.012\n";
	}
	ft_core::Tool loaded;
	std::string error;
	check(loaded.load(path, &error) && loaded.calibrated, "ft: the gripper's file loads");
	check(loaded.mass == 1.23 && loaded.com[1] == -0.006 && loaded.sign == -1.0 && loaded.rot[1] == -1.0 && loaded.rot[3] == 1.0 &&
	      loaded.origin[2] == 0.012, "ft: ... with its values");
	const char* bad[] = {"com_x = 0.01\n", "mass = 1\nsign = 2\n", "mass = 1\nrot_xx = 0\n", "mass = 1\nrot_xx = -1\n",
	                     "mass = 1\nmas = 2\n", "mass = 1\ncom_z = 0.9\n", "mass = 1 kg\n", "mass = nan\n", "mass 1\n"};
	for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); ++i) {
		{
			std::ofstream out(path.c_str());
			out << bad[i];
		}
		ft_core::Tool t;
		check(!t.load(path, &error) && !t.calibrated && !error.empty(), "ft: a wrong gripper file is refused");
	}
	std::remove(path.c_str());
	check(!ft_core::Tool().load("/nonexistent/tool.conf", &error), "ft: a missing gripper file is refused");
}

/* --ft-samples: the made-up gripper's readings in free space at many
 * orientations, as a follower recording (the columns ft_tool_cal.py reads),
 * with noise and a stretch of motion it must leave out. */
int writeFtSamples(const char* path) {
	const FakeSensor sensor;
	const double none[3] = {0.0, 0.0, 0.0};
	std::vector<std::string> columns;
	columns.push_back("t");
	const char* groups[] = {"dq", "ft_raw_f", "ft_raw_n", "ft_g"};
	const int sizes[] = {7, 3, 3, 3};
	for (int gi = 0; gi < 4; ++gi) {
		for (int i = 1; i <= sizes[gi]; ++i) columns.push_back(std::string(groups[gi]) + "_" + std::to_string(i));
	}
	columns.push_back("ft_new");
	TeleopRecorder recorder(path, columns, "low_latency_tests --ft-samples", 1 << 16);
	unsigned int seed = 12345;
	auto noise = [&seed](double scale) {
		seed = seed * 1664525u + 1013904223u;
		return scale * (static_cast<double>(seed >> 8) / 8388608.0 - 1.0);
	};
	int row = 0;
	for (int k = 0; k < 30; ++k) {
		double r[9], rawF[3], rawN[3], g[3];
		someRotation(k, r);
		sensor.reading(r, none, none, rawF, rawN);
		ft_core::gravityInTool(r, g);
		for (int n = 0; n < 500; ++n, ++row) {  // 1 s per pose, the first 0.3 s moving
			const bool moving = n < 150;
			double* out = NULL;
			while ((out = recorder.claim()) == NULL) usleep(1000);
			RecorderRow rr(out, recorder.columns());
			rr.put(0.002 * row);
			for (int i = 0; i < 7; ++i) rr.put(moving ? 0.4 : noise(0.002));
			for (int i = 0; i < 3; ++i) rr.put(rawF[i] + noise(0.15) + (moving ? 6.0 : 0.0));
			for (int i = 0; i < 3; ++i) rr.put(rawN[i] + noise(0.004) + (moving ? 0.5 : 0.0));
			for (int i = 0; i < 3; ++i) rr.put(g[i]);
			rr.put(1.0);
			recorder.commit();
		}
	}
	recorder.close();
	printf("wrote %lu rows to %s\n", recorder.written(), path);
	return recorder.written() == 15000 ? 0 : 1;
}

/* --ft-check: the file ft_tool_cal.py fitted from those samples describes the
 * made-up gripper, as this program's arithmetic uses it. */
int checkFtTool(const char* path) {
	const FakeSensor sensor;
	ft_core::Tool fitted;
	std::string error;
	if (!fitted.load(path, &error)) {
		printf("FAIL: %s\n", error.c_str());
		return 1;
	}
	check(nearTo(fitted.mass, sensor.tool.mass, 0.01), "ft-check: mass within 10 g");
	check(fitted.sign == sensor.tool.sign, "ft-check: sign");
	for (int i = 0; i < 3; ++i) check(nearTo(fitted.com[i], sensor.tool.com[i], 0.001), "ft-check: centre of mass within 1 mm");
	for (int i = 0; i < 9; ++i) check(nearTo(fitted.rot[i], sensor.tool.rot[i], 0.01), "ft-check: sensor axes");
	// With the fitted file, free space reads as nothing and a push as itself.
	fitted.origin[2] = sensor.tool.origin[2];  // not observable from free space; from the sensor's drawing
	const double none[3] = {0.0, 0.0, 0.0}, pushF[3] = {4.0, -7.5, 11.0}, pushN[3] = {0.3, 0.45, -0.2};
	double zeroR[9], zeroF[3], zeroN[3], zeroG[3], worstFree = 0.0, worstPushF = 0.0, worstPushN = 0.0;
	someRotation(100, zeroR);
	sensor.reading(zeroR, none, none, zeroF, zeroN);
	ft_core::gravityInTool(zeroR, zeroG);
	for (int k = 101; k < 140; ++k) {
		double r[9], rawF[3], rawN[3], g[3], f[3], n[3];
		someRotation(k, r);
		ft_core::gravityInTool(r, g);
		sensor.reading(r, none, none, rawF, rawN);
		ft_core::environmentWrench(fitted, rawF, rawN, zeroF, zeroN, g, zeroG, f, n);
		worstFree = std::max(worstFree, std::max(ft_core::norm(f), 10.0 * ft_core::norm(n)));
		sensor.reading(r, pushF, pushN, rawF, rawN);
		ft_core::environmentWrench(fitted, rawF, rawN, zeroF, zeroN, g, zeroG, f, n);
		for (int i = 0; i < 3; ++i) {
			worstPushF = std::max(worstPushF, std::fabs(f[i] - pushF[i]));
			worstPushN = std::max(worstPushN, std::fabs(n[i] - pushN[i]));
		}
	}
	printf("with the fitted file: free space reads at most %.3f N (or a tenth of that in Nm); a push is off by at most %.3f N, %.4f Nm\n",
	       worstFree, worstPushF, worstPushN);
	check(worstFree < 0.3, "ft-check: free space within 0.3 N / 0.03 Nm at every orientation");
	check(worstPushF < 0.3 && worstPushN < 0.03, "ft-check: a push comes back within 0.3 N, 0.03 Nm");
	printf("%d checks, %d failed\n", checks, failures);
	return failures == 0 ? 0 : 1;
}

int main(int argc, char** argv) {
	if (argc == 3 && std::string(argv[1]) == "--ft-samples") {
		return writeFtSamples(argv[2]);
	}
	if (argc == 3 && std::string(argv[1]) == "--ft-check") {
		return checkFtTool(argv[2]);
	}
	testFtCore();
	testLink(26554);
	testModeAndReference();
	testRecorder();
	testEstimator();
	testRegressorModel();
#ifdef __linux__
	testRealtimeThread();
#endif
	printf("%d checks, %d failed\n", checks, failures);
	return failures == 0 ? 0 : 1;
}
