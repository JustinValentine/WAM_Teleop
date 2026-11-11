#pragma once

/* Leader side of the teleop for the eight-joint wam8g, paired with a
 * seven-joint P2 follower (follower --sync wam8g). Same structure as
 * leader.h, except:
 *  - there is no separate wrist device: the Alberta wrist is J6-J8 of the WAM;
 *  - it works on eight joints but keeps the follower's seven-value message,
 *    [J1-J4, follower J5-J7], mapping the wrist through WristMap;
 *  - the law (teleop_law.h) acts on J1-J4 only, so the wrist keeps its
 *    gravity compensation;
 *  - with sendExtTorque false (no wam8g fit yet) it sends the follower zero
 *    external torque: without a model the estimate is the arm's gravity
 *    torque, which the follower's u4 would apply;
 *  - the wrist can take the follower's force/torque sensor (lib/ft_core.h):
 *    setWristForce() applies the wrist torques the follower measured, and
 *    setWristDeadband() makes the wrist's position coupling a tether that
 *    pulls only beyond that many radians. Both follow the 4C mode ('c' turns
 *    them off with the force channels) and fade in with it.
 *
 * The control loop itself talks to the follower (teleop_link.h: no helper
 * thread, no lock), sets up its own thread (rt_thread.h), and can write one
 * row per cycle to a recorder (teleop_recorder.h). Nothing in operate() or
 * receive() prints, allocates or waits. */

#include <atomic>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/helpers.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/units.h>

#include "ft_core.h"
#include "rt_thread.h"
#include "teleop_law.h"
#include "teleop_link.h"
#include "teleop_recorder.h"
#include "teleop_reference.h"
#include "wam8g_wrist_map.h"

class LeaderWam8g : public barrett::systems::System {
	BARRETT_UNITS_TYPEDEFS(8);

public:
	Input<jp_type> wamJPIn;
	Input<jv_type> wamJVIn;
	Input<jt_type> extTorqueIn;
	Input<jt_type> wamGravIn;
	Input<jt_type> wamDynIn;
	Input<jt_type> pidIn;  // the position controller's torque, for the recording only (may be unconnected)

	Output<jt_type> wamJPOutput;    // feedforward torque (J1-J4)

private:
	// Before theirJPOutput, its output.
	TeleopReference<LeaderWam8g, jp_type> referenceSource;
	Input<jp_type> referenceIn;

public:
	// Position reference: the follower, mapped to J1-J8 (teleop_reference.h).
	Output<jp_type>& theirJPOutput;

	enum class State { INIT, LINKED, UNLINKED };

	LeaderWam8g(barrett::systems::ExecutionManager* em, const std::string& remoteHost,
			int recPort, int sendPort, teleop_law::Law law,
			const wam8g_teleop::WristMap& wrist, bool sendExtTorque,
			const std::string& sysName = "LeaderWam8g")
		: System(sysName), wamJPIn(this), wamJVIn(this), extTorqueIn(this),
		  wamGravIn(this), wamDynIn(this), pidIn(this), wamJPOutput(this, &jtOutputValue),
		  referenceSource(this, sysName + "::Reference"), referenceIn(this),
		  theirJPOutput(referenceSource.output), theirJp(0.0), theirExtTorque(0.0),
		  control(0.0), link(remoteHost, sendPort, recPort), law(law),
		  wrist(wrist), sendExtTorque(sendExtTorque), state(State::INIT) {
		period = (em != NULL) ? em->getPeriod() : 0.002;
		fadeTime = 0.0;
		for (int i = 0; i < 4; ++i) {
			forceScale[i] = 1.0;
		}
		forceCutoff = 0.0;
		forceFiltered.setZero();
		notchHz = 0.0;
		j6Friction = 0.0;
		mode = teleop_law::FULL;
		modeEpoch = 0;
		modeAdoptions = 0;
		linkLostEvents = 0;
		linkAlive = false;
		halted = false;
		haveOwnJp = false;
		ageReference = false;
		rtPriority = 0;
		rtCpu = -1;
		rtConfigured = false;
		recorder = NULL;
		cycleStart = 0.0;
		cycleDt = 0.0;
		cycleCount = 0;
		rxNew = false;
		rxAge = std::numeric_limits<double>::quiet_NaN();
		sendOk = true;
		for (int k = 0; k < 3; ++k) {
			wristForceGain[k] = 0.0;
			theirWristExt[k] = wristFiltered[k] = wristFf[k] = wristRef[k] = 0.0;
		}
		wristForceCutoff = wristForceFilterDefault();
		wristDeadzone = wristForceDeadzoneDefault();
		wristDeadband = 0.0;
		deadbandNow = 0.0;
		wristUse = 0.0;
		theirFlags = 0;
		sendFlags = 0;
		barrett::systems::connect(referenceSource.output, referenceIn);
		if (em != NULL) {
			em->startManaging(*this);
		}
	}

	virtual ~LeaderWam8g() {
		referenceSource.detach();
		this->mandatoryCleanUp();
	}

	virtual bool inputsValid() { return true; }

	bool isLinked() const { return state == State::LINKED; }
	void tryLink() {
		BARRETT_SCOPED_LOCK(this->getEmMutex());
		if (!halted) {
			state = State::LINKED;
		}
	}
	// Stops the teleop for good (SafetyWatch, when this arm leaves ACTIVE; or
	// exiting): unlinked, and nothing more is sent, so the follower's link
	// times out and it holds where it is. This arm's reference becomes its own
	// position, so it does not jump when it is active again.
	void halt() { halted = true; }
	bool isHalted() const { return halted; }
	// The force channel's gain in u4 and u6 (teleop_law::parseForceGain).
	void setForceGain(const teleop_law::ForceGains& gains) {
		BARRETT_SCOPED_LOCK(this->getEmMutex());
		for (int i = 0; i < 4; ++i) {
			forceScale[i] = gains.g[i] / teleop_law::FORCE_GAIN;
		}
	}
	// The force channel's low-pass in Hz, 0: off (teleop_law::parseForceFilter).
	void setForceFilter(double hz) {
		BARRETT_SCOPED_LOCK(this->getEmMutex());
		forceCutoff = hz;
	}
	// The force channel's notch in Hz, 0: off (teleop_law::Notch).
	// J6's friction feedforward in Nm, 0: off (teleop_law::frictionFeedforward).
	void setJ6Friction(double nm) {
		BARRETT_SCOPED_LOCK(this->getEmMutex());
		j6Friction = nm;
	}
	double getJ6Friction() const { return j6Friction; }
	void setForceNotch(double hz) {
		BARRETT_SCOPED_LOCK(this->getEmMutex());
		notchHz = hz;
	}
	// The wrist's force channel: the gain on the wrist torques the follower's
	// force/torque sensor measured (follower --ft wrist), for J6, J7, J8;
	// 0: off. Like --force-gain on J1-J4, but on a measurement: 1 is the
	// follower's torque in full.
	void setWristForce(const double* gains) {
		BARRETT_SCOPED_LOCK(this->getEmMutex());
		for (int k = 0; k < 3; ++k) {
			wristForceGain[k] = gains[k];
		}
	}
	void getWristForce(double* gains) const {
		for (int k = 0; k < 3; ++k) {
			gains[k] = wristForceGain[k];
		}
	}
	// Its low-pass in Hz (0: off) and the torque below which it stays out, Nm.
	void setWristForceFilter(double hz, double deadzoneNm) {
		BARRETT_SCOPED_LOCK(this->getEmMutex());
		wristForceCutoff = hz;
		wristDeadzone = deadzoneNm;
	}
	// The wrist's position coupling as a tether: no pull on J6-J8 while the
	// follower's wrist is within this many radians (0: off, the plain
	// position coupling). With a wrist force gain set, the tether is only
	// slack while the follower's measured torques arrive: without them a
	// contact is felt through the position error, as before.
	void setWristDeadband(double rad) {
		BARRETT_SCOPED_LOCK(this->getEmMutex());
		wristDeadband = rad;
	}
	double getWristDeadband() const { return wristDeadband; }
	// 0..1: how far the follower's measured wrist torques are in use.
	double getWristUse() const { return wristUse; }
	static double wristForceLimit() { const double v = WRIST_FORCE_LIMIT; return v; }
	static double wristForceFilterDefault() { const double v = WRIST_FORCE_FILTER; return v; }
	static double wristForceDeadzoneDefault() { const double v = WRIST_FORCE_DEADZONE; return v; }
	// The mode (teleop_law::Mode): the law's terms fade to it. The change
	// also travels to the follower in this arm's packets (teleop_link.h).
	void setMode(teleop_law::Mode m) {
		mode = m;
		modeEpoch = modeEpoch.load() + 1;
	}
	teleop_law::Mode getMode() const { return static_cast<teleop_law::Mode>(mode.load()); }
	// Mode changes taken from the follower's packets so far.
	unsigned int getModeAdoptions() const { return modeAdoptions; }
	void unlink() {
		BARRETT_SCOPED_LOCK(this->getEmMutex());
		state = State::UNLINKED;
	}

	// Times the link dropped while linked (the loop no longer prints it).
	unsigned int getLinkLostEvents() const { return linkLostEvents; }
	// The follower's position extrapolated to this cycle with its sent
	// velocity and the packet's age (teleop_law::REFERENCE_AGE_LIMIT). Needs
	// both programs on one computer; otherwise, and when off, the reference
	// advances only in cycles without a new packet, as before.
	void setAgeReference(bool on) {
		BARRETT_SCOPED_LOCK(this->getEmMutex());
		ageReference = on;
	}
	// The control thread's setup, done in its next cycle (rt_thread.h).
	void setRealtime(int fifoPriority, int cpu) {
		BARRETT_SCOPED_LOCK(this->getEmMutex());
		rtPriority = fifoPriority;
		rtCpu = cpu;
		rtConfigured = false;
	}
	const rt_thread::Report& getRealtimeReport() const { return rtReport; }
	// One row per control cycle (recordColumns()); NULL: none.
	void setRecorder(TeleopRecorder* r) {
		BARRETT_SCOPED_LOCK(this->getEmMutex());
		recorder = (r != NULL && r->columns() == recordColumns().size()) ? r : NULL;
	}
	teleop_link::Link<7>::Stats getLinkStats() const {
		BARRETT_SCOPED_LOCK(this->getEmMutex());
		return link.getStats();
	}

	static std::vector<std::string> recordColumns() {
		std::vector<std::string> c;
		c.push_back("t");
		c.push_back("cycle");
		c.push_back("dt");
		addColumns(&c, "q", 8);
		addColumns(&c, "dq", 8);
		addColumns(&c, "tau_grav", 8);
		addColumns(&c, "tau_pid", 8);
		addColumns(&c, "tau_ff", 8);
		addColumns(&c, "ref", 8);
		addColumns(&c, "ext", 4);
		addColumns(&c, "ext_sent", 4);
		addColumns(&c, "ext_rx", 4);
		addColumns(&c, "ext_used", 4);
		addColumns(&c, "dyn", 4);
		addColumns(&c, "wrist_ext_rx", 3);  // the follower's wrist slots (its J5-J7), as received
		addColumns(&c, "wrist_ff", 3);      // the wrist force channel's torque on J6-J8 (part of tau_ff)
		addColumns(&c, "wrist_ref", 3);     // the follower's wrist as J6-J8, before the deadband (ref: after)
		const char* rest[] = {"wrist_use", "wrist_deadband", "tx_flags", "rx_flags",
				"rx_new", "rx_age", "rx_seq", "rx_lost", "link_alive", "state", "mode",
				"mix_force", "mix_own", "mix_dyn", "halted", "send_ok", "rec_dropped"};
		for (size_t i = 0; i < sizeof(rest) / sizeof(rest[0]); ++i) {
			c.push_back(rest[i]);
		}
		return c;
	}

protected:
	typedef Eigen::Matrix<double, 7, 1> Message;
	typedef teleop_link::Link<7> Link;

	Output<jt_type>::Value* jtOutputValue;

	virtual void operate() {
		// Receive first (referenceSource): the external torque estimate read
		// below runs the position controller, which must track this packet.
		referenceIn.valueDefined();

		// The inputs are connected after the constructor starts this system
		// being managed; until they are, output nothing (reading an
		// unconnected input dereferences a null output: a startup crash).
		if (!wamJPIn.valueDefined() || !wamJVIn.valueDefined() || !wamGravIn.valueDefined() ||
				!wamDynIn.valueDefined()) {
			control.setZero();
			jtOutputValue->setData(&control);
			return;
		}
		const jp_type& jp = wamJPIn.getValue();
		const jv_type& jv = wamJVIn.getValue();
		const jt_type& grav = wamGravIn.getValue();
		const jt_type& dyn = wamDynIn.getValue();
		theirJp[4] = jp[4];  // passive J5: never commanded
		ownJp = jp;  // for receive() next cycle
		haveOwnJp = true;
		jt_type ext(0.0);
		if (extTorqueIn.valueDefined()) {
			ext = extTorqueIn.getValue();
		}

		// Send [J1-J4, follower J5-J7]; the wrist's external torque is zero.
		Message sendJp, sendJv, sendExt;
		Eigen::Matrix<double, 3, 1> wristJp, wristJv;
		wrist.toFollower(jp, &wristJp);
		wrist.velocityToFollower(jv, &wristJv);
		for (int i = 0; i < 4; ++i) {
			sendJp[i] = jp[i];
			sendJv[i] = jv[i];
			// The external torque only while linked (teleop_law::forceChannel).
			sendExt[i] = (sendExtTorque && state == State::LINKED) ? ext[i] : 0.0;
		}
		for (int k = 0; k < 3; ++k) {
			sendJp[4 + k] = wristJp[k];
			sendJv[4 + k] = wristJv[k];
			sendExt[4 + k] = 0.0;
		}
		sendFlags = state == State::LINKED ? teleop_link::FLAG_LINKED : 0u;
		if (halted) {
			state = State::UNLINKED;
		} else {
			// Stamped with the start of this cycle, when the joints were sampled.
			sendOk = link.send(sendJp, sendJv, sendExt, cycleStart, static_cast<uint32_t>(mode.load()),
					modeEpoch.load(), sendFlags);
			if (!linkAlive && state == State::LINKED) {
				++linkLostEvents;  // TeleopWatch prints it
				state = State::UNLINKED;
			}
		}

		const bool otherLinked = link.legacyPeer() ? teleop_law::sendsExternalTorque(theirExtTorque)
				: (theirFlags & teleop_link::FLAG_LINKED) != 0;
		jt_type theirExtUsed = forceNotch(
				teleop_law::lowPass(teleop_law::forceChannel(theirExtTorque, state == State::LINKED, otherLinked, period,
						&fadeTime),
						forceCutoff, period, &forceFiltered),
				notchHz, period);
		mix.fade(getMode(), period);
		jt_type extUsed(ext), dynUsed(dyn);
		for (int i = 0; i < 4; ++i) {
			theirExtUsed[i] *= forceScale[i] * mix.force;
			extUsed[i] *= mix.own;
			dynUsed[i] = grav[i] + mix.dyn * (dyn[i] - grav[i]);
		}
		if (state == State::LINKED) {
			control = teleop_law::compute(law, theirExtUsed, extUsed, grav, dynUsed);
		} else {
			control.setZero();
		}
		if (!halted) {  // linked or not: J6's own bearing drag
			control[5] += teleop_law::frictionFeedforward(j6Friction, jv[5]);
		}

		// The wrist's force channel and tether (lib/ft_core.h). The follower's
		// wrist slots hold minus the torque from outside on its J5-J7, as its
		// J1-J4 slots do; they map to J6-J8 with the position map's signs
		// (equal work on both wrists). channel: both arms linked (the same
		// fade as J1-J4's force channel) and the mode at full.
		const bool measured = linkAlive && state == State::LINKED &&
				(theirFlags & teleop_link::FLAG_MEASURED_WRIST) != 0;
		const double rise = WRIST_RISE_TIME, fall = WRIST_FALL_TIME, limit = WRIST_FORCE_LIMIT;
		wristUse = ft_core::toward(wristUse, measured ? 1.0 : 0.0, period / (measured ? rise : fall));
		const double channel = (fadeTime / teleop_law::FORCE_FADE_TIME) * mix.force;
		const double kFilter = wristForceCutoff > 0.0 ? 1.0 - std::exp(-2.0 * M_PI * wristForceCutoff * period) : 1.0;
		bool anyGain = false;
		for (int k = 0; k < 3; ++k) {
			wristFf[k] = 0.0;
			const int s = wrist.source[k];
			if (s < 0) {
				continue;
			}
			const double raw = measured ? wrist.sign[k] * theirWristExt[k] : 0.0;
			wristFiltered[k] += kFilter * (raw - wristFiltered[k]);
			anyGain = anyGain || wristForceGain[k] > 0.0;
			if (state == State::LINKED && !halted) {
				wristFf[k] = channel * wristUse *
						ft_core::clamp(-wristForceGain[k] * ft_core::softDeadband(wristFiltered[k], wristDeadzone), limit);
				control[s] += wristFf[k];
			}
		}
		// The tether's slack next cycle (receive()).
		deadbandNow = wristDeadband * channel * (anyGain ? wristUse : 1.0);
		jtOutputValue->setData(&control);

		if (recorder != NULL) {
			double* row = recorder->claim();
			if (row != NULL) {
				const double nan = std::numeric_limits<double>::quiet_NaN();
				RecorderRow r(row, recorder->columns());
				r.put(cycleStart);
				r.put(static_cast<double>(cycleCount));
				r.put(cycleDt);
				r.put(jp, 8);
				r.put(jv, 8);
				r.put(grav, 8);
				if (pidIn.valueDefined()) {
					r.put(pidIn.getValue(), 8);
				} else {
					for (int i = 0; i < 8; ++i) r.put(nan);
				}
				r.put(control, 8);
				r.put(theirJp, 8);
				r.put(ext, 4);
				r.put(sendExt, 4);
				r.put(theirExtTorque, 4);
				r.put(theirExtUsed, 4);
				r.put(dyn, 4);
				for (int k = 0; k < 3; ++k) r.put(theirWristExt[k]);
				for (int k = 0; k < 3; ++k) r.put(wristFf[k]);
				for (int k = 0; k < 3; ++k) r.put(wristRef[k]);
				r.put(wristUse);
				r.put(deadbandNow);
				r.put(static_cast<double>(sendFlags));
				r.put(static_cast<double>(theirFlags));
				const Link::Received& rx = link.latest();
				r.put(rxNew ? 1.0 : 0.0);
				r.put(rxAge);
				r.put(static_cast<double>(rx.sequence));
				r.put(static_cast<double>(link.getStats().lost));
				r.put(linkAlive ? 1.0 : 0.0);
				r.put(static_cast<double>(static_cast<int>(state)));
				r.put(static_cast<double>(mode.load()));
				r.put(mix.force);
				r.put(mix.own);
				r.put(mix.dyn);
				r.put(halted ? 1.0 : 0.0);
				r.put(sendOk ? 1.0 : 0.0);
				r.put(static_cast<double>(recorder->dropped()));
				recorder->commit();
			}
		}
	}

	jp_type theirJp;
	jt_type theirExtTorque;
	jt_type control;

private:
	DISALLOW_COPY_AND_ASSIGN(LeaderWam8g);
	friend class TeleopReference<LeaderWam8g, jp_type>;

	static void addColumns(std::vector<std::string>* c, const char* name, int n) {
		for (int i = 1; i <= n; ++i) {
			c->push_back(std::string(name) + "_" + std::to_string(i));
		}
	}

	// referenceSource's: the newest packet (non-blocking), first thing each
	// cycle. theirJp[4], the passive J5, is set in operate().
	const jp_type* receive() {
		if (!rtConfigured) {
			rt_thread::configureThisThread(rtPriority, rtCpu, &rtReport);
			rtConfigured = true;
		}
		const double now = teleop_link::now();
		cycleDt = cycleCount > 0 ? now - cycleStart : 0.0;
		cycleStart = now;
		++cycleCount;
		rxNew = false;
		if (halted) {
			linkAlive = false;
			if (!haveOwnJp) {
				return NULL;
			}
			theirJp = ownJp;
			return &theirJp;
		}
		rxNew = link.poll(now);
		linkAlive = link.valid() && (now - link.latest().arrival <= TIMEOUT);
		if (!linkAlive) {
			return NULL;
		}
		const Link::Received& rx = link.latest();
		teleop_law::adoptMode(rx.mode, rx.modeEpoch, &mode, &modeEpoch, &modeAdoptions);
		rxAge = rx.timed ? now - rx.sampleTime : std::numeric_limits<double>::quiet_NaN();
		Message position;
		if (ageReference && rx.timed) {
			position = teleop_law::agedReference(rx.jp, rx.jv, rxAge);
		} else {
			reference.update(rx.jp, rx.jv, rx.arrival, period);
			position = reference.position;
		}
		for (int i = 0; i < 4; ++i) {
			theirJp[i] = position[i];
			theirExtTorque[i] = rx.extTorque[i];
		}
		theirFlags = rx.flags;
		for (int k = 0; k < 3; ++k) {
			theirWristExt[k] = rx.extTorque[4 + k];
		}
		const Eigen::Matrix<double, 3, 1> theirWrist = position.tail<3>();
		wrist.toLeader(theirWrist, &theirJp);
		for (int k = 0; k < 3; ++k) {
			wristRef[k] = theirJp[5 + k];
		}
		// The tether (setWristDeadband()): within the slack the reference is
		// this arm's own wrist, so its position controller asks for nothing;
		// beyond it, the usual spring on what is beyond.
		if (deadbandNow > 0.0 && wamJPIn.valueDefined()) {
			const jp_type& own = wamJPIn.getValue();
			for (int k = 0; k < 3; ++k) {
				const int s = wrist.source[k];
				if (s >= 0) {
					theirJp[s] = own[s] + ft_core::softDeadband(theirJp[s] - own[s], deadbandNow);
				}
			}
		}
		return &theirJp;
	}

	Link link;
	const teleop_law::Law law;
	const wam8g_teleop::WristMap wrist;
	const bool sendExtTorque;
	const double TIMEOUT = 0.020;  // s without a packet: the link is lost
	State state;
	bool linkAlive;  // a packet within TIMEOUT (receive())
	std::atomic<bool> halted;  // halt()
	jp_type ownJp;  // this arm's position last cycle, its reference once halted
	bool haveOwnJp;
	double period;    // s, the execution manager's
	double fadeTime;  // s both arms have been linked (teleop_law::forceChannel)
	double forceScale[4];  // J1-J4: --force-gain / teleop_law::FORCE_GAIN
	double forceCutoff;  // Hz, 0: off (--force-filter)
	jt_type forceFiltered;
	double notchHz;  // Hz, 0: off (--force-notch)
	double j6Friction;  // Nm, 0: off (--j6-friction)
	std::atomic<int> mode;  // teleop_law::Mode (setMode(), or the follower's packets)
	std::atomic<unsigned int> modeEpoch;  // its change counter (teleop_link.h)
	std::atomic<unsigned int> modeAdoptions;
	std::atomic<unsigned int> linkLostEvents;
	teleop_law::ModeMix mix;  // the law's terms, fading to the mode
	teleop_law::Notch<jt_type> forceNotch;
	teleop_law::ReferenceTracker<Message, double> reference;
	bool ageReference;  // setAgeReference()
	int rtPriority, rtCpu;  // setRealtime()
	bool rtConfigured;
	rt_thread::Report rtReport;
	TeleopRecorder* recorder;
	double cycleStart;  // s, CLOCK_MONOTONIC at receive(): this cycle's sample time
	double cycleDt;     // s since the previous cycle's
	unsigned long cycleCount;
	bool rxNew;         // a packet arrived this cycle
	double rxAge;       // s, this cycle's start minus the packet's sample time (NaN: not timed)
	bool sendOk;

	// The wrist's force channel and tether.
	static constexpr double WRIST_FORCE_LIMIT = 0.8;      // Nm on a wrist joint, whatever the follower sends
	static constexpr double WRIST_FORCE_FILTER = 15.0;    // Hz
	static constexpr double WRIST_FORCE_DEADZONE = 0.03;  // Nm
	static constexpr double WRIST_RISE_TIME = 0.5;        // s of measured torques to full use
	static constexpr double WRIST_FALL_TIME = 0.25;       // s to none without them
	double wristForceGain[3];   // J6-J8 (setWristForce())
	double wristForceCutoff, wristDeadzone;
	double wristDeadband;       // rad (setWristDeadband())
	double deadbandNow;         // rad, the slack in effect
	double wristUse;            // 0..1
	double theirWristExt[3];    // the follower's wrist slots (its J5-J7)
	double wristFiltered[3];    // ... mapped to J6-J8 and low-passed
	double wristFf[3];          // the torque applied on J6-J8
	double wristRef[3];         // the follower's wrist as J6-J8, before the deadband
	uint32_t theirFlags;        // teleop_link::FLAG_*, from the follower's last packet
	uint32_t sendFlags;         // ... in this arm's
};
