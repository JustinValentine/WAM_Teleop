#pragma once

/* Follower side of the teleop (a 7-DOF WAM). The control loop itself talks to
 * the leader (teleop_link.h: no helper thread, no lock), sets up its own
 * thread (rt_thread.h: the legacy library leaves it non-real-time), and can
 * write one row per cycle to a recorder (teleop_recorder.h). Nothing in
 * operate() or receive() prints, allocates or waits. */

#include <atomic>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "ft_core.h"
#include "rt_thread.h"
#include "teleop_law.h"
#include "teleop_link.h"
#include "teleop_recorder.h"
#include "teleop_reference.h"
#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/single_io.h>
#include <barrett/systems/helpers.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/units.h>

template <size_t DOF>
class Follower : public barrett::systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    Input<jp_type> wamJPIn;
    Input<jv_type> wamJVIn;
    Input<jt_type> extTorqueIn;
    Input<jt_type> wamGravIn;
    Input<jt_type> wamDynIn;
    Input<jt_type> pidIn;  // the position controller's torque, for the recording only (may be unconnected)
    // The force/torque sensor's joint torques (FtTorques, lib/ft_systems.h; may be unconnected): setFt().
    Input<jt_type> ftTorqueIn;
    Output<jt_type> wamJPOutput;

  private:
    // Before theirJPOutput, its output.
    TeleopReference<Follower, jp_type> referenceSource;
    Input<jp_type> referenceIn;

  public:
    Output<jp_type>& theirJPOutput;  // position reference: the leader (teleop_reference.h)

    enum class State { INIT, LINKED, UNLINKED };

    explicit Follower(barrett::systems::ExecutionManager* em, const std::string& remoteHost, int rec_port = 5554,
                      int send_port = 5555, const std::string& sysName = "Follower")
        : System(sysName)
        , theirJp(0.0)
        , theirJv(0.0)
        , theirExtTorque(0.0)
        , control(0.0)
        , wamJPIn(this)
        , wamJVIn(this)
        , extTorqueIn(this)
        , wamGravIn(this)
        , wamDynIn(this)
        , pidIn(this)
        , ftTorqueIn(this)
        , wamJPOutput(this, &jtOutputValue)
        , referenceSource(this, sysName + "::Reference")
        , referenceIn(this)
        , theirJPOutput(referenceSource.output)
        , link(remoteHost, send_port, rec_port)
        , state(State::INIT) {

        law = 4;
        kp << 750, 1000, 400, 200, 10, 10, 2.5;
        kd << 8.3, 8, 3.3, 0.8, 0.5, 0.5, 0.05;
        cf << 0.375, 0.4, 0.2, 0.1, 0.01, 0.01, 0.01;

        period = (em != NULL) ? em->getPeriod() : 0.002;
        fadeTime = 0.0;
        for (int i = 0; i < 4; ++i) {
            forceScale[i] = 1.0;
        }
        forceCutoff = 0.0;
        forceFiltered.setZero();
        notchHz = 0.0;
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
        extTorque.setZero();
        ftReport = NULL;
        ftArm = ftWrist = false;
        for (int i = 0; i < 3; ++i) {
            wristFriction[i] = 0.0;
        }
        theirFlags = 0;
        sendFlags = 0;
        sendExtTorqueMsg.setZero();

        barrett::systems::connect(referenceSource.output, referenceIn);
        if (em != NULL) {
            em->startManaging(*this);
        }
    }

    virtual ~Follower() {
        referenceSource.detach();
        this->mandatoryCleanUp();
    }

    virtual bool inputsValid() {return true;}

    bool isLinked() const {
        return state == State::LINKED;
    }
    void tryLink() {
        BARRETT_SCOPED_LOCK(this->getEmMutex());
        if (!halted) {
            state = State::LINKED;
        }
    }
    // Stops the teleop for good (SafetyWatch, when this arm leaves ACTIVE; or
    // exiting): unlinked, and nothing more is sent, so the leader's link times
    // out and it holds where it is. This arm's reference becomes its own
    // position, so it does not jump when it is active again.
    void halt() { halted = true; }
    bool isHalted() const { return halted; }
    void unlink() {
        BARRETT_SCOPED_LOCK(this->getEmMutex());
        state = State::UNLINKED;
    }
    // Feedforward law on J1-J4, numbered as below (u1, u2, u4, u6); default u4.
    // The force channel's gain in u4 (teleop_law::parseForceGain).
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
    void setForceNotch(double hz) {
        BARRETT_SCOPED_LOCK(this->getEmMutex());
        notchHz = hz;
    }
    void setLaw(int u) {
        BARRETT_SCOPED_LOCK(this->getEmMutex());
        law = u;
    }
    // The mode (teleop_law::Mode): the law's terms fade to it. The change
    // also travels to the leader in this arm's packets (teleop_link.h).
    void setMode(teleop_law::Mode m) {
        mode = m;
        modeEpoch = modeEpoch.load() + 1;
    }
    teleop_law::Mode getMode() const { return static_cast<teleop_law::Mode>(mode.load()); }
    // Mode changes taken from the leader's packets so far.
    unsigned int getModeAdoptions() const { return modeAdoptions; }

    // Times the link dropped while linked (the loop no longer prints it).
    unsigned int getLinkLostEvents() const { return linkLostEvents; }
    // The leader's position extrapolated to this cycle with its sent velocity
    // and the packet's age (teleop_law::agedReference). Needs both programs
    // on one computer; otherwise, and when off, the reference advances only
    // in cycles without a new packet, as before.
    void setAgeReference(bool on) {
        BARRETT_SCOPED_LOCK(this->getEmMutex());
        ageReference = on;
    }
    // The force/torque sensor (lib/ft_core.h). report: FtTorques' (its output
    // connected to ftTorqueIn), read and recorded every cycle; NULL: none.
    // arm: J1-J4's external torque sent to the leader is the sensor's instead
    // of the model's estimate (which still takes over, gradually, whenever
    // the sensor is not usable). wrist: the three wrist slots carry the
    // sensor's torques, flagged, for the leader's --wrist-force.
    void setFt(const ft_core::Report* report, bool arm, bool wrist) {
        BARRETT_SCOPED_LOCK(this->getEmMutex());
        ftReport = report;
        ftArm = report != NULL && arm;
        ftWrist = report != NULL && wrist;
    }
    // A friction feedforward on this arm's wrist (J5-J7), Nm, in the direction
    // the leader's wrist turns (teleop_law::frictionFeedforward on the
    // reference's velocity): the wrist then lags the leader's less, and its
    // lag is what the leader's position coupling makes the operator feel.
    void setWristFriction(const double* nm) {
        BARRETT_SCOPED_LOCK(this->getEmMutex());
        for (int i = 0; i < 3; ++i) {
            wristFriction[i] = nm[i];
        }
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
    typename teleop_link::Link<DOF>::Stats getLinkStats() const {
        BARRETT_SCOPED_LOCK(this->getEmMutex());
        return link.getStats();
    }

    static std::vector<std::string> recordColumns() {
        std::vector<std::string> c;
        c.push_back("t");
        c.push_back("cycle");
        c.push_back("dt");
        addColumns(&c, "q", DOF);
        addColumns(&c, "dq", DOF);
        addColumns(&c, "tau_grav", DOF);
        addColumns(&c, "tau_pid", DOF);
        addColumns(&c, "tau_ff", DOF);
        addColumns(&c, "ref", DOF);
        addColumns(&c, "ext", 4);
        addColumns(&c, "ext_sent", 4);
        addColumns(&c, "ext_rx", 4);
        addColumns(&c, "ext_used", 4);
        addColumns(&c, "dyn", 4);
        addColumns(&c, "ext_sent_wrist", 3);  // the wrist slots as sent (the sensor's when tx_flags has 4)
        addColumns(&c, "ft_raw_f", 3);        // the sensor's last reading, its own axes (N, Nm)
        addColumns(&c, "ft_raw_n", 3);
        addColumns(&c, "ft_g", 3);            // gravity's direction in the tool frame
        addColumns(&c, "ft_f", 3);            // from the surroundings: zero and gripper weight removed, tool frame
        addColumns(&c, "ft_n", 3);
        addColumns(&c, "ft_tau", DOF);        // the same as joint torques
        const char* rest[] = {"ft_use", "ft_new", "ft_zeros", "ft_poll", "tx_flags", "rx_flags",
                              "rx_new", "rx_age", "rx_seq", "rx_lost", "link_alive", "state", "mode",
                              "mix_force", "mix_own", "mix_dyn", "halted", "send_ok", "rec_dropped"};
        for (size_t i = 0; i < sizeof(rest) / sizeof(rest[0]); ++i) {
            c.push_back(rest[i]);
        }
        return c;
    }

  protected:
    typename Output<jt_type>::Value* jtOutputValue;
    jp_type wamJP;
    jv_type wamJV;
    jt_type extTorque;
    jt_type wamGrav;
    jt_type wamDyn;
    Eigen::Matrix<double, DOF, 1> sendJpMsg;
    Eigen::Matrix<double, DOF, 1> sendJvMsg;
    Eigen::Matrix<double, DOF, 1> sendExtTorqueMsg;

    typedef teleop_link::Link<DOF> Link;

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

        wamJP = wamJPIn.getValue();
        wamJV = wamJVIn.getValue();
        wamGrav = wamGravIn.getValue();
        wamDyn = wamDynIn.getValue();
        ownJp = wamJP;  // for receive() next cycle
        haveOwnJp = true;

        if (extTorqueIn.valueDefined()) {
            extTorque = extTorqueIn.getValue();
        } else {
            extTorque.setZero();
        }

        // The force/torque sensor, when there is one: the joint torques the
        // surroundings cause through the gripper, already times its use.
        jt_type ftTau;
        ftTau.setZero();
        double ftUse = 0.0;
        const bool ftOn = ftReport != NULL && ftTorqueIn.valueDefined();
        if (ftOn) {
            ftTau = ftTorqueIn.getValue();
            ftUse = ftReport->use;
        }

        sendJpMsg << wamJP;
        sendJvMsg << wamJV;
        // The external torque only while linked (teleop_law::forceChannel).
        // It is minus the torque from outside (commanded minus model), so the
        // sensor's joint torques go in with a minus sign.
        sendFlags = 0;
        if (state == State::LINKED) {
            sendExtTorqueMsg << extTorque;
            sendFlags |= teleop_link::FLAG_LINKED;
            if (ftOn && ftArm) {
                for (int i = 0; i < 4; ++i) {
                    sendExtTorqueMsg[i] = (1.0 - ftUse) * extTorque[i] - ftTau[i];
                }
                if (ftUse > 0.0) {
                    sendFlags |= teleop_link::FLAG_MEASURED_ARM;
                }
            }
            if (ftOn && ftWrist && ftUse > 0.0) {
                for (size_t i = 4; i < DOF; ++i) {
                    sendExtTorqueMsg[i] = -ftTau[i];
                }
                sendFlags |= teleop_link::FLAG_MEASURED_WRIST;
            }
        } else {
            sendExtTorqueMsg.setZero();
        }

        if (halted) {
            state = State::UNLINKED;
        } else {
            // Stamped with the start of this cycle, when the joints were sampled.
            sendOk = link.send(sendJpMsg, sendJvMsg, sendExtTorqueMsg, cycleStart, static_cast<uint32_t>(mode.load()),
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
        jt_type extUsed(extTorque), dynUsed(wamDyn);
        for (int i = 0; i < 4; ++i) {
            theirExtUsed[i] *= forceScale[i] * mix.force;
            extUsed[i] *= mix.own;
            dynUsed[i] = wamGrav[i] + mix.dyn * (wamDyn[i] - wamGrav[i]);
        }

        switch (state) {
            case State::INIT:
                control.setZero();
                jtOutputValue->setData(&control);
                break;
            case State::LINKED:
                // Active teleop. Only the callee can transition to LINKED
                control = compute_control(theirJp, theirJv, theirExtUsed, wamJP, wamJV, extUsed, wamGrav, dynUsed);
                for (size_t i = 4; i < DOF; ++i) {  // setWristFriction()
                    control[i] += teleop_law::frictionFeedforward(wristFriction[i - 4], theirJv[i]);
                }
                jtOutputValue->setData(&control);
                break;
            case State::UNLINKED:
                // Changed to unlinked with either timeout or callee.
                control.setZero();
                jtOutputValue->setData(&control);
                break;
        }

        if (recorder != NULL) {
            double* row = recorder->claim();
            if (row != NULL) {
                const double nan = std::numeric_limits<double>::quiet_NaN();
                const int n = static_cast<int>(DOF);
                RecorderRow r(row, recorder->columns());
                r.put(cycleStart);
                r.put(static_cast<double>(cycleCount));
                r.put(cycleDt);
                r.put(wamJP, n);
                r.put(wamJV, n);
                r.put(wamGrav, n);
                if (pidIn.valueDefined()) {
                    r.put(pidIn.getValue(), n);
                } else {
                    for (int i = 0; i < n; ++i) r.put(nan);
                }
                r.put(control, n);
                r.put(theirJp, n);
                r.put(extTorque, 4);
                r.put(sendExtTorqueMsg, 4);
                r.put(theirExtTorque, 4);
                r.put(theirExtUsed, 4);
                r.put(wamDyn, 4);
                for (size_t i = 4; i < DOF; ++i) r.put(sendExtTorqueMsg[i]);
                if (ftReport != NULL) {
                    for (int i = 0; i < 3; ++i) r.put(ftReport->rawForce[i]);
                    for (int i = 0; i < 3; ++i) r.put(ftReport->rawTorque[i]);
                    for (int i = 0; i < 3; ++i) r.put(ftReport->gravity[i]);
                    for (int i = 0; i < 3; ++i) r.put(ftReport->force[i]);
                    for (int i = 0; i < 3; ++i) r.put(ftReport->torque[i]);
                    for (int i = 0; i < n; ++i) r.put(i < 7 ? ftReport->jointTorque[i] : nan);
                    r.put(ftReport->use);
                    r.put(ftReport->fresh);
                    r.put(ftReport->zeros);
                    r.put(ftReport->pollSeconds);
                } else {
                    for (int i = 0; i < 15 + n + 4; ++i) r.put(nan);
                }
                r.put(static_cast<double>(sendFlags));
                r.put(static_cast<double>(theirFlags));
                const typename Link::Received& rx = link.latest();
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
    jv_type theirJv;
    jt_type theirExtTorque;
    jt_type control;

  private:
    DISALLOW_COPY_AND_ASSIGN(Follower);
    friend class TeleopReference<Follower, jp_type>;

    static void addColumns(std::vector<std::string>* c, const char* name, size_t n) {
        for (size_t i = 1; i <= n; ++i) {
            c->push_back(std::string(name) + "_" + std::to_string(i));
        }
    }

    // referenceSource's: the newest packet, first thing each cycle.
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
        const typename Link::Received& rx = link.latest();
        teleop_law::adoptMode(rx.mode, rx.modeEpoch, &mode, &modeEpoch, &modeAdoptions);
        rxAge = rx.timed ? now - rx.sampleTime : std::numeric_limits<double>::quiet_NaN();
        if (ageReference && rx.timed) {
            theirJp = teleop_law::agedReference(rx.jp, rx.jv, rxAge);
        } else {
            reference.update(rx.jp, rx.jv, rx.arrival, period);
            theirJp = reference.position;
        }
        theirJv = rx.jv;
        theirExtTorque = rx.extTorque;
        theirFlags = rx.flags;
        return &theirJp;
    }

    Link link;
    const double TIMEOUT = 0.020;  // s without a packet: the link is lost
    State state;
    bool linkAlive;  // a packet within TIMEOUT (receive())
    std::atomic<bool> halted;  // halt()
    jp_type ownJp;  // this arm's position last cycle, its reference once halted
    bool haveOwnJp;
    Eigen::Matrix<double, DOF, 1> kp;
    Eigen::Matrix<double, DOF, 1> kd;
    Eigen::Matrix<double, DOF, 1> cf;
    int law;
    double period;    // s, the execution manager's
    double fadeTime;  // s both arms have been linked (teleop_law::forceChannel)
    double forceScale[4];  // J1-J4: --force-gain / teleop_law::FORCE_GAIN
    double forceCutoff;  // Hz, 0: off (--force-filter)
    jt_type forceFiltered;
    double notchHz;  // Hz, 0: off (--force-notch)
    std::atomic<int> mode;  // teleop_law::Mode (setMode(), or the leader's packets)
    std::atomic<unsigned int> modeEpoch;  // its change counter (teleop_link.h)
    std::atomic<unsigned int> modeAdoptions;
    std::atomic<unsigned int> linkLostEvents;
    teleop_law::ModeMix mix;  // the law's terms, fading to the mode
    teleop_law::Notch<jt_type> forceNotch;
    teleop_law::ReferenceTracker<Eigen::Matrix<double, DOF, 1>, double> reference;
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
    const ft_core::Report* ftReport;  // setFt()
    bool ftArm, ftWrist;
    double wristFriction[3];  // Nm on J5-J7 (setWristFriction())
    uint32_t theirFlags;      // teleop_link::FLAG_*, from the leader's last packet
    uint32_t sendFlags;       // ... in this arm's


    jt_type compute_control(const jp_type& ref_pos, const jv_type& ref_vel, const jt_type& ref_extTorque,
                            const jp_type& cur_pos, const jv_type& cur_vel, const jt_type& cur_extTorque,
                            const jt_type& cur_grav, const jt_type& cur_dyn) {

        // cases where the follower and leader have the same control law

        jt_type u1 = 0.0 * cur_extTorque; // zero feedforward (equal to default P-P with gravity compensation)

        jt_type u2 = cur_dyn - cur_grav; // P-P with dynamic compensation

        jt_type u4 = -0.5 * ref_extTorque + cur_dyn - cur_grav; // PF-PF with ref external torque feedback and dynamic compensation (Lawrence's perfect transparency architecture);

        jt_type u6 = -0.5 * ref_extTorque -0.15 * (ref_extTorque + cur_extTorque) + cur_dyn - cur_grav; // it has the best performance

        jt_type u = (law == 1) ? u1 : (law == 2) ? u2 : (law == 6) ? u6 : u4;

        for (size_t i = 4; i < 7; ++i) {
            u[i] = 0.0;
        }
        return u;
    };
};
