#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <barrett/log.h>
#include <barrett/os.h>
#include <barrett/products/product_manager.h>
#ifdef EXCITATION_CANOPEN
#define BARRETT_SMF_CONFIGURE_PM
#endif
#include <barrett/standard_main_function.h>
#include <barrett/systems.h>
#include <barrett/units.h>

#include "lib/cpu_latency.h"
#include "lib/rt_thread.h"
#include "lib/wrist_hold.h"

// Changes on thor to upstream dc29e27 (4c-dyntorq, "Updated readme and working
// excitation"), for the wam8g on libbarrett-canopen:
//  - the wam8g's wrist (J6-J8) has no velocity-control gains, so it is held
//    at its start angles with the configured position gains (lib/wrist_hold.h);
//    so is the P2's (J5-J7), whose velocity loop oscillated;
//  - J1-J4 are steered back toward the reference position, since velocity
//    tracking alone let the P2's J2 drift out of bounds;
//  - a bounds fault ends the run (it used to leave the program waiting), and
//    the program asks before moving afterwards;
//  - the program waits for Shift-Idle before it exits;
//  - the CANopen build refuses an eight-puck arm without a wam8g profile;
//  - the CANopen build raises the safety board's velocity limits (below);
//  - the wam8g's J4 factor is 0.7 (0.8 elsewhere).

using namespace barrett;

// The control thread, set up by the trajectory system in its first cycle
// (lib/rt_thread.h): the legacy library leaves it non-real-time, so the
// excitation's torques and velocities were computed whenever the thread got
// to run. EXCITATION_RT_CPU=n in the environment pins it to CPU n.
static rt_thread::Report excitationThread;
static bool excitationThreadDone = false;
static void configureExcitationThread() {
    if (!excitationThreadDone) {
        const char* cpu = std::getenv("EXCITATION_RT_CPU");
        rt_thread::configureThisThread(50, (cpu != NULL && cpu[0] != '\0') ? std::atoi(cpu) : -1, &excitationThread);
        excitationThreadDone = true;
    }
}

#ifdef EXCITATION_CANOPEN
// The CANopen safety board powers up with VL1/VL2 = 0.5/1.0 rad/s per joint
// (J1-J4), below the trajectory's 1.45 rad/s peak on J1 and J3, and the library
// leaves them alone. They stay as set here until the board's next power-up.
static const double VL_WARNING = 1.5;  // rad/s
static const double VL_FAULT = 2.0;    // rad/s

// On thor the container has no ~/.barrett, so without BARRETT_CONFIG_FILE
// libbarrett reads /etc/barrett, the P2 WAM's configuration, which has no
// wam8g profile: the wam8g's eight pucks would be taken for a 7-DOF WAM.
bool configure_pm(int, char**, ProductManager& pm) {
    pm.waitForWam(false);
    Puck* puck8 = pm.getPuck(8);
    if (!pm.foundWam8g() && puck8 != NULL && puck8->getType() == Puck::PT_Motor) {
        printf("Eight motor pucks but no wam8g profile in the configuration. For the wam8g,\n"
               "run 'source scripts/setup_leader_wam8g.sh' first (sets BARRETT_CONFIG_FILE).\n");
        return false;
    }
    return true;
}
#endif

template <size_t DOF> class FourierTrajectory : public systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    Input<double> timeInput;
    Input<jp_type> feedbackInput;
    Output<jv_type> referenceOutput;

    FourierTrajectory()
        : timeInput(this), feedbackInput(this), referenceOutput(this, &outputValue), faulted(false), is_active(false),
          max_duration(60.0), ref_valid(false), t_prev(0.0) {
        if (DOF == 7) {
            lower_bounds[0] = -2.4;
            upper_bounds[0] = 2.4;
            lower_bounds[1] = -1.8;
            upper_bounds[1] = 1.8;
            lower_bounds[2] = -2.5;
            upper_bounds[2] = 2.5;
            lower_bounds[3] = -0.8;
            upper_bounds[3] = 3.0;
            lower_bounds[4] = -0.5;
            upper_bounds[4] = 0.5;
            lower_bounds[5] = -0.5;
            upper_bounds[5] = 0.5;
            lower_bounds[6] = -0.5;
            upper_bounds[6] = 0.5;
        } else if (DOF == 4) {
            lower_bounds[0] = -2.4;
            upper_bounds[0] = 2.4;
            lower_bounds[1] = -1.8;
            upper_bounds[1] = 1.8;
            lower_bounds[2] = -2.5;
            upper_bounds[2] = 2.5;
            lower_bounds[3] = -0.8;
            upper_bounds[3] = 3.0;
        } else if (DOF == 8) {
            lower_bounds[0] = -2.4;
            upper_bounds[0] = 2.4;
            lower_bounds[1] = -1.8;
            upper_bounds[1] = 1.8;
            lower_bounds[2] = -2.5;
            upper_bounds[2] = 2.5;
            lower_bounds[3] = -0.8;
            upper_bounds[3] = 3.0;
            lower_bounds[4] = -0.5;
            upper_bounds[4] = 0.5;
            lower_bounds[5] = -0.5;
            upper_bounds[5] = 0.5;
            lower_bounds[6] = -0.5;
            upper_bounds[6] = 0.5;
            lower_bounds[7] = -0.5;
            upper_bounds[7] = 0.5;
        } else {
            throw std::logic_error("FATAL: FourierTrajectory is strictly defined for 4, 7 and 8 DOF.");
        }

        static const double a_b_q0_full[77] = {
            -0.22453075, -0.06509415, 0.0084825,   0.7463596,   0.04482256,  0.15292831,  0.17945343,  0.13698774,
            0.15005761,  0.40428706,  -0.06912397, 0.64039236,  0.41406122,  -0.05345818, 0.46419411,  0.27601025,
            0.55370932,  0.065848,    0.04071695,  0.41272949,  -0.17373824, 0.89269347,  0.45028845,  -0.26077073,
            0.10794414,  -0.16322598, 0.22051963,  -0.13005116, 0.38388683,  0.93214031,  0.36874565,  0.11058303,
            -0.13207445, 0.39799543,  0.28705749,  0.09974301,  -0.09950086, 0.77226083,  -0.28470714, 0.06468834,
            0.0187677,   -0.05244391, -0.40604485, 0.43133299,  -0.57178278, -0.12185623, 0.51881781,  0.27173848,
            0.15961459,  -0.29374854, 0.2251367,   -0.29917266, 0.09730888,  0.24762081,  0.07477825,  -0.11031733,
            0.08532278,  0.21811307,  -0.10056108, 0.56254872,  -0.02235595, 0.34816564,  -0.24721517, -0.1987875,
            -0.29918626, 0.19194917,  -0.39180923, -0.60461852, 0.00473514,  -0.24540649, -0.24181945, 0.03114698,
            -0.08510632, 1.54641938,  -1.67238949, -0.17570127, 0.5569734};

        typedef Eigen::Matrix<double, K_used, L, Eigen::RowMajor> MatrixKL;
        A = Eigen::Map<const MatrixKL>(a_b_q0_full);
        B = Eigen::Map<const MatrixKL>(a_b_q0_full + OFFSET_B);
        q0 = Eigen::Map<const Eigen::Matrix<double, K_used, 1>>(a_b_q0_full + OFFSET_Q0);

        // NOTE: Reduce factor if joint out of bounds or velocity limit
        // wam8g J4: 0.7 on the IROS base (Sept); 0.75 on WAM 153 from 2026-11-11 (user's choice).
        factors << 1.0, 1.0, 1.0, (DOF == 8 ? 0.75 : 0.8);
        for (int l = 0; l < L; ++l) {
            omegas(l) = (l + 1) * 0.314;
        }
    }
    void start(double duration) {
        BARRETT_SCOPED_LOCK(this->getEmMutex());
        max_duration = duration;
        is_active = true;
        faulted = false;
        ref_valid = false;
    }

    bool isActive() const {
        BARRETT_SCOPED_LOCK(this->getEmMutex());
        return is_active;
    }

    bool hasFaulted() const {
        BARRETT_SCOPED_LOCK(this->getEmMutex());
        return faulted;
    }

    const jp_type &lowerBounds() const { return lower_bounds; }
    const jp_type &upperBounds() const { return upper_bounds; }

    void evaluateTrajectory(double t, jp_type &q_full, jv_type &qdot_full) {
        Eigen::Matrix<double, L, 1> C;
        Eigen::Matrix<double, L, 1> S;
        Eigen::Matrix<double, L, 1> C_w;
        Eigen::Matrix<double, L, 1> S_w;

        for (int l = 0; l < L; ++l) {
            double w = omegas(l);
            double wt = w * t;
            double cos_wt = std::cos(wt);
            double sin_wt = std::sin(wt);
            C(l) = cos_wt;
            S(l) = sin_wt;

            C_w(l) = cos_wt / w;
            S_w(l) = sin_wt / w;
        }
        Eigen::Matrix<double, K_used, 1> q_base = A * S_w - B * C_w;
        Eigen::Matrix<double, K_used, 1> qdot_base = A * C + B * S;

        q_full.setZero();
        qdot_full.setZero();

        q_full.template head<K_used>() = (q0.array() + q_base.array()) * factors.array();
        qdot_full.template head<K_used>() = qdot_base.array() * factors.array();
    }

  protected:
    void operate() override {
        configureExcitationThread();
        // Latch the fault state to prevent resuming if the arm bounces back into bounds
        if (faulted) {
            outputValue->setData(&zero_vel);
            return;
        }

        const jp_type &current_pos = feedbackInput.getValue();
        for (size_t i = 0; i < DOF; i++) {
            if (current_pos[i] > (upper_bounds[i] + 0.05) || current_pos[i] < (lower_bounds[i] - 0.05)) {
                printf("Real-time fault: Joint %zu out of bounds.\n", i + 1);
                faulted = true;
                is_active = false;
                outputValue->setData(&zero_vel);
                return;
            }
        }

        double t = timeInput.getValue();

        if (t >= max_duration) {
            is_active = false;
            outputValue->setData(&zero_vel);
            return;
        }

        jp_type q;
        jv_type qdot;
        evaluateTrajectory(t, q, qdot);

        const double decay_duration = 1.0;
        if (t > max_duration - decay_duration) {
            double time_remaining = max_duration - t;
            double scale = 0.5 * (1.0 - std::cos(M_PI * time_remaining / decay_duration));
            qdot = scale * qdot;
        }

        // Velocity tracking alone let Zeus's J2 drift back 1 rad in 50 s (about
        // 1 Nm unbalanced, well inside its torque limit) and out of bounds. So
        // J1-J4 are also steered toward the reference position: the start plus
        // the integral of the commanded velocity, which follows the final decay.
        if (!ref_valid) {
            q_ref = current_pos;
            t_prev = t;
            ref_valid = true;
        }
        const double dt = t - t_prev;
        t_prev = t;
        for (int i = 0; i < K_used; ++i) {
            q_ref[i] += qdot[i] * dt;
            qdot[i] += POSITION_GAIN * (q_ref[i] - current_pos[i]);
        }

        outputValue->setData(&qdot);
    }

  private:
    typename Output<jv_type>::Value *outputValue;
    bool faulted;
    bool is_active;
    double max_duration;
    bool ref_valid;
    double t_prev;
    jp_type q_ref;

    jp_type lower_bounds;
    jp_type upper_bounds;
    const jv_type zero_vel = jv_type(0.0);

    static constexpr int K_full = 7;
    static constexpr int K_used = 4;
    static constexpr int L = 5;
    static constexpr double POSITION_GAIN = 2.0;  // 1/s, on the J1-J4 position error

    static constexpr int OFFSET_B = K_full * L;
    static constexpr int OFFSET_Q0 = 2 * K_full * L;

    Eigen::Vector4d factors;
    Eigen::Matrix<double, K_used, L> A;
    Eigen::Matrix<double, K_used, L> B;
    Eigen::Matrix<double, K_used, 1> q0;
    Eigen::Matrix<double, L, 1> omegas;
};


/* --trajectory <file> [log]: play a designed excitation (ident/: python -m
 * cable_ident design) with position control, and log every joint at 500 Hz
 * (t, q1-qN, dq1-dqN, tau1-tauN) for the identification. The file holds
 * t q1-q4 dq1-dq4 at a fixed rate; the other joints hold where they start.
 * The run stops (and the arm holds where it is) if a J1-J4 position leaves
 * the bounds or lags the reference by more than MAX_TRACKING_ERROR. */
static const double MAX_TRACKING_ERROR = 0.1;  // rad
static const double MAX_FILE_SPEED = 1.0;      // rad/s, J1-J4

struct TrajectoryFile {
    double dt;
    std::vector<Eigen::Vector4d> q;
    bool load(const std::string& path) {
        std::ifstream in(path.c_str());
        if (!in) {
            printf("Cannot open %s\n", path.c_str());
            return false;
        }
        std::string line;
        std::vector<double> t;
        while (std::getline(in, line)) {
            if (line.empty() || line[0] == '#') {
                continue;
            }
            std::istringstream s(line);
            double v[9];
            for (int i = 0; i < 9; ++i) {
                if (!(s >> v[i])) {
                    printf("%s: a line with fewer than 9 numbers (t q1-q4 dq1-dq4)\n", path.c_str());
                    return false;
                }
            }
            t.push_back(v[0]);
            q.push_back(Eigen::Vector4d(v[1], v[2], v[3], v[4]));
        }
        if (q.size() < 2) {
            printf("%s: no trajectory\n", path.c_str());
            return false;
        }
        dt = (t.back() - t.front()) / (t.size() - 1);
        if (!(dt > 0.0)) {
            printf("%s: times do not increase\n", path.c_str());
            return false;
        }
        for (size_t k = 1; k < q.size(); ++k) {
            if ((q[k] - q[k - 1]).cwiseAbs().maxCoeff() / dt > MAX_FILE_SPEED) {
                printf("%s: J1-J4 faster than %.1f rad/s at t = %.2f s; refused\n", path.c_str(), MAX_FILE_SPEED, t[k]);
                return false;
            }
        }
        return true;
    }
    double duration() const { return dt * (q.size() - 1); }
    Eigen::Vector4d at(double t) const {
        const double x = std::max(0.0, std::min(t / dt, (double) (q.size() - 1)));
        const size_t k = std::min((size_t) x, q.size() - 2);
        const double a = x - k;
        return (1.0 - a) * q[k] + a * q[k + 1];
    }
};

template <size_t DOF> class FilePlayer : public systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    Input<double> timeInput;
    Input<jp_type> feedbackInput;
    Output<jp_type> referenceOutput;

    FilePlayer(const TrajectoryFile &file, const jp_type &start, const jp_type &lower, const jp_type &upper)
        : timeInput(this), feedbackInput(this), referenceOutput(this, &outputValue), file(file), ref(start),
          lower(lower), upper(upper), faulted(false), done(false) {}
    virtual ~FilePlayer() { this->mandatoryCleanUp(); }

    bool hasFaulted() const { return faulted; }
    bool isDone() const { return done || faulted; }

  protected:
    void operate() override {
        configureExcitationThread();
        const jp_type &fb = feedbackInput.getValue();
        if (!faulted) {
            const double t = timeInput.getValue();
            const Eigen::Vector4d r = file.at(t);
            for (int i = 0; i < 4; ++i) {
                ref[i] = r[i];
            }
            for (int i = 0; i < 4; ++i) {
                if (fb[i] > upper[i] + 0.05 || fb[i] < lower[i] - 0.05) {
                    printf("Real-time fault: joint %d out of bounds.\n", i + 1);
                    faulted = true;
                } else if (std::fabs(fb[i] - ref[i]) > MAX_TRACKING_ERROR) {
                    printf("Real-time fault: joint %d lags the reference by %.3f rad.\n", i + 1, fb[i] - ref[i]);
                    faulted = true;
                }
            }
            if (faulted) {
                ref = fb;  // hold where it is
            }
            done = t >= file.duration();
        }
        outputValue->setData(&ref);
    }

  private:
    typename Output<jp_type>::Value *outputValue;
    const TrajectoryFile &file;
    jp_type ref;
    const jp_type lower, upper;
    bool faulted, done;
};

template <size_t DOF>
int playTrajectory(ProductManager &pm, systems::Wam<DOF> &wam, const std::string &trajPath, const std::string &logPath) {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
    TrajectoryFile file;
    if (!file.load(trajPath)) {
        return 1;
    }
    FourierTrajectory<DOF> limits;  // its J1-J4 bounds
    jp_type lower = limits.lowerBounds(), upper = limits.upperBounds();
    for (size_t k = 0; k < file.q.size(); k += 10) {
        for (int i = 0; i < 4; ++i) {
            if (file.q[k][i] < lower[i] || file.q[k][i] > upper[i]) {
                printf("%s: J%d leaves its bounds at t = %.2f s; refused\n", trajPath.c_str(), i + 1, k * file.dt);
                return 1;
            }
        }
    }
    printf("Trajectory %s: %.0f s at %.0f Hz; log %s (all joints, 500 Hz)\n", trajPath.c_str(), file.duration(),
           1.0 / file.dt, logPath.c_str());
#ifdef EXCITATION_CANOPEN
    SafetyModule *sm = pm.getSafetyModule();
    sm->setVelocityLimit(VL_FAULT, VL_WARNING);
#endif
    wam.gravityCompensate();
    jp_type start = wam.getJointPositions();  // the wrist (J5..) holds where it is
    const Eigen::Vector4d q0 = file.at(0.0);
    for (int i = 0; i < 4; ++i) {
        start[i] = q0[i];
    }
    printf("Moving to the trajectory's start (J1-J4 %.3f %.3f %.3f %.3f) ...\n", q0[0], q0[1], q0[2], q0[3]);
    wam.moveTo(start);
    btsleep(1.0);

    char tmpFile[] = "/tmp/btXXXXXX";
    if (mkstemp(tmpFile) == -1) {
        printf("ERROR: Couldn't create temporary file!\n");
        return 1;
    }
    systems::Ramp time(pm.getExecutionManager(), 1.0);
    FilePlayer<DOF> player(file, start, lower, upper);
    typedef boost::tuple<double, jp_type, jv_type, jt_type> tuple_type;
    systems::TupleGrouper<double, jp_type, jv_type, jt_type> tg;
    systems::PeriodicDataLogger<tuple_type> logger(
        pm.getExecutionManager(),
        new log::RealTimeWriter<tuple_type>(tmpFile, pm.getExecutionManager()->getPeriod()), 1);
    systems::connect(time.output, player.timeInput);
    systems::connect(wam.jpOutput, player.feedbackInput);
    systems::connect(time.output, tg.template getInput<0>());
    systems::connect(wam.jpOutput, tg.template getInput<1>());
    systems::connect(wam.jvOutput, tg.template getInput<2>());
    systems::connect(wam.jtSum.output, tg.template getInput<3>());

    printf("Press [Enter] to play the trajectory (keep the E-stop at hand)");
    detail::waitForEnter();
    systems::connect(tg.output, logger.input);
    wam.trackReferenceSignal(player.referenceOutput);
    time.start();
    while (!player.isDone()) {
        btsleep(0.1);
    }
    btsleep(0.5);
    logger.closeLog();
    time.stop();
    wam.moveTo(wam.getJointPositions());  // hold
    printf(player.hasFaulted() ? "The run stopped early; the log holds what ran.\n" : "Done.\n");
    rt_thread::print("Excitation", excitationThread);

    log::Reader<tuple_type> lr(tmpFile);
    std::ofstream out(logPath.c_str());
    if (!out.is_open()) {
        printf("ERROR: Could not open %s for writing.\n", logPath.c_str());
    } else {
        out.precision(15);
        for (size_t i = 0; i < lr.numRecords(); ++i) {
            tuple_type r = lr.getRecord();
            out << boost::get<0>(r);
            for (size_t j = 0; j < DOF; ++j) out << " " << boost::get<1>(r)[j];
            for (size_t j = 0; j < DOF; ++j) out << " " << boost::get<2>(r)[j];
            for (size_t j = 0; j < DOF; ++j) out << " " << boost::get<3>(r)[j];
            out << "\n";
        }
        printf("Output written to %s.\n", logPath.c_str());
    }
    std::remove(tmpFile);
    printf("Shift-Idle the WAM to finish.\n");
    pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
    return player.hasFaulted() ? 1 : 0;
}

template <size_t DOF> int wam_main(int argc, char **argv, ProductManager &pm, systems::Wam<DOF> &wam) {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
    if (!rt_thread::lockMemory()) {
        printf("Warning: cannot lock the program's memory (mlockall): page faults can delay the loop.\n");
    }
    if (argc > 2 && std::string(argv[1]) == "--trajectory") {
        CpuLatencyHold cpuLatency;
        return playTrajectory<DOF>(pm, wam, argv[2], argc > 3 ? argv[3] : "rbtlog_trajectory.dat");
    }
    CpuLatencyHold cpuLatency;  // the 500 Hz loop's timing as in teleop (lib/cpu_latency.h)

    std::string file_path = "rbtlog_4_dof.dat";
    if (argc > 1) {
        file_path = argv[1];
    }

    // Parse the directory component and create it
    size_t last_slash_idx = file_path.find_last_of('/');
    if (last_slash_idx != std::string::npos) {
        std::string dir_path = file_path.substr(0, last_slash_idx);
        std::string mkdir_cmd = "mkdir -p " + dir_path;
        if (system(mkdir_cmd.c_str()) != 0) {
            printf("Could not create directory structure %s\n", dir_path.c_str());
            return 1;
        }
    }

    // Initialize temporary binary file for real-time logging
    char tmpFile[] = "/tmp/btXXXXXX";
    if (mkstemp(tmpFile) == -1) {
        printf("ERROR: Couldn't create temporary file!\n");
        return 1;
    }

#ifdef EXCITATION_CANOPEN
    SafetyModule *sm = pm.getSafetyModule();
    sm->setVelocityLimit(VL_FAULT, VL_WARNING);
    printf("Safety board velocity limits (rad/s per joint): warning %.2f, fault %.2f\n",
           sm->getPuck()->getProperty(Puck::VL1) / 4096.0, sm->getPuck()->getProperty(Puck::VL2) / 4096.0);
#endif

    wam.gravityCompensate();

    systems::Ramp time(pm.getExecutionManager(), 1.0);
    FourierTrajectory<DOF> trajGen;

    jp_type q_init;
    jv_type qdot_init;
    trajGen.evaluateTrajectory(0.0, q_init, qdot_init);

    wam.moveTo(q_init);
    btsleep(2.0);

    // Hold the wrist where moveTo left it, with the configured position gains,
    // while J1-J4 track the trajectory velocity: the wam8g's J6-J8 (J5 is
    // passive) and the P2's J5-J7. The P2 wrist's velocity loop, low-pass
    // filtered at omega_p 10/30/3, oscillated at 7 and 13 Hz on Zeus with its
    // hand, so its velocity gains are zeroed and the hold alone acts there.
    const bool holdWrist = (DOF == 7 || DOF == 8);
    const size_t wristFirst = (DOF == 8) ? 5 : 4;
    WristHold<DOF> wristHold(wristFirst, wam.getJointPositions(), wam.jpController.getKp(), wam.jpController.getKd(),
                             wam.jpController.getControlSignalLimit());
    if (holdWrist) {
        v_type kp = wam.jvController1.getKp(), ki = wam.jvController1.getKi(), kd = wam.jvController1.getKd();
        for (size_t i = wristFirst; i < DOF; ++i) {
            kp[i] = ki[i] = kd[i] = 0.0;
        }
        wam.jvController1.setKp(kp);
        wam.jvController1.setKi(ki);
        wam.jvController1.setKd(kd);
    }

    typedef boost::tuple<double, jp_type, jt_type> tuple_type;
    systems::TupleGrouper<double, jp_type, jt_type> tg;

    const size_t PERIOD_MULTIPLIER = 2;
    systems::PeriodicDataLogger<tuple_type> logger(
        pm.getExecutionManager(),
        new log::RealTimeWriter<tuple_type>(tmpFile, PERIOD_MULTIPLIER * pm.getExecutionManager()->getPeriod()),
        PERIOD_MULTIPLIER);

    systems::connect(time.output, trajGen.timeInput);
    systems::connect(wam.jpOutput, trajGen.feedbackInput);

    systems::connect(time.output, tg.template getInput<0>());
    systems::connect(wam.jpOutput, tg.template getInput<1>());
    systems::connect(wam.jtSum.output, tg.template getInput<2>());

    printf("Press [Enter] to excite the wam");
    detail::waitForEnter();
    printf("Starting trajectory generation and logging...\n");

    systems::connect(tg.output, logger.input);
    if (holdWrist) {
        systems::connect(wam.jpOutput, wristHold.jpInput);
        systems::connect(wam.jvOutput, wristHold.jvInput);
        systems::connect(wristHold.output, wam.input);
    }
    wam.trackReferenceSignal(trajGen.referenceOutput);
    trajGen.start(60.0);
    time.start();

    while (trajGen.isActive()) {
        btsleep(0.1);
    }

    printf("Stopping...\n");
    logger.closeLog();
    time.stop();
    rt_thread::print("Excitation", excitationThread);

    if (trajGen.hasFaulted()) {
        printf("The run stopped early. Press [Enter] to move to zero and home (or Shift-Idle now).");
        detail::waitForEnter();
    }
    if (holdWrist) {
        systems::disconnect(wam.input);
    }
    wam.idle();
    wam.moveTo(jp_type(0.0));
    wam.moveHome();

    log::Reader<tuple_type> lr(tmpFile);
    std::ofstream out(file_path);

    if (!out.is_open()) {
        printf("ERROR: Could not open %s for writing.\n", file_path.c_str());
    } else {
        out.precision(15);
        // NOTE: Temporarily save 4dof for demo. Use full DOF in future.
        const int DOF_WRITE = 4;

        for (size_t i = 0; i < lr.numRecords(); ++i) {
            tuple_type record = lr.getRecord();

            double t = boost::get<0>(record);
            jp_type pos = boost::get<1>(record);
            jt_type eff = boost::get<2>(record);

            out << t;
            for (size_t j = 0; j < DOF_WRITE; ++j)
                out << " " << pos[j];
            for (size_t j = 0; j < DOF_WRITE; ++j)
                out << " " << eff[j];
            out << "\n";
        }
        out.close();
        printf("Output written to %s.\n", file_path.c_str());
    }

    std::remove(tmpFile);
    wam.idle();

    printf("Shift-Idle the WAM to finish.\n");
    pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
    return 0;
}
