/*
 * ex11_master_master.cpp
 *
 *  Created on: Feb 22, 2010
 *      Author: Christopher Dellin
 *      Author: Dan Cody
 *      Author: Brian Zenowich
 */

// A version of 7-DOF follower.

#include "lib/external_torque.h"
#include <atomic>
#include <iostream>
#include <cstdlib>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <boost/thread.hpp>

#include <barrett/detail/stl_utils.h>
#include <barrett/os.h>
#include <barrett/products/product_manager.h>
#include <barrett/systems.h>
#include <barrett/units.h>

#define BARRETT_SMF_VALIDATE_ARGS
#include <barrett/standard_main_function.h>

#include "lib/follower.h"
#include "lib/ft_barrett.h"
#include "lib/ft_systems.h"
#include "lib/background_state_publisher.h"
#include "lib/clean_exit.h"
#include "lib/cpu_latency.h"
#include "lib/teleop_mode.h"
#include "lib/safety_watch.h"
// odin, 2026-11-11: the follower here is slax with the skid gripper and the F/T sensor
// (Serg's fit); thor's Zeus with its hand is lib/follower_beta_zeus_hand.h.
#include "lib/follower_beta_slax_skid_hand.h"
#include "lib/estimator_systems.h"
#include "lib/teleop_app.h"
#include "lib/teleop_recorder.h"
#include "lib/cable_drive_dynamics.h"
#include "lib/cable_params_zeus_hand.h"
#include "lib/dynamic_external_torque.h"
#include "lib/teleop_law.h"
#include "lib/wam8g_teleop.h"

#ifdef FOLLOWER_HAND
#include "lib/hand_teleop_ros.h"

// follower --hand: the hand's pucks at startup, where the hand does not move;
// HI, the fist and the command thread at the link pose (startHand), where the
// spread has room to swing. NULL/nullptr (and a message) if the hand is not
// usable; the teleop works without it.
barrett::Hand* findHand(barrett::ProductManager& pm) {
    try {
        barrett::Hand* hand = pm.getHand();
        printf("Hand: found; it is initialized at the link pose ('l').\n");
        return hand;
    } catch (const std::exception& e) {
        printf("Hand: %s\nHand: continuing without the hand.\n", e.what());
        return NULL;
    }
}

std::unique_ptr<HandTeleopThread> startHand(barrett::Hand& hand, const std::function<bool()>& armStopped,
                                            bool spreadPoses) {
    printf("BarrettHand: [Enter] to initialize it here (HI: the fingers open fully and the\n"
           "spread swings through its range; keep clear), or 's' [Enter] to skip HI when it\n"
           "was initialized since it was powered on. Either way it then closes to a fist: ");
    std::string line;
    if (!clean_exit::readLine(&line)) {  // stopped: not now
        printf("Hand: left as it is.\n");
        return nullptr;
    }
    try {
        HandTeleop::Params params;
        params.poses = spreadPoses;
        std::unique_ptr<HandTeleopThread> t = startHandTeleop(hand, line != "s", armStopped, params);
        printf("Hand: in a fist at spread 0%%, listening on follower/hand/velocity_setpoint (the grasp)\n"
               "and follower/hand/next_pose (the spread: 0%% -> 50%% pinch -> 100%%; L2 + cross).\n");
        return t;
    } catch (const std::exception& e) {
        printf("Hand: %s\nHand: continuing without the hand.\n", e.what());
        try {
            hand.idle();
        } catch (const std::exception&) {
        }
        return nullptr;
    }
}
#endif

using namespace barrett;
using detail::waitForEnter;

void printUsage(const std::string& programName, const std::string& remoteHost, int recPort, int sendPort) {
    std::cout << "Usage: " << programName << " [--law u1|u2|u4|u6] [--force-gain g] [--force-filter hz] [--force-notch hz] [--velocity-limit v] [--sync wam8g] [--hand [--no-spread-poses]]"
              << " [--model legacy|cable [--ff-friction s] [--ff-inertia s] [--no-wrist-gravity]]"
              << " [--ft log|wrist|arm|all [--ft-tool file] [--ft-filter hz]] [--wrist-friction nm]"
              << " [remoteHost] [recPort] [sendPort]"
              << std::endl;
    std::cout << "       Defaults: remoteHost=" << remoteHost << ", recPort=" << recPort << ", sendPort=" << sendPort
              << std::endl;
    std::cout << "       --law: feedforward on J1-J4 (lib/teleop_law.h), default u4." << std::endl;
    std::cout << "       --force-gain: the force channel's gain in u4 (0 to 1), one for J1-J4 or four" << std::endl;
    std::cout << "           comma-separated (e.g. 0.4,0.4,0.2,0.4), default 0.5." << std::endl;
    std::cout << "       --force-filter: a low-pass on the force channel in Hz, default 0 (off)." << std::endl;
    std::cout << "       --force-notch: a notch on the force channel in Hz (Q 0.7), default 0 (off)." << std::endl;
    std::cout << "       --velocity-limit: the safety module's velocity fault limit in m/s (0 to 5]," << std::endl;
    std::cout << "           default 3 (warning at 90%); the limits in effect are printed at startup." << std::endl;
    std::cout << "       --sync wam8g: the sync pose for the wam8g leader (leader_wam8g)." << std::endl;
    std::cout << "       --hand: drive the BarrettHand on this bus (pucks 11-14) from" << std::endl;
    std::cout << "               follower/hand/velocity_setpoint (the grasp) and follower/hand/next_pose (the" << std::endl;
    std::cout << "               spread: 0% -> 50% pinch -> 100%; launch/hand_nav_controller.launch)." << std::endl;
    std::cout << "       --no-spread-poses: with --hand, the grasp only; the spread stays at 0%." << std::endl;
    std::cout << "       --model: the dynamics model of the external-torque estimate and the feedforward:" << std::endl;
    std::cout << "           legacy (lib/follower_dynamics.h, default) or cable (the cable-drive model," << std::endl;
    std::cout << "           lib/cable_params_zeus_hand.h from ident/)." << std::endl;
    std::cout << "       --ff-friction, --ff-inertia: with --model cable, the feedforward's shares (0 to 1) of" << std::endl;
    std::cout << "           the cable friction (default 0) and of the inertial torque (default 0.25, as legacy);" << std::endl;
    std::cout << "           the estimate always uses all of both. 'm' changes them live." << std::endl;
    std::cout << "       --no-wrist-gravity: with --model cable, leave out the wrist's gravity correction." << std::endl;
    std::cout << "       --ft: the force/torque sensor on this bus (lib/ft_core.h), read every cycle without waiting:" << std::endl;
    std::cout << "           log    read and record it (--record), nothing else: the teleop is unchanged;" << std::endl;
    std::cout << "           wrist  + send the wrist torques it measures to the leader (its --wrist-force);" << std::endl;
    std::cout << "           arm    + J1-J4's external torque sent to the leader is the sensor's, not the model's;" << std::endl;
    std::cout << "           all    wrist and arm." << std::endl;
    std::cout << "           Its zero is taken at the link pose ('l') and by 'z': nothing may touch the gripper then." << std::endl;
    std::cout << "       --ft-tool: the gripper's mass, centre of mass and the sensor's axes, from" << std::endl;
    std::cout << "           scripts/ft_tool_cal.py on a recording made with --ft log. wrist, arm and all need it:" << std::endl;
    std::cout << "           without it the gripper's own weight would reach the leader as the wrist turns." << std::endl;
    std::cout << "       --ft-filter: the low-pass on the measured wrench in Hz (0: off), default 40." << std::endl;
    std::cout << "       --wrist-friction: a friction feedforward on this arm's J5-J7 in Nm (0 to 0.5), one value or" << std::endl;
    std::cout << "           three comma-separated, in the direction the leader's wrist turns; default 0 (off)." << std::endl;
    std::cout << teleop_app::usage();
    std::cout << "       -h or --help: Display this help message." << std::endl;
}

struct FollowerOptions {
    teleop_law::Law law = teleop_law::U4;
    teleop_law::ForceGains forceGain;  // 0.5 on J1-J4
    double forceFilter = 0.0;  // Hz, 0: off
    double forceNotch = 0.0;  // Hz, 0: off
    double velocityLimit = 3.0;  // m/s (--velocity-limit)
    bool wam8g = false;
    bool hand = false;
    bool spreadPoses = true;  // --no-spread-poses: the grasp only
    bool cableModel = false;  // --model cable
    double ffFriction = 0.0;  // --ff-friction
    double ffInertia = 0.25;  // --ff-inertia (legacy: the acceleration x0.25 in FollowerDynamics)
    bool wristGravity = true;  // --no-wrist-gravity clears it
    bool ft = false;           // --ft: read (and record) the force/torque sensor
    bool ftWrist = false;      // --ft wrist|all
    bool ftArm = false;        // --ft arm|all
    std::string ftToolFile;    // --ft-tool
    ft_core::Tool ftTool;      // ... loaded
    double ftFilter = ft_core::Limits().filterHz;  // --ft-filter, Hz
    double wristFriction[3] = {0.0, 0.0, 0.0};     // --wrist-friction, Nm on J5-J7
    teleop_app::Options app;  // the low-latency options (lib/teleop_app.h)
    std::vector<std::string> positional;
};

// A share from 0 to 1 (--ff-friction, --ff-inertia).
bool parseShare(const std::string& s, double* v) {
    char* end = NULL;
    const double x = std::strtod(s.c_str(), &end);
    if (end == s.c_str() || *end != '\0' || !(x >= 0.0 && x <= 1.0)) {
        return false;
    }
    *v = x;
    return true;
}

// --velocity-limit: the P2 safety module's velocity fault limit in m/s (it
// limits the arm's Cartesian speed, not a joint's); the library reloads the
// module's defaults at every start, so it is set again after startup.
bool parseVelocityLimit(const std::string& s, double* v) {
    char* end = NULL;
    const double x = std::strtod(s.c_str(), &end);
    if (end == s.c_str() || *end != '\0' || !(x > 0.0 && x <= 5.0)) {
        return false;
    }
    *v = x;
    return true;
}

bool parseOptions(int argc, char** argv, FollowerOptions* o) {
    for (int i = 1; i < argc; ++i) {
        const std::string a(argv[i]);
        const int app = teleop_app::parse(argc, argv, &i, &o->app, "follower");
        if (app < 0) {
            return false;
        } else if (app > 0) {
            continue;
        }
        if (a == "--law" && i + 1 < argc && teleop_law::parse(argv[i + 1], &o->law)) {
            ++i;
        } else if (a == "--force-gain" && i + 1 < argc && teleop_law::parseForceGain(argv[i + 1], &o->forceGain)) {
            ++i;
        } else if (a == "--force-filter" && i + 1 < argc && teleop_law::parseForceFilter(argv[i + 1], &o->forceFilter)) {
            ++i;
        } else if (a == "--force-notch" && i + 1 < argc && teleop_law::parseForceFilter(argv[i + 1], &o->forceNotch)) {
            ++i;
        } else if (a == "--velocity-limit" && i + 1 < argc && parseVelocityLimit(argv[i + 1], &o->velocityLimit)) {
            ++i;
        } else if (a == "--sync" && i + 1 < argc && std::string(argv[i + 1]) == "wam8g") {
            o->wam8g = true;
            ++i;
        } else if (a == "--model" && i + 1 < argc && (std::string(argv[i + 1]) == "legacy" || std::string(argv[i + 1]) == "cable")) {
            o->cableModel = std::string(argv[i + 1]) == "cable";
            ++i;
        } else if (a == "--ff-friction" && i + 1 < argc && parseShare(argv[i + 1], &o->ffFriction)) {
            ++i;
        } else if (a == "--ff-inertia" && i + 1 < argc && parseShare(argv[i + 1], &o->ffInertia)) {
            ++i;
        } else if (a == "--no-wrist-gravity") {
            o->wristGravity = false;
        } else if (a == "--ft") {
            const std::string v(i + 1 < argc ? argv[i + 1] : "");
            if (v != "log" && v != "wrist" && v != "arm" && v != "all") {
                printf("follower: --ft takes log, wrist, arm or all.\n");
                return false;
            }
            o->ft = true;
            o->ftWrist = (v == "wrist" || v == "all");
            o->ftArm = (v == "arm" || v == "all");
            ++i;
        } else if (a == "--ft-tool") {
            std::string error;
            if (i + 1 >= argc || !o->ftTool.load(argv[i + 1], &error)) {
                printf("follower: --ft-tool: %s\n", i + 1 < argc ? error.c_str() : "takes the file ft_tool_cal.py wrote");
                return false;
            }
            o->ftToolFile = argv[i + 1];
            ++i;
        } else if (a == "--ft-filter") {
            if (!teleop_app::parseList(i + 1 < argc ? argv[i + 1] : NULL, 1, 0.0, 250.0, &o->ftFilter)) {
                printf("follower: --ft-filter takes Hz from 0 (off) to 250.\n");
                return false;
            }
            ++i;
        } else if (a == "--wrist-friction") {
            if (!teleop_app::parseList(i + 1 < argc ? argv[i + 1] : NULL, 3, 0.0, 0.5, o->wristFriction)) {
                printf("follower: --wrist-friction takes Nm from 0 to 0.5, one value or three comma-separated (J5,J6,J7).\n");
                return false;
            }
            ++i;
        } else if (a == "--hand") {
            o->hand = true;
        } else if (a == "--no-spread-poses") {
            o->spreadPoses = false;
        } else if (a.find(":=") != std::string::npos) {
            // a ROS remapping argument; ros::init() takes it
        } else if (!a.empty() && a[0] == '-') {
            return false;
        } else {
            o->positional.push_back(a);
        }
    }
    if (o->hand && !o->app.ros) {
        printf("follower: --hand takes its commands over ROS; not with --no-ros.\n");
        return false;
    }
    if ((o->ftWrist || o->ftArm) && !o->ftTool.calibrated) {
        printf("follower: --ft wrist, arm and all need --ft-tool (the gripper's calibration): without it the\n"
               "gripper's own weight would push the leader as the wrist turns. Record with --ft log, then run\n"
               "scripts/ft_tool_cal.py on the recording.\n");
        return false;
    }
    if (!o->ft && !o->ftToolFile.empty()) {
        printf("follower: --ft-tool without --ft.\n");
        return false;
    }
    return o->positional.size() <= 3;
}

bool validate_args(int argc, char** argv) {
    FollowerOptions o;
    if (!parseOptions(argc, argv, &o)) {
        printUsage(argv[0], "127.0.0.1", 5554, 5555);
        return 0;
    }

    return true;
}

template <size_t DOF> int wam_main(int argc, char **argv, ProductManager &pm, systems::Wam<DOF> &wam) {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
    CpuLatencyHold cpuLatency;  // until the program ends

    FollowerOptions options;
    parseOptions(argc, argv, &options); // checked in validate_args()

    SafetyModule* safety = pm.getSafetyModule();
    if (safety != NULL) {
        safety->setVelocityLimit(options.velocityLimit);  // warning at 90%
        printf("Safety velocity limits (m/s): warning %.2f, fault %.2f\n",
               safety->getPuck()->getProperty(Puck::VL1) / 4096.0, safety->getPuck()->getProperty(Puck::VL2) / 4096.0);
    }

    jp_type SYNC_POS; // the position each WAM should move to before linking
    if (DOF == 7) {
        SYNC_POS[0] = 0.0;
        SYNC_POS[1] = -1.70;
        SYNC_POS[2] = 0.0;
        SYNC_POS[3] = 2.97;
        SYNC_POS[4] = 0.0;
        SYNC_POS[5] = 0.0;
        SYNC_POS[6] = 0.0;
        if (options.wam8g) {
            // the wam8g leader's sync pose (lib/wam8g_teleop.h)
            for (size_t i = 0; i < DOF; ++i) {
                SYNC_POS[i] = wam8g_teleop::FOLLOWER_SYNC_POSE[i];
            }
        }

    } else {
        printf("Error: 7 DOF supported\n");
        return false;
    }

#ifdef FOLLOWER_HAND
    if (options.hand && !pm.foundHand()) {
        printf("Error: --hand, but no BarrettHand (pucks 11-14) on this bus.\n");
        return 1;
    }
#else
    if (options.hand) {
        printf("Error: --hand: this follower was built without gripper_ros_common.\n");
        return 1;
    }
#endif

    std::string remoteHost = "127.0.0.1";
    int rec_port = 5554;
    int send_port = 5555;

    if (options.positional.size() >= 1) {
        remoteHost = options.positional[0];
    }
    if (options.positional.size() >= 2) {
        rec_port = std::atoi(options.positional[1].c_str());
    }
    if (options.positional.size() >= 3) {
        send_port = std::atoi(options.positional[2].c_str());
    }

    // --ft: the force/torque sensor (puck 8). Opening it wakes and tares it.
    ForceTorqueSensor* fts = NULL;
    if (options.ft) {
        if (!pm.foundForceTorqueSensor()) {
            printf("Error: --ft, but no force/torque sensor (puck 8) on this bus.\n");
            return 1;
        }
        fts = pm.getForceTorqueSensor();
    }

    // The legacy library neither makes the control thread real-time nor locks
    // the program's memory: the loop does the first itself (lib/rt_thread.h,
    // follower.setRealtime below), and the memory is locked here.
    if (!rt_thread::lockMemory()) {
        printf("Warning: cannot lock the program's memory (mlockall): page faults can delay the loop.\n");
    }

    // ROS: the joint_states/gravity publisher (the source of the ROS
    // recording; a 500 Hz thread that takes the control loop's lock), the mode
    // topic and the hand's commands. --no-ros-state drops the publisher,
    // --no-ros everything.
    teleop_app::Lazy<BackgroundStatePublisher<DOF> > state_publisher;
    if (options.app.ros) {
        ros::init(argc, argv, "follower");
        if (options.app.rosState) {
            state_publisher.emplace(pm.getExecutionManager(), wam);
        }
    }

    barrett::systems::Summer<jt_type, 3> customjtSum;
    pm.getExecutionManager()->startManaging(customjtSum);

    // The fitted model with a quarter of the acceleration (as
    // FollowerDynamics), on fixed arrays.
    RegressorDynamics<DOF> followerDynamics(initialize_follower_beta(), 0.25);

    ExternalTorque<DOF> externalTorque(pm.getExecutionManager());

    DynamicExternalTorque<DOF> dynamicExternalTorque(pm.getExecutionManager());
    
    barrett::systems::FirstOrderFilter<jt_type> extFilter;
    jt_type omega_p(180.0);
    extFilter.setLowPass(omega_p);
    pm.getExecutionManager()->startManaging(extFilter);

    ja_type ja;
    ja.setConstant(0.0);
    systems::Constant<ja_type> zeroAcceleration(ja);
    pm.getExecutionManager()->startManaging(zeroAcceleration);

    Follower<DOF> follower(pm.getExecutionManager(), remoteHost, rec_port, send_port);
    follower.setLaw(options.law);
    follower.setForceGain(options.forceGain);
    follower.setForceFilter(options.forceFilter);
    follower.setForceNotch(options.forceNotch);
    follower.setRealtime(options.app.rtPriority, options.app.rtCpu);
    follower.setAgeReference(options.app.ageReference);
    if (options.app.kiScale != 1.0) {  // --ki-scale: J1-J4's integral gain
        BARRETT_SCOPED_LOCK(pm.getExecutionManager()->getMutex());
        v_type ki = wam.jpController.getKi();
        for (size_t i = 0; i < 4 && i < DOF; ++i) {
            ki[i] *= options.app.kiScale;
        }
        wam.jpController.setKi(ki);
    }

    // 'c' (4C off/on) or 'p' (P-P on/off) here or in the other arm's program,
    // or `wam 4c|pp on|off`: the mode on both arms (lib/teleop_mode.h). The
    // message ends with this program's prompt, where `wam stop` looks for it,
    // unless the key was typed here (the loop prints it then).
    std::atomic<bool> ownMode(false);
    std::unique_ptr<TeleopModeSwitch> modeSwitch;
    if (options.app.ros) modeSwitch.reset(new TeleopModeSwitch([&follower, &options, &ownMode](teleop_law::Mode m) {
        if (m == follower.getMode()) {
            printf("\nMode already %s.\n", teleop_law::modeText(m, options.law).c_str());
        } else {
            printf("\nMode %s; the change fades in over %.0f s.\n", teleop_law::modeText(m, options.law).c_str(),
                   teleop_law::MODE_FADE_TIME);
        }
        if (!ownMode.exchange(false)) {
            printf(">>> ");
        }
        fflush(stdout);
        follower.setMode(m);  // after the message: the keys wait for this
    }));
    printf("Law: %s, force gain %s, ", teleop_law::name(options.law), teleop_law::forceGainText(options.forceGain).c_str());
    if (options.forceFilter > 0.0) {
        printf("force filter %.1f Hz", options.forceFilter);
    } else {
        printf("force filter off");
    }
    if (options.forceNotch > 0.0) {
        printf(", force notch %.1f Hz", options.forceNotch);
    } else {
        printf(", force notch off");
    }
    printf("%s\n", options.wam8g ? "; sync pose for the wam8g leader" : "");
    teleop_app::print(options.app);

    jt_type maxRate; // Nm · s-1 per joint
    maxRate << 50, 50, 50, 50;
    systems::RateLimiter<jt_type> wamJPOutputRamp(maxRate, "ffRamp");

    systems::PrintToStream<jt_type> printdynamicextTorque(pm.getExecutionManager(), "dynamicextTorque: ");
    systems::PrintToStream<jt_type> printSC(pm.getExecutionManager(), "SC: ");

    // systems::PrintToStream<jt_type> printcustomjtSum(pm.getExecutionManager(), "customjtSum: ");

    double h_omega_p = 25.0;
    barrett::systems::FirstOrderFilter<jv_type> hp1;
    hp1.setHighPass(jv_type(h_omega_p), jv_type(h_omega_p));
    systems::Gain<jv_type, double, ja_type> jaWAM(1.0);
    pm.getExecutionManager()->startManaging(hp1);

    barrett::systems::FirstOrderFilter<ja_type> jaFilter;
    ja_type l_omega_p = ja_type::Constant(50.0);
    jaFilter.setLowPass(l_omega_p);
    pm.getExecutionManager()->startManaging(jaFilter);


    systems::connect(wam.jvOutput, hp1.input);
    systems::connect(hp1.output, jaWAM.input);
    systems::connect(jaWAM.output, jaFilter.input);
    systems::connect(jaFilter.output, followerDynamics.jaInput);

    systems::connect(wam.jpOutput, follower.wamJPIn);
    systems::connect(wam.jvOutput, follower.wamJVIn);
    // systems::connect(extFilter.output, follower.extTorqueIn);
    systems::connect(dynamicExternalTorque.wamExternalTorqueOut, follower.extTorqueIn);
    systems::connect(wam.supervisoryController.output, follower.pidIn);  // for the recording

    systems::connect(wam.jpOutput, followerDynamics.jpInput);
    systems::connect(wam.jvOutput, followerDynamics.jvInput);

    systems::connect(follower.wamJPOutput, customjtSum.getInput(0));
    systems::connect(wam.gravity.output, customjtSum.getInput(1));
    systems::connect(wam.supervisoryController.output, customjtSum.getInput(2));

    systems::connect(customjtSum.output, dynamicExternalTorque.wamTorqueSumIn);
    systems::connect(followerDynamics.dynamicsFeedFWD, dynamicExternalTorque.wamDynamicsIn);

    systems::connect(wam.gravity.output, follower.wamGravIn);
    systems::connect(followerDynamics.dynamicsFeedFWD, follower.wamDynIn);

    systems::connect(dynamicExternalTorque.wamExternalTorqueOut, extFilter.input);

    // --model cable: the cable-drive model (lib/cable_drive_dynamics.h) in
    // place of FollowerDynamics, for the estimate and the feedforward.
    arm_gravity::Model armGravity;
    std::unique_ptr<CableDriveDynamics<DOF> > cable;
    if (options.cableModel) {
        if (options.wristGravity && !armGravity.load(pm.getConfig().lookup(pm.getWamDefaultConfigPath()), "follower")) {
            printf("follower: continuing without the wrist gravity correction.\n");
        }
        const bool useGravity = options.wristGravity && armGravity.getDof() == (int) DOF;
        cable.reset(new CableDriveDynamics<DOF>(pm.getExecutionManager(), cable_params_zeus_hand::params(),
                                                useGravity ? &armGravity : NULL, 4, options.ffInertia,
                                                options.ffFriction));
        systems::connect(wam.jpOutput, cable->jpInput);
        systems::connect(wam.jvOutput, cable->jvInput);
        systems::connect(jaFilter.output, cable->jaInput);
        systems::forceConnect(cable->modelOutput, dynamicExternalTorque.wamDynamicsIn);
        systems::forceConnect(cable->feedforwardOutput, follower.wamDynIn);
        printf("Model: cable (lib/cable_params_zeus_hand.h: ZEUS'S PARAMETERS, not this arm's); feedforward shares: friction %.2f, inertia %.2f;\n"
               "       wrist gravity correction %s.\n", options.ffFriction, options.ffInertia, useGravity ? "on" : "off");
    } else {
        printf("Model: legacy (lib/follower_beta_slax_skid_hand.h: slax, skid gripper, F/T sensor; Serg's fit of 2026-11-09).\n");
    }

    // --estimator matched: the commanded torque and the model's other terms
    // through the same filter as the model's acceleration, so the arm's own
    // inertial torque cancels at every frequency (lib/estimator_core.h). The
    // feedforward is not touched.
    const double emPeriod = pm.getExecutionManager()->getPeriod();
    MatchedAcceleration<DOF> matchedAcceleration(options.app.estimatorHp, options.app.estimatorLp, emPeriod);
    MatchedExternalTorque<DOF> matchedExternalTorque(options.app.estimatorHp, options.app.estimatorLp, emPeriod);
    if (options.app.matched) {
        systems::connect(wam.jvOutput, matchedAcceleration.jvInput);
        systems::connect(customjtSum.output, matchedExternalTorque.wamTorqueSumIn);
        if (cable) {
            systems::connect(matchedAcceleration.output, cable->jaEstimatorInput);
            systems::connect(cable->staticOutput, matchedExternalTorque.staticIn);
            systems::connect(cable->estimatorInertiaOutput, matchedExternalTorque.inertiaIn);
        } else {
            systems::connect(matchedAcceleration.output, followerDynamics.jaEstimatorInput);
            systems::connect(followerDynamics.staticOutput, matchedExternalTorque.staticIn);
            systems::connect(followerDynamics.estimatorInertiaOutput, matchedExternalTorque.inertiaIn);
        }
        systems::forceConnect(matchedExternalTorque.wamExternalTorqueOut, follower.extTorqueIn);
    }

    // --ft: the sensor in the loop (lib/ft_barrett.h: never waited for) and
    // its wrench as joint torques (lib/ft_systems.h), pulled by the follower
    // every cycle.
    std::unique_ptr<BarrettFtSource> ftSource;
    teleop_app::Lazy<FtTorques<DOF> > ftTorques;
    if (fts != NULL) {
        ft_core::Limits limits;
        limits.filterHz = options.ftFilter;
        ftSource.reset(new BarrettFtSource(fts));
        ftTorques.emplace(ftSource.get(), options.ftTool, limits, emPeriod);
        systems::connect(wam.kinematicsBase.kinOutput, ftTorques->kinInput);
        systems::connect(ftTorques->output, follower.ftTorqueIn);
        follower.setFt(&ftTorques->report(), options.ftArm, options.ftWrist);
        printf("Force/torque sensor: read every cycle%s%s%s.\n",
               options.ftWrist ? "; its wrist torques go to the leader (leader --wrist-force)" : "",
               options.ftArm ? "; J1-J4's external torque sent to the leader is the sensor's" : "",
               !options.ftWrist && !options.ftArm ? "; recorded only, the teleop is unchanged" : "");
        if (options.ftTool.calibrated) {
            printf("       gripper (%s): %.3f kg, centre of mass %.3f, %.3f, %.3f m, sign %+.0f; low-pass %.0f Hz.\n",
                   options.ftToolFile.c_str(), options.ftTool.mass, options.ftTool.com[0], options.ftTool.com[1],
                   options.ftTool.com[2], options.ftTool.sign, options.ftFilter);
        } else {
            printf("       no --ft-tool: the gripper's weight is not removed (the recording's raw readings\n"
                   "       are what scripts/ft_tool_cal.py needs).\n");
        }
        printf("       Its zero is taken at the link pose ('l'); 'z' takes it again.\n");
    }
    follower.setWristFriction(options.wristFriction);
    if (options.wristFriction[0] > 0.0 || options.wristFriction[1] > 0.0 || options.wristFriction[2] > 0.0) {
        printf("Wrist friction feedforward: %.2f, %.2f, %.2f Nm (J5-J7), in the direction the leader's wrist turns.\n",
               options.wristFriction[0], options.wristFriction[1], options.wristFriction[2]);
    }
    // The sensor's zero: the average of half a second with nothing touching
    // the gripper. False (and a message) if the sensor did not answer.
    auto zeroFt = [&]() -> bool {
        if (!ftTorques) {
            return true;
        }
        printf("Force/torque sensor: taking its zero; keep clear of the gripper ... ");
        fflush(stdout);
        btsleep(0.3);  // the arm settles
        const int before = ftTorques->zerosDone();
        ftTorques->startZero(0.5);
        for (int i = 0; i < 500 && ftTorques->zerosDone() == before; ++i) {  // 250 readings: 0.5 s at one a cycle
            btsleep(0.01);
        }
        if (ftTorques->zerosDone() == before) {
            printf("NOT DONE: the sensor is not answering (%lu readings so far, %lu timeouts).\n"
                   "The teleop goes on without it.\n", ftTorques->readingCount(), ftSource->getTimeouts());
            return false;
        }
        printf("done.\n");
        return true;
    };

    // --record: one row per control cycle, written by the loop itself
    // (lib/teleop_recorder.h); scripts/teleop_log.py reads it.
    std::unique_ptr<TeleopRecorder> recorder;
    if (!options.app.record.empty()) {
        const std::string file = teleop_app::recordFile(options.app.record, "follower");
        std::string note = "follower";
        for (int i = 1; i < argc; ++i) {
            note += std::string(" ") + argv[i];
        }
        try {
            recorder.reset(new TeleopRecorder(file, Follower<DOF>::recordColumns(), note));
            follower.setRecorder(recorder.get());
            printf("Recording to %s\n", file.c_str());
        } catch (const std::exception& e) {
            printf("NOT recording: %s\n", e.what());
        }
    }

    // What the loop no longer prints itself (lib/teleop_app.h).
    // Messages end with the prompt, where `wam stop` looks for it.
    bool recorderWarned = false;
    unsigned int lostSeen = 0, adoptedSeen = 0;
    unsigned long ftTimeoutsSeen = 0;
    int ftQuiet = 0;
    teleop_app::Watch watch([&]() {
        if (ftSource) {  // at most one message every 5 s
            const unsigned long timeouts = ftSource->getTimeouts() + ftSource->getBusErrors() + ftSource->getSendErrors();
            if (ftQuiet > 0) {
                --ftQuiet;
            } else if (timeouts != ftTimeoutsSeen) {
                printf("\nForce/torque sensor: %lu requests unanswered or failed so far; the teleop does without it"
                       " meanwhile.\n>>> ", timeouts);
                fflush(stdout);
                ftTimeoutsSeen = timeouts;
                ftQuiet = 50;
            }
        }
        const unsigned int lost = follower.getLinkLostEvents();
        if (lost != lostSeen) {
            lostSeen = lost;
            printf("\nlost link\n>>> ");
            fflush(stdout);
        }
        const unsigned int adopted = follower.getModeAdoptions();
        if (adopted != adoptedSeen) {
            adoptedSeen = adopted;
            printf("\nMode %s (from the leader); the change fades in over %.0f s.\n>>> ",
                   teleop_law::modeText(follower.getMode(), options.law).c_str(), teleop_law::MODE_FADE_TIME);
            fflush(stdout);
        }
        if (recorder && !recorderWarned && (recorder->writeFailed() || recorder->dropped() > 0)) {
            recorderWarned = true;
            printf("\nRECORDING PROBLEM: %s (%lu rows dropped so far). The teleop is not affected.\n>>> ",
                   recorder->writeFailed() ? "the file cannot be written (disk full?)" : "the disk is too slow",
                   recorder->dropped());
            fflush(stdout);
        }
    });

    // systems::connect(extFilter.output, printdynamicextTorque.input);
    // systems::connect(dynamicExternalTorque.wamExternalTorqueOut, printdynamicextTorque.input);
    // systems::connect(wam.supervisoryController.output, printSC.input);
    // systems::connect(extFilter.output, printcustomjtSum.input);

    wam.gravityCompensate();

#ifdef FOLLOWER_HAND
    // The hand moves only at the link pose ('l').
    barrett::Hand* hand = options.hand ? findHand(pm) : NULL;
    std::unique_ptr<HandTeleopThread> handThread;
#endif

    // From the first link: when this arm leaves ACTIVE, stop the teleop and the
    // hand (lib/safety_watch.h).
    std::atomic<bool> armInactive(false);
    std::unique_ptr<SafetyWatch> safetyWatch;

    std::string line;
    v_type gainTmp;

    bool going = true;

    // Ctrl-C, SIGTERM or the end of the input end the teleop as 'x' does
    // (lib/clean_exit.h).
    clean_exit::install();

    // What the control thread is now (lib/rt_thread.h), once it has run a cycle.
    for (int i = 0; i < 100 && !follower.getRealtimeReport().done; ++i) {
        btsleep(0.01);
    }
    rt_thread::print("Follower", follower.getRealtimeReport());

    while (going) {
        printf(">>> ");
        if (!clean_exit::readLine(&line)) {
            going = false;
            continue;
        }

        switch (line[0]) {
        case 'l':
            if (follower.isHalted()) {
                printf("The teleop has stopped (above); restart both programs to link again.\n");
            } else if (follower.isLinked()) {
                follower.unlink();
            } else {
                wam.moveTo(SYNC_POS, true);
#ifdef FOLLOWER_HAND
                if (hand != NULL && !handThread) {  // the first link
                    handThread = startHand(*hand, [&armInactive]() { return armInactive.load(); },
                                           options.spreadPoses);
                    if (!handThread) {
                        hand = NULL;
                    }
                }
#endif

                zeroFt();  // at the link pose, the hand as it will be
                printf("Press [Enter] to link with the other WAM.");
                waitForEnter();
                if (clean_exit::requested()) {  // a stop, not Enter: no link
                    going = false;
                    break;
                }
                follower.tryLink();
                wam.trackReferenceSignal(follower.theirJPOutput);
                systems::connect(follower.wamJPOutput, wam.input);
                // connect(follower.wamJPOutput, wamJPOutputRamp.input); // one of the problem with the joint limiter is that it adds delay in applying external torque to the robot.
                // connect(wamJPOutputRamp.output, wam.input);
                // systems::forceConnect(wam.jtSum.output, externalTorque.wamTorqueSumIn);

                btsleep(0.1); // wait an execution cycle or two
                if (follower.isLinked()) {
                    printf("Linked with remote WAM.\n");
                    if (!safetyWatch) {
                        safetyWatch.reset(new SafetyWatch(pm.getSafetyModule(), [&follower, &armInactive]() {
                            armInactive = true;
                            follower.halt();
                            printf("\nThe follower is not active (a fault, E-stop or idle): teleop stopped. It\n"
                                   "sends nothing more, so the leader holds. Restart both programs to link again.\n");
                            fflush(stdout);
                        }));
                    }
                } else {
                    printf("WARNING: Linking was unsuccessful.\n");
                }
            }

            break;

        case 't':
            size_t jointIndex;
            {
                size_t jointNumber;
                std::cout << "\tJoint: ";
                std::cin >> jointNumber;
                jointIndex = jointNumber - 1;

                if (jointIndex >= DOF) {
                    std::cout << "\tBad joint number: " << jointNumber;
                    break;
                }
            }

            char gainId;
            std::cout << "\tGain identifier (p, i, or d): ";
            std::cin >> line;
            gainId = line[0];

            std::cout << "\tCurrent value: ";
            switch (gainId) {
            case 'p':
                gainTmp = wam.jpController.getKp();
                break;
            case 'i':
                gainTmp = wam.jpController.getKi();
                break;
            case 'd':
                gainTmp = wam.jpController.getKd();
                break;

            default:
                std::cout << "\tBad gain identifier.";
            }
            std::cout << gainTmp[jointIndex] << std::endl;

            std::cout << "\tNew value: ";
            std::cin >> gainTmp[jointIndex];
            switch (gainId) {
            case 'p':
                wam.jpController.setKp(gainTmp);
                break;
            case 'i':
                wam.jpController.setKi(gainTmp);
                break;
            case 'd':
                wam.jpController.setKd(gainTmp);
                break;

            default:
                std::cout << "\tBad gain identifier.";
            }

            break;
        case 'c':
        case 'p': {
            teleop_law::Mode target = line[0] == 'c'
                    ? (follower.getMode() == teleop_law::FULL ? teleop_law::NO_4C : teleop_law::FULL)
                    : (follower.getMode() == teleop_law::PP ? teleop_law::FULL : teleop_law::PP);
            if (modeSwitch) {
                ownMode = true;
                modeSwitch->publish(target);
                for (int i = 0; i < 50 && follower.getMode() != target; ++i) {
                    btsleep(0.01);  // its message first, then the prompt
                }
            } else {  // no ROS: the leader takes it from this arm's packets
                follower.setMode(target);
                printf("Mode %s; the change fades in over %.0f s.\n", teleop_law::modeText(target, options.law).c_str(),
                       teleop_law::MODE_FADE_TIME);
            }
            } break;

        case 'z':
            if (!ftTorques) {
                printf("No force/torque sensor (--ft).\n");
            } else {  // linked or not: its torques fade out and back in around it
                zeroFt();
            }
            break;

        case 'm': {
            if (!cable) {
                std::cout << "\tThe feedforward shares need --model cable." << std::endl;
                break;
            }
            std::cout << "\tFeedforward shares now: friction " << cable->getFeedforwardFriction() << ", inertia "
                      << cable->getFeedforwardInertia() << "; new friction and inertia shares (0 to 1): ";
            double fs = -1.0, is = -1.0;
            std::cin >> fs >> is;
            if (!std::cin || fs < 0.0 || fs > 1.0 || is < 0.0 || is > 1.0) {
                std::cin.clear();
                std::cout << "\tKept them." << std::endl;
            } else {
                cable->setFeedforward(is, fs);
            }
            std::getline(std::cin, line);  // the rest of the line
            } break;

        case 'x':
            going = false;
            break;

        default:
            printf("\n");
            printf("    'l' to toggle linking with other WAM\n");
            printf("    'c' 4C off/on, 'p' P-P on/off: on both arms, fading over 1 s\n");
            printf("    't' to tune control gains\n");
            printf("    'z' to zero the force/torque sensor (nothing touching the gripper)\n");
            printf("    'm' to set the cable model's feedforward shares (friction, inertia)\n");
            printf("    'x' to exit\n");

            break;
        }
    }

    // The teleop ends here: nothing more is sent (the leader holds) and the arm
    // holds where it is.
    if (safetyWatch) {
        safetyWatch->stop();
    }
    follower.halt();
    wam.moveTo(wam.getJointPositions());

    // The session's link and recording, for the pane's log.
    watch.stop();
    follower.setFt(NULL, false, false);
    const typename teleop_link::Link<DOF>::Stats link = follower.getLinkStats();
    printf("Link: %llu packets sent (%llu refused), %llu received, %llu lost on the way, %llu arrived two in one "
           "cycle, %llu rejected; the link dropped %u time(s).\n",
           (unsigned long long) link.sent, (unsigned long long) link.sendErrors, (unsigned long long) link.received,
           (unsigned long long) link.lost, (unsigned long long) link.skipped, (unsigned long long) link.rejected,
           follower.getLinkLostEvents());
    if (ftTorques) {
        printf("Force/torque sensor: %lu readings, %lu requests unanswered, %lu bus errors; reading it took the loop "
               "at most %.0f us in a cycle.\n", ftTorques->readingCount(), ftSource->getTimeouts(),
               ftSource->getBusErrors() + ftSource->getSendErrors(), 1e6 * ftTorques->worstPollSeconds());
    }
    if (recorder) {
        follower.setRecorder(NULL);
        btsleep(0.01);  // the cycle in progress finishes its row
        recorder->close();
        printf("Recording: %lu rows in %s (%lu dropped%s).\n", recorder->written(), recorder->getPath().c_str(),
               recorder->dropped(), recorder->writeFailed() ? "; THE FILE COULD NOT BE WRITTEN" : "");
    }

#ifdef FOLLOWER_HAND
    handThread.reset();  // stops the hand, closes it to a fist, idles it
#endif

    clean_exit::waitUntilNotActive(pm.getSafetyModule());

    return 0;
}