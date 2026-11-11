#pragma once

#include <haptic_wrist/haptic_wrist.h>
#include <boost/asio.hpp>

#include "udp_handler.h"
#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/single_io.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/units.h>

template <size_t DOF>
class Leader : public barrett::systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    // Inputs (same as your first file)
    Input<jp_type> wamJPIn;
    Input<jv_type> wamJVIn;
    Input<jt_type> extTorqueIn;   // may be undefined
    Input<jt_type> wamGravIn;
    Input<jt_type> wamDynIn;

    // Outputs (same as your first file)
    Output<jt_type> wamJPOutput;      // control torque command for the WAM arm (DOF)
    Output<jp_type> theirJPOutput;    // peer arm JP (DOF) for logging/monitoring

    enum class State { INIT, LINKED, UNLINKED };

    explicit Leader(barrett::systems::ExecutionManager* em,
                    haptic_wrist::HapticWrist* hw,                 // NEW: wrist device
                    const std::string& remoteHost,
                    int rec_port = 5555,
                    int send_port = 5554,
                    const std::string& sysName = "Leader")
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
        , wamJPOutput(this, &jtOutputValue)
        , theirJPOutput(this, &theirJPOutputValue)
        , udp_handler(remoteHost, send_port, rec_port)
        , hw(hw)
        , state(State::INIT) {

        // Keep your original gains
        kp << 750, 1000, 400, 200;
        kd << 8.3, 8, 3.3, 0.8;
        cf << 0.375, 0.4, 0.2, 0.1;

        if (em != NULL) {
            em->startManaging(*this);
        }
    }

    virtual ~Leader() { this->mandatoryCleanUp(); }

    virtual bool inputsValid() { return true; }

    bool isLinked() const { return state == State::LINKED; }
    void tryLink() { BARRETT_SCOPED_LOCK(this->getEmMutex()); state = State::LINKED; }
    void unlink() { BARRETT_SCOPED_LOCK(this->getEmMutex()); state = State::UNLINKED; }

  protected:
    typename Output<jt_type>::Value* jtOutputValue;
    typename Output<jp_type>::Value* theirJPOutputValue;

    // Local copies
    jp_type wamJP;
    jv_type wamJV;
    jt_type extTorque;
    jt_type wamGrav;
    jt_type wamDyn;

    // Wrist state
    haptic_wrist::jp_type wristJP;      // local wrist pos
    haptic_wrist::jv_type wristJV;      // local wrist vel
    haptic_wrist::jp_type theirWristJp; // received wrist pos

    // Network payloads: arm (DOF) + wrist (3) = DOF+3
    Eigen::Matrix<double, DOF + 3, 1> sendJpMsg;
    Eigen::Matrix<double, DOF + 3, 1> sendJvMsg;
    Eigen::Matrix<double, DOF + 3, 1> sendExtTorqueMsg;

    using ReceivedData = typename UDPHandler<DOF + 3>::ReceivedData;

    virtual void operate() {
        // Optional scaling (like your second file)
        double j5_scale = 1.0;
        double j7_scale = 1.0;

        // Read WAM inputs
        wamJP  = wamJPIn.getValue();
        wamJV  = wamJVIn.getValue();
        wamGrav = wamGravIn.getValue();
        wamDyn  = wamDynIn.getValue();

        if (extTorqueIn.valueDefined()) {
            extTorque = extTorqueIn.getValue();
        } else {
            extTorque.setZero();
        }

        // Read wrist device (if available)
        haptic_wrist::jp_type wristJP = hw->getPosition();
        haptic_wrist::jp_type wristJV = hw->getVelocity();

        // Pack outgoing messages (arm + wrist)
        sendJpMsg << wamJP, wristJP;
        sendJvMsg << wamJV, wristJV;

        // Only arm external torque is meaningful here; pad wrist torques with zeros
        sendExtTorqueMsg << extTorque, 0.0, 0.0, 0.0;

        // Example scaling of J5 and J7 (index 4 and 6 in the concatenated vector)
        sendJpMsg(4) = j5_scale * sendJpMsg(4);
        sendJpMsg(6) = j7_scale * sendJpMsg(6);

        // Send
        udp_handler.send(sendJpMsg, sendJvMsg, sendExtTorqueMsg);

        // Receive (non-blocking)
        boost::optional<ReceivedData> received_data = udp_handler.getLatestReceived();
        auto now = std::chrono::steady_clock::now();
        if (received_data && (now - received_data->timestamp <= TIMEOUT_DURATION)) {
            // Split received arm & wrist
            theirJp        = received_data->jp.template head<DOF>();
            theirWristJp   = received_data->jp.template tail<3>();
            theirJv        = received_data->jv.template head<DOF>();
            theirExtTorque = received_data->extTorque.template head<DOF>();

            // Undo scaling on wrist channels
            theirWristJp(0) = theirWristJp(0) / j5_scale; // J5
            theirWristJp(2) = theirWristJp(2) / j7_scale; // J7

            // Publish peer arm JP (as before)
            theirJPOutputValue->setData(&theirJp);
        } else {
            if (state == State::LINKED) {
                std::cout << "lost link" << std::endl;
                state = State::UNLINKED;
            }
        }

        // State machine
        switch (state) {
            case State::INIT:
                // Hold wrist (don’t move) and zero arm torque
                hw->setTarget(wristJP);
                control.setZero();
                jtOutputValue->setData(&control);
                break;

            case State::LINKED:
                // Drive wrist to peer wrist pose
                hw->setTarget(theirWristJp);

                // Your arm control law (kept intact)
                control = compute_control(
                    theirJp, theirJv, theirExtTorque,
                    wamJP,   wamJV,   extTorque,
                    wamGrav, wamDyn
                );
                jtOutputValue->setData(&control);
                break;

            case State::UNLINKED:
                // Release to local wrist pose and zero arm torque
                hw->setTarget(wristJP);
                control.setZero();
                jtOutputValue->setData(&control);
                break;
        }
    }

    // Peer (arm) state & internal
    jp_type theirJp;
    jv_type theirJv;
    jt_type theirExtTorque;
    jt_type control;

  private:
    DISALLOW_COPY_AND_ASSIGN(Leader);

    haptic_wrist::HapticWrist* hw;     // NEW
    std::mutex state_mutex;
    jp_type joint_positions;
    UDPHandler<DOF + 3> udp_handler;
    const std::chrono::milliseconds TIMEOUT_DURATION = std::chrono::milliseconds(20);
    State state;

    // Gains you had
    Eigen::Matrix<double, DOF, 1> kp;
    Eigen::Matrix<double, DOF, 1> kd;
    Eigen::Matrix<double, DOF, 1> cf;

    // === Your original control choices preserved ===
    jt_type compute_control(const jp_type& ref_pos, const jv_type& ref_vel, const jt_type& ref_extTorque,
                            const jp_type& cur_pos, const jv_type& cur_vel, const jt_type& cur_extTorque,
                            const jt_type& cur_grav, const jt_type& cur_dyn) {

        jt_type u1 = 0.0 * cur_extTorque;                        // zero FF (P-P + g-comp only if you add it)
        jt_type u2 = cur_dyn - cur_grav;                          // P-P with dynamic comp (your comment)
        jt_type u3 = -0.5 * ref_extTorque;                        // PF-PF (ref ext torque FF)
        jt_type u4 = -0.5 * ref_extTorque + cur_dyn - cur_grav;   // PF-PF + dyn comp (Lawrence ideal)
        jt_type u5 = -0.5 * ref_extTorque - 0.15 * (ref_extTorque + cur_extTorque);
        jt_type u6 = -0.5 * ref_extTorque - 0.15 * (ref_extTorque + cur_extTorque) + cur_dyn - cur_grav;

        // Follower-different cases (kept for completeness)
        jt_type u7 = -0.5 * cur_extTorque;
        jt_type u8 = -0.25 * (ref_extTorque + cur_extTorque);

        // Default: u4 as you had
        return u1;
    };
};
