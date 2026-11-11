#pragma once

/* follower --hand: HandTeleop (hand_teleop.h) fed from ROS. Subscribes to
 * follower/hand/velocity_setpoint (gripper_ros_common/VelocitySetpoint, e.g.
 * bhand_mux's output; launch/hand_nav_controller.launch; its grasp is used,
 * its spread is not) and follower/hand/next_pose (std_msgs/Empty: the spread
 * one pose on, 0% -> 50% -> 100%), and runs the hand at 50 Hz in its own
 * thread, outside the realtime loop. It starts from a fist (startHandTeleop)
 * and closes to one when it ends, unless armStopped() (the arm left ACTIVE:
 * the follower's SafetyWatch) stopped it: then the fingers stop where they
 * are and it sends nothing more. */

#include <atomic>
#include <chrono>
#include <cstdio>
#include <functional>
#include <memory>
#include <stdexcept>
#include <thread>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <gripper_ros_common/VelocitySetpoint.h>
#include <std_msgs/Empty.h>

#include "hand_teleop.h"

class HandTeleopThread {
  public:
    HandTeleopThread(barrett::Hand& hand, const HandTeleop::Params& params,
                     const std::function<bool()>& armStopped = std::function<bool()>())
        : hand(hand), teleop(hand, params), timeout(params.timeout), armStopped(armStopped), nh("follower"),
          stopping(false), failed(false) {
        nh.setCallbackQueue(&queue);
        sub = nh.subscribe("hand/velocity_setpoint", 1, &HandTeleopThread::onSetpoint, this);
        poseSub = nh.subscribe("hand/next_pose", 1, &HandTeleopThread::onNextPose, this);
        if (teleop.spreadHoldWasOff()) {
            printf("Hand: the spread's HOLD was 0; set to 1 so that it holds its poses.\n");
        }
        printf("Hand: %s.\n", teleop.setupReport().c_str());
        thread = std::thread(&HandTeleopThread::run, this);
    }
    // Stops the hand, brings the spread back to 0% (as a switch does), closes
    // it to a fist and idles it.
    ~HandTeleopThread() {
        stopping = true;
        thread.join();
        if (failed) {  // or stopped with the arm
            return;
        }
        try {
            teleop.stop();
            if (teleop.getPose() != 0) {
                printf("Hand: the spread back to 0%% first.\n");
                if (!teleop.returnToStart()) {
                    printf("Hand: %s\n", teleop.note().c_str());
                }
                teleop.stop();
            }
            printf("Hand: closing to a fist.\n");
            if (!makeFist(hand)) {
                printf("Hand: the fist did not finish within 10 s.\n");
            }
            hand.idle();
        } catch (const std::exception& e) {
            printf("Hand: %s\n", e.what());
        }
    }

  private:
    void onSetpoint(const gripper_ros_common::VelocitySetpoint::ConstPtr& msg) {
        teleop.setCommand(msg->grasp);  // the spread goes by poses (onNextPose)
    }
    void onNextPose(const std_msgs::Empty::ConstPtr&) {
        int pose = 0;
        switch (teleop.nextPose(HandTeleop::Clock::now(), &pose)) {
        case HandTeleop::ACCEPTED:
            printf("Hand: L2 + X received: next, %s.\n", HandTeleop::poseName(pose));
            break;
        case HandTeleop::BUSY:
            printf("Hand: L2 + X received during a switch: ignored (one press, one step).\n");
            break;
        case HandTeleop::OFF:
            printf("Hand: L2 + X received, but the spread poses are off (see the Hand: line at the start).\n");
            break;
        default:  // the same press, still held
            return;
        }
        fflush(stdout);
    }

    void run() {
        const auto period = std::chrono::milliseconds(20);
        auto next = std::chrono::steady_clock::now();
        while (!stopping) {
            next += period;
            queue.callAvailable();
            if (armStopped && armStopped()) {
                failed = true;
                try {
                    teleop.stop();
                } catch (const std::exception&) {
                }
                printf("Hand: stopped with the arm.\n");
                return;
            }
            try {
                switch (teleop.update()) {
                case HandTeleop::TIMED_OUT:
                    printf("Hand: no command for %.2f s, stopped.\n", timeout);
                    break;
                case HandTeleop::SWITCH_STARTED:
                case HandTeleop::SPREAD_STARTED:
                case HandTeleop::SWITCH_DONE:
                case HandTeleop::SWITCH_FAILED:
                    printf("Hand: %s\n", teleop.note().c_str());
                    fflush(stdout);
                    break;
                default:
                    break;
                }
            } catch (const std::exception& e) {
                // A failed send throws (Puck::setProperty), and a hand puck
                // that stops answering in a pose switch; this thread must not
                // take the follower with it.
                failed = true;
                printf("Hand: %s\nHand: control stopped; restart the follower to use the hand.\n", e.what());
                try {
                    teleop.stop();  // what is still moving stops where it is
                } catch (const std::exception&) {
                }
                return;
            }
            const auto now = std::chrono::steady_clock::now();
            if (now < next) {
                std::this_thread::sleep_until(next);
            } else {
                next = now;
            }
        }
    }

    barrett::Hand& hand;
    HandTeleop teleop;
    const double timeout;
    const std::function<bool()> armStopped;
    ros::NodeHandle nh;
    ros::CallbackQueue queue;
    ros::Subscriber sub, poseSub;
    std::atomic<bool> stopping, failed;
    std::thread thread;
};

// Where the hand has room (the follower: at the link pose): HI unless hi is
// false (the hand was initialized since it was powered on; HI opens the
// fingers fully and swings the spread through its range), then the fist, then
// the command thread. Throws if the fist does not finish.
inline std::unique_ptr<HandTeleopThread> startHandTeleop(barrett::Hand& hand, bool hi,
        const std::function<bool()>& armStopped = std::function<bool()>(),
        HandTeleop::Params params = HandTeleop::Params()) {
    if (hi) {
        hand.initialize();
    }
    if (!makeFist(hand)) {
        throw std::runtime_error("the fist did not finish within 10 s");
    }
    params.afterFist = true;  // checks that the fingers read closed
    return std::unique_ptr<HandTeleopThread>(new HandTeleopThread(hand, params, armStopped));
}
