cmake_minimum_required(VERSION 3.5)
project(wam_teleop)

option(BUILD_LEADER "Build leader executable" ON)

# By default, build programs with compiler optimizations and debugging symbols
if (NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

find_package(catkin REQUIRED COMPONENTS
   roscpp
   std_msgs
   sensor_msgs
   message_generation
)

add_message_files(
   FILES
   GravityTorque.msg
)

generate_messages( DEPENDENCIES std_msgs)

catkin_package(
   # INCLUDE_DIRS include
   # LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp std_msgs sensor_msgs message_runtime
)


## Each arm has its own protocol implementation, headers, and runtime library.
set(BARRETT_LEADER_PREFIX /opt/barrett-canopen CACHE PATH "CANopen leader library")
set(BARRETT_FOLLOWER_PREFIX /opt/barrett CACHE PATH "Legacy P2 follower library")
include_directories(${catkin_INCLUDE_DIRS})

# link_arm_library(target prefix [WAM8G]): WAM8G for the programs written for
# the eight-joint wam8g; the others refuse it before drive wake.
function(link_arm_library target prefix)
   if(NOT EXISTS "${prefix}/share/barrett/barrett-config.cmake" OR
      NOT EXISTS "${prefix}/lib/libbarrett.so")
      message(FATAL_ERROR "Missing libbarrett installation for ${target}: ${prefix}")
   endif()
   include("${prefix}/share/barrett/barrett-config.cmake")
   target_include_directories(${target} BEFORE PRIVATE ${BARRETT_INCLUDE_DIRS})
   target_compile_options(${target} PRIVATE ${BARRETT_DEFINITIONS})
   if("${prefix}" STREQUAL "${BARRETT_LEADER_PREFIX}" AND NOT "WAM8G" IN_LIST ARGN)
      # These applications have no Wam8g mapping; reject it before drive wake.
      target_compile_definitions(${target} PRIVATE BARRETT_SMF_NO_WAM8G)
   endif()
   list(REMOVE_ITEM BARRETT_LIBRARIES barrett)
   target_link_libraries(${target} "${prefix}/lib/libbarrett.so"
      ${BARRETT_LIBRARIES} ${catkin_LIBRARIES} Boost::system Boost::thread)
   set_target_properties(${target} PROPERTIES
      BUILD_WITH_INSTALL_RPATH TRUE
      INSTALL_RPATH "${prefix}/lib;/usr/local/lib"
      INSTALL_RPATH_USE_LINK_PATH TRUE)
   message(STATUS "${target}: libbarrett from ${prefix}")
endfunction()

find_package(Boost REQUIRED COMPONENTS system thread)

if (BUILD_LEADER) 
   find_package(haptic_wrist REQUIRED)
   add_executable(leader src/leader.cpp src/lib/udp_handler.cpp)
   link_arm_library(leader "${BARRETT_LEADER_PREFIX}")
   target_link_libraries(leader haptic_wrist)
   target_compile_definitions(leader PRIVATE BUILD_LEADER)
   add_dependencies(leader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endif()

add_executable(follower src/follower.cpp src/lib/udp_handler.cpp)
link_arm_library(follower "${BARRETT_FOLLOWER_PREFIX}")
add_dependencies(follower ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## follower --hand: the BarrettHand on the follower's bus, commanded over
## follower/hand/velocity_setpoint (gripper_ros_common/VelocitySetpoint, the
## grasp) and follower/hand/next_pose (std_msgs/Empty, the spread's poses;
## launch/hand_nav_controller.launch). Needs gripper_ros_common (libgripper)
## in the workspace.
find_package(gripper_ros_common QUIET)
if(gripper_ros_common_FOUND)
   target_include_directories(follower PRIVATE ${gripper_ros_common_INCLUDE_DIRS})
   target_compile_definitions(follower PRIVATE FOLLOWER_HAND)
   add_dependencies(follower ${gripper_ros_common_EXPORTED_TARGETS})
   message(STATUS "follower: --hand available")
else()
   message(STATUS "follower: no gripper_ros_common, built without --hand")
endif()

add_executable(follower_ext src/follower_ext.cpp src/lib/udp_handler.cpp)
link_arm_library(follower_ext "${BARRETT_FOLLOWER_PREFIX}")
add_dependencies(follower_ext ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(leader_nowrist src/leader_nowrist.cpp src/lib/udp_handler.cpp)
link_arm_library(leader_nowrist "${BARRETT_LEADER_PREFIX}")
add_dependencies(leader_nowrist ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(leader_nowrist_data src/leader_nowrist_data.cpp src/lib/udp_handler.cpp)
link_arm_library(leader_nowrist_data "${BARRETT_LEADER_PREFIX}")
add_dependencies(leader_nowrist_data ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(leader_nowrist_data_ext src/leader_nowrist_data_ext.cpp src/lib/udp_handler.cpp)
link_arm_library(leader_nowrist_data_ext "${BARRETT_LEADER_PREFIX}")
add_dependencies(leader_nowrist_data_ext ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## The eight-joint wam8g (CANopen) as the leader for a P2 follower
## (follower --sync wam8g).
add_executable(leader_wam8g src/leader_wam8g.cpp src/lib/udp_handler.cpp)
link_arm_library(leader_wam8g "${BARRETT_LEADER_PREFIX}" WAM8G)
target_compile_definitions(leader_wam8g PRIVATE LEADER_WAM8G)
add_dependencies(leader_wam8g ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Excitation trajectory for the dynamics fit (upstream 4c-dyntorq dc29e27
## names). The CANopen build also runs on the wam8g.
add_executable(dynamics_excitation_canopen src/dynamics_excitation.cpp)
link_arm_library(dynamics_excitation_canopen "${BARRETT_LEADER_PREFIX}" WAM8G)
target_compile_definitions(dynamics_excitation_canopen PRIVATE EXCITATION_CANOPEN)

## The wam8g's J6 friction, its motor torque commanded directly
## (leader_wam8g --j6-friction).
add_executable(wam8g_j6_friction src/wam8g_j6_friction.cpp)
link_arm_library(wam8g_j6_friction "${BARRETT_LEADER_PREFIX}" WAM8G)

add_executable(dynamics_excitation_legacy src/dynamics_excitation.cpp)
link_arm_library(dynamics_excitation_legacy "${BARRETT_FOLLOWER_PREFIX}")

## Offline checks (no robot): the wrist map and the laws; and the wam8g
## leader and P2 follower systems over UDP on this computer, and the wrist hold.
find_package(Eigen3 REQUIRED)
## The cable-drive model (lib/cable_drive_model.h) against the identification
## (ident/, test/data) and its friction elements' properties; no robot.
add_executable(cable_model_tests test/test_cable_drive_model.cpp)
target_include_directories(cable_model_tests PRIVATE ${EIGEN3_INCLUDE_DIR} src/lib)
target_compile_definitions(cable_model_tests PRIVATE TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")

## lib/arm_gravity.h against the identification's gravity, on each arm's
## configuration: arm_gravity_tests config/wam8g/wam8g.conf config/follower-zeus-hand/wam7w.conf
add_executable(arm_gravity_tests test/test_arm_gravity.cpp)
link_arm_library(arm_gravity_tests "${BARRETT_LEADER_PREFIX}" WAM8G)
target_include_directories(arm_gravity_tests PRIVATE src/lib)
target_compile_definitions(arm_gravity_tests PRIVATE TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")

## The low-latency pieces (2026-11-09) without libbarrett: the control loops'
## link, the recording, the estimator's arithmetic, the control thread's setup.
add_executable(low_latency_tests test/test_low_latency.cpp)
target_include_directories(low_latency_tests PRIVATE ${EIGEN3_INCLUDE_DIR} src/lib)
target_link_libraries(low_latency_tests pthread)

add_executable(wam8g_teleop_tests test/test_wam8g_teleop.cpp)
target_include_directories(wam8g_teleop_tests PRIVATE ${EIGEN3_INCLUDE_DIR} src/lib)

add_executable(wam8g_loopback_tests test/test_wam8g_loopback.cpp src/lib/udp_handler.cpp)
link_arm_library(wam8g_loopback_tests "${BARRETT_LEADER_PREFIX}" WAM8G)
target_include_directories(wam8g_loopback_tests PRIVATE src/lib)
# The same checks on the follower's library.
add_executable(wam8g_loopback_tests_legacy test/test_wam8g_loopback.cpp src/lib/udp_handler.cpp)
link_arm_library(wam8g_loopback_tests_legacy "${BARRETT_FOLLOWER_PREFIX}")
target_include_directories(wam8g_loopback_tests_legacy PRIVATE src/lib)

## lib/clean_exit.h: the arm programs' end on a signal or closed input.
add_executable(clean_exit_tests test/test_clean_exit.cpp)
target_include_directories(clean_exit_tests PRIVATE src/lib)
target_link_libraries(clean_exit_tests pthread)

## lib/teleop_mode.h (the mode on both arms) between two programs; start
## roscore, then e.g. `teleop_mode_tests listen 6 &` and `teleop_mode_tests send pp`.
add_executable(teleop_mode_tests test/test_teleop_mode.cpp)
target_include_directories(teleop_mode_tests PRIVATE src/lib)
target_link_libraries(teleop_mode_tests ${catkin_LIBRARIES})

## follower --hand without a robot: libbarrett's Hand and lib/hand_teleop.h on
## a simulated bus (WAM pucks 1-7, hand 11-14) next to the arm's 500 Hz bus
## traffic.
add_executable(hand_teleop_tests test/test_hand_teleop.cpp)
link_arm_library(hand_teleop_tests "${BARRETT_FOLLOWER_PREFIX}")
target_include_directories(hand_teleop_tests PRIVATE src/lib)

## lib/ft_barrett.h: the force/torque sensor read without waiting, on
## libbarrett's own bus code and the simulated bus, next to the arm's traffic.
add_executable(ft_bus_tests test/test_ft_bus.cpp)
link_arm_library(ft_bus_tests "${BARRETT_FOLLOWER_PREFIX}")
target_include_directories(ft_bus_tests PRIVATE src/lib)

## ... and end to end: start roscore and launch/hand_nav_controller.launch,
## then run hand_chain_tests (it plays the joystick).
if(gripper_ros_common_FOUND)
   add_executable(hand_chain_tests test/test_hand_chain.cpp)
   link_arm_library(hand_chain_tests "${BARRETT_FOLLOWER_PREFIX}")
   target_include_directories(hand_chain_tests PRIVATE src/lib ${gripper_ros_common_INCLUDE_DIRS})
   add_dependencies(hand_chain_tests ${gripper_ros_common_EXPORTED_TARGETS})
endif()

## The Robotiq 2F gripper from the navigation controller
## (launch/robotiq_nav_controller.launch). lib/robotiq_teleop_core.h decides
## what to ask of the gripper; its checks need no gripper, no library and no
## ROS, and are always built.
add_executable(robotiq_teleop_tests test/test_robotiq_teleop.cpp)
target_include_directories(robotiq_teleop_tests PRIVATE src/lib)

## robotiq_teleop, the node around the core, owns the gripper through
## libgripper's gripper::robotiq::RobotiqGripper, which only its branch
## "robotiq" has (github dmiller12/libgripper, 2ddb4c7). With an older
## libgripper installed, or none, the node is left out and everything else is
## built as before. The check is made again at every configure, so that a
## newer library is found without clearing the build folder.
find_package(gripper QUIET)
find_package(std_srvs QUIET)
unset(GRIPPER_HAS_ROBOTIQ CACHE)
if(gripper_FOUND AND std_srvs_FOUND)
   include(CheckCXXSourceCompiles)
   set(CMAKE_REQUIRED_LIBRARIES gripper::gripper)
   set(CMAKE_REQUIRED_QUIET TRUE)
   check_cxx_source_compiles("
      #include <gripper/robotiq/robotiq_gripper.h>
      int main() { gripper::robotiq::RobotiqGripper g; return g.isConnected() ? 1 : 0; }"
      GRIPPER_HAS_ROBOTIQ)
   unset(CMAKE_REQUIRED_LIBRARIES)
   unset(CMAKE_REQUIRED_QUIET)
endif()
if(GRIPPER_HAS_ROBOTIQ)
   add_executable(robotiq_teleop src/robotiq_teleop.cpp)
   target_include_directories(robotiq_teleop PRIVATE ${std_srvs_INCLUDE_DIRS})
   target_link_libraries(robotiq_teleop ${catkin_LIBRARIES} ${std_srvs_LIBRARIES} gripper::gripper pthread)
   add_dependencies(robotiq_teleop ${catkin_EXPORTED_TARGETS})
   message(STATUS "robotiq_teleop: built (libgripper with the Robotiq classes)")
elseif(NOT gripper_FOUND)
   message(STATUS "robotiq_teleop: not built (no libgripper installed)")
elseif(NOT std_srvs_FOUND)
   message(STATUS "robotiq_teleop: not built (no std_srvs)")
else()
   message(STATUS "robotiq_teleop: not built (the installed libgripper has no gripper::robotiq::RobotiqGripper: its branch robotiq has)")
endif()
