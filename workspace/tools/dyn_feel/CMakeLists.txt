# wam8g_dyn_feel and its two offline tests, built outside the catkin workspace
# against the CANopen library the leader uses (as tools/lbc_tests is), with
# wam_teleop's src/lib headers:
#
#   cmake -S . -B build -DWAM_TELEOP=/path/to/wam_ws/src/wam_teleop
#   cmake --build build
#   build/dyn_feel_core_tests test/data/beta_star_correct_slax_20261109.dat \
#       test/data/notebook_rows_slax_20261109.txt <WAM_TELEOP>/src/lib/cable_params_wam8g.h
#   build/dyn_feel_system_tests test/data/beta_star_correct_slax_20261109.dat
#
# To build it inside the package instead, see package-CMakeLists-snippet.cmake.
cmake_minimum_required(VERSION 3.5)
project(wam8g_dyn_feel CXX)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(BARRETT_PREFIX /opt/barrett-canopen CACHE PATH "libbarrett-canopen's install prefix")
set(WAM_TELEOP "" CACHE PATH "the wam_teleop package (for its src/lib headers)")
if(NOT EXISTS "${WAM_TELEOP}/src/lib/cable_drive_model.h")
  message(FATAL_ERROR "Set -DWAM_TELEOP=<...>/wam_ws/src/wam_teleop")
endif()
if(NOT EXISTS "${BARRETT_PREFIX}/share/barrett/barrett-config.cmake")
  message(FATAL_ERROR "No libbarrett in ${BARRETT_PREFIX} (-DBARRETT_PREFIX=...)")
endif()

include(${BARRETT_PREFIX}/share/barrett/barrett-config.cmake)
list(REMOVE_ITEM BARRETT_LIBRARIES barrett)
find_package(Curses REQUIRED)
find_package(Eigen3 REQUIRED)

set(FEEL_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/src/lib
                  ${WAM_TELEOP}/src ${WAM_TELEOP}/src/lib)

function(link_canopen target)
  target_include_directories(${target} BEFORE PRIVATE ${BARRETT_INCLUDE_DIRS} ${FEEL_INCLUDES})
  target_compile_options(${target} PRIVATE ${BARRETT_DEFINITIONS})
  target_link_libraries(${target} ${BARRETT_PREFIX}/lib/libbarrett.so ${BARRETT_LIBRARIES})
  set_target_properties(${target} PROPERTIES BUILD_WITH_INSTALL_RPATH TRUE
                        INSTALL_RPATH "${BARRETT_PREFIX}/lib;/usr/local/lib")
endfunction()

add_executable(wam8g_dyn_feel src/wam8g_dyn_feel.cpp)
link_canopen(wam8g_dyn_feel)
target_include_directories(wam8g_dyn_feel PRIVATE ${CURSES_INCLUDE_DIR})
target_link_libraries(wam8g_dyn_feel ${CURSES_LIBRARIES})

# No libbarrett: the parameters, the model's terms, the mixer.
add_executable(dyn_feel_core_tests test/test_dyn_feel_core.cpp)
target_include_directories(dyn_feel_core_tests PRIVATE ${EIGEN3_INCLUDE_DIR} ${FEEL_INCLUDES})

# The system in a libbarrett control graph, no robot.
add_executable(dyn_feel_system_tests test/test_dyn_feel_system.cpp)
link_canopen(dyn_feel_system_tests)
