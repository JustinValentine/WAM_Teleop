## To build wam8g_dyn_feel inside wam_teleop: copy
##   src/wam8g_dyn_feel.cpp             -> wam_teleop/src/
##   src/lib/dyn_feel_core.h, dyn_feel_system.h -> wam_teleop/src/lib/
##   test/test_dyn_feel_core.cpp, test_dyn_feel_system.cpp -> wam_teleop/test/
## and add these lines to wam_teleop/CMakeLists.txt, after its find_package(Eigen3 REQUIRED).

## Feel an identified J1-J4 model on the wam8g by hand: ex04's display with
## the model's terms, switched and scaled from the keyboard (parameters from a
## file: the Feas-Ident notebook's 30 numbers or a cable_params_*.h).
find_package(Curses REQUIRED)
add_executable(wam8g_dyn_feel src/wam8g_dyn_feel.cpp)
link_arm_library(wam8g_dyn_feel "${BARRETT_LEADER_PREFIX}" WAM8G)
target_include_directories(wam8g_dyn_feel PRIVATE ${CURSES_INCLUDE_DIR})
target_link_libraries(wam8g_dyn_feel ${CURSES_LIBRARIES})

## lib/dyn_feel_core.h without libbarrett:
##   dyn_feel_core_tests <30 numbers.dat> <notebook rows.txt> src/lib/cable_params_wam8g.h
add_executable(dyn_feel_core_tests test/test_dyn_feel_core.cpp)
target_include_directories(dyn_feel_core_tests PRIVATE ${EIGEN3_INCLUDE_DIR} src/lib)

## lib/dyn_feel_system.h in a libbarrett control graph, no robot:
##   dyn_feel_system_tests <30 numbers.dat>
add_executable(dyn_feel_system_tests test/test_dyn_feel_system.cpp)
link_arm_library(dyn_feel_system_tests "${BARRETT_LEADER_PREFIX}" WAM8G)
target_include_directories(dyn_feel_system_tests PRIVATE src/lib)
