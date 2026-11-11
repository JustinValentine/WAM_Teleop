// Dynamics parameters of the wam8g's J1-J4 (30, gravity included, for
// regressor_W.h), with the wrist held at its excitation start angles (0).
//
// WAM 153 base with the Alberta wrist, on odin. Fitted from the excitation runs
// of 2026-11-11 (wam153_20261111_2/_3/_4.dat: dynamics_excitation_canopen, J4
// factor 0.75, velocity gains 21/18/9/9, after that night's zerocal and
// gravitycal with the corrected wrist model), by the Feas-Ident-WAM4 notebook's
// steps run as a script (fit-wam153-20261111/scripts/nb/notebook_route.py;
// beta30_wam153_runs234_coulomb_zeroed.dat). The Coulomb friction of J1-J4 is
// left out, as before (the four-channel procedure, and the velocity reading
// that is never exactly zero at rest). On the held-out run this form, without
// acceleration as the leader uses it, leaves 1.87 1.99 0.93 0.68 Nm rms on
// J1-J4; the previous values (the IROS base, 2026-09-25) left 2.06 2.36 0.93
// 0.83 on this arm. Before: leader_beta_wam8g.h.bak-20261111-iros-base.
#ifndef LEADER_BETA_VECTOR_H_
#define LEADER_BETA_VECTOR_H_

#define LEADER_BETA_WAM8G_FITTED 1

#include <eigen3/Eigen/Dense>
Eigen::Matrix<double, 30, 1> initialize_leader_beta() {
    Eigen::Matrix<double, 30, 1> leader_beta;
    leader_beta << 2.534154581728408351e-01,
    1.030621513435291625e+00,
    0.0, // fc_1, fitted 1.637163e+00
    1.188550737183274641e+00,
    7.865432614045908544e-02,
    -2.360886511687958905e-02,
    9.885789930535207048e-01,
    1.041572973555899423e-01,
    -1.158335273899324842e-02,
    2.391831552016203233e+00,
    2.028260232776163186e+00,
    0.0, // fc_2, fitted 7.330339e-01
    6.006382763007422909e-02,
    -2.416936647205147892e-02,
    7.325489079193679698e-02,
    4.391594782932115720e-02,
    4.324744103757754282e-02,
    1.262765147203026572e-01,
    1.543064981156969107e-02,
    1.050402820857772035e+00,
    0.0, // fc_3, fitted 3.897820e-01
    2.742824180972939230e-01,
    5.876470875515003162e-02,
    1.729273342256866172e-02,
    3.473443348391581087e-01,
    -3.799389574182869544e-02,
    -1.265837184647019276e-01,
    8.785108057252034586e-01,
    3.824370129123940143e-01,
    0.0; // fc_4, fitted 7.096424e-01

    return leader_beta;
}

#endif
