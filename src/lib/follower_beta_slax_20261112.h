// Estimated Pvector, gravity included (30, for regressor_W.h).
// The slax follower as it is on 2026-11-12: Barrett F/T sensor and the gripper mounted that day
// (1.183 kg beyond the sensor by its calibration), zero calibration of 2026-11-11 19:16, gravity
// calibration of 2026-11-12 07:37. Fitted on Justin's excitation runs of 2026-11-12
// (excitation/slax_20261112_1.dat, _2.dat, _3.dat; wrist held at zero) by the steps of
// feasible-dynamics-estimation Feas-Ident-WAM4 (beta_star_correct), as Serg's fit of 2026-11-09 in
// follower_beta_slax_skid_hand.h was. The Coulomb friction of J1-J4 is zero here, as in our other fits
// (fitted: 1.259, 0.677, 0.310, 0.693 Nm; follower_beta_slax_20261112_with_coulomb.h has them).
#ifndef FOLLOWER_BETA_VECTOR_H_
#define FOLLOWER_BETA_VECTOR_H_

#include <eigen3/Eigen/Dense>
Eigen::Matrix<double, 30, 1> initialize_follower_beta() {
    Eigen::Matrix<double, 30, 1> follower_beta;
    follower_beta << 2.271987791298703818e-01,
    6.924114960277371056e-01,
    0.0,
    1.170496389240955670e+00,
    -2.075314405625969250e-02,
    -1.178824250917368116e-01,
    1.206168420061516589e+00,
    1.762451315703721377e-01,
    7.544776111731070576e-04,
    2.467520824656351142e+00,
    1.598599691309538828e+00,
    0.0,
    1.186133412720664637e-01,
    -8.726185981458911556e-03,
    2.604722119386788937e-03,
    3.098127045869719653e-02,
    5.069850733096243484e-02,
    1.308210058891745575e-01,
    -4.150688887735435456e-02,
    9.584302861712080679e-01,
    0.0,
    1.923941841985074286e-01,
    4.620566931897480967e-02,
    7.525482630728558796e-03,
    2.589893014303173979e-01,
    1.719135502855589220e-02,
    -1.231497307869630631e-01,
    8.217633370930135239e-01,
    1.637470949628630490e-01,
    0.0;

    return follower_beta;
}

#endif
