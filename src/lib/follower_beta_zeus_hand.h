//Estimated Pvector, gravity included.
// Zeus with its hand: excitation of 2026-09-24 on thor (dynamics_excitation_legacy),
// feasible-dynamics-estimation Feas-Ident-WAM4, beta_star_correct. As in the
// previous version, the Coulomb friction of J1, J3 and J4 is left out, and
// J2's too, as the four-channel procedure (4c.pdf) requires.
#ifndef FOLLOWER_BETA_VECTOR_H_
#define FOLLOWER_BETA_VECTOR_H_

#include <eigen3/Eigen/Dense>
Eigen::Matrix<double, 30, 1> initialize_follower_beta() {
    Eigen::Matrix<double, 30, 1> follower_beta;
    follower_beta << 1.949487774315506183e-01,
    2.000766044326910187e+00,
    0.0, // fc_1, fitted 2.518373e+00
    1.263613056984229654e+00,
    -2.162980620260262013e-02,
    -2.079462864870156857e-01,
    1.462184589932638890e+00,
    1.976312978917887098e-01,
    4.959607610513053905e-02,
    2.797973251173130116e+00,
    3.272497072785244043e+00,
    0.0, // fc_2, fitted 6.940221e-01
    1.261504397625685669e-01,
    3.498782320002646884e-02,
    1.123623057442157784e-02,
    1.233296879643334953e-01,
    1.142444701845852228e-01,
    1.012158390694821719e-01,
    -1.063031667049794959e-01,
    1.573344956869777711e+00,
    0.0, // fc_3, fitted 4.816240e-01
    1.895996355433972047e-01,
    5.675633031371610931e-02,
    2.203677671641179198e-02,
    3.051775700567183613e-01,
    7.201604103702778474e-03,
    -1.398057878828242451e-01,
    9.433712255024097049e-01,
    3.746914590701703011e-07,
    0.0; // fc_4, fitted 1.147566e+00

    return follower_beta;
}

#endif
