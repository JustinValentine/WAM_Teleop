// Dynamics parameters of the wam8g's J1-J4 (30, gravity included, for
// regressor_W.h), with the wrist held at its excitation start angles (0).
//
// Fitted from the excitation of 2026-09-25 on thor, after the cables were
// re-tensioned and zerocal and gravitycal redone (dynamics_excitation_canopen,
// J4 factor 0.7): feasible-dynamics-estimation Feas-Ident-WAM4,
// solutions_wam8g_20260925/beta_star_correct. The Coulomb friction of J1-J4 is
// left out, as the four-channel procedure (4c.pdf) requires: the wam8g's
// velocity reading is never exactly zero at rest, so fc * tanh(100 dq) kept
// flipping and the leader shook when linked. The previous fit (2026-09-24,
// run 2) predicted today's data a little worse (J2 1.39, J4 1.29 Nm RMS
// against 1.15 and 1.08, Coulomb friction off in both).
#ifndef LEADER_BETA_VECTOR_H_
#define LEADER_BETA_VECTOR_H_

#define LEADER_BETA_WAM8G_FITTED 1

#include <eigen3/Eigen/Dense>
Eigen::Matrix<double, 30, 1> initialize_leader_beta() {
    Eigen::Matrix<double, 30, 1> leader_beta;
    leader_beta << 1.012390150855393967e-01,
    7.340656575214313806e-01,
    0.0, // fc_1, fitted 1.779646e+00
    9.632200541804165628e-01,
    -7.183926387877942465e-02,
    8.146251650617007678e-02,
    1.077635535193004035e+00,
    -8.927886808627412407e-02,
    4.059306580133112408e-02,
    2.253800000844360163e+00,
    2.105404013591184675e+00,
    0.0, // fc_2, fitted 6.077353e-01
    1.446128804382169619e-01,
    3.433034688328007178e-02,
    -1.026477709293652056e-02,
    1.766336708713055759e-01,
    8.437457069799782217e-02,
    6.316907677501149010e-02,
    3.134808117219139989e-03,
    1.155594458153297976e+00,
    0.0, // fc_3, fitted 3.166264e-01
    1.789067052626122090e-01,
    4.954458897423385383e-02,
    -1.185080033177087280e-02,
    3.586803390895657029e-01,
    -4.913888882763408489e-02,
    -1.383704141563423251e-01,
    9.895036052344310384e-01,
    7.249958420981289770e-07,
    0.0; // fc_4, fitted 1.026475e+00

    return leader_beta;
}

#endif
