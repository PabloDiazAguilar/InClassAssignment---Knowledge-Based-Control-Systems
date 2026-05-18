// ============================================================================
//  auv_control/auv_params.hpp
//
//  Physical / geometric parameters for the AUV and the thrust-allocation
//  configuration matrix B (Eq. 26 of Zhang et al., Sensors 24, 3029).
//
//  Control layout:
//      u = [u1 u2 u3 u4]^T   (four actuator channels)
//      tau = [tau_x tau_y tau_z tau_m tau_n]^T  (body-frame wrench)
//      tau = B * u
//
//      with B (5x4):
//
//          | l*cos(alpha)   cos(alpha)   cos(beta)    cos(beta)  |
//          | sin(alpha)     cos(alpha)   0            0          |
//          | 0              0            cos(beta)    sin(beta)  |
//          | A              A            A            A          |
//          | B_arm          B_arm        B_arm        B_arm      |
//
//      where A = (b/2)*sin(alpha) + (a/2)*cos(alpha)
//            B_arm = (b/2)*cos(beta) + (a/2)*sin(beta).
//
//  The paper leaves alpha, beta, a, b, l as geometric constants of the
//  propulsion system. We pick values consistent with a small torpedo AUV.
// ============================================================================
#ifndef AUV_CONTROL__AUV_PARAMS_HPP_
#define AUV_CONTROL__AUV_PARAMS_HPP_

#include <Eigen/Dense>

namespace auv_control {

// ---- Shared small-vector types (used by every block) ----------------------
using StateVec   = Eigen::Matrix<double, 5, 1>;   // [u v w q r]
using ControlVec = Eigen::Matrix<double, 4, 1>;   // [u1 u2 u3 u4]
using WrenchVec  = Eigen::Matrix<double, 5, 1>;   // [tau_x tau_y tau_z tau_m tau_n]

// ---- Vehicle mass / buoyancy parameters ------------------------------------
// Matches the URDF in auv_description (~22 kg hull + ~3.2 kg thrusters).
struct VehicleParams {
  double mass            = 25.0;    // kg
  double buoyancy_force  = 25.0 * 9.81 * 1.01;  // slightly positive buoyancy, N
  double gravity         = 9.81;    // m/s^2

  // Linear drag (diagonal) coefficients Xu, Yv, Zw, Mq, Nr in body frame.
  // Chosen so the AUV reaches ~1.5 m/s cruising with ~50 N surge thrust.
  double drag_u = 25.0;
  double drag_v = 60.0;
  double drag_w = 60.0;
  double drag_p = 5.0;
  double drag_q = 15.0;
  double drag_r = 15.0;

  // Quadratic drag (|v|*v terms)
  double drag_uu = 10.0;
  double drag_vv = 40.0;
  double drag_ww = 40.0;
  double drag_qq = 4.0;
  double drag_rr = 4.0;

  // Maximum thruster output (N) — used as saturation limit in QP.
  double thrust_max = 50.0;
  double thrust_min = -50.0;

  // Max norm per channel for virtual u (N before allocation scaling).
  // Matches the "Amplitude 1..2" range used in the paper's figures.
  double u_scale = 25.0;
};

// ---- Thrust-allocation geometry --------------------------------------------
// Actuator → DOF mapping (must match K gains in ts_fuzzy.cpp):
//   u1 → surge  (tau_x),  u2 → heave  (tau_z)
//   u3 → pitch  (tau_m),  u4 → yaw    (tau_n)
struct AllocParams {
  double a = 0.50;   // m, pitch rudder moment arm (rud_offset_x in URDF)
  double b = 0.50;   // m, yaw   rudder moment arm (rud_offset_x in URDF)
  double l = 1.00;   // surge thrust scale
};

// Build the 5x4 configuration matrix B.
// Each column maps one virtual actuator to the 5-DOF body-frame wrench.
inline Eigen::Matrix<double, 5, 4> build_B(const AllocParams & g) {
  Eigen::Matrix<double, 5, 4> B;
  //         u1(surge)  u2(heave)  u3(pitch)  u4(yaw)
  B << g.l,    0.0,       0.0,       0.0,   // tau_x
       0.0,    0.0,       0.0,       0.0,   // tau_y (uncontrolled)
       0.0,    1.0,       0.0,       0.0,   // tau_z
       0.0,    0.0,       g.a,       0.0,   // tau_m  (pitch rudder arm)
       0.0,    0.0,       0.0,       g.b;   // tau_n  (yaw   rudder arm)
  return B;
}

}  // namespace auv_control

#endif  // AUV_CONTROL__AUV_PARAMS_HPP_
