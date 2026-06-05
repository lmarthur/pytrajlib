#ifndef AERO_MOMENTS_H
#define AERO_MOMENTS_H

#define _USE_MATH_DEFINES

#include "../math/linalg.h"
#include "../models/vehicle.h"
#include "aero_forces.h"

/**
 * Compute body pitching-moment axis in body coordinates.
 *
 * For nontrivial AoA (s = sqrt(u1^2 + u2^2) > 0), Eq. 29 form is used:
 * $$\hat{\mathbf m}_B = \frac{1}{s}[u_2, -u_1, 0]^T.$$
 *
 * @param u_hat_B Unit relative-wind vector in body coordinates.
 * @return Unit body pitching-moment axis, or zero vector if near singular.
 */
static inline cartvec get_body_moment_axis(cartvec u_hat_B) {
  double s = sqrt(u_hat_B.x * u_hat_B.x + u_hat_B.y * u_hat_B.y);
  if (s < 1e-10) {
    return zeros();
  }

  cartvec m_hat_B = {u_hat_B.y / s, -u_hat_B.x / s, 0.0};
  return m_hat_B;
}

/**
 * Get transverse angular velocity in body coordinates.
 *
 * Eq. 33 form:
 * $$\boldsymbol\omega_{\perp,B} = [\omega_{1,B},\omega_{2,B},0]^T.$$
 *
 * @param current_state Current vehicle state containing body angular velocity.
 * @return Transverse body angular velocity vector.
 */
static inline cartvec get_omega_perp_body(state *current_state) {
  cartvec omega_perp_B = {current_state->angular_vel_B.x,
                          current_state->angular_vel_B.y, 0.0};
  return omega_perp_B;
}

static inline cartvec get_body_moment(state *current_state, atm_cond *atm_cond,
                                      vehicle *vehicle) {
  // There are two components to the body moment: the moment due to the angle of
  // attack, M_alpha, and the moment due to the pitching angular velocity, M_q

  // Both moment coefficients vary based on the angle of attack
  cartvec v_rel_E = get_relative_wind_eci(current_state, atm_cond);
  cartvec u_hat_E = sdivide(v_rel_E, norm(v_rel_E));
  cartvec u_hat_B = eci_to_body(u_hat_E, current_state->q_EB);

  double alpha = get_aoa(u_hat_B);
  double C_M, C_Mq;
  if (strcmp(vehicle->rv.name, "SWERVE") == 0) {
    // Interpolate the coefficient tables using the angle of attack
    double alpha_deg = alpha * 180.0 / M_PI;
    C_Mq = interpolate_swerve_coeff(alpha_deg, SWERVE_CMQ_TABLE);
    C_M = interpolate_swerve_coeff(alpha_deg, SWERVE_CM_TABLE);
  } else {
    C_M = vehicle->rv.c_m_alpha * alpha;
    C_Mq = vehicle->rv.c_m_q;
  }

  // Calculate dynamic pressure
  double v_rel = norm(v_rel_E);
  double q_inf = 0.5 * atm_cond->density * v_rel * v_rel;

  // Reference area and length for the moment coefficients
  double S_ref = vehicle->rv.rv_area;
  double c_ref = vehicle->rv.rv_length;

  // Calculate M_alpha
  double M_alpha = q_inf * S_ref * c_ref * C_M;

  // Calculate M_q
  double M_q = q_inf * S_ref * c_ref * c_ref / (2 * v_rel) * C_Mq;

  // Get moment directions
  cartvec m_hat_B = get_body_moment_axis(u_hat_B);
  cartvec omega_perp_B = get_omega_perp_body(current_state);

  // Compute body moment
  cartvec body_moment =
      add(smultiply(m_hat_B, M_alpha), smultiply(omega_perp_B, M_q));
  return body_moment;
}

/**
 * Compute incremental flap centroid displacement vectors in body coordinates.
 *
 * @param vehicle Vehicle model containing RV geometry.
 * @param flap_r_B Output array of 4 flap displacement vectors in body
 * coordinates.
 */
static inline void get_flap_displacements_body(vehicle *vehicle,
                                               cartvec flap_r_B[4]) {
  double r_f_perp = vehicle->rv.rv_radius + 0.01;
  double c_f = fabs(vehicle->rv.x_com - vehicle->rv.x_flap) + 0.01;

  flap_r_B[0] = (cartvec){r_f_perp, 0.0, c_f};
  flap_r_B[1] = (cartvec){0.0, r_f_perp, c_f};
  flap_r_B[2] = (cartvec){-r_f_perp, 0.0, c_f};
  flap_r_B[3] = (cartvec){0.0, -r_f_perp, c_f};
}

static inline cartvec sum_incremental_moments(state *current_state,
                                              atm_cond *atm_cond,
                                              vehicle *vehicle,
                                              runparams *run_params) {
  // Get forces from deflected and undeflected flaps
  cartvec undeflected_flap_forces_B[4];
  cartvec deflected_flap_forces_B[4];
  get_absolute_flap_forces_body(current_state, atm_cond, vehicle, 0, 0,
                                undeflected_flap_forces_B);
  get_absolute_flap_forces_body(current_state, atm_cond, vehicle,
                                current_state->delta_1, current_state->delta_2,
                                deflected_flap_forces_B);

  // Get flap displacements
  cartvec flap_r_B[4];
  get_flap_displacements_body(vehicle, flap_r_B);

  // Sum over incremental moments
  cartvec sum_moments = zeros();
  for (int i = 0; i < 4; i++) {
    // Flap force is limited by the actuator
    cartvec incremental_force_limited = get_limited_incremental_flap_force(
        deflected_flap_forces_B[i], undeflected_flap_forces_B[i], run_params,
        vehicle);
    // Incremental moment is the cross product of the distance from the flap to
    // the center of mass and the incremental flap force
    sum_moments =
        add(sum_moments, cross(flap_r_B[i], incremental_force_limited));
  }
  return sum_moments;
}

static inline cartvec get_angular_acceleration(double t, state *true_state,
                                               atm_cond *atm_cond,
                                               vehicle *vehicle,
                                               runparams *run_params) {
  if (run_params->rv_maneuv != 1 || !is_reentry(true_state, t)) {
    return zeros();
  }
  // Get body moment + sum incremental moments from the flaps
  cartvec body_moment = get_body_moment(true_state, atm_cond, vehicle);
  cartvec incremental_flap_moment =
      sum_incremental_moments(true_state, atm_cond, vehicle, run_params);
  cartvec total_moment = add(body_moment, incremental_flap_moment);

  // We neglect angular acceleration around the roll axis
  total_moment.z = 0;

  // Angular acceleration is the moment divided by the moment of inertia around
  // the pitch/yaw axes
  cartvec angular_acceleration = sdivide(total_moment, vehicle->rv.Iyy);
  return angular_acceleration;
}

#endif