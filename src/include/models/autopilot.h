#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#define _USE_MATH_DEFINES

#include "../math/linalg.h"
#include "../physics/aero_forces.h"
#include "../utils/propnav.h"
#include "../utils/run_logging.h"
#include "../utils/utils.h"
#include "atmosphere.h"
#include "state.h"
#include "vehicle.h"

/**
 * Get the desired flap deflection angle using nonlinear dynamic inversion (NDI)
 * with a PD controller that reduces the error between the current estimated
 * angle of attack and the desired angle of attack.
 */
cartvec NDI(cartvec a_cmd_B, state *est_state, cartvec aoa_est,
            vehicle *vehicle, double q_inf, runparams *run_params,
            double c_l_alpha, double c_m_alpha, double c_m_q) {
  // Get desired angle of attack
  cartvec aoa_des = smultiply(
      a_cmd_B, vehicle->rv.rv_mass / (q_inf * vehicle->rv.rv_area * c_l_alpha));

  // Proportional controller term
  cartvec aoa_term =
      add(smultiply(aoa_des, run_params->K_pp * c_m_alpha),
          smultiply(aoa_est, c_m_alpha * (1 - run_params->K_pp)));

  // Derivative controller term
  cartvec omega_term =
      smultiply(est_state->angular_vel_B,
                -run_params->K_q * c_m_q * vehicle->rv.rv_length /
                    (2.0 * norm(est_state->velocity)));

  // Calculate desired flap deflection
  cartvec delta_des =
      sdivide(add(aoa_term, omega_term), -vehicle->rv.c_m_delta);

  // Account for proper flap deflection direction
  delta_des = smultiply(delta_des, -1);
  return delta_des;
}

/**
 * Steer the vehicle by changing the flap angular velocity.
 *
 * This function also logs to the guidance log file.
 */
void get_flap_angular_acceleration(double t, state *est_state,
                                   runparams *run_params, vehicle *vehicle,
                                   grav *est_grav, atm_cond *est_atm,
                                   cartvec a_imu, double *ddot_deflection) {
  ddot_deflection[0] = 0;
  ddot_deflection[1] = 0;

  cartvec vhat = sdivide(est_state->velocity, norm(est_state->velocity));
  cartvec a_est_transverse = subtract(a_imu, smultiply(vhat, dot(a_imu, vhat)));

  if (run_params->rv_maneuv != 1 || !is_reentry(est_state, t)) {
    write_reentry_guidance_log_row(t, zeros(), a_est_transverse, 0.0, zeros());
    return;
  }
  // Estimate dynamic pressure
  cartvec v_rel_E = get_relative_wind_eci(est_state, est_atm);
  double v_rel = norm(v_rel_E);
  double q_inf = 0.5 * fmax(est_atm->density, 0.0) * v_rel * v_rel;

  // Estimate angle of attack
  cartvec u_hat_E = sdivide(v_rel_E, norm(v_rel_E));
  cartvec u_hat_B = eci_to_body(u_hat_E, est_state->q_EB);
  cartvec aoa_est = sdivide(u_hat_B, u_hat_B.z);
  // Estimated scalar angle of attack (radians) from the body-relative wind
  double alpha_est = get_aoa(u_hat_B);
  double alpha_deg_est = alpha_est * 180.0 / M_PI;

  // Compute interpolated/derived aerodynamic coefficients using the
  // estimated angle-of-attack.
  double c_l_alpha = vehicle->rv.c_l_alpha;
  double c_m_alpha = vehicle->rv.c_m_alpha;
  double c_m_q = vehicle->rv.c_m_q;
  if (vehicle->rv.aero_table_size > 0) {
    // interpolate_table_derivative returns dC/d(alpha_deg) (per degree)
    double dC_L_dd = interpolate_table_derivative(
        alpha_deg_est, vehicle->rv.aero_alpha_deg_table, vehicle->rv.c_l_table,
        vehicle->rv.aero_table_size);
    double dC_M_dd = interpolate_table_derivative(
        alpha_deg_est, vehicle->rv.aero_alpha_deg_table, vehicle->rv.c_m_table,
        vehicle->rv.aero_table_size);
    double C_Mq =
        interpolate_table(alpha_deg_est, vehicle->rv.aero_alpha_deg_table,
                          vehicle->rv.c_m_q_table, vehicle->rv.aero_table_size);

    // convert derivative per degree to per radian
    c_l_alpha = dC_L_dd / (M_PI / 180.0);
    c_m_alpha = dC_M_dd / (M_PI / 180.0);
    c_m_q = C_Mq;
  }

  // Get acceleration command & estimated gravitational acceleration
  cartvec a_cmd_E = prop_nav(est_state, run_params, est_grav);

  // Convert to body frame
  cartvec a_cmd_B_no_grav = eci_to_body(a_cmd_E, est_state->q_EB);

  // Determine change in deflection angle required to produce change in
  // acceleration
  cartvec desired_flap_deflection =
      NDI(a_cmd_B_no_grav, est_state, aoa_est, vehicle, q_inf, run_params,
          c_l_alpha, c_m_alpha, c_m_q);

  // Use a PD controller to set the flap acceleration
  ddot_deflection[0] =
      run_params->K_delta_p * (desired_flap_deflection.x - est_state->delta_1) -
      run_params->K_delta_d * est_state->dot_delta_1;
  ddot_deflection[1] =
      run_params->K_delta_p * (desired_flap_deflection.y - est_state->delta_2) -
      run_params->K_delta_d * est_state->dot_delta_2;

  // Log acceleration command and transverse imu
  double des_aoa = norm(a_cmd_E) * vehicle->rv.rv_mass /
                   (q_inf * vehicle->rv.rv_area * c_l_alpha);
  double desired_aoa_deg = des_aoa * 180 / M_PI;
  write_reentry_guidance_log_row(t, a_cmd_E, a_est_transverse, desired_aoa_deg,
                                 desired_flap_deflection);
}

#endif
