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
 * Get the the derivative of lift acceleration with respect to flap deflection
 * angle, assuming the vehicle remains in static trim.
 */
double get_da_ddelta(state *est_state, vehicle *vehicle, atm_cond *est_atm_cond,
                     double q_inf) {
  double da_ddelta = q_inf * vehicle->rv.rv_area * vehicle->rv.c_l_alpha *
                     (-vehicle->rv.c_m_delta / vehicle->rv.c_m_alpha) /
                     vehicle->rv.rv_mass;
  return da_ddelta;
}

/**
 * Get the desired flap deflection angle using nonlinear dynamic inversion (NDI)
 * with a PD controller that reduces the error between the current estimated
 * angle of attack and the desired angle of attack.
 */
cartvec NDI(cartvec a_cmd_B, state *est_state, cartvec aoa_est,
            vehicle *vehicle, double q_inf, runparams *run_params) {
  // Get desired angle of attack
  cartvec aoa_des =
      smultiply(a_cmd_B, vehicle->rv.rv_mass / (q_inf * vehicle->rv.rv_area *
                                                vehicle->rv.c_l_alpha));

  // Proportional controller term
  cartvec aoa_term =
      add(smultiply(aoa_des, run_params->K_pp * vehicle->rv.c_m_alpha),
          smultiply(aoa_est, vehicle->rv.c_m_alpha * (1 - run_params->K_pp)));

  // Derivative controller term
  cartvec omega_term =
      smultiply(est_state->angular_vel_B,
                run_params->K_q * vehicle->rv.c_m_q * vehicle->rv.rv_length /
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

  // Get acceleration command & estimated gravitational acceleration
  cartvec a_cmd_E = prop_nav(est_state, run_params, est_grav);

  // Convert to body frame
  cartvec a_cmd_B_no_grav = eci_to_body(a_cmd_E, est_state->q_EB);

  // Determine change in deflection angle required to produce change in
  // acceleration
  cartvec desired_flap_deflection =
      NDI(a_cmd_B_no_grav, est_state, aoa_est, vehicle, q_inf, run_params);

  // Use a PD controller to set the flap acceleration
  ddot_deflection[0] =
      run_params->K_delta_p * (desired_flap_deflection.x - est_state->delta_1) -
      run_params->K_delta_d * est_state->dot_delta_1;
  ddot_deflection[1] =
      run_params->K_delta_p * (desired_flap_deflection.y - est_state->delta_2) -
      run_params->K_delta_d * est_state->dot_delta_2;

  // Log acceleration command and transverse imu
  double des_aoa = norm(a_cmd_E) * vehicle->rv.rv_mass /
                   (q_inf * vehicle->rv.rv_area * vehicle->rv.c_l_alpha);
  double desired_aoa_deg = des_aoa * 180 / M_PI;
  write_reentry_guidance_log_row(t, a_cmd_E, a_est_transverse, desired_aoa_deg,
                                 desired_flap_deflection);
}

#endif
