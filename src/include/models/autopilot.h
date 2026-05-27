#ifndef AUTOPILOT_H
#define AUTOPILOT_H

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
 * Get the desired angle of attack (in degrees) by assuming a constant
 * C_L_alpha. The desired angle of attack is another way of representing the
 * desired lift magnitude.
 */
double get_desired_aoa(double a_cmd_mag, vehicle *vehicle, double q_inf) {
  double des_aoa = a_cmd_mag * vehicle->rv.rv_mass /
                   (q_inf * vehicle->rv.rv_area * vehicle->rv.c_l_alpha);

  return des_aoa * 180 / M_PI;
}

/**
 * Steer the vehicle by changing the flap angular velocity.
 *
 * This function also logs to the guidance log file.
 */
void get_flap_angular_velocity(double t, state *est_state,
                               runparams *run_params, vehicle *vehicle,
                               grav *est_grav, atm_cond *est_atm, cartvec a_imu,
                               double *dot_deflection) {
  dot_deflection[0] = 0;
  dot_deflection[1] = 0;

  cartvec vhat = sdivide(est_state->velocity, norm(est_state->velocity));
  cartvec a_est_transverse = subtract(a_imu, smultiply(vhat, dot(a_imu, vhat)));

  if (run_params->rv_maneuv != 1 || !is_reentry(est_state, t)) {
    write_reentry_guidance_log_row(t, zeros(), a_est_transverse, 0.0, zeros(),
                                   zeros());
    return;
  }
  // Estimate dynamic pressure
  cartvec v_rel_E = get_relative_wind_eci(est_state, est_atm);
  double v_rel = norm(v_rel_E);
  double q_inf = 0.5 * fmax(est_atm->density, 0.0) * v_rel * v_rel;

  // Get acceleration command & estimated gravitational acceleration
  cartvec a_cmd_E = prop_nav(est_state, run_params);
  cartvec a_grav_E = get_gravity_acc(est_grav, est_state);

  // Subtract gravity from the acceleration command
  cartvec a_cmd_E_no_grav = subtract(a_cmd_E, a_grav_E);

  // Convert to body frame
  cartvec a_cmd_B_no_grav = eci_to_body(a_cmd_E_no_grav, est_state->q_EB);

  // Get derivative of lift acceleration wrt deflection angle
  double da_ddelta = get_da_ddelta(est_state, vehicle, est_atm, q_inf);

  // Determine change in deflection angle required to produce change in
  // acceleration. Multiply by -1 to ensure flaps deflect in the correct
  // direction
  cartvec desired_flap_deflection = sdivide(a_cmd_B_no_grav, -1.0 * da_ddelta);

  // Set the deflection speed to be proportional to the difference between the
  // current deflection angle and the desired deflection angle, subject to the
  // pre-programmed deflection time constant tau
  double dot_deflection_1 = (desired_flap_deflection.x - est_state->delta_1) /
                            run_params->tau_deflect;
  double dot_deflection_2 = (desired_flap_deflection.y - est_state->delta_2) /
                            run_params->tau_deflect;

  // Clip the deflection angular velocity to the maximum flap angular velocity
  double max_deflection_angle = run_params->max_deflection_angle * M_PI / 180.0;
  double max_deflection_speed =
      max_deflection_angle /
      (run_params->deflection_time * run_params->gearing_ratio);

  dot_deflection[0] =
      clip(dot_deflection_1, -max_deflection_speed, max_deflection_speed);
  dot_deflection[1] =
      clip(dot_deflection_2, -max_deflection_speed, max_deflection_speed);
  // Log acceleration command and transverse imu acceleration
  double desired_aoa_deg =
      get_desired_aoa(norm(a_cmd_E_no_grav), vehicle, q_inf);
  write_reentry_guidance_log_row(t, a_cmd_E, a_est_transverse, desired_aoa_deg,
                                 desired_flap_deflection, zeros());
}

#endif
