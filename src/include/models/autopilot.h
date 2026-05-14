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

double get_da_ddelta(state *est_state, vehicle *vehicle, atm_cond *est_atm_cond,
                     double q_inf) {
  double da_ddelta = q_inf * vehicle->rv.rv_area * vehicle->rv.c_l_alpha *
                     (-vehicle->rv.c_m_delta / vehicle->rv.c_m_alpha) /
                     vehicle->rv.rv_mass;
  return da_ddelta;
}

void get_flap_angular_velocity(double t, state *est_state,
                               runparams *run_params, vehicle *vehicle,
                               grav *est_grav, atm_cond *est_atm, cartvec a_imu,
                               double *dot_deflection) {
  dot_deflection[0] = 0;
  dot_deflection[1] = 0;

  cartvec vhat = sdivide(est_state->velocity, norm(est_state->velocity));
  cartvec a_est_transverse = subtract(a_imu, smultiply(vhat, dot(a_imu, vhat)));

  if (run_params->rv_maneuv != 1 || !is_reentry(est_state, t)) {
    write_reentry_guidance_log_row(t, zeros(), a_est_transverse);
    return;
  }
  // Estimate dynamic pressure
  cartvec v_rel_E = get_relative_wind_eci(est_state, est_atm);
  double v_rel = norm(v_rel_E);
  double q_inf = 0.5 * fmax(est_atm->density, 0.0) * v_rel * v_rel;

  // Get acceleration command
  cartvec a_cmd_E = prop_nav(est_state, run_params);

  // Transform acceleration change from ECI frame to local body frame
  cartvec a_cmd_B = eci_to_body(a_cmd_E, est_state->q_EB);

  // Get derivative of acceleration wrt deflection angle
  double da_ddelta = get_da_ddelta(est_state, vehicle, est_atm, q_inf);

  // Determine change in deflection angle required to produce change in
  // acceleration
  cartvec desired_flap_deflection = sdivide(a_cmd_B, -1 * da_ddelta);

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
  write_reentry_guidance_log_row(t, a_cmd_E, a_est_transverse);
}

#endif
