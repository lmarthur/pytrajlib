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
 * Bound the two flap-channel angle-of-attack components to the range the
 * aerodynamic model covers.
 *
 * Only the x and y components drive the flap pairs, so the pair is rescaled
 * together. Clamping each channel on its own would rotate the commanded
 * direction rather than shorten it.
 *
 * @param aoa Angle-of-attack components in radians, indexed by flap pair.
 * @param max_aoa Maximum modeled angle of attack in radians.
 * @return The input, rescaled if its transverse magnitude exceeded max_aoa.
 */
static inline cartvec clamp_aoa_channels(cartvec aoa, double max_aoa) {
  double magnitude = sqrt(aoa.x * aoa.x + aoa.y * aoa.y);
  if (magnitude <= max_aoa || magnitude < 1e-12) {
    return aoa;
  }
  double scale = max_aoa / magnitude;
  return (cartvec){aoa.x * scale, aoa.y * scale, aoa.z};
}

/**
 * Largest angle of attack the flaps can hold in trim.
 *
 * Commanding more than this asks the inversion for a deflection the actuator
 * cannot reach, which leaves the tracking error dominated by the unreachable
 * setpoint rather than by the vehicle's actual state, so the feedback gains
 * have nothing left to regulate.
 *
 * @param vehicle Pointer to vehicle model.
 * @param run_params Pointer to run configuration parameters.
 * @return Maximum trimmable angle of attack in radians.
 */
static inline double get_max_trimmable_aoa(vehicle *vehicle,
                                           runparams *run_params) {
  if (fabs(vehicle->rv.c_m_alpha) < 1e-12) {
    return INFINITY;
  }
  return fabs(vehicle->rv.c_m_delta * get_max_deflection_extent(run_params) /
              vehicle->rv.c_m_alpha);
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

  // Set realistic clamps
  double max_aoa = get_max_modeled_aoa(vehicle);
  double trimmable_aoa = get_max_trimmable_aoa(vehicle, run_params);
  if (trimmable_aoa < max_aoa) {
    max_aoa = trimmable_aoa;
  }
  aoa_des = clamp_aoa_channels(aoa_des, max_aoa);

  // Proportional controller term
  cartvec aoa_term =
      add(smultiply(aoa_des, run_params->K_pp * vehicle->rv.c_m_alpha),
          smultiply(aoa_est, vehicle->rv.c_m_alpha * (1 - run_params->K_pp)));

  // Derivative controller term. The two channels are indexed by flap pair, not
  // by body axis: flap pair {1, 3} trims aoa.x, which a body rate changes at
  // d(aoa.x)/dt = -omega_B.y, and flap pair {2, 4} trims aoa.y, which changes
  // at d(aoa.y)/dt = +omega_B.x.
  cartvec e3_B = {0.0, 0.0, 1.0};
  cartvec omega_channel_B = cross(e3_B, est_state->angular_vel_B);
  cartvec omega_term =
      smultiply(omega_channel_B, -run_params->K_q * vehicle->rv.c_m_q *
                                     vehicle->rv.rv_length /
                                     (2.0 * norm(est_state->velocity)));

  // Calculate desired flap deflection
  cartvec delta_des =
      sdivide(add(aoa_term, omega_term), -vehicle->rv.c_m_delta);

  // Hold the command inside the actuator's limits
  double max_extent = get_max_deflection_extent(run_params);
  delta_des.x = clip(delta_des.x, -max_extent, max_extent);
  delta_des.y = clip(delta_des.y, -max_extent, max_extent);

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

  double est_speed = norm(est_state->velocity);
  cartvec a_est_transverse = zeros();
  if (est_speed > 1e-12) {
    cartvec vhat = sdivide(est_state->velocity, est_speed);
    a_est_transverse = subtract(a_imu, smultiply(vhat, dot(a_imu, vhat)));
  }

  if (run_params->rv_maneuv != 1 || !is_reentry(est_state, t, run_params)) {
    record_reentry_guidance_sample(t, zeros(), a_est_transverse, 0.0, zeros());
    return;
  }
  // Estimate dynamic pressure
  cartvec v_rel_E = get_relative_wind_eci(est_state, est_atm);
  double v_rel = norm(v_rel_E);
  double q_inf = 0.5 * fmax(est_atm->density, 0.0) * v_rel * v_rel;

  // Zero atmospheric density means no aerodynamic control authority.
  if (q_inf < 1e-12) {
    record_reentry_guidance_sample(t, zeros(), a_est_transverse, 0.0, zeros());
    return;
  }

  // Estimate angle of attack
  cartvec u_hat_E = sdivide(v_rel_E, v_rel);
  cartvec u_hat_B = eci_to_body(u_hat_E, est_state->q_EB);

  double max_aoa = get_max_modeled_aoa(vehicle);
  double cos_aoa = u_hat_B.z;
  if (fabs(cos_aoa) < 1e-6) {
    cos_aoa = cos_aoa < 0.0 ? -1e-6 : 1e-6;
  }
  cartvec aoa_est = clamp_aoa_channels(sdivide(u_hat_B, cos_aoa), max_aoa);

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
  double desired_aoa_deg = fmin(des_aoa, max_aoa) * 180 / M_PI;
  record_reentry_guidance_sample(t, a_cmd_E, a_est_transverse, desired_aoa_deg,
                                 desired_flap_deflection);
}

#endif
