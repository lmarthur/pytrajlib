#ifndef DERIVATIVES_H
#define DERIVATIVES_H

#include "models/atmosphere.h"
#include "models/autopilot.h"
#include "models/grav.h"
#include "models/sensors.h"
#include "models/state.h"
#include "models/vehicle.h"
#include "physics/aero_forces.h"
#include "physics/aero_moments.h"
#include "physics/drag.h"
#include "physics/gravity.h"
#include "physics/thrust.h"
#include "utils/utils.h"

typedef int (*drift_func)(runparams *run_params, imu *imu, vehicle *vehicle,
                          grav *true_grav, grav *est_grav,
                          atm_cond *true_atm_cond, atm_cond *est_atm_cond,
                          state *true_state, state *est_state, double true_t,
                          double est_t, state *true_state_drift,
                          state *est_state_drift);

typedef void (*diffusion_func)(imu *imu, state *true_state_diffusion,
                               state *est_state_diffusion);

/**
 * Calculate deterministic drift component of the state update.
 *
 * This function writes derivative outputs into `true_state_drift` and
 * `est_state_drift`. These output state structs should be initialized to
 * `{0}` before being passed in.
 *
 * @param run_params Pointer to run configuration parameters.
 * @param imu Pointer to IMU model.
 * @param vehicle Pointer to vehicle model.
 * @param true_grav Pointer to true gravity model.
 * @param est_grav Pointer to estimated gravity model.
 * @param true_atm_cond Pointer to true atmospheric conditions.
 * @param est_atm_cond Pointer to estimated atmospheric conditions.
 * @param true_state Pointer to true state.
 * @param est_state Pointer to estimated state.
 * @param true_t Current true simulation time in seconds.
 * @param est_t Current estimated simulation time in seconds.
 * @param true_state_drift Output deterministic drift for true state; initialize
 *                         to `{0}` before passing.
 * @param est_state_drift Output deterministic drift for estimated state;
 *                        initialize to `{0}` before passing.
 * @return 1 on success, 0 if thrust guidance fails
 */
int drift(runparams *run_params, imu *imu, vehicle *vehicle, grav *true_grav,
          grav *est_grav, atm_cond *true_atm_cond, atm_cond *est_atm_cond,
          state *true_state, state *est_state, double true_t, double est_t,
          state *true_state_drift, state *est_state_drift) {

  // Get thrust acceleration
  cartvec a_thrust_true = get_thrust_acc(
      true_state, est_state, vehicle, run_params, true_grav, est_grav, true_t);

  // If Lambert Guidance fails, quickly exit
  if (isnan(a_thrust_true.x)) {
    return 0;
  }

  // Calculate total acceleration
  cartvec a_grav_true = get_gravity_acc(true_grav, true_state);
  cartvec a_grav_est = get_gravity_acc(est_grav, est_state);
  cartvec a_aero_true = get_aerodynamic_acc(true_t, true_state, true_atm_cond,
                                            vehicle, run_params);
  cartvec angular_vel_B = get_angular_acceleration(
      true_t, true_state, true_atm_cond, vehicle, run_params);

  cartvec a_total_true = add(add(a_thrust_true, a_aero_true), a_grav_true);
  cartvec a_total_est = imu_measurement(
      imu, run_params, true_atm_cond, est_atm_cond, true_t, est_t, true_state,
      est_state, a_total_true, a_grav_true, a_grav_est, est_grav);

  double dot_deflection[2] = {0, 0};
  get_flap_angular_velocity(true_t, est_state, run_params, vehicle, est_grav,
                            est_atm_cond, a_total_est, dot_deflection);

  // Set true state derivatives
  true_state_drift->position = true_state->velocity;
  true_state_drift->velocity = a_total_true;
  true_state_drift->angular_vel_B = angular_vel_B;
  true_state_drift->orientation_angle_change =
      (anglevec){angular_vel_B.y, angular_vel_B.x};
  true_state_drift->gyro_error = (anglevec){0};
  true_state_drift->delta_1 = dot_deflection[0];
  true_state_drift->delta_2 = dot_deflection[1];

  // Set estimated state derivatives
  est_state_drift->position = est_state->velocity;
  est_state_drift->velocity = a_total_est;
  est_state_drift->angular_vel_B = zeros();
  est_state_drift->orientation_angle_change =
      (anglevec){angular_vel_B.y + imu->gyro_bias.pitch,
                 angular_vel_B.x + imu->gyro_bias.yaw};
  est_state_drift->delta_1 = dot_deflection[0];
  est_state_drift->delta_2 = dot_deflection[1];

  true_state_drift->gyro_error = get_gyro_drift(imu);
  return 1;
}

/**
 * Calculate stochastic diffusion component of the state update.
 *
 * This function writes diffusion outputs into `true_state_diffusion`.
 * The output state struct should be initialized to `{0}` before being passed
 * in.
 *
 * @param imu Pointer to IMU model.
 * @param true_state_diffusion Output stochastic diffusion for true state;
 *                             initialize to `{0}` before passing.
 * @param est_state_diffusion Output stochastic diffusion for estimated state;
 *                            initialize to `{0}` before passing.
 */
void diffusion(imu *imu, state *true_state_diffusion,
               state *est_state_diffusion) {
  true_state_diffusion->gyro_error.pitch = get_gyro_diffusion(imu);
  true_state_diffusion->gyro_error.yaw = get_gyro_diffusion(imu);
  true_state_diffusion->orientation_angle_change = (anglevec){0};
  est_state_diffusion->orientation_angle_change.pitch = get_gyro_diffusion(imu);
  est_state_diffusion->orientation_angle_change.yaw = get_gyro_diffusion(imu);
}

#endif