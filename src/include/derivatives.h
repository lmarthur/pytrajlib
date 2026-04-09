#ifndef DERIVATIVES_H
#define DERIVATIVES_H

#include "forces/drag.h"
#include "forces/gravity.h"
#include "forces/lift.h"
#include "forces/thrust.h"
#include "models/atmosphere.h"
#include "models/grav.h"
#include "models/sensors.h"
#include "models/state.h"
#include "models/vehicle.h"
#include "utils/utils.h"

typedef int (*drift_func)(runparams *run_params, imu *imu, vehicle *vehicle,
                          grav *true_grav, grav *est_grav,
                          atm_cond *true_atm_cond, atm_cond *est_atm_cond,
                          state *true_state, state *est_state, double true_t,
                          double est_t, state *true_state_drift,
                          state *est_state_drift);

typedef void (*diffusion_func)(imu *imu, state *est_state_diffusion);

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
  cartvec a_lift_true = get_lift_acc(true_state, est_state, run_params, vehicle,
                                     true_atm_cond, true_t, est_grav);
  cartvec a_grav_true = get_gravity_acc(true_grav, true_state);
  cartvec a_grav_est = get_gravity_acc(est_grav, est_state);
  cartvec a_drag_true;
  if (run_params->include_drag) {
    a_drag_true =
        get_drag_acc(run_params, vehicle, true_atm_cond, true_state, true_t);
  } else {
    a_drag_true = zeros();
  }

  cartvec a_total_true =
      add(add(add(a_thrust_true, a_drag_true), a_lift_true), a_grav_true);
  cartvec a_total_est = imu_measurement(
      imu, run_params, true_atm_cond, est_atm_cond, true_t, est_t, true_state,
      est_state, a_total_true, a_grav_true, a_grav_est, est_grav);
  double dot_delta = get_deflection_angular_speed(
      est_state, vehicle, est_atm_cond, run_params, true_t, est_grav);
  double ddot_alpha = get_aoa_angular_acceleration(
      true_state, run_params, vehicle, true_atm_cond, true_t);

  // Set true state derivatives
  true_state_drift->position = true_state->velocity;
  true_state_drift->velocity = a_total_true;
  true_state_drift->gyro_error = (anglevec){0};
  true_state_drift->alpha = true_state->d_alpha_dt;
  true_state_drift->d_alpha_dt = ddot_alpha;
  true_state_drift->deflection_angle = dot_delta;

  // Set estimated state derivatives
  est_state_drift->position = est_state->velocity;
  est_state_drift->velocity = a_total_est;

  // If ballistic run, turn off gyro error accumulation after boost phase
  // to avoid slightly overestimating Coriolis error
  if (run_params->rv_maneuv == 0 &&
      true_t >= vehicle->booster.total_burn_time) {
    true_state_drift->gyro_error = (anglevec){0};
  } else {
    true_state_drift->gyro_error = get_gyro_drift(imu);
  }
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
 */
void diffusion(imu *imu, state *true_state_diffusion) {
  true_state_diffusion->gyro_error.pitch = get_gyro_diffusion(imu);
  true_state_diffusion->gyro_error.yaw = get_gyro_diffusion(imu);
}

#endif