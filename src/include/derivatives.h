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
 */
int drift(runparams *run_params, imu *imu, vehicle *vehicle, grav *true_grav,
          grav *est_grav, atm_cond *true_atm_cond, atm_cond *est_atm_cond,
          state *true_state, state *est_state, double true_t, double est_t,
          state *true_state_drift, state *est_state_drift) {

  // Get thrust acceleration
  cartvec a_thrust_true;
  if (run_params->perfect_boost) {
    a_thrust_true =
        get_thrust_acc(true_state, vehicle, run_params, true_grav, true_t);
  } else {
    a_thrust_true =
        get_thrust_acc(est_state, vehicle, run_params, est_grav, est_t);
  }
  // If Lambert Guidance fails, quickly exit
  if (isnan(a_thrust_true.x)) {
    return 0;
  }
  // Get time derivative of available lift acceleration
  cartvec d_a_lift_avail_dt, d_a_lift_dt;
  if (run_params->rv_maneuv == 1) {
    d_a_lift_avail_dt = get_a_lift_avail_jerk(est_state, run_params, vehicle,
                                              est_atm_cond, est_t);

    d_a_lift_dt =
        get_a_lift_jerk(true_state, run_params, vehicle, true_atm_cond, true_t);
  } else {
    d_a_lift_avail_dt = zeros();
    d_a_lift_dt = zeros();
  }

  // Calculate total acceleration
  cartvec a_grav_true = get_gravity_acc(true_grav, true_state);
  cartvec a_grav_est = get_gravity_acc(est_grav, est_state);
  cartvec a_drag_true;
  if (run_params->include_drag) {
    a_drag_true =
        get_drag_acc(run_params, vehicle, true_atm_cond, true_state, true_t);
  } else {
    a_drag_true = zeros();
  }

  cartvec a_total_true = add(
      add(add(a_thrust_true, a_drag_true), true_state->a_lift), a_grav_true);
  cartvec a_total_est = imu_measurement(imu, true_state, est_state,
                                        a_total_true, a_grav_true, a_grav_est);

  // Set true state derivatives
  true_state_drift->position = true_state->velocity;
  true_state_drift->velocity = a_total_true;
  true_state_drift->a_lift = d_a_lift_dt;
  true_state_drift->a_lift_avail = d_a_lift_avail_dt;
  true_state_drift->gyro_error = (anglevec){0};

  // Set estimated state derivatives
  est_state_drift->position = est_state->velocity;
  est_state_drift->velocity = a_total_est;
  est_state_drift->a_lift = zeros();
  est_state_drift->a_lift_avail = d_a_lift_avail_dt;

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

void diffusion(imu *imu, state *true_state_diffusion) {
  true_state_diffusion->gyro_error.lat = get_gyro_diffusion(imu);
  true_state_diffusion->gyro_error.lon = get_gyro_diffusion(imu);
}

#endif