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
#include "utils.h"

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
  cartvec a_thrust;
  if (run_params->perfect_boost) {
    a_thrust =
        get_thrust_acc(true_state, vehicle, run_params, true_grav, true_t);
  } else {
    a_thrust = get_thrust_acc(est_state, vehicle, run_params, est_grav, est_t);
  }
  // If Lambert Guidance fails, quickly exit
  if (isnan(a_thrust.x)) {
    return 0;
  }
  // Get time derivative of available lift acceleration (same for true &
  // estimated states)
  cartvec d_a_lift_avail_dt = get_a_lift_avail_jerk(
      true_state, est_state, run_params, vehicle, est_atm_cond, est_t);

  state *states[2] = {true_state, est_state};
  state *state_derivs[2] = {true_state_drift, est_state_drift};
  grav *grav_models[2] = {true_grav, est_grav};
  atm_cond *atm_conds[2] = {true_atm_cond, est_atm_cond};
  cartvec a_gravs[2];
  cartvec a_total_true;
  double times[2] = {true_t, est_t};
  for (int i = 0; i < 2; i++) {
    // Calculate total acceleration
    cartvec a_total;
    cartvec d_a_lift_dt = zeros();
    a_gravs[i] = get_gravity_acc(grav_models[i], states[i]);

    // If using INS navigation, then the estimated state does not need to
    // calculate every force individually
    if (run_params->ins_nav == 1 && i == 1) {
      // If ballistic run, turn off gyro error accumulation after boost phase
      // to avoid slightly overestimating Coriolis error
      if (run_params->rv_maneuv == 0 &&
          true_t >= vehicle->booster.total_burn_time) {
        a_total = a_total_true;
      } else {
        a_total = imu_measurement(imu, true_state, est_state, a_total_true,
                                  a_gravs[0], a_gravs[1]);
      }
    } else {
      cartvec a_drag;
      if (run_params->include_drag == 1) {
        a_drag = get_drag_acc(run_params, vehicle, atm_conds[i], states[i],
                              times[i]);
      } else {
        a_drag = zeros();
      }
      a_total = add(add(add(a_thrust, a_drag), states[i]->a_lift), a_gravs[i]);

      // If realistic maneuverable RV, use proportional navigation during
      // reentry
      if (run_params->rv_maneuv == 1 && is_reentry(states[i], times[i])) {
        // Get lift jerk
        d_a_lift_dt = get_a_lift_jerk(states[i], run_params, vehicle,
                                      atm_conds[i], times[i]);
      }
    }
    // Keep track of true total acceleration for estimated state's accelerometer
    // measurement
    if (run_params->ins_nav == 1 && i == 0) {
      a_total_true = a_total;
    }

    // Set state derivatives
    state_derivs[i]->position = states[i]->velocity;
    state_derivs[i]->velocity = a_total;
    state_derivs[i]->a_lift = d_a_lift_dt;
    state_derivs[i]->a_lift_avail = d_a_lift_avail_dt;
    state_derivs[i]->gyro_error = get_gyro_drift(imu);
  }
  return 1;
}

void diffusion(imu *imu, state *est_state_diffusion) {
  est_state_diffusion->gyro_error.lat = get_gyro_diffusion(imu);
  est_state_diffusion->gyro_error.lon = get_gyro_diffusion(imu);
}

#endif