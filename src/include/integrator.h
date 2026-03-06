#ifndef INTEGRATOR_H
#define INTEGRATOR_H

#include "forces/drag.h"
#include "forces/gravity.h"
#include "forces/lift.h"
#include "forces/thrust.h"
#include "models/atmosphere.h"
#include "models/grav.h"
#include "models/sensors.h"
#include "models/state.h"
#include "models/vehicle.h"
#include "rng/rng.h"
#include "utils.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

/**
 * Calculate deterministic drift component of the state update
 */
int drift(runparams *run_params, imu *imu, vehicle *vehicle, gnss *gnss,
          grav *true_grav, grav *est_grav, atm_cond *true_atm_cond,
          atm_cond *est_atm_cond, state *true_state, state *est_state,
          state *true_state_drift, state *est_state_drift) {

  // Get thrust acceleration
  cartvec a_thrust;
  if (run_params->perfect_boost) {
    a_thrust = get_thrust_acc(true_state, vehicle, run_params, true_grav);
  } else {
    a_thrust = get_thrust_acc(est_state, vehicle, run_params, est_grav);
  }
  // If Lambert Guidance fails, quickly exit
  if (isnan(a_thrust.x)) {
    return 0;
  }
  // Get time derivative of available lift acceleration (same for true &
  // estimated states)
  cartvec d_a_lift_avail_dt = get_a_lift_avail_jerk(
      true_state, est_state, run_params, vehicle, est_atm_cond);

  state *states[2] = {true_state, est_state};
  state *state_derivs[2] = {true_state_drift, est_state_drift};
  grav *grav_models[2] = {true_grav, est_grav};
  atm_cond *atm_conds[2] = {true_atm_cond, est_atm_cond};
  cartvec a_gravs[2];
  cartvec a_total_true;
  for (int i = 0; i < 2; i++) {
    // Calculate total acceleration
    cartvec a_total;
    cartvec d_a_lift_dt = zeros();
    a_gravs[i] = get_gravity_acc(grav_models[i], states[i]);

    // If using INS navigation, then the estimated state does not need to
    // calculate every force individually
    if (run_params->ins_nav == 1 && i == 1) {
      a_total = imu_measurement(imu, true_state, est_state, a_total_true,
                                a_gravs[0], a_gravs[1]);
    } else {
      cartvec a_drag =
          get_drag_acc(run_params, vehicle, atm_conds[i], states[i]);
      a_total = add(add(add(a_thrust, a_drag), states[i]->a_lift), a_gravs[i]);

      // If realistic maneuverable RV, use proportional navigation during
      // reentry
      if (run_params->rv_maneuv == 1 && is_reentry(states[i])) {
        // Get lift jerk
        d_a_lift_dt =
            get_a_lift_jerk(states[i], run_params, vehicle, atm_conds[i]);
      }
    }
    // Keep track of true total acceleration for estimated state's accelerometer
    // measurement
    if (run_params->ins_nav == 1 && i == 0) {
      a_total_true = a_total;
    }

    // Set state derivatives
    state_derivs[i]->t = 1;
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

/**
 * Advance the state by one Euler-Maruyama step. Note, it is not strictly EM
 * because the position is updated using the velocity and the acceleration.
 *
 * @param state Pointer to state to be updated in place
 * @param time_step Integration time step in seconds
 */
int euler_maruyama_step(runparams *run_params, imu *imu, vehicle *vehicle,
                        gnss *gnss, grav *true_grav, grav *est_grav,
                        atm_cond *true_atm_cond, atm_cond *est_atm_cond,
                        state *true_state, state *est_state, double time_step) {
  // GNSS Measurement
  if ((run_params->gnss_nav == 1) &&
      (get_altitude(true_state->position) > 100e3)) {
    gnss_measurement(gnss, true_state, est_state);
  }

  // Each of the drift/diffusion states contains the derivative (wrt to time or
  // weiner process) In other words, true_state_drift.position is velocity,
  // true_state_drift.velocity is acceleration ...
  state true_state_drift = {0};
  state est_state_drift = {0};
  state est_state_diffusion = {0};
  int success = drift(run_params, imu, vehicle, gnss, true_grav, est_grav,
                      true_atm_cond, est_atm_cond, true_state, est_state,
                      &true_state_drift, &est_state_drift);
  if (!success) {
    return 0;
  }
  diffusion(imu, &est_state_diffusion);

  *true_state =
      add_state(*true_state, smultiply_state(true_state_drift, time_step));
  *est_state =
      add_state(*est_state, smultiply_state(est_state_drift, time_step));

  // Additional position update using 1/2 dt acceleration
  true_state->position =
      add(true_state->position,
          smultiply(true_state_drift.velocity, 0.5 * time_step * time_step));
  est_state->position =
      add(est_state->position,
          smultiply(est_state_drift.velocity, 0.5 * time_step * time_step));

  anglevec dW = smultiply_angle(gaussian_anglevec(), sqrt(time_step));
  est_state->gyro_error =
      add_anglevec(est_state->gyro_error,
                   multiply_anglevec(est_state_diffusion.gyro_error, dW));
  return 1;
}

#endif