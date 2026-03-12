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

state sra3_H(state drift_evals[3], state diffusion_evals[3], state Y, int i,
             anglevec I0, double time_step) {
  const double A0[3][3] = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.25, 0.25, 0.0}};
  const double B0[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {1.0, 0.5, 0.0}};

  state H = Y;
  for (int j = 0; j < i; j++) {
    H = add_state(H, smultiply_state(drift_evals[j], A0[i][j] * time_step));

    state diffusion_update = {0};
    diffusion_update.gyro_error.lat =
        B0[i][j] * diffusion_evals[j].gyro_error.lat * I0.lat;
    diffusion_update.gyro_error.lon =
        B0[i][j] * diffusion_evals[j].gyro_error.lon * I0.lon;
    H = add_state(H, diffusion_update);
  }

  return H;
}

/**
 * Advance the state by one Euler-Maruyama step. Note, it is not strictly EM
 * because the position is updated using the velocity and the acceleration.
 *
 * @param state Pointer to state to be updated in place
 * @param time_step Integration time step in seconds
 */
int euler_maruyama_step(runparams *run_params, imu *imu, vehicle *vehicle,
                        grav *true_grav, grav *est_grav,
                        atm_cond *true_atm_cond, atm_cond *est_atm_cond,
                        state *true_state, state *est_state, double *true_t,
                        double *est_t, double time_step) {

  // Each of the drift/diffusion states contains the derivative (wrt to time or
  // weiner process) In other words, true_state_drift.position is velocity,
  // true_state_drift.velocity is acceleration ...
  state true_state_drift = {0};
  state est_state_drift = {0};
  state est_state_diffusion = {0};
  int success = drift(run_params, imu, vehicle, true_grav, est_grav,
                      true_atm_cond, est_atm_cond, true_state, est_state,
                      *true_t, *est_t, &true_state_drift, &est_state_drift);
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

  *true_t += time_step;
  *est_t += time_step;
  return 1;
}

int sra3_step(runparams *run_params, imu *imu, vehicle *vehicle,
              grav *true_grav, grav *est_grav, atm_cond *true_atm_cond,
              atm_cond *est_atm_cond, state *true_state, state *est_state,
              double *true_t, double *est_t, double time_step) {

  const int num_stages = 3;
  const double c0[3] = {0.0, 1.0, 0.5};
  const double alpha[3] = {1.0 / 6.0, 1.0 / 6.0, 2.0 / 3.0};
  const double beta1[3] = {1.0, 0.0, 0.0};
  const double beta2[3] = {1.0, -1.0, 0.0};

  state true_state_initial = *true_state;
  state est_state_initial = *est_state;

  state true_state_drift_eval[3] = {0};
  state true_state_diffusion_eval[3] = {0};
  state est_state_drift_eval[3] = {0};
  state est_state_diffusion_eval[3] = {0};

  state true_state_update = {0};
  state est_state_drift_update = {0};
  state est_state_diffusion_update = {0};

  anglevec dW = smultiply_angle(gaussian_anglevec(), sqrt(time_step));
  anglevec zeta = smultiply_angle(gaussian_anglevec(), sqrt(time_step));
  anglevec I0;
  I0.lat = 0.5 * (dW.lat + (1.0 / sqrt(3.0)) * zeta.lat);
  I0.lon = 0.5 * (dW.lon + (1.0 / sqrt(3.0)) * zeta.lon);

  for (int i = 0; i < num_stages; i++) {
    diffusion(imu, &est_state_diffusion_eval[i]);
  }

  for (int i = 0; i < num_stages; i++) {
    state H_true = sra3_H(true_state_drift_eval, true_state_diffusion_eval,
                          true_state_initial, i, I0, time_step);
    state H_est = sra3_H(est_state_drift_eval, est_state_diffusion_eval,
                         est_state_initial, i, I0, time_step);

    state true_drift = {0};
    state est_drift = {0};
    int success =
        drift(run_params, imu, vehicle, true_grav, est_grav, true_atm_cond,
              est_atm_cond, &H_true, &H_est, *true_t + c0[i] * time_step,
              *est_t + c0[i] * time_step, &true_drift, &est_drift);
    if (!success) {
      return 0;
    }

    true_state_drift_eval[i] = true_drift;
    est_state_drift_eval[i] = est_drift;

    true_state_update =
        add_state(true_state_update, smultiply_state(true_state_drift_eval[i],
                                                     alpha[i] * time_step));

    est_state_drift_update = add_state(
        est_state_drift_update,
        smultiply_state(est_state_drift_eval[i], alpha[i] * time_step));
  }

  for (int i = 0; i < num_stages; i++) {
    est_state_diffusion_update.gyro_error.lat +=
        (beta1[i] * dW.lat + beta2[i] * I0.lat) *
        est_state_diffusion_eval[i].gyro_error.lat;
    est_state_diffusion_update.gyro_error.lon +=
        (beta1[i] * dW.lon + beta2[i] * I0.lon) *
        est_state_diffusion_eval[i].gyro_error.lon;
  }

  *true_state = add_state(true_state_initial, true_state_update);
  *est_state = add_state(add_state(est_state_initial, est_state_drift_update),
                         est_state_diffusion_update);
  *true_t += time_step;
  *est_t += time_step;
  return 1;
}

#endif