#ifndef INTEGRATOR_H
#define INTEGRATOR_H

#include "derivatives.h"
#include "rng/rng.h"
#include "utils/utils.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

state sra3_H(state drift_evals[3], state diffusion_evals[3], state Y, int i,
             anglevec I0, double time_step) {
  const double A0[3][3] = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.25, 0.25, 0.0}};
  const double B0[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {1.0, 0.5, 0.0}};

  state H = Y;
  for (int j = 0; j < i; j++) {
    H = add_state(H, smultiply_state(drift_evals[j], A0[i][j] * time_step));

    state diffusion_update = {0};
    diffusion_update.gyro_error.pitch =
        B0[i][j] * diffusion_evals[j].gyro_error.pitch * I0.pitch;
    diffusion_update.gyro_error.yaw =
        B0[i][j] * diffusion_evals[j].gyro_error.yaw * I0.yaw;
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
                        double *est_t, double time_step, drift_func drift_fn,
                        diffusion_func diffusion_fn) {

  // Each of the drift/diffusion states contains the derivative (wrt to time or
  // weiner process) In other words, true_state_drift.position is velocity,
  // true_state_drift.velocity is acceleration ...
  state true_state_drift = {0};
  state est_state_drift = {0};
  state true_state_diffusion = {0};
  int success = drift_fn(run_params, imu, vehicle, true_grav, est_grav,
                         true_atm_cond, est_atm_cond, true_state, est_state,
                         *true_t, *est_t, &true_state_drift, &est_state_drift);
  if (!success) {
    return 0;
  }
  diffusion_fn(imu, &true_state_diffusion);

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
  true_state->gyro_error =
      add_anglevec(true_state->gyro_error,
                   multiply_anglevec(true_state_diffusion.gyro_error, dW));

  *true_t += time_step;
  *est_t += time_step;
  return 1;
}

/**
 * Advance the state using SRA3 (Stochastic Runge-Kutta for Additive Noise).
 *
 * References:
 *
 * >Rößler, A. (2010). Runge–Kutta Methods for the Strong Approximation of
 * Solutions of Stochastic Differential Equations. SIAM Journal on Numerical
 * Analysis, 48(3), 922–952. https://doi.org/10.1137/09076636X
 */
int sra3_step(runparams *run_params, imu *imu, vehicle *vehicle,
              grav *true_grav, grav *est_grav, atm_cond *true_atm_cond,
              atm_cond *est_atm_cond, state *true_state, state *est_state,
              double *true_t, double *est_t, double time_step,
              drift_func drift_fn, diffusion_func diffusion_fn) {

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

  state true_state_drift_update = {0};
  state true_state_diffusion_update = {0};
  state est_state_drift_update = {0};

  anglevec dW = smultiply_angle(gaussian_anglevec(), sqrt(time_step));
  anglevec zeta = smultiply_angle(gaussian_anglevec(), sqrt(time_step));
  anglevec I0;
  I0.pitch = 0.5 * (dW.pitch + (1.0 / sqrt(3.0)) * zeta.pitch);
  I0.yaw = 0.5 * (dW.yaw + (1.0 / sqrt(3.0)) * zeta.yaw);

  for (int i = 0; i < num_stages; i++) {
    diffusion_fn(imu, &true_state_diffusion_eval[i]);
  }

  for (int i = 0; i < num_stages; i++) {
    state H_true = sra3_H(true_state_drift_eval, true_state_diffusion_eval,
                          true_state_initial, i, I0, time_step);
    state H_est = sra3_H(est_state_drift_eval, est_state_diffusion_eval,
                         est_state_initial, i, I0, time_step);

    state true_drift = {0};
    state est_drift = {0};
    int success =
        drift_fn(run_params, imu, vehicle, true_grav, est_grav, true_atm_cond,
                 est_atm_cond, &H_true, &H_est, *true_t + c0[i] * time_step,
                 *est_t + c0[i] * time_step, &true_drift, &est_drift);
    if (!success) {
      return 0;
    }

    true_state_drift_eval[i] = true_drift;
    est_state_drift_eval[i] = est_drift;

    true_state_drift_update = add_state(
        true_state_drift_update,
        smultiply_state(true_state_drift_eval[i], alpha[i] * time_step));

    est_state_drift_update = add_state(
        est_state_drift_update,
        smultiply_state(est_state_drift_eval[i], alpha[i] * time_step));
  }

  for (int i = 0; i < num_stages; i++) {
    true_state_diffusion_update.gyro_error.pitch +=
        (beta1[i] * dW.pitch + beta2[i] * I0.pitch) *
        true_state_diffusion_eval[i].gyro_error.pitch;
    true_state_diffusion_update.gyro_error.yaw +=
        (beta1[i] * dW.yaw + beta2[i] * I0.yaw) *
        true_state_diffusion_eval[i].gyro_error.yaw;
  }

  *true_state =
      add_state(add_state(true_state_initial, true_state_drift_update),
                true_state_diffusion_update);
  *est_state = add_state(est_state_initial, est_state_drift_update);
  *true_t += time_step;
  *est_t += time_step;
  return 1;
}

#endif