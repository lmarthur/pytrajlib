#ifndef INTEGRATOR_H
#define INTEGRATOR_H

#include "derivatives.h"
#include "rng/rng.h"
#include "utils/utils.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

/**
The quaternion update step is implemented as a function of the incremental
angular change. The incremental angular change is
calculated with the SRA3 algorithm and takes a rotating angular rate into
account. The quaternion rotation increment with
$\theta = |\boldsymbol  \theta_B|$ is
\begin{equation}
  \Delta \mathbf q = \begin{bmatrix}
    \cos(\theta/2) \\
    \sin(\theta/2) \hat{\boldsymbol{\mathbf\theta}}\_B
  \end{bmatrix},
\end{equation}
so the update is
\begin{equation}
  \mathbf q_{EB,t+1} = \mathbf q_{EB,t} \otimes \Delta \mathbf q
\end{equation}
where the $\otimes$ operator is the Hamilton product.
 */
static inline quaternion integrate_quaternion_step(state current_state) {
  cartvec delta_angle_B = current_state.orientation_angle_change;
  double theta = norm(delta_angle_B);
  double half_theta = 0.5 * theta;

  double sinc_half_theta;
  if (fabs(half_theta) < 1e-8) {
    sinc_half_theta = 1.0;
  } else {
    sinc_half_theta = sin(half_theta) / half_theta;
  }

  double vec_scale = 0.5 * sinc_half_theta;
  quaternion delta_q = {cos(half_theta), vec_scale * delta_angle_B.x,
                        vec_scale * delta_angle_B.y,
                        vec_scale * delta_angle_B.z};

  quaternion q_next = qmultiply(current_state.q_EB, delta_q);

  double q_norm = qnorm(q_next);
  if (q_norm < 1e-12) {
    return identity_quaternion();
  }

  return qsmultiply(q_next, 1.0 / q_norm);
}

/**
 * Following Bortz (1971) and Titterton & Weston (2004) we compute the time
 * derivative of the incremental orientation change and take into account the
 * changing orientation of the vehicle.
 */
static inline cartvec rotation_vector_rate(cartvec phi, cartvec omega) {
  cartvec phi_cross_omega = cross(phi, omega);
  cartvec phi_cross_phi_cross_omega = cross(phi, phi_cross_omega);
  return add(add(omega, smultiply(phi_cross_omega, 0.5)),
             sdivide(phi_cross_phi_cross_omega, 12.0));
}

state sra3_H(state drift_evals[3], state diffusion_evals[3], state Y, int i,
             cartvec I0, double time_step) {
  const double A0[3][3] = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.25, 0.25, 0.0}};
  const double B0[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {1.0, 0.5, 0.0}};

  state H = Y;

  // Clear the previous step's angle increment
  H.orientation_angle_change = zeros();

  for (int j = 0; j < i; j++) {
    H = add_state(H, smultiply_state(drift_evals[j], A0[i][j] * time_step));

    state diffusion_update = {0};
    cartvec stochastic_scale = smultiply(I0, B0[i][j]);

    diffusion_update.orientation_angle_change = multiply_cartvec(
        diffusion_evals[j].orientation_angle_change, stochastic_scale);

    H = add_state(H, diffusion_update);
  }

  // Rotate the stage attitude by the angle increment accumulated above
  H.q_EB = integrate_quaternion_step(H);

  return H;
}

/**
 * Advance the state by one Euler-Maruyama step. Note, it is not strictly EM
 * because the position is updated using the velocity and the acceleration.
 *
 * This integrator is currently not used in the trajectory simulation path.
 *
 * @param run_params Pointer to run configuration parameters.
 * @param imu Pointer to IMU model.
 * @param vehicle Pointer to vehicle model.
 * @param true_grav Pointer to true gravity model.
 * @param est_grav Pointer to estimated gravity model.
 * @param true_atm_cond Pointer to true atmospheric conditions.
 * @param est_atm_cond Pointer to estimated atmospheric conditions.
 * @param true_state Pointer to true state, updated in place.
 * @param est_state Pointer to estimated state, updated in place.
 * @param true_t Pointer to true simulation time, incremented by `time_step`.
 * @param est_t Pointer to estimated simulation time, incremented by
 *              `time_step`.
 * @param time_step Integration time step in seconds.
 * @param drift_fn Drift callback used to compute deterministic derivatives.
 * @param diffusion_fn Diffusion callback used to compute stochastic
 *                     derivatives.
 * @return 1 on success, 0 if the drift callback reports failure.
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
  state est_state_diffusion = {0};
  int success = drift_fn(run_params, imu, vehicle, true_grav, est_grav,
                         true_atm_cond, est_atm_cond, true_state, est_state,
                         *true_t, *est_t, &true_state_drift, &est_state_drift);
  if (!success) {
    return 0;
  }
  diffusion_fn(imu, &est_state_diffusion, &true_state_diffusion, run_params);

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

  // Only draw a single dW because all of the diffusion terms are related to the
  // gyroscope measurement
  cartvec dW = smultiply(gaussian_cartvec(), sqrt(time_step));
  true_state->orientation_angle_change =
      add(smultiply(true_state_drift.orientation_angle_change, time_step),
          multiply_cartvec(true_state_diffusion.orientation_angle_change, dW));
  est_state->orientation_angle_change =
      add(smultiply(est_state_drift.orientation_angle_change, time_step),
          multiply_cartvec(est_state_diffusion.orientation_angle_change, dW));

  true_state->q_EB = integrate_quaternion_step(*true_state);
  est_state->q_EB = integrate_quaternion_step(*est_state);

  *true_t += time_step;
  *est_t += time_step;
  return 1;
}

/**
Stochastic differential equations admit two standard interpretations, Itô and
Stratonovich, which correspond to left-endpoint and midpoint evaluation rules,
respectively. These generally yield different solutions but are equivalent for
stochastic differential equations with additive noise, as is the case in our
simulation, where the gyroscope noise is assumed to be constant.

We integrate the state using the Stochastic Runge-Kutta for Additive Noise
(SRA3) method, as described by Rössler (2010). SRA3 is of strong order 3 for
deterministic differential equations and strong order 1.5 for stochastic
differential equations. This is a significant improvement over the standard
Euler-Maruyama method, which is of strong order 1 for drift and diffusion with
additive noise, as discussed by Higham and Kloeden (2021), especially because
the deterministic components drive most of the dynamics. Higher-order numerical
integrators allow using larger time steps without sacrificing integration
accuracy.

Solutions to stochastic differential equations are probability distributions
over trajectories. We use a Monte Carlo approach to sample from both the initial
error terms and the solution to the stochastic differential equation.

 * @param run_params Pointer to run configuration parameters.
 * @param imu Pointer to IMU model.
 * @param vehicle Pointer to vehicle model.
 * @param true_grav Pointer to true gravity model.
 * @param est_grav Pointer to estimated gravity model.
 * @param true_atm_cond Pointer to true atmospheric conditions.
 * @param est_atm_cond Pointer to estimated atmospheric conditions.
 * @param true_state Pointer to true state, updated in place.
 * @param est_state Pointer to estimated state, updated in place.
 * @param true_t Pointer to true simulation time, incremented by `time_step`.
 * @param est_t Pointer to estimated simulation time, incremented by
 *              `time_step`.
 * @param time_step Integration time step in seconds.
 * @param drift_fn Drift callback used for deterministic stage evaluations.
 * @param diffusion_fn Diffusion callback used for additive-noise stage
 *                     evaluations.
 * @return 1 on success, 0 if any drift stage reports failure.
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
  const double beta2[3] = {-1.0, 1.0,
                           0.0}; // Modified to match StochasticDiffEq.jl

  state true_state_initial = *true_state;
  state est_state_initial = *est_state;

  state true_state_drift_eval[3] = {0};
  state true_state_diffusion_eval[3] = {0};
  state est_state_drift_eval[3] = {0};
  state est_state_diffusion_eval[3] = {0};

  state true_state_drift_update = {0};
  state true_state_diffusion_update = {0};
  state est_state_drift_update = {0};
  state est_state_diffusion_update = {0};

  cartvec dW = smultiply(gaussian_cartvec(), sqrt(time_step));
  cartvec zeta = smultiply(gaussian_cartvec(), sqrt(time_step));
  cartvec I0;
  I0 = smultiply(add(dW, smultiply(zeta, 1.0 / sqrt(3.0))), 0.5);

  for (int i = 0; i < num_stages; i++) {
    diffusion_fn(imu, &est_state_diffusion_eval[i],
                 &true_state_diffusion_eval[i], run_params);
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

    // The current step's rotation rate depends on the previous accumulated
    // orientation angle change
    true_drift.orientation_angle_change = rotation_vector_rate(
        H_true.orientation_angle_change, true_drift.orientation_angle_change);
    est_drift.orientation_angle_change = rotation_vector_rate(
        H_est.orientation_angle_change, est_drift.orientation_angle_change);

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
    cartvec stochastic_gain =
        add(smultiply(dW, beta1[i]), smultiply(I0, beta2[i]));

    true_state_diffusion_update.orientation_angle_change = add(
        true_state_diffusion_update.orientation_angle_change,
        multiply_cartvec(true_state_diffusion_eval[i].orientation_angle_change,
                         stochastic_gain));

    est_state_diffusion_update.orientation_angle_change = add(
        est_state_diffusion_update.orientation_angle_change,
        multiply_cartvec(est_state_diffusion_eval[i].orientation_angle_change,
                         stochastic_gain));
  }

  state true_state_total_update =
      add_state(true_state_drift_update, true_state_diffusion_update);
  *true_state = add_state(true_state_initial, true_state_total_update);
  true_state->orientation_angle_change =
      true_state_total_update.orientation_angle_change;

  state est_state_total_update =
      add_state(est_state_drift_update, est_state_diffusion_update);
  *est_state = add_state(est_state_initial, est_state_total_update);
  est_state->orientation_angle_change =
      est_state_total_update.orientation_angle_change;

  true_state->q_EB = integrate_quaternion_step(*true_state);
  est_state->q_EB = integrate_quaternion_step(*est_state);

  *true_t += time_step;
  *est_t += time_step;
  return 1;
}

#endif