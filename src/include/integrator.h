#ifndef INTEGRATOR_H
#define INTEGRATOR_H

#include "derivatives.h"
#include "rng/rng.h"
#include "utils/utils.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

quaternion align_roll_axis_with_velocity(cartvec velocity) {
  // Normalize velocity
  double v_norm_val = norm(velocity);
  if (v_norm_val < 1e-12) {
    return identity_quaternion(); // Default orientation
  }
  cartvec v_normalized = smultiply(velocity, 1.0 / v_norm_val);

  // Create quaternion that rotates the roll axis [0,0,-1] to v_normalized
  cartvec from = {0.0, 0.0, -1.0};

  // Rotation axis: from × to
  cartvec rot_axis = cross(from, v_normalized);
  double axis_norm = norm(rot_axis);

  if (axis_norm < 1e-12) {
    // Aligned or opposite
    double dot_prod = dot(from, v_normalized);
    if (dot_prod > 0) {
      return identity_quaternion();
    } else {
      // 180° rotation around Y-axis
      return (quaternion){0.0, 0.0, 1.0, 0.0};
    }
  }

  rot_axis = smultiply(rot_axis, 1.0 / axis_norm);
  double angle = acos(dot(from, v_normalized));

  // Quaternion from axis-angle
  double half_angle = angle / 2.0;
  return (quaternion){cos(half_angle), sin(half_angle) * rot_axis.x,
                      sin(half_angle) * rot_axis.y,
                      sin(half_angle) * rot_axis.z};
}

/**
 * Integrate body-to-ECI quaternion using the state's incremental orientation
 * angle change over the current step.
 *
 * Standard first-order kinematics:
 * q_{k+1} = normalize(q_k \otimes \delta q).
 */
static inline quaternion integrate_quaternion_step(state current_state) {
  cartvec delta_angle_B = {current_state.orientation_angle_change.yaw,
                           current_state.orientation_angle_change.pitch, 0.0};
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

state sra3_H(state drift_evals[3], state diffusion_evals[3], state Y, int i,
             anglevec I0, double time_step) {
  const double A0[3][3] = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.25, 0.25, 0.0}};
  const double B0[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {1.0, 0.5, 0.0}};

  state H = Y;
  for (int j = 0; j < i; j++) {
    H = add_state(H, smultiply_state(drift_evals[j], A0[i][j] * time_step));

    state diffusion_update = {0};
    anglevec stochastic_scale = smultiply_angle(I0, B0[i][j]);
    diffusion_update.gyro_error =
        multiply_anglevec(diffusion_evals[j].gyro_error, stochastic_scale);
    diffusion_update.orientation_angle_change = multiply_anglevec(
        diffusion_evals[j].orientation_angle_change, stochastic_scale);
    H = add_state(H, diffusion_update);
  }

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
  diffusion_fn(imu, &true_state_diffusion, &est_state_diffusion);

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
  anglevec dW = smultiply_angle(gaussian_anglevec(), sqrt(time_step));
  true_state->gyro_error =
      add_anglevec(true_state->gyro_error,
                   multiply_anglevec(true_state_diffusion.gyro_error, dW));

  true_state->orientation_angle_change = add_anglevec(
      smultiply_angle(true_state_drift.orientation_angle_change, time_step),
      multiply_anglevec(true_state_diffusion.orientation_angle_change, dW));
  est_state->orientation_angle_change = add_anglevec(
      smultiply_angle(est_state_drift.orientation_angle_change, time_step),
      multiply_anglevec(est_state_diffusion.orientation_angle_change, dW));

  true_state->q_EB = integrate_quaternion_step(*true_state);
  est_state->q_EB = integrate_quaternion_step(*est_state);
  // Convert resolution from degrees to radians
  double resolution = run_params->actuator_resolution * M_PI / 180;
  double max_extent = run_params->max_deflection_angle * M_PI / 180;
  double clipped_delta_1 =
      clip(fmod(true_state->delta_1, 2 * M_PI), -max_extent, max_extent);
  double clipped_delta_2 =
      clip(fmod(true_state->delta_2, 2 * M_PI), -max_extent, max_extent);
  // TODO add actuator resolution back in
  true_state->delta_1 = clipped_delta_1;
  true_state->delta_2 = clipped_delta_2;
  // true_state->delta_1 = round(clipped_delta_1 / resolution) * resolution;
  // true_state->delta_2 = round(clipped_delta_2 / resolution) * resolution;
  double clipped_est_delta_1 =
      clip(fmod(est_state->delta_1, 2 * M_PI), -max_extent, max_extent);
  double clipped_est_delta_2 =
      clip(fmod(est_state->delta_2, 2 * M_PI), -max_extent, max_extent);
  est_state->delta_1 = clipped_est_delta_1;
  est_state->delta_2 = clipped_est_delta_2;
  // est_state->delta_1 = round(clipped_est_delta_1 / resolution) * resolution;
  // est_state->delta_2 = round(clipped_est_delta_2 / resolution) * resolution;

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
  state est_state_diffusion_update = {0};

  // Preserve controller memory updated inside drift() across SRA3 stage copies.
  double next_prev_a_cmd_1 = est_state_initial.prev_a_cmd_1;
  double next_prev_a_cmd_2 = est_state_initial.prev_a_cmd_2;
  double next_prev_delta_1 = est_state_initial.prev_delta_1;
  double next_prev_delta_2 = est_state_initial.prev_delta_2;

  anglevec dW = smultiply_angle(gaussian_anglevec(), sqrt(time_step));
  anglevec zeta = smultiply_angle(gaussian_anglevec(), sqrt(time_step));
  anglevec I0;
  I0.pitch = 0.5 * (dW.pitch + (1.0 / sqrt(3.0)) * zeta.pitch);
  I0.yaw = 0.5 * (dW.yaw + (1.0 / sqrt(3.0)) * zeta.yaw);

  for (int i = 0; i < num_stages; i++) {
    diffusion_fn(imu, &true_state_diffusion_eval[i],
                 &est_state_diffusion_eval[i]);
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

    next_prev_a_cmd_1 = H_est.prev_a_cmd_1;
    next_prev_a_cmd_2 = H_est.prev_a_cmd_2;
    next_prev_delta_1 = H_est.prev_delta_1;
    next_prev_delta_2 = H_est.prev_delta_2;

    true_state_drift_update = add_state(
        true_state_drift_update,
        smultiply_state(true_state_drift_eval[i], alpha[i] * time_step));

    est_state_drift_update = add_state(
        est_state_drift_update,
        smultiply_state(est_state_drift_eval[i], alpha[i] * time_step));
  }

  for (int i = 0; i < num_stages; i++) {
    anglevec stochastic_gain = add_anglevec(smultiply_angle(dW, beta1[i]),
                                            smultiply_angle(I0, beta2[i]));

    true_state_diffusion_update.gyro_error =
        add_anglevec(true_state_diffusion_update.gyro_error,
                     multiply_anglevec(true_state_diffusion_eval[i].gyro_error,
                                       stochastic_gain));

    true_state_diffusion_update.orientation_angle_change = add_anglevec(
        true_state_diffusion_update.orientation_angle_change,
        multiply_anglevec(true_state_diffusion_eval[i].orientation_angle_change,
                          stochastic_gain));

    est_state_diffusion_update.orientation_angle_change = add_anglevec(
        est_state_diffusion_update.orientation_angle_change,
        multiply_anglevec(est_state_diffusion_eval[i].orientation_angle_change,
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

  est_state->prev_a_cmd_1 = next_prev_a_cmd_1;
  est_state->prev_a_cmd_2 = next_prev_a_cmd_2;
  est_state->prev_delta_1 = next_prev_delta_1;
  est_state->prev_delta_2 = next_prev_delta_2;

  // Convert resolution from degrees to radians
  double resolution = run_params->actuator_resolution * M_PI / 180;
  double max_extent = run_params->max_deflection_angle * M_PI / 180;
  double clipped_delta_1 =
      clip(fmod(true_state->delta_1, 2 * M_PI), -max_extent, max_extent);
  double clipped_delta_2 =
      clip(fmod(true_state->delta_2, 2 * M_PI), -max_extent, max_extent);
  // TODO add actuator resolution back in
  true_state->delta_1 = clipped_delta_1;
  true_state->delta_2 = clipped_delta_2;
  // true_state->delta_1 = round(clipped_delta_1 / resolution) * resolution;
  // true_state->delta_2 = round(clipped_delta_2 / resolution) * resolution;
  double clipped_est_delta_1 =
      clip(fmod(est_state->delta_1, 2 * M_PI), -max_extent, max_extent);
  double clipped_est_delta_2 =
      clip(fmod(est_state->delta_2, 2 * M_PI), -max_extent, max_extent);
  est_state->delta_1 = clipped_est_delta_1;
  est_state->delta_2 = clipped_est_delta_2;
  // est_state->delta_1 = round(clipped_est_delta_1 / resolution) * resolution;
  // est_state->delta_2 = round(clipped_est_delta_2 / resolution) * resolution;

  true_state->q_EB = integrate_quaternion_step(*true_state);
  est_state->q_EB = integrate_quaternion_step(*est_state);
  // est_state->q_EB = integrate_quaternion_step(
  //     q_est_prev, true_state->angular_vel_B, time_step);

  *true_t += time_step;
  *est_t += time_step;
  return 1;
}

#endif