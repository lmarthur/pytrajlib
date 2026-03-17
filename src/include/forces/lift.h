#ifndef LIFT_H
#define LIFT_H

#include "../math/linalg.h"
#include "../models/atmosphere.h"
#include "../models/state.h"
#include "../models/vehicle.h"
#include "../utils/body_frame.h"
#include "../utils/utils.h"

/**
 * Calculate the time constant of the reentry vehicle based on the current
 * state.
 *
 * The time constant $\tau$ is calculated from $I$, the moment of inertia around
 * the vehicle's y-axis, $C_{m_{\alpha}}$, the pitching moment coefficient
 * derivative (per radian), $A$, the reentry vehicle reference area, $\rho$, the
 * atmospheric density, and $v$, the speed of the vehicle:
 * $$
 * \tau = \sqrt{-\frac{2I}{C_{m_{\alpha}} A r_e \rho v^2}}
 * $$
 *
 * @param current_state pointer to current vehicle state
 * @param atm_cond pointer to atmospheric conditions
 * @param vehicle pointer to vehicle model
 * @return time constant in seconds
 */
double rv_time_constant(state *current_state, atm_cond *atm_cond,
                        vehicle *vehicle) {
  double velocity = norm(current_state->velocity);

  // Calculate the time constant
  double time_constant =
      sqrt(-2 * vehicle->rv.Iyy /
           (vehicle->rv.c_m_alpha * vehicle->rv.rv_area * atm_cond->density *
            velocity * velocity * vehicle->rv.rv_length));

  return time_constant;
}

/**
 * Get commanded acceleration $\vec a_n$.
 * The proportional navigation guidance law that flies the vehicle towards the
 target
 * is given by
 * $$
 * \vec a_n = -N \vec v_r \times \Omega,
 * $$
 * where $N$ is the navigation gain,
 * $\vec v_r$ is the relative velocity between the vehicle and the aimpoint,
 known
 * as the closing velocity, and $\Omega$ is the line-of-sight rotation vector:
 * $$
 * \Omega = \frac{\vec r \times \vec v_r}{\vec r \cdot \vec{r}}
 * $$
 * with $\vec r$ as the displacement between the vehicle and the aimpoint.

 * The target is assumed to be stationary, so $\vec v_r$ is the estimated
 vehicle velocity.
 * @param estimated_state the vehicle's internal estimated state
 * @param run_params the run parameters struct
 * @return commanded acceleration in the inertial-frame Cartesian basis (m/s^2)
 */
cartvec prop_nav(state *estimated_state, runparams *run_params) {
  cartvec aimpoint = {run_params->x_aim, run_params->y_aim, run_params->z_aim};
  // Calculate the relative position vector to the target
  cartvec r_target = subtract(aimpoint, estimated_state->position);

  // Calculate the relative velocity vector to the (stationary) target
  cartvec v_rel = smultiply(estimated_state->velocity, -1.0);

  // Get the rotation vector by taking the cross product of the relative
  // position and velocity vectors and dividing by |r|^2
  double r_dot_r = dot(r_target, r_target);
  cartvec cross_product = cross(r_target, v_rel);
  cartvec rot = sdivide(cross_product, r_dot_r);

  // Calculate the acceleration command by taking the cross product of the
  // relative velocity and the rotation vector, scaled by the navigation gain
  cartvec cross_v_rot = cross(v_rel, rot);
  cartvec a_command = smultiply(cross_v_rot, run_params->nav_gain);

  return a_command;
}

/**
 * Compute lift-acceleration quantization from actuator angular resolution.
 *
 * Based on ISO 3408-3 grade 5, we assume the actuator has a ±10 degree range
 * with a 0.01 degree resolution.
 *
 * @param run_params run parameters struct
 * @param vehicle vehicle struct
 * @return Lift acceleration resolution in m/s^2
 */
double get_acc_resolution(runparams *run_params, vehicle *vehicle) {
  double max_a_exec = get_max_a_exec(run_params, vehicle);
  double deflection_max =
      M_PI / 18; // maximum flap deflection in radians (10 degrees)
  double actuator_resolution =
      0.01 * M_PI / 180; // 0.01 degree resolution in radians

  // Acceleration resolution is proportional to the angular resolution
  double acc_resolution = max_a_exec * actuator_resolution / deflection_max;

  return acc_resolution;
}

/**
 * Calculate the maximum jerk
 *
 * @param run_params run parameters struct
 * @param vehicle vehicle struct
 * @return Maximum jerk in m/s^3
 */
double get_jerk_max(runparams *run_params, vehicle *vehicle) {
  double max_a_exec = get_max_a_exec(run_params, vehicle);
  double deflection_time =
      run_params->deflection_time * run_params->gearing_ratio;
  double jerk_max = max_a_exec / deflection_time;

  return jerk_max;
}

/**
 * Project a vector into the lift plane, clip, then reconstruct in Cartesian.
 *
 * Given orthonormal lift-plane basis vectors $(\mathbf e_2,\mathbf e_3)$,
 * the vector components are clipped independently to
 * $[-\text{max\_val},\text{max\_val}]$ and mapped back.
 *
 * @param e2 Lift-plane basis vector
 * @param e3 Lift-plane basis vector
 * @param arr Vector to project and clip
 * @param max_val Absolute clip limit in projected coordinates
 * @return Projected and clipped vector in Cartesian coordinates
 */
cartvec project_and_clip(cartvec e2, cartvec e3, cartvec arr, double max_val) {
  // Project onto e2 and e3
  double arr_e2 = dot(arr, e2);
  double arr_e3 = dot(arr, e3);

  // Clip to max_val
  arr_e2 = clip(arr_e2, -max_val, max_val);
  arr_e3 = clip(arr_e3, -max_val, max_val);

  // Project back to Cartesian basis
  cartvec result = add(smultiply(e2, arr_e2), smultiply(e3, arr_e3));
  return result;
}

/**
 * Determine whether the vehicle is in reentry conditions.
 *
 * Reentry is detected when altitude is below 100 km and velocity is directed
 * generally toward Earth center.
 *
 * @param state Pointer to current state
 * @param t Current simulation time in seconds
 * @return 1 if in reentry, else 0
 */
int is_reentry(state *state, double t) {
  // Check for small t to account for initial velocity error that might make the
  // vehicle appear to be below altitude 0 after a single step
  if (t < 10)
    return 0;
  double v_mag = norm(state->velocity);
  if (v_mag < 1e-6)
    return 0;
  double altitude = get_altitude(state->position);
  if (altitude >= 1e5)
    return 0;
  double cos_angle = dot(state->velocity, smultiply(state->position, -1)) /
                     (v_mag * norm(state->position));
  cos_angle =
      clip(cos_angle, -1.0, 1.0); // guard against floating-point overshoot
  double angle_v_grav = acos(cos_angle);
  return (angle_v_grav > 0) && (angle_v_grav < M_PI_2);
}

/**
 * Get the time derivative of the available lift acceleration.
 *
 * The available lift acceleration encodes the position of the control surfaces.
 * The control surfaces are assumed to instantaneously accelerate to
 * a fixed maximum angular velocity. The maximum angular velocity of the control
 * surfaces is equivalent to a maximum available jerk.
 *
 * To avoid oscillations, as the available acceleration approaches the commanded
 * acceleration, the jerk reduces from the maximum jerk to a jerk proportional
 * to the difference. When the difference is less than the actuator resolution,
 * the derivative is zero.
 *
 * Only valid during reentry phase.
 *
 * @param est_state pointer to the estimated state
 * @param run_params pointer to the run parameters struct
 * @param vehicle pointer to the vehicle struct
 * @param est_atm_cond pointer to the estimated atmospheric conditions
 * @param est_t current estimated flight time (seconds)
 * @return Time derivative of available lift acceleration (m/s^3)
 */
cartvec get_a_lift_avail_jerk(state *est_state, runparams *run_params,
                              vehicle *vehicle, atm_cond *est_atm_cond,
                              double est_t) {
  // Determine if vehicle is in reentry phase
  if (!is_reentry(est_state, est_t)) {
    return zeros();
  }

  // Calculate maximum parameters
  double max_a_exec = get_max_a_exec(run_params, vehicle);
  double jerk_max = get_jerk_max(run_params, vehicle);

  // Commanded acceleration is based on the aimpoint and the estimated state's
  // position and velocity
  cartvec a_command = prop_nav(est_state, run_params);

  // Change available lift at a fixed rate unless the difference between
  // current and target is small. For small differences, let the difference
  // reduce exponentially to keep the derivative continuous.
  cartvec a_avail_err = subtract(a_command, est_state->a_lift_avail);

  // Apply proportional gain and clip to jerk limits
  cartvec d_a_lift_avail_dt = smultiply(a_avail_err, run_params->flap_gain);
  d_a_lift_avail_dt.x = clip(d_a_lift_avail_dt.x, -jerk_max, jerk_max);
  d_a_lift_avail_dt.y = clip(d_a_lift_avail_dt.y, -jerk_max, jerk_max);
  d_a_lift_avail_dt.z = clip(d_a_lift_avail_dt.z, -jerk_max, jerk_max);

  return d_a_lift_avail_dt;
}

/**
 * Compute lift jerk from first-order lag toward available lift.
 *
 * The lift acceleration approaches the available lift acceleration
 * exponentially based on the time constant:
 * $$
 * a(t) = a_\text{avail} (1 - e^{-t/\tau})
 * $$
 *
 * The lift jerk is zero outside reentry or when a valid lift basis is
 * unavailable.
 *
 * The proportional navigation commands may produce a commanded lift
 * acceleration with a component in the direction of the velocity, but
 * this function will only attempt to produce lift acceleration in the plane
 * perpendicular to the relative velocity (e_2, e_3 directions).
 *
 * @param current_state Pointer to current state
 * @param run_params Pointer to run configuration parameters
 * @param vehicle Pointer to vehicle model
 * @param atm_cond Pointer to atmospheric conditions
 * @param t Current simulation time in seconds
 * @return Time derivative of lift acceleration (jerk) in m/s^3
 */
cartvec get_a_lift_jerk(state *true_state, runparams *run_params,
                        vehicle *vehicle, atm_cond *atm_cond, double t) {

  // Determine if vehicle is in reentry phase
  if (!is_reentry(true_state, t)) {
    return zeros();
  }

  // Calculate maximum parameters
  double max_a_exec = get_max_a_exec(run_params, vehicle);

  // Get time constant to simulate pressure build-up
  double time_constant = rv_time_constant(true_state, atm_cond, vehicle);

  // Quantize available lift to the resolution of the actuator
  double acc_resolution = get_acc_resolution(run_params, vehicle);
  cartvec a_lift_avail = true_state->a_lift_avail;
  double mag_a_lift_avail = norm(a_lift_avail);

  // Limit available lift magnitudes to quantized values
  if (mag_a_lift_avail > 0) {
    double quantized_mag =
        round(mag_a_lift_avail / acc_resolution) * acc_resolution;
    a_lift_avail = smultiply(a_lift_avail, quantized_mag / mag_a_lift_avail);
  } else {
    a_lift_avail = zeros();
  }

  // Get the lift basis vectors. Don't include gyro_error because the true lift
  // acceleration does not depend on the internal state's estimated attitude
  cartvec e1, e2, e3;
  int valid_basis = get_body_frame(true_state, atm_cond, &e1, &e2, &e3, 0);

  if (!valid_basis) {
    return zeros();
  }

  // The lift available to be generated by the current flap positions depends
  // on the attitude of the vehicle, so the available lift should be
  // projected onto the lift basis and clipped to the maximum achievable lift
  cartvec a_lift_avail_projected =
      project_and_clip(e2, e3, a_lift_avail, max_a_exec);

  // Calculate the jerk
  cartvec d_a_lift_dt = sdivide(
      subtract(a_lift_avail_projected, true_state->a_lift), time_constant);

  return d_a_lift_dt;
}

#endif