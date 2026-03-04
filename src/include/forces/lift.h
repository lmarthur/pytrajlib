#ifndef LIFT_H
#define LIFT_H

#include "../math/linalg.h"
#include "../models/atmosphere.h"
#include "../models/state.h"
#include "../models/vehicle.h"
#include "../utils.h"

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
 * Get commanded acceleration using proportional navigation guidance law.
 *
 * The proportional navigation guidance law that flies the vehicle towards the
 * target is given by a_n = -N * v_r × Ω, where N is the navigation gain, v_r is
 * the relative velocity (closing velocity), and Ω is the line-of-sight rotation
 * vector: Ω = (r × v_r) / (r · r), where r is the distance between the vehicle
 * and the aimpoint.
 *
 * The target is assumed to be stationary, so v_r is the negative of the
 * estimated vehicle velocity.
 *
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
 * Calculate the acceleration resolution based on actuator resolution.
 *
 * Based on ISO 3408-3 grade 5, we assume the actuator has a ±10 degree range
 * with a 0.01 degree resolution.
 *
 * @param run_params run parameters struct
 * @param vehicle vehicle struct
 * @return acceleration resolution in m/s^2
 */
double get_acc_resolution(runparams *run_params, vehicle *vehicle) {
  double max_a_exec = get_max_a_exec(run_params, vehicle);
  double deflection_max =
      M_PI / 6; // maximum flap deflection in radians (30 degrees)
  double actuator_resolution =
      0.01 * M_PI / 180; // 0.01 degree resolution in radians

  // Acceleration resolution is proportional to the angular resolution
  double acc_resolution = max_a_exec * actuator_resolution / deflection_max;

  return acc_resolution;
}

/**
 * Define a local coordinate system where
 *
 * - $\vec e_1$ points in the direction of relative velocity
 *
 * - $\vec e_2$ points in the direction of lift acceleration (or the global
 * z-axis if lift is zero)
 *
 * - $\vec e_3$ is orthogonal to both ($\vec e_3 = \vec e_1 \times \vec e_2$)
 *
 *
 * The basis is only successfully defined if
 *
 * 1. the relative velocity is not zero AND
 *
 * 2. inside the atmosphere (below 100km) AND
 *
 * 3. $|e_2| > 0$
 * @return 1 if successfully defined basis, 0 if unsuccessful
 */
int compute_lift_basis(state *current_state, atm_cond *atm_cond, cartvec *e_1,
                       cartvec *e_2, cartvec *e_3) {

  cartvec velocity = current_state->velocity;
  cartvec a_lift = current_state->a_lift;

  cartvec wind_vec = get_cart_wind(current_state, atm_cond);
  cartvec v_rel = subtract(velocity, wind_vec);
  double v_rel_mag = norm(v_rel);
  double altitude = get_altitude(current_state->position);
  double initial_lift_mag = norm(a_lift);

  // Special case for zero relative velocity or high altitude that simply
  // returns the state with zero lift and drag
  if (v_rel_mag < 1e-6 || altitude > 1e5) {
    // If the relative velocity is zero, we cannot define a local coordinate
    // system
    return 0;
  }

  // e_1 is the unit vector in direction of relative velocity
  // e_2 is the lift vector.
  // e_3 = e_1 x e_2
  cartvec e1 = sdivide(v_rel, v_rel_mag);
  cartvec e2;
  cartvec e3;
  // If the initial lift magnitude is zero, define e_2 based on a cross
  // product between e_1 and global z-axis
  if (initial_lift_mag < 1e-6) {
    cartvec global_z_axis;
    global_z_axis.x = 0;
    global_z_axis.y = 0;
    global_z_axis.z = 1;

    e2 = cross(e1, global_z_axis);

    // Normalize e_2 to make it a unit vector

    double e_2_mag = norm(e2);
    // If e_2 magnitude is still zero, we cannot define a local coordinate
    // system
    if (e_2_mag < 1e-6) {
      return 0;
    }
    // normalize e_2
    e2 = sdivide(e2, e_2_mag);

  } else {
    // set e_2 to the unit vector in the direction of the lift acceleration
    // vector
    e2 = sdivide(a_lift, initial_lift_mag);
  }

  e3 = cross(e1, e2);

  *e_1 = e1;
  *e_2 = e2;
  *e_3 = e3;
  return 1;
}

/**
 * Calculate the maximum jerk (rate of change of acceleration).
 *
 * @param run_params run parameters struct
 * @param vehicle vehicle struct
 * @return maximum jerk in m/s^3
 */
double get_jerk_max(runparams *run_params, vehicle *vehicle) {
  double max_a_exec = get_max_a_exec(run_params, vehicle);
  double deflection_time =
      run_params->deflection_time * run_params->gearing_ratio;
  double jerk_max = max_a_exec / deflection_time;

  return jerk_max;
}

/**
 * Project arr to e2 and e3 basis, clip to the max_val in that basis, then
 * project back to the Cartesian basis.
 *
 * @param e2 basis vector
 * @param e3 basis vector
 * @param array to project and clip
 * @param max_val maximum value for clipping
 * @return projected and clipped vector
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
 * Get the time derivative of the available lift acceleration.
 *
 * The available lift acceleration encodes the position of the control flaps.
 * The control flaps are assumed to move at an instantaneous acceleration up to
 * a fixed maximum angular velocity. The maximum angular velocity of the control
 * flaps is equivalent to a maximum available jerk.
 *
 * To avoid oscillations, as the available acceleration approaches the commanded
 * acceleration, the jerk reduces from the maximum jerk to a jerk proportional
 * to the difference. When the difference is less than the actuator resolution,
 * the derivative is zero.
 *
 * The proportional navigation commands may produce a commanded lift
 * acceleration with a component in the direction of the velocity, but the
 * control flaps will only attempt to produce lift acceleration in the plane
 * perpendicular to the estimated relative velocity (e_2, e_3 directions).
 *
 * Only valid during reentry phase.
 *
 * @param t current flight time (seconds)
 * @param true_state pointer to the true state
 * @param estimated_state pointer to the estimated state
 * @param run_params pointer to the run parameters struct
 * @param vehicle pointer to the vehicle struct
 * @param est_atm_cond pointer to the estimated atmospheric conditions
 * @param d_a_lift_avail_dt_true time derivative of true available lift
 * acceleration (m/s^3)
 * @param d_a_lift_avail_dt_est time derivative of estimated available lift
 * acceleration (m/s^3)
 * @return 0 if invalid, 1 if valid
 */
int get_a_lift_avail_jerk(double t, state *true_state, state *est_state,
                          runparams *run_params, vehicle *vehicle,
                          atm_cond *est_atm_cond,
                          cartvec *d_a_lift_avail_dt_true,
                          cartvec *d_a_lift_avail_dt_est) {
  // Determine if vehicle is in reentry phase
  double altitude = get_altitude(est_state->position);
  double angle_v_grav =
      acos(dot(est_state->velocity, est_state->a_grav) /
           (norm(est_state->velocity) * norm(est_state->a_grav)));
  int is_reentry =
      (angle_v_grav > 0) && (angle_v_grav < M_PI_2) && (altitude < 1e5);

  if (!is_reentry) {
    return 0;
  }

  // Calculate maximum parameters
  double max_a_exec = get_max_a_exec(run_params, vehicle);
  double jerk_max = get_jerk_max(run_params, vehicle);

  // Commanded acceleration is based on the aimpoint and the estimated state's
  // position and velocity
  cartvec a_command = prop_nav(est_state, run_params);

  // Get the lift basis vectors for the estimated state
  cartvec est_e1, est_e2, est_e3;
  int valid_basis =
      compute_lift_basis(est_state, est_atm_cond, &est_e1, &est_e2, &est_e3);
  if (!valid_basis) {
    return 0;
  }

  // Project the commanded acceleration onto the estimated lift basis vectors
  // e_2 and e_3 because all lift acceleration must be generated orthogonal to
  // the relative velocity
  cartvec a_target;
  a_target = project_and_clip(est_e2, est_e3, a_command, max_a_exec);
  // Change available lift at a fixed rate unless the difference between
  // current and target is small. For small differences, let the difference
  // reduce exponentially to keep the derivative continuous.
  cartvec a_avail_err = subtract(a_target, est_state->a_lift_avail);

  // Apply proportional gain and clip to jerk limits
  *d_a_lift_avail_dt_est = smultiply(a_avail_err, run_params->flap_gain);
  d_a_lift_avail_dt_est->x =
      clip(d_a_lift_avail_dt_est->x, -jerk_max, jerk_max);
  d_a_lift_avail_dt_est->y =
      clip(d_a_lift_avail_dt_est->y, -jerk_max, jerk_max);
  d_a_lift_avail_dt_est->z =
      clip(d_a_lift_avail_dt_est->z, -jerk_max, jerk_max);

  // True and estimated available lift are the same because the available lift
  // encodes the flap positions (no noise in flap position)
  d_a_lift_avail_dt_true->x = d_a_lift_avail_dt_est->x;
  d_a_lift_avail_dt_true->y = d_a_lift_avail_dt_est->y;
  d_a_lift_avail_dt_true->z = d_a_lift_avail_dt_est->z;

  return 1;
}

/**
 * Get time derivative of the lift acceleration for a single state.
 *
 * The lift acceleration approaches the available lift acceleration
 * exponentially based on the time constant: a(t) = a_avail * (1 - e^(-t/tau))
 */
cartvec get_a_lift_jerk_single_state(double t, state *current_state,
                                     runparams *run_params, vehicle *vehicle,
                                     atm_cond *atm_cond) {
  // Determine if vehicle is in reentry phase
  double altitude = get_altitude(current_state->position);
  int is_reentry = (t > vehicle->booster.total_burn_time) && (altitude < 1e5);

  if (!is_reentry) {
    return zeros();
  }

  // Calculate maximum parameters
  double max_a_exec = get_max_a_exec(run_params, vehicle);

  // Get time constant to simulate pressure build-up
  double time_constant = rv_time_constant(current_state, atm_cond, vehicle);

  // Quantize available lift to the resolution of the actuator
  double acc_resolution = get_acc_resolution(run_params, vehicle);
  cartvec a_lift_avail = current_state->a_lift_avail;

  cartvec ar = sdivide(a_lift_avail, acc_resolution);
  a_lift_avail.x = round(ar.x) * acc_resolution;
  a_lift_avail.y = round(ar.y) * acc_resolution;
  a_lift_avail.z = round(ar.z) * acc_resolution;

  // Get the lift basis vectors
  cartvec e1, e2, e3;
  int valid_basis = compute_lift_basis(current_state, atm_cond, &e1, &e2, &e3);

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
      subtract(a_lift_avail_projected, current_state->a_lift), time_constant);

  return d_a_lift_dt;
}

/**
 * Get time derivative of the lift acceleration for both true and estimated
 * states.
 *
 * The lift acceleration approaches the available lift acceleration
 * exponentially based on the time constant:
 * $$
 * a(t) = a_\text{avail} (1 - e^{-t/\tau})
 * $$
 *
 * The lift jerk is zero if it is not during the reentry phase or when the
 * lift basis cannot be calculated.
 */
void get_a_lift_jerk(double t, state *true_state, state *est_state,
                     runparams *run_params, vehicle *vehicle,
                     atm_cond *est_atm_cond, atm_cond *true_atm_cond,
                     cartvec *d_a_lift_dt_true, cartvec *d_a_lift_dt_est) {

  *d_a_lift_dt_true = get_a_lift_jerk_single_state(t, true_state, run_params,
                                                   vehicle, true_atm_cond);
  *d_a_lift_dt_est = get_a_lift_jerk_single_state(t, est_state, run_params,
                                                  vehicle, est_atm_cond);
}

void update_roll(state *true_state, state *est_state, cartvec true_d_a_lift,
                 cartvec est_d_a_lift, atm_cond *true_atm_cond,
                 atm_cond *est_atm_cond) {
  // Update roll based on the change in pitch and yaw accelerations
  cartvec e1, e2, e3;
  compute_lift_basis(true_state, true_atm_cond, &e1, &e2, &e3);
  double true_pitch_acceleration = dot(true_d_a_lift, e2);
  double true_yaw_acceleration = dot(true_d_a_lift, e3);
  true_state->roll += atan2(true_yaw_acceleration, true_pitch_acceleration);
  true_state->roll = fmod(true_state->roll + 2 * M_PI, 2 * M_PI);

  compute_lift_basis(est_state, est_atm_cond, &e1, &e2, &e3);
  double est_pitch_acceleration = dot(est_d_a_lift, e2);
  double est_yaw_acceleration = dot(est_d_a_lift, e3);
  est_state->roll += atan2(est_yaw_acceleration, est_pitch_acceleration);
  est_state->roll = fmod(est_state->roll + 2 * M_PI, 2 * M_PI);
}

void update_lift(state *true_state, state *est_state, runparams *run_params,
                 atm_cond *true_atm_cond, atm_cond *est_atm_cond,
                 vehicle *vehicle, double time_step) {
  cartvec true_d_a_lift_dt;
  cartvec est_d_a_lift_dt;

  cartvec true_d_a_lift_avail_dt;
  cartvec est_d_a_lift_avail_dt;

  get_a_lift_jerk(true_state->t, true_state, est_state, run_params, vehicle,
                  true_atm_cond, est_atm_cond, &true_d_a_lift_dt,
                  &est_d_a_lift_dt);

  int valid = get_a_lift_avail_jerk(
      true_state->t, true_state, est_state, run_params, vehicle, est_atm_cond,
      &true_d_a_lift_avail_dt, &est_d_a_lift_avail_dt);
  if (!valid) {
    true_d_a_lift_avail_dt = zeros();
    est_d_a_lift_avail_dt = zeros();
  }

  // Update true state
  cartvec true_d_a_lift = smultiply(true_d_a_lift_dt, time_step);
  true_state->a_lift = add(true_state->a_lift, true_d_a_lift);

  true_state->a_lift_avail = add(true_state->a_lift_avail,
                                 smultiply(true_d_a_lift_avail_dt, time_step));

  // Update estimated state
  cartvec est_d_a_lift = smultiply(est_d_a_lift_dt, time_step);
  est_state->a_lift = add(est_state->a_lift, est_d_a_lift);

  est_state->a_lift_avail =
      add(est_state->a_lift_avail, smultiply(est_d_a_lift_avail_dt, time_step));

  update_roll(true_state, est_state, true_d_a_lift, est_d_a_lift, true_atm_cond,
              est_atm_cond);
}

#endif