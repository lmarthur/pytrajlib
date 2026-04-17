#ifndef STATE_H
#define STATE_H

#include "../math/linalg.h"
#include "../utils/constants.h"
#include "../utils/utils.h"

// Define a struct to store the state of a vehicle in 3D space
typedef struct state {
  // State parameters
  cartvec position;  // position in meters
  cartvec velocity;  // velocity in meters per second
  double theta_long; // thrust angle in the longitudinal direction measured from
                     // the x-z plane in radians
  double theta_lat;  // thrust angle in the latitudinal direction measured from
                     // the x-y plane in radians
  double deflection_angle; // control surface deflection angle in radians
  anglevec gyro_error;

  double alpha;      // angle of attack
  double d_alpha_dt; // time derivative of the angle of attack

} state;
/**
 * Initialize the true vehicle state at launch/reentry with stochastic
 * position, velocity, and attitude perturbations.
 *
 * @param run_params Pointer to run configuration parameters
 * @return Initialized true state
 */
state init_true_state(runparams *run_params) {

  state state;
  cartvec position_noise = gaussian_cartvec();
  cartvec velocity_noise = gaussian_cartvec();

  state.position.x =
      EARTH_RADIUS_M + run_params->initial_x_error * position_noise.x;
  state.position.y = run_params->initial_pos_error * position_noise.y;
  state.position.z = run_params->initial_pos_error * position_noise.z;

  state.velocity = smultiply(velocity_noise, run_params->initial_vel_error);

  double initial_rot_pert = run_params->initial_angle_error * ran_gaussian(1);
  double initial_theta_lat_pert =
      run_params->initial_angle_error * ran_gaussian(1) +
      run_params->theta_long * initial_rot_pert -
      fabs(run_params->theta_lat * initial_rot_pert);
  double initial_theta_long_pert =
      run_params->initial_angle_error * ran_gaussian(1) -
      run_params->theta_lat * initial_rot_pert -
      fabs(run_params->theta_long * initial_rot_pert);

  state.theta_long = run_params->theta_long + initial_theta_long_pert;
  state.theta_lat = run_params->theta_lat + initial_theta_lat_pert;
  state.deflection_angle = 0;

  // The true state holds the gyro error for ease of calculating the body frame
  // which uses the gyro error to perturb the true body frame.
  state.gyro_error.pitch = initial_theta_lat_pert;
  state.gyro_error.yaw = initial_theta_long_pert;

  state.alpha = 0;
  state.d_alpha_dt = 0;

  return state;
}

/**
 * Initialize the estimated vehicle state without stochastic perturbations.
 *
 * @param run_params Pointer to run configuration parameters
 * @return Initialized estimated state
 */
state init_est_state(runparams *run_params) {

  state state;
  // Initialize for full trajectory run
  state.position.x = EARTH_RADIUS_M;
  state.position.y = 0;
  state.position.z = 0;

  state.velocity = zeros();

  state.theta_long = run_params->theta_long;
  state.theta_lat = run_params->theta_lat;
  state.deflection_angle = 0;

  state.gyro_error.pitch = 0;
  state.gyro_error.yaw = 0;

  state.alpha = 0;
  state.d_alpha_dt = 0;

  return state;
}

/**
 * Add two states together component-wise
 */
state add_state(state a, state b) {
  state result;

  result.position = add(a.position, b.position);
  result.velocity = add(a.velocity, b.velocity);
  result.theta_long = a.theta_long + b.theta_long;
  result.theta_lat = a.theta_lat + b.theta_lat;
  result.deflection_angle = a.deflection_angle + b.deflection_angle;

  result.gyro_error.pitch = a.gyro_error.pitch + b.gyro_error.pitch;
  result.gyro_error.yaw = a.gyro_error.yaw + b.gyro_error.yaw;

  result.alpha = a.alpha + b.alpha;
  result.d_alpha_dt = a.d_alpha_dt + b.d_alpha_dt;

  return result;
}

/**
 * Multiply each element of the state by a scalar double
 */
state smultiply_state(state a, double s) {
  state result;

  result.position = smultiply(a.position, s);
  result.velocity = smultiply(a.velocity, s);
  result.theta_long = a.theta_long * s;
  result.theta_lat = a.theta_lat * s;
  result.deflection_angle = a.deflection_angle * s;

  result.gyro_error.pitch = a.gyro_error.pitch * s;
  result.gyro_error.yaw = a.gyro_error.yaw * s;

  result.alpha = a.alpha * s;
  result.d_alpha_dt = a.d_alpha_dt * s;

  return result;
}

#endif