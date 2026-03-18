#ifndef STATE_H
#define STATE_H

#include "../math/linalg.h"
#include "../utils/constants.h"
#include "../utils/utils.h"

// Define a struct to store the state of a vehicle in 3D space
typedef struct state {
  // State parameters
  cartvec position;     // position in meters
  cartvec velocity;     // velocity in meters per second
  cartvec a_lift;       // acceleration due to lift in meters per second squared
  cartvec a_lift_avail; // "available" lift. Encodes flap positions
  double initial_theta_long_pert; // initial perturbation in the longitudinal
                                  // thrust angle in radians
  double initial_theta_lat_pert;  // initial perturbation in the latitudinal
                                  // thrust angle in radians
  double theta_long; // thrust angle in the longitudinal direction measured from
                     // the x-z plane in radians
  double theta_lat;  // thrust angle in the latitudinal direction measured from
                     // the x-y plane in radians
  anglevec gyro_error;

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

  state.initial_theta_lat_pert =
      run_params->initial_angle_error * ran_gaussian(1) +
      run_params->theta_long * initial_rot_pert -
      fabs(run_params->theta_lat * initial_rot_pert);
  state.initial_theta_long_pert =
      run_params->initial_angle_error * ran_gaussian(1) -
      run_params->theta_lat * initial_rot_pert -
      fabs(run_params->theta_long * initial_rot_pert);
  state.theta_long = run_params->theta_long + state.initial_theta_long_pert;
  state.theta_lat = run_params->theta_lat + state.initial_theta_lat_pert;

  state.a_lift = zeros();

  state.a_lift_avail = zeros();

  // The true state holds the gyro error for ease of calculating the body frame
  // which uses the gyro error to perturb the true body frame.
  state.gyro_error.pitch = state.initial_theta_lat_pert;
  state.gyro_error.yaw = state.initial_theta_long_pert;

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
  state.initial_theta_lat_pert = 0;
  state.initial_theta_long_pert = 0;

  state.a_lift = zeros();

  state.a_lift_avail = zeros();

  state.gyro_error.pitch = 0;
  state.gyro_error.yaw = 0;

  return state;
}

/**
 * Add two states together component-wise
 */
state add_state(state a, state b) {
  state result;

  result.position = add(a.position, b.position);
  result.velocity = add(a.velocity, b.velocity);
  result.a_lift = add(a.a_lift, b.a_lift);
  result.a_lift_avail = add(a.a_lift_avail, b.a_lift_avail);
  result.initial_theta_long_pert =
      a.initial_theta_long_pert + b.initial_theta_long_pert;
  result.initial_theta_lat_pert =
      a.initial_theta_lat_pert + b.initial_theta_lat_pert;
  result.theta_long = a.theta_long + b.theta_long;
  result.theta_lat = a.theta_lat + b.theta_lat;

  result.gyro_error.pitch = a.gyro_error.pitch + b.gyro_error.pitch;
  result.gyro_error.yaw = a.gyro_error.yaw + b.gyro_error.yaw;

  return result;
}

/**
 * Multiply each element of the state by a scalar double
 */
state smultiply_state(state a, double s) {
  state result;

  result.position = smultiply(a.position, s);
  result.velocity = smultiply(a.velocity, s);
  result.a_lift = smultiply(a.a_lift, s);
  result.a_lift_avail = smultiply(a.a_lift_avail, s);
  result.initial_theta_long_pert = a.initial_theta_long_pert * s;
  result.initial_theta_lat_pert = a.initial_theta_lat_pert * s;
  result.theta_long = a.theta_long * s;
  result.theta_lat = a.theta_lat * s;

  result.gyro_error.pitch = a.gyro_error.pitch * s;
  result.gyro_error.yaw = a.gyro_error.yaw * s;

  return result;
}

#endif