#ifndef STATE_H
#define STATE_H

#include "../math/linalg.h"
#include "../utils/constants.h"
#include "../utils/utils.h"

// Define a struct to store the state of a vehicle in 3D space
typedef struct state {
  // State parameters
  cartvec position;      // position in meters, ECI frame
  cartvec velocity;      // velocity in meters per second, ECI frame
  quaternion q_EB;       // rotation from body frame to ECI
  cartvec angular_vel_B; // body-frame angular velocity
  double theta_long; // thrust angle in the longitudinal direction measured from
                     // the x-z plane in radians
  double theta_lat;  // thrust angle in the latitudinal direction measured from
                     // the x-y plane in radians
  anglevec gyro_error;
  anglevec orientation_angle_change; // incremental body orientation change
  double delta_1;      // flap pair {1, 3} deflection extent
  double delta_2;      // flap pair {2, 4} deflection extent
  double prev_a_cmd_1; // previous transverse command acceleration (body axis 1)
  double prev_a_cmd_2; // previous transverse command acceleration (body axis 2)
  double prev_delta_1; // previous flap pair {1, 3} deflection extent
  double prev_delta_2; // previous flap pair {2, 4} deflection extent

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
  state.q_EB = identity_quaternion();
  state.angular_vel_B = zeros();

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
  state.delta_1 = 0;
  state.delta_2 = 0;
  state.prev_a_cmd_1 = 0;
  state.prev_a_cmd_2 = 0;
  state.prev_delta_1 = 0;
  state.prev_delta_2 = 0;

  // The true state holds the gyro error
  state.gyro_error.pitch = initial_theta_lat_pert;
  state.gyro_error.yaw = initial_theta_long_pert;
  state.orientation_angle_change = (anglevec){0};

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
  state.q_EB = identity_quaternion();
  state.angular_vel_B = zeros();

  state.theta_long = run_params->theta_long;
  state.theta_lat = run_params->theta_lat;
  state.delta_1 = 0;
  state.delta_2 = 0;
  state.prev_a_cmd_1 = 0;
  state.prev_a_cmd_2 = 0;
  state.prev_delta_1 = 0;
  state.prev_delta_2 = 0;

  state.gyro_error.pitch = 0;
  state.gyro_error.yaw = 0;
  state.orientation_angle_change = (anglevec){0};

  return state;
}

/**
 * Add two states together component-wise
 */
state add_state(state a, state b) {
  state result;

  result.position = add(a.position, b.position);
  result.velocity = add(a.velocity, b.velocity);
  result.q_EB = a.q_EB;
  result.angular_vel_B = add(a.angular_vel_B, b.angular_vel_B);
  result.theta_long = a.theta_long + b.theta_long;
  result.theta_lat = a.theta_lat + b.theta_lat;
  result.delta_1 = a.delta_1 + b.delta_1;
  result.delta_2 = a.delta_2 + b.delta_2;
  result.prev_a_cmd_1 = a.prev_a_cmd_1 + b.prev_a_cmd_1;
  result.prev_a_cmd_2 = a.prev_a_cmd_2 + b.prev_a_cmd_2;
  result.prev_delta_1 = a.prev_delta_1 + b.prev_delta_1;
  result.prev_delta_2 = a.prev_delta_2 + b.prev_delta_2;

  result.gyro_error.pitch = a.gyro_error.pitch + b.gyro_error.pitch;
  result.gyro_error.yaw = a.gyro_error.yaw + b.gyro_error.yaw;
  result.orientation_angle_change = add_anglevec(a.orientation_angle_change,
                                                 b.orientation_angle_change);

  return result;
}

/**
 * Multiply each element of the state by a scalar double
 */
state smultiply_state(state a, double s) {
  state result;

  result.position = smultiply(a.position, s);
  result.velocity = smultiply(a.velocity, s);
  result.q_EB = a.q_EB;
  result.angular_vel_B = smultiply(a.angular_vel_B, s);
  result.theta_long = a.theta_long * s;
  result.theta_lat = a.theta_lat * s;
  result.delta_1 = a.delta_1 * s;
  result.delta_2 = a.delta_2 * s;
  result.prev_a_cmd_1 = a.prev_a_cmd_1 * s;
  result.prev_a_cmd_2 = a.prev_a_cmd_2 * s;
  result.prev_delta_1 = a.prev_delta_1 * s;
  result.prev_delta_2 = a.prev_delta_2 * s;

  result.gyro_error.pitch = a.gyro_error.pitch * s;
  result.gyro_error.yaw = a.gyro_error.yaw * s;
  result.orientation_angle_change = smultiply_angle(a.orientation_angle_change, s);

  return result;
}

#endif