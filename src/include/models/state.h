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
  cartvec orientation_angle_change; // incremental body orientation change
  double delta_1;                   // flap pair {1, 3} deflection extent
  double delta_2;                   // flap pair {2, 4} deflection extent
  double dot_delta_1;               // flap pair {1, 3} deflection rate
  double dot_delta_2;               // flap pair {2, 4} deflection rate

} state;
/**
 * Initialize the true vehicle state at launch with stochastic
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
  state.angular_vel_B = zeros();

  state.delta_1 = 0;
  state.delta_2 = 0;
  state.dot_delta_1 = 0;
  state.dot_delta_2 = 0;

  // At launch the vehicle stands vertically on the pad, so the guidance system
  // believes the roll axis points along the local vertical. The launch point
  // sits on the +x axis, so that is the vertical here.
  cartvec launch_vertical = {1.0, 0.0, 0.0};

  // The vehicle is really misaligned from that belief by initial_angle_error
  // radians, tipped a random way around the vertical. The rotation axis is
  // taken perpendicular to the vertical so the whole angle appears as a
  // pointing error; a rotation about the vehicle's own axis is pure roll and
  // would not tip the nose at all.
  cartvec axis_noise = gaussian_cartvec();
  cartvec axis = subtract(axis_noise, project(axis_noise, launch_vertical));
  double axis_norm = norm(axis);
  if (axis_norm < 1e-12) {
    // Degenerate draw parallel to the vertical; any horizontal axis will do.
    axis = (cartvec){0.0, 1.0, 0.0};
    axis_norm = 1.0;
  }
  cartvec tilt_axis = sdivide(axis, axis_norm);
  cartvec attitude_error =
      smultiply(tilt_axis, run_params->initial_angle_error);

  // The true attitude is the believed attitude carrying that rotation. It
  // composes on the left because the tilt axis is given in inertial
  // components; a body-frame increment, like the gyro's in the integrator,
  // composes on the right instead.
  state.q_EB = qmultiply(quaternion_from_rotation_vector(attitude_error),
                         align_roll_axis_with_vector(launch_vertical));

  // The commanded thrust angles carry no perturbation of their own. The lean
  // reaches the thrust through the frames instead: get_thrust_acc resolves the
  // command into the body frame with the estimated attitude and back out with
  // the true one. That same round trip is applied to the acceleration in
  // imu_measurement, so the two rotations cancel inside the navigator and the
  // lean stays unobservable, which is what makes it an alignment error rather
  // than a thrust deflection the guidance could simply steer out.
  state.theta_long = run_params->theta_long;
  state.theta_lat = run_params->theta_lat;

  // The incremental angle change is cleared at the start of every integrator
  // step, so it carries no initial condition.
  state.orientation_angle_change = (cartvec){0};

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
  // At launch the vehicle stands vertically and the guidance system has no
  // attitude error of its own. align_roll_axis_with_vector points the roll
  // axis, body -z, along the vector it is given, so this puts the nose up.
  cartvec launch_vertical = {1.0, 0.0, 0.0};
  state.q_EB = align_roll_axis_with_vector(launch_vertical);
  state.angular_vel_B = zeros();

  state.theta_long = run_params->theta_long;
  state.theta_lat = run_params->theta_lat;
  state.delta_1 = 0;
  state.delta_2 = 0;
  state.dot_delta_1 = 0;
  state.dot_delta_2 = 0;
  state.orientation_angle_change = (cartvec){0};

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
  result.dot_delta_1 = a.dot_delta_1 + b.dot_delta_1;
  result.dot_delta_2 = a.dot_delta_2 + b.dot_delta_2;
  result.orientation_angle_change =
      add(a.orientation_angle_change, b.orientation_angle_change);

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
  result.dot_delta_1 = a.dot_delta_1 * s;
  result.dot_delta_2 = a.dot_delta_2 * s;
  result.orientation_angle_change = smultiply(a.orientation_angle_change, s);

  return result;
}

#endif