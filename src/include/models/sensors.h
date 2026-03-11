#ifndef SENSORS_H
#define SENSORS_H

#include "../body_frame.h"
#include "../rng/rng.h"
#include "../utils.h"
#include "state.h"

// Define an inertial measurement unit struct
typedef struct imu {
  // Accelerometer parameters
  cartvec acc_scale; // Scale factor (ppm)

  // Gyroscope parameters
  double gyro_bias_stability; // Gyro bias (rad/s)
  double gyro_noise;          // Gyro noise/random walk (rad/s/sqrt(s))

  anglevec gyro_bias; // Gyro bias (rad/s)

} imu;

/**
 * Initialize IMU parameters and initial gyro error states.
 *
 * @param run_params Pointer to run configuration parameters
 * @param initial_state Pointer to initial vehicle state
 * @return Initialized IMU model
 */
imu imu_init(runparams *run_params, state *initial_state) {

  imu imu;
  imu.acc_scale =
      smultiply(gaussian_cartvec(), run_params->acc_scale_stability); // ppm

  imu.gyro_bias_stability = run_params->gyro_bias_stability;
  imu.gyro_noise = run_params->gyro_noise;

  imu.gyro_bias.lat = imu.gyro_bias_stability * ran_gaussian(1); // rad/s
  imu.gyro_bias.lon = imu.gyro_bias_stability * ran_gaussian(1); // rad/s

  return imu;
}

/**
 * Apply IMU attitude and acceleration measurement model to update
 * estimated state.
 *
 * @param imu Pointer to IMU model/state
 * @param true_state Pointer to true vehicle state
 * @param est_state Pointer to estimated vehicle state to update
 */
cartvec imu_measurement(imu *imu, state *true_state, state *est_state,
                        cartvec a_total_true, cartvec a_grav_true,
                        cartvec a_grav_est) {

  // Gyroscope measurements
  est_state->theta_long = true_state->theta_long + est_state->gyro_error.lon -
                          true_state->initial_theta_long_pert;
  est_state->theta_lat = true_state->theta_lat + est_state->gyro_error.lat -
                         true_state->initial_theta_lat_pert;
  cartvec a_measurable = subtract(a_total_true, a_grav_true);

  // Get body-centric basis vectors
  cartvec e1;
  cartvec e2;
  cartvec e3;
  int valid = get_body_frame(true_state, NULL, &e1, &e2, &e3);
  if (!valid) {
    printf("Warning: Invalid body frame\n");
  }

  // Change to body-centric basis
  cartvec a_measurable_body_frame = {
      dot(a_measurable, e1),
      dot(a_measurable, e2),
      dot(a_measurable, e3),
  };

  // Small angle rotation matrix around y (pitch/lat) and z (yaw/lon)
  double cross_coupling[3][3] = {
      {1 + imu->acc_scale.x, est_state->gyro_error.lon,
       -est_state->gyro_error.lat},
      {-est_state->gyro_error.lon, 1 + imu->acc_scale.y, 0},
      {est_state->gyro_error.lat, 0, 1 + imu->acc_scale.z},
  };

  // Get measured acceleration in the body-centric basis
  cartvec a_measured_body_frame =
      matvec_multiply(cross_coupling, a_measurable_body_frame);

  // Change back to global basis
  double B[3][3] = {{e1.x, e2.x, e3.x}, {e1.y, e2.y, e3.y}, {e1.z, e2.z, e3.z}};
  cartvec a_measured = matvec_multiply(B, a_measured_body_frame);

  // Total acceleration is measured + estimated gravity
  cartvec a_total_est = add(a_measured, a_grav_est);

  return a_total_est;
}

/**
 * Get drift component of gyro error (scales with dt)
 */
anglevec get_gyro_drift(imu *imu) { return imu->gyro_bias; }

/**
 * Get stochastic diffusion component of gyro error (scales with sqrt(dt))
 */
double get_gyro_diffusion(imu *imu) { return imu->gyro_noise; }

// define a gnss measurement unit struct
typedef struct gnss {
  double noise;                  // GNSS noise in meters
  double time_since_last_update; // time since last GNSS update in seconds

} gnss;

/**
 * Initialize GNSS measurement model.
 *
 * @param run_params Pointer to run configuration parameters
 * @return Initialized GNSS model
 */
gnss gnss_init(runparams *run_params) {

  gnss gnss;
  gnss.noise = run_params->gnss_noise;
  gnss.time_since_last_update = 0.0;

  return gnss;
}

/**
 * Apply GNSS position measurement model to update estimated state position.
 *
 * @param gnss Pointer to GNSS model
 * @param true_state Pointer to true vehicle state
 * @param est_state Pointer to estimated state to update
 */
void gnss_measurement(gnss *gnss, state *true_state, state *est_state) {

  // Position measurements
  est_state->position.x =
      true_state->position.x + gnss->noise * ran_gaussian(1);
  est_state->position.y =
      true_state->position.y + gnss->noise * ran_gaussian(1);
  est_state->position.z =
      true_state->position.z + gnss->noise * ran_gaussian(1);
}

/**
 * Set estimated state equal to true state (ideal measurement).
 *
 * @param true_state Pointer to true vehicle state
 * @param est_state Pointer to estimated vehicle state
 */
void perfect_measurement(state *true_state, state *est_state) {

  est_state = true_state;
}

#endif