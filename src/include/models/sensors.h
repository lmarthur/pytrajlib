#ifndef SENSORS_H
#define SENSORS_H

#include "../rng/rng.h"
#include "../utils.h"
#include "state.h"

// Define an inertial measurement unit struct
typedef struct imu {
  // Accelerometer parameters
  double acc_scale_stability; // Scale stability (ppm)

  double acc_scale_x; // Scale factor for x-axis (ppm)
  double acc_scale_y; // Scale factor for y-axis (ppm)
  double acc_scale_z; // Scale factor for z-axis (ppm)

  // Gyroscope parameters
  double gyro_bias_stability; // Gyro bias (rad/s)
  double gyro_noise;          // Gyro noise/random walk (rad/s/sqrt(s))

  double gyro_bias_lat;  // Gyro bias in the latitude direction (rad/s)
  double gyro_bias_long; // Gyro bias in the longitude direction (rad/s)

  double gyro_error_lat; // Gyro error in the latitude direction (rad/s, defined
                         // recursively)
  double gyro_error_long; // Gyro error in the longitude direction (rad/s,
                          // defined recursively)

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
  imu.acc_scale_stability = run_params->acc_scale_stability;
  imu.acc_scale_x = imu.acc_scale_stability * ran_gaussian(1); // ppm
  imu.acc_scale_y = imu.acc_scale_stability * ran_gaussian(1); // ppm
  imu.acc_scale_z = imu.acc_scale_stability * ran_gaussian(1); // ppm

  imu.gyro_bias_stability = run_params->gyro_bias_stability;
  imu.gyro_noise = run_params->gyro_noise;

  imu.gyro_bias_lat = imu.gyro_bias_stability * ran_gaussian(1);  // rad/s
  imu.gyro_bias_long = imu.gyro_bias_stability * ran_gaussian(1); // rad/s

  imu.gyro_error_lat = initial_state->initial_theta_lat_pert;
  imu.gyro_error_long = initial_state->initial_theta_long_pert;

  return imu;
}

/**
 * Apply IMU attitude and acceleration measurement model to update
 * estimated state.
 *
 * @param imu Pointer to IMU model/state
 * @param true_state Pointer to true vehicle state
 * @param est_state Pointer to estimated vehicle state to update
 * @param vehicle Pointer to vehicle model
 */
void imu_measurement(imu *imu, state *true_state, state *est_state,
                     vehicle *vehicle) {

  // Gyroscope measurements
  est_state->theta_long = true_state->theta_long + imu->gyro_error_long -
                          true_state->initial_theta_long_pert;
  est_state->theta_lat = true_state->theta_lat + imu->gyro_error_lat -
                         true_state->initial_theta_lat_pert;
  cartvec a_measurable = subtract(true_state->a_total, true_state->a_grav);

  // Accelerometer measurements
  est_state->a_total.x = a_measurable.x * (1 + imu->acc_scale_x) +
                         a_measurable.y * imu->gyro_error_long -
                         a_measurable.z * imu->gyro_error_lat +
                         est_state->a_grav.x;
  est_state->a_total.y =
      a_measurable.y * (1 + imu->acc_scale_y) -
      a_measurable.x * imu->gyro_error_long +
      a_measurable.z * imu->gyro_error_long * imu->gyro_error_lat +
      est_state->a_grav.y;
  est_state->a_total.z = a_measurable.z * (1 + imu->acc_scale_z) +
                         a_measurable.x * imu->gyro_error_lat +
                         est_state->a_grav.z;
}

/**
 * Propagate IMU gyro error state with bias and random walk.
 *
 * @param imu Pointer to IMU model/state
 * @param time_step Simulation time step in seconds
 */
void update_imu(imu *imu, double time_step) {

  // Update the gyro error by recursively adding noise and bias drift
  imu->gyro_error_long = imu->gyro_error_long +
                         imu->gyro_noise * ran_gaussian(1) * sqrt(time_step) +
                         imu->gyro_bias_long * time_step;
  imu->gyro_error_lat = imu->gyro_error_lat +
                        imu->gyro_noise * ran_gaussian(1) * sqrt(time_step) +
                        imu->gyro_bias_lat * time_step;
}

// define a gnss measurement unit struct
typedef struct gnss {
  double noise; // GNSS noise in meters

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