#ifndef SENSORS_H
#define SENSORS_H

#include "../rng/rng.h"
#include "../utils/utils.h"
#include "atmosphere.h"
#include "grav.h"
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

  imu.gyro_bias.pitch = imu.gyro_bias_stability * ran_gaussian(1); // rad/s
  imu.gyro_bias.yaw = imu.gyro_bias_stability * ran_gaussian(1);   // rad/s

  return imu;
}

/**
 *
 * @param imu Pointer to IMU model/state
 * @param run_params Pointer to run configuration parameters
 * @param true_atm_cond Pointer to true atmospheric conditions
 * @param est_atm_cond Pointer to estimated atmospheric conditions
 * @param true_t Current true simulation time in seconds
 * @param est_t Current estimated simulation time in seconds
 * @param true_state Pointer to true vehicle state
 * @param est_state Pointer to estimated vehicle state to update
 * @param a_total_true Total acceleration of true state in m/s^2.
 * @param a_grav_true Gravitational acceleration at true position in m/s^2.
 * @param a_grav_est Gravitational acceleration at estimated position in m/s^2.
 * @param est_grav Pointer to estimated gravity model.
 * @return Measured total estimated acceleration in ECI coordinates in m/s^2.
 */
cartvec imu_measurement(imu *imu, runparams *run_params,
                        atm_cond *true_atm_cond, atm_cond *est_atm_cond,
                        double true_t, double est_t, state *true_state,
                        state *est_state, cartvec a_total_true,
                        cartvec a_grav_true, cartvec a_grav_est,
                        grav *est_grav) {
  // IMU measures total acceleration minus gravity
  cartvec a_measurable_E = subtract(a_total_true, a_grav_true);

  // Change from ECI to body-frame
  cartvec a_measurable_B = eci_to_body(a_measurable_E, true_state->q_EB);

  // Get measured acceleration in the body frame with accelerometer scale errors
  double scale_errors[3][3] = {
      {1 + imu->acc_scale.x, 0, 0},
      {0, 1 + imu->acc_scale.y, 0},
      {0, 0, 1 + imu->acc_scale.z},
  };
  cartvec a_measured_B = matvec_multiply(scale_errors, a_measurable_B);

  // Transform back to the estimated ECI frame
  cartvec a_measured_E = body_to_eci(a_measured_B, est_state->q_EB);

  // Total acceleration is measured + estimated gravity
  cartvec a_total_est = add(a_measured_E, a_grav_est);
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