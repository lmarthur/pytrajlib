#ifndef IMU_H
#define IMU_H

#include "models/state.h"
#include "models/vehicle.h"
#include "rng/rng.h"
#include "utils/run_params.h"

// Define an inertial measurement unit struct
typedef struct imu {
  // Accelerometer parameters
  double acc_scale_stability; // Scale stability (ppm)

  cartvec acc_scale; // Scale factor

  // Gyroscope parameters
  double gyro_bias_stability; // Gyro bias (rad/s)
  double gyro_noise;          // Gyro noise/random walk (rad/s/sqrt(s))

  double gyro_bias_lat;  // Gyro bias in the latitude direction (rad/s)
  double gyro_bias_long; // Gyro bias in the longitude direction (rad/s)

  double gyro_error_lat; // Gyro error in the latitude direction (rad/s, defined
                         // recursively)
  double gyro_error_long; // Gyro error in the longitude direction (rad/s,
                          // defined recursively)

  double
      theta_long;   // Measured thrust angle in the longitudinal direction (rad)
  double theta_lat; // Measured thrust angle in the latitudinal direction (rad)

} imu;

imu imu_init(runparams *run_params, state *initial_state) {
  /*
  Initializes an accelerometer struct

  INPUTS:
  ----------
      run_params: runparams *
          pointer to the run parameters struct
      initial_state: state *
          pointer to the initial state of the vehicle

  OUTPUTS:
  ----------
      imu: imu
          pointer to the inertial measurement unit struct
  */

  imu imu;
  imu.acc_scale_stability = run_params->acc_scale_stability;
  imu.acc_scale.x = imu.acc_scale_stability * ran_gaussian(1);
  imu.acc_scale.y = imu.acc_scale_stability * ran_gaussian(1);
  imu.acc_scale.z = imu.acc_scale_stability * ran_gaussian(1);

  imu.gyro_bias_stability = run_params->gyro_bias_stability;
  imu.gyro_noise = run_params->gyro_noise;

  imu.gyro_bias_lat = imu.gyro_bias_stability * ran_gaussian(1);  // rad/s
  imu.gyro_bias_long = imu.gyro_bias_stability * ran_gaussian(1); // rad/s

  return imu;
}

/**
 * Calculate gyro-measured thrust angles adjusted for gyro error.
 *
 * True state: returns unmodified vehicle thrust angles because (by default) the
 * true state initializes gyro_error to init_thrust_angle_pert. Estimated state:
 * returns perturbed angles with accumulated drift. The perturbation appears
 * because estimated state (by default) initializes gyro_error to zero.
 */
anglevec gyro_measurement(state *current_state, vehicle *vehicle,
                          runparams *run_params) {
  anglevec thrust_angles;

  thrust_angles.lat = vehicle->booster.thrust_angles.lat +
                      current_state->gyro_error.lat -
                      run_params->init_thrust_lat_pert;
  thrust_angles.lon = vehicle->booster.thrust_angles.lon +
                      current_state->gyro_error.lon -
                      run_params->init_thrust_lon_pert;
  return thrust_angles;
}

/**
 * Drift/deterministic component of the gyro error derivative (for the estimated
 * state. The true state's d_gyro_error_dt is always 0).
 */
anglevec update_imu_drift(imu imu) {
  anglevec d_gyro_error_dt_est;
  d_gyro_error_dt_est.lat = imu.gyro_bias_lat;
  d_gyro_error_dt_est.lon = imu.gyro_bias_long;

  return d_gyro_error_dt_est;
}

/**
 * Diffusion/stochastic component of the gyro error derivative (for the
 * estimated state. The true state's d_gyro_error_dt is always 0).
 */
anglevec update_imu_diffusion(imu imu) {
  anglevec d_gyro_error_dW_est;
  d_gyro_error_dW_est.lat = imu.gyro_noise;
  d_gyro_error_dW_est.lon = imu.gyro_noise;
  return d_gyro_error_dW_est;
}

/**
 * Simulate accelerometer measurement for the estimated state
 */
cartvec accelerometer_measurement(imu *imu, cartvec a_true_total,
                                  cartvec a_true_grav, cartvec a_est_grav,
                                  state *est_state) {
  // Calculate measurable acceleration (remove gravity)
  cartvec a_measurable;
  a_measurable.x = a_true_total.x - a_true_grav.x;
  a_measurable.y = a_true_total.y - a_true_grav.y;
  a_measurable.z = a_true_total.z - a_true_grav.z;

  double cross_coupling[3][3] = {
      {1 + imu->acc_scale.x, est_state->gyro_error.lon,
       -est_state->gyro_error.lat},
      {est_state->gyro_error.lon, 1 + imu->acc_scale.y,
       est_state->gyro_error.lon * est_state->gyro_error.lat},
      {est_state->gyro_error.lat, 0, 1 + imu->acc_scale.z}};

  cartvec a_coupled = matvec_multiply(cross_coupling, a_measurable);
  cartvec a_est_total;
  a_est_total.x = a_coupled.x + a_est_grav.x;
  a_est_total.y = a_coupled.y + a_est_grav.y;
  a_est_total.z = a_coupled.z + a_est_grav.z;
  return a_est_total;
}

#endif