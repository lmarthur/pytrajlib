#include "../src/include/models/sensors.h"
#include <tau/tau.h>

TEST(sensors, imu_init) {
  // Verify IMU initialization copies configuration values into the sensor
  // model and produces zero-valued random terms when the configured noise is
  // zero.
  runparams run_params = {0};
  state true_state = init_true_state(&run_params);

  imu imu = imu_init(&run_params, &true_state);

  REQUIRE_EQ(norm(imu.acc_scale), 0);
  REQUIRE_EQ(imu.gyro_bias_stability, run_params.gyro_bias_stability);
  REQUIRE_EQ(imu.gyro_noise, run_params.gyro_noise);
  REQUIRE_EQ(imu.gyro_bias.lat, 0);
  REQUIRE_EQ(imu.gyro_bias.lon, 0);
}

TEST(sensors, imu_meas) {
  // Check the accelerometer measurement model in three regimes:
  // exact pass-through with no sensor error, zero input acceleration, and a
  // perturbed output when accelerometer scale error is enabled.
  runparams run_params = {0};

  state true_state = init_true_state(&run_params);
  state est_state = init_est_state(&run_params, true_state);
  imu imu = imu_init(&run_params, &true_state);

  cartvec a_total_true = {10, 10, 10};
  cartvec a_grav_true = zeros();
  cartvec a_grav_est = zeros();

  cartvec a_total_est = imu_measurement(&imu, &true_state, &est_state,
                                        a_total_true, a_grav_true, a_grav_est);

  REQUIRE_EQ(fabs(a_total_est.x - a_total_true.x), 0);
  REQUIRE_EQ(fabs(a_total_est.y - a_total_true.y), 0);
  REQUIRE_EQ(fabs(a_total_est.z - a_total_true.z), 0);

  a_total_true = zeros();
  a_total_est = imu_measurement(&imu, &true_state, &est_state, a_total_true,
                                a_grav_true, a_grav_est);
  REQUIRE_EQ(a_total_est.x, 0);
  REQUIRE_EQ(a_total_est.y, 0);
  REQUIRE_EQ(a_total_est.z, 0);

  run_params.acc_scale_stability = 1e-3;
  imu = imu_init(&run_params, &true_state);
  a_total_true.x = 10;
  a_total_true.y = 10;
  a_total_true.z = 10;
  a_total_est = imu_measurement(&imu, &true_state, &est_state, a_total_true,
                                a_grav_true, a_grav_est);
  REQUIRE_GT(norm(subtract(a_total_est, a_total_true)), 0);
}

TEST(sensors, imu_update) {
  // Confirm the attitude estimate is reconstructed from the stored gyro error,
  // and that the gyro drift/diffusion accessors expose the IMU parameters used
  // by the integrator.
  runparams run_params = {0};
  state true_state = init_true_state(&run_params);
  state est_state = init_est_state(&run_params, true_state);

  true_state.theta_long = 1;
  true_state.theta_lat = -0.5;
  est_state.gyro_error.lon = 0.1;
  est_state.gyro_error.lat = -0.2;

  imu imu = imu_init(&run_params, &true_state);
  cartvec a_total_est =
      imu_measurement(&imu, &true_state, &est_state, zeros(), zeros(), zeros());

  REQUIRE_EQ(a_total_est.x, 0);
  REQUIRE_EQ(a_total_est.y, 0);
  REQUIRE_EQ(a_total_est.z, 0);
  REQUIRE_EQ(est_state.theta_long,
             true_state.theta_long + est_state.gyro_error.lon);
  REQUIRE_EQ(est_state.theta_lat,
             true_state.theta_lat + est_state.gyro_error.lat);

  run_params.gyro_noise = 1e-3;
  run_params.gyro_bias_stability = 2e-3;
  imu = imu_init(&run_params, &true_state);

  anglevec drift = get_gyro_drift(&imu);
  double diffusion = get_gyro_diffusion(&imu);

  REQUIRE_EQ(drift.lat, imu.gyro_bias.lat);
  REQUIRE_EQ(drift.lon, imu.gyro_bias.lon);
  REQUIRE_EQ(diffusion, imu.gyro_noise);
}

TEST(sensors, gnss_init) {
  // GNSS initialization should preserve the configured position noise level.
  runparams run_params = {0};
  run_params.gnss_noise = 0;

  gnss gnss = gnss_init(&run_params);
  REQUIRE_EQ(gnss.noise, run_params.gnss_noise);
}

TEST(sensors, gnss_meas) {
  // GNSS measurements should match truth when noise is disabled and deviate
  // from truth once measurement noise is enabled.
  runparams run_params = {0};
  run_params.gnss_noise = 0;

  gnss gnss = gnss_init(&run_params);
  state true_state = init_true_state(&run_params);
  state est_state = init_est_state(&run_params, true_state);
  true_state.position.x = 10;
  true_state.position.y = 10;
  true_state.position.z = 10;

  gnss_measurement(&gnss, &true_state, &est_state);
  REQUIRE_EQ(fabs(est_state.position.x - true_state.position.x), 0);
  REQUIRE_EQ(fabs(est_state.position.y - true_state.position.y), 0);
  REQUIRE_EQ(fabs(est_state.position.z - true_state.position.z), 0);

  run_params.gnss_noise = 1e-3;
  gnss = gnss_init(&run_params);
  gnss_measurement(&gnss, &true_state, &est_state);
  REQUIRE_NE(est_state.position.x, true_state.position.x);
  REQUIRE_NE(est_state.position.y, true_state.position.y);
  REQUIRE_NE(est_state.position.z, true_state.position.z);
}
