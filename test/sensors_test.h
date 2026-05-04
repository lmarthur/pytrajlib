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
  REQUIRE_EQ(imu.gyro_bias.pitch, 0);
  REQUIRE_EQ(imu.gyro_bias.yaw, 0);
}

TEST(sensors, imu_meas) {
  // Check the accelerometer measurement model in three regimes:
  // exact pass-through with no sensor error, zero input acceleration, and a
  // perturbed output when accelerometer scale error is enabled.
  runparams run_params = {0};

  state true_state = init_true_state(&run_params);
  state est_state = init_est_state(&run_params);
  imu imu = imu_init(&run_params, &true_state);

  cartvec a_total_true = {10, 10, 10};
  cartvec a_grav_true = zeros();
  cartvec a_grav_est = zeros();
  grav est_grav = init_grav(&run_params);

  cartvec a_total_est = imu_measurement(&imu, &run_params, NULL, NULL, 0.0, 0.0,
                                        &true_state, &est_state, a_total_true,
                                        a_grav_true, a_grav_est, &est_grav);

  REQUIRE_EQ(fabs(a_total_est.x - a_total_true.x), 0);
  REQUIRE_EQ(fabs(a_total_est.y - a_total_true.y), 0);
  REQUIRE_EQ(fabs(a_total_est.z - a_total_true.z), 0);

  a_total_true = zeros();
  a_total_est = imu_measurement(&imu, &run_params, NULL, NULL, 0.0, 0.0,
                                &true_state, &est_state, a_total_true,
                                a_grav_true, a_grav_est, &est_grav);
  REQUIRE_EQ(a_total_est.x, 0);
  REQUIRE_EQ(a_total_est.y, 0);
  REQUIRE_EQ(a_total_est.z, 0);

  run_params.acc_scale_stability = 1e-3;
  imu = imu_init(&run_params, &true_state);
  a_total_true.x = 10;
  a_total_true.y = 10;
  a_total_true.z = 10;
  a_total_est = imu_measurement(&imu, &run_params, NULL, NULL, 0.0, 0.0,
                                &true_state, &est_state, a_total_true,
                                a_grav_true, a_grav_est, &est_grav);
  REQUIRE_GT(norm(subtract(a_total_est, a_total_true)), 0);
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
  state est_state = init_est_state(&run_params);
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
