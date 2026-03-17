#include "../src/include/trajectory.h"
#include <tau/tau.h>

TEST(trajectory, impact_linterp) {
  // This test checks the simple linear interpolation used to estimate the
  // impact crossing between one state above the surface and one below it.
  // With symmetric inputs, the interpolated impact time and state should land
  // exactly at the midpoint.
  runparams run_params = {0};
  run_params.grav_error = 0;

  grav grav = init_grav(&run_params);
  state state_0 = {0};
  state state_1 = {0};
  state_0.position.x = grav.earth_radius + 1;
  state_0.position.y = 0;
  state_0.position.z = 0;
  state_0.velocity.x = -2;
  state_0.velocity.y = 0;
  state_0.velocity.z = 0;

  state_1.position.x = grav.earth_radius - 1;
  state_1.position.y = 0;
  state_1.position.z = 0;
  state_1.velocity.x = 0;
  state_1.velocity.y = 0;
  state_1.velocity.z = 0;

  double impact_t = 0;
  state impact_state = impact_linterp(&state_0, &state_1, 0.0, 1.0, &impact_t);

  REQUIRE_EQ(impact_t, 0.5);
  REQUIRE_EQ(impact_state.position.x, grav.earth_radius);
  REQUIRE_EQ(impact_state.position.y, 0);
  REQUIRE_EQ(impact_state.position.z, 0);
  REQUIRE_EQ(impact_state.velocity.x, -1);
  REQUIRE_EQ(impact_state.velocity.y, 0);
  REQUIRE_EQ(impact_state.velocity.z, 0);
}

TEST(trajectory, fly) {
  // Use the mock vehicle
  vehicle vehicle = init_mock_vehicle();
  runparams run_params = {0};
  run_params.run_name = "test_run";
  run_params.traj_output = 0;
  run_params.time_step_lambert = 1;
  run_params.time_step_midcourse = 1;
  run_params.time_step_atm = 1;
  run_params.x_aim = EARTH_RADIUS_M;
  run_params.y_aim = 0;
  run_params.z_aim = 0;
  run_params.theta_long = 0;
  run_params.theta_lat = 0;
  run_params.integrator = 0;
  run_params.rv_type = 0;
  run_params.grav_error = 0;
  run_params.include_drag = 0;
  run_params.atm_model = 0;
  run_params.gnss_nav = 0;
  run_params.rv_maneuv = 0;
  run_params.perfect_boost = 0;
  run_params.optimize_boost = 0;
  run_params.initial_x_error = 0;
  run_params.initial_pos_error = 0;
  run_params.initial_vel_error = 0;
  run_params.initial_angle_error = 0;
  run_params.acc_scale_stability = 0;
  run_params.gyro_bias_stability = 0;
  run_params.gyro_noise = 0;
  run_params.gnss_noise = 0;

  state initial_state = init_true_state(&run_params);
  initial_state.position.x += 10;

  double impact_time = 0;
  double burnout_vel_mag = 0;
  double burnout_alt = 0;
  double burnout_ang = 0;
  double apogee_alt = 0;
  double reentry_vel = 0;
  double reentry_ang = 0;

  state final_state =
      fly(&run_params, &initial_state, &vehicle, &impact_time, &burnout_vel_mag,
          &burnout_alt, &burnout_ang, &apogee_alt, &reentry_vel, &reentry_ang);

  // Dropping from 10 m with a 1 s step should impact after roughly one step.
  REQUIRE_GT(impact_time, 1.0);
  REQUIRE_LT(impact_time, 2.0);
  initial_state = init_true_state(&run_params);
  initial_state.velocity.x = 10;
  initial_state.velocity.y = 10;
  initial_state.velocity.z = 10;

  double impact_time_fast = 0;
  final_state = fly(&run_params, &initial_state, &vehicle, &impact_time_fast,
                    &burnout_vel_mag, &burnout_alt, &burnout_ang, &apogee_alt,
                    &reentry_vel, &reentry_ang);

  // Giving the initial state upward velocity should delay impact relative to
  // the pure drop case above.
  REQUIRE_GT(impact_time_fast, impact_time);
}