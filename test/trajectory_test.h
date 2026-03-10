#include "../src/include/trajectory.h"
#include <tau/tau.h>

TEST(trajectory, impact_linterp) {
  runparams run_params;
  run_params.grav_error = 0;

  grav grav = init_grav(&run_params);
  state state_0;
  state state_1;
  state_0.t = 0;
  state_0.position.x = grav.earth_radius + 1;
  state_0.position.y = 0;
  state_0.position.z = 0;
  state_0.velocity.x = -2;
  state_0.velocity.y = 0;
  state_0.velocity.z = 0;

  state_1.t = 1;
  state_1.position.x = grav.earth_radius - 1;
  state_1.position.y = 0;
  state_1.position.z = 0;
  state_1.velocity.x = 0;
  state_1.velocity.y = 0;
  state_1.velocity.z = 0;

  state impact_state = impact_linterp(&state_0, &state_1);

  REQUIRE_EQ(impact_state.t, 0.5);
  REQUIRE_EQ(impact_state.position.x, grav.earth_radius);
  REQUIRE_EQ(impact_state.position.y, 0);
  REQUIRE_EQ(impact_state.position.z, 0);
  REQUIRE_EQ(impact_state.velocity.x, -1);
  REQUIRE_EQ(impact_state.velocity.y, 0);
  REQUIRE_EQ(impact_state.velocity.z, 0);
}

TEST(trajectory, fly) {
  // Initialize the random number generator

  vehicle vehicle = init_mock_vehicle();
  runparams run_params;
  // Set the run parameters
  run_params.run_name = "test_run";
  run_params.run_type = 0;
  run_params.traj_output = 0;
  run_params.time_step_midcourse = 1;
  run_params.time_step_reentry = 1;
  run_params.x_aim = EARTH_RADIUS_M;
  run_params.y_aim = 0;
  run_params.z_aim = 0;
  run_params.theta_long = 0;
  run_params.theta_lat = 0;

  run_params.rv_type = 0;
  run_params.grav_error = 0;
  run_params.atm_model = 0;
  run_params.gnss_nav = 0;
  run_params.ins_nav = 1;
  run_params.rv_maneuv = 0;
  run_params.initial_x_error = 0;
  run_params.initial_pos_error = 0;
  run_params.initial_vel_error = 0;
  run_params.initial_angle_error = 0;
  run_params.acc_scale_stability = 0;
  run_params.gyro_bias_stability = 0;
  run_params.gyro_noise = 0;
  run_params.gnss_noise = 0;

  // print all of the vehicle parameters
  // printf("Booster total mass: %f\n", vehicle.booster.total_mass);
  // Mock vehicle with no thrust dropped from 10m above the surface
  state initial_state = init_true_state(&run_params);
  initial_state.theta_long = 0;
  initial_state.position.x += 10;

  state final_state = fly(&run_params, &initial_state, &vehicle);

  REQUIRE_LT(fabs(final_state.t - 1), 1);
  REQUIRE_EQ(final_state.a_thrust.x, 0);
  REQUIRE_EQ(final_state.a_thrust.y, 0);
  REQUIRE_EQ(final_state.a_thrust.z, 0);

  // Mock vehicle with no thrust launched from the surface
  initial_state = init_true_state(&run_params);
  initial_state.theta_long = 0;
  initial_state.velocity.x = 10;
  initial_state.velocity.y = 10;
  initial_state.velocity.z = 10;
  final_state = fly(&run_params, &initial_state, &vehicle);

  REQUIRE_LT(fabs(final_state.t - 2), 1);
}