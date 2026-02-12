#include "../../src/include/forces/lift.h"
#include <tau/tau.h>

TEST(lift, update_lift) {
  // Initialize the state
  state true_state = {0};
  true_state.t = 1000;
  true_state.x = 6371e3 + 10;
  true_state.vx = -100;

  state est_state = true_state;

  // Initialize the run parameters
  runparams run_params;
  run_params.deflection_time = 0.1; // Time to deflect the lift vector (seconds)
  run_params.actuator_force = 100;
  // Initialize the vehicle
  vehicle vehicle;
  vehicle.rv = init_swerve_rv();
  vehicle.booster = init_mmiii_booster();
  vehicle.total_mass = vehicle.booster.total_mass + vehicle.rv.rv_mass;
  vehicle.current_mass = vehicle.total_mass;

  // Initialize the atmospheric conditions
  atm_cond atm_cond;
  atm_cond.altitude = 0;
  atm_cond.density = 1.225;
  atm_cond.meridional_wind = 0;
  atm_cond.zonal_wind = 0;
  atm_cond.vertical_wind = 0;

  // Initialize the acceleration command
  cart_vector a_command;
  a_command.x = 0;
  a_command.y = 0;
  a_command.z = 0;

  // Update the lift
  update_lift(&true_state, &est_state, &run_params, &atm_cond, &atm_cond,
              &vehicle, 0.1);
  // Verify that the lift is unchanged when the command is zero and the current
  // lift is zero
  REQUIRE_EQ(true_state.ax_lift, 0);
  REQUIRE_EQ(true_state.ay_lift, 0);
  REQUIRE_EQ(true_state.az_lift, 0);

  // Verify that the executed acceleration decays exponentially to the command
  a_command.x = 0;
  a_command.y = 0;
  a_command.z = 0;

  true_state.ax_lift = 1;
  true_state.ay_lift = 1;
  true_state.az_lift = 1;

  update_lift(&true_state, &est_state, &run_params, &atm_cond, &atm_cond,
              &vehicle, 0.1);
  update_lift(&true_state, &est_state, &run_params, &atm_cond, &atm_cond,
              &vehicle, 0.1);
  // Verify that the lift is updated correctly
  REQUIRE_LT(true_state.ax_lift, 1);
  REQUIRE_LT(true_state.ay_lift, 1);
  REQUIRE_LT(true_state.az_lift, 1);
}

TEST(lift, update_roll) {
  // Initialize the state
  state true_state = {0};
  true_state.t = 1000;
  true_state.x = 6371e3 + 10;
  true_state.vx = -100;

  state est_state = true_state;

  // Initialize the atmospheric conditions
  atm_cond atm_cond;
  atm_cond.altitude = 0;
  atm_cond.density = 1.225;
  atm_cond.meridional_wind = 0;
  atm_cond.zonal_wind = 0;
  atm_cond.vertical_wind = 0;

  // If pitch acceleration = 0, then the roll will be π/2 when yaw is positive
  // along yaw axis e1 = [-1, 0, 0], e2 = [0, 1, 0], e3 = [0, 0, -1]
  cart_vector d_a_lift_yaw_only;
  d_a_lift_yaw_only.x = 0;
  d_a_lift_yaw_only.y = 0;   // no pitch acceleration
  d_a_lift_yaw_only.z = -10; // yaw acceleration along e3 (z-axis)

  true_state.roll = 0;
  est_state.roll = 0;
  update_roll(&true_state, &est_state, d_a_lift_yaw_only, d_a_lift_yaw_only,
              &atm_cond, &atm_cond);
  REQUIRE_EQ(true_state.roll, M_PI / 2);
  REQUIRE_EQ(est_state.roll, M_PI / 2);

  // If yaw acceleration = 0, the roll must be 0
  cart_vector d_a_lift_pitch_only;
  d_a_lift_pitch_only.x = 0;
  d_a_lift_pitch_only.y = 10; // pitch acceleration along e2 (y-axis)
  d_a_lift_pitch_only.z = 0;  // no yaw acceleration

  true_state.roll = 0;
  est_state.roll = 0;
  update_roll(&true_state, &est_state, d_a_lift_pitch_only, d_a_lift_pitch_only,
              &atm_cond, &atm_cond);
  REQUIRE_EQ(fmod(true_state.roll, 2 * M_PI), 0);
  REQUIRE_EQ(fmod(est_state.roll, 2 * M_PI), 0);

  // If pitch and yaw are equal and >0, then the roll must be π/4
  cart_vector d_a_lift_equal_pos;
  d_a_lift_equal_pos.x = 0;
  d_a_lift_equal_pos.y = 10; // pitch acceleration
  d_a_lift_equal_pos.z =
      -10; // equal positive yaw acceleration (negative z is positive yaw)

  true_state.roll = 0;
  est_state.roll = 0;
  update_roll(&true_state, &est_state, d_a_lift_equal_pos, d_a_lift_equal_pos,
              &atm_cond, &atm_cond);
  REQUIRE_EQ(fmod(true_state.roll, 2 * M_PI), M_PI / 4);
  REQUIRE_EQ(fmod(est_state.roll, 2 * M_PI), M_PI / 4);

  // If pitch and yaw are equal and <0, then the roll must be 5π/4 (or -3π/4)
  cart_vector d_a_lift_equal_neg;
  d_a_lift_equal_neg.x = 0;
  d_a_lift_equal_neg.y = -10; // negative pitch acceleration
  d_a_lift_equal_neg.z = 10;  // equal negative yaw acceleration

  true_state.roll = 0;
  est_state.roll = 0;
  update_roll(&true_state, &est_state, d_a_lift_equal_neg, d_a_lift_equal_neg,
              &atm_cond, &atm_cond);
  REQUIRE_EQ(true_state.roll, 5 * M_PI / 4);
  REQUIRE_EQ(est_state.roll, 5 * M_PI / 4);

  // If pitch > 0 and yaw < 0 with equal magnitudes, then the roll will be -π/4 or 7π/4
  cart_vector d_a_lift_equal_neg2;
  d_a_lift_equal_neg2.x = 0;
  d_a_lift_equal_neg2.y = 10; // positive pitch acceleration
  d_a_lift_equal_neg2.z = 10;  // negative yaw acceleration (positive z gives negative yaw)

  true_state.roll = 0;
  est_state.roll = 0;
  update_roll(&true_state, &est_state, d_a_lift_equal_neg2, d_a_lift_equal_neg2,
              &atm_cond, &atm_cond);
  REQUIRE_EQ(true_state.roll, 7 * M_PI / 4);
  REQUIRE_EQ(est_state.roll, 7 * M_PI / 4);
}