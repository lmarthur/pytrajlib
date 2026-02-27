#include "../src/include/maneuverability.h"
#include <tau/tau.h>

TEST(maneuverability, perfect_maneuv) {
  // initialize the true state
  state true_state;
  true_state.x = 6371e3 + 10;
  true_state.y = 0;
  true_state.z = 0;
  true_state.theta_lat = 0;
  true_state.theta_long = 0;
  true_state.vx = -1;
  true_state.vy = 1;
  true_state.vz = 0;
  true_state.ax_grav = 0;
  true_state.ay_grav = 0;
  true_state.az_grav = 0;
  true_state.ax_drag = 0;
  true_state.ay_drag = 0;
  true_state.az_drag = 0;
  true_state.ax_lift = 0;
  true_state.ay_lift = 0;
  true_state.az_lift = 0;
  true_state.ax_thrust = 0;
  true_state.ay_thrust = 0;
  true_state.az_thrust = 0;
  true_state.ax_total = 0;
  true_state.ay_total = 0;
  true_state.az_total = 0;

  state estimated_state = true_state;
  state desired_state = true_state;

  // For desired = estimated, the true state should not change
  state new_true_state =
      perfect_maneuv(&true_state, &estimated_state, &desired_state);

  REQUIRE_EQ(new_true_state.x, true_state.x);
  REQUIRE_EQ(new_true_state.y, true_state.y);
  REQUIRE_EQ(new_true_state.z, true_state.z);
  REQUIRE_EQ(new_true_state.vx, true_state.vx);
  REQUIRE_EQ(new_true_state.vy, true_state.vy);
  REQUIRE_EQ(new_true_state.vz, true_state.vz);
  REQUIRE_EQ(new_true_state.ax_grav, true_state.ax_grav);
  REQUIRE_EQ(new_true_state.ay_grav, true_state.ay_grav);
  REQUIRE_EQ(new_true_state.az_grav, true_state.az_grav);
  REQUIRE_EQ(new_true_state.ax_drag, true_state.ax_drag);
  REQUIRE_EQ(new_true_state.ay_drag, true_state.ay_drag);
  REQUIRE_EQ(new_true_state.az_drag, true_state.az_drag);
  REQUIRE_EQ(new_true_state.ax_lift, true_state.ax_lift);
  REQUIRE_EQ(new_true_state.ay_lift, true_state.ay_lift);
  REQUIRE_EQ(new_true_state.az_lift, true_state.az_lift);
  REQUIRE_EQ(new_true_state.ax_thrust, true_state.ax_thrust);
  REQUIRE_EQ(new_true_state.ay_thrust, true_state.ay_thrust);
  REQUIRE_EQ(new_true_state.az_thrust, true_state.az_thrust);
  REQUIRE_EQ(new_true_state.ax_total, true_state.ax_total);
  REQUIRE_EQ(new_true_state.ay_total, true_state.ay_total);
  REQUIRE_EQ(new_true_state.az_total, true_state.az_total);

  // For a drag acceleration offset, the true state should change
  desired_state.ax_drag = 1;
  desired_state.ay_drag = 1;
  desired_state.az_drag = 1;
  desired_state.ax_total = desired_state.ax_grav + desired_state.ax_drag +
                           desired_state.ax_lift + desired_state.ax_thrust;
  desired_state.ay_total = desired_state.ay_grav + desired_state.ay_drag +
                           desired_state.ay_lift + desired_state.ay_thrust;
  desired_state.az_total = desired_state.az_grav + desired_state.az_drag +
                           desired_state.az_lift + desired_state.az_thrust;

  new_true_state =
      perfect_maneuv(&true_state, &estimated_state, &desired_state);

  REQUIRE_NE(new_true_state.ax_total, true_state.ax_total);
  REQUIRE_NE(new_true_state.ay_total, true_state.ay_total);
  REQUIRE_NE(new_true_state.az_total, true_state.az_total);
}

TEST(maneuverability, rv_time_constant) {
  // Initialize the state
  state true_state;
  true_state.x = 6371e3 + 10;
  true_state.y = 0;
  true_state.z = 0;
  true_state.theta_lat = 0;
  true_state.vx = -1;
  true_state.vy = 0;
  true_state.vz = 0;
  true_state.ax_grav = 0;
  true_state.ay_grav = 0;
  true_state.az_grav = 0;
  true_state.ax_drag = 0;
  true_state.ay_drag = 0;
  true_state.az_drag = 0;
  true_state.ax_lift = 0;
  true_state.ay_lift = 0;
  true_state.az_lift = 0;
  true_state.ax_thrust = 0;
  true_state.ay_thrust = 0;
  true_state.az_thrust = 0;

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

  // Get the time constant
  double time_constant_0 = rv_time_constant(&true_state, &atm_cond, &vehicle);

  // Verify that the time constant is correct
  REQUIRE_GT(time_constant_0, 0);

  // For a different state, the time constant should be different
  true_state.vx = -10;
  double time_constant_1 = rv_time_constant(&true_state, &atm_cond, &vehicle);

  REQUIRE_GT(time_constant_1, 0);
  REQUIRE_NE(time_constant_0, time_constant_1);
}