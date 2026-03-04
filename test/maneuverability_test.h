#include "../src/include/forces/lift.h"
#include <tau/tau.h>

TEST(maneuverability, rv_time_constant) {
  // Initialize the state
  state true_state;
  true_state.position.x = EARTH_RADIUS_M + 10;
  true_state.position.y = 0;
  true_state.position.z = 0;
  true_state.theta_lat = 0;
  true_state.velocity.x = -1;
  true_state.velocity.y = 0;
  true_state.velocity.z = 0;
  true_state.a_grav = zeros();
  true_state.a_drag = zeros();
  true_state.a_lift = zeros();
  true_state.a_thrust = zeros();

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
  true_state.velocity.x = -10;
  double time_constant_1 = rv_time_constant(&true_state, &atm_cond, &vehicle);

  REQUIRE_GT(time_constant_1, 0);
  REQUIRE_NE(time_constant_0, time_constant_1);
}