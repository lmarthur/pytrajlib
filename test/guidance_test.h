#include "../src/include/forces/lift.h"
#include <tau/tau.h>

TEST(guidance, prop_nav) {
  runparams run_params = {0};
  state estimated_state = init_true_state(&run_params);
  grav grav_model = init_grav(&run_params);

  // Set aimpoint near the equator at sea level
  run_params.x_aim = EARTH_RADIUS_M;
  run_params.y_aim = 0;
  run_params.z_aim = 0;
  run_params.nav_gain = 3.0; // Higher gain for more responsive guidance

  // Starting position 10 km north of aimpoint at altitude
  estimated_state.position.x = EARTH_RADIUS_M;
  estimated_state.position.y = 10000;
  estimated_state.position.z = 0;
  // Velocity pointing toward aimpoint with significant speed
  estimated_state.velocity.x = 0;
  estimated_state.velocity.y =
      -5000; // Higher velocity for ballistic trajectory
  estimated_state.velocity.z = 0;

  cartvec a_command = prop_nav(&estimated_state, &run_params, &grav_model);
  // Proportional navigation should produce a reasonable acceleration command
  REQUIRE_TRUE(isfinite(a_command.x));
  REQUIRE_TRUE(isfinite(a_command.y));
  REQUIRE_TRUE(isfinite(a_command.z));

  // Now add cross-track velocity to test guidance correction
  estimated_state.position.x = EARTH_RADIUS_M;
  estimated_state.position.y = 10000;
  estimated_state.position.z = 0;
  estimated_state.velocity.x = 0;
  estimated_state.velocity.y = -5000;
  estimated_state.velocity.z = 2000; // Cross-track velocity component

  a_command = prop_nav(&estimated_state, &run_params, &grav_model);
  // Command should be finite and reasonable for guiding back to target
  REQUIRE_TRUE(isfinite(a_command.x));
  REQUIRE_TRUE(isfinite(a_command.y));
  REQUIRE_TRUE(isfinite(a_command.z));
}