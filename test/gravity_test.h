#include "../src/include/models/grav.h"
#include <tau/tau.h>

TEST(gravity, init_grav) {
  // No gravitational error case
  runparams run_params;
  run_params.grav_error = 0;

  grav grav = init_grav(run_params);

  REQUIRE_GT(grav.earth_mass, 0);
  REQUIRE_GT(grav.earth_radius, 0);
  REQUIRE_GT(grav.grav_const, 0);
  REQUIRE_NE(grav.geoid_height_std, 0);
  REQUIRE_EQ(grav.geoid_height_error, 0);

  // Gravitational error case
  run_params.grav_error = 1;

  grav = init_grav(run_params);

  REQUIRE_NE(grav.geoid_height_std, 0);
  REQUIRE_NE(grav.geoid_height_error, 0);
}