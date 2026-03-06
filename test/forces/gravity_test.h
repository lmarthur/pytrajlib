#include "../../src/include/forces/gravity.h"
#include <tau/tau.h>

TEST(gravity, get_gravity_acc) {
  // Define a grav struct
  grav grav;
  // Define a state struct
  state state;
  runparams run_params;
  run_params.grav_error = 0;

  // Initialize the grav struct
  grav = init_grav(&run_params);

  // Initialize the state struct with the vehicle at one earth radius
  state.position.x = grav.earth_radius;
  state.position.y = 0;
  state.position.z = 0;

  // Update the gravity acceleration components
  get_gravity_acc(&grav, &state);

  // Check that the gravitational acceleration components are correct
  REQUIRE_LT(state.a_grav.x + 9.81, 0.01);
  REQUIRE_EQ(state.a_grav.y, 0);
  REQUIRE_EQ(state.a_grav.z, 0);

  // Move the vehicle to a different location
  state.position.x = grav.earth_radius / sqrt(2);
  state.position.y = grav.earth_radius / sqrt(2);
  state.position.z = 0;

  // Update the gravity acceleration components
  get_gravity_acc(&grav, &state);

  // Check that the gravitational acceleration components are correct
  double r = norm(state.position);
  double ar_grav = -grav.grav_const * grav.earth_mass / (r * r);
  double ar_grav_surface = ar_grav;
  REQUIRE_EQ(state.a_grav.x, ar_grav * state.position.x / r);
  REQUIRE_EQ(state.a_grav.y, ar_grav * state.position.y / r);
  REQUIRE_EQ(state.a_grav.z, ar_grav * state.position.z / r);
  REQUIRE_EQ(ar_grav, -norm(state.a_grav));

  // Move the vehicle to a different height
  state.position.x = grav.earth_radius / sqrt(2) + 1000;
  state.position.y = grav.earth_radius / sqrt(2) + 1000;
  state.position.z = 1000;

  // Update the gravity acceleration components
  get_gravity_acc(&grav, &state);

  // Check that the gravitational acceleration components are correct
  r = norm(state.position);
  ar_grav = -grav.grav_const * grav.earth_mass / (r * r);
  REQUIRE_LT(state.a_grav.x, 0);
  REQUIRE_LT(state.a_grav.y, 0);
  REQUIRE_LT(state.a_grav.z, 0);
  REQUIRE_GT(ar_grav, ar_grav_surface);
  REQUIRE_EQ(ar_grav, -norm(state.a_grav));
}
