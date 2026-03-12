#include "../../src/include/forces/gravity.h"
#include <tau/tau.h>

TEST(gravity, get_gravity_acc) {
  // Define a grav struct
  grav grav;
  // Define a state struct
  state state = {0};
  runparams run_params = {0};
  run_params.grav_error = 0;

  // Initialize the grav struct
  grav = init_grav(&run_params);

  // Initialize the state struct with the vehicle at one earth radius
  state.position.x = grav.earth_radius;
  state.position.y = 0;
  state.position.z = 0;

  // Update the gravity acceleration components
  cartvec a_grav = get_gravity_acc(&grav, &state);

  // Check that the gravitational acceleration components are correct
  REQUIRE_LT(fabs(a_grav.x + 9.81), 0.01);
  REQUIRE_LT(fabs(a_grav.y), 1e-12);
  REQUIRE_LT(fabs(a_grav.z), 1e-12);

  // Move the vehicle to a different location
  state.position.x = grav.earth_radius / sqrt(2);
  state.position.y = grav.earth_radius / sqrt(2);
  state.position.z = 0;

  // Update the gravity acceleration components
  a_grav = get_gravity_acc(&grav, &state);

  // Check that the gravitational acceleration components are correct
  double r = norm(state.position);
  double ar_grav = grav.grav_g0 * pow(grav.earth_radius, 2) / (r * r);
  double ar_grav_surface = ar_grav;
  REQUIRE_LT(fabs(a_grav.x - ar_grav * state.position.x / r), 1e-12);
  REQUIRE_LT(fabs(a_grav.y - ar_grav * state.position.y / r), 1e-12);
  REQUIRE_LT(fabs(a_grav.z - ar_grav * state.position.z / r), 1e-12);
  REQUIRE_LT(fabs(ar_grav + norm(a_grav)), 1e-12);

  // Move the vehicle to a different height
  state.position.x = grav.earth_radius / sqrt(2) + 1000;
  state.position.y = grav.earth_radius / sqrt(2) + 1000;
  state.position.z = 1000;

  // Update the gravity acceleration components
  a_grav = get_gravity_acc(&grav, &state);

  // Check that the gravitational acceleration components are correct
  r = norm(state.position);
  ar_grav = grav.grav_g0 * pow(grav.earth_radius, 2) / (r * r);
  REQUIRE_LT(a_grav.x, 0);
  REQUIRE_LT(a_grav.y, 0);
  REQUIRE_LT(a_grav.z, 0);
  REQUIRE_GT(ar_grav, ar_grav_surface);
  REQUIRE_LT(fabs(ar_grav + norm(a_grav)), 1e-12);
}
