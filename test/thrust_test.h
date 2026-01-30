#include "../src/include/forces/thrust.h"
#include "../src/include/integrator/args.h"
#include "../src/include/models/state.h"
#include "../src/include/models/vehicle.h"
#include <tau/tau.h>

TEST(physics, get_thrust_acceleration) {
  runparams run_params = {0};
  run_params.theta_lat = 0.1;
  run_params.theta_long = 0.2;
  run_params.init_thrust_lat_pert = 0.01;
  run_params.init_thrust_lon_pert = 0.02;

  vehicle vehicle = init_mmiii_ballistic(run_params);
  state state;

  integrator_args args;
  args.vehicle = vehicle;
  args.run_params = run_params;

  // Check that the thrust acceleration components are along the x-axis at t=0
  cartvec a_thrust = get_thrust_acceleration(0, state, args);
  REQUIRE_GT(a_thrust.x, 0);
  REQUIRE_EQ(a_thrust.y, 0);
  REQUIRE_EQ(a_thrust.z, 0);

  // Check that the thrust acceleration components are along the x-axis at t=1
  a_thrust = get_thrust_acceleration(1, state, args);
  REQUIRE_GT(a_thrust.x, 0);
  REQUIRE_EQ(a_thrust.y, 0);
  REQUIRE_EQ(a_thrust.z, 0);

  // Check that the thrust acceleration components are zero after the burn time
  a_thrust =
      get_thrust_acceleration(vehicle.booster.total_burn_time + 1, state, args);
  REQUIRE_EQ(a_thrust.x, 0);
  REQUIRE_EQ(a_thrust.y, 0);
  REQUIRE_EQ(a_thrust.z, 0);

  // Check that the thrust acceleration at time t + 1 is greater than at time t
  cartvec a_thrust_t2 = get_thrust_acceleration(2, state, args);
  cartvec a_thrust_t3 = get_thrust_acceleration(3, state, args);
  REQUIRE_GT(norm(a_thrust_t3), norm(a_thrust_t2));

  // Perform checks across full booster burn
  for (int i = 0; i <= vehicle.booster.total_burn_time + 1; i++) {
    a_thrust = get_thrust_acceleration(i, state, args);

    // Check that the thrust acceleration components are zero after the burn
    // time
    if (i > vehicle.booster.total_burn_time) {
      REQUIRE_EQ(a_thrust.x, 0);
      REQUIRE_EQ(a_thrust.y, 0);
      REQUIRE_EQ(a_thrust.z, 0);
    }

    // Check that the thrust acceleration components do not exceed 10^3 m/s^2
    REQUIRE_LT(fabs(a_thrust.x), 1e3);
    REQUIRE_LT(fabs(a_thrust.y), 1e3);
    REQUIRE_LT(fabs(a_thrust.z), 1e3);
  }
}
