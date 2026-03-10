#include "../src/include/integrator.h"
#include <tau/tau.h>

TEST(physics, update_mass) {
  // Define a state struct
  state state;
  // Define a vehicle struct
  vehicle vehicle;

  // Initialize the vehicle
  vehicle.booster = init_mmiii_booster();
  vehicle.rv = init_ballistic_rv();
  double time_step = 1;

  vehicle.total_mass = vehicle.booster.total_mass + vehicle.rv.rv_mass;
  state.t = 0;

  // Loop through the burn time of the booster
  for (int i = 0; i <= vehicle.booster.total_burn_time + time_step; i++) {
    state.t = i * time_step;

    update_mass(&vehicle, state.t);

    // Check that mass is always positive
    REQUIRE_GT(get_vehicle_mass(&vehicle, state.t), 0);

    // First time step
    if (state.t == time_step) {
      REQUIRE_EQ(get_vehicle_mass(&vehicle, state.t),
                 vehicle.total_mass -
                     time_step * vehicle.booster.fuel_burn_rate[0]);
    }

    // First stage separation
    if (state.t == vehicle.booster.burn_time[0] + time_step) {
      REQUIRE_EQ(get_vehicle_mass(&vehicle, state.t),
                 vehicle.total_mass - vehicle.booster.wet_mass[0] -
                     time_step * vehicle.booster.fuel_burn_rate[1]);
    }

    // Second stage separation
    if (state.t == vehicle.booster.burn_time[0] + vehicle.booster.burn_time[1] +
                       time_step) {
      REQUIRE_EQ(get_vehicle_mass(&vehicle, state.t),
                 vehicle.total_mass - vehicle.booster.wet_mass[0] -
                     vehicle.booster.wet_mass[1] -
                     time_step * vehicle.booster.fuel_burn_rate[2]);
    }

    // After separation
    if (state.t == vehicle.booster.total_burn_time + time_step) {
      REQUIRE_EQ(get_vehicle_mass(&vehicle, state.t), vehicle.rv.rv_mass);
    }
  }
}

TEST(physics, euler_maruyama_step) {
  state state;
  state.position = zeros();
  state.velocity = zeros();
  state.a_total = zeros();

  double time_step = 1;

  euler_maruyama_step(&state, time_step);

  // Check that the new position is zero
  REQUIRE_EQ(state.position.x, 0);
  REQUIRE_EQ(state.position.y, 0);
  REQUIRE_EQ(state.position.z, 0);

  // Check that the new velocity is zero
  REQUIRE_EQ(state.velocity.x, 0);
  REQUIRE_EQ(state.velocity.y, 0);
  REQUIRE_EQ(state.velocity.z, 0);

  // Check that the new acceleration is zero
  REQUIRE_EQ(state.a_total.x, 0);
  REQUIRE_EQ(state.a_total.y, 0);
  REQUIRE_EQ(state.a_total.z, 0);

  // Check that the new position is the time step times the velocity
  state.position = zeros();
  state.velocity.x = 1;
  state.velocity.y = 1;
  state.velocity.z = 1;

  euler_maruyama_step(&state, time_step);

  REQUIRE_EQ(state.position.x, time_step);
  REQUIRE_EQ(state.position.y, time_step);
  REQUIRE_EQ(state.position.z, time_step);

  // Check that the new velocity is the time step times the acceleration
  state.velocity = zeros();
  state.a_total.x = 1;
  state.a_total.y = 1;
  state.a_total.z = 1;

  euler_maruyama_step(&state, time_step);

  REQUIRE_EQ(state.velocity.x, time_step);
  REQUIRE_EQ(state.velocity.y, time_step);
  REQUIRE_EQ(state.velocity.z, time_step);

  // Check that the new position is the time step times the velocity plus 0.5
  // times the acceleration
  state.position = zeros();
  state.velocity.x = 1;
  state.velocity.y = 1;
  state.velocity.z = 1;
  state.a_total.x = 1;
  state.a_total.y = 1;
  state.a_total.z = 1;

  euler_maruyama_step(&state, time_step);

  REQUIRE_EQ(state.position.x, time_step + 0.5);
  REQUIRE_EQ(state.position.y, time_step + 0.5);
  REQUIRE_EQ(state.position.z, time_step + 0.5);

  // Check that the new velocity is the time step times the acceleration
  state.velocity = zeros();
  state.a_total.x = 2;
  state.a_total.y = 2;
  state.a_total.z = 2;

  euler_maruyama_step(&state, time_step);

  REQUIRE_EQ(state.velocity.x, time_step + 1);
  REQUIRE_EQ(state.velocity.y, time_step + 1);
  REQUIRE_EQ(state.velocity.z, time_step + 1);
}