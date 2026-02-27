#include "../src/include/physics.h"
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
  vehicle.current_mass = vehicle.total_mass;
  state.t = 0;

  // Loop through the burn time of the booster
  for (int i = 0; i <= vehicle.booster.total_burn_time + time_step; i++) {
    state.t = i * time_step;

    update_mass(&vehicle, state.t);

    // Check that mass is always positive
    REQUIRE_GT(vehicle.current_mass, 0);

    // First time step
    if (state.t == time_step) {
      REQUIRE_EQ(vehicle.current_mass,
                 vehicle.total_mass -
                     time_step * vehicle.booster.fuel_burn_rate[0]);
    }

    // First stage separation
    if (state.t == vehicle.booster.burn_time[0] + time_step) {
      REQUIRE_EQ(vehicle.current_mass,
                 vehicle.total_mass - vehicle.booster.wet_mass[0] -
                     time_step * vehicle.booster.fuel_burn_rate[1]);
    }

    // Second stage separation
    if (state.t == vehicle.booster.burn_time[0] + vehicle.booster.burn_time[1] +
                       time_step) {
      REQUIRE_EQ(vehicle.current_mass,
                 vehicle.total_mass - vehicle.booster.wet_mass[0] -
                     vehicle.booster.wet_mass[1] -
                     time_step * vehicle.booster.fuel_burn_rate[2]);
    }

    // After separation
    if (state.t == vehicle.booster.total_burn_time + time_step) {
      REQUIRE_EQ(vehicle.current_mass, vehicle.rv.rv_mass);
    }
  }
}

TEST(physics, rk4step) {
  state state;
  state.x = 0;
  state.y = 0;
  state.z = 0;
  state.vx = 0;
  state.vy = 0;
  state.vz = 0;
  state.ax_total = 0;
  state.ay_total = 0;
  state.az_total = 0;

  double time_step = 1;

  rk4step(&state, time_step);

  // Check that the new position is zero
  REQUIRE_EQ(state.x, 0);
  REQUIRE_EQ(state.y, 0);
  REQUIRE_EQ(state.z, 0);

  // Check that the new velocity is zero
  REQUIRE_EQ(state.vx, 0);
  REQUIRE_EQ(state.vy, 0);
  REQUIRE_EQ(state.vz, 0);

  // Check that the new acceleration is zero
  REQUIRE_EQ(state.ax_total, 0);
  REQUIRE_EQ(state.ay_total, 0);
  REQUIRE_EQ(state.az_total, 0);

  // Check that the new position is the time step times the velocity
  state.x = 0;
  state.y = 0;
  state.z = 0;
  state.vx = 1;
  state.vy = 1;
  state.vz = 1;

  rk4step(&state, time_step);

  REQUIRE_EQ(state.x, time_step);
  REQUIRE_EQ(state.y, time_step);
  REQUIRE_EQ(state.z, time_step);

  // Check that the new velocity is the time step times the acceleration
  state.vx = 0;
  state.vy = 0;
  state.vz = 0;
  state.ax_total = 1;
  state.ay_total = 1;
  state.az_total = 1;

  rk4step(&state, time_step);

  REQUIRE_EQ(state.vx, time_step);
  REQUIRE_EQ(state.vy, time_step);
  REQUIRE_EQ(state.vz, time_step);

  // Check that the new position is the time step times the velocity plus 0.5
  // times the acceleration
  state.x = 0;
  state.y = 0;
  state.z = 0;
  state.vx = 1;
  state.vy = 1;
  state.vz = 1;
  state.ax_total = 1;
  state.ay_total = 1;
  state.az_total = 1;

  rk4step(&state, time_step);

  REQUIRE_EQ(state.x, time_step + 0.5);
  REQUIRE_EQ(state.y, time_step + 0.5);
  REQUIRE_EQ(state.z, time_step + 0.5);

  // Check that the new velocity is the time step times the acceleration
  state.vx = 0;
  state.vy = 0;
  state.vz = 0;
  state.ax_total = 2;
  state.ay_total = 2;
  state.az_total = 2;

  rk4step(&state, time_step);

  REQUIRE_EQ(state.vx, time_step + 1);
  REQUIRE_EQ(state.vy, time_step + 1);
  REQUIRE_EQ(state.vz, time_step + 1);
}