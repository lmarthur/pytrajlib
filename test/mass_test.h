#include "../src/include/models/vehicle.h"
#include <tau/tau.h>

static inline booster init_test_booster(void) {
  booster booster = {0};
  booster.num_stages = 3;
  booster.wet_mass[0] = 100.0;
  booster.wet_mass[1] = 80.0;
  booster.wet_mass[2] = 60.0;
  booster.burn_time[0] = 1.0;
  booster.burn_time[1] = 2.0;
  booster.burn_time[2] = 3.0;
  booster.fuel_burn_rate[0] = booster.wet_mass[0] / booster.burn_time[0];
  booster.fuel_burn_rate[1] = booster.wet_mass[1] / booster.burn_time[1];
  booster.fuel_burn_rate[2] = booster.wet_mass[2] / booster.burn_time[2];
  booster.total_burn_time =
      booster.burn_time[0] + booster.burn_time[1] + booster.burn_time[2];
  booster.total_mass =
      booster.wet_mass[0] + booster.wet_mass[1] + booster.wet_mass[2];
  return booster;
}

static inline rv init_test_rv(void) {
  rv rv = {0};
  rv.rv_mass = 50.0;
  return rv;
}

TEST(mass, get_vehicle_mass) {
  vehicle vehicle;
  vehicle.booster = init_test_booster();
  vehicle.rv = init_test_rv();
  vehicle.total_mass = vehicle.booster.total_mass + vehicle.rv.rv_mass;

  double time_step = 1;

  for (int i = 0; i <= vehicle.booster.total_burn_time + time_step; i++) {
    double t = i * time_step;

    REQUIRE_GT(get_vehicle_mass(&vehicle, t), 0);

    if (t == time_step) {
      REQUIRE_EQ(get_vehicle_mass(&vehicle, t),
                 vehicle.total_mass -
                     time_step * vehicle.booster.fuel_burn_rate[0]);
    }

    if (t == vehicle.booster.burn_time[0] + time_step) {
      REQUIRE_EQ(get_vehicle_mass(&vehicle, t),
                 vehicle.total_mass - vehicle.booster.wet_mass[0] -
                     time_step * vehicle.booster.fuel_burn_rate[1]);
    }

    if (t == vehicle.booster.burn_time[0] + vehicle.booster.burn_time[1] +
                 time_step) {
      REQUIRE_EQ(get_vehicle_mass(&vehicle, t),
                 vehicle.total_mass - vehicle.booster.wet_mass[0] -
                     vehicle.booster.wet_mass[1] -
                     time_step * vehicle.booster.fuel_burn_rate[2]);
    }

    if (t == vehicle.booster.total_burn_time + time_step) {
      REQUIRE_EQ(get_vehicle_mass(&vehicle, t), vehicle.rv.rv_mass);
    }
  }
}
