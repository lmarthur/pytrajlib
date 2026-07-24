#include "../../src/include/physics/drag.h"
#include <tau/tau.h>

static inline rv init_simple_rv(void) {
  rv simple_rv = {0};
  simple_rv.rv_mass = 1000.0;
  simple_rv.rv_area = 1.0;
  simple_rv.c_d_0 = 0.5;
  return simple_rv;
}

static inline booster init_simple_booster(void) {
  booster simple_booster = {0};
  simple_booster.num_stages = 1;
  simple_booster.area = 1.0;
  simple_booster.c_d_0 = 0.5;
  simple_booster.wet_mass[0] = 1000.0;
  simple_booster.fuel_mass[0] = 500.0;
  simple_booster.dry_mass[0] = 500.0;
  simple_booster.isp0[0] = 250.0;
  simple_booster.burn_time[0] = 10.0;
  simple_booster.fuel_burn_rate[0] = 50.0;
  simple_booster.total_burn_time = 10.0;
  simple_booster.total_mass = 1000.0;
  return simple_booster;
}

TEST(drag, get_drag_acc) {
  vehicle vehicle;
  vehicle.rv = init_simple_rv();
  vehicle.booster = init_simple_booster();
  vehicle.total_mass = vehicle.booster.total_mass + vehicle.rv.rv_mass;
  atm_cond atm_cond;
  state state = {0};
  runparams run_params = {0};
  run_params.grav_error = 0;

  grav grav = init_grav(&run_params, 1);
  atm_model atm_model = init_exp_atm(&run_params);

  atm_cond = get_exp_atm_cond(0, &atm_model);

  state.position.x = grav.earth_radius;
  state.position.y = 0;
  state.position.z = 0;
  state.velocity = zeros();

  cartvec a_drag = get_drag_acceleration_generic(
      0.0, &state, &atm_cond, &vehicle, vehicle.rv.c_d_0, vehicle.rv.rv_area);

  // Check that the drag acceleration components are zero
  REQUIRE_LT(fabs(a_drag.x), 1e-6);
  REQUIRE_LT(fabs(a_drag.y), 1e-6);
  REQUIRE_LT(fabs(a_drag.z), 1e-6);

  // Check that for wind and velocity in the same direction, drag is zero
  atm_cond.vertical_wind = 1;
  atm_cond.zonal_wind = 1;
  atm_cond.meridional_wind = 1;
  state.velocity.x = 1;
  state.velocity.y = 1;
  state.velocity.z = 1;

  a_drag = get_drag_acceleration_generic(0.0, &state, &atm_cond, &vehicle,
                                         vehicle.rv.c_d_0, vehicle.rv.rv_area);

  REQUIRE_LT(fabs(a_drag.x), 1e-6);
  REQUIRE_LT(fabs(a_drag.y), 1e-6);
  REQUIRE_LT(fabs(a_drag.z), 1e-6);

  // Check that for wind and velocity in opposite directions, drag is non-zero
  atm_cond.vertical_wind = -1;
  atm_cond.zonal_wind = -1;
  atm_cond.meridional_wind = -1;
  state.velocity.x = 1;
  state.velocity.y = 1;
  state.velocity.z = 1;

  a_drag = get_drag_acceleration_generic(0.0, &state, &atm_cond, &vehicle,
                                         vehicle.rv.c_d_0, vehicle.rv.rv_area);

  REQUIRE_GT(fabs(a_drag.x), 1e-6);
  REQUIRE_GT(fabs(a_drag.y), 1e-6);
  REQUIRE_GT(fabs(a_drag.z), 1e-6);

  // Check that for no wind, drag is only in the opposite direction of velocity
  atm_cond.vertical_wind = 0;
  atm_cond.zonal_wind = 0;
  atm_cond.meridional_wind = 0;

  state.velocity.x = 100;
  state.velocity.y = 0;
  state.velocity.z = 0;

  a_drag = get_drag_acceleration_generic(0.0, &state, &atm_cond, &vehicle,
                                         vehicle.rv.c_d_0, vehicle.rv.rv_area);

  REQUIRE_LT(a_drag.x, 0);
  REQUIRE_LT(fabs(a_drag.y), 1e-12);
  REQUIRE_LT(fabs(a_drag.z), 1e-12);
}
