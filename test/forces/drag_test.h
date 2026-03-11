#include <tau/tau.h>

TEST(drag, get_drag_acc) {
  vehicle vehicle;
  vehicle.rv = init_ballistic_rv();
  vehicle.booster = init_mmiii_booster();
  vehicle.total_mass = vehicle.booster.total_mass + vehicle.rv.rv_mass;
  atm_cond atm_cond;
  state state;
  runparams run_params;
  run_params.grav_error = 0;
  run_params.cl_pert =
      0; // Set to zero for this test, as we are only testing drag

  grav grav = init_grav(&run_params);
  atm_model atm_model = init_exp_atm(&run_params);

  atm_cond = get_exp_atm_cond(0, &atm_model);

  state.t = 1;
  state.position.x = grav.earth_radius;
  state.position.y = 0;
  state.position.z = 0;
  state.velocity = zeros();

  get_drag_acc(&run_params, &vehicle, &atm_cond, &state, 1.0);

  // Check that the drag acceleration components are zero
  REQUIRE_LT(state.a_drag.x, 1e-6);
  REQUIRE_LT(state.a_drag.y, 1e-6);
  REQUIRE_LT(state.a_drag.z, 1e-6);

  // Check that for wind and velocity in the same direction, drag is zero
  atm_cond.vertical_wind = 1;
  atm_cond.zonal_wind = 1;
  atm_cond.meridional_wind = 1;
  state.velocity.x = 1;
  state.velocity.y = 1;
  state.velocity.z = 1;

  get_drag_acc(&run_params, &vehicle, &atm_cond, &state, 1.0);

  REQUIRE_LT(state.a_drag.x, 1e-6);
  REQUIRE_LT(state.a_drag.y, 1e-6);
  REQUIRE_LT(state.a_drag.z, 1e-6);

  // Check that for wind and velocity in opposite directions, drag is non-zero
  atm_cond.vertical_wind = -1;
  atm_cond.zonal_wind = -1;
  atm_cond.meridional_wind = -1;
  state.velocity.x = 1;
  state.velocity.y = 1;
  state.velocity.z = 1;

  get_drag_acc(&run_params, &vehicle, &atm_cond, &state, 1.0);

  REQUIRE_NE(state.a_drag.x, 0);
  REQUIRE_NE(state.a_drag.y, 0);
  REQUIRE_NE(state.a_drag.z, 0);

  // Check that for no wind, drag is only in the opposite direction of velocity
  atm_cond.vertical_wind = 0;
  atm_cond.zonal_wind = 0;
  atm_cond.meridional_wind = 0;

  state.velocity.x = 100;
  state.velocity.y = 0;
  state.velocity.z = 0;

  get_drag_acc(&run_params, &vehicle, &atm_cond, &state, 1.0);

  REQUIRE_LT(state.a_drag.x, 0);
  REQUIRE_EQ(state.a_drag.y, 0);
  REQUIRE_EQ(state.a_drag.z, 0);
}
