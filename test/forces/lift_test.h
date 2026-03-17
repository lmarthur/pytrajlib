#include "../../src/include/forces/lift.h"
#include <tau/tau.h>

TEST(lift, get_a_lift_avail_jerk) {
  state est_state = {0};
  est_state.position.x = EARTH_RADIUS_M + 50e3;
  est_state.velocity.x = -4000;
  est_state.velocity.y = 100;

  runparams run_params = {0};
  run_params.deflection_time = 0.02;
  run_params.actuator_force = 100;
  run_params.gearing_ratio = 1;
  run_params.nav_gain = 5;
  run_params.flap_gain = 100;
  run_params.x_aim = EARTH_RADIUS_M * cos(M_PI / 6);
  run_params.y_aim = EARTH_RADIUS_M * sin(M_PI / 6);
  run_params.z_aim = 0;

  vehicle vehicle;
  vehicle.rv = init_swerve_rv();
  vehicle.booster = init_mmiii_booster();
  vehicle.total_mass = vehicle.booster.total_mass + vehicle.rv.rv_mass;

  atm_cond atm_cond = {0};
  atm_cond.altitude = 50e3;
  atm_cond.density = 1.225;

  double est_t = 100.0;
  cartvec d_a_lift_avail_dt = get_a_lift_avail_jerk(&est_state, &run_params,
                                                    &vehicle, &atm_cond, est_t);

  double jerk_max = get_jerk_max(&run_params, &vehicle);
  cartvec est_e1, est_e2, est_e3;
  int valid_basis =
      get_body_frame(&est_state, &atm_cond, &est_e1, &est_e2, &est_e3, 0);
  REQUIRE_EQ(valid_basis, 1);

  REQUIRE_LE(fabs(d_a_lift_avail_dt.x), jerk_max);
  REQUIRE_LE(fabs(d_a_lift_avail_dt.y), jerk_max);
  REQUIRE_LE(fabs(d_a_lift_avail_dt.z), jerk_max);
  REQUIRE_LT(fabs(dot(d_a_lift_avail_dt, est_e1)), 1e-10);

  double non_reentry_t = 0.0;
  cartvec zero_jerk = get_a_lift_avail_jerk(&est_state, &run_params, &vehicle,
                                            &atm_cond, non_reentry_t);
  REQUIRE_EQ(zero_jerk.x, 0);
  REQUIRE_EQ(zero_jerk.y, 0);
  REQUIRE_EQ(zero_jerk.z, 0);
}

TEST(lift, get_a_lift_jerk) {
  state current_state = {0};
  current_state.position.x = EARTH_RADIUS_M + 50e3;
  current_state.velocity.x = -4000;
  current_state.velocity.y = 100;

  runparams run_params = {0};
  run_params.deflection_time = 0.02;
  run_params.actuator_force = 100;
  run_params.gearing_ratio = 1;

  vehicle vehicle;
  vehicle.rv = init_swerve_rv();
  vehicle.booster = init_mmiii_booster();
  vehicle.total_mass = vehicle.booster.total_mass + vehicle.rv.rv_mass;

  atm_cond atm_cond = {0};
  atm_cond.altitude = 50e3;
  atm_cond.density = 1.225;

  double acc_resolution = get_acc_resolution(&run_params, &vehicle);
  current_state.a_lift_avail.z = 3 * acc_resolution;
  current_state.a_lift.z = 3 * acc_resolution;

  double t = 100.0;
  cartvec zero_jerk =
      get_a_lift_jerk(&current_state, &run_params, &vehicle, &atm_cond, t);
  REQUIRE_LT(fabs(zero_jerk.x), 1e-12);
  REQUIRE_LT(fabs(zero_jerk.y), 1e-12);
  REQUIRE_LT(fabs(zero_jerk.z), 1e-12);

  current_state.a_lift = zeros();
  cartvec d_a_lift_dt =
      get_a_lift_jerk(&current_state, &run_params, &vehicle, &atm_cond, t);

  cartvec e1, e2, e3;
  int valid_basis = get_body_frame(&current_state, &atm_cond, &e1, &e2, &e3, 0);
  REQUIRE_EQ(valid_basis, 1);

  REQUIRE_GT(norm(d_a_lift_dt), 0);
  REQUIRE_LT(fabs(dot(d_a_lift_dt, e1)), 1e-10);

  cartvec no_reentry =
      get_a_lift_jerk(&current_state, &run_params, &vehicle, &atm_cond, 0.0);
  REQUIRE_EQ(no_reentry.x, 0);
  REQUIRE_EQ(no_reentry.y, 0);
  REQUIRE_EQ(no_reentry.z, 0);
}