#include "../../src/include/forces/lift.h"
#include <math.h>
#include <tau/tau.h>

TEST(lift, get_a_lift_mag) {
  runparams run_params = {0};
  state current_state = init_true_state(&run_params);
  current_state.velocity.x = 1000;
  current_state.alpha = 0.1;

  vehicle vehicle = {0};
  vehicle.rv = init_swerve_rv();

  atm_cond atm_cond = {0};
  atm_cond.density = 1.225;

  double a_lift_mag = get_a_lift_mag(&current_state, &vehicle, &atm_cond);

  REQUIRE_GT(a_lift_mag, 0);
  REQUIRE_TRUE(isfinite(a_lift_mag));
}

TEST(lift, get_lift_acc) {
  runparams run_params = {0};
  run_params.rv_maneuv = 1;
  run_params.theta_long = 0; // Initialize attitudes
  run_params.theta_lat = 0;

  state true_state = init_true_state(&run_params);
  state est_state = init_est_state(&run_params);
  true_state.position.x = EARTH_RADIUS_M + 50000; // 50 km altitude
  true_state.velocity.x = -5000; // Moving toward Earth for reentry
  true_state.velocity.y = 1000;  // Lateral velocity component
  true_state.velocity.z = -2000; // Downward velocity
  true_state.alpha = 0.1;
  est_state.position.x = EARTH_RADIUS_M + 50000;
  est_state.velocity.x = -5000;
  est_state.velocity.y = 1000;
  est_state.velocity.z = -2000;

  vehicle vehicle = {0};
  vehicle.rv = init_swerve_rv();

  atm_cond atm_cond = {0};
  atm_cond.density = 1.225;

  grav grav = init_grav(&run_params);
  cartvec a_lift = get_lift_acc(&true_state, &est_state, &run_params, &vehicle,
                                &atm_cond, 100.0, &grav);

  REQUIRE_TRUE(isfinite(a_lift.x));
  REQUIRE_TRUE(isfinite(a_lift.y));
  REQUIRE_TRUE(isfinite(a_lift.z));
  REQUIRE_GT(norm(a_lift), 0);
}

TEST(lift, get_aoa_angular_acceleration) {
  runparams run_params = {0};

  state current_state = init_true_state(&run_params);
  current_state.velocity.x = 1000;
  current_state.alpha = 0.05;
  current_state.d_alpha_dt = 0.01;
  current_state.deflection_angle = 0.02;

  vehicle vehicle = {0};
  vehicle.rv = init_swerve_rv();

  atm_cond atm_cond = {0};
  atm_cond.density = 1.225;

  double ddot_alpha = get_aoa_angular_acceleration(&current_state, &run_params,
                                                   &vehicle, &atm_cond, 100.0);

  REQUIRE_TRUE(isfinite(ddot_alpha));
}