#include "../src/include/math/linalg.h"
#include "../src/include/utils/propnav.h"
#include <math.h>
#include <tau/tau.h>

// When the velocity is exactly toward the aimpoint, the
// proportional-navigation command should be (near) zero because there is no
// line-of-sight rotation.
TEST(guidance, prop_nav_radial_velocity_zero) {
  runparams run_params = {0};
  state estimated_state = init_true_state(&run_params);
  grav grav_model = init_grav(&run_params, 1);

  run_params.x_aim = EARTH_RADIUS_M;
  run_params.y_aim = 0;
  run_params.z_aim = 0;
  run_params.nav_gain_0 = 3.0;
  run_params.nav_gain_1 = 5.0;

  estimated_state.position.x = EARTH_RADIUS_M;
  estimated_state.position.y = 10000;
  estimated_state.position.z = 0;
  // Velocity directly toward aimpoint (radial)
  estimated_state.velocity.x = 0;
  estimated_state.velocity.y = -5000;
  estimated_state.velocity.z = 0;

  cartvec a_command = prop_nav(&estimated_state, &run_params, &grav_model);
  double a_norm = norm(a_command);
  REQUIRE_LT(a_norm, 1e-8);
}

// When the velocity is perpendicular to the line-of-sight, the closed-form
// expression gives a_command = N * r * |v|^2 / |r|^2 -> magnitude = N*|v|^2/|r|
// and the acceleration should be parallel to the displacement vector `r`.
TEST(guidance, prop_nav_perp_velocity_magnitude_and_direction) {
  runparams run_params = {0};
  state estimated_state = init_true_state(&run_params);
  grav grav_model = init_grav(&run_params, 1);

  run_params.x_aim = EARTH_RADIUS_M;
  run_params.y_aim = 0;
  run_params.z_aim = 0;
  run_params.nav_gain_0 = 3.0;
  run_params.nav_gain_1 = 5.0;

  estimated_state.position.x = EARTH_RADIUS_M;
  estimated_state.position.y = 10000;
  estimated_state.position.z = 0;
  // Velocity perpendicular to r (cross-track)
  estimated_state.velocity.x = 0;
  estimated_state.velocity.y = 0;
  estimated_state.velocity.z = 5000;

  cartvec a_command = prop_nav(&estimated_state, &run_params, &grav_model);

  cartvec r_target =
      subtract((cartvec){run_params.x_aim, run_params.y_aim, run_params.z_aim},
               estimated_state.position);
  cartvec v_rel = smultiply(estimated_state.velocity, -1.0);
  double r_norm = norm(r_target);
  double v_norm = norm(estimated_state.velocity);
  double gain = run_params.nav_gain_0 +
                (run_params.nav_gain_1 - run_params.nav_gain_0) / 120e3 *
                    get_altitude(estimated_state.position);

  cartvec rot = sdivide(cross(r_target, v_rel), dot(r_target, r_target));
  cartvec a_cmd_E = smultiply(cross(v_rel, rot), gain);
  double mag_a_cmd = norm(a_cmd_E);
  cartvec expected = a_cmd_E;
  if (mag_a_cmd >= 1e-9) {
    cartvec a_cmd_E_hat = sdivide(a_cmd_E, mag_a_cmd);
    cartvec a_grav_E = get_gravity_acc(&grav_model, &estimated_state);
    cartvec a_grav_E_perp = smultiply(a_cmd_E_hat, dot(a_grav_E, a_cmd_E_hat));
    a_grav_E_perp = smultiply(a_grav_E_perp, gain / 2.0);
    expected = subtract(a_cmd_E, a_grav_E_perp);
  }

  double a_norm = norm(a_command);
  double rel_err = fabs(a_norm - norm(expected)) / (norm(expected) + 1e-12);
  REQUIRE_LT(rel_err, 1e-6);

  cartvec delta = subtract(a_command, expected);
  REQUIRE_LT(norm(delta), 1e-8);

  cartvec cross_ra = cross(r_target, a_command);
  REQUIRE_LT(norm(cross_ra), 1e-8);
}