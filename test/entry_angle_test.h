#define _USE_MATH_DEFINES

#include "../src/include/trajectory.h"
#include <math.h>
#include <tau/tau.h>

// Angle between two attitudes, in radians.
static double attitude_gap(quaternion a, quaternion b) {
  quaternion b_conj = {b.w, -b.x, -b.y, -b.z};
  quaternion rel = qmultiply(a, b_conj);
  return 2.0 * acos(fmin(1.0, fabs(rel.w)));
}

// The post-boost maneuver is a rotation the vehicle performs, not a teleport to
// a canonical attitude, so it must leave the navigation attitude error alone.
TEST(entry_angle, maneuver_preserves_the_attitude_error) {
  runparams rp = {0};
  rp.theta_long = 0.9320122849463917;
  rp.theta_lat = 0.0;
  rp.initial_angle_error = 1e-3;
  rp.x_aim = EARTH_RADIUS_M * cos(2.0405);
  rp.y_aim = EARTH_RADIUS_M * sin(2.0405);
  rp.z_aim = 0.0;
  rp.grav_error = 0;

  vehicle vehicle = {0};
  vehicle.rv.rv_mass = 517.609493782838;
  grav grav_model = init_grav(&rp, 1);

  for (int trial = 0; trial < 10; trial++) {
    state true_state = init_true_state(&rp);
    state est_state = init_est_state(&rp);

    // Representative burnout conditions: high, fast, climbing downrange.
    cartvec position = {5.0e6, 3.0e6, 0.0};
    cartvec velocity = {2.0e3, 5.0e3, 0.0};
    true_state.position = position;
    true_state.velocity = velocity;
    est_state.position = position;
    est_state.velocity = velocity;

    double before = attitude_gap(true_state.q_EB, est_state.q_EB);
    REQUIRE_LT(fabs(before - rp.initial_angle_error), 1e-9);

    set_entry_angle(&true_state, &est_state, &rp, &vehicle, &grav_model);

    double after = attitude_gap(true_state.q_EB, est_state.q_EB);
    REQUIRE_LT(fabs(after - before), 1e-9);
  }
}