#define _USE_MATH_DEFINES

#include "../../src/include/physics/thrust.h"
#include <tau/tau.h>

TEST(thrust, get_central_angle) {
  cartvec position = {1.0, 0.0, 0.0};
  cartvec same = {1.0, 0.0, 0.0};
  cartvec orthogonal = {0.0, 1.0, 0.0};
  cartvec opposite = {-1.0, 0.0, 0.0};

  double phi_same = get_central_angle(position, same);
  double phi_orthogonal = get_central_angle(position, orthogonal);
  double phi_opposite = get_central_angle(position, opposite);

  REQUIRE_LT(fabs(phi_same - 0.0), 1e-12);
  REQUIRE_LT(fabs(phi_orthogonal - M_PI / 2), 1e-12);
  REQUIRE_LT(fabs(phi_opposite - M_PI), 1e-12);

  double earth_radius = EARTH_RADIUS_M;
  cartvec position2 = {earth_radius, 0, 0.0};
  cartvec aimpoint = {earth_radius * cos(M_PI / 4),
                      earth_radius * sin(M_PI / 4), 0.0};
  double phi = get_central_angle(position2, aimpoint);
  REQUIRE_LT(fabs(phi - M_PI_4), 1e-12);
}

TEST(thrust, get_lambert_velocity_vector) {
  double earth_radius = EARTH_RADIUS_M;
  cartvec position = {earth_radius, 0, 0.0};
  cartvec aimpoint = {earth_radius * cos(M_PI / 4),
                      earth_radius * sin(M_PI / 4), 0.0};
  double tf_des = 1000.0; // seconds
  runparams rp;
  rp.grav_error = 0;
  grav grav_model = init_grav(&rp, 1);

  cartvec v_lambert =
      get_lambert_velocity_vector(position, aimpoint, tf_des, &grav_model, &rp);
  double expected_vx = 7549.722571 * 0.3048;
  double expected_vy = 18391.612895 * 0.3048;

  REQUIRE_LT(fabs(v_lambert.x - expected_vx), 1.0);
  REQUIRE_LT(fabs(v_lambert.y - expected_vy), 1.0);
  REQUIRE_LT(fabs(v_lambert.z), 1e-9);
}

TEST(thrust, get_lambert_velocity_vector_altitude) {
  double earth_radius = EARTH_RADIUS_M;
  cartvec position = {earth_radius + 100e3, 0, 0.0};
  cartvec aimpoint = {earth_radius * cos(M_PI / 4),
                      earth_radius * sin(M_PI / 4), 0.0};
  double tf_des = 1000.0; // seconds
  runparams rp;
  rp.grav_error = 0;
  grav grav_model = init_grav(&rp, 1);

  cartvec v_lambert =
      get_lambert_velocity_vector(position, aimpoint, tf_des, &grav_model, &rp);
  double expected_vx = 6993.352308 * 0.3048;
  double expected_vy = 18322.670437 * 0.3048;
  printf("Lambert velocity at altitude x %f y %f z %f\n", v_lambert.x,
         v_lambert.y, v_lambert.z);

  REQUIRE_LT(fabs(v_lambert.x - expected_vx), 1.0);
  REQUIRE_LT(fabs(v_lambert.y - expected_vy), 1.0);
  REQUIRE_LT(fabs(v_lambert.z), 1e-9);
}

TEST(thrust, time_to_fly_positive_across_bracket) {
  /* Flight time must stay positive over the whole feasible flight-path-angle
     bracket that get_min/get_max_flight_angle declare. If it goes negative the
     bracket and the function disagree about the valid domain, and
     get_flight_angle's bisection will drive its bounds into the bad region. */
  double r0 = EARTH_RADIUS_M + 200e3;
  double rf = EARTH_RADIUS_M;
  double phi = 85.4 * M_PI / 180.0; /* ~10,000 km of downrange */
  runparams rp;
  rp.grav_error = 0;
  grav grav_model = init_grav(&rp, 1);

  double gmin = get_min_flight_angle(r0, rf, phi);
  double gmax = get_max_flight_angle(r0, rf, phi);

  int negative_samples = 0;
  for (int i = 1; i < 200; i++) {
    double gamma = gmin + (gmax - gmin) * i / 200.0;
    double v = get_lambert_velocity(r0, rf, phi, gamma, &grav_model);
    double t = time_to_fly(r0, phi, gamma, v, &grav_model);
    if (t < 0.0) {
      negative_samples++;
    }
  }
  REQUIRE_EQ(negative_samples, 0);
}

TEST(thrust, time_to_fly_monotonic_across_bracket) {
  /* Flight time must increase monotonically with flight-path angle: a steeper
     lob takes longer. get_flight_angle's bracket update assumes exactly this,
     so any reversal breaks the search. */
  double r0 = EARTH_RADIUS_M + 200e3;
  double rf = EARTH_RADIUS_M;
  double phi = 85.4 * M_PI / 180.0;
  runparams rp;
  rp.grav_error = 0;
  grav grav_model = init_grav(&rp, 1);

  double gmin = get_min_flight_angle(r0, rf, phi);
  double gmax = get_max_flight_angle(r0, rf, phi);

  int reversals = 0;
  double prev = -1.0;
  for (int i = 1; i < 200; i++) {
    double gamma = gmin + (gmax - gmin) * i / 200.0;
    double v = get_lambert_velocity(r0, rf, phi, gamma, &grav_model);
    double t = time_to_fly(r0, phi, gamma, v, &grav_model);
    if (prev > 0.0 && t < prev) {
      reversals++;
    }
    prev = t;
  }
  REQUIRE_EQ(reversals, 0);
}

TEST(thrust, get_flight_angle_converges_for_long_flight_times) {
  /* A desired flight time inside the achievable range must produce a finite
     flight-path angle. Returning NAN here poisons the Lambert velocity vector,
     which then propagates into the integrator and aborts the run. */
  double r0 = EARTH_RADIUS_M + 200e3;
  double rf = EARTH_RADIUS_M;
  double phi = 85.4 * M_PI / 180.0;
  runparams rp;
  rp.grav_error = 0;
  grav grav_model = init_grav(&rp, 1);

  double t_des[] = {1500.0, 2500.0, 3500.0, 4500.0, 6000.0};
  for (int i = 0; i < 5; i++) {
    double gamma = get_flight_angle(r0, rf, phi, t_des[i], &grav_model);
    REQUIRE(!isnan(gamma));
  }
}

TEST(thrust,
     initial_alignment_error_tips_the_vehicle_by_exactly_the_parameter) {
  // At launch the vehicle stands vertically. The guidance system believes that
  // exactly; the vehicle is really tipped from it by initial_angle_error.
  runparams rp = {0};
  rp.theta_long = 0.9320122849463917;
  rp.theta_lat = 0.0;
  rp.initial_angle_error = 1e-3;

  cartvec launch_vertical = {1.0, 0.0, 0.0};

  for (int trial = 0; trial < 25; trial++) {
    state true_state = init_true_state(&rp);
    state est_state = init_est_state(&rp);

    // Angle between the true and estimated body frames.
    quaternion est_conj = {est_state.q_EB.w, -est_state.q_EB.x,
                           -est_state.q_EB.y, -est_state.q_EB.z};
    quaternion rel = qmultiply(true_state.q_EB, est_conj);
    double misalignment = 2.0 * acos(fmin(1.0, fabs(rel.w)));
    REQUIRE_LT(fabs(misalignment - rp.initial_angle_error), 1e-9);

    // A vertical thrust command, which is what the vehicle flies for the first
    // t_vert_boost seconds, is tipped by that same angle. This is the transform
    // get_thrust_acc performs.
    cartvec applied = body_to_eci(eci_to_body(launch_vertical, est_state.q_EB),
                                  true_state.q_EB);
    double deflection =
        acos(fmin(1.0, fmax(-1.0, dot(applied, launch_vertical))));
    REQUIRE_LT(fabs(deflection - rp.initial_angle_error), 1e-9);
    REQUIRE_LT(fabs(norm(applied) - 1.0), 1e-12);

    // The commanded thrust angles carry no perturbation of their own, so the
    // lean reaches the thrust only once, through the frames.
    REQUIRE_LT(fabs(true_state.theta_long - rp.theta_long), 1e-15);
    REQUIRE_LT(fabs(true_state.theta_lat - rp.theta_lat), 1e-15);
    REQUIRE_LT(fabs(true_state.theta_long - est_state.theta_long), 1e-15);
    REQUIRE_LT(fabs(true_state.theta_lat - est_state.theta_lat), 1e-15);
  }
}

TEST(thrust, zero_alignment_error_leaves_frames_identical) {
  runparams rp = {0};
  rp.theta_long = 0.9320122849463917;
  rp.theta_lat = 0.0;
  rp.initial_angle_error = 0.0;

  state true_state = init_true_state(&rp);
  state est_state = init_est_state(&rp);

  REQUIRE_LT(fabs(true_state.theta_long - rp.theta_long), 1e-12);
  REQUIRE_LT(fabs(true_state.theta_lat - rp.theta_lat), 1e-12);
  REQUIRE_LT(fabs(true_state.q_EB.w - est_state.q_EB.w), 1e-15);
  REQUIRE_LT(fabs(true_state.q_EB.x - est_state.q_EB.x), 1e-15);
  REQUIRE_LT(fabs(true_state.q_EB.y - est_state.q_EB.y), 1e-15);
  REQUIRE_LT(fabs(true_state.q_EB.z - est_state.q_EB.z), 1e-15);
}
