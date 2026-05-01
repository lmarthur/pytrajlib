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
  grav grav_model = init_grav(&rp);

  cartvec v_lambert =
      get_lambert_velocity_vector(position, aimpoint, tf_des, &grav_model);
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
  grav grav_model = init_grav(&rp);

  cartvec v_lambert =
      get_lambert_velocity_vector(position, aimpoint, tf_des, &grav_model);
  double expected_vx = 6993.352308 * 0.3048;
  double expected_vy = 18322.670437 * 0.3048;
  printf("Lambert velocity at altitude x %f y %f z %f\n", v_lambert.x,
         v_lambert.y, v_lambert.z);

  REQUIRE_LT(fabs(v_lambert.x - expected_vx), 1.0);
  REQUIRE_LT(fabs(v_lambert.y - expected_vy), 1.0);
  REQUIRE_LT(fabs(v_lambert.z), 1e-9);
}
