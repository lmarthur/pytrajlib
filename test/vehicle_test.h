#include "../src/include/vehicle.h"
#include <tau/tau.h>

TEST(vehicle, init_ballistic_rv) {
  rv rv = init_ballistic_rv();

  REQUIRE_EQ(rv.maneuverability_flag, 0);
  REQUIRE_LT(rv.c_m_alpha, 0);
  REQUIRE_LT(rv.c_m_q, 0);
  REQUIRE_EQ(rv.flap_area, 0);
  REQUIRE_GT(rv.rv_area, 0);
}

TEST(vehicle, init_swerve_rv) {
  rv rv = init_swerve_rv();

  REQUIRE_EQ(rv.maneuverability_flag, 1);
  REQUIRE_LT(rv.c_m_alpha, 0);
  REQUIRE_LT(rv.c_m_q, 0);
  REQUIRE_GT(rv.flap_area, 0);
  REQUIRE_GT(rv.rv_area, 0);
}

TEST(vehicle, init_mmiii_booster) {
  booster booster = init_mmiii_booster();

  REQUIRE_EQ(booster.num_stages, 3);
  REQUIRE_GT(booster.maxdiam, 0);
  REQUIRE_GT(booster.area, 0);
  REQUIRE_EQ(booster.total_burn_time, 188);
}

TEST(vehicle, init_mmiii_ballistic) {
  vehicle vehicle = init_mmiii_ballistic();

  REQUIRE_EQ(vehicle.booster.num_stages, 3);
  REQUIRE_GT(vehicle.booster.maxdiam, 0);
  REQUIRE_GT(vehicle.booster.area, 0);
  REQUIRE_EQ(vehicle.booster.total_burn_time, 188);
  REQUIRE_EQ(vehicle.rv.maneuverability_flag, 0);
  REQUIRE_GT(vehicle.rv.rv_area, 0);
  REQUIRE_LT(vehicle.rv.c_m_alpha, 0);
  REQUIRE_LT(vehicle.rv.c_m_q, 0);
  REQUIRE_EQ(vehicle.rv.flap_area, 0);
}

TEST(vehicle, init_mmiii_swerve) {
  vehicle vehicle = init_mmiii_swerve();

  REQUIRE_EQ(vehicle.booster.num_stages, 3);
  REQUIRE_GT(vehicle.booster.maxdiam, 0);
  REQUIRE_GT(vehicle.booster.area, 0);
  REQUIRE_EQ(vehicle.booster.total_burn_time, 188);
  REQUIRE_EQ(vehicle.rv.maneuverability_flag, 1);
  REQUIRE_LT(vehicle.rv.c_m_alpha, 0);
  REQUIRE_LT(vehicle.rv.c_m_q, 0);
  REQUIRE_GT(vehicle.rv.flap_area, 0);
}

TEST(vehicle, init_mock_vehicle) {
  vehicle vehicle = init_mock_vehicle();

  REQUIRE_EQ(vehicle.booster.num_stages, 3);
  REQUIRE_EQ(vehicle.booster.total_burn_time, 0);
  REQUIRE_NE(vehicle.total_mass, 0);
  REQUIRE_EQ(vehicle.total_mass, vehicle.current_mass);
}