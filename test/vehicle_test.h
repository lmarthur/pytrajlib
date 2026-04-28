#include "../src/include/models/vehicle.h"
#include "../src/include/utils/utils.h"
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
  vehicle vehicle = init_mmiii_ballistic(NULL);

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
  vehicle vehicle = init_mmiii_swerve(NULL);

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
  REQUIRE_EQ(vehicle.total_mass, get_vehicle_mass(&vehicle, 0));
}

TEST(vehicle, swerve_table_linterp_exact_knot) {
  double alpha_deg = 4.0;
  double cd = linterp(alpha_deg, (double *)SWERVE_ALPHA_DEG_TABLE,
                      (double *)SWERVE_CD_TABLE, SWERVE_AERO_TABLE_SIZE);

  REQUIRE_LT(fabs(cd - 0.0309), 1e-12);
}

TEST(vehicle, swerve_table_linterp_midpoint_cd) {
  double alpha_deg = 0.1;
  double cd = linterp(alpha_deg, (double *)SWERVE_ALPHA_DEG_TABLE,
                      (double *)SWERVE_CD_TABLE, SWERVE_AERO_TABLE_SIZE);

  /* Midpoint between alpha=0.0 (0.0180) and alpha=0.2 (0.0181). */
  REQUIRE_LT(fabs(cd - 0.01805), 1e-12);
}

TEST(vehicle, swerve_table_linterp_midpoint_cn_and_cmq) {
  double alpha_deg_cn = 9.9;
  double cn = linterp(alpha_deg_cn, (double *)SWERVE_ALPHA_DEG_TABLE,
                      (double *)SWERVE_CN_TABLE, SWERVE_AERO_TABLE_SIZE);

  double alpha_deg_cmq = 6.3;
  double cmq = linterp(alpha_deg_cmq, (double *)SWERVE_ALPHA_DEG_TABLE,
                       (double *)SWERVE_CMQ_TABLE, SWERVE_AERO_TABLE_SIZE);

  /* 9.9 lies halfway between 9.8 (0.350) and 10.0 (0.360). */
  REQUIRE_LT(fabs(cn - 0.355), 1e-12);
  /* 6.3 lies halfway between 6.2 (-0.147) and 6.4 (-0.149). */
  REQUIRE_LT(fabs(cmq + 0.148), 1e-12);
}