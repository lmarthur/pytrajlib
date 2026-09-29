#include "../src/include/models/atmosphere.h"
#include <tau/tau.h>

TEST(atmosphere, init_exp_atm) {
  // Initialize the run parameters
  runparams run_params = {0};
  run_params.atm_model = 0;

  // Initialize the atmospheric model
  atm_model atm_model = init_exp_atm(&run_params);

  // Check the sea level density
  REQUIRE_EQ(atm_model.sea_level_density, 1.225);

  // Check the scale height
  REQUIRE_EQ(atm_model.scale_height, 8000);

  // Check the standard deviations and perturbations
  for (int i = 0; i < ATM_PERT_BANDS; i++) {
    REQUIRE_EQ(atm_model.std_densities[i], 0);
    REQUIRE_EQ(atm_model.std_zonal_winds[i], 0);
    REQUIRE_EQ(atm_model.std_meridional_winds[i], 0);
    REQUIRE_EQ(atm_model.std_vert_winds[i], 0);
    REQUIRE_EQ(atm_model.pert_densities[i], 0);
    REQUIRE_EQ(atm_model.pert_zonal_winds[i], 0);
    REQUIRE_EQ(atm_model.pert_meridional_winds[i], 0);
    REQUIRE_EQ(atm_model.pert_vert_winds[i], 0);
  }

  run_params.atm_model = 1;
  atm_model = init_exp_atm(&run_params);

  // Check the perturbations
  for (int i = 0; i < ATM_PERT_BANDS; i++) {
    REQUIRE_GT(atm_model.std_densities[i], 0);
    REQUIRE_GT(atm_model.std_zonal_winds[i], 0);
    REQUIRE_GT(atm_model.std_meridional_winds[i], 0);
    REQUIRE_GT(atm_model.std_vert_winds[i], 0);
    REQUIRE_NE(atm_model.pert_densities[i], 0);
    REQUIRE_NE(atm_model.pert_densities[i], atm_model.std_densities[i]);
    REQUIRE_NE(atm_model.pert_zonal_winds[i], 0);
    REQUIRE_NE(atm_model.pert_zonal_winds[i], atm_model.std_zonal_winds[i]);
    REQUIRE_NE(atm_model.pert_meridional_winds[i], 0);
    REQUIRE_NE(atm_model.pert_meridional_winds[i],
               atm_model.std_meridional_winds[i]);
    REQUIRE_NE(atm_model.pert_vert_winds[i], 0);
    REQUIRE_NE(atm_model.pert_vert_winds[i], atm_model.std_vert_winds[i]);
  }
}

TEST(atmosphere, get_exp_atm_cond) {
  // Initialize the run parameters
  runparams run_params = {0};
  run_params.atm_model = 0;

  // Initialize the atmospheric model
  atm_model atm_model = init_exp_atm(&run_params);

  // Get the atmospheric conditions at sea level
  atm_cond atm_conditions = get_exp_atm_cond(0, &atm_model);

  // Check the altitude
  REQUIRE_EQ(atm_conditions.altitude, 0);

  // Check the density
  REQUIRE_EQ(atm_conditions.density, atm_model.sea_level_density);

  // Check the wind components
  REQUIRE_EQ(atm_conditions.meridional_wind, 0);
  REQUIRE_EQ(atm_conditions.zonal_wind, 0);
  REQUIRE_EQ(atm_conditions.vertical_wind, 0);

  // Get the atmospheric conditions at 10 km
  atm_conditions = get_exp_atm_cond(10000, &atm_model);

  // Check the altitude
  REQUIRE_EQ(atm_conditions.altitude, 10000);

  // Check the density
  REQUIRE_LT(atm_conditions.density, atm_model.sea_level_density);

  // Check the wind components
  REQUIRE_EQ(atm_conditions.meridional_wind, 0);
  REQUIRE_EQ(atm_conditions.zonal_wind, 0);
  REQUIRE_EQ(atm_conditions.vertical_wind, 0);

  // Get the atmospheric conditions at 100 km
  atm_conditions = get_exp_atm_cond(100000, &atm_model);

  // Check the altitude
  REQUIRE_EQ(atm_conditions.altitude, 100000);

  // Check the density
  REQUIRE_LT(atm_conditions.density, atm_model.sea_level_density);

  // Check the wind components
  REQUIRE_EQ(atm_conditions.meridional_wind, 0);
  REQUIRE_EQ(atm_conditions.zonal_wind, 0);
  REQUIRE_EQ(atm_conditions.vertical_wind, 0);

  // Get the atmospheric conditions at 1000 km
  atm_conditions = get_exp_atm_cond(1000000, &atm_model);

  // Check the altitude
  REQUIRE_EQ(atm_conditions.altitude, 1000000);

  // Check the density
  REQUIRE_LT(atm_conditions.density, atm_model.sea_level_density);

  // Repeat the test with perturbation flag enabled
  run_params.atm_model = 1;

  // Initialize the atmospheric model
  atm_model = init_exp_atm(&run_params);

  // Get the atmospheric conditions at sea level
  atm_conditions = get_exp_atm_cond(0, &atm_model);

  // Check the altitude
  REQUIRE_EQ(atm_conditions.altitude, 0);

  // Check the density
  REQUIRE_EQ(atm_conditions.density, atm_model.sea_level_density);

  // Check the wind components
  REQUIRE_EQ(atm_conditions.meridional_wind, 0);
  REQUIRE_EQ(atm_conditions.zonal_wind, 0);
  REQUIRE_EQ(atm_conditions.vertical_wind, 0);

  // Get the atmospheric conditions at 10 km
  atm_conditions = get_exp_atm_cond(10000, &atm_model);

  // Check the altitude
  REQUIRE_EQ(atm_conditions.altitude, 10000);

  // Check the density
  REQUIRE_LT(atm_conditions.density, atm_model.sea_level_density);

  // Check the wind components
  REQUIRE_EQ(atm_conditions.meridional_wind, 0);
  REQUIRE_EQ(atm_conditions.zonal_wind, 0);
  REQUIRE_EQ(atm_conditions.vertical_wind, 0);

  // Get the atmospheric conditions at 100 km
  atm_conditions = get_exp_atm_cond(100000, &atm_model);

  // Check the altitude
  REQUIRE_EQ(atm_conditions.altitude, 100000);

  // Check the density
  REQUIRE_LT(atm_conditions.density, atm_model.sea_level_density);

  // Check the wind components
  REQUIRE_EQ(atm_conditions.meridional_wind, 0);
  REQUIRE_EQ(atm_conditions.zonal_wind, 0);
  REQUIRE_EQ(atm_conditions.vertical_wind, 0);

  // Get the atmospheric conditions at 1000 km
  atm_conditions = get_exp_atm_cond(1000000, &atm_model);

  // Check the altitude
  REQUIRE_EQ(atm_conditions.altitude, 1000000);

  // Check the density
  REQUIRE_LT(atm_conditions.density, atm_model.sea_level_density);

  // Check the atmospheric conditions at negative altitude
  atm_conditions = get_exp_atm_cond(-1000, &atm_model);

  // Check the altitude
  REQUIRE_EQ(atm_conditions.altitude, 0);

  // Check the density
  REQUIRE_EQ(atm_conditions.density, atm_model.sea_level_density);

  // Check the wind components
  REQUIRE_EQ(atm_conditions.meridional_wind, 0);
  REQUIRE_EQ(atm_conditions.zonal_wind, 0);
  REQUIRE_EQ(atm_conditions.vertical_wind, 0);
}

TEST(atmosphere, get_pert_atm_cond) {
  // Initialize the run parameters
  runparams run_params = {0};
  run_params.atm_model = 0;

  // Initialize the atmospheric model
  atm_model atm_model = init_exp_atm(&run_params);

  // Get the atmospheric conditions at sea level
  atm_cond atm_conditions = get_pert_atm_cond(0, &atm_model);

  // Check the altitude
  REQUIRE_EQ(atm_conditions.altitude, 0);

  // Check the density
  REQUIRE_EQ(atm_conditions.density, atm_model.sea_level_density);

  // Check the wind components
  REQUIRE_EQ(atm_conditions.meridional_wind, 0);
  REQUIRE_EQ(atm_conditions.zonal_wind, 0);
  REQUIRE_EQ(atm_conditions.vertical_wind, 0);

  // Get the atmospheric conditions at 10 km
  atm_conditions = get_pert_atm_cond(10000, &atm_model);

  // Check the altitude
  REQUIRE_EQ(atm_conditions.altitude, 10000);

  // Check the density
  REQUIRE_LT(atm_conditions.density, atm_model.sea_level_density);

  // Check the wind components
  REQUIRE_EQ(atm_conditions.meridional_wind, 0);
  REQUIRE_EQ(atm_conditions.zonal_wind, 0);
  REQUIRE_EQ(atm_conditions.vertical_wind, 0);

  // Get the atmospheric conditions at 100 km
  atm_conditions = get_pert_atm_cond(100000, &atm_model);

  // Check the altitude
  REQUIRE_EQ(atm_conditions.altitude, 100000);

  // Check the density
  REQUIRE_LT(atm_conditions.density, atm_model.sea_level_density);

  // Check the wind components
  REQUIRE_EQ(atm_conditions.meridional_wind, 0);
  REQUIRE_EQ(atm_conditions.zonal_wind, 0);
  REQUIRE_EQ(atm_conditions.vertical_wind, 0);

  // Get the atmospheric conditions at 1000 km
  atm_conditions = get_pert_atm_cond(1000000, &atm_model);

  // Check the altitude
  REQUIRE_EQ(atm_conditions.altitude, 1000000);

  // Check the density
  REQUIRE_LT(atm_conditions.density, atm_model.sea_level_density);

  // Repeat the test with perturbation flag enabled
  run_params.atm_model = 1;

  // Initialize the atmospheric model
  atm_model = init_exp_atm(&run_params);

  // Get the atmospheric conditions at sea level
  atm_conditions = get_pert_atm_cond(0, &atm_model);

  // Check the altitude
  REQUIRE_EQ(atm_conditions.altitude, 0);

  // Check the density
  REQUIRE_NE(atm_conditions.density, atm_model.sea_level_density);

  // Check the wind components
  REQUIRE_NE(atm_conditions.meridional_wind, 0);
  REQUIRE_NE(atm_conditions.zonal_wind, 0);
  REQUIRE_NE(atm_conditions.vertical_wind, 0);
  REQUIRE_NE(atm_conditions.meridional_wind, atm_conditions.zonal_wind);
  REQUIRE_NE(atm_conditions.meridional_wind, atm_conditions.vertical_wind);
  REQUIRE_NE(atm_model.std_meridional_winds[0], atm_conditions.meridional_wind);
  REQUIRE_NE(atm_model.std_zonal_winds[0], atm_conditions.zonal_wind);
  REQUIRE_NE(atm_model.std_vert_winds[0], atm_conditions.vertical_wind);

  // Get the atmospheric conditions at 10 km
  atm_conditions = get_pert_atm_cond(10000, &atm_model);

  // Check the altitude
  REQUIRE_EQ(atm_conditions.altitude, 10000);

  // Check the density
  REQUIRE_NE(atm_conditions.density, atm_model.sea_level_density);

  // Check the wind components
  REQUIRE_NE(atm_conditions.meridional_wind, 0);
  REQUIRE_NE(atm_conditions.zonal_wind, 0);
  REQUIRE_NE(atm_conditions.vertical_wind, 0);
  REQUIRE_NE(atm_conditions.meridional_wind, atm_conditions.zonal_wind);
  REQUIRE_NE(atm_conditions.meridional_wind, atm_conditions.vertical_wind);
  REQUIRE_NE(atm_model.std_meridional_winds[2], atm_conditions.meridional_wind);
  REQUIRE_NE(atm_model.std_zonal_winds[2], atm_conditions.zonal_wind);
  REQUIRE_NE(atm_model.std_vert_winds[2], atm_conditions.vertical_wind);

  // Get the atmospheric conditions at 100 km
  atm_conditions = get_pert_atm_cond(100000, &atm_model);

  // Check the altitude
  REQUIRE_EQ(atm_conditions.altitude, 100000);

  // Check the density
  REQUIRE_LT(atm_conditions.density, atm_model.sea_level_density);

  // Check the wind components
  REQUIRE_NE(atm_conditions.meridional_wind, 0);
  REQUIRE_NE(atm_conditions.zonal_wind, 0);
  REQUIRE_NE(atm_conditions.vertical_wind, 0);
  REQUIRE_NE(atm_conditions.meridional_wind, atm_conditions.zonal_wind);
  REQUIRE_NE(atm_conditions.meridional_wind, atm_conditions.vertical_wind);
  REQUIRE_NE(atm_model.std_meridional_winds[7], atm_conditions.meridional_wind);
  REQUIRE_NE(atm_model.std_zonal_winds[7], atm_conditions.zonal_wind);
  REQUIRE_NE(atm_model.std_vert_winds[7], atm_conditions.vertical_wind);
}

TEST(atmosphere, pert_atm_bands) {
  runparams run_params = {0};
  run_params.atm_model = 1;
  atm_model atm_model = init_exp_atm(&run_params);

  // Each altitude reads the perturbations of its own band, with band edges
  // belonging to the band above
  double altitudes[] = {0,     4999,  5000,  9999,  10000, 19999,
                        20000, 45000, 50000, 69999, 70000, 1000000};
  int bands[] = {0, 0, 1, 1, 2, 2, 3, 5, 6, 6, 7, 7};
  for (int i = 0; i < 12; i++) {
    atm_cond atm_conditions = get_pert_atm_cond(altitudes[i], &atm_model);
    int band = bands[i];
    double exp_density = atm_model.sea_level_density *
                         exp(-altitudes[i] / atm_model.scale_height);
    REQUIRE_LT(fabs(atm_conditions.density -
                    exp_density * (1 + atm_model.pert_densities[band])),
               1e-12);
    REQUIRE_EQ(atm_conditions.zonal_wind, atm_model.pert_zonal_winds[band]);
    REQUIRE_EQ(atm_conditions.meridional_wind,
               atm_model.pert_meridional_winds[band]);
    REQUIRE_EQ(atm_conditions.vertical_wind, atm_model.pert_vert_winds[band]);
  }
}

TEST(atmosphere, atm_pert_scale) {
  runparams run_params = {0};
  run_params.atm_model = 1;

  // Unset (0) and 1 give the same standard deviations
  atm_model unscaled = init_exp_atm(&run_params);
  run_params.atm_pert_scale = 1;
  atm_model scale_one = init_exp_atm(&run_params);
  run_params.atm_pert_scale = 3;
  atm_model scaled = init_exp_atm(&run_params);

  for (int i = 0; i < ATM_PERT_BANDS; i++) {
    REQUIRE_EQ(unscaled.std_densities[i], scale_one.std_densities[i]);
    REQUIRE_EQ(unscaled.std_zonal_winds[i], scale_one.std_zonal_winds[i]);
    REQUIRE_LT(fabs(scaled.std_densities[i] - 3 * unscaled.std_densities[i]),
               1e-12);
    REQUIRE_LT(
        fabs(scaled.std_zonal_winds[i] - 3 * unscaled.std_zonal_winds[i]),
        1e-12);
    REQUIRE_LT(fabs(scaled.std_meridional_winds[i] -
                    3 * unscaled.std_meridional_winds[i]),
               1e-12);
    REQUIRE_LT(fabs(scaled.std_vert_winds[i] - 3 * unscaled.std_vert_winds[i]),
               1e-12);
  }

  // The scale has no effect on the unperturbed model
  run_params.atm_model = 0;
  atm_model unperturbed = init_exp_atm(&run_params);
  for (int i = 0; i < ATM_PERT_BANDS; i++) {
    REQUIRE_EQ(unperturbed.std_densities[i], 0);
    REQUIRE_EQ(unperturbed.pert_zonal_winds[i], 0);
  }
}

TEST(atmosphere, get_atm_cond) {
  runparams run_params = {0};
  run_params.atm_model = 0;

  // Initialize the atmospheric model
  atm_model atm_model = init_exp_atm(&run_params);

  eg16_profile atm_profile;

  // Get the atmospheric conditions at sea level
  atm_cond atm_conditions =
      get_atm_cond(0, &atm_model, &run_params, &atm_profile);

  // Check that the wind components are zero
  REQUIRE_EQ(atm_conditions.meridional_wind, 0);
  REQUIRE_EQ(atm_conditions.zonal_wind, 0);
  REQUIRE_EQ(atm_conditions.vertical_wind, 0);

  // Repeat the test with perturbation flag enabled
  run_params.atm_model = 1;

  // Initialize the atmospheric model
  atm_model = init_exp_atm(&run_params);

  // Get the atmospheric conditions at sea level

  atm_conditions = get_atm_cond(0, &atm_model, &run_params, &atm_profile);

  // Check that the wind components are not zero
  REQUIRE_NE(atm_conditions.meridional_wind, 0);
  REQUIRE_NE(atm_conditions.zonal_wind, 0);
  REQUIRE_NE(atm_conditions.vertical_wind, 0);
}

TEST(atmosphere, parse_atm) {
  int profilenum = 0;
  char *atmprofile = "src/pytrajlib/config/atmprofiles.csv";

  eg16_profile atm_data = parse_atm(atmprofile, profilenum);
  REQUIRE_EQ(atm_data.profile_num, 0);
  REQUIRE_EQ(atm_data.alt_data[0], 0.0);
  REQUIRE_EQ(atm_data.alt_data[1], 1.0);
  REQUIRE_GT(atm_data.density_data[0], 1);
  REQUIRE_LT(atm_data.density_data[0], 1.35);

  double density_0 = atm_data.density_data[0];
  double density_1 = atm_data.density_data[1];
  REQUIRE_GT(density_0, density_1);

  profilenum = 1;

  atm_data = parse_atm(atmprofile, profilenum);
  density_1 = atm_data.density_data[0];
  REQUIRE_EQ(atm_data.profile_num, 1);
  REQUIRE_EQ(atm_data.alt_data[0], 0.0);
  REQUIRE_EQ(atm_data.alt_data[1], 1.0);
  REQUIRE_GT(atm_data.density_data[0], 1);
  REQUIRE_LT(atm_data.density_data[0], 1.35);

  REQUIRE_NE(density_0, density_1);

  // The last profile in the file is readable, and sampled indices stay in range
  REQUIRE_GT(atm_profile_count, 1);
  atm_data = parse_atm(atmprofile, atm_profile_count - 1);
  REQUIRE_EQ(atm_data.profile_num, atm_profile_count - 1);
  REQUIRE_EQ(atm_data.alt_data[ATM_PROFILE_LEN - 1], ATM_PROFILE_LEN - 1.0);
  for (int i = 0; i < 1000; i++) {
    int sampled = sample_atm_profile_num(atmprofile);
    REQUIRE_GE(sampled, 0);
    REQUIRE_LT(sampled, atm_profile_count);
  }
}

TEST(atmosphere, get_eg_atm_cond) {
  // Test the get_eg_atm_cond function. atm_cond.altitude is reported in meters
  // for every atmosphere model; only the EarthGRAM table lookup is in
  // kilometers, so alt_data is compared against altitude / 1000.

  int profilenum = 0;
  char *atmprofile = "src/pytrajlib/config/atmprofiles.csv";

  eg16_profile atm_data = parse_atm(atmprofile, profilenum);

  // Test for altitude = -1 m
  double altitude = -1;
  atm_cond atm_conditions = get_eg_atm_cond(altitude, &atm_data);

  REQUIRE_EQ(atm_conditions.altitude, 0);
  REQUIRE_LT(fabs(atm_conditions.density - atm_data.density_data[0]), 1e-6);

  // Test for altitude = 0 m
  altitude = 0;
  atm_conditions = get_eg_atm_cond(altitude, &atm_data);

  REQUIRE_EQ(atm_conditions.altitude, 0);
  REQUIRE_LT(fabs(atm_conditions.density - atm_data.density_data[0]), 1e-6);

  // Test for altitude = 500 m
  altitude = 500;
  atm_conditions = get_eg_atm_cond(altitude, &atm_data);

  REQUIRE_EQ(atm_conditions.altitude, 500);
  REQUIRE_LT(atm_data.alt_data[0], atm_conditions.altitude / 1000);
  REQUIRE_GT(atm_data.alt_data[1], atm_conditions.altitude / 1000);
  REQUIRE_LT(atm_data.density_data[1], atm_conditions.density);
  REQUIRE_GT(atm_data.density_data[0], atm_conditions.density);

  // Test for altitude = 1000 m
  altitude = 1000;
  atm_conditions = get_eg_atm_cond(altitude, &atm_data);

  REQUIRE_EQ(atm_conditions.altitude, 1000);
  REQUIRE_LT(fabs(atm_conditions.density - atm_data.density_data[1]), 1e-6);

  // Test for altitude = 99000 m
  altitude = 99000;
  atm_conditions = get_eg_atm_cond(altitude, &atm_data);

  REQUIRE_EQ(atm_conditions.altitude, 99000);
  REQUIRE_LT(fabs(atm_conditions.density - atm_data.density_data[99]), 1e-6);

  // Test for altitude = 100000 m, above the top of the table
  altitude = 100000;
  atm_conditions = get_eg_atm_cond(altitude, &atm_data);

  REQUIRE_EQ(atm_conditions.altitude, 100000);
  REQUIRE_GT(atm_data.density_data[99], atm_conditions.density);
}