#ifndef ATMOSPHERE_H
#define ATMOSPHERE_H

#include <math.h>

#include "../math/linalg.h"
#include "../rng/rng.h"
#include "../utils/utils.h"
#include "state.h"

#include <stdio.h>
#include <stdlib.h>

// Number of altitude rows per EarthGRAM profile (0-99 km at 1 km spacing). The
// number of profiles is determined at runtime from the profile file.
#define ATM_PROFILE_LEN 100

// Define an atm_cond struct to store local atmospheric conditions
typedef struct atm_cond {
  double altitude;        // altitude in meters
  double density;         // density in kg/m^3
  double meridional_wind; // meridional wind in m/s
  double zonal_wind;      // zonal wind in m/s
  double vertical_wind;   // vertical wind in m/s

  // General atmospheric parameters
} atm_cond;

// Altitude bands of the perturbed exponential model (atm_model == 1). Band i
// spans [ATM_PERT_BAND_TOPS_M[i-1], ATM_PERT_BAND_TOPS_M[i]); the last band
// extends upward indefinitely.
#define ATM_PERT_BANDS 8
static const double ATM_PERT_BAND_TOPS_M[ATM_PERT_BANDS - 1] = {
    5000, 10000, 20000, 30000, 40000, 50000, 70000};

// Define an atm_model struct to store the atmospheric model
typedef struct atm_model {
  // Constants
  double scale_height;      // scale height in meters
  double sea_level_density; // sea level density in kg/m^3

  // Standard deviations per altitude band
  double std_densities[ATM_PERT_BANDS]; // fractional
  double std_zonal_winds[ATM_PERT_BANDS];
  double std_meridional_winds[ATM_PERT_BANDS];
  double std_vert_winds[ATM_PERT_BANDS];

  // Perturbations, drawn once per run per band
  double pert_densities[ATM_PERT_BANDS];
  double pert_zonal_winds[ATM_PERT_BANDS];
  double pert_meridional_winds[ATM_PERT_BANDS];
  double pert_vert_winds[ATM_PERT_BANDS];

} atm_model;

// Define an eg16_profile struct to store the atmospheric profile data
typedef struct eg16_profile {
  int profile_num;                              // profile number
  double alt_data[ATM_PROFILE_LEN];             // altitude data
  double density_data[ATM_PROFILE_LEN];         // density data
  double meridional_wind_data[ATM_PROFILE_LEN]; // meridional wind data
  double zonal_wind_data[ATM_PROFILE_LEN];      // zonal wind data
  double vertical_wind_data[ATM_PROFILE_LEN];   // vertical wind data

} eg16_profile;

// Heap-allocated table of all EarthGRAM profiles, ATM_PROFILE_LEN rows each
double (*atm_data)[6] = NULL;
int atm_profile_count = 0;
// The mean atmospheric profile data has 5 columns, not 6, because there is no
// profile number column
double mean_atm_data[ATM_PROFILE_LEN][5];
int atm_data_is_filled = 0;
int mean_atm_data_is_filled = 0;

/**
 * Initializes atmospheric profile data so the file is read only once.
 *
 * The number of profiles is inferred from the number of data rows, which must
 * be a multiple of `ATM_PROFILE_LEN`.
 *
 * @param atmprofilepath Path to the atmospheric profile file.
 */
void init_atm_data(char *atmprofilepath) {

  if (atm_data_is_filled == 1) {
    return;
  }

  // Open the atmospheric profile file
  FILE *fp = fopen(atmprofilepath, "r");
  if (fp == NULL) {
    printf("Error opening atmospheric profile file %s\n", atmprofilepath);
    exit(1);
  }

  // Skip the column-name header row if present.
  char header_line[1024];
  fgets(header_line, sizeof(header_line), fp);

  // Read rows until EOF, growing the table as needed. Each value is followed
  // by a comma.
  int capacity = ATM_PROFILE_LEN * 100;
  int num_rows = 0;
  atm_data = (double (*)[6])malloc(capacity * sizeof(*atm_data));
  double row[6];
  while (fscanf(fp, "%lf,%lf,%lf,%lf,%lf,%lf,", &row[0], &row[1], &row[2],
                &row[3], &row[4], &row[5]) == 6) {
    if (num_rows == capacity) {
      capacity *= 2;
      atm_data = (double (*)[6])realloc(atm_data, capacity * sizeof(*atm_data));
    }
    for (int j = 0; j < 6; j++) {
      atm_data[num_rows][j] = row[j];
    }
    num_rows++;
  }
  fclose(fp);

  if (num_rows == 0 || num_rows % ATM_PROFILE_LEN != 0) {
    printf("Error: %s has %d data rows, expected a nonzero multiple of %d\n",
           atmprofilepath, num_rows, ATM_PROFILE_LEN);
    exit(1);
  }
  atm_profile_count = num_rows / ATM_PROFILE_LEN;
  atm_data_is_filled = 1;
}

/**
 * Draws a uniformly random EarthGRAM profile index.
 *
 * @param atmprofilepath Path to the atmospheric profile file.
 * @return Profile index in `[0, atm_profile_count)`.
 */
int sample_atm_profile_num(char *atmprofilepath) {
  init_atm_data(atmprofilepath);
  int profilenum = (int)ran_flat(0, atm_profile_count);
  // ran_flat can return its upper bound exactly
  return profilenum < atm_profile_count ? profilenum : atm_profile_count - 1;
}

/**
 * Initializes mean atmospheric profile data so the file is read only once.
 *
 * @param atmprofilepath Path to the mean atmospheric profile file.
 */
void init_mean_atm_data(char *atmprofilepath) {

  if (mean_atm_data_is_filled == 1) {
    return;
  }

  // Open the atmospheric profile file
  FILE *fp = fopen(atmprofilepath, "r");
  if (fp == NULL) {
    printf("Error opening mean atmospheric profile file %s\n", atmprofilepath);
    return;
  }

  // Read the atmospheric profile data delimited by spaces.
  for (int i = 0; i < ATM_PROFILE_LEN; i++) {
    for (int j = 0; j < 5; j++) {
      fscanf(fp, "%lf", &mean_atm_data[i][j]);
    }
  }
  mean_atm_data_is_filled = 1;
  fclose(fp);
}

/**
 * Initializes the exponential atmosphere model and optional perturbations.
 *
 * @param run_params Pointer to simulation run parameters.
 * @return Initialized atmosphere model.
 */
atm_model init_exp_atm(runparams *run_params) {

  atm_model atm_model;

  // Define constants
  atm_model.scale_height = 8000;       // scale height in meters
  atm_model.sea_level_density = 1.225; // sea level density in kg/m^3

  // Band standard deviations: RMS over the EarthGRAM 2016 profiles
  // (atmprofiles.csv) within each band. Density is the fractional deviation
  // from the per-altitude mean; winds are RMS rather than std so the zero-mean
  // draws reproduce typical EarthGRAM wind speeds, including the mean wind.
  static const double std_densities[ATM_PERT_BANDS] = {
      0.0265, 0.0151, 0.0490, 0.0236, 0.0324, 0.0526, 0.0843, 0.1238};
  static const double std_zonal_winds[ATM_PERT_BANDS] = {
      8.00, 16.72, 21.92, 15.26, 28.17, 42.61, 48.18, 51.08};
  static const double std_meridional_winds[ATM_PERT_BANDS] = {
      4.93, 8.21, 8.92, 4.24, 7.02, 9.77, 13.93, 32.00};
  static const double std_vert_winds[ATM_PERT_BANDS] = {3.24, 1.31, 0.60, 0.46,
                                                        0.63, 0.90, 1.52, 3.96};

  // Perturbations are zero for the non-perturbed branch (atm_model == 0)
  double pert_scale = 0.0;
  if (run_params->atm_model != 0) {
    // Optional multiplier on all standard deviations
    pert_scale =
        run_params->atm_pert_scale > 0 ? run_params->atm_pert_scale : 1.0;
  }

  for (int i = 0; i < ATM_PERT_BANDS; i++) {
    atm_model.std_densities[i] = pert_scale * std_densities[i];
    atm_model.std_zonal_winds[i] = pert_scale * std_zonal_winds[i];
    atm_model.std_meridional_winds[i] = pert_scale * std_meridional_winds[i];
    atm_model.std_vert_winds[i] = pert_scale * std_vert_winds[i];

    atm_model.pert_densities[i] = 0;
    atm_model.pert_zonal_winds[i] = 0;
    atm_model.pert_meridional_winds[i] = 0;
    atm_model.pert_vert_winds[i] = 0;
    if (run_params->atm_model != 0) {
      // Generate perturbations, which are then used by the get_atm_cond
      // function to generate the true conditions
      atm_model.pert_densities[i] =
          atm_model.std_densities[i] * ran_gaussian(1);
      atm_model.pert_zonal_winds[i] =
          atm_model.std_zonal_winds[i] * ran_gaussian(1);
      atm_model.pert_meridional_winds[i] =
          atm_model.std_meridional_winds[i] * ran_gaussian(1);
      atm_model.pert_vert_winds[i] =
          atm_model.std_vert_winds[i] * ran_gaussian(1);
    }
  }

  return atm_model;
}

/**
 * Calculates atmospheric conditions at a given altitude using an exponential
 * model.
 *
 * @param altitude Altitude in meters.
 * @param atm_model Pointer to atmospheric model.
 * @return Local atmospheric conditions.
 */
atm_cond get_exp_atm_cond(double altitude, atm_model *atm_model) {

  atm_cond atm_conditions;
  if (altitude < 0) {
    altitude = 0;
  }
  atm_conditions.altitude = altitude;
  atm_conditions.density =
      atm_model->sea_level_density * exp(-altitude / atm_model->scale_height);
  atm_conditions.meridional_wind = 0;
  atm_conditions.zonal_wind = 0;
  atm_conditions.vertical_wind = 0;

  return atm_conditions;
}

/**
 * Calculates atmospheric conditions using the perturbed exponential model.
 *
 * @param altitude Altitude in meters.
 * @param atm_model Pointer to atmospheric model.
 * @return Local atmospheric conditions.
 */
atm_cond get_pert_atm_cond(double altitude, atm_model *atm_model) {

  atm_cond atm_conditions;
  if (altitude < 0) {
    altitude = 0;
  }
  atm_conditions.altitude = altitude;

  // Find the altitude band
  int band = 0;
  while (band < ATM_PERT_BANDS - 1 && altitude >= ATM_PERT_BAND_TOPS_M[band]) {
    band++;
  }

  atm_conditions.density =
      atm_model->sea_level_density * exp(-altitude / atm_model->scale_height);
  atm_conditions.density *= 1 + atm_model->pert_densities[band];

  atm_conditions.meridional_wind = atm_model->pert_meridional_winds[band];
  atm_conditions.zonal_wind = atm_model->pert_zonal_winds[band];
  atm_conditions.vertical_wind = atm_model->pert_vert_winds[band];

  return atm_conditions;
}

/**
 * Calculates atmospheric conditions at a given altitude using an EarthGRAM
 * 2016 profile.
 *
 * @param altitude Altitude in meters.
 * @param atm_profile Pointer to EarthGRAM 2016 profile.
 * @return Local atmospheric conditions.
 */
atm_cond get_eg_atm_cond(double altitude, eg16_profile *atm_profile) {

  atm_cond atm_conditions;
  if (altitude < 0) {
    altitude = 0;
  }

  // atm_cond.altitude is in meters for every model. The EarthGRAM tables are
  // tabulated in kilometers, so convert only for the lookup.
  atm_conditions.altitude = altitude;
  double altitude_km = altitude / 1000;

  if (altitude_km > 99) {
    atm_conditions.density = 0;
    atm_conditions.meridional_wind = 0;
    atm_conditions.zonal_wind = 0;
    atm_conditions.vertical_wind = 0;
    return atm_conditions;
  }
  int num_heights = ATM_PROFILE_LEN;
  // Use linear interpolation to get the atmospheric conditions
  atm_conditions.density = linterp(altitude_km, atm_profile->alt_data,
                                   atm_profile->density_data, num_heights);
  atm_conditions.meridional_wind =
      linterp(altitude_km, atm_profile->alt_data,
              atm_profile->meridional_wind_data, num_heights);
  atm_conditions.zonal_wind =
      linterp(altitude_km, atm_profile->alt_data, atm_profile->zonal_wind_data,
              num_heights);
  atm_conditions.vertical_wind =
      linterp(altitude_km, atm_profile->alt_data,
              atm_profile->vertical_wind_data, num_heights);

  return atm_conditions;
}

/**
 * Calculates atmospheric conditions at a given altitude for the configured
 * atmosphere model.
 *
 * @param altitude Altitude in meters.
 * @param exp_atm_model Pointer to exponential atmosphere model.
 * @param run_params Pointer to run parameters.
 * @param atm_profile Pointer to EarthGRAM profile.
 * @return Local atmospheric conditions.
 */
atm_cond get_atm_cond(double altitude, atm_model *exp_atm_model,
                      runparams *run_params, eg16_profile *atm_profile) {

  atm_cond atm_conditions;

  if (run_params->atm_model == 0) {
    // Exponential model without perturbations
    atm_conditions = get_exp_atm_cond(altitude, exp_atm_model);
  } else if (run_params->atm_model == 1) {
    // Exponential model with Gaussian wind perturbations
    atm_conditions = get_pert_atm_cond(altitude, exp_atm_model);
  } else if (run_params->atm_model >= 2) {
    // EarthGRAM model (random or average)
    atm_conditions = get_eg_atm_cond(altitude, atm_profile);
  }

  return atm_conditions;
}

/**
 * Parses atmospheric profile data and selects the requested profile.
 *
 * @param atmprofilepath Path to atmospheric profile data file.
 * @param profilenum Profile index to use; `-1` selects the mean profile.
 * @return Atmospheric profile data.
 */
eg16_profile parse_atm(char *atmprofilepath, int profilenum) {
  // Initialize the atmospheric profile struct
  eg16_profile atm_profile;
  atm_profile.profile_num = profilenum;

  if (profilenum < 0) {
    init_mean_atm_data(atmprofilepath);
    // Update the atmospheric profile struct using mean data
    for (int i = 0; i < ATM_PROFILE_LEN; i++) {
      atm_profile.alt_data[i] = mean_atm_data[i][0];
      atm_profile.density_data[i] = mean_atm_data[i][1];
      atm_profile.meridional_wind_data[i] = mean_atm_data[i][2];
      atm_profile.zonal_wind_data[i] = mean_atm_data[i][3];
      atm_profile.vertical_wind_data[i] = mean_atm_data[i][4];
    }
  } else {
    init_atm_data(atmprofilepath);
    if (profilenum >= atm_profile_count) {
      printf("Error: atmospheric profile %d requested, but only %d exist\n",
             profilenum, atm_profile_count);
      exit(1);
    }
    // Update the atmospheric profile struct by iterating over only the
    // requested profile
    for (int i = 0; i < ATM_PROFILE_LEN; i++) {
      atm_profile.alt_data[i] = atm_data[ATM_PROFILE_LEN * profilenum + i][1];
      atm_profile.density_data[i] =
          atm_data[ATM_PROFILE_LEN * profilenum + i][2];
      atm_profile.zonal_wind_data[i] =
          atm_data[ATM_PROFILE_LEN * profilenum + i][3];
      atm_profile.meridional_wind_data[i] =
          atm_data[ATM_PROFILE_LEN * profilenum + i][4];
      atm_profile.vertical_wind_data[i] =
          atm_data[ATM_PROFILE_LEN * profilenum + i][5];
    }
  }

  return atm_profile;
}

/**
 * Gets wind at the current location in standard Cartesian coordinates.
 *
 * @param state Pointer to current vehicle state.
 * @param atm_cond Pointer to local atmospheric conditions.
 * @return Wind vector in Cartesian coordinates.
 */
cartvec get_cart_wind(state *state, atm_cond *atm_cond) {
  double cart_wind[3];
  double spher_wind[3] = {atm_cond->vertical_wind, atm_cond->zonal_wind,
                          atm_cond->meridional_wind};
  double spher_coords[3];
  double cart_coords[3] = {state->position.x, state->position.y,
                           state->position.z};
  cartcoords_to_sphercoords(cart_coords, spher_coords);

  sphervec_to_cartvec(spher_wind, cart_wind, spher_coords);

  cartvec cartvec_wind;
  cartvec_wind.x = cart_wind[0];
  cartvec_wind.y = cart_wind[1];
  cartvec_wind.z = cart_wind[2];

  return cartvec_wind;
}

/**
 * Gets relative wind in ECI coordinates.
 *
 * The relative wind is defined as wind minus vehicle velocity:
 * $$\mathbf V_{\mathrm{rel},E} = \mathbf V_{\mathrm{wind},E} - \mathbf v_E$$
 *
 * @param current_state Pointer to current vehicle state.
 * @param atm_cond Pointer to local atmospheric conditions.
 * @return Relative wind vector in ECI coordinates.
 */
static inline cartvec get_relative_wind_eci(state *current_state,
                                            atm_cond *atm_cond) {
  cartvec wind_vec = get_cart_wind(current_state, atm_cond);
  return subtract(wind_vec, current_state->velocity);
}

#endif