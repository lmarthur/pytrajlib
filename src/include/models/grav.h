#ifndef GRAV_H
#define GRAV_H

#include "../rng/rng.h"
#include "../utils/utils.h"

// Define a grav struct to store gravity parameters
typedef struct grav {
  double earth_mass;   // mass of the Earth in kg
  double earth_radius; // radius of the Earth in meters
  double grav_const;   // gravitational constant in m^3/kg/s^2
  double grav_g0; // acceleration due to gravity at the geoid surface in m/s^2
  double geoid_height_error;
  double geoid_height_std;

} grav;

/**
 * Initializes gravity parameters.
 *
 * @param run_params Pointer to run parameters.
 * @return Initialized gravity model.
 */
grav init_grav(runparams *run_params, int include_grav_error) {

  grav grav;
  // Define parameters for gravity
  grav.earth_mass = 5.972e24;
  grav.earth_radius = EARTH_RADIUS_M;
  grav.grav_const = 6.67408e-11;
  grav.grav_g0 = -grav.grav_const * grav.earth_mass /
                 (grav.earth_radius * grav.earth_radius);
  grav.geoid_height_std = run_params->geoid_height_error;
  if (include_grav_error != 0) {
    // Set nonzero geoid height error
    grav.geoid_height_error = ran_gaussian(grav.geoid_height_std);
  } else {
    grav.geoid_height_error = 0;
  }

  return grav;
}

#endif