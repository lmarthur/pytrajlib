#ifndef GRAVITY_H
#define GRAVITY_H

#include "integrator/args.h"
#include "math/linalg.h"
#include "models/state.h"

/**
 * Get the acceleration due to gravity.
 *
 * For a given distance from the center of the Earth $r$, the gravitational
 * acceleration at the surface $g_0$ and the radius of the Earth $r_e$:
 * $$
 * |\vec a_g| = g_0 \frac{r_e^2}{r^2}
 * $$
 *
 * @note The Earth's radius is the sum of the radius of the Earth assuming it is
 *       a sphere with a geoid error term. See models.gravity.
 *
 * @param t current flight time (seconds)
 * @param current_state True or estimated state. See State in the API
 * documentation.
 * @param args True or estimated DerivArgs. See DerivArgs in the API
 * documentation.
 *
 * @return Acceleration due to gravity in Cartesian coordinates x, y, z (m/s^2)
 */
cartvec get_gravitational_acceleration(double t, state current_state,
                                       integrator_args args) {
  // Distance from Earth's center
  double r = norm(current_state.position);

  // Radial gravitational acceleration magnitude
  double r_earth = R_EARTH + args.gravity.geoid_height_error;
  double ar_grav = args.gravity.grav_g0 * r_earth * r_earth / (r * r);

  // Acceleration vector in direction of Earth's center
  cartvec acceleration = smultiply(current_state.position, ar_grav / r);
  return acceleration;
}

#endif