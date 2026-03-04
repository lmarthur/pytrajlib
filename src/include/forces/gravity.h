#ifndef GRAVITY_H
#define GRAVITY_H

#include <math.h>

#include "../models/grav.h"
#include "../models/state.h"
#include "../models/vehicle.h"
#include "../utils.h"

/**
 * Update gravitational acceleration for the current state position.
 *
 * @param grav Pointer to gravity model parameters
 * @param state Pointer to state updated with gravity acceleration
 */
void update_gravity(grav *grav, state *state) {
  double r;

  r = norm(state->position);

  double ar_grav = grav->grav_g0 *
                   pow((grav->earth_radius + grav->geoid_height_error), 2) /
                   pow(r, 2);
  state->a_grav = smultiply(state->position, ar_grav / r);
}

#endif