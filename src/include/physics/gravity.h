#ifndef GRAVITY_H
#define GRAVITY_H

#include <math.h>

#include "../models/grav.h"
#include "../models/state.h"
#include "../models/vehicle.h"
#include "../utils/utils.h"

/**
The gravity model assumes a spherical earth of radius $R_e = 6,371,000$ m, and
uncertainties in geoid height are modeled with an uncertainty parameter $\delta
h$ as described by Arthur and Kemp (2025). The acceleration due to gravity is a
function of the geoid height, current position, represented as the distance in
meters from the center of the Earth $r$, and the gravitational acceleration at
the surface $g_0$ (m/$s^2$):
\begin{align}
\mathbf a_\text{grav} = -g_0 \frac{(R_e + \delta h)^2}{r^2} \hat{\mathbf r}.
\end{align}
 * @param grav Pointer to gravity model parameters
 * @param state Pointer to state updated with gravity acceleration
 */
cartvec get_gravity_acc(grav *grav, state *state) {
  double r;

  r = norm(state->position);

  double ar_grav = grav->grav_g0 *
                   pow((grav->earth_radius + grav->geoid_height_error), 2) /
                   pow(r, 2);
  return smultiply(state->position, ar_grav / r);
}

#endif