#ifndef INTEGRATOR_H
#define INTEGRATOR_H

#include <math.h>

#include "models/grav.h"
#include "models/state.h"
#include "models/vehicle.h"
#include "utils.h"

// Define a series of functions to calculate acceleration components

/**
 * Advance the state by one Euler-Maruyama step. Note, it is not strictly EM
 * because the position is updated using the velocity and the acceleration.
 *
 * @param state Pointer to state to be updated in place
 * @param time_step Integration time step in seconds
 */
void euler_maruyama_step(state *state, double time_step) {
  state->t = state->t + time_step;
  state->position =
      add(state->position,
          add(smultiply(state->velocity, time_step),
              smultiply(state->a_total, 0.5 * time_step * time_step)));
  state->velocity = add(state->velocity, smultiply(state->a_total, time_step));
}

#endif