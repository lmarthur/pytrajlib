#ifndef INTERPOLATE_H
#define INTERPOLATE_H

#include "linalg.h"
#include "models/state.h"

/**
 * Linear interpolation function
 *
 * @param x value to interpolate
 * @param xs pointer to the x-values
 * @param ys pointer to the y-values
 * @param n number of data points
 * @return interpolated value
 */
double linterp(double x, double xs[], double ys[], int n) {
  // Initialize the output value
  double y = 0;

  // Find the two points to interpolate between
  int i = 0;
  while (x > xs[i]) {
    i++;
  }

  if (i == 0) {
    y = ys[0];
    return y;
  }

  // Perform the interpolation
  y = ys[i - 1] + (ys[i] - ys[i - 1]) * (x - xs[i - 1]) / (xs[i] - xs[i - 1]);

  return y;
}

/**
 * Performs spatial linear interpolation between two multistates to find a point
 * at a specified altitude Interpolates across true and estimated states, and
 * interpolates time
 *
 * @param multistate_0 pointer to initial multistate of the vehicle
 * @param multistate_1 pointer to final multistate of the vehicle
 * @param target_altitude target altitude above sea level (in meters)
 * @param time_0 time corresponding to multistate_0 (seconds)
 * @param time_1 time corresponding to multistate_1 (seconds)
 * @param impact_time pointer to store the interpolated time at target altitude
 * @return impact_multistate multistate of the vehicle at the target altitude
 */
multistate impact_linterp(multistate *multistate_0, multistate *multistate_1,
                          double target_altitude, double time_0, double time_1,
                          double *impact_time) {
  // Calculate the interpolation factor based on true state altitude
  // Add 6371e3 to target_altitude to convert to radius
  double target_radius = target_altitude + 6371e3;
  double radius_0 = norm(multistate_0->true_state.position);
  double radius_1 = norm(multistate_1->true_state.position);
  double interp_factor = (radius_0 - target_radius) / (radius_0 - radius_1);

  // Interpolate time
  *impact_time = time_0 + interp_factor * (time_1 - time_0);

  multistate impact_multistate = *multistate_0;

  // Array of state pointers for true and estimated states
  state *states_0[] = {&multistate_0->true_state, &multistate_0->est_state};
  state *states_1[] = {&multistate_1->true_state, &multistate_1->est_state};
  state *impact_states[] = {&impact_multistate.true_state,
                            &impact_multistate.est_state};

  // Interpolate true and estimated states
  for (int i = 0; i < 2; i++) {
    impact_states[i]->position =
        add(states_0[i]->position,
            smultiply(subtract(states_1[i]->position, states_0[i]->position),
                      interp_factor));
    impact_states[i]->velocity =
        add(states_0[i]->velocity,
            smultiply(subtract(states_1[i]->velocity, states_0[i]->velocity),
                      interp_factor));
    impact_states[i]->a_lift =
        add(states_0[i]->a_lift,
            smultiply(subtract(states_1[i]->a_lift, states_0[i]->a_lift),
                      interp_factor));
    impact_states[i]->a_lift_avail =
        add(states_0[i]->a_lift_avail,
            smultiply(
                subtract(states_1[i]->a_lift_avail, states_0[i]->a_lift_avail),
                interp_factor));
  }

  return impact_multistate;
}

#endif