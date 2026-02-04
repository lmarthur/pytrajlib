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
 * Performs spatial linear interpolation between two states to find a point
 * at a specified altitude and interpolates time
 *
 * @param state_0 pointer to initial state of the vehicle
 * @param state_1 pointer to final state of the vehicle
 * @param target_altitude target altitude above sea level (in meters)
 * @param time_0 time corresponding to state_0 (seconds)
 * @param time_1 time corresponding to state_1 (seconds)
 * @param impact_time pointer to store the interpolated time at target altitude
 * @return impact_state state of the vehicle at the target altitude
 */
state impact_linterp(state *state_0, state *state_1, double target_altitude,
                     double time_0, double time_1, double *impact_time) {
    // Calculate the interpolation factor based on state altitude
    // Add 6371e3 to target_altitude to convert to radius
    double target_radius = target_altitude + 6371e3;
    double radius_0 = norm(state_0->position);
    double radius_1 = norm(state_1->position);
    double interp_factor = (radius_0 - target_radius) / (radius_0 - radius_1);

    // Interpolate time
    *impact_time = time_0 + interp_factor * (time_1 - time_0);

    state impact_state = *state_0;

    // Interpolate position and velocity
    impact_state.position =
        add(state_0->position,
            smultiply(subtract(state_1->position, state_0->position),
                      interp_factor));
    impact_state.velocity =
        add(state_0->velocity,
            smultiply(subtract(state_1->velocity, state_0->velocity),
                      interp_factor));
    impact_state.a_lift = add(
        state_0->a_lift,
        smultiply(subtract(state_1->a_lift, state_0->a_lift), interp_factor));
    impact_state.a_lift_avail =
        add(state_0->a_lift_avail,
            smultiply(subtract(state_1->a_lift_avail, state_0->a_lift_avail),
                      interp_factor));

    return impact_state;
}

#endif