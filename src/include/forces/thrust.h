#ifndef THRUST_H
#define THRUST_H

#include <math.h>
#include "../vehicle.h"
#include "../grav.h"
#include "../models/atmosphere.h"
#include "../utils.h"

void update_thrust(vehicle *vehicle, state *state){
    /*
    Updates the thrust acceleration components

    INPUTS:
    ----------
        vehicle: vehicle *
            pointer to the vehicle struct
        state: state *
            pointer to the state struct
    */
    double a_thrust_mag;

    if (state->t > vehicle->booster.total_burn_time){
        state->ax_thrust = 0;
        state->ay_thrust = 0;
        state->az_thrust = 0;
        return;
    }
    
    // Get the current stage
    int stage = 0;
    if (state->t > vehicle->booster.burn_time[0]){
        stage = 1;
    }
    if (state->t > vehicle->booster.burn_time[0] + vehicle->booster.burn_time[1]){
        stage = 2;
    }

    // Calculate the thrust acceleration components
    a_thrust_mag = vehicle->booster.isp0[stage] * vehicle->booster.fuel_burn_rate[stage] / vehicle->current_mass;

    // Vertical thrust for the beginning of the flight
    if (state->t < 5){
        state->ax_thrust = a_thrust_mag;
        state->ay_thrust = 0;
        state->az_thrust = 0;
        return;
    }

    state->ax_thrust = a_thrust_mag * cos(state->theta_long) * cos(state->theta_lat);
    state->ay_thrust = a_thrust_mag * sin(state->theta_long) * cos(state->theta_lat);
    state->az_thrust = a_thrust_mag * sin(state->theta_lat);
    
}
#endif