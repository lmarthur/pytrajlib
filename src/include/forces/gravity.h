#ifndef GRAVITY_H
#define GRAVITY_H

#include <math.h>
#include "../vehicle.h"
#include "../grav.h"
#include "../utils.h"


void update_gravity(grav *grav, state *state){
    /*
    Updates the gravitational acceleration components

    INPUTS:
    ----------
        grav: grav *
            pointer to the grav struct
        state: state *
            pointer to the state struct
    */
    double r;
    // Non-perturbed gravity model
    // if (grav->perturb_flag == 0){
    //     // Calculate the gravitational acceleration components
    //     r = sqrt(state->x*state->x + state->y*state->y + state->z*state->z);
    // }
    // else{
    //     // 
    //     r = sqrt(state->x*state->x + state->y*state->y + state->z*state->z) + grav->geoid_height_error;
    // }
    r = sqrt(state->x*state->x + state->y*state->y + state->z*state->z);

    double ar_grav = grav->grav_g0 * pow((grav->earth_radius + grav->geoid_height_error), 2) / pow(r, 2);
    state->ax_grav = ar_grav * state->x / r;
    state->ay_grav = ar_grav * state->y / r;
    state->az_grav = ar_grav * state->z / r;

}

#endif