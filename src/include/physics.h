#ifndef PHYSICS_H
#define PHYSICS_H

#include <math.h>
#include "vehicle.h"
#include "grav.h"
#include "atmosphere.h"
#include "utils.h"

// Define a struct to store the state of a vehicle in 3D space
typedef struct state{
    // State parameters
    double t; // time in seconds since launch
    double x; // x-coordinate in meters
    double y; // y-coordinate in meters
    double z; // z-coordinate in meters
    double vx; // x-velocity in meters per second
    double vy; // y-velocity in meters per second
    double vz; // z-velocity in meters per second
    double ax_grav; // x-acceleration due to gravity in meters per second squared
    double ay_grav; // y-acceleration due to gravity in meters per second squared
    double az_grav; // z-acceleration due to gravity in meters per second squared
    double ax_drag; // x-acceleration due to drag in meters per second squared
    double ay_drag; // y-acceleration due to drag in meters per second squared
    double az_drag; // z-acceleration due to drag in meters per second squared
    double ax_lift; // x-acceleration due to lift in meters per second squared
    double ay_lift; // y-acceleration due to lift in meters per second squared
    double az_lift; // z-acceleration due to lift in meters per second squared
    double ax_thrust; // x-acceleration due to thrust in meters per second squared
    double ay_thrust; // y-acceleration due to thrust in meters per second squared
    double az_thrust; // z-acceleration due to thrust in meters per second squared
    double ax_total; // total x-acceleration in meters per second squared
    double ay_total; // total y-acceleration in meters per second squared
    double az_total; // total z-acceleration in meters per second squared
    double initial_theta_long_pert; // initial perturbation in the longitudinal thrust angle in radians
    double initial_theta_lat_pert; // initial perturbation in the latitudinal thrust angle in radians
    double theta_long; // thrust angle in the longitudinal direction measured from the x-z plane in radians
    double theta_lat; // thrust angle in the latitudinal direction measured from the x-y plane in radians


} state;

// Define a series of functions to calculate acceleration components


void rk4step(state *state, double time_step){
    /*
    Calculates the new position and velocity of the vehicle using a 4th order Runge-Kutta method

    INPUTS:
    ----------
        state: state
            initial state of the vehicle
        time_step: double
            time step in seconds
    */

    // Define the Runge-Kutta coefficients
    double k1[6], k2[6], k3[6], k4[6];
    double l1[6], l2[6], l3[6], l4[6];

    // Calculate the k1 coefficients
    k1[0] = state->vx;
    k1[1] = state->vy;
    k1[2] = state->vz;
    k1[3] = state->ax_total;
    k1[4] = state->ay_total;
    k1[5] = state->az_total;

    // Calculate the l1 coefficients
    l1[0] = state->ax_total;
    l1[1] = state->ay_total;
    l1[2] = state->az_total;
    l1[3] = 0;
    l1[4] = 0;
    l1[5] = 0;

    // Calculate the k2 coefficients
    k2[0] = state->vx + 0.5 * time_step * k1[3];
    k2[1] = state->vy + 0.5 * time_step * k1[4];
    k2[2] = state->vz + 0.5 * time_step * k1[5];
    k2[3] = state->ax_total + 0.5 * time_step * l1[3];
    k2[4] = state->ay_total + 0.5 * time_step * l1[4];
    k2[5] = state->az_total + 0.5 * time_step * l1[5];

    // Calculate the l2 coefficients
    l2[0] = state->ax_total + 0.5 * time_step * l1[3];
    l2[1] = state->ay_total + 0.5 * time_step * l1[4];
    l2[2] = state->az_total + 0.5 * time_step * l1[5];
    l2[3] = 0;
    l2[4] = 0;
    l2[5] = 0;

    // Calculate the k3 coefficients
    k3[0] = state->vx + 0.5 * time_step * k2[3];
    k3[1] = state->vy + 0.5 * time_step * k2[4];
    k3[2] = state->vz + 0.5 * time_step * k2[5];
    k3[3] = state->ax_total + 0.5 * time_step * l2[3];
    k3[4] = state->ay_total + 0.5 * time_step * l2[4];
    k3[5] = state->az_total + 0.5 * time_step * l2[5];

    // Calculate the l3 coefficients
    l3[0] = state->ax_total + 0.5 * time_step * l2[3];
    l3[1] = state->ay_total + 0.5 * time_step * l2[4];
    l3[2] = state->az_total + 0.5 * time_step * l2[5];
    l3[3] = 0;
    l3[4] = 0;
    l3[5] = 0;

    // Calculate the k4 coefficients
    k4[0] = state->vx + time_step * k3[3];
    k4[1] = state->vy + time_step * k3[4];
    k4[2] = state->vz + time_step * k3[5];
    k4[3] = state->ax_total + time_step * l3[3];
    k4[4] = state->ay_total + time_step * l3[4];
    k4[5] = state->az_total + time_step * l3[5];

    // Calculate the l4 coefficients
    l4[0] = state->ax_total + time_step * l3[3];
    l4[1] = state->ay_total + time_step * l3[4];
    l4[2] = state->az_total + time_step * l3[5];
    l4[3] = 0;
    l4[4] = 0;
    l4[5] = 0;

    state->t = state->t + time_step;
    state->x = state->x + time_step / 6 * (k1[0] + 2*k2[0] + 2*k3[0] + k4[0]);
    state->y = state->y + time_step / 6 * (k1[1] + 2*k2[1] + 2*k3[1] + k4[1]);
    state->z = state->z + time_step / 6 * (k1[2] + 2*k2[2] + 2*k3[2] + k4[2]);
    state->vx = state->vx + time_step / 6 * (k1[3] + 2*k2[3] + 2*k3[3] + k4[3]);
    state->vy = state->vy + time_step / 6 * (k1[4] + 2*k2[4] + 2*k3[4] + k4[4]);
    state->vz = state->vz + time_step / 6 * (k1[5] + 2*k2[5] + 2*k3[5] + k4[5]);

}

#endif