#ifndef PHYSICS_H
#define PHYSICS_H

#include <math.h>
#include "vehicle.h"
#include "gravity.h"
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

double rv_time_constant(vehicle *vehicle, state *true_state, atm_cond *atm_cond){
    /*
    Calculates the time constant of the reentry vehicle based on the current state

    INPUTS:
    ----------
        vehicle: vehicle *
            pointer to the vehicle struct
        true_state: state *
            pointer to the true state of the vehicle
        atm_cond: atm_cond *
            pointer to the atmospheric conditions

    OUTPUTS:
    ----------
        double: time_constant
            time constant of the vehicle
    */

    // Get the current velocity
    double velocity = sqrt(true_state->vx*true_state->vx + true_state->vy*true_state->vy + true_state->vz*true_state->vz);
    
    // Calculate the time constant
    double time_constant = sqrt(-2 * vehicle->rv.Iyy / (vehicle->rv.c_m_alpha * vehicle->rv.rv_area * atm_cond->density * pow(velocity, 2) * vehicle->rv.rv_length));

    return time_constant;
}

// Define a series of functions to calculate acceleration components

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

void update_drag(runparams *run_params, vehicle *vehicle, atm_cond *atm_cond, state *state, double *step_timer){
    /*
    Updates the drag acceleration components

    INPUTS:
    ----------
        vehicle: vehicle *
            pointer to the vehicle struct
        atm_cond: atm_cond *
            pointer to the atmospheric conditions
        state: state *
            pointer to the state struct
    */
    
    // Get the relative airspeed 
    double cart_wind[3];
    double spher_wind[3] = {atm_cond->vertical_wind, atm_cond->zonal_wind, atm_cond->meridional_wind};
    double spher_coords[3];
    double cart_coords[3] = {state->x, state->y, state->z};
    cartcoords_to_sphercoords(cart_coords, spher_coords);
    // printf("Cartesian coordinates: %f, %f, %f\n", cart_coords[0], cart_coords[1], cart_coords[2]);
    // printf("Spherical coordinates: %f, %f, %f\n", spher_coords[0], spher_coords[1], spher_coords[2]);
    sphervec_to_cartvec(spher_wind, cart_wind, spher_coords);
    // printf("Wind vector: %f, %f, %f\n", spher_wind[0], spher_wind[1], spher_wind[2]);
    // printf("Cartesian wind vector: %f, %f, %f\n", cart_wind[0], cart_wind[1], cart_wind[2]);

    double v_rel[3] = {state->vx - cart_wind[0], state->vy - cart_wind[1], state->vz - cart_wind[2]};
    // print the relative velocity
    // printf("Relative velocity: %f, %f, %f\n", v_rel[0], v_rel[1], v_rel[2]);
    double v_rel_mag = sqrt(v_rel[0]*v_rel[0] + v_rel[1]*v_rel[1] + v_rel[2]*v_rel[2]);
    
    if (v_rel_mag < 1e-2){
        state->ax_drag = 0;
        state->ay_drag = 0;
        state->az_drag = 0;
        return;
    }

    // Calculate the drag acceleration components for a booster or reentry vehicle
    if (state->t > vehicle->booster.total_burn_time){
        // Calculate the drag acceleration components for a reentry vehicle
        double a_drag_mag = 0.5 * atm_cond->density * v_rel_mag * v_rel_mag * vehicle->rv.rv_area * vehicle->rv.c_d_0 / vehicle->current_mass;
        state->ax_drag = -a_drag_mag * v_rel[0] / v_rel_mag;
        state->ay_drag = -a_drag_mag * v_rel[1] / v_rel_mag;
        state->az_drag = -a_drag_mag * v_rel[2] / v_rel_mag;
        // printf("Drag acceleration: %f, %f, %f\n", state->ax_drag, state->ay_drag, state->az_drag);
    }
    else{
        // Calculate the drag acceleration components for a booster
        double a_drag_mag = 0.5 * atm_cond->density * v_rel_mag * v_rel_mag * vehicle->booster.area * vehicle->booster.c_d_0 / vehicle->current_mass;
        state->ax_drag = -a_drag_mag * v_rel[0] / v_rel_mag;
        state->ay_drag = -a_drag_mag * v_rel[1] / v_rel_mag;
        state->az_drag = -a_drag_mag * v_rel[2] / v_rel_mag;
    }

    // Add anomalous lift forces
    double dynamic_pressure = 0.5 * atm_cond->density * v_rel_mag * v_rel_mag; // dynamic pressure in Pascals (N/m^2)
    // printf("Dynamic pressure: %f\n", dynamic_pressure);
    if (run_params->run_type == 1){
        // printf("run_params->cl_pert: %f\n", run_params->cl_pert);
        state->ay_drag = state->ay_drag + 0.01 * run_params->cl_pert * dynamic_pressure * vehicle->rv.rv_area/vehicle->current_mass; // add lift in the y-direction for reentry vehicles
    }

    double time_constant = rv_time_constant(vehicle, state, atm_cond);
    if (run_params->run_type == 1 && (run_params->step_acc_mag != 0)){
        double step_acc_duration = run_params->step_acc_dur;
        
        if (run_params->step_acc_dur < 0){
            // If the step acceleration duration is negative, it means that the step duration is based on the time constant
            step_acc_duration = time_constant/M_PI_2; // set the step duration based on the time constant
            
        }
        if ((get_altitude(state->x, state->y, state->z) < run_params->step_acc_hgt) && (*step_timer < step_acc_duration)) {
            // start timer
            // printf("Time constant: %f seconds\n", time_constant);

            // if negative step_acc_mag, it means that the step acceleration duration is based on the dynamic pressure at the current altitude and velocity
            if (run_params->step_acc_mag < 0){
                // If the step acceleration magnitude is negative, it means that the step acceleration is based on the dynamic pressure
                run_params->step_acc_mag = 0.0315* dynamic_pressure * vehicle->rv.rv_radius * vehicle->rv.rv_length / vehicle->current_mass; // set the step acceleration magnitude based on the dynamic pressure
                printf("Step acceleration magnitude set to: %f m/s^2\n", run_params->step_acc_mag);
                printf("Anomaly impulse: %f Ns\n", run_params->step_acc_mag * step_acc_duration);
            }

            *step_timer += run_params->time_step_reentry; // increment the timer by the time step
            // printf("Step timer: %f seconds\n", *step_timer);
            // apply step function
            state->ay_drag += run_params->step_acc_mag;
            // printf("Applying step function anomaly: %f at altitude: %f and time: %f\n", run_params->step_acc_mag, get_altitude(state->x, state->y, state->z), *step_timer);

        }
  
    }

    return;
}

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