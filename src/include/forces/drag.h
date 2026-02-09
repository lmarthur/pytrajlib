#ifndef DRAG_H
#define DRAG_H


#include <math.h>
#include "../vehicle.h"
#include "../atmosphere.h"
#include "../utils.h"

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

    sphervec_to_cartvec(spher_wind, cart_wind, spher_coords);

    double v_rel[3] = {state->vx - cart_wind[0], state->vy - cart_wind[1], state->vz - cart_wind[2]};

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
        state->ay_drag = state->ay_drag + run_params->cl_pert * dynamic_pressure * vehicle->rv.rv_area/vehicle->current_mass; // add lift in the y-direction for reentry vehicles
    }
        
    if (run_params->run_type == 1 && (run_params->step_acc_mag != 0)){
        
        if ((get_altitude(state->x, state->y, state->z) < run_params->step_acc_hgt) && (*step_timer < run_params->step_acc_dur)) {
            // start timer
            *step_timer += run_params->time_step_reentry; // increment the timer by the time step
            // apply step function
            state->ay_drag += run_params->step_acc_mag;
            printf("Applying step function anomaly: %f at altitude: %f and time: %f\n", run_params->step_acc_mag, get_altitude(state->x, state->y, state->z), *step_timer);

        }
  
    }

    return;
}


#endif