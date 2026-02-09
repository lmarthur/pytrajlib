#ifndef LIFT_H
#define LIFT_H


#include <math.h>
#include "../vehicle.h"
#include "../utils.h"

void update_lift(runparams *run_params, state *state, cart_vector *a_command, atm_cond *atm_cond, vehicle *vehicle, double time_step){
    /*
    Simulates maneuverability of a reentry vehicle by applying a commanded acceleration vector with a time delay and realistic atmospheric model

    INPUTS:
    ----------
        true_state: state *
            pointer to the state of the vehicle
        a_command: cart_vector *
            pointer to the commanded acceleration vector
        atm_cond: atm_cond *
            pointer to the atmospheric conditions
        vehicle: vehicle *
            pointer to the vehicle struct
        time_step: double
            time step for the simulation
    */

    // Calculate the time constant of the vehicle
    double time_constant = rv_time_constant(vehicle, state, atm_cond);
    double max_a_exec = 25 * 9.8; // maximum acceleration in m/s^2
    double aoa_max = 10 * M_PI / 180; // maximum angle of attack in radians
    double deflection_time = run_params->deflection_time; // time to reach maximum flap deflection (seconds), this should be defined in runparams
    double deflection_rate = aoa_max / deflection_time; // deflection rate in rad/seconds
    
    // get current trim angle
    double trim_angle_x = state->ax_lift * aoa_max / sqrt(max_a_exec/3); // this is the current flap deflection in the x-direction
    double trim_angle_y = state->ay_lift * aoa_max / sqrt(max_a_exec/3); // this is the current flap deflection in the y-direction
    double trim_angle_z = state->az_lift * aoa_max / sqrt(max_a_exec/3); // this is the current flap deflection in the z-direction
    // printf("Current trim angles: trim_angle_x: %f, trim_angle_y: %f, trim_angle_z: %f\n", trim_angle_x, trim_angle_y, trim_angle_z);
    
    // Define target flap deflections
    double target_trim_angle_x = a_command->x / max_a_exec * aoa_max; // target flap deflection in the x-direction
    double target_trim_angle_y = a_command->y / max_a_exec * aoa_max; // target flap deflection in the y-direction
    double target_trim_angle_z = a_command->z / max_a_exec * aoa_max; // target flap deflection in the z-direction
    // printf("Target trim angles: target_trim_angle_x: %f, target_trim_angle_y: %f, target_trim_angle_z: %f\n", target_trim_angle_x, target_trim_angle_y, target_trim_angle_z);

    // Update the current flap deflections with linear step
    // Case 0: current flap deflection is within deflection_rate*dt of target flap deflection
    if (fabs(trim_angle_x - target_trim_angle_x) < deflection_rate * time_step){
        trim_angle_x = target_trim_angle_x;
        // printf("Case 0");
    }
    // Case 1: current flap deflection is less than target flap deflection
    if (trim_angle_x < target_trim_angle_x){
        trim_angle_x = trim_angle_x + deflection_rate * time_step/aoa_max;
        // printf("Case 1");
    }
    // Case 2: current flap deflection is greater than target flap deflection
    else if (trim_angle_x > target_trim_angle_x){
        trim_angle_x = trim_angle_x - deflection_rate * time_step/aoa_max;
        // printf("Case 2");
    }

    // Repeat for y and z components
    if (fabs(trim_angle_y - target_trim_angle_y) < deflection_rate * time_step){
        trim_angle_y = target_trim_angle_y; // within range, set to target
        // printf("Case 0 for Y\n");
    }
    if (trim_angle_y < target_trim_angle_y){
        trim_angle_y = trim_angle_y + deflection_rate * time_step/aoa_max; // increment towards target
        // printf("Case 1 for Y\n");
    }
    else if (trim_angle_y > target_trim_angle_y){
        trim_angle_y = trim_angle_y - deflection_rate * time_step/aoa_max; // decrement towards target
        // printf("Case 2 for Y\n");
    }

    if (fabs(trim_angle_z - target_trim_angle_z) < deflection_rate * time_step){
        trim_angle_z = target_trim_angle_z; // within range, set to target
        // printf("Case 0 for Z\n");
    }
    if (trim_angle_z < target_trim_angle_z){
        trim_angle_z = trim_angle_z + deflection_rate * time_step/aoa_max; // increment towards target
        // printf("Case 1 for Z\n");
    }
    else if (trim_angle_z > target_trim_angle_z){
        trim_angle_z = trim_angle_z - deflection_rate * time_step/aoa_max; // decrement towards target
        // printf("Case 2 for Z\n");
    }

    // if target flap deflection is nonphysical, set to max deflection
    if (trim_angle_x > aoa_max) {
        trim_angle_x = aoa_max;
    }
    if (trim_angle_x < -aoa_max) {
        trim_angle_x = -aoa_max;
    }
    if (trim_angle_y > aoa_max) {
        trim_angle_y = aoa_max;
    }
    if (trim_angle_y < -aoa_max) {
        trim_angle_y = -aoa_max;
    }
    if (trim_angle_z > aoa_max) {
        trim_angle_z = aoa_max;
    }
    if (trim_angle_z < -aoa_max) {
        trim_angle_z = -aoa_max;
    }

    // Update the acceleration "commands" based on the current flap deflections
    double a_transfer_x = trim_angle_x * max_a_exec/aoa_max;
    double a_transfer_y = trim_angle_y * max_a_exec/aoa_max;
    double a_transfer_z = trim_angle_z * max_a_exec/aoa_max;
    // printf("a_transfer_x: %f, a_transfer_y: %f, a_transfer_z: %f\n", a_transfer_x, a_transfer_y, a_transfer_z);

    state->ax_lift = state->ax_lift + (a_transfer_x - state->ax_lift) * time_step / time_constant;
    state->ay_lift = state->ay_lift + (a_transfer_y - state->ay_lift) * time_step / time_constant;
    state->az_lift = state->az_lift + (a_transfer_z - state->az_lift) * time_step / time_constant;

    // state->ax_lift = state->ax_lift + (a_command->x - state->ax_lift) * time_step / time_constant;
    // state->ay_lift = state->ay_lift + (a_command->y - state->ay_lift) * time_step / time_constant;
    // state->az_lift = state->az_lift + (a_command->z - state->az_lift) * time_step / time_constant;

    double velocity = sqrt(state->vx*state->vx + state->vy*state->vy + state->vz*state->vz);
    double a_exec_mag = sqrt(state->ax_lift*state->ax_lift + state->ay_lift*state->ay_lift + state->az_lift*state->az_lift);
    
    double angle_of_attack = vehicle->current_mass * a_exec_mag / (0.5 * atm_cond->density * velocity * velocity * vehicle->rv.c_l_alpha);
    
    if (angle_of_attack > aoa_max || angle_of_attack < -aoa_max){
        double new_a_exec_mag = 0.5 * atm_cond->density * velocity * velocity * vehicle->rv.c_l_alpha * aoa_max / vehicle->current_mass;
        state->ax_lift = new_a_exec_mag * state->ax_lift / a_exec_mag;
        state->ay_lift = new_a_exec_mag * state->ay_lift / a_exec_mag;
        state->az_lift = new_a_exec_mag * state->az_lift / a_exec_mag;
    }

    if (a_exec_mag > max_a_exec){
        double new_a_exec_mag = max_a_exec;
        state->ax_lift = new_a_exec_mag * state->ax_lift / a_exec_mag;
        state->ay_lift = new_a_exec_mag * state->ay_lift / a_exec_mag;
        state->az_lift = new_a_exec_mag * state->az_lift / a_exec_mag;
    }
}


#endif