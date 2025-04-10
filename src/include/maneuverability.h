#ifndef MANEUVERABILITY_H
#define MANEUVERABILITY_H

#include "trajectory.h"
#include "guidance.h"

state instant_maneuv(state *true_state, cart_vector *a_command){
    /*
    Simulates instantaneous maneuverability of the vehicle by applying a commanded acceleration vector with no time delay

    INPUTS:
    ----------
        true_state: state *
            pointer to the true state of the vehicle
        a_command: cart_vector *
            pointer to the commanded acceleration vector
    
    OUTPUTS:
    ----------
        state: updated_state
            state of the vehicle after the maneuver
    */

    // Initialize the new state
    state updated_state = *true_state;

    // Update the acceleration components
    updated_state.ax_lift = a_command->x;
    updated_state.ay_lift = a_command->y;
    updated_state.az_lift = a_command->z;

    // Update the total acceleration components
    updated_state.ax_total = updated_state.ax_grav + updated_state.ax_drag + updated_state.ax_lift + updated_state.ax_thrust;
    updated_state.ay_total = updated_state.ay_grav + updated_state.ay_drag + updated_state.ay_lift + updated_state.ay_thrust;
    updated_state.az_total = updated_state.az_grav + updated_state.az_drag + updated_state.az_lift + updated_state.az_thrust;

    return updated_state;
}

state perfect_maneuv(state *true_state, state *estimated_state, state *desired_state){
    /*
    Simulates perfect maneuverability by shifting the true state by the difference between the estimated and desired states

    INPUTS:
    ----------
        true_state: state *
            pointer to the true state of the vehicle
        estimated_state: state *
            pointer to the estimated state of the vehicle
        desired_state: state *
            pointer to the desired state of the vehicle

    OUTPUTS:
    ----------
        state: updated_state
            true state of the vehicle after the maneuver
    */

    // Initialize the new state
    state updated_state = *true_state;

    // Calculate the difference between the desired and estimated states
    updated_state.x = true_state->x + (desired_state->x - estimated_state->x);
    updated_state.y = true_state->y + (desired_state->y - estimated_state->y);
    updated_state.z = true_state->z + (desired_state->z - estimated_state->z);
    updated_state.vx = true_state->vx + (desired_state->vx - estimated_state->vx);
    updated_state.vy = true_state->vy + (desired_state->vy - estimated_state->vy);
    updated_state.vz = true_state->vz + (desired_state->vz - estimated_state->vz);
    updated_state.ax_grav = true_state->ax_grav + (desired_state->ax_grav - estimated_state->ax_grav);
    updated_state.ay_grav = true_state->ay_grav + (desired_state->ay_grav - estimated_state->ay_grav);
    updated_state.az_grav = true_state->az_grav + (desired_state->az_grav - estimated_state->az_grav);
    updated_state.ax_drag = true_state->ax_drag + (desired_state->ax_drag - estimated_state->ax_drag);
    updated_state.ay_drag = true_state->ay_drag + (desired_state->ay_drag - estimated_state->ay_drag);
    updated_state.az_drag = true_state->az_drag + (desired_state->az_drag - estimated_state->az_drag);
    updated_state.ax_thrust = true_state->ax_thrust + (desired_state->ax_thrust - estimated_state->ax_thrust);
    updated_state.ay_thrust = true_state->ay_thrust + (desired_state->ay_thrust - estimated_state->ay_thrust);
    updated_state.az_thrust = true_state->az_thrust + (desired_state->az_thrust - estimated_state->az_thrust);
    updated_state.ax_lift = true_state->ax_lift + (desired_state->ax_lift - estimated_state->ax_lift);
    updated_state.ay_lift = true_state->ay_lift + (desired_state->ay_lift - estimated_state->ay_lift);
    updated_state.az_lift = true_state->az_lift + (desired_state->az_lift - estimated_state->az_lift);
    updated_state.ax_total = true_state->ax_total + (desired_state->ax_total - estimated_state->ax_total);
    updated_state.ay_total = true_state->ay_total + (desired_state->ay_total - estimated_state->ay_total);
    updated_state.az_total = true_state->az_total + (desired_state->az_total - estimated_state->az_total);
    updated_state.theta_long = true_state->theta_long + (desired_state->theta_long - estimated_state->theta_long);
    updated_state.theta_lat = true_state->theta_lat + (desired_state->theta_lat - estimated_state->theta_lat);
    updated_state.initial_theta_lat_pert = true_state->initial_theta_lat_pert;
    updated_state.initial_theta_long_pert = true_state->initial_theta_long_pert;

    // Update the estimated state
    estimated_state->x = desired_state->x;
    estimated_state->y = desired_state->y;
    estimated_state->z = desired_state->z;
    estimated_state->vx = desired_state->vx;
    estimated_state->vy = desired_state->vy;
    estimated_state->vz = desired_state->vz;
    estimated_state->ax_grav = desired_state->ax_grav;
    estimated_state->ay_grav = desired_state->ay_grav;
    estimated_state->az_grav = desired_state->az_grav;
    estimated_state->ax_drag = desired_state->ax_drag;
    estimated_state->ay_drag = desired_state->ay_drag;
    estimated_state->az_drag = desired_state->az_drag;
    estimated_state->ax_thrust = desired_state->ax_thrust;
    estimated_state->ay_thrust = desired_state->ay_thrust;
    estimated_state->az_thrust = desired_state->az_thrust;
    estimated_state->ax_lift = desired_state->ax_lift;
    estimated_state->ay_lift = desired_state->ay_lift;
    estimated_state->az_lift = desired_state->az_lift;
    estimated_state->ax_total = desired_state->ax_total;
    estimated_state->ay_total = desired_state->ay_total;
    estimated_state->az_total = desired_state->az_total;
    estimated_state->theta_long = desired_state->theta_long;
    estimated_state->theta_lat = desired_state->theta_lat;
    estimated_state->initial_theta_lat_pert = desired_state->initial_theta_lat_pert;
    estimated_state->initial_theta_long_pert = desired_state->initial_theta_long_pert;

    return updated_state;
}

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