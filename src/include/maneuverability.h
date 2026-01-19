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

void add_anomalous_lift_forces(runparams *run_params, vehicle *vehicle, atm_cond *atm_cond, state *state, double *step_timer, double v_rel_mag) {
    /*
    Simulates trajectory anomalies by adding lift and drag forces for a specified
    duration.

    */
    if ((get_altitude(state->x, state->y, state->z) < run_params->step_acc_hgt) && (*step_timer < run_params->step_acc_dur)) {
        double lift_magnitude = sqrt(state->ax_lift * state->ax_lift + state->ay_lift * state->ay_lift + state->az_lift * state->az_lift); // magnitude of the lift acceleration vector
        *step_timer += run_params->time_step_reentry; // increment the timer by the time step
        
        // Angle from lift
        double step_acc_angle;
        if (run_params->step_acc_angle < 0) {
            step_acc_angle = ran_flat(0, 2 * M_PI);
            run_params->step_acc_angle = step_acc_angle;
            // printf("angle of anomaly: %f degrees\n", step_acc_angle * 180 / M_PI);
        }
        else {
            step_acc_angle = run_params->step_acc_angle;
        }

        cart_vector drag = {state->ax_drag, state->ay_drag, state->az_drag};
        cart_vector lift = {state->ax_lift, state->ay_lift, state->az_lift};
        double drag_mag = sqrt(dot_product(drag, drag));
        
        cart_vector drag_unit = {state->ax_drag / drag_mag, state->ay_drag / drag_mag, state->az_drag / drag_mag};
        
        // Anomaly in the direction of lift
        double anomaly_lift_x = run_params->step_acc_mag * state->ax_lift / lift_magnitude;
        double anomaly_lift_y = run_params->step_acc_mag * state->ay_lift / lift_magnitude;
        double anomaly_lift_z = run_params->step_acc_mag * state->az_lift / lift_magnitude;

        cart_vector anomaly = {anomaly_lift_x, anomaly_lift_y, anomaly_lift_z};
        cart_vector drag_unit_cross_anomaly;
        cross_product(&drag_unit, &anomaly, &drag_unit_cross_anomaly);
        double drag_unit_dot_anomaly = dot_product(drag_unit, anomaly);

        // Anomaly rotated around the unit vector in the drag direction using Rodrigues' rotation formula
        double anomaly_x = anomaly_lift_x * cos(step_acc_angle) + drag_unit_cross_anomaly.x * sin(step_acc_angle) + drag_unit.x * drag_unit_dot_anomaly * (1 - cos(step_acc_angle));
        double anomaly_y = anomaly_lift_y * cos(step_acc_angle) + drag_unit_cross_anomaly.y * sin(step_acc_angle) + drag_unit.y * drag_unit_dot_anomaly * (1 - cos(step_acc_angle));
        double anomaly_z = anomaly_lift_z * cos(step_acc_angle) + drag_unit_cross_anomaly.z * sin(step_acc_angle) + drag_unit.z * drag_unit_dot_anomaly * (1 - cos(step_acc_angle));
        state->ax_drag += anomaly_x;
        state->ay_drag += anomaly_y;
        state->az_drag += anomaly_z;
    }
}

// void reentry_lift_drag(runparams *run_params, state *state, cart_vector *a_command, atm_cond *atm_cond, vehicle *vehicle, double time_step, double *step_timer){
//     /*
//     Simulates maneuverability of a reentry vehicle by applying a commanded acceleration vector with a time delay and realistic atmospheric model

//     INPUTS:
//     ----------
//         run_params: runparams *
//             pointer to the run parameters struct
//         state: state *
//             pointer to the state of the vehicle
//         a_command: cart_vector *
//             pointer to the commanded acceleration vector
//         atm_cond: atm_cond *
//             pointer to the atmospheric conditions
//         vehicle: vehicle *
//             pointer to the vehicle struct
//         time_step: double
//             time step for the simulation
//         step_timer: double *
//             pointer to the step timer. The step timer keeps track of elapsed time
//             for anomalous accelerations because they only last for step_acc_duration.
//     */

//     // First, get the lift acceleration

//     // Calculate the time constant of the vehicle
//     double time_constant = rv_time_constant(vehicle, state, atm_cond);
    
//     double max_flap_force = run_params->actuator_force * run_params->gearing_ratio * 1000; // maximum flap force in N
//     double max_lift_force = vehicle->rv.c_l_alpha * max_flap_force * (vehicle->rv.x_flap-vehicle->rv.x_com) / (vehicle->rv.c_m_alpha * vehicle->rv.rv_length); // maximum lift force in N, based on moment arm and lift properties
//     double max_a_exec = max_lift_force / vehicle->rv.rv_mass; // maximum acceleration that can be executed by the flaps in m/s^2
//     double aoa_max = 10 * M_PI / 180; // maximum angle of attack in radians
//     double deflection_max = M_PI / 6; // maximum flap deflection in radians (30 degrees)
//     double deflection_time = run_params->deflection_time * run_params->gearing_ratio; // time to reach maximum flap deflection (seconds), this should be defined in runparams
//     double deflection_rate = aoa_max / deflection_time; // deflection rate in rad/seconds

//     // Get the relative airspeed
//     double cart_wind[3];
//     double spher_wind[3] = {atm_cond->vertical_wind, atm_cond->zonal_wind, atm_cond->meridional_wind};
//     double spher_coords[3];
//     double cart_coords[3] = {state->x, state->y, state->z};
//     cartcoords_to_sphercoords(cart_coords, spher_coords);

//     sphervec_to_cartvec(spher_wind, cart_wind, spher_coords);
//     // Get the relative velocity vector
//     double v_rel[3] = {state->vx - cart_wind[0], state->vy - cart_wind[1], state->vz - cart_wind[2]};
//     double v_rel_mag = sqrt(v_rel[0]*v_rel[0] + v_rel[1]*v_rel[1] + v_rel[2]*v_rel[2]);

//     double altitude = get_altitude(state->x, state->y, state->z); // Get the altitude of the vehicle
//     cart_vector initial_lift_vector;
//     initial_lift_vector.x = state->ax_lift;
//     initial_lift_vector.y = state->ay_lift;
//     initial_lift_vector.z = state->az_lift;

//     double initial_lift_mag = sqrt(initial_lift_vector.x * initial_lift_vector.x + initial_lift_vector.y * initial_lift_vector.y + initial_lift_vector.z * initial_lift_vector.z); // magnitude of the initial lift acceleration vector
//     // Define a local coordinate system such that unit vector e_1 points in the direction of the relative velocity vector
//     // and e_2 points in the direction of the lift acceleration vector
//     // e_3 will be orthogonal to both e_1 and e_2 defined as e_3 = e_1 x e_2

//     // Special case for zero relative velocity or high altitude that simply returns the state with zero lift and drag
//     if (v_rel_mag < 1e-6 || altitude > 1e5) {
//         // If the relative velocity is zero, we cannot define a local coordinate system
//         // Set the lift and drag to zero and return the state
//         state->ax_lift = 0.0;
//         state->ay_lift = 0.0;
//         state->az_lift = 0.0;
//         state->ax_drag = 0.0;
//         state->ay_drag = 0.0;
//         state->az_drag = 0.0;

//         return;
//     }

    
//     cart_vector e_1, e_2, e_3;
//      // unit vector in the direction of the relative velocity vector
//     e_1.x = v_rel[0] / v_rel_mag;
//     e_1.y = v_rel[1] / v_rel_mag;
//     e_1.z = v_rel[2] / v_rel_mag;

//     // Special case for zero initial lift
//     if (initial_lift_mag < 1e-6) {
//         // If the initial lift magnitude is zero, define e_2 based on a cross product between e_1 and global z-axis

//         double global_z_axis[3] = {0.0, 0.0, 1.0}; // global z-axis unit vector
//         e_2.x = e_1.y * global_z_axis[2] - e_1.z * global_z_axis[1];
//         e_2.y = e_1.z * global_z_axis[0] - e_1.x * global_z_axis[2];
//         e_2.z = e_1.x * global_z_axis[1] - e_1.y * global_z_axis[0];

//         // Normalize e_2 to make it a unit vector
//         double e_2_mag = sqrt(e_2.x * e_2.x + e_2.y * e_2.y + e_2.z * e_2.z);
//         if (e_2_mag < 1e-6) {
//             // If e_2 magnitude is still zero, we cannot define a local coordinate system
//             state->ax_lift = 0.0;
//             state->ay_lift = 0.0;
//             state->az_lift = 0.0;
//             state->ax_drag = 0.0;
//             state->ay_drag = 0.0;
//             state->az_drag = 0.0;
        
//             return;
//         }
//         // normalize e_2
//         e_2.x /= e_2_mag;
//         e_2.y /= e_2_mag;
//         e_2.z /= e_2_mag;

//     } else {
//         // unit vector in the direction of the lift acceleration vector
//         e_2.x = initial_lift_vector.x / initial_lift_mag;
//         e_2.y = initial_lift_vector.y / initial_lift_mag;
//         e_2.z = initial_lift_vector.z / initial_lift_mag;
//     }

//     // Calculate the cross product to get e_3
//     e_3.x = e_1.y * e_2.z - e_1.z * e_2.y; // x-component of e_3
//     e_3.y = e_1.z * e_2.x - e_1.x * e_2.z; // y-component of e_3
//     e_3.z = e_1.x * e_2.y - e_1.y * e_2.x; // z-component of e_3

//     // Project the commanded acceleration vector onto the lift direction (e_2)
//     double a_command_e2 = (a_command->x * e_2.x + a_command->y * e_2.y + a_command->z * e_2.z);

//     // Project the commanded acceleration vector onto the e_3 direction
//     double a_command_e3 = (a_command->x * e_3.x + a_command->y * e_3.y + a_command->z * e_3.z);

//     // Update the control surface deflections based on the commanded acceleration vector
//     double pitch_deflection;  // pitch deflection is defined in the a_lift direction
//     double yaw_deflection = 0.0;    // yaw deflection is defined in the e_3 direction
    
//     // Define the current pitch deflection based on the current lift acceleration
//     pitch_deflection = initial_lift_mag * deflection_max / max_a_exec; // pitch deflection in radians

//     // Define the target flap deflections based on the commanded acceleration vector
//     double target_pitch_deflection = a_command_e2 * deflection_max / max_a_exec; // target pitch deflection in radians
//     double target_yaw_deflection = a_command_e3 * deflection_max / max_a_exec; // target yaw deflection in radians

//     // Case 0: If the current flap deflection is within deflection_rate*dt of target flap deflection
//     if (fabs(pitch_deflection - target_pitch_deflection) < deflection_rate * time_step){
//         pitch_deflection = target_pitch_deflection; // within range, set to target
//     }
//     // Case 1: If the current flap deflection is less than target flap deflection
//     else if (pitch_deflection < target_pitch_deflection){
//         pitch_deflection += deflection_rate * time_step; // increment towards target
//     }
//     // Case 2: If the current flap deflection is greater than target flap deflection
//     else if (pitch_deflection > target_pitch_deflection){
//         pitch_deflection -= deflection_rate * time_step; // decrement towards target
//     }

//     // Repeat for yaw deflection
//     if (fabs(yaw_deflection - target_yaw_deflection) < deflection_rate * time_step){
//         yaw_deflection = target_yaw_deflection; // within range, set to target
//     }
//     else if (yaw_deflection < target_yaw_deflection){
//         yaw_deflection += deflection_rate * time_step; // increment towards target
//     }
//     else if (yaw_deflection > target_yaw_deflection){
//         yaw_deflection -= deflection_rate * time_step; // decrement towards target
//     }

//     // Enforce limits on the flap deflections
//     if (pitch_deflection > deflection_max){
//         pitch_deflection = deflection_max; // enforce maximum flap deflection
//     }
//     else if (pitch_deflection < -deflection_max){
//         pitch_deflection = -deflection_max; // enforce minimum flap deflection
//     }
//     if (yaw_deflection > deflection_max){
//         yaw_deflection = deflection_max; // enforce maximum flap deflection
//     }
//     else if (yaw_deflection < -deflection_max){
//         yaw_deflection = -deflection_max; // enforce minimum flap deflection
//     }

//     // Update the transferred acceleration vector based on the current flap deflections
//     double a_transfer_e2 = max_a_exec * (pitch_deflection / deflection_max); // transferred acceleration in the e_2 direction
//     double a_transfer_e3 = max_a_exec * (yaw_deflection / deflection_max); // transferred acceleration in the e_3 direction

//     // Get the new lift acceleration and update the state struct
//     double a_lift_e2 = initial_lift_mag + (a_transfer_e2 - initial_lift_mag) * time_step / time_constant;
//     double a_lift_e3 = a_transfer_e3 * time_step / time_constant; // lift acceleration in the e_3 direction

//     // Enforce limits on the lift acceleration

//     // Enforce the lift acceleration direction to be orthogonal to the relative velocity vector

//     // Transform the lift acceleration back to the global Cartesian basis
//     double lift_acc_x = a_lift_e2 * e_2.x + a_lift_e3 * e_3.x; // x-component of the lift acceleration
//     double lift_acc_y = a_lift_e2 * e_2.y + a_lift_e3 * e_3.y; // y-component of the lift acceleration
//     double lift_acc_z = a_lift_e2 * e_2.z + a_lift_e3 * e_3.z; // z-component of the lift acceleration

//     // Update the state with the new lift acceleration
//     state->ax_lift = lift_acc_x; // update the x-component of the lift
//     state->ay_lift = lift_acc_y; // update the y-component of the lift
//     state->az_lift = lift_acc_z; // update the z-component of the lift

//     // Second, get the drag acceleration

//     // Get the new total angle of attack based on the lift acceleration
//     double lift_magnitude = sqrt(state->ax_lift * state->ax_lift + state->ay_lift * state->ay_lift + state->az_lift * state->az_lift); // magnitude of the lift acceleration vector
//     double aoa = lift_magnitude * aoa_max / max_a_exec; // angle of attack in radians

//     // Get the drag coefficient based on the angle of attack
//     double c_d = vehicle->rv.c_d_0 + fabs(vehicle->rv.c_d_alpha * aoa); // drag coefficient based on angle of attack
//     // Get the drag magnitude
//     double a_drag_mag = 0.5 * atm_cond->density * v_rel_mag * v_rel_mag * vehicle->rv.rv_area * c_d / vehicle->current_mass;
//     // Update the drag acceleration vector based on the drag magnitude and direction
//     state->ax_drag = -a_drag_mag * v_rel[0] / v_rel_mag;
//     state->ay_drag = -a_drag_mag * v_rel[1] / v_rel_mag;
//     state->az_drag = -a_drag_mag * v_rel[2] / v_rel_mag;


//     // // Add anomalous lift forces for reentry-only simulations
//     // if (run_params->run_type == 1){
//     //     add_anomalous_lift_forces(run_params, vehicle, atm_cond, state, step_timer, v_rel_mag);
//     // }

// }

void compute_lift_basis(state *state, double v_rel_mag, double *v_rel, cart_vector *e_1, cart_vector *e_2, cart_vector *e_3) {
    double altitude = get_altitude(state->x, state->y, state->z); // Get the altitude of the vehicle
    
    cart_vector initial_lift_vector;
    initial_lift_vector.x = state->ax_lift;
    initial_lift_vector.y = state->ay_lift;
    initial_lift_vector.z = state->az_lift;
    
    double initial_lift_mag = sqrt(initial_lift_vector.x * initial_lift_vector.x + initial_lift_vector.y * initial_lift_vector.y + initial_lift_vector.z * initial_lift_vector.z); // magnitude of the initial lift acceleration vector
    // Define a local coordinate system such that unit vector e_1 points in the direction of the relative velocity vector
    // and e_2 points in the direction of the lift acceleration vector
    // e_3 will be orthogonal to both e_1 and e_2 defined as e_3 = e_1 x e_2

    // Special case for zero relative velocity or high altitude that simply returns the state with zero lift and drag
    if (v_rel_mag < 1e-6 || altitude > 1e5) {
        // If the relative velocity is zero, we cannot define a local coordinate system
        // Set the lift and drag to zero and return the state
        state->ax_lift = 0.0;
        state->ay_lift = 0.0;
        state->az_lift = 0.0;
        state->ax_drag = 0.0;
        state->ay_drag = 0.0;
        state->az_drag = 0.0;

        return;
    }
    
     // unit vector in the direction of the relative velocity vector
    e_1->x = v_rel[0] / v_rel_mag;
    e_1->y = v_rel[1] / v_rel_mag;
    e_1->z = v_rel[2] / v_rel_mag;

    // Special case for zero initial lift
    if (initial_lift_mag < 1e-6) {
        // If the initial lift magnitude is zero, define e_2 based on a cross product between e_1 and global z-axis

        double global_z_axis[3] = {0.0, 0.0, 1.0}; // global z-axis unit vector
        e_2->x = e_1->y * global_z_axis[2] - e_1->z * global_z_axis[1];
        e_2->y = e_1->z * global_z_axis[0] - e_1->x * global_z_axis[2];
        e_2->z = e_1->x * global_z_axis[1] - e_1->y * global_z_axis[0];

        // Normalize e_2 to make it a unit vector
        double e_2_mag = sqrt(e_2->x * e_2->x + e_2->y * e_2->y + e_2->z * e_2->z);
        if (e_2_mag < 1e-6) {
            // If e_2 magnitude is still zero, we cannot define a local coordinate system
            state->ax_lift = 0.0;
            state->ay_lift = 0.0;
            state->az_lift = 0.0;
            state->ax_drag = 0.0;
            state->ay_drag = 0.0;
            state->az_drag = 0.0;
        
            return;
        }
        // normalize e_2
        e_2->x /= e_2_mag;
        e_2->y /= e_2_mag;
        e_2->z /= e_2_mag;

    } else {
        // unit vector in the direction of the lift acceleration vector
        e_2->x = initial_lift_vector.x / initial_lift_mag;
        e_2->y = initial_lift_vector.y / initial_lift_mag;
        e_2->z = initial_lift_vector.z / initial_lift_mag;
    }

    // Calculate the cross product to get e_3
    e_3->x = e_1->y * e_2->z - e_1->z * e_2->y; // x-component of e_3
    e_3->y = e_1->z * e_2->x - e_1->x * e_2->z; // y-component of e_3
    e_3->z = e_1->x * e_2->y - e_1->y * e_2->x; // z-component of e_3

}

// void reentry_lift_drag_dt(runparams *run_params, state *state, cart_vector *a_command, atm_cond *atm_cond, vehicle *vehicle, double time_step, double *step_timer){
//     /*
//     Simulates maneuverability of a reentry vehicle by applying a commanded acceleration vector with a time delay and realistic atmospheric model

//     INPUTS:
//     ----------
//         run_params: runparams *
//             pointer to the run parameters struct
//         state: state *
//             pointer to the state of the vehicle
//         a_command: cart_vector *
//             pointer to the commanded acceleration vector
//         atm_cond: atm_cond *
//             pointer to the atmospheric conditions
//         vehicle: vehicle *
//             pointer to the vehicle struct
//         time_step: double
//             time step for the simulation
//         step_timer: double *
//             pointer to the step timer. The step timer keeps track of elapsed time
//             for anomalous accelerations because they only last for step_acc_duration.
//     */
//     // Calculate the time constant of the vehicle
//     double time_constant = rv_time_constant(vehicle, state, atm_cond);
    
//     // Calculate and define maximum parameters
//     double max_flap_force = run_params->actuator_force * run_params->gearing_ratio * 1000; // maximum flap force in N
//     double max_lift_force = vehicle->rv.c_l_alpha * max_flap_force * (vehicle->rv.x_flap-vehicle->rv.x_com) / (vehicle->rv.c_m_alpha * vehicle->rv.rv_length); // maximum lift force in N, based on moment arm and lift properties
//     double max_a_exec = max_lift_force / vehicle->rv.rv_mass; // maximum acceleration that can be executed by the flaps in m/s^2
//     double aoa_max = 10 * M_PI / 180; // maximum angle of attack in radians
//     double deflection_time = run_params->deflection_time * run_params->gearing_ratio; // time to reach maximum flap deflection (seconds), this should be defined in runparams
//     double jerk_max = max_a_exec / deflection_time; // maximum jerk in m/s^3
    
//     // Prevent the integrator from overshooting the target lift by clipping the 
//     // available lift acceleration to the previous step's targeted lift. This 
//     // simulates the linear motion of the actuator up to exactly the achievable  
//     // commanded acceleration. 
//     int x_increasing_and_passed_target = state->d_a_lift_avail_x_dt > 0 && state->ax_lift_avail > state->ax_lift_target;
//     int x_decreasing_and_passed_target = state->d_a_lift_avail_x_dt < 0 && state->ax_lift_avail < state->ax_lift_target;
//     int y_increasing_and_passed_target = state->d_a_lift_avail_y_dt > 0 && state->ay_lift_avail > state->ay_lift_target;
//     int y_decreasing_and_passed_target = state->d_a_lift_avail_y_dt < 0 && state->ay_lift_avail < state->ay_lift_target;
//     int z_increasing_and_passed_target = state->d_a_lift_avail_z_dt > 0 && state->az_lift_avail > state->az_lift_target;
//     int z_decreasing_and_passed_target = state->d_a_lift_avail_z_dt < 0 && state->az_lift_avail < state->az_lift_target;
//     if (x_increasing_and_passed_target || x_decreasing_and_passed_target) {
//         state->ax_lift_avail = state->ax_lift_target;
//     }
//     if (y_increasing_and_passed_target || y_decreasing_and_passed_target) {
//         state->ay_lift_avail = state->ay_lift_target;
//     }
//     if (z_increasing_and_passed_target || z_decreasing_and_passed_target) {
//         state->az_lift_avail = state->az_lift_target;
//     }

//     // Get the relative airspeed
//     double cart_wind[3];
//     double spher_wind[3] = {atm_cond->vertical_wind, atm_cond->zonal_wind, atm_cond->meridional_wind};
//     double spher_coords[3];
//     double cart_coords[3] = {state->x, state->y, state->z};
//     cartcoords_to_sphercoords(cart_coords, spher_coords);
//     sphervec_to_cartvec(spher_wind, cart_wind, spher_coords);

//     // Get the relative velocity vector
//     double v_rel[3] = {state->vx - cart_wind[0], state->vy - cart_wind[1], state->vz - cart_wind[2]};
//     double v_rel_mag = sqrt(v_rel[0]*v_rel[0] + v_rel[1]*v_rel[1] + v_rel[2]*v_rel[2]);

//     // Get the local lift basis vectors
//     cart_vector e_1, e_2, e_3;
//     compute_lift_basis(state, v_rel_mag, v_rel, &e_1, &e_2, &e_3);

//     // Project the commanded acceleration vector onto the lift direction (e_2)
//     // and e_3 directions and restrict the commanded acceleration to the maximum
//     // achievable by the control surfaces
//     double a_command_e2 = (a_command->x * e_2.x + a_command->y * e_2.y + a_command->z * e_2.z);
//     double a_command_e3 = (a_command->x * e_3.x + a_command->y * e_3.y + a_command->z * e_3.z);
//     a_command_e2 = clip(a_command_e2, -max_a_exec, max_a_exec);
//     a_command_e3 = clip(a_command_e3, -max_a_exec, max_a_exec);

//     // Project the target lift acceleration back to the Cartesian basis
//     double target_lift_x = a_command_e2 * e_2.x + a_command_e3 * e_3.x;
//     double target_lift_y = a_command_e2 * e_2.y + a_command_e3 * e_3.y;
//     double target_lift_z = a_command_e2 * e_2.z + a_command_e3 * e_3.z;
//     state->ax_lift_target = target_lift_x;
//     state->ay_lift_target = target_lift_y;
//     state->az_lift_target = target_lift_z;

//     // If available is < target, increase lift, otherwise decrease lift at maximum jerk
//     // The available lift encodes the flap positions, which move at a constant angular velocity
//     // which is associated with a constant change in available lift acceleration. 
//     state->d_a_lift_avail_x_dt = sign(target_lift_x - state->ax_lift_avail) * jerk_max;
//     state->d_a_lift_avail_y_dt = sign(target_lift_y - state->ay_lift_avail) * jerk_max;
//     state->d_a_lift_avail_z_dt = sign(target_lift_z - state->az_lift_avail) * jerk_max;

//     // The true lift acceleration approaches the available lift acceleration 
//     // exponentially based on the aerodynamic time constant. tau simulates the time it
//     // takes for pressure to build up.
//     state->d_a_lift_x_dt = (state->ax_lift_avail - state->ax_lift) / time_constant;
//     state->d_a_lift_y_dt = (state->ay_lift_avail - state->ay_lift) / time_constant;
//     state->d_a_lift_z_dt = (state->az_lift_avail - state->az_lift) / time_constant;

//     double lift_magnitude = sqrt(state->ax_lift * state->ax_lift + state->ay_lift * state->ay_lift + state->az_lift * state->az_lift); // magnitude of the lift acceleration vector
//     double aoa = lift_magnitude * aoa_max / max_a_exec; // angle of attack in radians

//     // Get the drag coefficient based on the angle of attack
//     double c_d = vehicle->rv.c_d_0 + fabs(vehicle->rv.c_d_alpha * aoa); // drag coefficient based on angle of attack
//     // Get the drag magnitude
//     double a_drag_mag = 0.5 * atm_cond->density * v_rel_mag * v_rel_mag * vehicle->rv.rv_area * c_d / vehicle->current_mass;
//     // Update the drag acceleration vector based on the drag magnitude and direction
//     state->ax_drag = -a_drag_mag * v_rel[0] / v_rel_mag;
//     state->ay_drag = -a_drag_mag * v_rel[1] / v_rel_mag;
//     state->az_drag = -a_drag_mag * v_rel[2] / v_rel_mag;

//     // // Add anomalous lift forces for reentry-only simulations
//     // if (run_params->run_type == 1){
//     //     add_anomalous_lift_forces(run_params, vehicle, atm_cond, state, step_timer, v_rel_mag);
//     // }

// }

double get_max_a_exec(runparams *run_params, vehicle *vehicle) {
    /*
    Calculate the maximum achievable acceleration by the control flaps.

    INPUTS:
    ----------
        run_params: runparams *
            pointer to the run parameters struct
        vehicle: vehicle *
            pointer to the vehicle struct

    OUTPUTS:
    ----------
        double: max_a_exec
            maximum acceleration that can be executed by the flaps in m/s^2
    */
    double max_flap_force = run_params->actuator_force * run_params->gearing_ratio * 1000; // maximum flap force in N
    double max_lift_force = vehicle->rv.c_l_alpha * max_flap_force * (vehicle->rv.x_flap - vehicle->rv.x_com) / (vehicle->rv.c_m_alpha * vehicle->rv.rv_length); // maximum lift force in N
    double max_a_exec = max_lift_force / vehicle->rv.rv_mass; // maximum acceleration in m/s^2
    
    return max_a_exec;
}

double get_jerk_max(runparams *run_params, vehicle *vehicle) {
    /*
    Calculate the maximum jerk (rate of change of acceleration).

    INPUTS:
    ----------
        run_params: runparams *
            pointer to the run parameters struct
        vehicle: vehicle *
            pointer to the vehicle struct

    OUTPUTS:
    ----------
        double: jerk_max
            maximum jerk in m/s^3
    */
    double max_a_exec = get_max_a_exec(run_params, vehicle);
    double deflection_time = run_params->deflection_time * run_params->gearing_ratio;
    double jerk_max = max_a_exec / deflection_time;
    
    return jerk_max;
}

double get_acc_resolution(runparams *run_params, vehicle *vehicle) {
    /*
    Calculate the acceleration resolution based on actuator resolution.
    Based on ISO 3408-3 grade 5, we assume the actuator has a ±10 degree range
    with a 0.01 degree resolution.

    INPUTS:
    ----------
        run_params: runparams *
            pointer to the run parameters struct
        vehicle: vehicle *
            pointer to the vehicle struct

    OUTPUTS:
    ----------
        double: acc_resolution
            acceleration resolution in m/s^2
    */
    double max_a_exec = get_max_a_exec(run_params, vehicle);
    double deflection_max = M_PI / 6; // maximum flap deflection in radians (30 degrees)
    double actuator_resolution = 0.01 * M_PI / 180; // 0.01 degree resolution in radians
    
    // Acceleration resolution is proportional to the angular resolution
    double acc_resolution = max_a_exec * actuator_resolution / deflection_max;
    
    return acc_resolution;
}

void project_and_clip(cart_vector *e2, cart_vector *e3, cart_vector *arr, double max_val, cart_vector *result) {
    /*
    Project arr to e2 and e3 basis, clip to the max_val in that basis, then project back to the Cartesian basis.

    INPUTS:
    ----------
        e2: cart_vector *
            pointer to the e2 basis vector
        e3: cart_vector *
            pointer to the e3 basis vector
        arr: cart_vector *
            pointer to the array to project and clip
        max_val: double
            maximum value for clipping
        result: cart_vector *
            pointer to store the result

    OUTPUTS:
    ----------
        result: cart_vector *
            projected and clipped vector in Cartesian coordinates
    */
    // Project onto e2 and e3
    double arr_e2 = arr->x * e2->x + arr->y * e2->y + arr->z * e2->z;
    double arr_e3 = arr->x * e3->x + arr->y * e3->y + arr->z * e3->z;
    
    // Clip to max_val
    arr_e2 = clip(arr_e2, -max_val, max_val);
    arr_e3 = clip(arr_e3, -max_val, max_val);
    
    // Project back to Cartesian basis
    result->x = arr_e2 * e2->x + arr_e3 * e3->x;
    result->y = arr_e2 * e2->y + arr_e3 * e3->y;
    result->z = arr_e2 * e2->z + arr_e3 * e3->z;
}

void get_a_lift_avail_jerk(double t, state *true_state, state *estimated_state, 
                           runparams *run_params, vehicle *vehicle, 
                           atm_cond *true_atm, atm_cond *est_atm,
                           cart_vector *d_a_lift_avail_dt_true, 
                           cart_vector *d_a_lift_avail_dt_est) {
    /*
    Get the time derivative of the available lift acceleration.

    The available lift acceleration encodes the position of the control flaps.
    The control flaps are assumed to move at an instantaneous acceleration up to a fixed
    maximum angular velocity. We assume a proportional relationship between
    the available lift acceleration and the flap position, so the maximum angular velocity
    of the control flaps is equivalent to a maximum available jerk.

    To avoid oscillations, as the available acceleration approaches the commanded
    acceleration, the jerk reduces from the maximum jerk to a jerk proportional
    to the difference between. When the difference is less than the actuator
    resolution, the derivative is zero.

    The proportional navigation commands may produce a commanded lift acceleration 
    with a component in the direction of the velocity, but the control flaps will 
    only attempt to produce lift acceleration in the plane perpendicular to the 
    estimated relative velocity (e_2, e_3 directions).

    Only valid during reentry phase.

    INPUTS:
    ----------
        t: double
            current flight time (seconds)
        true_state: state *
            pointer to the true state
        estimated_state: state *
            pointer to the estimated state
        run_params: runparams *
            pointer to the run parameters struct
        vehicle: vehicle *
            pointer to the vehicle struct
        true_atm: atm_cond *
            pointer to the true atmospheric conditions
        est_atm: atm_cond *
            pointer to the estimated atmospheric conditions
        d_a_lift_avail_dt_true: cart_vector *
            pointer to store the true available lift jerk
        d_a_lift_avail_dt_est: cart_vector *
            pointer to store the estimated available lift jerk

    OUTPUTS:
    ----------
        d_a_lift_avail_dt_true: cart_vector *
            time derivative of true available lift acceleration (m/s^3)
        d_a_lift_avail_dt_est: cart_vector *
            time derivative of estimated available lift acceleration (m/s^3)
    */
    
    // Initialize output to zero
    d_a_lift_avail_dt_true->x = 0.0;
    d_a_lift_avail_dt_true->y = 0.0;
    d_a_lift_avail_dt_true->z = 0.0;
    d_a_lift_avail_dt_est->x = 0.0;
    d_a_lift_avail_dt_est->y = 0.0;
    d_a_lift_avail_dt_est->z = 0.0;
    
    // Determine if vehicle is in reentry phase
    double altitude = get_altitude(estimated_state->x, estimated_state->y, estimated_state->z);
    int is_reentry = (t > vehicle->booster.total_burn_time) && (altitude < 1e5);
    
    if (!is_reentry) {
        return;
    }
    
    // Calculate maximum parameters
    double max_a_exec = get_max_a_exec(run_params, vehicle);
    double jerk_max = get_jerk_max(run_params, vehicle);
    
    // Commanded acceleration is based on the aimpoint and the estimated state's
    // position and velocity
    cart_vector a_command = prop_nav(run_params, estimated_state);
    
    // Get the relative velocity for the estimated state
    double cart_wind[3];
    double spher_wind[3] = {est_atm->vertical_wind, est_atm->zonal_wind, est_atm->meridional_wind};
    double spher_coords[3];
    double cart_coords[3] = {estimated_state->x, estimated_state->y, estimated_state->z};
    cartcoords_to_sphercoords(cart_coords, spher_coords);
    sphervec_to_cartvec(spher_wind, cart_wind, spher_coords);
    
    double v_rel[3] = {estimated_state->vx - cart_wind[0], 
                       estimated_state->vy - cart_wind[1], 
                       estimated_state->vz - cart_wind[2]};
    double v_rel_mag = sqrt(v_rel[0]*v_rel[0] + v_rel[1]*v_rel[1] + v_rel[2]*v_rel[2]);
    
    // Get the lift basis vectors for the estimated state
    cart_vector est_e1, est_e2, est_e3;
    compute_lift_basis(estimated_state, v_rel_mag, v_rel, &est_e1, &est_e2, &est_e3);
    
    // Check if basis is valid (non-zero velocity and not too high altitude)
    int valid_basis = (v_rel_mag >= 1e-6) && (altitude <= 1e5);
    
    if (!valid_basis) {
        return;
    }
    
    // Project the commanded acceleration onto the estimated lift basis vectors e_2 and e_3
    // because all lift acceleration must be generated orthogonal to the relative velocity
    cart_vector a_target;
    project_and_clip(&est_e2, &est_e3, &a_command, max_a_exec, &a_target);
    
    // Change available lift at a rate proportional to the error, clipped to max jerk
    // a_avail_err = a_target - estimated_state->a_lift_avail
    double a_avail_err_x = a_target.x - estimated_state->ax_lift_avail;
    double a_avail_err_y = a_target.y - estimated_state->ay_lift_avail;
    double a_avail_err_z = a_target.z - estimated_state->az_lift_avail;
    
    double flap_gain = 100;
    // Apply proportional gain and clip to jerk limits
    d_a_lift_avail_dt_est->x = clip(flap_gain * a_avail_err_x, -jerk_max, jerk_max);
    d_a_lift_avail_dt_est->y = clip(flap_gain * a_avail_err_y, -jerk_max, jerk_max);
    d_a_lift_avail_dt_est->z = clip(flap_gain * a_avail_err_z, -jerk_max, jerk_max);
    
    // True and estimated available lift are the same because the available lift
    // encodes the flap positions (no noise in flap position)
    d_a_lift_avail_dt_true->x = d_a_lift_avail_dt_est->x;
    d_a_lift_avail_dt_true->y = d_a_lift_avail_dt_est->y;
    d_a_lift_avail_dt_true->z = d_a_lift_avail_dt_est->z;
    
    // Debug: Print when function is called with non-zero results
    double avail_jerk_mag = sqrt(d_a_lift_avail_dt_est->x * d_a_lift_avail_dt_est->x + 
                                  d_a_lift_avail_dt_est->y * d_a_lift_avail_dt_est->y + 
                                  d_a_lift_avail_dt_est->z * d_a_lift_avail_dt_est->z);
    // if (avail_jerk_mag > 1e-6) {
    //     printf("[DEBUG] get_a_lift_avail_jerk called at t=%.2f: avail_jerk_mag=%.6f m/s^3\n", t, avail_jerk_mag);
    // }
}

void get_a_lift_jerk_single_state(double t, state *state_ptr, runparams *run_params, 
                                  vehicle *vehicle, atm_cond *atm_cond, 
                                  cart_vector *d_a_lift_dt) {
    /*
    Get time derivative of the lift acceleration for a single state.

    The lift acceleration approaches the available lift acceleration exponentially
    based on the time constant: a(t) = a_avail * (1 - e^(-t/tau))

    INPUTS:
    ----------
        t: double
            current flight time (seconds)
        state_ptr: state *
            pointer to the state
        run_params: runparams *
            pointer to the run parameters struct
        vehicle: vehicle *
            pointer to the vehicle struct
        atm_cond: atm_cond *
            pointer to the atmospheric conditions
        d_a_lift_dt: cart_vector *
            pointer to store the lift jerk

    OUTPUTS:
    ----------
        d_a_lift_dt: cart_vector *
            time derivative of lift acceleration (m/s^3)
    */
    
    // Initialize output to zero
    d_a_lift_dt->x = 0.0;
    d_a_lift_dt->y = 0.0;
    d_a_lift_dt->z = 0.0;
    
    // Determine if vehicle is in reentry phase
    double altitude = get_altitude(state_ptr->x, state_ptr->y, state_ptr->z);
    int is_reentry = (t > vehicle->booster.total_burn_time) && (altitude < 1e5);
    
    if (!is_reentry) {
        return;
    }
    
    // Calculate maximum parameters
    double max_a_exec = get_max_a_exec(run_params, vehicle);
    
    // Get time constant to simulate pressure build-up
    double time_constant = rv_time_constant(vehicle, state_ptr, atm_cond);
    
    // Quantize available lift to the resolution of the actuator
    double acc_resolution = get_acc_resolution(run_params, vehicle);
    double a_lift_avail_x = round(state_ptr->ax_lift_avail / acc_resolution) * acc_resolution;
    double a_lift_avail_y = round(state_ptr->ay_lift_avail / acc_resolution) * acc_resolution;
    double a_lift_avail_z = round(state_ptr->az_lift_avail / acc_resolution) * acc_resolution;
    
    // Get the relative velocity
    double cart_wind[3];
    double spher_wind[3] = {atm_cond->vertical_wind, atm_cond->zonal_wind, atm_cond->meridional_wind};
    double spher_coords[3];
    double cart_coords[3] = {state_ptr->x, state_ptr->y, state_ptr->z};
    cartcoords_to_sphercoords(cart_coords, spher_coords);
    sphervec_to_cartvec(spher_wind, cart_wind, spher_coords);
    
    double v_rel[3] = {state_ptr->vx - cart_wind[0], 
                       state_ptr->vy - cart_wind[1], 
                       state_ptr->vz - cart_wind[2]};
    double v_rel_mag = sqrt(v_rel[0]*v_rel[0] + v_rel[1]*v_rel[1] + v_rel[2]*v_rel[2]);
    
    // Get the lift basis vectors
    cart_vector e1, e2, e3;
    compute_lift_basis(state_ptr, v_rel_mag, v_rel, &e1, &e2, &e3);
    
    // Check if basis is valid
    int valid_basis = (v_rel_mag >= 1e-6) && (altitude <= 1e5);
    
    if (!valid_basis) {
        return;
    }
    
    // The lift available to be generated by the current flap positions depends
    // on the attitude of the vehicle, so the available lift should be
    // projected onto the lift basis and clipped to the maximum achievable lift
    cart_vector a_lift_avail_vec = {a_lift_avail_x, a_lift_avail_y, a_lift_avail_z};
    cart_vector a_lift_avail_projected;
    project_and_clip(&e2, &e3, &a_lift_avail_vec, max_a_exec, &a_lift_avail_projected);
    
    // Calculate the jerk: (a_avail - a_lift) / time_constant
    d_a_lift_dt->x = (a_lift_avail_projected.x - state_ptr->ax_lift) / time_constant;
    d_a_lift_dt->y = (a_lift_avail_projected.y - state_ptr->ay_lift) / time_constant;
    d_a_lift_dt->z = (a_lift_avail_projected.z - state_ptr->az_lift) / time_constant;
    
    // Debug: Print when function is called with non-zero results
    double lift_jerk_mag = sqrt(d_a_lift_dt->x * d_a_lift_dt->x + 
                                 d_a_lift_dt->y * d_a_lift_dt->y + 
                                 d_a_lift_dt->z * d_a_lift_dt->z);
    // if (lift_jerk_mag > 1e-6) {
    //     printf("[DEBUG] get_a_lift_jerk_single_state at t=%.2f: lift_jerk_mag=%.6f m/s^3, time_constant=%.6f s\n", 
    //            t, lift_jerk_mag, time_constant);
    // }
}

void get_a_lift_jerk(double t, state *true_state, state *estimated_state, 
                    runparams *run_params, vehicle *vehicle, 
                    atm_cond *true_atm, atm_cond *est_atm,
                    cart_vector *d_a_lift_dt_true, cart_vector *d_a_lift_dt_est) {
    /*
    Get time derivative of the lift acceleration for both true and estimated states.

    The lift acceleration approaches the available lift acceleration exponentially
    based on the time constant: a(t) = a_avail * (1 - e^(-t/tau))

    The lift jerk is zero if it is not during the reentry phase or when the
    lift basis cannot be calculated.

    INPUTS:
    ----------
        t: double
            current flight time (seconds)
        true_state: state *
            pointer to the true state
        estimated_state: state *
            pointer to the estimated state
        run_params: runparams *
            pointer to the run parameters struct
        vehicle: vehicle *
            pointer to the vehicle struct
        true_atm: atm_cond *
            pointer to the true atmospheric conditions
        est_atm: atm_cond *
            pointer to the estimated atmospheric conditions
        d_a_lift_dt_true: cart_vector *
            pointer to store the true lift jerk
        d_a_lift_dt_est: cart_vector *
            pointer to store the estimated lift jerk

    OUTPUTS:
    ----------
        d_a_lift_dt_true: cart_vector *
            time derivative of true lift acceleration (m/s^3)
        d_a_lift_dt_est: cart_vector *
            time derivative of estimated lift acceleration (m/s^3)
    */
    
    get_a_lift_jerk_single_state(t, true_state, run_params, vehicle, true_atm, d_a_lift_dt_true);
    get_a_lift_jerk_single_state(t, estimated_state, run_params, vehicle, est_atm, d_a_lift_dt_est);
}

#endif