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


void add_anomalous_lift_forces(runparams *run_params, vehicle *vehicle, atm_cond *atm_cond, state *state, double *step_timer, double v_rel[3], double v_rel_mag) {
    /*
    Simulates trajectory anomalies by adding lift and drag forces for a specified
    duration.

    Anomalies:
    - Drag due to shear stress from asymmetric transition to turbulence caused by 
        boundary layer excitations. The magnitude depends on the perturbed angle 
        of attack. Duration and height are free parameters.
    - Pitching mode excitations. The magnitude depends on the perturbed angle of 
        attack. Duration depends on the time constant. Height is a free parameter.
    - Asymmetric ablation modelled as a frozen perturbation to the lift coefficient.
    */
    // Add anomalous lift forces
    double dynamic_pressure = 0.5 * atm_cond->density * v_rel_mag * v_rel_mag; // dynamic pressure in Pascals (N/m^2)
    // Update the drag based on a perturbed coefficient of lift
    state->ay_drag = state->ay_drag + run_params->cl_pert * dynamic_pressure * vehicle->rv.rv_area/vehicle->current_mass; // add lift in the y-direction for reentry vehicles

    if (run_params->step_acc_mag != 0){
        double step_acc_duration = run_params->step_acc_dur;
        
        // If the step acceleration duration is negative, it means that the step duration is based on the time constant
        if (run_params->step_acc_dur < 0){
            double time_constant = rv_time_constant(vehicle, state, atm_cond);
            step_acc_duration = time_constant/M_PI_2; // set the step duration based on the time constant
        }
        if ((get_altitude(state->x, state->y, state->z) < run_params->step_acc_hgt) && (*step_timer < step_acc_duration)) {
            // if negative step_acc_mag, it means that the step acceleration duration is based on the dynamic pressure at the current altitude and velocity
            if (run_params->step_acc_mag < 0){
                // Calculate lift and drag coefficients based on perturbed angle of attack
                double c_d = vehicle->rv.c_d_alpha * run_params->aoa_pert;
                double c_l = vehicle->rv.c_l_alpha * run_params->aoa_pert;

                // Update drag and lift
                double a_drag_mag = dynamic_pressure * vehicle->rv.rv_area * c_d / vehicle->current_mass;
                double a_lift_mag = dynamic_pressure * vehicle->rv.rv_area * c_l / vehicle->current_mass;
                
                double theta = atan2(v_rel[1], v_rel[0]);
                state->ax_drag -= cos(theta) * a_drag_mag; 
                state->ay_drag -= sin(theta) * a_drag_mag;

                state->ax_lift -= sin(theta) * a_lift_mag;
                state->ay_lift += cos(theta) * a_lift_mag;
            }

            *step_timer += run_params->time_step_reentry; // increment the timer by the time step
        }
    }
}

void reentry_lift_drag(runparams *run_params, state *state, cart_vector *a_command, atm_cond *atm_cond, vehicle *vehicle, double time_step, double *step_timer){
    /*
    Simulates maneuverability of a reentry vehicle by applying a commanded acceleration vector with a time delay and realistic atmospheric model

    INPUTS:
    ----------
        run_params: runparams *
            pointer to the run parameters struct
        state: state *
            pointer to the state of the vehicle
        a_command: cart_vector *
            pointer to the commanded acceleration vector
        atm_cond: atm_cond *
            pointer to the atmospheric conditions
        vehicle: vehicle *
            pointer to the vehicle struct
        time_step: double
            time step for the simulation
        step_timer: double *
            pointer to the step timer. The step timer keeps track of elapsed time
            for anomalous accelerations because they only last for step_acc_duration.
    */

    // First, get the lift acceleration

    // Calculate the time constant of the vehicle
    double time_constant = rv_time_constant(vehicle, state, atm_cond);
    
    double max_flap_force = run_params->actuator_force * run_params->gearing_ratio * 1000; // maximum flap force in N
    double max_lift_force = vehicle->rv.c_l_alpha * max_flap_force * (vehicle->rv.x_flap-vehicle->rv.x_com) / (vehicle->rv.c_m_alpha * vehicle->rv.rv_length); // maximum lift force in N, based on moment arm and lift properties
    double max_a_exec = max_lift_force / vehicle->rv.rv_mass; // maximum acceleration that can be executed by the flaps in m/s^2
    double aoa_max = 10 * M_PI / 180; // maximum angle of attack in radians
    double deflection_max = M_PI / 6; // maximum flap deflection in radians (30 degrees)
    double deflection_time = run_params->deflection_time * run_params->gearing_ratio; // time to reach maximum flap deflection (seconds), this should be defined in runparams
    double deflection_rate = aoa_max / deflection_time; // deflection rate in rad/seconds

    // Get the relative airspeed
    double cart_wind[3];
    double spher_wind[3] = {atm_cond->vertical_wind, atm_cond->zonal_wind, atm_cond->meridional_wind};
    double spher_coords[3];
    double cart_coords[3] = {state->x, state->y, state->z};
    cartcoords_to_sphercoords(cart_coords, spher_coords);

    sphervec_to_cartvec(spher_wind, cart_wind, spher_coords);
    // Get the relative velocity vector
    double v_rel[3] = {state->vx - cart_wind[0], state->vy - cart_wind[1], state->vz - cart_wind[2]};
    double v_rel_mag = sqrt(v_rel[0]*v_rel[0] + v_rel[1]*v_rel[1] + v_rel[2]*v_rel[2]);

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

    
    cart_vector e_1, e_2, e_3;
     // unit vector in the direction of the relative velocity vector
    e_1.x = v_rel[0] / v_rel_mag;
    e_1.y = v_rel[1] / v_rel_mag;
    e_1.z = v_rel[2] / v_rel_mag;

    // Special case for zero initial lift
    if (initial_lift_mag < 1e-6) {
        // If the initial lift magnitude is zero, define e_2 based on a cross product between e_1 and global z-axis

        double global_z_axis[3] = {0.0, 0.0, 1.0}; // global z-axis unit vector
        e_2.x = e_1.y * global_z_axis[2] - e_1.z * global_z_axis[1];
        e_2.y = e_1.z * global_z_axis[0] - e_1.x * global_z_axis[2];
        e_2.z = e_1.x * global_z_axis[1] - e_1.y * global_z_axis[0];

        // Normalize e_2 to make it a unit vector
        double e_2_mag = sqrt(e_2.x * e_2.x + e_2.y * e_2.y + e_2.z * e_2.z);
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
        e_2.x /= e_2_mag;
        e_2.y /= e_2_mag;
        e_2.z /= e_2_mag;

    } else {
        // unit vector in the direction of the lift acceleration vector
        e_2.x = initial_lift_vector.x / initial_lift_mag;
        e_2.y = initial_lift_vector.y / initial_lift_mag;
        e_2.z = initial_lift_vector.z / initial_lift_mag;
    }

    // Calculate the cross product to get e_3
    e_3.x = e_1.y * e_2.z - e_1.z * e_2.y; // x-component of e_3
    e_3.y = e_1.z * e_2.x - e_1.x * e_2.z; // y-component of e_3
    e_3.z = e_1.x * e_2.y - e_1.y * e_2.x; // z-component of e_3

    // Project the commanded acceleration vector onto the lift direction (e_2)
    double a_command_e2 = (a_command->x * e_2.x + a_command->y * e_2.y + a_command->z * e_2.z);

    // Project the commanded acceleration vector onto the e_3 direction
    double a_command_e3 = (a_command->x * e_3.x + a_command->y * e_3.y + a_command->z * e_3.z);

    // Update the control surface deflections based on the commanded acceleration vector
    double pitch_deflection;  // pitch deflection is defined in the a_lift direction
    double yaw_deflection = 0.0;    // yaw deflection is defined in the e_3 direction
    
    // Define the current pitch deflection based on the current lift acceleration
    pitch_deflection = initial_lift_mag * deflection_max / max_a_exec; // pitch deflection in radians

    // Define the target flap deflections based on the commanded acceleration vector
    double target_pitch_deflection = a_command_e2 * deflection_max / max_a_exec; // target pitch deflection in radians
    double target_yaw_deflection = a_command_e3 * deflection_max / max_a_exec; // target yaw deflection in radians

    // Case 0: If the current flap deflection is within deflection_rate*dt of target flap deflection
    if (fabs(pitch_deflection - target_pitch_deflection) < deflection_rate * time_step){
        pitch_deflection = target_pitch_deflection; // within range, set to target
    }
    // Case 1: If the current flap deflection is less than target flap deflection
    else if (pitch_deflection < target_pitch_deflection){
        pitch_deflection += deflection_rate * time_step; // increment towards target
    }
    // Case 2: If the current flap deflection is greater than target flap deflection
    else if (pitch_deflection > target_pitch_deflection){
        pitch_deflection -= deflection_rate * time_step; // decrement towards target
    }

    // Repeat for yaw deflection
    if (fabs(yaw_deflection - target_yaw_deflection) < deflection_rate * time_step){
        yaw_deflection = target_yaw_deflection; // within range, set to target
    }
    else if (yaw_deflection < target_yaw_deflection){
        yaw_deflection += deflection_rate * time_step; // increment towards target
    }
    else if (yaw_deflection > target_yaw_deflection){
        yaw_deflection -= deflection_rate * time_step; // decrement towards target
    }

    // Enforce limits on the flap deflections
    if (pitch_deflection > deflection_max){
        pitch_deflection = deflection_max; // enforce maximum flap deflection
    }
    else if (pitch_deflection < -deflection_max){
        pitch_deflection = -deflection_max; // enforce minimum flap deflection
    }
    if (yaw_deflection > deflection_max){
        yaw_deflection = deflection_max; // enforce maximum flap deflection
    }
    else if (yaw_deflection < -deflection_max){
        yaw_deflection = -deflection_max; // enforce minimum flap deflection
    }

    // Update the transferred acceleration vector based on the current flap deflections
    double a_transfer_e2 = max_a_exec * (pitch_deflection / deflection_max); // transferred acceleration in the e_2 direction
    double a_transfer_e3 = max_a_exec * (yaw_deflection / deflection_max); // transferred acceleration in the e_3 direction

    // Get the new lift acceleration and update the state struct
    double a_lift_e2 = initial_lift_mag + (a_transfer_e2 - initial_lift_mag) * time_step / time_constant;
    double a_lift_e3 = a_transfer_e3 * time_step / time_constant; // lift acceleration in the e_3 direction

    // Enforce limits on the lift acceleration

    // Enforce the lift acceleration direction to be orthogonal to the relative velocity vector

    // Transform the lift acceleration back to the global Cartesian basis
    double lift_acc_x = a_lift_e2 * e_2.x + a_lift_e3 * e_3.x; // x-component of the lift acceleration
    double lift_acc_y = a_lift_e2 * e_2.y + a_lift_e3 * e_3.y; // y-component of the lift acceleration
    double lift_acc_z = a_lift_e2 * e_2.z + a_lift_e3 * e_3.z; // z-component of the lift acceleration

    // Update the state with the new lift acceleration
    state->ax_lift = lift_acc_x; // update the x-component of the lift
    state->ay_lift = lift_acc_y; // update the y-component of the lift
    state->az_lift = lift_acc_z; // update the z-component of the lift

    // Second, get the drag acceleration

    // Get the new total angle of attack based on the lift acceleration
    double lift_magnitude = sqrt(state->ax_lift * state->ax_lift + state->ay_lift * state->ay_lift + state->az_lift * state->az_lift); // magnitude of the lift acceleration vector
    double aoa = lift_magnitude * aoa_max / max_a_exec; // angle of attack in radians

    // Get the drag coefficient based on the angle of attack
    double c_d = vehicle->rv.c_d_0 + fabs(vehicle->rv.c_d_alpha * aoa); // drag coefficient based on angle of attack
    // Get the drag magnitude
    double a_drag_mag = 0.5 * atm_cond->density * v_rel_mag * v_rel_mag * vehicle->rv.rv_area * c_d / vehicle->current_mass;
    // Update the drag acceleration vector based on the drag magnitude and direction
    state->ax_drag = -a_drag_mag * v_rel[0] / v_rel_mag;
    state->ay_drag = -a_drag_mag * v_rel[1] / v_rel_mag;
    state->az_drag = -a_drag_mag * v_rel[2] / v_rel_mag;


    // Add anomalous lift forces for reentry-only simulations
    if (run_params->run_type == 1){
        add_anomalous_lift_forces(run_params, vehicle, atm_cond, state, step_timer, v_rel, v_rel_mag);
    }

}

#endif