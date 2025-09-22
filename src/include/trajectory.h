#ifndef TRAJECTORY_H
#define TRAJECTORY_H
#define _USE_MATH_DEFINES

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "utils.h"
#include "vehicle.h"
#include "gravity.h"
#include "atmosphere.h"
#include "physics.h"
#include "sensors.h"
#include "maneuverability.h"
#include "rng/rng.h"
#include "optimize/mnbrak.h"
#include "optimize/brent.h"

// Define a constant upper limit for the number of Monte Carlo runs
#define MAX_RUNS 10000

// Define a struct to store impact data
typedef struct impact_data{
    // Impact data
    state impact_states[MAX_RUNS];

} impact_data;

state init_true_state(runparams *run_params){
    /*
    Initializes a true state struct at the launch site with zero velocity and acceleration

    INPUTS:
    ----------
        run_params: runparams *
            pointer to the run parameters struct

    OUTPUTS:
    ----------
        state: state
            initial state of the vehicle
    */

    state state;
    // branch for initializing full trajectory run
    if (run_params->run_type == 0){
        state.t = 0;
        state.x = 6371e3 + run_params->initial_x_error * ran_gaussian(1);
        state.y = run_params->initial_pos_error * ran_gaussian(1);
        state.z = run_params->initial_pos_error * ran_gaussian(1);

        state.vx = run_params->initial_vel_error * ran_gaussian(1);
        state.vy = run_params->initial_vel_error * ran_gaussian(1);
        state.vz = run_params->initial_vel_error * ran_gaussian(1);
        
    }
    // branch for initializing reentry only run
    if (run_params->run_type == 1){
        state.t = 0;
        state.z = run_params->initial_pos_error * ran_gaussian(1);
        state.vz = run_params->initial_vel_error * ran_gaussian(1);

        // If the reentry angle is pi/2, drop the vehicle from 500km directly above the
        // aimpoint. Otherwise, set position to form a right triangle with 500km
        // above the aimpoint and the aimpoint.
        if (fabs(run_params->reentry_angle - M_PI_2) > 1e-5){
            state.x = 6371e3 + 500e3;
            state.y = 6371e3 / tan(run_params->reentry_angle);

            state.vx = -run_params->reentry_vel * sin(run_params->reentry_angle);
            state.vy = -run_params->reentry_vel * cos(run_params->reentry_angle);
            printf("vx, vy: %f, %f %f\n", state.vx, state.vy, sqrt(pow(state.vx, 2) + pow(state.vy, 2)));
        }
        else {
            state.x = 6371e3 + 500e3;
            state.y = 0;

            state.vx = -run_params->reentry_vel;
            state.vy = 0;
            state.vz = 0;
        }

        state.vx += run_params->initial_vel_error * ran_gaussian(1);
        state.vy += run_params->initial_vel_error * ran_gaussian(1);
        state.vz += run_params->initial_vel_error * ran_gaussian(1);
    }
    
    double initial_rot_pert = run_params->initial_angle_error * ran_gaussian(1);

    state.initial_theta_lat_pert = run_params->initial_angle_error * ran_gaussian(1) + run_params->theta_long * initial_rot_pert - fabs(run_params->theta_lat * initial_rot_pert);
    state.initial_theta_long_pert = run_params->initial_angle_error * ran_gaussian(1) - run_params->theta_lat * initial_rot_pert - fabs(run_params->theta_long * initial_rot_pert);
    state.theta_long = run_params->theta_long + state.initial_theta_long_pert;
    state.theta_lat = run_params->theta_lat + state.initial_theta_lat_pert;
        
    state.ax_grav = 0;
    state.ay_grav = 0;
    state.az_grav = 0;
    state.ax_drag = 0;
    state.ay_drag = 0;
    state.az_drag = 0;
    state.ax_lift = 0;
    state.ay_lift = 0;
    state.az_lift = 0;
    state.ax_thrust = 0;
    state.ay_thrust = 0;
    state.az_thrust = 0;
    state.ax_total = 0;
    state.ay_total = 0;
    state.az_total = 0;

    return state;
}

state init_est_state(runparams *run_params){
    /*
    Initializes an estimated state struct at the launch site with zero velocity and acceleration

    INPUTS:
    ----------
        run_params: runparams *
            pointer to the run parameters struct
    OUTPUTS:
    ----------
        state: state
            initial state of the vehicle
    */

    state state;
    if (run_params->run_type == 0){
        state.t = 0;
        state.x = 6371e3;
        state.y = 0;
        state.z = 0;

        state.vx = 0;
        state.vy = 0;
        state.vz = 0;
        
    }
    // branch for initializing reentry only run
    if (run_params->run_type == 1){
        state.t = 0;
        state.z = 0;
        state.vz = 0;

        // If the reentry angle is pi/2, drop the vehicle from 500km directly above the
        // aimpoint. Otherwise, set position to form a right triangle with 500km
        // above the aimpoint and the aimpoint.
        if (fabs(run_params->reentry_angle - M_PI_2) > 1e-5){
            state.x = 6371e3 + 500e3;
            state.y = 6371e3 / tan(run_params->reentry_angle);

            state.vx = -run_params->reentry_vel * sin(run_params->reentry_angle);
            state.vy = -run_params->reentry_vel * cos(run_params->reentry_angle);
        }
        else {
            state.x = 6371e3 + 500e3;
            state.y = 0;

            state.vx = -run_params->reentry_vel;
            state.vy = 0;
        }
    }

    state.theta_long = run_params->theta_long;
    state.theta_lat = run_params->theta_lat;
    state.initial_theta_lat_pert = 0;
    state.initial_theta_long_pert = 0;

    state.ax_grav = 0;
    state.ay_grav = 0;
    state.az_grav = 0;
    state.ax_drag = 0;
    state.ay_drag = 0;
    state.az_drag = 0;
    state.ax_lift = 0;
    state.ay_lift = 0;
    state.az_lift = 0;
    state.ax_thrust = 0;
    state.ay_thrust = 0;
    state.az_thrust = 0;
    state.ax_total = 0;
    state.ay_total = 0;
    state.az_total = 0;

    return state;
}

state impact_linterp(state *state_0, state *state_1){
    /*
    Performs a spatial linear interpolation between two states to find the impact point, velocity, and time

    INPUTS:
    ----------
        state_0: state *
            pointer to initial state of the vehicle
        state_1: state *
            pointer to final state of the vehicle
    OUTPUTS:
    ----------
        impact_state: state
            state of the vehicle at impact
    */

    // Calculate the interpolation factor
    double altitude_0 = sqrt(state_0->x*state_0->x + state_0->y*state_0->y + state_0->z*state_0->z) - 6371e3;
    double altitude_1 = sqrt(state_1->x*state_1->x + state_1->y*state_1->y + state_1->z*state_1->z) - 6371e3;
    double interp_factor = altitude_0 / (altitude_0 - altitude_1);

    // Perform the interpolation
    state impact_state = *state_0;
    impact_state.t = state_0->t + interp_factor * (state_1->t - state_0->t);
    impact_state.x = state_0->x + interp_factor * (state_1->x - state_0->x);
    impact_state.y = state_0->y + interp_factor * (state_1->y - state_0->y);
    impact_state.z = state_0->z + interp_factor * (state_1->z - state_0->z);
    impact_state.vx = state_0->vx + interp_factor * (state_1->vx - state_0->vx);
    impact_state.vy = state_0->vy + interp_factor * (state_1->vy - state_0->vy);
    impact_state.vz = state_0->vz + interp_factor * (state_1->vz - state_0->vz);
    impact_state.ax_grav = state_0->ax_grav + interp_factor * (state_1->ax_grav - state_0->ax_grav);
    impact_state.ay_grav = state_0->ay_grav + interp_factor * (state_1->ay_grav - state_0->ay_grav);
    impact_state.az_grav = state_0->az_grav + interp_factor * (state_1->az_grav - state_0->az_grav);
    impact_state.ax_drag = state_0->ax_drag + interp_factor * (state_1->ax_drag - state_0->ax_drag);
    impact_state.ay_drag = state_0->ay_drag + interp_factor * (state_1->ay_drag - state_0->ay_drag);
    impact_state.az_drag = state_0->az_drag + interp_factor * (state_1->az_drag - state_0->az_drag);
    impact_state.ax_lift = state_0->ax_lift + interp_factor * (state_1->ax_lift - state_0->ax_lift);
    impact_state.ay_lift = state_0->ay_lift + interp_factor * (state_1->ay_lift - state_0->ay_lift);
    impact_state.az_lift = state_0->az_lift + interp_factor * (state_1->az_lift - state_0->az_lift);
    impact_state.ax_thrust = state_0->ax_thrust + interp_factor * (state_1->ax_thrust - state_0->ax_thrust);
    impact_state.ay_thrust = state_0->ay_thrust + interp_factor * (state_1->ay_thrust - state_0->ay_thrust);
    impact_state.az_thrust = state_0->az_thrust + interp_factor * (state_1->az_thrust - state_0->az_thrust);
    impact_state.ax_total = state_0->ax_total + interp_factor * (state_1->ax_total - state_0->ax_total);
    impact_state.ay_total = state_0->ay_total + interp_factor * (state_1->ay_total - state_0->ay_total);
    impact_state.az_total = state_0->az_total + interp_factor * (state_1->az_total - state_0->az_total);


    return impact_state;
}

void output_impact(FILE *impact_file, impact_data *impact_data, int num_runs){
    /*
    Function that outputs the impact data struct to the impact file
    
    INPUTS:
    ----------
        impact_file: * FILE
            Pointer to the impact file stream
        impact_data: * impact_data
            Pointer to the impact data struct
        num_runs: int
            Number of Monte Carlo runs
    */

    // Iterate through the number of runs and output the impact data
    for (int i = 0; i < num_runs; i++){
        fprintf(impact_file, "%f, %f, %f, %f, %f, %f, %f\n", impact_data->impact_states[i].t, impact_data->impact_states[i].x, impact_data->impact_states[i].y, impact_data->impact_states[i].z, impact_data->impact_states[i].vx, impact_data->impact_states[i].vy, impact_data->impact_states[i].vz);
    }

    // Close the impact file
    fclose(impact_file);
    
}

void append_traj_data(FILE *traj_file, state new_state, state est_state, vehicle *vehicle, double a_command_total, double a_lift_total, atm_cond true_atm_cond, atm_cond est_atm_cond){
    /*
    Function that writes the current state of the vehicle to the trajectory file

    INPUTS:
    ----------
        traj_file: * FILE
            Pointer to the trajectory file stream
        new_state: state
            current state of the vehicle
        est_state: state
            estimated state of the vehicle
        vehicle: vehicle *
            pointer to the vehicle struct
        a_command_total: double
            total commanded lift acceleration
        a_lift_total: double
            total lift acceleration
        atm_cond: true_atm_cond
            atmospheric conditions at the current state
        
    */
    fprintf(traj_file, "%.15g, %.15g, %.15g, %.15g, %.15g, %.15g, %.15g, %.15g, %.15g, %.15g, %.15g, %.15g, %.15g, %.15g, %.15g, %.15g, %.15g, %.15g, %.15g, %.15g, %.15g, %.15g, %.15g, %.15g, %.15g, %.15g, %.15g, %.15g, %.15g, %.15g, %.15g, %.15g, %.15g, %.15g, %.15g, %.15g, %.15g, %.15g, %.15g\n", \
        new_state.t, vehicle->current_mass, \
        new_state.x, new_state.y, new_state.z, \
        new_state.vx, new_state.vy, new_state.vz, \
        new_state.ax_grav, new_state.ay_grav, new_state.az_grav, \
        new_state.ax_drag, new_state.ay_drag, new_state.az_drag, \
        a_command_total, a_lift_total, \
        new_state.ax_thrust, new_state.ay_thrust, new_state.az_thrust, \
        new_state.ax_total, new_state.ay_total, new_state.az_total, \
        est_state.x, est_state.y, est_state.z, \
        est_state.vx, est_state.vy, est_state.vz, \
        est_state.ax_total, est_state.ay_total, est_state.az_total, \
        true_atm_cond.density, \
        true_atm_cond.meridional_wind, true_atm_cond.zonal_wind, true_atm_cond.vertical_wind, \
        est_atm_cond.density, \
        est_state.ax_drag, est_state.ay_drag, est_state.az_drag);
}

state fly(runparams *run_params, state *initial_state, vehicle *vehicle){
    /*
    Function that simulates the flight of a vehicle, updating the state of the vehicle at each time step
    
    INPUTS:
    ----------
        run_params: runparams *
            pointer to the run parameters struct
        initial_state: state *
            pointer to the initial state of the vehicle
        vehicle: vehicle *
            pointer to the vehicle struct

    OUTPUTS:
    ----------
        final_state: state
            final state of the vehicle (impact point)
    */

    // Initialize the variables and structures
    int max_steps = 25000000;

    grav true_grav = init_grav(run_params);
    grav est_grav = init_grav(run_params);

    atm_model exp_atm_model = init_exp_atm(run_params);

    double a_command_total = 0;
    double a_lift_total = 0;

    // Initialize either a randomly chosen EarthGRAM profile or the average EarthGRAM profile
    eg16_profile atm_profile;
    if (run_params->atm_model == 2) {
        // Generate a random integer between 0 and 100
        int atm_profile_num = (int)ran_flat(0, 100); 
        
        atm_profile = parse_atm(run_params->atm_profile_path, atm_profile_num);
    }
    else if (run_params->atm_model == 3) {
        atm_profile = parse_atm(run_params->mean_atm_profile_path, -1);
    }

    state old_true_state = *initial_state;
    state new_true_state = *initial_state;

    state old_est_state = init_est_state(run_params);
    state new_est_state = init_est_state(run_params);
    state old_des_state = init_est_state(run_params);
    state new_des_state = init_est_state(run_params);

    int traj_output = run_params->traj_output;
    double time_step;
    // Initialize the IMU
    imu imu = imu_init(run_params, initial_state);

    // Initialize the GNSS
    gnss gnss = gnss_init(run_params);

    atm_cond true_atm_cond;
    atm_cond est_atm_cond;

    // Create a .txt file to store the trajectory data
    FILE *traj_file;
    if (traj_output == 1){
        traj_file = fopen(run_params->trajectory_path, "a");
        if (ftell(traj_file) == 0) {
            fprintf(traj_file, "t, current_mass, x, y, z, vx, vy, vz, ax_grav, ay_grav, az_grav, ax_drag, ay_drag, az_drag, a_command, a_lift, ax_thrust, ay_thrust, az_thrust, ax_total, ay_total, az_total, est_x, est_y, est_z, est_vx, est_vy, est_vz, est_ax_total, est_ay_total, est_az_total, density, meridional_wind, zonal_wind, vertical_wind, est_ax_drag, est_ay_drag, est_az_drag\n");
        }
        // Write the initial state to the trajectory file
        append_traj_data(traj_file, old_true_state, old_est_state, vehicle, a_command_total, a_lift_total, true_atm_cond, est_atm_cond);
    }

    // Variables for step function anomaly (only used for run_type = 1)
    double step_timer = 0; // time since step function was activated

    // Begin the integration loop
    for (int i = 0; i < max_steps; i++){
        int during_boost_phase = old_true_state.t < vehicle->booster.total_burn_time;
        // Get the atmospheric conditions
        double old_altitude = get_altitude(old_true_state.x, old_true_state.y, old_true_state.z);
        
        true_atm_cond = get_atm_cond(old_altitude, &exp_atm_model, run_params, &atm_profile);
        est_atm_cond = get_exp_atm_cond(old_altitude, &exp_atm_model);
        // if during boost or outside atmosphere, dt = main time step, else dt = reentry time step
        if (during_boost_phase || old_altitude > 1e5){
            time_step = run_params->time_step_main;
        }
        else{
            time_step = run_params->time_step_reentry;
        }
        // Update the thrust of the vehicle
        update_thrust(vehicle, &new_true_state);
        update_thrust(vehicle, &new_est_state);
        if (during_boost_phase) {
            update_thrust(vehicle, &new_des_state);
        }
        // Update the gravity acceleration components
        update_gravity(&true_grav, &new_true_state);
        update_gravity(&est_grav, &new_est_state);
        if (during_boost_phase) {
            update_gravity(&true_grav, &new_des_state);
        }

        // Aerodynamic drag and lift calculations
        // Branch 0: No aerodynamic forces (outside atmosphere)
        if (old_altitude > 1e5){
            // No aerodynamic forces
            new_true_state.ax_drag = 0;
            new_true_state.ay_drag = 0;
            new_true_state.az_drag = 0;
            new_true_state.ax_lift = 0;
            new_true_state.ay_lift = 0;
            new_true_state.az_lift = 0;

            new_est_state.ax_drag = 0;
            new_est_state.ay_drag = 0;
            new_est_state.az_drag = 0;
            new_est_state.ax_lift = 0;
            new_est_state.ay_lift = 0;
            new_est_state.az_lift = 0;

            new_des_state.ax_drag = 0;
            new_des_state.ay_drag = 0;
            new_des_state.az_drag = 0;
            new_des_state.ax_lift = 0;
            new_des_state.ay_lift = 0;
            new_des_state.az_lift = 0;

        }
        // Branch 1: Boost phase
        if (during_boost_phase){
            // Calculate the aerodynamic drag
            boost_drag(run_params, vehicle, &true_atm_cond, &new_true_state);
            boost_drag(run_params, vehicle, &est_atm_cond, &new_est_state);
            boost_drag(run_params, vehicle, &est_atm_cond, &new_des_state);

        }
        // Branch 2: Reentry phase, with lift and maneuverability
        else if (run_params->rv_maneuv == 1){
            // Calculate the aerodynamic drag and lift
            
            // Calculate the acceleration command
            cart_vector a_command = prop_nav(run_params, &new_est_state);

            // Calculate the aerodynamic drag and lift
            reentry_lift_drag(run_params, &new_true_state, &a_command, &true_atm_cond, vehicle, time_step, &step_timer);
            reentry_lift_drag(run_params, &new_est_state, &a_command, &est_atm_cond, vehicle, time_step, &step_timer);
            
            a_command_total = sqrt(pow(a_command.x, 2) + pow(a_command.y, 2) + pow(a_command.z, 2));
            a_lift_total = sqrt(pow(new_true_state.ax_lift, 2) + pow(new_true_state.ay_lift, 2) + pow(new_true_state.az_lift, 2));

        }
        // Branch 3: Reentry phase, drag only
        else if (run_params->rv_maneuv == 0){
            // Calculate the aerodynamic drag only
            reentry_drag(run_params, vehicle, &true_atm_cond, &new_true_state);
            reentry_drag(run_params, vehicle, &est_atm_cond, &new_est_state);
        }

        // Calculate the total acceleration components
        new_true_state.ax_total = new_true_state.ax_grav + new_true_state.ax_drag + new_true_state.ax_lift + new_true_state.ax_thrust;
        new_true_state.ay_total = new_true_state.ay_grav + new_true_state.ay_drag + new_true_state.ay_lift + new_true_state.ay_thrust;
        new_true_state.az_total = new_true_state.az_grav + new_true_state.az_drag + new_true_state.az_lift + new_true_state.az_thrust;
        new_est_state.ax_total = new_est_state.ax_grav + new_est_state.ax_drag + new_est_state.ax_lift + new_est_state.ax_thrust;
        new_est_state.ay_total = new_est_state.ay_grav + new_est_state.ay_drag + new_est_state.ay_lift + new_est_state.ay_thrust;
        new_est_state.az_total = new_est_state.az_grav + new_est_state.az_drag + new_est_state.az_lift + new_est_state.az_thrust;
        
        if (during_boost_phase) {
            new_des_state.ax_total = new_des_state.ax_grav + new_des_state.ax_drag + new_des_state.ax_lift + new_des_state.ax_thrust;
            new_des_state.ay_total = new_des_state.ay_grav + new_des_state.ay_drag + new_des_state.ay_lift + new_des_state.ay_thrust;
            new_des_state.az_total = new_des_state.az_grav + new_des_state.az_drag + new_des_state.az_lift + new_des_state.az_thrust;
        }

        double a_drag = sqrt(new_true_state.ax_drag*new_true_state.ax_drag + new_true_state.ay_drag*new_true_state.ay_drag + new_true_state.az_drag*new_true_state.az_drag);
        if (run_params->ins_nav == 1){
            // INS Measurement
            imu_measurement(&imu, &new_true_state, &new_est_state, vehicle);

            if (run_params->rv_maneuv == 0 ){ 
                update_imu(&imu, time_step);
            }
            else if (a_drag > 1e-3 || during_boost_phase){
                update_imu(&imu, time_step);
            }
        }

        if (run_params->gnss_nav == 1){
            // GNSS Measurement
            gnss_measurement(&gnss, &new_true_state, &new_est_state);
        }

        // Add conditional for t > 0 to account for mock booster with 0s for burn time
        if  (new_true_state.t > 0 && new_true_state.t == (vehicle->booster.total_burn_time) && run_params->run_type == 0){
            // Perform a perfect maneuver if before burnout
            // This accounts for the fact that we do not consider maneuverability errors during the boost phase, i.e. atmospheric errors during the boost phase are not considered
            new_true_state = perfect_maneuv(&new_true_state, &new_est_state, &new_des_state);
        }
    
        // Perform a Runge-Kutta step
        rk4step(&new_true_state, time_step);
        rk4step(&new_est_state, time_step);
        if (during_boost_phase) {
            rk4step(&new_des_state, time_step);
        }
        // Update the mass of the vehicle
        update_mass(vehicle, new_true_state.t);

        // Check if the vehicle has impacted the Earth
        double new_altitude = get_altitude(new_true_state.x, new_true_state.y, new_true_state.z);

        if (new_altitude < 0){
            state true_final_state = impact_linterp(&old_true_state, &new_true_state);
            state est_final_state = impact_linterp(&old_est_state, &new_est_state);
            state des_final_state = impact_linterp(&old_des_state, &new_des_state);
            // Add coriolis effect based on the latitude and the impact time error
            double lat = ran_flat(-M_PI/2, M_PI/2);
            double lon = ran_flat(-M_PI, M_PI);
            double time_error = true_final_state.t - est_final_state.t;
            double rot_speed = 464 * cos(lat);
            double coriolis = rot_speed * time_error;

            // based on the coriolis effect, update the final state x and y
            // This might seem like a bug, but I promise it's just clever
            // This replicates flying in a random direction, not just along the equator
            true_final_state.x = true_final_state.x - coriolis * sin(lon)*cos(lat);
            true_final_state.y = true_final_state.y + coriolis * cos(lon)*cos(lat);
            true_final_state.z = true_final_state.z + coriolis * sin(lat);

            if (run_params->rv_maneuv == 2){
                // If perfect rv maneuver, update the final position to isolate 
                // errors due to the guidance system. All errors in the guidance
                // system are from the difference of the true final state and the
                // estimated state. These errors are shifted to be over the aimpoint
                // for ease of plotting and analysis.
                true_final_state.x = run_params->x_aim + (true_final_state.x - est_final_state.x);
                true_final_state.y = run_params->y_aim + (true_final_state.y - est_final_state.y);
                true_final_state.z = run_params->z_aim + (true_final_state.z - est_final_state.z);
            }
            if (traj_output == 1){
                // Write the final state to the trajectory file
                append_traj_data(traj_file, true_final_state, est_final_state, vehicle, a_command_total, a_lift_total, true_atm_cond, est_atm_cond);
                fclose(traj_file);
            }

            return true_final_state;
        }

        // output the trajectory data
        if (traj_output == 1){
            append_traj_data(traj_file, new_true_state, new_est_state, vehicle, a_command_total, a_lift_total, true_atm_cond, est_atm_cond);
        }

        // Update the old state
        old_true_state = new_true_state;
        old_est_state = new_est_state;
        if (during_boost_phase) {
            old_des_state = new_des_state;
        }
    }
    
    printf("Warning: Maximum number of steps reached with no impact\n");

    // Close the trajectory file
    if (traj_output == 1){
        fclose(traj_file);
    }

    return new_true_state;
}

cart_vector update_aimpoint(runparams run_params){
    /*
    Updates the aimpoint based on the thrust angle and other run parameters

    INPUTS:
    ----------
        run_params: runparams
            run parameters struct
    OUTPUTS:
    ----------
        cart_vector: aimpoint
            Cartesian vector to the updated aimpoint
    */

    cart_vector aimpoint;
    
    // If reentry only run, return the origin/launchpoint
    if (run_params.run_type == 1){
        aimpoint.x = 6371e3;
        aimpoint.y = 0;
        aimpoint.z = 0;
        return aimpoint;
    }

    runparams run_params_temp = sanitize_runparams_for_aimpoint(run_params);
    
    // Initialize the vehicle 
    vehicle vehicle = init_vehicle(run_params_temp.booster_type, run_params_temp.rv_type);

    state initial_state = init_true_state(&run_params_temp);
    initial_state.theta_long = run_params.theta_long;

    // Call the fly function to get the final state
    state final_state = fly(&run_params_temp, &initial_state, &vehicle);

    // Update the aimpoint based on the final state
    aimpoint.x = final_state.x;
    aimpoint.y = final_state.y;
    aimpoint.z = final_state.z;

    return aimpoint;
}

double enu_to_long_lat() {
    /*
    Convert the ENU coordinates (up, east, north) to thrust angles (theta_lat, theta_long)
    at the origin. 
    */
    // ENU --> global xyz @ 0, 0 --> lat, long
    double enu_vector[3] = {global_run_params->up, global_run_params->east, global_run_params->north};
    double global_xyz[3];
    double launch[3] = {6371e3, 0, 0}; // Set coordinate system origin to the launch point
    sphervec_to_cartvec(enu_vector, global_xyz, launch);
    double x = global_xyz[0];
    double y = global_xyz[1];
    double z = global_xyz[2];
    global_run_params->theta_long = atan2(y, x);
    global_run_params->theta_lat = atan(z / sqrt(x*x + y*y));
}

double aimpoint_error(cart_vector aimpoint){
    /*
    Calculate the distance between the current aimpoint and the goal aimpoint.

    INPUTS:
    ----------
        aimpoint: cart_vector
            Cartesian vector to the aimpoint
    OUTPUTS:
    ----------
        error: double
            distance between the current aimpoint and the goal aimpoint
    */
    double current_x = aimpoint.x;
    double current_y = aimpoint.y;
    double current_z = aimpoint.z;

    double goal_x = global_run_params->x_aim;
    double goal_y = global_run_params->y_aim;
    double goal_z = global_run_params->z_aim;

    double error = sqrt((goal_x - current_x)*(goal_x - current_x) + \
    (goal_y - current_y)*(goal_y - current_y) + \
    (goal_z - current_z)*(goal_z - current_z));
    return error;
}

float aimpoint_error_wrapper(float up){
    /*
    Wrapper so the optimizer can call a function that only takes the single float
    argument that is the optimizer's decision variable. 

    INPUTS:
    ----------
        up: float
            The up component of the ENU coordinate system
    OUTPUTS:
    ----------
        error: float
            The error between the current aimpoint and the goal aimpoint (m)
    */
    global_run_params->up = up;
    enu_to_long_lat();
    cart_vector aimpoint = update_aimpoint(*global_run_params);
    return (float)aimpoint_error(aimpoint);
}

void optimize_thrust_angles(double aim_lat, double aim_lon, double launch_lat, double launch_lon) {
    /*
    Calculate the bearing, optimize for the up component of the ENU thrust, and 
    transform ENU thrust vector to the thrust angles (theta_lat, theta_long) at
    the origin.

    INPUTS:
    ----------
        aim_lat: double
            latitude of the aimpoint, in radians
        aim_lon: double
            longitude of the aimpoint, in radians
        launch_lat: double
            latitude of the launch point, in radians
        launch_lon: double
            longitude of the launch point, in radians
        p: float *
            pointer to the array to store the bearing components
    */
    double lon_diff = aim_lon - launch_lon;

    // East component
    global_run_params->east = sin(lon_diff) * cos(aim_lat);

    // North component
    global_run_params->north = cos(launch_lat) * sin(aim_lat) - sin(launch_lat) * cos(aim_lat) * cos(lon_diff);

    float ax = 0.5f, bx = 1.5f, cx;
    float fa, fb, fc;
    mnbrak(&ax, &bx, &cx, &fa, &fb, &fc, aimpoint_error_wrapper);

    float tol = 1e-6f;
    float xmin, fmin;
    fmin = brent(ax, bx, cx, aimpoint_error_wrapper, tol, &xmin);
    if (fmin > 10) {
        printf("Warning: Aimpoint error is high: %fm > 10m\n", fmin);
    }
}

int get_thrust_angle(runparams *run_params){
    /*
    Find the thrust angles (theta_lat, theta_long) based on the latitude and longitude
    of the aimpoint. Update the run_params object with the new angles.

    INPUTS:
    ----------
        aim_lat: double
            latitude of the aimpoint, in radians
        aim_lon: double
            longitude of the aimpoint, in radians
        run_params: runparams*
            pointer to the run parameters struct
    OUTPUTS:
    ----------
        1 if successful, 0 otherwise
    */
    double earth_radius = 6371e3; 

    double aim_cart_coords[3] = {run_params->x_aim, run_params->y_aim, run_params->z_aim};
    double launch_cart_coords[3] = {run_params->x_launch, run_params->y_launch, run_params->z_launch};
    double aim_spher_coords[3];
    double launch_spher_coords[3];
    cartcoords_to_sphercoords(aim_cart_coords, aim_spher_coords);
    cartcoords_to_sphercoords(launch_cart_coords, launch_spher_coords);

    // Use a coordinate system where the launch point is always the origin.
    // This requires transforming the user-provided aimpoint to an equivalent one
    // in the new coordinate system.
    double bearing = calc_bearing(launch_spher_coords[2], launch_spher_coords[1], 
                            aim_spher_coords[2], aim_spher_coords[1]);
    double distance = haversine_distance(launch_spher_coords[2], launch_spher_coords[1], 
                            aim_spher_coords[2], aim_spher_coords[1]);
    double origin[2] = {0.0, 0.0};
    double new_aim_lat_long[2];
    get_location(bearing, distance, origin, new_aim_lat_long);

    double new_aim_spher_coords[3] = {earth_radius, new_aim_lat_long[1], new_aim_lat_long[0]};
    double new_aim_cart_coords[3];
    sphercoords_to_cartcoords(new_aim_spher_coords, new_aim_cart_coords);
    run_params->x_aim = new_aim_cart_coords[0];
    run_params->y_aim = new_aim_cart_coords[1];
    run_params->z_aim = new_aim_cart_coords[2];

    run_params->x_launch = 6371e3; // Set the launch point to the origin
    run_params->y_launch = 0.0;
    run_params->z_launch = 0.0;

    // Check if the aimpoint is within the range of the vehicle
    vehicle vehicle = init_vehicle(run_params->booster_type, run_params->rv_type);
    if (distance > vehicle.range) {
        printf("Error: Aimpoint is out of range of the vehicle.\n");
        printf("Distance: %f, Range: %f\n", distance, vehicle.range);
        return 0;
    }

    runparams rp = sanitize_runparams_for_aimpoint(*run_params);
    global_run_params = &rp;

    optimize_thrust_angles(new_aim_lat_long[0], new_aim_lat_long[1], 0.0, 0.0);

    run_params->east = global_run_params->east;
    run_params->north = global_run_params->north;
    run_params->up = global_run_params->up;

    run_params->theta_lat = global_run_params->theta_lat;
    run_params->theta_long = global_run_params->theta_long;
    global_run_params = NULL;
    return 1;
}

impact_data mc_run(runparams run_params){
    /*
    Function that runs a Monte Carlo simulation of the vehicle flight
    
    INPUTS:
    ----------
        run_params: runparams
            run parameters struct
    */

   impact_data impact_data;
    // Print the run parameters to the console
    // print_config(&run_params);

    // Sets theta lat and theta long based on the user-provided launch and aimpoints
    if (!get_thrust_angle(&run_params)) {
        return impact_data;
    }
    printf("Thrust angles (radians): theta_lat = %f, theta_long = %f\n", run_params.theta_lat, run_params.theta_long);

    // Initialize the variables
    int num_runs = run_params.num_runs;
    if (num_runs > MAX_RUNS){
        printf("Error: Number of runs exceeds the maximum limit. Increase MAX_RUNS in src/include/trajectory.h and recompile. \n");
        printf("num_runs: %d, MAX_RUNS: %d\n", num_runs, MAX_RUNS);
        return impact_data;
    }
    
    // Create a .txt file to store the impact data
    FILE *impact_file;
    if (run_params.impact_output == 1){
        impact_file = fopen(run_params.impact_data_path, "w");
        fprintf(impact_file, "t, x, y, z, vx, vy, vz\n");
    }
    
    // Run the Monte Carlo simulation
    // Write the trajectory data to file for the first run if traj_output is 2
    int original_traj_output = run_params.traj_output;
    if (run_params.traj_output == 2){
        run_params.traj_output = 1;
    }
    for (int i = 0; i < num_runs; i++){

        vehicle vehicle;
        if (run_params.run_type == 0){
            vehicle = init_vehicle(run_params.booster_type, run_params.rv_type);
        }
        else if (run_params.run_type == 1){
            vehicle = init_reentry_only();
        }
        else{
            printf("Error: Invalid run type\n");
            exit(1);
        }
        

        state initial_true_state = init_true_state(&run_params);
        
        impact_data.impact_states[i] = fly(&run_params, &initial_true_state, &vehicle);
        // Reset flag for writing trajectory data to file. Ensures only the first
        // run writes to the file if the original trajectory output flag is 2.
        run_params.traj_output = original_traj_output;

        #ifdef FROM_PYTHON
        if ((i + 1) % 10 == 0) {
            update_loading_bar(10, num_runs);
        }
        if (i == num_runs - 1) {
            update_loading_bar(num_runs % 10, num_runs);
        }
        #endif
    }

    // Output the impact data
    if (run_params.impact_output == 1){
        output_impact(impact_file, &impact_data, num_runs);
    }

    return impact_data;

}

#endif