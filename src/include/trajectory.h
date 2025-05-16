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

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#include <nlopt.h>
EM_JS(void, num_run_counter, (), {
    console.log("Incrementing run counter");
  });

EM_JS(void, error_tracker, (double error), {
    console.log("Current error:", error);
  });
#endif

// Define a constant upper limit for the number of Monte Carlo runs
#define MAX_RUNS 1000

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
        // printf("Initializing full trajectory run\n");
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
        // printf("Initializing reentry only run\n");
        state.t = 0;
        state.x = 6371e3 + 500e3 + run_params->initial_x_error * ran_gaussian(1);
        state.y = run_params->initial_pos_error * ran_gaussian(1);
        state.z = run_params->initial_pos_error * ran_gaussian(1);

        state.vx = -run_params->reentry_vel + run_params->initial_vel_error * ran_gaussian(1);
        state.vy = run_params->initial_vel_error * ran_gaussian(1);
        state.vz = run_params->initial_vel_error * ran_gaussian(1);

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
        // printf("Initializing full trajectory run\n");
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
        // printf("Initializing reentry only run\n");
        state.t = 0;
        state.x = 6371e3 + 1000e3;
        state.y = 0;
        state.z = 0;

        state.vx = -run_params->reentry_vel;
        state.vy = 0;
        state.vz = 0;

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
    int max_steps = 1000000;

    grav true_grav = init_grav(run_params);
    grav est_grav = init_grav(run_params);
    est_grav.perturb_flag = 0;

    atm_model exp_atm_model = init_exp_atm(run_params);

    double a_command_total = 0;
    double a_lift_total = 0;

    int atm_profile_num;
    // Generate a random integer between 0 and 100
    atm_profile_num = (int)ran_flat(0, 100); 

    eg16_profile atm_profile = parse_atm(run_params->atm_profile_path, atm_profile_num);

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

    // Create a .txt file to store the trajectory data
    FILE *traj_file;
    if (traj_output == 1){
        traj_file = fopen(run_params->trajectory_path, "w");
        fprintf(traj_file, "t, current_mass, x, y, z, vx, vy, vz, ax_grav, ay_grav, az_grav, ax_drag, ay_drag, az_drag, a_command, a_lift, ax_thrust, ay_thrust, az_thrust, ax_total, ay_total, az_total, est_x, est_y, est_z, est_vx, est_vy, est_vz, est_ax_total, est_ay_total, est_az_total \n");
        // Write the initial state to the trajectory file
        fprintf(traj_file, "%g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g\n", old_true_state.t, vehicle->current_mass, old_true_state.x, old_true_state.y, old_true_state.z, old_true_state.vx, old_true_state.vy, old_true_state.vz, old_true_state.ax_grav, old_true_state.ay_grav, old_true_state.az_grav, old_true_state.ax_drag, old_true_state.ay_drag, old_true_state.az_drag, a_command_total, a_lift_total, old_true_state.ax_thrust, old_true_state.ay_thrust, old_true_state.az_thrust, old_true_state.ax_total, old_true_state.ay_total, old_true_state.az_total, old_est_state.x, old_est_state.y, old_est_state.z, old_est_state.vx, old_est_state.vy, old_est_state.vz, old_est_state.ax_total, old_est_state.ay_total, old_est_state.az_total);
    }

    // Variables for step function anomaly (only used for run_type = 1)
    double step_timer = 0; // time since step function was activated

    // Begin the integration loop
    for (int i = 0; i < max_steps; i++){
        // Get the atmospheric conditions
        double old_altitude = get_altitude(old_true_state.x, old_true_state.y, old_true_state.z);
        
        atm_cond true_atm_cond = get_atm_cond(old_altitude, &exp_atm_model, run_params, &atm_profile);
        // printf("true_atm_cond: %f, %f, %f\n", true_atm_cond.density, true_atm_cond.meridional_wind, true_atm_cond.zonal_wind);
        atm_cond est_atm_cond = get_exp_atm_cond(old_altitude, &exp_atm_model);
        // if during boost or outside atmosphere, dt = main time step, else dt = reentry time step
        if (old_true_state.t < vehicle->booster.total_burn_time || old_altitude > 1e6){
            time_step = run_params->time_step_main;
        }
        else{
            time_step = run_params->time_step_reentry;
        }
        // Update the thrust of the vehicle
        update_thrust(vehicle, &new_true_state);
        update_thrust(vehicle, &new_est_state);
        update_thrust(vehicle, &new_des_state);
        // Update the gravity acceleration components
        update_gravity(&true_grav, &new_true_state);
        update_gravity(&est_grav, &new_est_state);
        update_gravity(&true_grav, &new_des_state);

        // Update the drag acceleration components
        update_drag(run_params, vehicle, &true_atm_cond, &new_true_state, &step_timer);
        update_drag(run_params, vehicle, &est_atm_cond, &new_est_state, &step_timer);
        update_drag(run_params, vehicle, &est_atm_cond, &new_des_state, &step_timer);

        // If maneuverable RV, use proportional navigation during reentry
        if (run_params->rv_maneuv == 1 && old_true_state.t >= vehicle->booster.total_burn_time && get_altitude(new_true_state.x, new_true_state.y, new_true_state.z) < 1e6){
            // Get the acceleration command
            cart_vector a_command = prop_nav(run_params, &new_est_state);
            // printf("a_command: %f, %f, %f\n", a_command.x, a_command.y, a_command.z);
            // Update the lift acceleration components
            update_lift(run_params, &new_true_state, &a_command, &true_atm_cond, vehicle, time_step);
            // get the total acceleration command and the total lift acceleration
            a_command_total = sqrt(a_command.x*a_command.x + a_command.y*a_command.y + a_command.z*a_command.z);
            a_lift_total = sqrt(new_true_state.ax_lift*new_true_state.ax_lift + new_true_state.ay_lift*new_true_state.ay_lift + new_true_state.az_lift*new_true_state.az_lift);
            // printf("a_command_total: %f, a_lift_total: %f\n", a_command_total, a_lift_total);

            update_lift(run_params, &new_est_state, &a_command, &est_atm_cond, vehicle, time_step);

        }

        // Calculate the total acceleration components
        new_true_state.ax_total = new_true_state.ax_grav + new_true_state.ax_drag + new_true_state.ax_lift + new_true_state.ax_thrust;
        new_true_state.ay_total = new_true_state.ay_grav + new_true_state.ay_drag + new_true_state.ay_lift + new_true_state.ay_thrust;
        new_true_state.az_total = new_true_state.az_grav + new_true_state.az_drag + new_true_state.az_lift + new_true_state.az_thrust;
        new_est_state.ax_total = new_est_state.ax_grav + new_est_state.ax_drag + new_est_state.ax_lift + new_est_state.ax_thrust;
        new_est_state.ay_total = new_est_state.ay_grav + new_est_state.ay_drag + new_est_state.ay_lift + new_est_state.ay_thrust;
        new_est_state.az_total = new_est_state.az_grav + new_est_state.az_drag + new_est_state.az_lift + new_est_state.az_thrust;
        new_des_state.ax_total = new_des_state.ax_grav + new_des_state.ax_drag + new_des_state.ax_lift + new_des_state.ax_thrust;
        new_des_state.ay_total = new_des_state.ay_grav + new_des_state.ay_drag + new_des_state.ay_lift + new_des_state.ay_thrust;
        new_des_state.az_total = new_des_state.az_grav + new_des_state.az_drag + new_des_state.az_lift + new_des_state.az_thrust;

        double a_drag = sqrt(new_true_state.ax_drag*new_true_state.ax_drag + new_true_state.ay_drag*new_true_state.ay_drag + new_true_state.az_drag*new_true_state.az_drag);
        if (run_params->ins_nav == 1){
            // INS Measurement
            imu_measurement(&imu, &new_true_state, &new_est_state, vehicle);

            if (run_params->rv_maneuv == 0 ){ 
                update_imu(&imu, time_step);
            }
            else if (a_drag > 1e-3 || old_true_state.t < vehicle->booster.total_burn_time){
                update_imu(&imu, time_step);
            }
        }

        if (run_params->gnss_nav == 1){
            // GNSS Measurement
            gnss_measurement(&gnss, &new_true_state, &new_est_state);
        }

        if  (new_true_state.t == (vehicle->booster.total_burn_time) && run_params->run_type == 0){
            // Perform a perfect maneuver if before burnout

            new_true_state = perfect_maneuv(&new_true_state, &new_est_state, &new_des_state);
            imu.gyro_error_lat = 0;
            imu.gyro_error_long = 0;

        }
    
        // Perform a Runge-Kutta step
        rk4step(&new_true_state, time_step);
        rk4step(&new_est_state, time_step);
        rk4step(&new_des_state, time_step);
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
            // printf("Impact time error: %f\n", time_error);
            double coriolis = rot_speed * time_error;

            // based on the coriolis effect, update the final state x and y
            // This might seem like a bug, but I promise it's just clever
            // This replicates flying in a random direction, not just along the equator
            true_final_state.x = true_final_state.x - coriolis * sin(lon)*cos(lat);
            true_final_state.y = true_final_state.y + coriolis * cos(lon)*cos(lat);
            true_final_state.z = true_final_state.z + coriolis * sin(lat);
            if (run_params->rv_maneuv == 2){
                // If perfect rv maneuver, update the final position
                true_final_state.x = true_final_state.x - est_final_state.x;
                true_final_state.y = true_final_state.y - est_final_state.y;
                true_final_state.z = true_final_state.z - est_final_state.z;
            }
            if (traj_output == 1){
                // Write the final state to the trajectory file
                fprintf(traj_file, "%g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g\n", true_final_state.t, vehicle->current_mass, true_final_state.x, true_final_state.y, true_final_state.z, true_final_state.vx, true_final_state.vy, true_final_state.vz, true_final_state.ax_grav, true_final_state.ay_grav, true_final_state.az_grav, true_final_state.ax_drag, true_final_state.ay_drag, true_final_state.az_drag, a_command_total, a_lift_total, true_final_state.ax_thrust, true_final_state.ay_thrust, true_final_state.az_thrust, true_final_state.ax_total, true_final_state.ay_total, true_final_state.az_total, est_final_state.x, est_final_state.y, est_final_state.z, est_final_state.vx, est_final_state.vy, est_final_state.vz, est_final_state.ax_total, est_final_state.ay_total, est_final_state.az_total);
                fclose(traj_file);
            }

            return true_final_state;
        }

        // output the trajectory data
        if (traj_output == 1){
            fprintf(traj_file, "%g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g\n", new_true_state.t, vehicle->current_mass, new_true_state.x, new_true_state.y, new_true_state.z, new_true_state.vx, new_true_state.vy, new_true_state.vz, new_true_state.ax_grav, new_true_state.ay_grav, new_true_state.az_grav, new_true_state.ax_drag, new_true_state.ay_drag, new_true_state.az_drag, a_command_total, a_lift_total, new_true_state.ax_thrust, new_true_state.ay_thrust, new_true_state.az_thrust, new_true_state.ax_total, new_true_state.ay_total, new_true_state.az_total, new_est_state.x, new_est_state.y, new_est_state.z, new_est_state.vx, new_est_state.vy, new_est_state.vz, new_est_state.ax_total, new_est_state.ay_total, new_est_state.az_total);
        }

        // Update the old state
        old_true_state = new_true_state;
        old_est_state = new_est_state;
        old_des_state = new_des_state;
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
    vehicle vehicle;
    if (run_params_temp.rv_type == 0){
            vehicle = init_mmiii_ballistic();
    }
    else if (run_params_temp.rv_type == 1){
            vehicle = init_mmiii_swerve();
    }
    else{
            printf("Error: Invalid RV type\n");
            exit(1);
    }
    

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

impact_data mc_run(runparams run_params){
    /*
    Function that runs a Monte Carlo simulation of the vehicle flight
    
    INPUTS:
    ----------
        run_params: runparams
            run parameters struct
    */

    // Print the run parameters to the console
    // print_config(&run_params);

    // Initialize the variables
    int num_runs = run_params.num_runs;
    // printf("Simulating %d Monte Carlo runs...\n", num_runs);
    if (num_runs > MAX_RUNS){
        printf("Error: Number of runs exceeds the maximum limit. Increase MAX_RUNS in src/include/trajectory.h and recompile. \n");
        printf("num_runs: %d, MAX_RUNS: %d\n", num_runs, MAX_RUNS);
        exit(1);
    }
    
    impact_data impact_data;
    

    // Create a .txt file to store the impact data
    FILE *impact_file;
    if (run_params.impact_output == 1){
        impact_file = fopen(run_params.impact_data_path, "w");
        fprintf(impact_file, "t, x, y, z, vx, vy, vz\n");
    }
    
    // Run the Monte Carlo simulation
    // Write the trajectory data to file for the first run. If traj_output is 2, 
    // then do not write to file because this signifies a run from the web.
    int original_traj_output = run_params.traj_output;
    if (run_params.traj_output == 0){
        run_params.traj_output = 1;
    }
    for (int i = 0; i < num_runs; i++){

        vehicle vehicle;
        if (run_params.run_type == 0){
            if (run_params.rv_type == 0){
                vehicle = init_mmiii_ballistic();
            }
            else if (run_params.rv_type == 1){
                                vehicle = init_mmiii_swerve();
            }
            else{
                printf("Error: Invalid RV type\n");
                exit(1);
            }
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
        // run writes to the file if the original trajectory output flag is 0.
        run_params.traj_output = original_traj_output;

        // Allow time for browser to update.
        #ifdef __EMSCRIPTEN__
            num_run_counter();
            emscripten_sleep(0);
        #endif

    }

    // Output the impact data
    if (run_params.impact_output == 1){
        output_impact(impact_file, &impact_data, num_runs);
    }

    return impact_data;

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

double aimpoint_error_theta_wrapper(double *x){
    global_run_params->theta_lat = x[0];
    global_run_params->theta_long = x[1];
    cart_vector aimpoint = update_aimpoint(*global_run_params);
    double error = aimpoint_error(aimpoint);
    printf("Error: %f Guess: %f, %f\n", error, x[0], x[1]);
    return error;
}

float aimpoint_error_magnitude_wrapper(float magnitude){
    global_run_params->theta_lat = magnitude * global_run_params->theta_lat;
    global_run_params->theta_long = magnitude * global_run_params->theta_long;
    cart_vector aimpoint = update_aimpoint(*global_run_params);
    return (float)aimpoint_error(aimpoint);
}

double nlopt_objective(unsigned n, const double *x, double *grad, void *data) {
    double fx = aimpoint_error_theta_wrapper(x);

    // Calculate the gradient with a simple finite diff
    if (grad) {
        double h = 1e-6, xtmp[n];
        for (unsigned i = 0; i < n; ++i) {
            memcpy(xtmp, x, sizeof(xtmp));
            xtmp[i] += h;
            double fi = aimpoint_error_theta_wrapper(xtmp);
            // Initial gradients can be very large; scale the gradient so the
            // optimizer does not crash
            grad[i] = (fi - fx) / h / 1e6; 
        }
    }
    return fx;
}

void get_bearing(double aim_lat, double aim_lon, double launch_lat, double launch_lon) {
    /*
    Function that calculates the bearing from the launch point to the aim point

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

    // North component
    global_run_params->theta_lat = cos(launch_lat) * sin(aim_lat) - sin(launch_lat) * cos(aim_lat) * cos(lon_diff);
    // East component
    global_run_params->theta_long = sin(lon_diff) * cos(aim_lat);

    float ax = 0.5f, bx = 1.5f, cx;
    float fa, fb, fc;
    mnbrak(&ax, &bx, &cx, &fa, &fb, &fc, aimpoint_error_magnitude_wrapper);

    float tol = 1e-6f;
    float xmin, fmin;
    fmin = brent(ax, bx, cx, aimpoint_error_magnitude_wrapper, tol, &xmin);

    global_run_params->theta_lat = global_run_params->theta_lat * xmin;
    global_run_params->theta_long = global_run_params->theta_long * xmin;
}

double dummy_constraint(unsigned n, const double *x, double *grad, void *data) {
    return 1.0;  // Always positive, so always satisfied
}

#ifdef __EMSCRIPTEN__
void get_thrust_angle(double aim_lat, double aim_lon, runparams *run_params){
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
    */
    double earth_radius = 6371e3; 
    double spher_coords[3] = {earth_radius, aim_lon, aim_lat};
    double cart_coords[3];
    sphercoords_to_cartcoords(spher_coords, cart_coords);

    runparams rp = sanitize_runparams_for_aimpoint(*run_params);
    global_run_params = &rp;
    global_run_params->x_aim = cart_coords[0];
    global_run_params->y_aim = cart_coords[1];
    global_run_params->z_aim = cart_coords[2];

    printf("Optimizing...\n");
    get_bearing(aim_lat, aim_lon, 0, 0);

    // Take the remainder to ensure the initial guess is within the lower and upper bounds 
    run_params->theta_lat = remainder(global_run_params->theta_lat, 2 * M_PI);
    run_params->theta_long = remainder(global_run_params->theta_long, 2 * M_PI);
    printf("init guesses: %f, %f\n", run_params->theta_lat, run_params->theta_long);

    nlopt_opt opt = nlopt_create(NLOPT_LD_SLSQP, 2);

    double lb[2] = { -M_PI, -M_PI };
    nlopt_set_lower_bounds(opt, lb);

    double ub[2] = { M_PI, M_PI };
    nlopt_set_upper_bounds(opt, ub);

    nlopt_set_min_objective(opt, nlopt_objective, NULL);

    // At least one constraint seems to be needed
    nlopt_add_inequality_constraint(opt, dummy_constraint, NULL, 1e-8);
    
    nlopt_set_ftol_rel(opt, 1e-6);
    nlopt_set_maxeval(opt, 100);

    double x[2] = { run_params->theta_lat, run_params->theta_long };
    double minf;

    clock_t start, end;
    double cpu_time_used;
    start = clock();

    int ret = nlopt_optimize(opt, x, &minf);
    end = clock();
    cpu_time_used = ((double)(end - start)) / CLOCKS_PER_SEC;
    printf("nlopt_optimize took %f seconds\n", cpu_time_used);
    if (ret < 0) {
        printf("NLopt failed with code %d\n", ret);
        
    } else {
        printf("Found minimum at (%g, %g) with value %0.10g\n", x[0], x[1], minf);
    }
    run_params->theta_lat = x[0];
    run_params->theta_long = x[1];

    nlopt_destroy(opt);
}


char* mc_run_wrapper(char *run_name, int run_type, char *output_path, 
    char *impact_data_path, char *trajectory_path, char *atm_profile_path, 
    int num_runs, double time_step_main, double time_step_reentry, int traj_output, 
    int impact_output, double x_aim, double y_aim, double z_aim, double theta_long, 
    double theta_lat, int grav_error, int atm_model, int atm_error, int gnss_nav, 
    int ins_nav, int rv_maneuv, double reentry_vel, double deflection_time, 
    int rv_type, double initial_x_error, double initial_pos_error, 
    double initial_vel_error, double initial_angle_error, double acc_scale_stability, 
    double gyro_bias_stability, double gyro_noise, double gnss_noise, double cl_pert, 
    double step_acc_mag, double step_acc_hgt, double step_acc_dur, double aim_lat, double aim_lon){
    /*
    Return the impact data as a string. This is used for the web version of the 
    code because it is easier for javascript to handle strings than pointers or
    structs.

    Also computes the best thrust angles to achieve the provided aimpoint. 

    INPUT:
    ----------
        All parameters used in the runparams struct + the aimpoint latitude and 
        longitude (radians).
    
    OUTPUTS:
    ----------
        data_str: char*
            string containing the impact data. First row is aim_x, aim_y, aim_z
            Each subsequent row is t, x, y, z, vx, vy, vz.
    */

    runparams run_params;
    run_params.run_name = run_name;
    run_params.run_type = run_type;
    run_params.output_path = output_path;
    run_params.impact_data_path = impact_data_path;
    run_params.trajectory_path = trajectory_path;
    run_params.atm_profile_path = atm_profile_path;
    run_params.num_runs = num_runs;
    run_params.time_step_main = time_step_main;
    run_params.time_step_reentry = time_step_reentry;
    run_params.traj_output = traj_output;
    run_params.impact_output = impact_output;
    run_params.x_aim = x_aim;
    run_params.y_aim = y_aim;
    run_params.z_aim = z_aim;
    run_params.theta_long = theta_long;
    run_params.theta_lat = theta_lat;
    run_params.grav_error = grav_error;
    run_params.atm_model = atm_model;
    run_params.atm_error = atm_error;
    run_params.gnss_nav = gnss_nav;
    run_params.ins_nav = ins_nav;
    run_params.rv_maneuv = rv_maneuv;
    run_params.reentry_vel = reentry_vel;
    run_params.deflection_time = deflection_time;
    run_params.rv_type = rv_type;
    run_params.initial_x_error = initial_x_error;
    run_params.initial_pos_error = initial_pos_error;
    run_params.initial_vel_error = initial_vel_error;
    run_params.initial_angle_error = initial_angle_error;
    run_params.acc_scale_stability = acc_scale_stability;
    run_params.gyro_bias_stability = gyro_bias_stability;
    run_params.gyro_noise = gyro_noise;
    run_params.gnss_noise = gnss_noise;
    run_params.cl_pert = cl_pert;
    run_params.step_acc_mag = step_acc_mag;
    run_params.step_acc_hgt = step_acc_hgt;
    run_params.step_acc_dur = step_acc_dur;

    get_thrust_angle(aim_lat, aim_lon, &run_params);

    cart_vector aimpoint = update_aimpoint(run_params);
    printf("Aimpoint in c: %f, %f, %f\n", aimpoint.x, aimpoint.y, aimpoint.z);
    run_params.x_aim = aimpoint.x;
    run_params.y_aim = aimpoint.y;
    run_params.z_aim = aimpoint.z;

    impact_data data = mc_run(run_params);

    // 24 chars for each double, 2 chars for the comma and space, 1 char for the newline
    int size = ((24 + 2) * 7 + 1);
    // Add 1 to num_rus so the first row has the aimpoint
    char* data_str = (char*)malloc(size * (run_params.num_runs + 1) * sizeof(char));
    snprintf(data_str, size, "%f, %f, %f\n", run_params.x_aim, run_params.y_aim, run_params.z_aim);
    for (int i = 0; i < run_params.num_runs; i++){
        char* row_str;
        snprintf(row_str, size, "%f, %f, %f, %f, %f, %f, %f\n", data.impact_states[i].t, data.impact_states[i].x, data.impact_states[i].y, data.impact_states[i].z, data.impact_states[i].vx, data.impact_states[i].vy, data.impact_states[i].vz);
        strcat(data_str, row_str);
    }
    return data_str;
}

int test() {
    /*
    Simple test the code is working; useful to see if the web version has loaded
    properly.
    */
    printf("test\n");
    return 1;
}
#endif

#endif