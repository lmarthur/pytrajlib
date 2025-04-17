#ifndef UTILS_H
#define UTILS_H

#include <math.h>
#include <stdio.h>

typedef struct runparams{
    char *run_name; // name of the run
    int run_type; // 0 for full trajectory, 1 for reentry only
    char *output_path; // path to the output directory
    char *impact_data_path; // path to the impact data file
    char *trajectory_path; // path to the trajectory data file
    char *atm_profile_path; // path to the atmospheric profile file
    int num_runs; // number of Monte Carlo runs
    double time_step_main; // time step in seconds during boost and outside the atmosphere
    double time_step_reentry; // time step in seconds during reentry
    int traj_output; // flag to output trajectory data
    int impact_output; // flag to output impact data
    double x_aim; // target x-coordinate in meters
    double y_aim; // target y-coordinate in meters
    double z_aim; // target z-coordinate in meters
    double theta_long; // thrust angle in the longitudinal direction in radians
    double theta_lat; // thrust angle in the latitudinal direction in radians

    int grav_error; // flag to include gravitational perturbations
    int atm_model; // flag to select the atmospheric model
    int atm_error; // flag to include atmospheric perturbations
    int gnss_nav; // flag to include GNSS navigation
    int ins_nav; // flag to include INS navigation
    int rv_maneuv; // flag to include guidance during the reentry phase
    double reentry_vel; // reentry velocity in meters per second
    double deflection_time; // time to make full flap deflection in seconds, used for maneuverability

    int rv_type; // reentry vehicle type (0: ballistic, 1: maneuverable)

    double initial_x_error; // initial x-error in meters
    double initial_pos_error; // initial position error in meters
    double initial_vel_error; // initial velocity error in meters per second
    double initial_angle_error; // initial angle error in radians
    double acc_scale_stability; // accelerometer scale stability in ppm
    double gyro_bias_stability; // gyro bias stability in rad/s
    double gyro_noise; // gyro noise in rad/s/sqrt(s)
    double gnss_noise; // GNSS error in meters
    double cl_pert; // Coefficient of lift perturbation 
    double step_acc_mag; // Step acceleration perturbation magnitude
    double step_acc_hgt; // Step acceleration perturbation height (altitude) in meters
    double step_acc_dur; // Step acceleration perturbation duration in seconds

} runparams;

typedef struct cart_vector{
    double x;
    double y;
    double z;
} cart_vector;

double get_altitude(double x, double y, double z){
    /*
    Calculates the altitude of a point above the Earth's surface

    INPUTS:
    ----------
        x: double
            x-coordinate of the point
        y: double
            y-coordinate of the point
        z: double
            z-coordinate of the point
    OUTPUTS:
    ----------
        altitude: double
            altitude of the point above the Earth's surface
    */

    return sqrt(x*x + y*y + z*z) - 6371e3;
}

void cartcoords_to_sphercoords(double *cart_coords, double *spher_coords){
    /*
    Converts Cartesian coordinates to spherical coordinates

    INPUTS:
    ----------
        cart_coords: double *
            pointer to Cartesian coordinates [x, y, z]
        spher_coords: double *
            pointer to spherical coordinates [r, long, lat]
    */

    // Calculate the radial coordinate
    spher_coords[0] = sqrt(cart_coords[0]*cart_coords[0] + cart_coords[1]*cart_coords[1] + cart_coords[2]*cart_coords[2]);

    // Calculate the longitudinal coordinate
    spher_coords[1] = atan2(cart_coords[1], cart_coords[0]);

    // Calculate the latitudinal coordinate
    spher_coords[2] = atan(cart_coords[2] / sqrt(cart_coords[0]*cart_coords[0] + cart_coords[1]*cart_coords[1]));
}

void sphercoords_to_cartcoords(double *spher_coords, double *cart_coords){
    /*
    Converts spherical coordinates to Cartesian coordinates

    INPUTS:
    ----------
        spher_coords: double *
            pointer to spherical coordinates [r, long, lat]
        cart_coords: double *
            pointer to Cartesian coordinates [x, y, z]
    */

    // Calculate the x-coordinate
    cart_coords[0] = spher_coords[0] * cos(spher_coords[1]) * cos(spher_coords[2]);

    // Calculate the y-coordinate
    cart_coords[1] = spher_coords[0] * sin(spher_coords[1]) * cos(spher_coords[2]);

    // Calculate the z-coordinate
    cart_coords[2] = spher_coords[0] * sin(spher_coords[2]);

}

void sphervec_to_cartvec(double *sphervec, double *cartvec, double *spher_coords){
    /*
    Converts a spherical vector to a Cartesian vector at a given set of spherical coordinates

    INPUTS:
    ----------
        sphervec: double *
            pointer to spherical vector [r, long, lat]
        cartvec: double *
            pointer to Cartesian vector [x, y, z]
        spher_coords: double *
            pointer to spherical coordinates [r, long, lat]
    */
    cartvec[0] = -sphervec[1]*sin(spher_coords[1]) - sphervec[2]*sin(spher_coords[2])*cos(spher_coords[1]) + sphervec[0]*cos(spher_coords[1]) * cos(spher_coords[2]);
    // Get the y-component of the spherical vector
    cartvec[1] = sphervec[1]*cos(spher_coords[1]) - sphervec[2]*sin(spher_coords[2])*sin(spher_coords[1]) + sphervec[0]*sin(spher_coords[1])*cos(spher_coords[2]);

    // Get the z-component of the spherical vector
    cartvec[2] = sphervec[2]*cos(spher_coords[2]) + sphervec[0]*sin(spher_coords[2]);
    
}

void print_config(runparams *run_params){
    /*
    Prints the run parameters to the console at runtime

    INPUTS:
    ----------
        run_params: runparams *
            pointer to the run parameters struct
    */
    printf("Run name: %s\n", run_params->run_name);
    printf("Run type: %d\n", run_params->run_type);
    printf("Output path: %s\n", run_params->output_path);
    printf("Impact data path: %s\n", run_params->impact_data_path);
    printf("Trajectory path: %s\n", run_params->trajectory_path);
    // TODO add atmospheric profile path
    printf("Number of Monte Carlo runs: %d\n", run_params->num_runs);
    printf("Time step: %f\n", run_params->time_step_main);
    printf("Reentry time step: %f\n", run_params->time_step_reentry);
    printf("Trajectory output: %d\n", run_params->traj_output);
    printf("Impact output: %d\n", run_params->impact_output);
    printf("Target x-coordinate: %f\n", run_params->x_aim);
    printf("Target y-coordinate: %f\n", run_params->y_aim);
    printf("Target z-coordinate: %f\n", run_params->z_aim);
    printf("Longitudinal thrust angle: %f\n", run_params->theta_long);
    printf("Latitudinal thrust angle: %f\n", run_params->theta_lat);

    printf("Gravitational perturbations: %d\n", run_params->grav_error);
    printf("Atmospheric model: %d\n", run_params->atm_model);
    printf("Atmospheric perturbations: %d\n", run_params->atm_error);
    printf("GNSS navigation: %d\n", run_params->gnss_nav);
    printf("INS navigation: %d\n", run_params->ins_nav);
    printf("Reentry phase guidance: %d\n", run_params->rv_maneuv);
    printf("Reentry velocity: %f\n", run_params->reentry_vel);

    printf("Reentry vehicle type: %d\n", run_params->rv_type);

    printf("Initial x-error: %f\n", run_params->initial_x_error);
    printf("Initial position error: %f\n", run_params->initial_pos_error);
    printf("Initial velocity error: %f\n", run_params->initial_vel_error);
    printf("Initial angle error: %f\n", run_params->initial_angle_error);
    printf("Accelerometer scale stability: %f\n", run_params->acc_scale_stability);
    printf("Gyro bias stability: %f\n", run_params->gyro_bias_stability);
    printf("Gyro noise: %f\n", run_params->gyro_noise);
    printf("GNSS noise: %f\n", run_params->gnss_noise);
    printf("Coefficient of lift perturbation: %f\n", run_params->cl_pert);
    printf("Step acceleration perturbation magnitude: %f\n", run_params->step_acc_mag);
    printf("Step acceleration perturbation height: %f\n", run_params->step_acc_hgt);
    printf("Step acceleration perturbation duration: %f\n", run_params->step_acc_dur);

}

double linterp(double x, double xs[], double ys[], int n){
    /*
    Linear interpolation function

    INPUTS:
    ----------
        x: double
            value to interpolate
        xs: double *
            pointer to the x-values
        ys: double *
            pointer to the y-values
        n: int
            number of data points
    OUTPUTS:
    ----------
        y: double
            interpolated value
    */

    // Initialize the output value
    double y = 0;

    // Find the two points to interpolate between
    int i = 0;
    while (x > xs[i]){
        i++;
    }

    if (i == 0){
        y = ys[0];
        return y;
    }

    // Perform the interpolation
    y = ys[i-1] + (ys[i] - ys[i-1]) * (x - xs[i-1]) / (xs[i] - xs[i-1]);

    return y;
}

double min(double a, double b){
    /*
    Returns the minimum of two values

    INPUTS:
    ----------
        a: double
            first value
        b: double
            second value
    OUTPUTS:
    ----------
        min: double
            minimum value
    */

    if (a < b){
        return a;
    }
    else{
        return b;
    }
}

double sign(double x){
    /*
    Returns the sign of a value

    INPUTS:
    ----------
        x: double
            value to get the sign of
    OUTPUTS:
    ----------
        sign: double
            sign of the value
    */

    if (x > 0){
        return 1;
    }
    else if (x < 0){
        return -1;
    }
    else{
        return 0;
    }
}

#endif