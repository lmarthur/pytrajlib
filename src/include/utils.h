#ifndef UTILS_H
#define UTILS_H

#include <math.h>
#include <stdio.h>

#include "constants.h"
#include "math/linalg.h"
#include "models/vehicle.h"

typedef struct runparams {
  char *run_name;         // name of the run
  int run_type;           // 0 for full trajectory, 1 for reentry only
  char *output_path;      // path to the output directory
  char *trajectory_path;  // path to the trajectory data file
  char *atm_path;         // path to "atmprofiles.txt"
  char *mean_atm_path;    // path to "mean_atm.txt"
  int num_runs;           // number of Monte Carlo runs
  int num_runs_optimizer; // number of runs to use during trajectory
                          // optimization
  double
      time_step_lambert; // time step in seconds during Lambert guidance phase
  double time_step_midcourse; // time step in seconds during midcourse flight
  double time_step_atm; // time step in seconds inside atmosphere (reentry and
                        // first part of boost)
  int traj_output;      // flag to output trajectory data
  double range;         // downrange distance along equator in meters
  double x_aim;         // target x-coordinate in meters
  double y_aim;         // target y-coordinate in meters
  double z_aim;         // target z-coordinate in meters
  double theta_long;    // thrust angle in the longitudinal direction in radians
  double theta_lat;     // thrust angle in the latitudinal direction in radians
  int integrator; // flag for which numerical integration method to use (0 for
                  // modified Euler-Maruyama, 1 for SRA3)

  int grav_error;      // flag to include gravitational perturbations
  int include_drag;    // flag to include aerodynamic drag force
  int atm_model;       // flag to select the atmospheric model
  int gnss_nav;        // flag to include GNSS navigation
  int ins_nav;         // flag to include INS navigation
  int rv_maneuv;       // flag to include guidance during the reentry phase
  double reentry_vel;  // reentry velocity in meters per second
  int perfect_boost;   // 1 perfect boost, 0 realistic
  double t_des_final;  // desired flight time (optimized by code)
  double t_vert_boost; // Duration of vertical boost (optimized by code)

  int rv_type; // reentry vehicle type (0: ballistic, 1: maneuverable)
  double deflection_time; // time to make full flap deflection in seconds, used
                          // for maneuverability
  double actuator_force;  // actuator max force in kilonewtons, used for
                          // maneuverability
  double gearing_ratio;   // gearing ratio of the control surfaces, used for
                          // maneuverability
  double nav_gain;  // navigation gain for proportional navigation guidance
  double flap_gain; // Gain for approaching the commanded flap position when
                    // slower than max rate.

  double initial_x_error;     // initial x-error in meters
  double initial_pos_error;   // initial position error in meters
  double initial_vel_error;   // initial velocity error in meters per second
  double initial_angle_error; // initial angle error in radians
  double acc_scale_stability; // accelerometer scale stability in ppm
  double gyro_bias_stability; // gyro bias stability in rad/s
  double gyro_noise;          // gyro noise in rad/s/sqrt(s)
  double gnss_noise;          // GNSS error in meters
  double cl_pert;             // Coefficient of lift perturbation
  double step_acc_mag;        // Step acceleration perturbation magnitude
  double step_acc_hgt; // Step acceleration perturbation height (altitude) in
                       // meters
  double step_acc_dur; // Step acceleration perturbation duration in seconds

} runparams;

#include "models/state.h"

/**
 * Calculates the altitude of a point above Earth's surface.
 *
 * @param position Cartesian position vector.
 * @return Altitude above Earth's mean radius in meters.
 */
double get_altitude(cartvec position) {

  return norm(position) - EARTH_RADIUS_M;
}

/**
 * Converts Cartesian coordinates to spherical coordinates.
 *
 * @param cart_coords Pointer to Cartesian coordinates `[x, y, z]`.
 * @param spher_coords Output spherical coordinates `[r, long, lat]`.
 */
void cartcoords_to_sphercoords(double *cart_coords, double *spher_coords) {

  // Calculate the radial coordinate
  spher_coords[0] =
      sqrt(cart_coords[0] * cart_coords[0] + cart_coords[1] * cart_coords[1] +
           cart_coords[2] * cart_coords[2]);

  // Calculate the longitudinal coordinate
  spher_coords[1] = atan2(cart_coords[1], cart_coords[0]);

  // Calculate the latitudinal coordinate
  spher_coords[2] =
      atan(cart_coords[2] / sqrt(cart_coords[0] * cart_coords[0] +
                                 cart_coords[1] * cart_coords[1]));
}

/**
 * Converts spherical coordinates to Cartesian coordinates.
 *
 * @param spher_coords Pointer to spherical coordinates `[r, long, lat]`.
 * @param cart_coords Output Cartesian coordinates `[x, y, z]`.
 */
void sphercoords_to_cartcoords(double *spher_coords, double *cart_coords) {

  // Calculate the x-coordinate
  cart_coords[0] =
      spher_coords[0] * cos(spher_coords[1]) * cos(spher_coords[2]);

  // Calculate the y-coordinate
  cart_coords[1] =
      spher_coords[0] * sin(spher_coords[1]) * cos(spher_coords[2]);

  // Calculate the z-coordinate
  cart_coords[2] = spher_coords[0] * sin(spher_coords[2]);
}

/**
 * Converts a spherical vector to Cartesian components at given spherical
 * coordinates.
 *
 * @param sphervec Pointer to spherical vector components.
 * @param cartvec Output Cartesian vector components.
 * @param spher_coords Pointer to spherical coordinates `[r, long, lat]`.
 */
void sphervec_to_cartvec(double *sphervec, double *cartvec,
                         double *spher_coords) {
  cartvec[0] = -sphervec[1] * sin(spher_coords[1]) -
               sphervec[2] * sin(spher_coords[2]) * cos(spher_coords[1]) +
               sphervec[0] * cos(spher_coords[1]) * cos(spher_coords[2]);
  // Get the y-component of the spherical vector
  cartvec[1] = sphervec[1] * cos(spher_coords[1]) -
               sphervec[2] * sin(spher_coords[2]) * sin(spher_coords[1]) +
               sphervec[0] * sin(spher_coords[1]) * cos(spher_coords[2]);

  // Get the z-component of the spherical vector
  cartvec[2] =
      sphervec[2] * cos(spher_coords[2]) + sphervec[0] * sin(spher_coords[2]);
}

/**
 * Prints run parameters to the console.
 *
 * @param run_params Pointer to run parameters struct.
 */
void print_config(runparams *run_params) {
  printf("Run name: %s\n", run_params->run_name);
  printf("Run type: %d\n", run_params->run_type);
  printf("Output path: %s\n", run_params->output_path);
  printf("Trajectory path: %s\n", run_params->trajectory_path);
  printf("Number of Monte Carlo runs: %d\n", run_params->num_runs);
  printf("Time step: %f\n", run_params->time_step_midcourse);
  printf("Time step lambert: %f\n", run_params->time_step_lambert);
  printf("Time step midcourse: %f\n", run_params->time_step_midcourse);
  printf("Time step in atmosphere: %f\n", run_params->time_step_atm);
  printf("Trajectory output: %d\n", run_params->traj_output);
  printf("Target x-coordinate: %f\n", run_params->x_aim);
  printf("Target y-coordinate: %f\n", run_params->y_aim);
  printf("Target z-coordinate: %f\n", run_params->z_aim);
  printf("Longitudinal thrust angle: %f\n", run_params->theta_long);
  printf("Latitudinal thrust angle: %f\n", run_params->theta_lat);

  printf("Gravitational perturbations: %d\n", run_params->grav_error);
  printf("Include drag: %d\n", run_params->include_drag);
  printf("Atmospheric model: %d\n", run_params->atm_model);
  printf("GNSS navigation: %d\n", run_params->gnss_nav);
  printf("INS navigation: %d\n", run_params->ins_nav);
  printf("Reentry phase guidance: %d\n", run_params->rv_maneuv);
  printf("Reentry velocity: %f\n", run_params->reentry_vel);

  printf("Reentry vehicle type: %d\n", run_params->rv_type);

  printf("Initial x-error: %f\n", run_params->initial_x_error);
  printf("Initial position error: %f\n", run_params->initial_pos_error);
  printf("Initial velocity error: %f\n", run_params->initial_vel_error);
  printf("Initial angle error: %f\n", run_params->initial_angle_error);
  printf("Accelerometer scale stability: %f\n",
         run_params->acc_scale_stability);
  printf("Gyro bias stability: %f\n", run_params->gyro_bias_stability);
  printf("Gyro noise: %f\n", run_params->gyro_noise);
  printf("GNSS noise: %f\n", run_params->gnss_noise);
  printf("Coefficient of lift perturbation: %f\n", run_params->cl_pert);
  printf("Step acceleration perturbation magnitude: %f\n",
         run_params->step_acc_mag);
  printf("Step acceleration perturbation height: %f\n",
         run_params->step_acc_hgt);
  printf("Step acceleration perturbation duration: %f\n",
         run_params->step_acc_dur);
}

/**
 * Writes a trajectory state row to an output file.
 *
 * @param traj_file Output trajectory file stream.
 * @param t Simulation time in seconds.
 * @param current_mass Vehicle mass in kilograms.
 * @param true_state Pointer to true state.
 * @param est_state Pointer to estimated state.
 */
void write_trajectory_state(FILE *traj_file, double t, double current_mass,
                            state *true_state, state *est_state) {
  fprintf(traj_file,
          "%g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, "
          "%g, %g, %g, %g, %g, %g, %g, %g, %g, %g\n",
          t, current_mass, true_state->position.x, true_state->position.y,
          true_state->position.z, true_state->velocity.x,
          true_state->velocity.y, true_state->velocity.z,
          norm(true_state->a_lift), 0.0, 0.0, 0.0, est_state->position.x,
          est_state->position.y, est_state->position.z, est_state->velocity.x,
          est_state->velocity.y, est_state->velocity.z, 0.0, 0.0, 0.0,
          true_state->a_lift.x, true_state->a_lift.y, true_state->a_lift.z,
          est_state->a_lift.x, est_state->a_lift.y, est_state->a_lift.z);
}

/**
 * Performs linear interpolation on tabulated data.
 *
 * @param x Query value.
 * @param xs Monotonic x-value array.
 * @param ys Corresponding y-value array.
 * @param n Number of data points.
 * @return Interpolated y-value.
 */
double linterp(double x, double xs[], double ys[], int n) {

  // Initialize the output value
  double y = 0;

  // Find the two points to interpolate between
  int i = 0;
  while (x > xs[i]) {
    i++;
  }

  if (i == 0) {
    y = ys[0];
    return y;
  }

  // Perform the interpolation
  y = ys[i - 1] + (ys[i] - ys[i - 1]) * (x - xs[i - 1]) / (xs[i] - xs[i - 1]);

  return y;
}

/**
 * Returns the minimum of two values.
 *
 * @param a First value.
 * @param b Second value.
 * @return Smaller of `a` and `b`.
 */
double min(double a, double b) {

  if (a < b) {
    return a;
  } else {
    return b;
  }
}

/**
 * Returns the sign of a value.
 *
 * @param x Input value.
 * @return `1` if positive, `-1` if negative, otherwise `0`.
 */
double sign(double x) {

  if (x > 0) {
    return 1;
  } else if (x < 0) {
    return -1;
  } else {
    return 0;
  }
}

double get_max_a_exec(runparams *run_params, vehicle *vehicle) {
  double max_flap_force =
      run_params->actuator_force * run_params->gearing_ratio * 1000;
  double max_lift_force =
      (vehicle->rv.c_l_alpha * max_flap_force *
       (vehicle->rv.x_flap - vehicle->rv.x_com) /
       (vehicle->rv.c_m_alpha *
        vehicle->rv.rv_length)); // maximum lift force in N, based on moment arm
                                 // and lift properties
  double max_a_exec = (max_lift_force /
                       vehicle->rv.rv_mass); // maximum acceleration that can be
                                             // executed by the flaps in m/s^2
  return max_a_exec;
}

/**
 * Clip a value to a specified range
 *
 * @param value value to be clipped
 * @param min minimum value
 * @param max maximum value
 * @return clipped value
 */
double clip(double value, double min, double max) {

  if (value < min) {
    return min;
  } else if (value > max) {
    return max;
  } else {
    return value;
  }
}

#endif