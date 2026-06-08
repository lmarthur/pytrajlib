#ifndef UTILS_H
#define UTILS_H

#include <math.h>
#include <stdio.h>

#include "../math/linalg.h"
#include "../models/vehicle.h"
#include "constants.h"
#include "runparams.h"

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
  printf("Output path: %s\n", run_params->output_path);
  printf("Trajectory path: %s\n", run_params->trajectory_path);
  printf("Number of Monte Carlo runs: %d\n", run_params->num_runs);
  printf("Time step boost: %f\n", run_params->time_step_boost);
  printf("Time step lambert: %f\n", run_params->time_step_lambert);
  printf("Time step midcourse: %f\n", run_params->time_step_midcourse);
  printf("Time step reentry: %f\n", run_params->time_step_reentry);
  printf("Trajectory output: %d\n", run_params->traj_output);
  printf("Target x-coordinate: %f\n", run_params->x_aim);
  printf("Target y-coordinate: %f\n", run_params->y_aim);
  printf("Target z-coordinate: %f\n", run_params->z_aim);
  printf("Longitudinal thrust angle: %f\n", run_params->theta_long);
  printf("Latitudinal thrust angle: %f\n", run_params->theta_lat);

  printf("Gravitational perturbations: %d\n", run_params->grav_error);
  printf("Ballistic drag: %d\n", run_params->ballistic_drag);
  printf("Atmospheric model: %d\n", run_params->atm_model);
  printf("GNSS navigation: %d\n", run_params->gnss_nav);
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
  printf("GNSS frequency: %f Hz\n", run_params->gnss_freq);
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

/**
 * The maximum achievable flap force is limited by actuator force $F_a$ and
 * gearing ratio $G$:
 * $$
 * \begin{align}
 * F_\text{flap,max} = G F_a.
 * \end{align}
 * $$
 * The actuator rate limit used by lift control is modeled as
 * $$
 * \begin{align}
 * \dot\delta_\text{max} = \frac{\delta_\text{max}}{t_\text{deflect}}.
 * \end{align}
 * $$
 */
double get_max_flap_force(runparams *run_params, vehicle *vehicle) {
  double max_flap_force =
      run_params->actuator_force * run_params->gearing_ratio * 1000;
  return max_flap_force;
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

quaternion align_roll_axis_with_vector(cartvec velocity) {
  // Normalize velocity
  double v_norm_val = norm(velocity);
  if (v_norm_val < 1e-12) {
    return identity_quaternion(); // Default orientation
  }
  cartvec v_normalized = smultiply(velocity, 1.0 / v_norm_val);

  // Create quaternion that rotates the roll axis [0,0,-1] to v_normalized
  cartvec from = {0.0, 0.0, -1.0};

  // Rotation axis: from × to
  cartvec rot_axis = cross(from, v_normalized);
  double axis_norm = norm(rot_axis);

  if (axis_norm < 1e-12) {
    // Aligned or opposite
    double dot_prod = dot(from, v_normalized);
    if (dot_prod > 0) {
      return identity_quaternion();
    } else {
      // 180° rotation around Y-axis
      return (quaternion){0.0, 0.0, 1.0, 0.0};
    }
  }

  rot_axis = smultiply(rot_axis, 1.0 / axis_norm);
  double angle = acos(dot(from, v_normalized));

  // Quaternion from axis-angle
  double half_angle = angle / 2.0;
  return (quaternion){cos(half_angle), sin(half_angle) * rot_axis.x,
                      sin(half_angle) * rot_axis.y,
                      sin(half_angle) * rot_axis.z};
}

#endif