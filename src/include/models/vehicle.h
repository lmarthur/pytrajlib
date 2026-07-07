#ifndef VEHICLE_H
#define VEHICLE_H

#define _USE_MATH_DEFINES

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "../rng/rng.h"
#include "../utils/runparams.h"

#define MAX_BOOSTER_STAGES 10
#define VEHICLE_NAME_LEN 32
#define AERO_TABLE_MAX_POINTS 51

// Define a booster struct to store booster parameters
typedef struct booster {
  // Booster parameters
  char name[VEHICLE_NAME_LEN]; // name of the booster
  int num_stages;              // number of stages
  double area;                 // reference area in square meters
  double total_burn_time;      // total burn time in seconds
  double bus_mass;             // mass of the bus in kg
  double total_mass;           // total mass in kg
  double c_d_0;                // zero lift drag coefficient

  // Stage parameters
  double wet_mass[MAX_BOOSTER_STAGES];  // wet mass of each stage in kg
  double fuel_mass[MAX_BOOSTER_STAGES]; // fuel mass of each stage in kg
  double dry_mass[MAX_BOOSTER_STAGES];  // dry mass of each stage in kg
  double isp0[MAX_BOOSTER_STAGES]; // sea level specific impulse of each stage
                                   // in seconds
  double burn_time[MAX_BOOSTER_STAGES]; // burn time of each stage in seconds
  double fuel_burn_rate[MAX_BOOSTER_STAGES]; // fuel burn rate of each stage in
                                             // kg/s

} booster;

// Define a reentry_vehicle struct to store reentry vehicle parameters
typedef struct rv {
  // Reentry vehicle parameters
  char name[VEHICLE_NAME_LEN]; // name of the reentry vehicle
  int maneuverability_flag;    // flag to indicate if the reentry vehicle is
                               // maneuverable (1) or not (0)
  double rv_mass;              // mass of the reentry vehicle in kg
  double rv_length;            // length of the reentry vehicle in meters
  double rv_radius;            // radius of the reentry vehicle in meters
  double half_angle;           // cone half-angle in radians
  double rv_area;   // reference area of the reentry vehicle in square meters
  double c_d_0;     // zero lift drag coefficient
  double c_d_alpha; // drag coefficient derivative (per radian)
  double c_m_alpha; // pitching moment coefficient derivative (per radian)
  double c_m_q;     // pitch damping coefficient
  double c_m_delta; // derivative of pitching moment wrt flap deflection angle
  double c_l_alpha; // lift coefficient derivative (per radian, valid for small
                    // angles of attack)
  double flap_area; // flap area in square meters
  double x_flap;    // x-coordinate of the flap hinge in meters
  double x_com;     // x-coordinate of the center of mass in meters
  double Iyy; // moment of inertia about the y-axis and x-axis (axisymmetric
              // vehicle) in kg*m^2

  int aero_table_size;
  double aero_alpha_deg_table[AERO_TABLE_MAX_POINTS];
  double c_d_table[AERO_TABLE_MAX_POINTS];
  double c_l_table[AERO_TABLE_MAX_POINTS];
  double c_m_table[AERO_TABLE_MAX_POINTS];
  double c_m_q_table[AERO_TABLE_MAX_POINTS];

} rv;

// Define a vehicle struct to store vehicle parameters
typedef struct vehicle {
  booster booster; // booster struct
  rv rv;           // reentry vehicle struct

  // Vehicle parameters
  double total_mass; // total mass in kg

} vehicle;

/**
 * Updates vehicle mass based on stage burn timing.
 *
 * @param vehicle Pointer to vehicle struct.
 * @param t Current simulation time in seconds.
 */
double get_vehicle_mass(vehicle *vehicle, double t) {

  double cumulative_burn_time = 0;
  double cumulative_wet_mass = 0;
  for (int i = 0; i < vehicle->booster.num_stages; i++) {
    double stage_end_time =
        cumulative_burn_time + vehicle->booster.burn_time[i];
    if (t <= stage_end_time) {
      return vehicle->total_mass - cumulative_wet_mass -
             (t - cumulative_burn_time) * vehicle->booster.fuel_burn_rate[i];
    }
    cumulative_burn_time = stage_end_time;
    cumulative_wet_mass += vehicle->booster.wet_mass[i];
  }

  if (t > vehicle->booster.total_burn_time) {
    return vehicle->rv.rv_mass;
  }

  printf("Warning: Unable to calculate vehicle mass\n");
  return NAN;
}

/**
 * Apply independent burn-time error to each booster stage.
 *
 * Recomputes stage burn rates and total burn time from perturbed stage times.
 *
 * @param run_params Pointer to run configuration parameters.
 * @param vehicle Pointer to vehicle struct.
 */
static inline void apply_burn_time_error(runparams *run_params,
                                         vehicle *vehicle) {
  double total_burn_time = 0.0;
  for (int i = 0; i < vehicle->booster.num_stages; i++) {
    double perturbed_burn_time = vehicle->booster.burn_time[i] +
                                 ran_gaussian(run_params->burn_time_error);

    vehicle->booster.burn_time[i] = perturbed_burn_time;
    vehicle->booster.fuel_burn_rate[i] =
        vehicle->booster.fuel_mass[i] / perturbed_burn_time;

    total_burn_time += perturbed_burn_time;
  }

  vehicle->booster.total_burn_time = total_burn_time;
}

#endif