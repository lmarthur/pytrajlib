#ifndef VEHICLE_H
#define VEHICLE_H

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "../utils/runparams.h"

#define SWERVE_AERO_TABLE_SIZE 51

/* Mach-12 undeflected-body aerodynamic table vs alpha (deg). */
static const double SWERVE_ALPHA_DEG_TABLE[SWERVE_AERO_TABLE_SIZE] = {
    0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.2,  2.4,
    2.6, 2.8, 3.0, 3.2, 3.4, 3.6, 3.8, 4.0, 4.2, 4.4, 4.6, 4.8,  5.0,
    5.2, 5.4, 5.6, 5.8, 6.0, 6.2, 6.4, 6.6, 6.8, 7.0, 7.2, 7.4,  7.6,
    7.8, 8.0, 8.2, 8.4, 8.6, 8.8, 9.0, 9.2, 9.4, 9.6, 9.8, 10.0,
};

static const double SWERVE_CD_TABLE[SWERVE_AERO_TABLE_SIZE] = {
    0.0180, 0.0181, 0.0182, 0.0183, 0.0185, 0.0188, 0.0192, 0.0196, 0.0201,
    0.0205, 0.0212, 0.0219, 0.0226, 0.0234, 0.0243, 0.0252, 0.0262, 0.0273,
    0.0284, 0.0296, 0.0309, 0.0322, 0.0337, 0.0351, 0.0367, 0.0383, 0.0399,
    0.0417, 0.0436, 0.0455, 0.0474, 0.0494, 0.0515, 0.0538, 0.0559, 0.0583,
    0.0607, 0.0632, 0.0657, 0.0684, 0.0711, 0.0740, 0.0769, 0.0797, 0.0828,
    0.0861, 0.0893, 0.0925, 0.0960, 0.0994, 0.1031,
};

static const double SWERVE_CL_TABLE[SWERVE_AERO_TABLE_SIZE] = {
    0.000, 0.006, 0.012, 0.018, 0.024, 0.030, 0.036, 0.042, 0.048, 0.055, 0.061,
    0.067, 0.073, 0.080, 0.086, 0.092, 0.099, 0.105, 0.112, 0.118, 0.125, 0.131,
    0.138, 0.144, 0.151, 0.157, 0.164, 0.171, 0.177, 0.184, 0.191, 0.198, 0.205,
    0.212, 0.219, 0.226, 0.234, 0.241, 0.249, 0.256, 0.264, 0.272, 0.280, 0.288,
    0.296, 0.304, 0.313, 0.321, 0.329, 0.338, 0.347,
};

static const double SWERVE_CN_TABLE[SWERVE_AERO_TABLE_SIZE] = {
    0.000, 0.006, 0.012, 0.018, 0.024, 0.030, 0.036, 0.043, 0.049, 0.055, 0.061,
    0.068, 0.074, 0.081, 0.087, 0.094, 0.100, 0.107, 0.113, 0.120, 0.126, 0.133,
    0.140, 0.147, 0.153, 0.160, 0.167, 0.174, 0.181, 0.188, 0.195, 0.202, 0.209,
    0.217, 0.224, 0.232, 0.239, 0.247, 0.255, 0.263, 0.271, 0.280, 0.288, 0.296,
    0.305, 0.314, 0.323, 0.331, 0.341, 0.350, 0.360,
};

static const double SWERVE_CM_TABLE[SWERVE_AERO_TABLE_SIZE] = {
    0.0000,  -0.0002, -0.0003, -0.0005, -0.0008, -0.0010, -0.0012, -0.0014,
    -0.0016, -0.0018, -0.0021, -0.0023, -0.0026, -0.0029, -0.0032, -0.0035,
    -0.0038, -0.0041, -0.0043, -0.0047, -0.0050, -0.0054, -0.0057, -0.0061,
    -0.0065, -0.0069, -0.0072, -0.0076, -0.0079, -0.0084, -0.0088, -0.0093,
    -0.0098, -0.0101, -0.0106, -0.0110, -0.0115, -0.0120, -0.0126, -0.0130,
    -0.0135, -0.0140, -0.0146, -0.0152, -0.0157, -0.0164, -0.0169, -0.0175,
    -0.0181, -0.0187, -0.0194,
};

static const double SWERVE_CMQ_TABLE[SWERVE_AERO_TABLE_SIZE] = {
    -0.118, -0.119, -0.120, -0.121, -0.121, -0.122, -0.123, -0.124, -0.125,
    -0.125, -0.126, -0.127, -0.128, -0.129, -0.129, -0.130, -0.131, -0.131,
    -0.132, -0.133, -0.134, -0.135, -0.136, -0.136, -0.137, -0.138, -0.138,
    -0.140, -0.141, -0.143, -0.145, -0.147, -0.149, -0.151, -0.153, -0.155,
    -0.158, -0.160, -0.163, -0.165, -0.167, -0.170, -0.172, -0.175, -0.177,
    -0.180, -0.182, -0.185, -0.187, -0.190, -0.193,
};

// Define a booster struct to store booster parameters
typedef struct booster {
  // Booster parameters
  char name[8];           // name of the booster
  int num_stages;         // number of stages
  double maxdiam;         // maximum diameter in meters
  double area;            // reference area in square meters
  double total_burn_time; // total burn time in seconds
  double bus_mass;        // mass of the bus in kg
  double total_mass;      // total mass in kg
  double c_d_0;           // zero lift drag coefficient

  // Stage parameters
  double wet_mass[3];  // wet mass of each stage in kg
  double fuel_mass[3]; // fuel mass of each stage in kg
  double dry_mass[3];  // dry mass of each stage in kg
  double isp0[3];      // sea level specific impulse of each stage in seconds
  double burn_time[3]; // burn time of each stage in seconds
  double fuel_burn_rate[3]; // fuel burn rate of each stage in kg/s

} booster;

// Define a reentry_vehicle struct to store reentry vehicle parameters
typedef struct rv {
  // Reentry vehicle parameters
  char name[8];             // name of the reentry vehicle
  int maneuverability_flag; // flag to indicate if the reentry vehicle is
                            // maneuverable (1) or not (0)
  double rv_mass;           // mass of the reentry vehicle in kg
  double rv_length;         // length of the reentry vehicle in meters
  double rv_radius;         // radius of the reentry vehicle in meters
  double half_angle;        // cone half-angle in radians
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

} rv;

// Define a vehicle struct to store vehicle parameters
typedef struct vehicle {
  booster booster; // booster struct
  rv rv;           // reentry vehicle struct

  // Vehicle parameters
  double total_mass; // total mass in kg

} vehicle;

// Define a function to initialize a ballistic rv
/**
 * Initializes a ballistic reentry vehicle.
 *
 * @return Ballistic reentry vehicle parameters.
 */
rv init_ballistic_rv() {

  rv rv;
  // Define parameters for a ballistic reentry vehicle
  strcpy(rv.name, "Ball");
  rv.maneuverability_flag = 0;
  rv.rv_mass = 400;
  rv.rv_length = 1.5;
  rv.rv_radius = 0.23;
  rv.half_angle = 0.0916; // 5.25 degrees
  rv.rv_area = M_PI * rv.rv_radius * rv.rv_radius;
  rv.c_d_0 = 0.1;
  rv.c_d_alpha = 0.4;
  rv.c_m_alpha = -0.1;
  rv.c_m_q = -0.1;
  rv.c_l_alpha = 1.5;
  rv.flap_area = 0;
  rv.x_flap = 0;
  rv.x_com = 0.75;
  rv.Iyy = 290;

  return rv;
}

// Define a function to initialize a maneuverable rv
/**
 * Initializes a maneuverable reentry vehicle.
 *
 * @return Maneuverable reentry vehicle parameters.
 */
rv init_swerve_rv() {

  rv rv;
  // Define parameters for a maneuverable reentry vehicle
  // NOTE: C_D, C_L, C_M, C_M_q all use the tables above for a maneuvering
  // reentry vehicle
  strcpy(rv.name, "SWERVE");
  rv.maneuverability_flag = 1;
  rv.rv_mass = 450;
  rv.rv_length = 2.75;
  rv.rv_radius = 0.277;
  rv.half_angle = 0.0916; // 5.25 degrees
  rv.rv_area = M_PI * rv.rv_radius * rv.rv_radius;
  rv.c_d_0 = 0.0180;
  rv.c_d_alpha = 0.487;
  rv.c_m_alpha = -0.111;
  rv.c_m_q = -0.429;
  rv.c_l_alpha = 1.988;
  rv.c_m_delta = 0.059;
  rv.flap_area = 0.04;
  rv.x_flap = -2.65;
  rv.x_com = -0.6 * rv.rv_length;
  rv.Iyy = 290;

  return rv;
}

// Define a function to initialize a MMIII booster
/**
 * Initializes a MMIII booster.
 *
 * @return MMIII booster parameters.
 */
booster init_mmiii_booster() {

  booster booster;
  // Define parameters for a MMIII booster
  strcpy(booster.name, "MMIII");
  booster.num_stages = 3;
  booster.maxdiam = 1.7;
  booster.area = 2.2698;
  booster.c_d_0 = 0.15;
  booster.bus_mass = 100; // mass of the bus/payload carrier in kg

  // Define stage parameters for a MMIII booster
  booster.wet_mass[0] = 23230;
  booster.fuel_mass[0] = 20780;
  booster.dry_mass[0] = booster.wet_mass[0] - booster.fuel_mass[0];
  booster.isp0[0] = 267 * 9.81;
  booster.burn_time[0] = 61;
  booster.fuel_burn_rate[0] = booster.fuel_mass[0] / booster.burn_time[0];

  booster.wet_mass[1] = 7270;
  booster.fuel_mass[1] = 6240;
  booster.dry_mass[1] = booster.wet_mass[1] - booster.fuel_mass[1];
  booster.isp0[1] = 287 * 9.81;
  booster.burn_time[1] = 66;
  booster.fuel_burn_rate[1] = booster.fuel_mass[1] / booster.burn_time[1];

  booster.wet_mass[2] = 3710;
  booster.fuel_mass[2] = 3306;
  booster.dry_mass[2] = booster.wet_mass[2] - booster.fuel_mass[2];
  booster.isp0[2] = 285 * 9.81;
  booster.burn_time[2] = 61;
  booster.fuel_burn_rate[2] = booster.fuel_mass[2] / booster.burn_time[2];

  // Define total burn time and mass
  booster.total_burn_time = 0;
  booster.total_mass = booster.bus_mass;
  for (int i = 0; i < booster.num_stages; i++) {
    booster.total_burn_time += booster.burn_time[i];
    booster.total_mass += booster.wet_mass[i];
  }

  return booster;
}

/**
 * Apply optional vehicle parameter overrides.
 *
 * @param vehicle Pointer to vehicle to update
 * @param run_params Run parameters containing override values.
 */
void apply_vehicle_overrides(vehicle *vehicle, runparams *run_params) {
  if (run_params == NULL) {
    return;
  }

  vehicle->rv.rv_mass = run_params->rv_mass;
  vehicle->rv.rv_length = run_params->rv_length;
  vehicle->rv.rv_radius = run_params->rv_radius;
  vehicle->rv.rv_area = M_PI * run_params->rv_radius * run_params->rv_radius;
  vehicle->rv.c_d_0 = run_params->rv_c_d_0;
  vehicle->rv.c_d_alpha = run_params->rv_c_d_alpha;

  vehicle->booster.area = run_params->booster_area;
  vehicle->booster.maxdiam = run_params->booster_maxdiam;
  vehicle->booster.c_d_0 = run_params->booster_c_d_0;
  double old_bus_mass = vehicle->booster.bus_mass;
  vehicle->booster.bus_mass = run_params->booster_bus_mass;
  vehicle->booster.total_mass += run_params->booster_bus_mass - old_bus_mass;

  vehicle->total_mass = vehicle->booster.total_mass + vehicle->rv.rv_mass;
}

// Define a function to initialize a mmiii vehicle carrying a ballistic reentry
// vehicle
/**
 * Initializes a MMIII vehicle carrying a ballistic reentry vehicle.
 *
 * @return Vehicle parameters.
 */
vehicle init_mmiii_ballistic(runparams *run_params) {
  vehicle vehicle;
  // Define parameters for a MMIII vehicle carrying a ballistic reentry vehicle
  vehicle.booster = init_mmiii_booster();
  vehicle.rv = init_ballistic_rv();
  vehicle.total_mass = vehicle.booster.total_mass + vehicle.rv.rv_mass;
  apply_vehicle_overrides(&vehicle, run_params);

  return vehicle;
}

// Define a function to initialize a mmiii vehicle carrying a maneuverable
// reentry vehicle
/**
 * Initializes a MMIII vehicle carrying a maneuverable reentry vehicle.
 *
 * @param run_params Run parameters containing optional override values.
 * @return Vehicle parameters.
 */
vehicle init_mmiii_swerve(runparams *run_params) {

  vehicle vehicle;
  // Define parameters for a MMIII vehicle carrying a maneuverable reentry
  // vehicle
  vehicle.booster = init_mmiii_booster();
  vehicle.rv = init_swerve_rv();
  vehicle.total_mass = vehicle.booster.total_mass + vehicle.rv.rv_mass;
  apply_vehicle_overrides(&vehicle, run_params);

  return vehicle;
}

// Define a function to initialize a mock booster
/**
 * Initializes a mock booster.
 *
 * @return Mock booster parameters.
 */
booster init_mock_booster() {

  booster booster;
  // Define parameters for a MMIII booster
  strcpy(booster.name, "MMIII");
  booster.num_stages = 3;
  booster.maxdiam = 1.7;
  booster.area = 2.2698;
  booster.c_d_0 = 0.15;
  booster.bus_mass = 0; // mass of the bus/payload carrier in kg

  // Define stage parameters for a MMIII booster
  booster.wet_mass[0] = 0;
  booster.fuel_mass[0] = 0;
  booster.dry_mass[0] = booster.wet_mass[0] - booster.fuel_mass[0];
  booster.isp0[0] = 0;
  booster.burn_time[0] = 0;
  booster.fuel_burn_rate[0] = 0;

  booster.wet_mass[1] = 0;
  booster.fuel_mass[1] = 0;
  booster.dry_mass[1] = booster.wet_mass[1] - booster.fuel_mass[1];
  booster.isp0[1] = 0;
  booster.burn_time[1] = 0;
  booster.fuel_burn_rate[1] = 0;

  booster.wet_mass[2] = 0;
  booster.fuel_mass[2] = 0;
  booster.dry_mass[2] = booster.wet_mass[2] - booster.fuel_mass[2];
  booster.isp0[2] = 0;
  booster.burn_time[2] = 0;
  booster.fuel_burn_rate[2] = 0;

  // Define total burn time and mass
  booster.total_burn_time = 0;
  booster.total_mass = 0;
  for (int i = 0; i < booster.num_stages; i++) {
    booster.total_burn_time += booster.burn_time[i];
    booster.total_mass += booster.wet_mass[i];
  }

  return booster;
}

// Define a function to initialize a mock reentry vehicle
/**
 * Initializes a mock reentry vehicle.
 *
 * @return Mock reentry vehicle parameters.
 */
rv init_mock_rv() {

  rv rv;
  // Define parameters for a ballistic reentry vehicle
  strcpy(rv.name, "Mock");
  rv.maneuverability_flag = 0;
  rv.rv_mass = 100;
  rv.rv_length = 1;
  rv.rv_radius = 1;
  rv.half_angle = 0.0916; // 5.25 degrees
  rv.rv_area = 1;
  rv.c_d_0 = 0.1;
  rv.c_d_alpha = 0;
  rv.c_m_alpha = 0;
  rv.c_m_q = 0;
  rv.c_l_alpha = 0;
  rv.flap_area = 0;
  rv.x_flap = 0;
  rv.x_com = 0;
  rv.Iyy = 0;

  return rv;
}

// Define a function to initialize a mock vehicle
/**
 * Initializes a mock vehicle.
 *
 * @return Mock vehicle parameters.
 */
vehicle init_mock_vehicle() {

  vehicle vehicle;
  vehicle.booster = init_mock_booster();
  vehicle.rv = init_mock_rv();
  vehicle.total_mass = vehicle.booster.total_mass + vehicle.rv.rv_mass;
  return vehicle;
}

/**
 * Initializes a reentry-only vehicle configuration.
 *
 * @return Reentry-only vehicle parameters.
 */
vehicle init_reentry_only() {

  vehicle vehicle;
  vehicle.booster = init_mock_booster();
  vehicle.rv = init_swerve_rv();
  vehicle.total_mass = vehicle.booster.total_mass + vehicle.rv.rv_mass;

  return vehicle;
}

/**
 * Updates vehicle mass based on stage burn timing.
 *
 * @param vehicle Pointer to vehicle struct.
 * @param t Current simulation time in seconds.
 */
double get_vehicle_mass(vehicle *vehicle, double t) {

  // If after burnout, set the mass to the reentry vehicle mass

  if (t > vehicle->booster.total_burn_time) {
    return vehicle->rv.rv_mass;
  } else {
    if (t <= vehicle->booster.burn_time[0]) {
      // First stage is burning
      return vehicle->total_mass - t * vehicle->booster.fuel_burn_rate[0];
    }
    if (t <= (vehicle->booster.burn_time[1] + vehicle->booster.burn_time[0]) &&
        t > vehicle->booster.burn_time[0]) {
      // Second stage is burning
      return vehicle->total_mass - vehicle->booster.wet_mass[0] -
             (t - vehicle->booster.burn_time[0]) *
                 vehicle->booster.fuel_burn_rate[1];
    }
    if (t <= (vehicle->booster.burn_time[2] + vehicle->booster.burn_time[1] +
              vehicle->booster.burn_time[0]) &&
        t > (vehicle->booster.burn_time[1] + vehicle->booster.burn_time[0])) {
      // Third stage is burning
      return vehicle->total_mass - vehicle->booster.wet_mass[0] -
             vehicle->booster.wet_mass[1] -
             (t - vehicle->booster.burn_time[0] -
              vehicle->booster.burn_time[1]) *
                 vehicle->booster.fuel_burn_rate[2];
    }
  }

  printf("Warning: Unable to calculate vehicle mass\n");
  return NAN;
}

#endif