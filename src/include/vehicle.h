#ifndef VEHICLE_H
#define VEHICLE_H
#define _USE_MATH_DEFINES

#include <math.h>
#include <string.h>
#include <stdio.h>

// Define a booster struct to store booster parameters
typedef struct booster{
    // Booster parameters
    char name[8]; // name of the booster
    int num_stages; // number of stages
    double maxdiam; // maximum diameter in meters
    double area; // reference area in square meters
    double total_burn_time; // total burn time in seconds
    double bus_mass; // mass of the bus in kg
    double total_mass; // total mass in kg
    double c_d_0; // zero lift drag coefficient
    
    // Stage parameters
    double wet_mass[3]; // wet mass of each stage in kg
    double fuel_mass[3]; // fuel mass of each stage in kg
    double dry_mass[3]; // dry mass of each stage in kg
    double isp0[3]; // sea level specific impulse of each stage in seconds
    double burn_time[3]; // burn time of each stage in seconds
    double fuel_burn_rate[3]; // fuel burn rate of each stage in kg/s

} booster;

// Define a reentry_vehicle struct to store reentry vehicle parameters
typedef struct rv{
    // Reentry vehicle parameters
    char name[8]; // name of the reentry vehicle
    int maneuverability_flag; // flag to indicate if the reentry vehicle is maneuverable (1) or not (0)
    double rv_mass; // mass of the reentry vehicle in kg
    double rv_length; // length of the reentry vehicle in meters
    double rv_radius; // radius of the reentry vehicle in meters
    double rv_area; // reference area of the reentry vehicle in square meters
    double c_d_0; // zero lift drag coefficient
    double c_d_alpha; // drag coefficient derivative (per radian)
    double c_m_alpha; // pitching moment coefficient derivative (per radian)
    double c_m_q; // pitch damping coefficient
    double c_l_alpha; // lift coefficient derivative (per radian, valid for small angles of attack)
    double flap_area; // flap area in square meters
    double x_flap; // x-coordinate of the flap hinge in meters
    double x_com; // x-coordinate of the center of mass in meters
    double Iyy; // moment of inertia about the y-axis and x-axis (axisymmetric vehicle) in kg*m^2

} rv;

// Define a vehicle struct to store vehicle parameters
typedef struct vehicle{
    booster booster; // booster struct
    rv rv; // reentry vehicle struct

    // Vehicle parameters
    double total_mass; // total mass in kg
    double current_mass; // current mass in kg
    
} vehicle;

// Define a function to initialize a ballistic rv
rv init_ballistic_rv(){
    /*
    Initializes a ballistic reentry vehicle

    OUTPUTS:
    ----------
        rv: rv
            reentry vehicle struct
    */

    rv rv;
    // Define parameters for a ballistic reentry vehicle
    strcpy(rv.name, "Ball");
    rv.maneuverability_flag = 0;
    rv.rv_mass = 400;
    rv.rv_length = 1.5;
    rv.rv_radius = 0.23;
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
rv init_swerve_rv(){
    /*
    Initializes a maneuverable reentry vehicle

    OUTPUTS:
    ----------
        rv: rv
            reentry vehicle struct
    */

    rv rv;
    // Define parameters for a maneuverable reentry vehicle
    strcpy(rv.name, "SWERVE");
    rv.maneuverability_flag = 1;
    rv.rv_mass = 450;
    rv.rv_length = 2.75;
    rv.rv_radius = 0.23;
    rv.rv_area = M_PI * rv.rv_radius * rv.rv_radius;
    rv.c_d_0 = 0.1;
    rv.c_d_alpha = 0.487;
    rv.c_m_alpha = -0.15;
    rv.c_m_q = -0.2;
    rv.c_l_alpha = 1.72;
    rv.flap_area = 0.04;
    rv.x_flap = -2.65;
    rv.x_com = -0.6*rv.rv_length;
    rv.Iyy = 290;

    return rv;
}

// Define a function to initialize a MMIII booster
booster init_mmiii_booster(){
    /*
    Initializes a MMIII booster

    OUTPUTS:
    ----------
        booster: booster
            booster struct
    */

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
    booster.fuel_burn_rate[0] = booster.fuel_mass[0]/booster.burn_time[0];

    booster.wet_mass[1] = 7270;
    booster.fuel_mass[1] = 6240;
    booster.dry_mass[1] = booster.wet_mass[1] - booster.fuel_mass[1];
    booster.isp0[1] = 287 * 9.81;
    booster.burn_time[1] = 66;
    booster.fuel_burn_rate[1] = booster.fuel_mass[1]/booster.burn_time[1];

    booster.wet_mass[2] = 3710;
    booster.fuel_mass[2] = 3306;
    booster.dry_mass[2] = booster.wet_mass[2] - booster.fuel_mass[2];
    booster.isp0[2] = 285 * 9.81;
    booster.burn_time[2] = 61;
    booster.fuel_burn_rate[2] = booster.fuel_mass[2]/booster.burn_time[2];

    // Define total burn time and mass
    booster.total_burn_time = 0;
    booster.total_mass = booster.bus_mass;
    for (int i = 0; i < booster.num_stages; i++){
        booster.total_burn_time += booster.burn_time[i];
        booster.total_mass += booster.wet_mass[i];
    }

    return booster;
}

// Define a function to initialize a mmiii vehicle carrying a ballistic reentry vehicle
vehicle init_mmiii_ballistic(){
    /*
    Initializes a MMIII vehicle carrying a ballistic reentry vehicle

    OUTPUTS:
    ----------
        vehicle: vehicle
            vehicle struct
    */
    vehicle vehicle;
    // Define parameters for a MMIII vehicle carrying a ballistic reentry vehicle
    vehicle.booster = init_mmiii_booster();
    vehicle.rv = init_ballistic_rv();
    vehicle.total_mass = vehicle.booster.total_mass + vehicle.rv.rv_mass;
    vehicle.current_mass = vehicle.total_mass;

    return vehicle;
}

// Define a function to initialize a mmiii vehicle carrying a maneuverable reentry vehicle
vehicle init_mmiii_swerve(){
    /*
    Initializes a MMIII vehicle carrying a maneuverable reentry vehicle

    OUTPUTS:
    ----------
        vehicle: vehicle
            vehicle struct
    */

    vehicle vehicle;
    // Define parameters for a MMIII vehicle carrying a maneuverable reentry vehicle
    vehicle.booster = init_mmiii_booster();
    vehicle.rv = init_swerve_rv();
    vehicle.total_mass = vehicle.booster.total_mass + vehicle.rv.rv_mass;
    vehicle.current_mass = vehicle.total_mass;

    return vehicle;
}

// Define a function to initialize a mock booster
booster init_mock_booster(){
    /*
    Initializes a mock booster

    OUTPUTS:
    ----------
        booster: booster
            booster struct
    */

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
    for (int i = 0; i < booster.num_stages; i++){
        booster.total_burn_time += booster.burn_time[i];
        booster.total_mass += booster.wet_mass[i];
    }

    return booster;
}

// Define a function to initialize a mock reentry vehicle
rv init_mock_rv(){
    /*
    Initializes a mock reentry vehicle

    OUTPUTS:
    ----------
        rv: rv
            reentry vehicle struct
    */

    rv rv;
    // Define parameters for a ballistic reentry vehicle
    strcpy(rv.name, "Mock");
    rv.maneuverability_flag = 0;
    rv.rv_mass = 100;
    rv.rv_length = 1;
    rv.rv_radius = 1;
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
vehicle init_mock_vehicle(){
    /*
    Initializes a mock vehicle

    OUTPUTS:
    ----------
        vehicle: vehicle
            vehicle struct
    */

    vehicle vehicle;
    vehicle.booster = init_mock_booster();
    vehicle.rv = init_mock_rv();
    vehicle.total_mass = vehicle.booster.total_mass + vehicle.rv.rv_mass;
    vehicle.current_mass = vehicle.total_mass;
    return vehicle;
}

vehicle init_reentry_only(){
    /*
    Initializes a vehicle for reentry only

    OUTPUTS:
    ----------
        vehicle: vehicle
            vehicle struct
    */

    vehicle vehicle;
    vehicle.booster = init_mock_booster();
    vehicle.rv = init_swerve_rv();
    vehicle.total_mass = vehicle.booster.total_mass + vehicle.rv.rv_mass;
    vehicle.current_mass = vehicle.total_mass;

    return vehicle;
}

void update_mass(vehicle *vehicle, double t){
    /*
    Updates the mass of the vehicle based on the current stage and burn time

    INPUTS:
    ----------
        vehicle: vehicle *
            pointer to the vehicle struct
        t: double
            current time in seconds
    */

    // If after burnout, set the mass to the reentry vehicle mass

    if (t > vehicle->booster.total_burn_time){
        vehicle->current_mass = vehicle->rv.rv_mass;
        // break out of the function
        return;
    }
    else{
        if (t <= vehicle->booster.burn_time[0]){
            // First stage is burning
            vehicle->current_mass = vehicle->total_mass - t * vehicle->booster.fuel_burn_rate[0];
        }
        if (t <= (vehicle->booster.burn_time[1] + vehicle->booster.burn_time[0]) && t > vehicle->booster.burn_time[0]){
            // Second stage is burning
            vehicle->current_mass = vehicle->total_mass - vehicle->booster.wet_mass[0] - (t - vehicle->booster.burn_time[0]) * vehicle->booster.fuel_burn_rate[1];
        }
        if (t <= (vehicle->booster.burn_time[2] + vehicle->booster.burn_time[1] + vehicle->booster.burn_time[0]) && t > (vehicle->booster.burn_time[1] + vehicle->booster.burn_time[0])){
            // Third stage is burning
            vehicle->current_mass = vehicle->total_mass - vehicle->booster.wet_mass[0] - vehicle->booster.wet_mass[1] - (t - vehicle->booster.burn_time[0] - vehicle->booster.burn_time[1]) * vehicle->booster.fuel_burn_rate[2];
        }

    }
    
    return;
}

booster init_scud_booster(){
    /*
    Initializes a SCUD missile booster

    OUTPUTS:
    ----------
        booster: booster
            booster struct
    */

    booster booster;
    strcpy(booster.name, "SCUD");
    booster.num_stages = 1;
    booster.maxdiam = 0.88; // [m] max diameter of missile
    booster.area = M_PI * (booster.maxdiam / 2) * (booster.maxdiam / 2);
    booster.c_d_0 = 0.15; // Using V2 drag characteristics
    booster.bus_mass = 987; // TODO NOTE: this is the payload mass, not the bus mass

    // Stage 1 parameters
    booster.wet_mass[0] = 4873;
    booster.fuel_mass[0] = 3771;
    booster.dry_mass[0] = booster.wet_mass[0] - booster.fuel_mass[0];
    booster.isp0[0] = 226 * 9.81;
    booster.burn_time[0] = 62;
    booster.fuel_burn_rate[0] = 58.8;

    // Calculate totals
    booster.total_burn_time = booster.burn_time[0];
    booster.total_mass = booster.wet_mass[0] + booster.bus_mass;

    return booster;
}

vehicle init_scud_ballistic() {
    vehicle vehicle;
    vehicle.booster = init_scud_booster();
    vehicle.rv = init_ballistic_rv();
    vehicle.total_mass = vehicle.booster.total_mass + vehicle.rv.rv_mass;
    vehicle.current_mass = vehicle.total_mass;
}

vehicle init_scud_swerve() {
    vehicle vehicle;
    vehicle.booster = init_scud_booster();
    vehicle.rv = init_swerve_rv();
    vehicle.total_mass = vehicle.booster.total_mass + vehicle.rv.rv_mass;
    vehicle.current_mass = vehicle.total_mass;
}

booster init_scud_er_booster(){
    /*
    Initializes a SCUD-ER missile booster

    OUTPUTS:
    ----------
        booster: booster
            booster struct
    */

    booster booster;
    strcpy(booster.name, "SCUD-ER");
    booster.num_stages = 1;
    booster.maxdiam = 1.0;
    booster.area = M_PI * (booster.maxdiam / 2) * (booster.maxdiam / 2);
    booster.c_d_0 = 0.15; // Using V2 drag characteristics
    booster.bus_mass = 500; // payload mass

    // Stage 1 parameters
    booster.wet_mass[0] = 8730;
    booster.fuel_mass[0] = 7730;
    booster.dry_mass[0] = booster.wet_mass[0] - booster.fuel_mass[0];
    booster.isp0[0] = 230 * 9.81;
    booster.burn_time[0] = 127.8;
    booster.fuel_burn_rate[0] = 57.83;

    // Calculate totals
    booster.total_burn_time = booster.burn_time[0];
    booster.total_mass = booster.wet_mass[0] + booster.bus_mass;

    return booster;
}

vehicle init_scud_er_ballistic() {
    /*
    Initializes a SCUD-ER vehicle carrying a ballistic reentry vehicle

    OUTPUTS:
    ----------
        vehicle: vehicle
            vehicle struct
    */
    vehicle vehicle;
    vehicle.booster = init_scud_er_booster();
    vehicle.rv = init_ballistic_rv();
    vehicle.total_mass = vehicle.booster.total_mass + vehicle.rv.rv_mass;
    vehicle.current_mass = vehicle.total_mass;

    return vehicle;
}

vehicle init_scud_er_swerve() {
    /*
    Initializes a SCUD-ER vehicle carrying a maneuverable reentry vehicle
    OUTPUTS:
    ----------
        vehicle: vehicle
            vehicle struct
    */
    vehicle vehicle;
    vehicle.booster = init_scud_er_booster();
    vehicle.rv = init_swerve_rv();
    vehicle.total_mass = vehicle.booster.total_mass + vehicle.rv.rv_mass;
    vehicle.current_mass = vehicle.total_mass;

    return vehicle;
}

booster init_gbsd_booster(){
    /*
    Initializes a GBSD missile booster

    OUTPUTS:
    ----------
        booster: booster
            booster struct
    */

    booster booster;
    strcpy(booster.name, "GBSD");
    booster.num_stages = 3;
    booster.maxdiam = 1.7;
    booster.area = M_PI * (booster.maxdiam / 2) * (booster.maxdiam / 2);
    booster.c_d_0 = 0.15;
    booster.bus_mass = 600; // W78 + bus

    // Stage 1 parameters
    booster.wet_mass[0] = 23230;
    booster.fuel_mass[0] = 0.89 * booster.wet_mass[0];
    booster.dry_mass[0] = booster.wet_mass[0] - booster.fuel_mass[0];
    booster.isp0[0] = 267 * 9.81;
    booster.burn_time[0] = 61;
    booster.fuel_burn_rate[0] = booster.fuel_mass[0] / booster.burn_time[0];

    // Stage 2 parameters
    booster.wet_mass[1] = 7270;
    booster.fuel_mass[1] = 0.86 * booster.wet_mass[1]; // 6240
    booster.dry_mass[1] = booster.wet_mass[1] - booster.fuel_mass[1];
    booster.isp0[1] = 287 * 9.81;
    booster.burn_time[1] = 66;
    booster.fuel_burn_rate[1] = booster.fuel_mass[1] / booster.burn_time[1];

    // Stage 3 parameters
    booster.wet_mass[2] = 3710;
    booster.fuel_mass[2] = 0.89 * booster.wet_mass[2]; // 3306
    booster.dry_mass[2] = booster.wet_mass[2] - booster.fuel_mass[2];
    booster.isp0[2] = 285 * 9.81;
    booster.burn_time[2] = 61;
    booster.fuel_burn_rate[2] = booster.fuel_mass[2] / booster.burn_time[2];

    // Calculate totals
    booster.total_burn_time = 0;
    booster.total_mass = booster.bus_mass;
    for (int i = 0; i < booster.num_stages; i++){
        booster.total_burn_time += booster.burn_time[i];
        booster.total_mass += booster.wet_mass[i];
    }

    return booster;
}

vehicle init_gbsd_ballistic() {
    /*
    Initializes a GBSD vehicle carrying a ballistic reentry vehicle

    OUTPUTS:
    ----------
        vehicle: vehicle
            vehicle struct
    */

    vehicle vehicle;
    vehicle.booster = init_gbsd_booster();
    vehicle.rv = init_ballistic_rv();
    vehicle.total_mass = vehicle.booster.total_mass + vehicle.rv.rv_mass;
    vehicle.current_mass = vehicle.total_mass;

    return vehicle;
}

vehicle init_gbsd_swerve() {
    /*
    Initializes a GBSD vehicle carrying a maneuverable reentry vehicle

    OUTPUTS:
    ----------
        vehicle: vehicle
            vehicle struct
    */

    vehicle vehicle;
    vehicle.booster = init_gbsd_booster();
    vehicle.rv = init_swerve_rv();
    vehicle.total_mass = vehicle.booster.total_mass + vehicle.rv.rv_mass;
    vehicle.current_mass = vehicle.total_mass;

    return vehicle;
}


booster init_d5_booster(){
    /*
    Initializes a D5 (Trident) missile booster

    OUTPUTS:
    ----------
        booster: booster
            booster struct
    */

    booster booster;
    strcpy(booster.name, "D5");
    booster.num_stages = 3;
    booster.maxdiam = 2.11;
    booster.area = M_PI * (booster.maxdiam / 2) * (booster.maxdiam / 2);
    booster.c_d_0 = 0.15; // Using solid missile drag characteristics
    booster.bus_mass = 1000; // payload mass

    // Stage 1 parameters
    booster.wet_mass[0] = 39241;
    booster.fuel_mass[0] = 33355;
    booster.dry_mass[0] = booster.wet_mass[0] - booster.fuel_mass[0];
    booster.isp0[0] = 281 * 9.81; // m/s
    booster.burn_time[0] = 63;
    booster.fuel_burn_rate[0] = booster.fuel_mass[0] / booster.burn_time[0];

    // Stage 2 parameters
    booster.wet_mass[1] = 11866;
    booster.fuel_mass[1] = 10320;
    booster.dry_mass[1] = booster.wet_mass[1] - booster.fuel_mass[1];
    booster.isp0[1] = 281 * 9.81;
    booster.burn_time[1] = 64;
    booster.fuel_burn_rate[1] = booster.fuel_mass[1] / booster.burn_time[1];

    // Stage 3 parameters
    booster.wet_mass[2] = 2191;
    booster.fuel_mass[2] = 1970;
    booster.dry_mass[2] = booster.wet_mass[2] - booster.fuel_mass[2];
    booster.isp0[2] = 281 * 9.81;
    booster.burn_time[2] = 43;
    booster.fuel_burn_rate[2] = booster.fuel_mass[2] / booster.burn_time[2];

    // Calculate totals
    booster.total_burn_time = 0;
    booster.total_mass = booster.bus_mass;
    for (int i = 0; i < booster.num_stages; i++){
        booster.total_burn_time += booster.burn_time[i];
        booster.total_mass += booster.wet_mass[i];
    }

    return booster;
}

vehicle init_d5_ballistic() {
    /*
    Initializes a D5 vehicle carrying a ballistic reentry vehicle

    OUTPUTS:
    ----------
        vehicle: vehicle
            vehicle struct
    */

    vehicle vehicle;
    vehicle.booster = init_d5_booster();
    vehicle.rv = init_ballistic_rv();
    vehicle.total_mass = vehicle.booster.total_mass + vehicle.rv.rv_mass;
    vehicle.current_mass = vehicle.total_mass;

    return vehicle;
}

vehicle init_d5_swerve() {
    /*
    Initializes a D5 vehicle carrying a maneuverable reentry vehicle

    OUTPUTS:
    ----------
        vehicle: vehicle
            vehicle struct
    */

    vehicle vehicle;
    vehicle.booster = init_d5_booster();
    vehicle.rv = init_swerve_rv();
    vehicle.total_mass = vehicle.booster.total_mass + vehicle.rv.rv_mass;
    vehicle.current_mass = vehicle.total_mass;

    return vehicle;
}

vehicle init_vehicle(int booster_type, int rv_type) {
    /*
    Initializes a vehicle based on the specified booster and reentry vehicle types
    INPUTS:
    ----------
        booster_type: int
            type of booster to use (0: MMIII, 1: SCUD, 2: SCUD-ER, 3: GBSD, 4: D5,
            5: Mock)
        rv_type: int
            type of reentry vehicle to use (0: ballistic, 1: maneuverable)
    OUTPUTS:
    ----------
        vehicle: vehicle
            vehicle struct
    */
    if (booster_type < 0 || booster_type > 5) {
        printf("Invalid booster type specified.\n");
        exit(1);
    }
    if (rv_type < 0 || rv_type > 1) {
        printf("Invalid reentry vehicle type specified.\n");
        exit(1);
    }

    if (booster_type == 0) {
        // MMIII booster
        if (rv_type == 0) {
            return init_mmiii_ballistic();
        } else {
            return init_mmiii_swerve();
        }
    } else if (booster_type == 1) {
        // SCUD booster
        if (rv_type == 0) {
            return init_scud_ballistic();
        } else {
            return init_scud_swerve();
        }
    } else if (booster_type == 2) {
        // SCUD-ER booster
        if (rv_type == 0) {
            return init_scud_er_ballistic();
        } else {
            return init_scud_er_swerve();
        }
    } else if (booster_type == 3) {
        // GBSD booster
        if (rv_type == 0) {
            return init_gbsd_ballistic();
        } else {
            return init_gbsd_swerve();
        }
    } else if (booster_type == 4) {
        // D5 booster
        if (rv_type == 0) {
            return init_d5_ballistic();
        } else {
            return init_d5_swerve();
        }
    } else if (booster_type == 5) {
        // Mock booster
        return init_mock_vehicle();
    }
}

#endif