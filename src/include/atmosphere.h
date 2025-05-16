#ifndef ATMOSPHERE_H
#define ATMOSPHERE_H

#include <math.h>
#include "rng/rng.h"
#include "utils.h"

// Define an atm_cond struct to store local atmospheric conditions
typedef struct atm_cond{
    double altitude; // altitude in meters
    double density; // density in kg/m^3
    double meridional_wind; // meridional wind in m/s
    double zonal_wind; // zonal wind in m/s
    double vertical_wind; // vertical wind in m/s

    // General atmospheric parameters
} atm_cond;

// Define an atm_model struct to store the atmospheric model
typedef struct atm_model{
    // Constants
    double scale_height; // scale height in meters
    double sea_level_density; // sea level density in kg/m^3

    // Standard deviations
    double std_densities[4];
    double std_winds[4];
    double std_vert_winds[4];

    // Perturbations
    double pert_densities[4];
    double pert_zonal_winds[4];
    double pert_meridional_winds[4];
    double pert_vert_winds[4];

} atm_model;

// Define an eg16_profile struct to store the atmospheric profile data
typedef struct eg16_profile{
    int profile_num; // profile number
    double alt_data[100]; // altitude data
    double density_data[100]; // density data
    double meridional_wind_data[100]; // meridional wind data
    double zonal_wind_data[100]; // zonal wind data
    double vertical_wind_data[100]; // vertical wind data

} eg16_profile;

double atm_data[10000][6];
int atm_data_is_filled = 0;

void init_atm_data(char* atmprofilepath){
    /*
    Initializes the atmospheric profile data so we only read the file once.

    INPUTS:
    ----------
        atmprofilepath: char *
            path to the atmospheric profile file
    OUTPUTS:
    ----------
        atm_profile: eg16_profile
            atmospheric profile struct
    */

    if (atm_data_is_filled == 1){
        return;
    }

    // Open the atmospheric profile file
    FILE *fp = fopen(atmprofilepath, "r");
    if (fp == NULL){
        printf("Error opening atmospheric profile file\n");
    }

    // read the atmospheric profile data delimited by spaces
    for (int i = 0; i < 100*100; i++){
        for (int j = 0; j < 6; j++){
            fscanf(fp, "%lfe", &atm_data[i][j]);
        }
    }
    atm_data_is_filled = 1;
    fclose(fp);
    }
    

atm_model init_exp_atm(runparams *run_params){
    /*
    Initializes the atmospheric model

    INPUTS:
    ----------
        run_params: runparams *
            pointer to the run parameters struct
    OUTPUT:
    ----------
        atm_model: atm_model
            atmospheric model
    */

    atm_model atm_model;
    
    // Define constants
    atm_model.scale_height = 8000; // scale height in meters
    atm_model.sea_level_density = 1.225; // sea level density in kg/m^3


    // Non-perturbed branch
    if (run_params->atm_error == 0){

        for (int i = 0; i < 4; i++){
            atm_model.std_densities[i] = 0;
            atm_model.std_winds[i] = 0;
            atm_model.std_vert_winds[i] = 0;
            atm_model.pert_densities[i] = 0;
            atm_model.pert_zonal_winds[i] = 0;
            atm_model.pert_meridional_winds[i] = 0;
            atm_model.pert_vert_winds[i] = 0;
        }

    }
    else{
        // Density standard deviations
        atm_model.std_densities[0] = 0.00009;
        atm_model.std_densities[1] = 0.00001;
        atm_model.std_densities[2] = 0.00262;
        atm_model.std_densities[3] = 0.00662;

        // Wind standard deviations
        atm_model.std_winds[0] = 0.223;
        atm_model.std_winds[1] = 0.098;
        atm_model.std_winds[2] = 1.13;
        atm_model.std_winds[3] = 2.23;

        // Vertical wind standard deviations
        atm_model.std_vert_winds[0] = 0.058;
        atm_model.std_vert_winds[1] = 0.016;
        atm_model.std_vert_winds[2] = 0.070;
        atm_model.std_vert_winds[3] = 0.244;

        for (int i = 0; i < 4; i++){
            // Generate perturbations, which are then used by the get_atm_cond function to generate the true conditions
            atm_model.pert_densities[i] = atm_model.std_densities[i] * ran_gaussian(1);
            atm_model.pert_zonal_winds[i] = atm_model.std_winds[i] * ran_gaussian(1);
            atm_model.pert_meridional_winds[i] = atm_model.std_winds[i] * ran_gaussian(1);
            atm_model.pert_vert_winds[i] = atm_model.std_vert_winds[i] * ran_gaussian(1);
        }

    }

    
    
    return atm_model;
}


atm_cond get_exp_atm_cond(double altitude, atm_model *atm_model){
    /*
    Calculates the atmospheric conditions at a given altitude using an exponential model

    INPUTS:
    ----------
        altitude: double
            altitude in meters
        atm_model: atm_model *
            pointer to the atmospheric model
    OUTPUT:
    ----------
        atm_conditions: atm_cond
            local atmospheric conditions
    */

    atm_cond atm_conditions;
    if (altitude < 0){
        altitude = 0;
    }
    atm_conditions.altitude = altitude;
    atm_conditions.density = atm_model->sea_level_density * exp(-altitude/atm_model->scale_height);
    atm_conditions.meridional_wind = 0;
    atm_conditions.zonal_wind = 0;
    atm_conditions.vertical_wind = 0;

    return atm_conditions;
}


atm_cond get_pert_atm_cond(double altitude, atm_model *atm_model){
    /*
    Calculates the atmospheric conditions at a given altitude using a model based on EarthGRAM 2016 results

    INPUTS:
    ----------
        altitude: double
            altitude in meters
        atm_model: atm_model *
            pointer to the atmospheric model
    OUTPUT:
    ----------
        atm_conditions: atm_cond
            local atmospheric conditions
    */

    atm_cond atm_conditions;
    if (altitude < 0){
        altitude = 0;
    }
    atm_conditions.altitude = altitude;

    // Use if statements to determine the standard deviations to use
    
    // Density
    if (altitude < 5000 && altitude >= 0){
        atm_conditions.density = atm_model->sea_level_density * exp(-altitude/atm_model->scale_height);
        atm_conditions.density += atm_model->pert_densities[0] * atm_conditions.density;
    }
    else if (altitude < 50000){
        atm_conditions.density = atm_model->sea_level_density * exp(-altitude/atm_model->scale_height);
        atm_conditions.density += atm_model->pert_densities[1] * atm_conditions.density;
    }
    else if (altitude < 100000){
        atm_conditions.density = atm_model->sea_level_density * exp(-altitude/atm_model->scale_height);
        atm_conditions.density += atm_model->pert_densities[2] * atm_conditions.density;
    }
    else{
        atm_conditions.density = atm_model->sea_level_density * exp(-altitude/atm_model->scale_height);
        atm_conditions.density += atm_model->pert_densities[3] * atm_conditions.density;
    }

    // Wind
    if (altitude < 5000 && altitude >= 0){
        atm_conditions.meridional_wind = atm_model->pert_meridional_winds[0];
        atm_conditions.zonal_wind = atm_model->pert_zonal_winds[0];
        atm_conditions.vertical_wind = atm_model->pert_vert_winds[0];
    }
    else if (altitude < 50000){
        atm_conditions.meridional_wind = atm_model->pert_meridional_winds[1];
        atm_conditions.zonal_wind = atm_model->pert_zonal_winds[1];
        atm_conditions.vertical_wind = atm_model->pert_vert_winds[1];
    }
    else if (altitude < 100000){
        atm_conditions.meridional_wind = atm_model->pert_meridional_winds[2];
        atm_conditions.zonal_wind = atm_model->pert_zonal_winds[2];
        atm_conditions.vertical_wind = atm_model->pert_vert_winds[2];
    }
    else{
        atm_conditions.meridional_wind = atm_model->pert_meridional_winds[3];
        atm_conditions.zonal_wind = atm_model->pert_zonal_winds[3];
        atm_conditions.vertical_wind = atm_model->pert_vert_winds[3];
    }

    return atm_conditions;
}

atm_cond get_eg_atm_cond(double altitude, eg16_profile *atm_profile){
    /*
    Calculates the atmospheric conditions at a given altitude using an EarthGRAM 2016 profile

    INPUTS:
    ----------
        altitude: double
            altitude in meters
        atm_profile: eg16_profile *
            pointer to the EarthGRAM 2016 profile
    OUTPUT:
    ----------
        atm_conditions: atm_cond
            local atmospheric conditions
    */

    atm_cond atm_conditions;
    altitude = altitude/1000;
    
    if (altitude < 0){
        altitude = 0;
    }

    atm_conditions.altitude = altitude;

    if (altitude > 99){
        atm_conditions.density = 0;
        atm_conditions.meridional_wind = 0;
        atm_conditions.zonal_wind = 0;
        atm_conditions.vertical_wind = 0;
        return atm_conditions;
    }

    // Use linear interpolation to get the atmospheric conditions
    atm_conditions.density = linterp(altitude, atm_profile->alt_data, atm_profile->density_data, 100);
    atm_conditions.meridional_wind = linterp(altitude, atm_profile->alt_data, atm_profile->meridional_wind_data, 100);
    atm_conditions.zonal_wind = linterp(altitude, atm_profile->alt_data, atm_profile->zonal_wind_data, 100);
    atm_conditions.vertical_wind = linterp(altitude, atm_profile->alt_data, atm_profile->vertical_wind_data, 100);
    
    return atm_conditions;
}

atm_cond get_atm_cond(double altitude, atm_model *exp_atm_model, runparams *run_params, eg16_profile *atm_profile){
    /*
    Calculates the atmospheric conditions at a given altitude

    INPUTS:
    ----------
        altitude: double
            altitude in meters
        atm_model: atm_model *
            pointer to the exponential atmospheric model
        run_params: runparams *
            pointer to the run parameters struct
    OUTPUT:
    ----------
        atm_conditions: atm_cond
            local atmospheric conditions
    */

    atm_cond atm_conditions;

    if (run_params->atm_error == 0){
        atm_conditions = get_exp_atm_cond(altitude, exp_atm_model);
    }
    else{
        if (run_params->atm_model == 0){
            atm_conditions = get_pert_atm_cond(altitude, exp_atm_model);
        }
        else{
            // EarthGRAM branch

            atm_conditions = get_eg_atm_cond(altitude, atm_profile);
        }

    }
    
    return atm_conditions;
}

eg16_profile parse_atm(char* atmprofilepath, int profilenum){
    /*
    Parses the atmospheric profile data and updates the 2d atm_data array by selecting only the requested profile

    INPUTS:
    ----------
        atmprofilepath: char *
            path to the atmospheric profile file
        profilenum: int
            index number of the atmospheric profile to use
    OUTPUTS:
    ----------
        atm_profile: eg16_profile
            atmospheric profile struct
        
    */
    // Initialize the atmospheric profile struct
    eg16_profile atm_profile;
    atm_profile.profile_num = profilenum;

    init_atm_data(atmprofilepath);
    
    // Update the atmospheric profile struct by iterating over only the requested profile
    for (int i = 0; i < 100; i++){
        atm_profile.alt_data[i] = atm_data[100*profilenum+i][1];
        atm_profile.density_data[i] = atm_data[100*profilenum+i][2];
        atm_profile.meridional_wind_data[i] = atm_data[100*profilenum+i][3];
        atm_profile.zonal_wind_data[i] = atm_data[100*profilenum+i][4];
        atm_profile.vertical_wind_data[i] = atm_data[100*profilenum+i][5];
    }

    return atm_profile;
}

#endif