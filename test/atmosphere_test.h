#include <tau/tau.h>
#include "../src/include/atmosphere.h"

TEST(atmosphere, init_atm){
    // Initialize the run parameters
    runparams run_params;
    run_params.atm_model = 0;
    run_params.atm_error = 0;

    // Initialize the random number generator
    const gsl_rng_type *T;
    gsl_rng *rng;
    gsl_rng_env_setup();
    T = gsl_rng_default;
    rng = gsl_rng_alloc(T);

    // Initialize the atmospheric model
    atm_model atm_model = init_atm(&run_params, rng);

    // Check the sea level density
    REQUIRE_EQ(atm_model.sea_level_density, 1.225);

    // Check the scale height
    REQUIRE_EQ(atm_model.scale_height, 8000);

    // Check the standard deviations and perturbations
    for (int i = 0; i < 4; i++){
        REQUIRE_EQ(atm_model.std_densities[i], 0);
        REQUIRE_EQ(atm_model.std_winds[i], 0);
        REQUIRE_EQ(atm_model.std_vert_winds[i], 0);
        REQUIRE_EQ(atm_model.pert_densities[i], 0);
        REQUIRE_EQ(atm_model.pert_zonal_winds[i], 0);
        REQUIRE_EQ(atm_model.pert_meridional_winds[i], 0);
        REQUIRE_EQ(atm_model.pert_vert_winds[i], 0);
    }

    run_params.atm_error = 1;
    atm_model = init_atm(&run_params, rng);

    // Check the perturbations
    for (int i = 0; i < 4; i++){
        REQUIRE_GT(atm_model.std_densities[i], 0);
        REQUIRE_GT(atm_model.std_winds[i], 0);
        REQUIRE_GT(atm_model.std_vert_winds[i], 0);
        REQUIRE_NE(atm_model.pert_densities[i], 0);
        REQUIRE_NE(atm_model.pert_densities[i], atm_model.std_densities[i]);
        REQUIRE_NE(atm_model.pert_zonal_winds[i], 0);
        REQUIRE_NE(atm_model.pert_zonal_winds[i], atm_model.std_winds[i]);
        REQUIRE_NE(atm_model.pert_meridional_winds[i], 0);
        REQUIRE_NE(atm_model.pert_meridional_winds[i], atm_model.std_winds[i]);
        REQUIRE_NE(atm_model.pert_vert_winds[i], 0);
        REQUIRE_NE(atm_model.pert_vert_winds[i], atm_model.std_vert_winds[i]);

    }

}

TEST(atmosphere, get_exp_atm_cond){
    // Initialize the run parameters
    runparams run_params;
    run_params.atm_model = 0;
    run_params.atm_error = 0;

    // Initialize the random number generator
    const gsl_rng_type *T;
    gsl_rng *rng;
    gsl_rng_env_setup();
    T = gsl_rng_default;
    rng = gsl_rng_alloc(T);

    // Initialize the atmospheric model
    atm_model atm_model = init_atm(&run_params, rng);

    // Get the atmospheric conditions at sea level
    atm_cond atm_conditions = get_exp_atm_cond(0, &atm_model);

    // Check the altitude
    REQUIRE_EQ(atm_conditions.altitude, 0);

    // Check the density
    REQUIRE_EQ(atm_conditions.density, atm_model.sea_level_density);

    // Check the wind components
    REQUIRE_EQ(atm_conditions.meridional_wind, 0);
    REQUIRE_EQ(atm_conditions.zonal_wind, 0);
    REQUIRE_EQ(atm_conditions.vertical_wind, 0);

    // Get the atmospheric conditions at 10 km
    atm_conditions = get_exp_atm_cond(10000, &atm_model);

    // Check the altitude
    REQUIRE_EQ(atm_conditions.altitude, 10000);

    // Check the density
    REQUIRE_LT(atm_conditions.density, atm_model.sea_level_density);

    // Check the wind components
    REQUIRE_EQ(atm_conditions.meridional_wind, 0);
    REQUIRE_EQ(atm_conditions.zonal_wind, 0);
    REQUIRE_EQ(atm_conditions.vertical_wind, 0);

    // Get the atmospheric conditions at 100 km
    atm_conditions = get_exp_atm_cond(100000, &atm_model);

    // Check the altitude
    REQUIRE_EQ(atm_conditions.altitude, 100000);

    // Check the density
    REQUIRE_LT(atm_conditions.density, atm_model.sea_level_density);

    // Check the wind components
    REQUIRE_EQ(atm_conditions.meridional_wind, 0);
    REQUIRE_EQ(atm_conditions.zonal_wind, 0);
    REQUIRE_EQ(atm_conditions.vertical_wind, 0);

    // Get the atmospheric conditions at 1000 km
    atm_conditions = get_exp_atm_cond(1000000, &atm_model);

    // Check the altitude
    REQUIRE_EQ(atm_conditions.altitude, 1000000);

    // Check the density
    REQUIRE_LT(atm_conditions.density, atm_model.sea_level_density);

    // Repeat the test with perturbation flag enabled
    run_params.atm_error = 1;

    // Initialize the atmospheric model
    atm_model = init_atm(&run_params, rng);

    // Get the atmospheric conditions at sea level
    atm_conditions = get_exp_atm_cond(0, &atm_model);

    // Check the altitude
    REQUIRE_EQ(atm_conditions.altitude, 0);

    // Check the density
    REQUIRE_EQ(atm_conditions.density, atm_model.sea_level_density);

    // Check the wind components
    REQUIRE_EQ(atm_conditions.meridional_wind, 0);
    REQUIRE_EQ(atm_conditions.zonal_wind, 0);
    REQUIRE_EQ(atm_conditions.vertical_wind, 0);

    // Get the atmospheric conditions at 10 km
    atm_conditions = get_exp_atm_cond(10000, &atm_model);

    // Check the altitude
    REQUIRE_EQ(atm_conditions.altitude, 10000);

    // Check the density
    REQUIRE_LT(atm_conditions.density, atm_model.sea_level_density);

    // Check the wind components
    REQUIRE_EQ(atm_conditions.meridional_wind, 0);
    REQUIRE_EQ(atm_conditions.zonal_wind, 0);
    REQUIRE_EQ(atm_conditions.vertical_wind, 0);

    // Get the atmospheric conditions at 100 km
    atm_conditions = get_exp_atm_cond(100000, &atm_model);

    // Check the altitude
    REQUIRE_EQ(atm_conditions.altitude, 100000);

    // Check the density
    REQUIRE_LT(atm_conditions.density, atm_model.sea_level_density);

    // Check the wind components
    REQUIRE_EQ(atm_conditions.meridional_wind, 0);
    REQUIRE_EQ(atm_conditions.zonal_wind, 0);
    REQUIRE_EQ(atm_conditions.vertical_wind, 0);

    // Get the atmospheric conditions at 1000 km
    atm_conditions = get_exp_atm_cond(1000000, &atm_model);

    // Check the altitude
    REQUIRE_EQ(atm_conditions.altitude, 1000000);

    // Check the density
    REQUIRE_LT(atm_conditions.density, atm_model.sea_level_density);

    // Check the atmospheric conditions at negative altitude
    atm_conditions = get_exp_atm_cond(-1000, &atm_model);

    // Check the altitude
    REQUIRE_EQ(atm_conditions.altitude, 0);

    // Check the density
    REQUIRE_EQ(atm_conditions.density, atm_model.sea_level_density);

    // Check the wind components
    REQUIRE_EQ(atm_conditions.meridional_wind, 0);
    REQUIRE_EQ(atm_conditions.zonal_wind, 0);
    REQUIRE_EQ(atm_conditions.vertical_wind, 0);

}

TEST(atmosphere, get_pert_atm_cond){
    // Initialize the run parameters
    runparams run_params;
    run_params.atm_model = 0;
    run_params.atm_error = 0;

    // Initialize the random number generator
    const gsl_rng_type *T;
    gsl_rng *rng;
    gsl_rng_env_setup();
    T = gsl_rng_default;
    rng = gsl_rng_alloc(T);

    // Initialize the atmospheric model
    atm_model atm_model = init_atm(&run_params, rng);

    // Get the atmospheric conditions at sea level
    atm_cond atm_conditions = get_pert_atm_cond(0, &atm_model);

    // Check the altitude
    REQUIRE_EQ(atm_conditions.altitude, 0);

    // Check the density
    REQUIRE_EQ(atm_conditions.density, atm_model.sea_level_density);

    // Check the wind components
    REQUIRE_EQ(atm_conditions.meridional_wind, 0);
    REQUIRE_EQ(atm_conditions.zonal_wind, 0);
    REQUIRE_EQ(atm_conditions.vertical_wind, 0);

    // Get the atmospheric conditions at 10 km
    atm_conditions = get_pert_atm_cond(10000, &atm_model);

    // Check the altitude
    REQUIRE_EQ(atm_conditions.altitude, 10000);

    // Check the density
    REQUIRE_LT(atm_conditions.density, atm_model.sea_level_density);

    // Check the wind components
    REQUIRE_EQ(atm_conditions.meridional_wind, 0);
    REQUIRE_EQ(atm_conditions.zonal_wind, 0);
    REQUIRE_EQ(atm_conditions.vertical_wind, 0);

    // Get the atmospheric conditions at 100 km
    atm_conditions = get_pert_atm_cond(100000, &atm_model);

    // Check the altitude
    REQUIRE_EQ(atm_conditions.altitude, 100000);

    // Check the density
    REQUIRE_LT(atm_conditions.density, atm_model.sea_level_density);

    // Check the wind components
    REQUIRE_EQ(atm_conditions.meridional_wind, 0);
    REQUIRE_EQ(atm_conditions.zonal_wind, 0);
    REQUIRE_EQ(atm_conditions.vertical_wind, 0);

    // Get the atmospheric conditions at 1000 km
    atm_conditions = get_pert_atm_cond(1000000, &atm_model);

    // Check the altitude
    REQUIRE_EQ(atm_conditions.altitude, 1000000);

    // Check the density
    REQUIRE_LT(atm_conditions.density, atm_model.sea_level_density);

    // Repeat the test with perturbation flag enabled
    run_params.atm_error = 1;

    // Initialize the atmospheric model
    atm_model = init_atm(&run_params, rng);

    // Get the atmospheric conditions at sea level
    atm_conditions = get_pert_atm_cond(0, &atm_model);

    // Check the altitude
    REQUIRE_EQ(atm_conditions.altitude, 0);

    // Check the density
    REQUIRE_NE(atm_conditions.density, atm_model.sea_level_density);

    // Check the wind components
    REQUIRE_NE(atm_conditions.meridional_wind, 0);
    REQUIRE_NE(atm_conditions.zonal_wind, 0);
    REQUIRE_NE(atm_conditions.vertical_wind, 0);
    REQUIRE_NE(atm_conditions.meridional_wind, atm_conditions.zonal_wind);
    REQUIRE_NE(atm_conditions.meridional_wind, atm_conditions.vertical_wind);
    REQUIRE_NE(atm_model.std_winds[0], atm_conditions.meridional_wind);
    REQUIRE_NE(atm_model.std_winds[0], atm_conditions.zonal_wind);
    REQUIRE_NE(atm_model.std_vert_winds[0], atm_conditions.vertical_wind);

    // Get the atmospheric conditions at 10 km
    atm_conditions = get_pert_atm_cond(10000, &atm_model);

    // Check the altitude
    REQUIRE_EQ(atm_conditions.altitude, 10000);

    // Check the density
    REQUIRE_NE(atm_conditions.density, atm_model.sea_level_density);

    // Check the wind components
    REQUIRE_NE(atm_conditions.meridional_wind, 0);
    REQUIRE_NE(atm_conditions.zonal_wind, 0);
    REQUIRE_NE(atm_conditions.vertical_wind, 0);
    REQUIRE_NE(atm_conditions.meridional_wind, atm_conditions.zonal_wind);
    REQUIRE_NE(atm_conditions.meridional_wind, atm_conditions.vertical_wind);
    REQUIRE_NE(atm_model.std_winds[1], atm_conditions.meridional_wind);
    REQUIRE_NE(atm_model.std_winds[1], atm_conditions.zonal_wind);
    REQUIRE_NE(atm_model.std_vert_winds[1], atm_conditions.vertical_wind);

    // Get the atmospheric conditions at 100 km
    atm_conditions = get_pert_atm_cond(100000, &atm_model);

    // Check the altitude
    REQUIRE_EQ(atm_conditions.altitude, 100000);

    // Check the density
    REQUIRE_LT(atm_conditions.density, atm_model.sea_level_density);

    // Check the wind components
    REQUIRE_NE(atm_conditions.meridional_wind, 0);
    REQUIRE_NE(atm_conditions.zonal_wind, 0);
    REQUIRE_NE(atm_conditions.vertical_wind, 0);
    REQUIRE_NE(atm_conditions.meridional_wind, atm_conditions.zonal_wind);
    REQUIRE_NE(atm_conditions.meridional_wind, atm_conditions.vertical_wind);
    REQUIRE_NE(atm_model.std_winds[2], atm_conditions.meridional_wind);
    REQUIRE_NE(atm_model.std_winds[2], atm_conditions.zonal_wind);
    REQUIRE_NE(atm_model.std_vert_winds[2], atm_conditions.vertical_wind);

}

TEST(atmosphere, get_atm_cond){
    runparams run_params;
    run_params.atm_model = 0;
    run_params.atm_error = 0;

    // Initialize the random number generator
    const gsl_rng_type *T;
    gsl_rng *rng;
    gsl_rng_env_setup();
    T = gsl_rng_default;
    rng = gsl_rng_alloc(T);

    // Initialize the atmospheric model
    atm_model atm_model = init_atm(&run_params, rng);

    // Get the atmospheric conditions at sea level
    atm_cond atm_conditions = get_atm_cond(0, &atm_model, &run_params);

    // Check that the wind components are zero
    REQUIRE_EQ(atm_conditions.meridional_wind, 0);
    REQUIRE_EQ(atm_conditions.zonal_wind, 0);
    REQUIRE_EQ(atm_conditions.vertical_wind, 0);

    // Repeat the test with perturbation flag enabled
    run_params.atm_error = 1;

    // Initialize the atmospheric model
    atm_model = init_atm(&run_params, rng);

    // Get the atmospheric conditions at sea level
    atm_conditions = get_atm_cond(0, &atm_model, &run_params);

    // Check that the wind components are not zero
    REQUIRE_NE(atm_conditions.meridional_wind, 0);
    REQUIRE_NE(atm_conditions.zonal_wind, 0);
    REQUIRE_NE(atm_conditions.vertical_wind, 0);
    
}