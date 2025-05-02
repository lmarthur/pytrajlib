#include <tau/tau.h>
#include "../src/include/trajectory.h"

TEST(trajectory, impact_linterp){
    runparams run_params;
    run_params.grav_error = 0;

    const gsl_rng_type *T;
    gsl_rng *rng;
    gsl_rng_env_setup();
    T = gsl_rng_default;
    rng = gsl_rng_alloc(T);

    grav grav = init_grav(&run_params, rng);
    state state_0;
    state state_1;
    state_0.t = 0;
    state_0.x = grav.earth_radius+1;
    state_0.y = 0;
    state_0.z = 0;
    state_0.vx = -2;
    state_0.vy = 0;
    state_0.vz = 0;

    state_1.t = 1;
    state_1.x = grav.earth_radius-1;
    state_1.y = 0;
    state_1.z = 0;
    state_1.vx = 0;
    state_1.vy = 0;
    state_1.vz = 0;

    state impact_state = impact_linterp(&state_0, &state_1);

    REQUIRE_EQ(impact_state.t, 0.5);
    REQUIRE_EQ(impact_state.x, grav.earth_radius);
    REQUIRE_EQ(impact_state.y, 0);
    REQUIRE_EQ(impact_state.z, 0);
    REQUIRE_EQ(impact_state.vx, -1);
    REQUIRE_EQ(impact_state.vy, 0);
    REQUIRE_EQ(impact_state.vz, 0);

}

TEST(trajectory, fly){
    // Initialize the random number generator
    const gsl_rng_type *T;
    gsl_rng *rng;
    gsl_rng_env_setup();
    T = gsl_rng_default;
    rng = gsl_rng_alloc(T);

    vehicle vehicle = init_mock_vehicle();
    runparams run_params;
    // Set the run parameters
    run_params.run_name = "test_run";
    run_params.run_type = 0;
    run_params.traj_output = 0;
    run_params.impact_output = 1;
    run_params.time_step_main = 1;
    run_params.time_step_reentry = 1;
    run_params.x_aim = 6371e3;
    run_params.y_aim = 0;
    run_params.z_aim = 0;
    run_params.theta_long = 0;
    run_params.theta_lat = 0;

    run_params.rv_type = 0;
    run_params.grav_error = 0;
    run_params.atm_error = 0;
    run_params.gnss_nav = 0;
    run_params.ins_nav = 1;
    run_params.rv_maneuv = 0;
    run_params.initial_x_error = 0;
    run_params.initial_pos_error = 0;
    run_params.initial_vel_error = 0;
    run_params.initial_angle_error = 0;
    run_params.acc_scale_stability = 0;
    run_params.gyro_bias_stability = 0;
    run_params.gyro_noise = 0;
    run_params.gnss_noise = 0;

    // print all of the vehicle parameters
    // printf("Booster total mass: %f\n", vehicle.booster.total_mass);
    // Mock vehicle with no thrust dropped from 10m above the surface
    state initial_state = init_true_state(&run_params, rng);
    initial_state.theta_long = 0;
    initial_state.x += 10;
    
    state final_state = fly(&run_params, &initial_state, &vehicle, rng);

    REQUIRE_LT(fabs(final_state.t - 1), 1);
    REQUIRE_EQ(final_state.ax_thrust, 0);
    REQUIRE_EQ(final_state.ay_thrust, 0);
    REQUIRE_EQ(final_state.az_thrust, 0);
    
    // Mock vehicle with no thrust launched from the surface
    initial_state = init_true_state(&run_params, rng);
    initial_state.theta_long = 0;
    initial_state.vx = 10;
    initial_state.vy = 10;
    initial_state.vz = 10;
    final_state = fly(&run_params, &initial_state, &vehicle, rng);

    REQUIRE_LT(fabs(final_state.t - 2), 1);

    // MMIII ballistic vehicle launched vertically from the surface
    vehicle = init_mmiii_ballistic();
    initial_state = init_true_state(&run_params, rng);
    initial_state.theta_long = 0;
    final_state = fly(&run_params, &initial_state, &vehicle, rng);

    REQUIRE_GT(final_state.t, 0);
    REQUIRE_LT(fabs(final_state.x - 6371e3), 1e-6);
    REQUIRE_LT(fabs(final_state.y), 1);
    REQUIRE_LT(fabs(final_state.z), 1);

    // MMIII ballistic vehicle launched along the equator
    initial_state = init_true_state(&run_params, rng);
    initial_state.theta_long = M_PI/4;
    run_params.traj_output = 0;

    final_state = fly(&run_params, &initial_state, &vehicle, rng);

    REQUIRE_GT(final_state.t, 0);
    REQUIRE_LT(fabs(sqrt(final_state.x*final_state.x + final_state.y*final_state.y) - 6371e3), 1);

}

TEST(trajectory, update_aimpoint){
    // Set the run parameters
    runparams run_params;
    run_params.run_type = 0;
    run_params.traj_output = 0;
    run_params.impact_output = 1;
    run_params.time_step_main = 1;
    run_params.time_step_reentry = 1;
    run_params.x_aim = 6371e3;
    run_params.y_aim = 0;
    run_params.z_aim = 0;
    run_params.theta_long = 0;
    run_params.theta_lat = 0;

    run_params.grav_error = 0;
    run_params.atm_error = 0;
    run_params.gnss_nav = 0;
    run_params.ins_nav = 0;

    run_params.rv_type = 0;

    run_params.initial_x_error = 0;
    run_params.initial_pos_error = 1;
    run_params.initial_vel_error = 0;
    run_params.initial_angle_error = 0;
    run_params.acc_scale_stability = 0;
    run_params.gyro_bias_stability = 0;
    run_params.gyro_noise = 0;
    run_params.gnss_noise = 0;

    cart_vector aimpoint = update_aimpoint(run_params);
    // printf("Aimpoint: %f, %f, %f\n", aimpoint.x, aimpoint.y, aimpoint.z);
    REQUIRE_LT(fabs(get_altitude(aimpoint.x, aimpoint.y, aimpoint.z)), 1);
    REQUIRE_EQ(run_params.initial_pos_error, 1);
}