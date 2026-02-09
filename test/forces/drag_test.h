#include <tau/tau.h>

TEST(drag, update_drag){
    vehicle vehicle;
    vehicle.rv = init_ballistic_rv();
    vehicle.booster = init_mmiii_booster();
    vehicle.total_mass = vehicle.booster.total_mass + vehicle.rv.rv_mass;
    vehicle.current_mass = vehicle.total_mass;
    atm_cond atm_cond;
    state state;
    runparams run_params;
    run_params.grav_error = 0;
    run_params.run_type = 0;
    run_params.cl_pert = 0; // Set to zero for this test, as we are only testing drag

    // Initialize the random number generator
    const gsl_rng_type *T;
    gsl_rng *rng;
    gsl_rng_env_setup();
    T = gsl_rng_default;
    rng = gsl_rng_alloc(T);

    grav grav = init_grav(&run_params, rng);
    atm_model atm_model = init_exp_atm(&run_params, rng);

    // Step function anomaly timer (unused in this test, but required for the function signature)
    double step_timer = 0; // Timer for the step function
    
    atm_cond = get_exp_atm_cond(0, &atm_model);

    state.t = 1;
    state.x = grav.earth_radius;
    state.y = 0;
    state.z = 0;
    state.vx = 0;
    state.vy = 0;
    state.vz = 0;

    update_drag(&run_params, &vehicle, &atm_cond, &state, &step_timer);

    // Check that the drag acceleration components are zero
    REQUIRE_LT(state.ax_drag, 1e-6);
    REQUIRE_LT(state.ay_drag, 1e-6);
    REQUIRE_LT(state.az_drag, 1e-6);

    // Check that for wind and velocity in the same direction, drag is zero
    atm_cond.vertical_wind = 1;
    atm_cond.zonal_wind = 1;
    atm_cond.meridional_wind = 1;
    state.vx = 1;
    state.vy = 1;
    state.vz = 1;
    
    update_drag(&run_params, &vehicle, &atm_cond, &state, &step_timer);

    REQUIRE_LT(state.ax_drag, 1e-6);
    REQUIRE_LT(state.ay_drag, 1e-6);
    REQUIRE_LT(state.az_drag, 1e-6);

    // Check that for wind and velocity in opposite directions, drag is non-zero
    atm_cond.vertical_wind = -1;
    atm_cond.zonal_wind = -1;
    atm_cond.meridional_wind = -1;
    state.vx = 1;
    state.vy = 1;
    state.vz = 1;

    update_drag(&run_params, &vehicle, &atm_cond, &state, &step_timer);

    REQUIRE_NE(state.ax_drag, 0);
    REQUIRE_NE(state.ay_drag, 0);
    REQUIRE_NE(state.az_drag, 0);

    // Check that for no wind, drag is only in the opposite direction of velocity
    atm_cond.vertical_wind = 0;
    atm_cond.zonal_wind = 0;
    atm_cond.meridional_wind = 0;

    state.vx = 100;
    state.vy = 0;
    state.vz = 0;

    update_drag(&run_params, &vehicle, &atm_cond, &state, &step_timer);
    
    REQUIRE_LT(state.ax_drag, 0);
    REQUIRE_EQ(state.ay_drag, 0);
    REQUIRE_EQ(state.az_drag, 0);
    
}
