#include <tau/tau.h>
#include "../src/include/trajectory.h"

TEST(trajectory, impact_linterp){
    grav grav = init_grav();
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
    // TODO: clean up this test to amend deprecated vehicle initialization
    vehicle vehicle = init_mock_vehicle();
    runparams run_params;
    // Set the run parameters
    run_params.rv_type = 0;
    run_params.traj_output = 0;
    run_params.time_step = 1;
    // Mock vehicle with no thrust dropped from 10m above the surface
    state initial_state = init_state();
    initial_state.theta_long = 0;
    initial_state.x += 10;
    state final_state = fly(&run_params, &initial_state, &vehicle.booster, &vehicle.rv);
    
    REQUIRE_LT(fabs(final_state.t - 1), 1);
    REQUIRE_LT(fabs(final_state.x - 6371e3), 1e-6);
    REQUIRE_EQ(final_state.ax_thrust, 0);
    REQUIRE_EQ(final_state.ay_thrust, 0);
    REQUIRE_EQ(final_state.az_thrust, 0);
    

    // Mock vehicle with no thrust launched from the surface
    initial_state = init_state();
    initial_state.theta_long = 0;
    initial_state.vx = 10;
    initial_state.vy = 10;
    initial_state.vz = 10;
    final_state = fly(&run_params, &initial_state, &vehicle.booster, &vehicle.rv);

    REQUIRE_LT(fabs(final_state.t - 2), 1);

    // MMIII ballistic vehicle launched vertically from the surface
    vehicle = init_mmiii_ballistic();
    initial_state = init_state();
    initial_state.theta_long = 0;
    final_state = fly(&run_params, &initial_state, &vehicle.booster, &vehicle.rv);

    REQUIRE_GT(final_state.t, 0);
    REQUIRE_LT(fabs(final_state.x - 6371e3), 1e-6);
    REQUIRE_LT(fabs(final_state.y), 1);
    REQUIRE_LT(fabs(final_state.z), 1);

    // MMIII ballistic vehicle launched along the equator
    initial_state = init_state();
    initial_state.theta_long = M_PI/4;
    run_params.traj_output = 1;

    final_state = fly(&run_params, &initial_state, &vehicle.booster, &vehicle.rv);

    REQUIRE_GT(final_state.t, 0);
    REQUIRE_LT(fabs(sqrt(final_state.x*final_state.x + final_state.y*final_state.y) - 6371e3), 1);
    REQUIRE_NE(final_state.ax_total, 0);
    REQUIRE_NE(final_state.ay_total, 0);

}