#include <tau/tau.h>
#include "../../src/include/forces/thrust.h"

TEST(thrust, update_thrust){
    vehicle vehicle;
    vehicle.rv = init_ballistic_rv();
    vehicle.booster = init_mmiii_booster();
    vehicle.total_mass = vehicle.booster.total_mass + vehicle.rv.rv_mass;
    vehicle.current_mass = vehicle.total_mass;
    state state;

    state.t = 0;
    state.theta_long = 0;
    state.theta_lat = 0;
    state.vx = 0;
    state.vy = 0;
    state.vz = 0;

    update_thrust(&vehicle, &state);

    // Check that the thrust acceleration components are along the x-axis
    REQUIRE_GT(state.ax_thrust, 0);
    REQUIRE_EQ(state.ay_thrust, 0);
    REQUIRE_EQ(state.az_thrust, 0);

    state.t = 1;
    state.theta_long = 0;
    state.theta_lat = 0;
    state.vx = 0;
    state.vy = 0;
    state.vz = 0;

    update_thrust(&vehicle, &state);

    // Check that the thrust acceleration components are along the x-axis
    REQUIRE_GT(state.ax_thrust, 0);
    REQUIRE_EQ(state.ay_thrust, 0);
    REQUIRE_EQ(state.az_thrust, 0);

    // Check that the thrust acceleration components are zero after the burn time
    state.t = vehicle.booster.total_burn_time + 1;

    update_thrust(&vehicle, &state);

    REQUIRE_EQ(state.ax_thrust, 0);
    REQUIRE_EQ(state.ay_thrust, 0);
    REQUIRE_EQ(state.az_thrust, 0);

    // Check that the thrust acceleration at time t + 1 is greater than at time t
    double ax_thrust_0 = state.ax_thrust;
    state.t = 2;
    update_mass(&vehicle, state.t);

    update_thrust(&vehicle, &state);

    REQUIRE_GT(state.ax_thrust, ax_thrust_0);

    state.t = 0;
    state.vx = 1;
    state.vy = 0;
    state.vz = 0;
    vehicle.current_mass = vehicle.total_mass;

    // Perform a full booster burn
    for (int i = 0; i <= vehicle.booster.total_burn_time + 1; i++){
        state.t = i;
        update_mass(&vehicle, state.t);
        update_thrust(&vehicle, &state);

        // Check that the thrust acceleration components are zero after the burn time
        if (state.t > vehicle.booster.total_burn_time){
            REQUIRE_EQ(state.ax_thrust, 0);
            REQUIRE_EQ(state.ay_thrust, 0);
            REQUIRE_EQ(state.az_thrust, 0);
        }

        // Check that the thrust acceleration components do not exceed 10^3 m/s^2
        REQUIRE_LT(state.ax_thrust, 1e3);
        REQUIRE_LT(state.ay_thrust, 1e3);
        REQUIRE_LT(state.az_thrust, 1e3);
    }

}
