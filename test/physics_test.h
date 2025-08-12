#include <tau/tau.h>
#include "../src/include/physics.h"

TEST(physics, update_mass){
    // Define a state struct
    state state;
    // Define a vehicle struct
    vehicle vehicle;

    // Initialize the vehicle
    vehicle.booster = init_mmiii_booster();
    vehicle.rv = init_ballistic_rv();
    double time_step = 1;

    vehicle.total_mass = vehicle.booster.total_mass + vehicle.rv.rv_mass;
    vehicle.current_mass = vehicle.total_mass;
    state.t = 0;
    
    // Loop through the burn time of the booster
    for (int i = 0; i <= vehicle.booster.total_burn_time + time_step; i++){
        state.t = i * time_step;

        update_mass(&vehicle, state.t);

        // Check that mass is always positive
        REQUIRE_GT(vehicle.current_mass, 0);
        
        // First time step
        if (state.t == time_step){
            REQUIRE_EQ(vehicle.current_mass, vehicle.total_mass - time_step * vehicle.booster.fuel_burn_rate[0]);
        }

        // First stage separation
        if (state.t == vehicle.booster.burn_time[0] + time_step){
            REQUIRE_EQ(vehicle.current_mass, vehicle.total_mass - vehicle.booster.wet_mass[0] - time_step*vehicle.booster.fuel_burn_rate[1]);
        }

        // Second stage separation
        if (state.t == vehicle.booster.burn_time[0] + vehicle.booster.burn_time[1] + time_step){
            REQUIRE_EQ(vehicle.current_mass, vehicle.total_mass - vehicle.booster.wet_mass[0] - vehicle.booster.wet_mass[1] - time_step*vehicle.booster.fuel_burn_rate[2]);
        }

        // After separation
        if (state.t == vehicle.booster.total_burn_time + time_step){
            REQUIRE_EQ(vehicle.current_mass, vehicle.rv.rv_mass);
        }
    }

}

TEST(physics, update_gravity){
    // Define a grav struct
    grav grav;
    // Define a state struct
    state state;
    runparams run_params;
    run_params.grav_error = 0;

    // Initialize the random number generator
    const gsl_rng_type *T;
    gsl_rng *rng;
    gsl_rng_env_setup();
    T = gsl_rng_default;
    rng = gsl_rng_alloc(T);

    // Initialize the grav struct
    grav = init_grav(&run_params, rng);

    // Initialize the state struct with the vehicle at one earth radius
    state.x = grav.earth_radius;
    state.y = 0;
    state.z = 0;

    // Update the gravity acceleration components
    update_gravity(&grav, &state);

    // Check that the gravitational acceleration components are correct
    REQUIRE_LT(state.ax_grav + 9.81, 0.01);
    REQUIRE_EQ(state.ay_grav, 0);
    REQUIRE_EQ(state.az_grav, 0);

    // Move the vehicle to a different location
    state.x = grav.earth_radius / sqrt(2);
    state.y = grav.earth_radius / sqrt(2);
    state.z = 0;

    // Update the gravity acceleration components
    update_gravity(&grav, &state);
    
    // Check that the gravitational acceleration components are correct
    double r = sqrt(state.x*state.x + state.y*state.y + state.z*state.z);
    double ar_grav = -grav.grav_const * grav.earth_mass / (r*r);
    double ar_grav_surface = ar_grav;
    REQUIRE_EQ(state.ax_grav, ar_grav * state.x / r);
    REQUIRE_EQ(state.ay_grav, ar_grav * state.y / r);
    REQUIRE_EQ(state.az_grav, ar_grav * state.z / r);
    REQUIRE_EQ(ar_grav, -sqrt(state.ax_grav*state.ax_grav + state.ay_grav*state.ay_grav + state.az_grav*state.az_grav));

    // Move the vehicle to a different height
    state.x = grav.earth_radius / sqrt(2) + 1000;
    state.y = grav.earth_radius / sqrt(2) + 1000;
    state.z = 1000;

    // Update the gravity acceleration components
    update_gravity(&grav, &state);

    // Check that the gravitational acceleration components are correct
    r = sqrt(state.x*state.x + state.y*state.y + state.z*state.z);
    ar_grav = -grav.grav_const * grav.earth_mass / (r*r);
    REQUIRE_LT(state.ax_grav, 0);
    REQUIRE_LT(state.ay_grav, 0);
    REQUIRE_LT(state.az_grav, 0);
    REQUIRE_GT(ar_grav, ar_grav_surface);
    REQUIRE_EQ(ar_grav, -sqrt(state.ax_grav*state.ax_grav + state.ay_grav*state.ay_grav + state.az_grav*state.az_grav));
}

TEST(physics, update_thrust){
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

TEST(physics, rk4step){
    state state;
    state.x = 0;
    state.y = 0;
    state.z = 0;
    state.vx = 0;
    state.vy = 0;
    state.vz = 0;
    state.ax_total = 0;
    state.ay_total = 0;
    state.az_total = 0;


    double time_step = 1;

    rk4step(&state, time_step);

    // Check that the new position is zero
    REQUIRE_EQ(state.x, 0);
    REQUIRE_EQ(state.y, 0);
    REQUIRE_EQ(state.z, 0);

    // Check that the new velocity is zero
    REQUIRE_EQ(state.vx, 0);
    REQUIRE_EQ(state.vy, 0);
    REQUIRE_EQ(state.vz, 0);

    // Check that the new acceleration is zero
    REQUIRE_EQ(state.ax_total, 0);
    REQUIRE_EQ(state.ay_total, 0);
    REQUIRE_EQ(state.az_total, 0);

    // Check that the new position is the time step times the velocity
    state.x = 0;
    state.y = 0;
    state.z = 0;
    state.vx = 1;
    state.vy = 1;
    state.vz = 1;

    rk4step(&state, time_step);

    REQUIRE_EQ(state.x, time_step);
    REQUIRE_EQ(state.y, time_step);
    REQUIRE_EQ(state.z, time_step);

    // Check that the new velocity is the time step times the acceleration
    state.vx = 0;
    state.vy = 0;
    state.vz = 0;
    state.ax_total = 1;
    state.ay_total = 1;
    state.az_total = 1;

    rk4step(&state, time_step);

    REQUIRE_EQ(state.vx, time_step);
    REQUIRE_EQ(state.vy, time_step);
    REQUIRE_EQ(state.vz, time_step);

    // Check that the new position is the time step times the velocity plus 0.5 times the acceleration
    state.x = 0;
    state.y = 0;
    state.z = 0;
    state.vx = 1;
    state.vy = 1;
    state.vz = 1;
    state.ax_total = 1;
    state.ay_total = 1;
    state.az_total = 1;

    rk4step(&state, time_step);

    REQUIRE_EQ(state.x, time_step + 0.5);
    REQUIRE_EQ(state.y, time_step + 0.5);
    REQUIRE_EQ(state.z, time_step + 0.5);

    // Check that the new velocity is the time step times the acceleration
    state.vx = 0;
    state.vy = 0;
    state.vz = 0;
    state.ax_total = 2;
    state.ay_total = 2;
    state.az_total = 2;

    rk4step(&state, time_step);

    REQUIRE_EQ(state.vx, time_step + 1);
    REQUIRE_EQ(state.vy, time_step + 1);
    REQUIRE_EQ(state.vz, time_step + 1);

}