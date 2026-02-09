#include <tau/tau.h>
#include "../../src/include/forces/gravity.h"

TEST(gravity, update_gravity){
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
