#include <tau/tau.h>
#include "../../src/include/forces/lift.h"

TEST(lift, update_lift){
    // Initialize the state
    state true_state;
    true_state.t = 1000;
    true_state.x = 6371e3 + 10;
    true_state.y = 0;
    true_state.z = 0;
    true_state.theta_lat = 0;
    true_state.vx = -100;
    true_state.vy = 0;
    true_state.vz = 0;
    true_state.ax_grav = 0;
    true_state.ay_grav = 0;
    true_state.az_grav = 0;
    true_state.ax_drag = 0;
    true_state.ay_drag = 0;
    true_state.az_drag = 0;
    true_state.ax_lift = 0;
    true_state.ay_lift = 0;
    true_state.az_lift = 0;
    true_state.ax_thrust = 0;
    true_state.ay_thrust = 0;
    true_state.az_thrust = 0;
    true_state.ax_lift_avail = 0;
    true_state.ay_lift_avail = 0;
    true_state.az_lift_avail = 0;

    state est_state = true_state;

    // Initialize the run parameters
    runparams run_params;
    run_params.deflection_time = 0.1; // Time to deflect the lift vector (seconds)
    run_params.actuator_force = 100;
    // Initialize the vehicle
    vehicle vehicle;
    vehicle.rv = init_swerve_rv();
    vehicle.booster = init_mmiii_booster();
    vehicle.total_mass = vehicle.booster.total_mass + vehicle.rv.rv_mass;
    vehicle.current_mass = vehicle.total_mass;

    // Initialize the atmospheric conditions
    atm_cond atm_cond;
    atm_cond.altitude = 0;
    atm_cond.density = 1.225;
    atm_cond.meridional_wind = 0;
    atm_cond.zonal_wind = 0;
    atm_cond.vertical_wind = 0;

    // Initialize the acceleration command
    cart_vector a_command;
    a_command.x = 0;
    a_command.y = 0;
    a_command.z = 0;

    // Update the lift
    update_lift(&true_state, &est_state, &run_params, &atm_cond, &atm_cond, &vehicle, 0.1);
    // Verify that the lift is unchanged when the command is zero and the current lift is zero
    REQUIRE_EQ(true_state.ax_lift, 0);
    REQUIRE_EQ(true_state.ay_lift, 0);
    REQUIRE_EQ(true_state.az_lift, 0);

    // Verify that the executed acceleration decays exponentially to the command
    a_command.x = 0;
    a_command.y = 0;
    a_command.z = 0;

    true_state.ax_lift = 1;
    true_state.ay_lift = 1;
    true_state.az_lift = 1;

    update_lift(&true_state, &est_state, &run_params, &atm_cond, &atm_cond, &vehicle, 0.1);
    update_lift(&true_state, &est_state, &run_params, &atm_cond, &atm_cond, &vehicle, 0.1);
    // Verify that the lift is updated correctly
    REQUIRE_LT(true_state.ax_lift, 1);
    REQUIRE_LT(true_state.ay_lift, 1);
    REQUIRE_LT(true_state.az_lift, 1);

}