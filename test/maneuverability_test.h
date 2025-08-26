#include <tau/tau.h>
#include "../src/include/maneuverability.h"

TEST(maneuverability, instant_maneuv){
    // Get the true state and the aimpoint
    runparams run_params;
    run_params.x_aim = 6371e3;
    run_params.y_aim = 0;
    run_params.z_aim = 0;
    run_params.nav_gain = 5;

    state true_state;
    true_state.x = 6371e3 + 10;
    true_state.y = 0;
    true_state.z = 0;
    true_state.vx = -1;
    true_state.vy = 1;
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

    state estimated_state = true_state;

    // Get the commanded acceleration vector
    cart_vector a_command;
    a_command = prop_nav(&run_params, &estimated_state);

    // Get the updated state
    true_state = instant_maneuv(&true_state, &a_command);

    // Verify that the updated state is correct
    REQUIRE_EQ(true_state.ax_lift, a_command.x);
    REQUIRE_EQ(true_state.ay_lift, a_command.y);
    REQUIRE_EQ(true_state.az_lift, a_command.z);
    REQUIRE_EQ(true_state.ax_total, true_state.ax_grav + true_state.ax_drag + true_state.ax_lift + true_state.ax_thrust);
    REQUIRE_EQ(true_state.ay_total, true_state.ay_grav + true_state.ay_drag + true_state.ay_lift + true_state.ay_thrust);
    REQUIRE_EQ(true_state.az_total, true_state.az_grav + true_state.az_drag + true_state.az_lift + true_state.az_thrust);

}

TEST(maneuverability, perfect_maneuv){
    // initialize the true state
    state true_state;
    true_state.x = 6371e3 + 10;
    true_state.y = 0;
    true_state.z = 0;
    true_state.theta_lat = 0;
    true_state.theta_long = 0;
    true_state.vx = -1;
    true_state.vy = 1;
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
    true_state.ax_total = 0;
    true_state.ay_total = 0;
    true_state.az_total = 0;


    state estimated_state = true_state;
    state desired_state = true_state;

    // For desired = estimated, the true state should not change
    state new_true_state = perfect_maneuv(&true_state, &estimated_state, &desired_state);

    REQUIRE_EQ(new_true_state.x, true_state.x);
    REQUIRE_EQ(new_true_state.y, true_state.y);
    REQUIRE_EQ(new_true_state.z, true_state.z);
    REQUIRE_EQ(new_true_state.vx, true_state.vx);
    REQUIRE_EQ(new_true_state.vy, true_state.vy);
    REQUIRE_EQ(new_true_state.vz, true_state.vz);
    REQUIRE_EQ(new_true_state.ax_grav, true_state.ax_grav);
    REQUIRE_EQ(new_true_state.ay_grav, true_state.ay_grav);
    REQUIRE_EQ(new_true_state.az_grav, true_state.az_grav);
    REQUIRE_EQ(new_true_state.ax_drag, true_state.ax_drag);
    REQUIRE_EQ(new_true_state.ay_drag, true_state.ay_drag);
    REQUIRE_EQ(new_true_state.az_drag, true_state.az_drag);
    REQUIRE_EQ(new_true_state.ax_lift, true_state.ax_lift);
    REQUIRE_EQ(new_true_state.ay_lift, true_state.ay_lift);
    REQUIRE_EQ(new_true_state.az_lift, true_state.az_lift);
    REQUIRE_EQ(new_true_state.ax_thrust, true_state.ax_thrust);
    REQUIRE_EQ(new_true_state.ay_thrust, true_state.ay_thrust);
    REQUIRE_EQ(new_true_state.az_thrust, true_state.az_thrust);
    REQUIRE_EQ(new_true_state.ax_total, true_state.ax_total);
    REQUIRE_EQ(new_true_state.ay_total, true_state.ay_total);
    REQUIRE_EQ(new_true_state.az_total, true_state.az_total);

    // For a drag acceleration offset, the true state should change
    desired_state.ax_drag = 1;
    desired_state.ay_drag = 1;
    desired_state.az_drag = 1;
    desired_state.ax_total = desired_state.ax_grav + desired_state.ax_drag + desired_state.ax_lift + desired_state.ax_thrust;
    desired_state.ay_total = desired_state.ay_grav + desired_state.ay_drag + desired_state.ay_lift + desired_state.ay_thrust;
    desired_state.az_total = desired_state.az_grav + desired_state.az_drag + desired_state.az_lift + desired_state.az_thrust;
    
    new_true_state = perfect_maneuv(&true_state, &estimated_state, &desired_state);

    REQUIRE_NE(new_true_state.ax_total, true_state.ax_total);
    REQUIRE_NE(new_true_state.ay_total, true_state.ay_total);
    REQUIRE_NE(new_true_state.az_total, true_state.az_total);



}

TEST(maneuverability, rv_time_constant){
    // Initialize the state
    state true_state;
    true_state.x = 6371e3 + 10;
    true_state.y = 0;
    true_state.z = 0;
    true_state.theta_lat = 0;
    true_state.vx = -1;
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

    // Get the time constant
    double time_constant_0 = rv_time_constant(&vehicle, &true_state, &atm_cond);

    // Verify that the time constant is correct
    REQUIRE_GT(time_constant_0, 0);

    // For a different state, the time constant should be different
    true_state.vx = -10;
    double time_constant_1 = rv_time_constant(&vehicle, &true_state, &atm_cond);

    REQUIRE_GT(time_constant_1, 0);
    REQUIRE_NE(time_constant_0, time_constant_1);

}

TEST(maneuverability, reentry_lift){
    // Initialize the state
    state true_state;
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

    // Initialize the run parameters
    runparams run_params;
    run_params.deflection_time = 0.1; // Time to deflect the lift vector (seconds)
    run_params.actuator_force = 10; // Maximum actuator force (kN)
    run_params.gearing_ratio = 1; // Gearing ratio of the control surfaces
    run_params.x_aim = 6371e3; // Target x-coordinate in meters
    run_params.y_aim = 0; // Target y-coordinate in meters
    run_params.z_aim = 0; // Target z-coordinate in meters
    run_params.nav_gain = 5; // Navigation gain for proportional navigation guidance

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
    reentry_lift_drag(&run_params, &true_state, &a_command, &atm_cond, &vehicle, 0.1);

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

    reentry_lift_drag(&run_params, &true_state, &a_command, &atm_cond, &vehicle, 0.1);
    
    // Verify that the lift is updated correctly
    REQUIRE_LT(true_state.ax_lift, 1);
    REQUIRE_LT(true_state.ay_lift, 1);
    REQUIRE_LT(true_state.az_lift, 1);


    // Get command from proportional navigation
    true_state.x = 6371e3 + 1000;
    true_state.y = 0;
    true_state.z = 0;
    true_state.theta_lat = 0;
    true_state.vx = -100;
    true_state.vy = 100;
    true_state.vz = 0;
    true_state.ax_lift = 0;
    true_state.ay_lift = 0;
    true_state.az_lift = 0;

    a_command = prop_nav(&run_params, &true_state);
    // printf("Commanded acceleration: (%f, %f, %f)\n", a_command.x, a_command.y, a_command.z);
    // Update the lift with the commanded acceleration
    reentry_lift_drag(&run_params, &true_state, &a_command, &atm_cond, &vehicle, 0.1);
    // printf("Updated lift: (%f, %f, %f)\n", true_state.ax_lift, true_state.ay_lift, true_state.az_lift);
    double a_dot_v = a_command.x * true_state.vx + a_command.y * true_state.vy + a_command.z * true_state.vz;
    double a_com_dot_a_lift = a_command.x * true_state.ax_lift + a_command.y * true_state.ay_lift + a_command.z * true_state.az_lift;
    double a_lift_norm = sqrt(true_state.ax_lift * true_state.ax_lift + true_state.ay_lift * true_state.ay_lift + true_state.az_lift * true_state.az_lift);
    double a_command_norm = sqrt(a_command.x * a_command.x + a_command.y * a_command.y + a_command.z * a_command.z);
    double a_com_dot_a_lift_norm = a_com_dot_a_lift / (a_lift_norm * a_command_norm);

    REQUIRE_LT(fabs(a_com_dot_a_lift_norm - 1.0), 1e-5);
    REQUIRE_LT(fabs(a_dot_v), 1e-6);

}

TEST(maneuverability, reentry_drag){
    vehicle vehicle;
    vehicle.rv = init_swerve_rv();
    vehicle.booster = init_mmiii_booster();
    vehicle.total_mass = vehicle.booster.total_mass + vehicle.rv.rv_mass;
    vehicle.current_mass = vehicle.total_mass;
    atm_cond atm_cond;
    state state;
    runparams run_params;
    run_params.deflection_time = 0.1; // Time to deflect the lift vector (seconds)
    run_params.actuator_force = 10; // Maximum actuator force (kN)
    run_params.gearing_ratio = 1; // Gearing ratio of the control surfaces
    run_params.x_aim = 6371e3; // Target x-coordinate in meters
    run_params.y_aim = 0; // Target y-coordinate in meters
    run_params.z_aim = 0; // Target z-coordinate in meters
    run_params.nav_gain = 5; // Navigation gain for proportional navigation guidance
    run_params.grav_error = 0;
    run_params.run_type = 0;
    run_params.cl_pert = 0; // Set to zero for this test, as we are only testing drag

    // Initialize the acceleration command
    cart_vector a_command;
    a_command.x = 0;
    a_command.y = 0;
    a_command.z = 0;

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
    state.ax_grav = 0;
    state.ay_grav = 0;
    state.az_grav = 0;
    state.ax_drag = 0;
    state.ay_drag = 0;
    state.az_drag = 0;
    state.ax_lift = 0;
    state.ay_lift = 0;
    state.az_lift = 0;
    state.ax_thrust = 0;
    state.ay_thrust = 0;
    state.az_thrust = 0;

    reentry_lift_drag(&run_params, &state, &a_command, &atm_cond, &vehicle, 0.1);

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
    state.ax_drag = 0;
    state.ay_drag = 0;
    state.az_drag = 0;
    state.ax_lift = 0;
    state.ay_lift = 0;
    state.az_lift = 0;
    
    reentry_lift_drag(&run_params, &state, &a_command, &atm_cond, &vehicle, 0.1);

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
    state.ax_drag = 0;
    state.ay_drag = 0;
    state.az_drag = 0;
    state.ax_lift = 0;
    state.ay_lift = 0;
    state.az_lift = 0;

    reentry_lift_drag(&run_params, &state, &a_command, &atm_cond, &vehicle, 0.1);

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
    state.ax_drag = 0;
    state.ay_drag = 0;
    state.az_drag = 0;
    state.ax_lift = 0;
    state.ay_lift = 0;
    state.az_lift = 0;

    reentry_lift_drag(&run_params, &state, &a_command, &atm_cond, &vehicle, 0.1);

    REQUIRE_LT(state.ax_drag, 0);
    REQUIRE_EQ(state.ay_drag, 0);
    REQUIRE_EQ(state.az_drag, 0);

}