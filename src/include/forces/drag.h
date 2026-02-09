#ifndef DRAG_H
#define DRAG_H


#include <math.h>
#include "../vehicle.h"
#include "../atmosphere.h"
#include "../utils.h"
#include "../math/linalg.h"

static const double AOA_MAX = 10; // Maximum angle of attack is 10 degrees

/**
 * Helper function to get wind at current location in standard cartesian basis
 */
cart_vector get_cart_wind(state *state, atm_cond *atm_cond) {
    double cart_wind[3];
    double spher_wind[3] = {atm_cond->vertical_wind, atm_cond->zonal_wind, atm_cond->meridional_wind};
    double spher_coords[3];
    double cart_coords[3] = {state->x, state->y, state->z};
    cartcoords_to_sphercoords(cart_coords, spher_coords);

    sphervec_to_cartvec(spher_wind, cart_wind, spher_coords);

    cart_vector cartvec_wind;
    cartvec_wind.x = cart_wind[0];
    cartvec_wind.y = cart_wind[1];
    cartvec_wind.z = cart_wind[2];

    return cartvec_wind;

}

double get_max_a_exec(runparams run_params, vehicle veh) {
    double max_flap_force =
        run_params.actuator_force * run_params.gearing_ratio * 1000;
    double max_lift_force =
        (veh.rv.c_l_alpha * max_flap_force * (veh.rv.x_flap - veh.rv.x_com) /
         (veh.rv.c_m_alpha *
          veh.rv.rv_length)); // maximum lift force in N, based on moment arm
                              // and lift properties
    double max_a_exec =
        (max_lift_force / veh.rv.rv_mass); // maximum acceleration that can be
                                           // executed by the flaps in m/s^2
    return max_a_exec;
}


/**
 * Get the drag acceleration based on the drag coefficient
 * and the characteristic area.
 */
cart_vector get_drag_acceleration_generic(double t, state current_state,
                                      atm_cond *atm_cond, vehicle *vehicle, double c_d,
                                      double area) {
    cart_vector velocity;
    velocity.x = current_state.vx;
    velocity.y = current_state.vy;                               
    velocity.z = current_state.vz;                               

    cart_vector wind_vec = get_cart_wind(&current_state, atm_cond);
    cart_vector v_rel = subtract(velocity, wind_vec);
    double v_rel_mag = norm(v_rel);

    if (v_rel_mag < 1e-2){
        return zeros();
    }

    double a_drag_mag =
        0.5 * atm_cond->density * v_rel_mag * v_rel_mag * area * c_d / vehicle->current_mass;

    cart_vector drag = smultiply(v_rel, -a_drag_mag / v_rel_mag);

    return drag;

}

void update_drag(runparams *run_params, vehicle *vehicle, atm_cond *atm_cond, state *state, double *step_timer){
    /*
    Updates the drag acceleration components

    INPUTS:
    ----------
        vehicle: vehicle *
            pointer to the vehicle struct
        atm_cond: atm_cond *
            pointer to the atmospheric conditions
        state: state *
            pointer to the state struct
    */    

    cart_vector drag;
    // Calculate the drag acceleration components for a booster
    if (state->t < vehicle->booster.total_burn_time){
        drag = get_drag_acceleration_generic(state->t, *state, atm_cond, vehicle, vehicle->booster.c_d_0, vehicle->booster.area);
    }
    // Calculate drag acceleration for realistic maneuvering vehicle
    else if (run_params->rv_maneuv == 1){
        cart_vector lift;
        lift.x = state->ax_lift;
        lift.y = state->ay_lift;
        lift.z = state->az_lift;
        double lift_magnitude = norm(lift);
        double max_a_exec = get_max_a_exec(*run_params, *vehicle);
        double aoa = lift_magnitude * (AOA_MAX / max_a_exec * M_PI / 180);
        double c_d = vehicle->rv.c_d_0 + fabs(vehicle->rv.c_d_alpha * aoa);
        drag = get_drag_acceleration_generic(state->t, *state, atm_cond, vehicle, c_d, vehicle->rv.rv_area);
    }
    // Calculate the drag acceleration components for a ballistic or perfectly maneuvering reentry vehicle
    else{
        cart_vector winds_cart = get_cart_wind(state, atm_cond);
        double wind_mag = norm(winds_cart);
        cart_vector velocity;
        velocity.x = state->vx;
        velocity.y = state->vy;
        velocity.z = state->vz;

        double v_mag = norm(velocity);

        // Angle of attack (assuming vehicle oriented along velocity vector)
        double aoa = atan(wind_mag / v_mag);

        // Drag coefficient varies with angle of attack
        double c_d = vehicle->rv.c_d_0 + fabs(vehicle->rv.c_d_alpha * aoa);
        drag = get_drag_acceleration_generic(state->t, *state, atm_cond, vehicle, c_d, vehicle->rv.rv_area);
    }
    state->ax_drag = drag.x;
    state->ay_drag = drag.y;
    state->az_drag = drag.z;

    return;
}


#endif