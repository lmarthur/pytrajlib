#ifndef DRAG_H
#define DRAG_H

#include <math.h>

#include "../math/linalg.h"
#include "../models/atmosphere.h"
#include "../models/vehicle.h"
#include "../utils.h"

static const double AOA_MAX = 10; // Maximum angle of attack is 10 degrees

/**
 * Get the drag acceleration based on the drag coefficient
 * and the characteristic area.
 */
cart_vector get_drag_acceleration_generic(double t, state current_state,
                                          atm_cond *atm_cond, vehicle *vehicle,
                                          double c_d, double area) {
  cart_vector velocity;
  velocity.x = current_state.vx;
  velocity.y = current_state.vy;
  velocity.z = current_state.vz;

  cart_vector wind_vec = get_cart_wind(&current_state, atm_cond);
  cart_vector v_rel = subtract(velocity, wind_vec);
  double v_rel_mag = norm(v_rel);

  if (v_rel_mag < 1e-2) {
    return zeros();
  }

  double a_drag_mag = 0.5 * atm_cond->density * v_rel_mag * v_rel_mag * area *
                      c_d / vehicle->current_mass;

  cart_vector drag = smultiply(v_rel, -a_drag_mag / v_rel_mag);

  return drag;
}

void update_drag(runparams *run_params, vehicle *vehicle, atm_cond *atm_cond,
                 state *state, double *step_timer) {
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
  // Calculate drag acceleration for realistic maneuvering vehicle
  if (run_params->rv_maneuv == 1) {
    cart_vector lift;
    lift.x = state->ax_lift;
    lift.y = state->ay_lift;
    lift.z = state->az_lift;
    double lift_magnitude = norm(lift);
    double max_a_exec = get_max_a_exec(run_params, vehicle);
    double aoa = lift_magnitude * (AOA_MAX / max_a_exec * M_PI / 180);
    double c_d = vehicle->rv.c_d_0 + fabs(vehicle->rv.c_d_alpha * aoa);
    drag = get_drag_acceleration_generic(state->t, *state, atm_cond, vehicle,
                                         c_d, vehicle->rv.rv_area);
  }
  // Calculate the drag acceleration components for boost phase & reentry of
  // ballistic or perfectly maneuvering reentry vehicle
  else {
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

    if (state->t < vehicle->booster.total_burn_time) {
      drag = get_drag_acceleration_generic(state->t, *state, atm_cond, vehicle,
                                           c_d, vehicle->booster.area);
    } else {
      drag = get_drag_acceleration_generic(state->t, *state, atm_cond, vehicle,
                                           c_d, vehicle->rv.rv_area);
    }
  }
  state->ax_drag = drag.x;
  state->ay_drag = drag.y;
  state->az_drag = drag.z;

  return;
}

#endif