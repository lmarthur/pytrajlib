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
cartvec get_drag_acceleration_generic(double t, state current_state,
                                      atm_cond *atm_cond, vehicle *vehicle,
                                      double c_d, double area) {
  cartvec wind_vec = get_cart_wind(&current_state, atm_cond);
  cartvec v_rel = subtract(current_state.velocity, wind_vec);
  double v_rel_mag = norm(v_rel);

  if (v_rel_mag < 1e-2) {
    return zeros();
  }

  double a_drag_mag = 0.5 * atm_cond->density * v_rel_mag * v_rel_mag * area *
                      c_d / get_vehicle_mass(vehicle, t);

  cartvec drag = smultiply(v_rel, -a_drag_mag / v_rel_mag);

  return drag;
}

/**
 * Update drag acceleration for the current phase and vehicle model.
 *
 * @param run_params Pointer to run configuration parameters
 * @param vehicle Pointer to vehicle model/state
 * @param atm_cond Pointer to atmospheric conditions
 * @param state Pointer to state updated with drag acceleration
 */
cartvec get_drag_acc(runparams *run_params, vehicle *vehicle,
                     atm_cond *atm_cond, state *state, double t) {

  cartvec drag;
  // Calculate drag acceleration for realistic maneuvering vehicle
  if (run_params->rv_maneuv == 1) {
    double lift_magnitude = norm(state->a_lift);
    double max_a_exec = get_max_a_exec(run_params, vehicle);
    double aoa = lift_magnitude * (AOA_MAX / max_a_exec * M_PI / 180);
    double c_d = vehicle->rv.c_d_0 + fabs(vehicle->rv.c_d_alpha * aoa);
    drag = get_drag_acceleration_generic(t, *state, atm_cond, vehicle, c_d,
                                         vehicle->rv.rv_area);
  }
  // Calculate the drag acceleration components for boost phase & reentry of
  // ballistic or perfectly maneuvering reentry vehicle
  else {
    double c_d;
    double area;

    if (t < vehicle->booster.total_burn_time) {
      // Drag coefficient varies with angle of attack
      cartvec winds_cart = get_cart_wind(state, atm_cond);
      double wind_mag = norm(winds_cart);
      double v_mag = norm(state->velocity);

      // Angle of attack (assuming vehicle oriented along velocity vector)
      double aoa = atan(wind_mag / v_mag);
      c_d = vehicle->rv.c_d_0 + fabs(vehicle->rv.c_d_alpha * aoa);
      area = vehicle->rv.rv_area;
    } else {
      c_d = vehicle->booster.c_d_0;
      area = vehicle->booster.area;
    }
    drag =
        get_drag_acceleration_generic(t, *state, atm_cond, vehicle, c_d, area);
  }
  return drag;
}

#endif