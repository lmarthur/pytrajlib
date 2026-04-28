#ifndef DRAG_H
#define DRAG_H

#include <math.h>

#include "../math/linalg.h"
#include "../models/atmosphere.h"
#include "../models/vehicle.h"
#include "../utils/utils.h"

/**
 * @param t Current simulation time in seconds
 * @param current_state Current vehicle state
 * @param atm_cond Pointer to atmospheric conditions
 * @param vehicle Pointer to vehicle model
 * @param c_d Drag coefficient
 * @param area Reference area in square meters
 * @return Drag acceleration in inertial Cartesian coordinates
 */
static inline cartvec get_drag_acceleration_generic(double t,
                                                    state *current_state,
                                                    atm_cond *atm_cond,
                                                    vehicle *vehicle,
                                                    double c_d, double area) {
  cartvec v_rel = get_relative_wind_eci(current_state, atm_cond);
  double v_rel_mag = norm(v_rel);

  if (v_rel_mag < 1e-2) {
    return zeros();
  }

  double a_drag_mag = 0.5 * atm_cond->density * v_rel_mag * v_rel_mag * area *
                      c_d / get_vehicle_mass(vehicle, t);

  cartvec drag = smultiply(v_rel, a_drag_mag / v_rel_mag);

  return drag;
}

static inline cartvec boost_drag(double t, state *current_state,
                                 atm_cond *atm_cond, vehicle *vehicle) {
  double c_d = vehicle->booster.c_d_0;
  double area = vehicle->booster.area;
  cartvec a_drag = get_drag_acceleration_generic(t, current_state, atm_cond,
                                                 vehicle, c_d, area);
  return a_drag;
}

static inline cartvec ballistic_reentry_drag(double t, state *current_state,
                                             atm_cond *atm_cond,
                                             vehicle *vehicle) {
  double area = vehicle->rv.rv_area;
  // If not maneuvering, calculate angle of attack based on wind direction
  // assuming vehicle oriented along velocity vector
  cartvec winds_cart = get_cart_wind(current_state, atm_cond);
  double wind_mag = norm(winds_cart);
  double v_mag = norm(current_state->velocity);
  double aoa = atan(wind_mag / v_mag);
  double c_d = vehicle->rv.c_d_0 + fabs(vehicle->rv.c_d_alpha * aoa);

  cartvec a_drag = get_drag_acceleration_generic(t, current_state, atm_cond,
                                                 vehicle, c_d, area);
  return a_drag;
}

#endif