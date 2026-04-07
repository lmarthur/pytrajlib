#ifndef DRAG_H
#define DRAG_H

#include <math.h>

#include "../math/linalg.h"
#include "../models/atmosphere.h"
#include "../models/vehicle.h"
#include "../utils/utils.h"

static const double AOA_MAX = 10; // Maximum angle of attack is 10 degrees

/**
 * Calculate the drag acceleration for any drag coefficient and characteristic
 * area.
 *
 * Given the atmospheric density $\rho$ ($kg/m^3$), the magnitude of the
 * relative velocity with respect to the current wind $v_\text{rel}$ ($m/s$),
 * the cross-sectional area of the booster $A$ ($m^2$), the booster's drag
 * coefficient $C_D$, and the current mass $m$, the magnitude of the drag
 * acceleration ($m/s^2$) is
 * $$
 * |\vec a_\text{drag}| = \frac{1}{2} \rho v^2 A C_D / m.
 * $$
 * The drag vector opposes the direction of relative velocity.
 *
 * @param t Current simulation time in seconds
 * @param current_state Current vehicle state
 * @param atm_cond Pointer to atmospheric conditions
 * @param vehicle Pointer to vehicle model
 * @param c_d Drag coefficient
 * @param area Reference area in square meters
 * @return Drag acceleration in inertial Cartesian coordinates
 */
static inline cartvec
get_drag_acceleration_generic(double t, state current_state, atm_cond *atm_cond,
                              vehicle *vehicle, double c_d, double area) {
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
 * Compute drag acceleration for each regime (boost, reentry) and mode
 * (ballistic, maneuverable).
 *
 * During boost phase, the change of drag coefficient as a function of angle of
 * attack is proportional to $\cos^2(\alpha)$ for an axially symmetric vehicle.
 * The boost-phase angle of attack remains small ($<1\degree$), so the boost
 * drag coefficient is modeled as independent of angle of attack.
 *
 * During reentry, drag coefficient is modeled as a linear function of angle of
 * attack, where $C_{D,0}$ is the drag coefficient at zero angle of attack and
 * $C_{D,\alpha}$ is its derivative with respect to angle of attack:
 * $$
 * C_D = C_{D,0} + C_{D,\alpha} \alpha.
 * $$
 * For ballistic reentry vehicles, angle of attack is estimated assuming the
 * vehicle is oriented along the velocity vector:
 * $$
 * \alpha = \arctan(|\vec v_\text{wind}| / |\vec v|).
 * $$
 * For maneuverable reentry vehicles, angle of attack is estimated from current
 * lift, maximum achievable lift, and maximum achievable angle of attack:
 * $$
 * \alpha = |\vec a_\text{lift}| \alpha_\text{max} / a_\text{lift,max}.
 * $$
 *
 *
 * @param run_params Pointer to run configuration parameters
 * @param vehicle Pointer to vehicle model/state
 * @param atm_cond Pointer to atmospheric conditions
 * @param state Pointer to state updated with drag acceleration
 * @param t Current simulation time in seconds
 * @return Drag acceleration in inertial Cartesian coordinates
 */
cartvec get_drag_acc(runparams *run_params, vehicle *vehicle,
                     atm_cond *atm_cond, state *state, double t) {
  // No drag outside the atmosphere
  if (get_altitude(state->position) > 100e3) {
    return zeros();
  }

  double c_d;
  double area;

  // Boost drag
  if (t < vehicle->booster.total_burn_time) {
    c_d = vehicle->booster.c_d_0;
    area = vehicle->booster.area;
  }
  // Reentry drag
  else {
    area = vehicle->rv.rv_area;
    double aoa;
    // When maneuvering use the state's angle of attack
    if (run_params->rv_maneuv == 1) {
      aoa = state->alpha;
    }
    // If not maneuvering, calculate angle of attack based on wind direction
    // assuming vehicle oriented along velocity vector
    else {
      cartvec winds_cart = get_cart_wind(state, atm_cond);
      double wind_mag = norm(winds_cart);
      double v_mag = norm(state->velocity);
      aoa = atan(wind_mag / v_mag);
    }
    c_d = vehicle->rv.c_d_0 + fabs(vehicle->rv.c_d_alpha * aoa);
  }

  cartvec drag =
      get_drag_acceleration_generic(t, *state, atm_cond, vehicle, c_d, area);
  return drag;
}

#endif