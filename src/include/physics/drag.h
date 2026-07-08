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

/**
During the boost phase, the change in the drag coefficient as a function of the
angle of attack is proportional to $\cos^2(\alpha)$ for an axisymmetric vehicle,
as described by Jorgensen and Center (1973). Using a small-angle approximation,
the boost-phase drag coefficient is modeled as independent of the angle of
attack. The boost phase drag acceleration is oriented along the direction of
relative wind $\hat{\mathbf{u}}\_E$
\begin{equation}
    \mathbf{a}\_\text{drag} = q_\infty C\_D S/m \hat{\mathbf{u}}\_E
\end{equation}
where $C\_D, S$ refer to the drag coefficient and cross-sectional base area,
respectively, of the missile with the booster.
 */
static inline cartvec boost_drag(double t, state *current_state,
                                 atm_cond *atm_cond, vehicle *vehicle) {
  double c_d = vehicle->booster.c_d_0;
  double area = vehicle->booster.area;
  cartvec a_drag = get_drag_acceleration_generic(t, current_state, atm_cond,
                                                 vehicle, c_d, area);
  return a_drag;
}

/**
 * Ballistic reentry vehicles have the option to use the same physics described
in the aero forces functions or a simplified ballistic drag where the
vehicle's roll axis is aligned with the current velocity and the vehicle is
assumed to be at a trim angle of attack:
\begin{equation}
    \alpha = \arctan(v_\text{wind} / v).
\end{equation}
The drag coefficient is a function of the angle of attack, so the total drag
acceleration is
\begin{equation}
    \mathbf{a}\_\text{drag} = q\_\infty (C\_{D_0} + C_{D\_\alpha} \alpha) S/m
\hat{\mathbf{u}}\_E.
\end{equation}
 */
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