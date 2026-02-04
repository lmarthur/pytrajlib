/*
`get_drag_acceleration` is the only function that should be called outside this
module. It calls one of the following methods to calculate the current
acceleration due to drag:

1. Boost-phase drag. See `get_boost_drag`
2. Reentry-phase drag for ballistic trajectories. See
`_get_ballistic_reentry_drag
3. Reentry-phase drag for maneuverable trajectories. See
`_get_maneuverable_reentry_drag`

Each of these functions calls `get_drag_acceleration_generic` passing the
appropriate drag coefficient and characteristic area for that phase of flight.

Given the atmospheric density $\rho$ (kg/m^3), the magnitude of the relative
velocity (wrt the wind) $v$ (m/s), the cross-sectional area of the booster
$A$ (m^2), the booster's drag coefficient $C_D$,
and the current mass $m$, the magnitude of the drag acceleration is
$$
|\vec a_\text{drag}| = \frac{1}{2} \rho v^2 A C_D / m
$$

The drag acceleration is in the direction opposing the relative velocity.
*/
#ifndef DRAG_H
#define DRAG_H

#include "integrator/args.h"
#include "math/linalg.h"
#include "models/atmosphere.h"
#include "models/vehicle.h"
#include "utils/utils.h"

/**
 * Get the drag acceleration based on the drag coefficient
 * and the characteristic area.
 */
cartvec get_drag_acceleration_generic(double t, state current_state,
                                      integrator_args args, double c_d,
                                      double area) {
    cartvec wind_vec = get_cart_wind(current_state, args);
    cartvec v_rel = subtract(current_state.velocity, wind_vec);
    double v_rel_mag = norm(v_rel);
    double mass = get_mass(t, args.vehicle);
    double density = get_atm_density(current_state, args);

    double a_drag_mag =
        0.5 * density * v_rel_mag * v_rel_mag * area * c_d / mass;

    return smultiply(v_rel, -a_drag_mag / v_rel_mag);
}

/**
 * Calculate the acceleration due to drag during the boost phase.
 * Assumes zero angle of attack, so the drag coefficient is $C_{D_0}$ (set
 * in run_params. The characteristic area is the booster's cross-sectional area.
 */
cartvec get_boost_drag(double t, state current_state, integrator_args args) {
    return get_drag_acceleration_generic(t, current_state, args,
                                         args.vehicle.booster.c_d_0,
                                         args.vehicle.booster.area);
}

/*
  Calculate drag acceleration vector during reentry phase for ballistic
  trajectories.

  The angle of attack, $\alpha$, is based on the wind speed, $w$, and the
  vehicle speed, $v$:
  $$
  \alpha = \arctan(w / v)
  $$

  The drag coefficient is
  $$
  C_D = C_{D_0} + \frac{d C_D}{d \alpha} \alpha.
  $$

  The characteristic area is the reentry vehicle's cross-sectional area.
*/
cartvec get_ballistic_reentry_drag(double t, state current_state,
                                   integrator_args args) {
    cartvec winds_cart = get_cart_wind(current_state, args);
    double wind_mag = norm(winds_cart);
    double v_mag = norm(current_state.velocity);

    // Angle of attack (assuming vehicle oriented along velocity vector)
    double aoa = atan(wind_mag / v_mag);

    // Drag coefficient varies with angle of attack
    double c_d = args.vehicle.rv.c_d_0 + fabs(args.vehicle.rv.c_d_alpha * aoa);

    cartvec a_drag = get_drag_acceleration_generic(t, current_state, args, c_d,
                                                   args.vehicle.rv.rv_area);
    return a_drag;
}

/*
  Calculate the acceleration due to drag during the reentry phase for a
  maneuverable trajectory.

  The angle of attack is proportional to the magnitude of the lift
  acceleration. The maximum lift acceleration is calculated in `get_max_a_exec`.

  The characteristic area is the reentry vehicle's cross-sectional area.
*/
cartvec get_maneuverable_reentry_drag(double t, state current_state,
                                      integrator_args args) {
    double lift_magnitude = norm(current_state.a_lift);
    double max_a_exec = get_max_a_exec(args.run_params, args.vehicle);
    cartvec wind_vec = get_cart_wind(current_state, args);
    cartvec v_rel = subtract(current_state.velocity, wind_vec);

    double v_rel_mag = norm(v_rel);
    double density = get_atm_density(current_state, args);
    double mass = get_mass(t, args.vehicle);

    // Calculate angle of attack in radians
    double aoa = lift_magnitude * (AOA_MAX / max_a_exec * M_PI / 180);

    // Get the drag coefficient based on the angle of attack
    double c_d = args.vehicle.rv.c_d_0 + fabs(args.vehicle.rv.c_d_alpha * aoa);

    // Get the drag magnitude
    double a_drag_mag = 0.5 * density * v_rel_mag * v_rel_mag *
                        args.vehicle.rv.rv_area * c_d /
                        mass; // Update the drag acceleration vector based on
                              // the drag magnitude and direction

    cartvec a_drag = smultiply(v_rel, -a_drag_mag / v_rel_mag);
    return a_drag;
}

/**
 * Calculate drag acceleration
 *
 * @param t current flight time (seconds)
 * @param current_state true or estimated state
 * @param args true or estimated integrator args
 * @returns acceleration due to drag in cartesian x, y, z coordinates (m/s^2)
 */
cartvec get_drag_acceleration(double t, state current_state,
                              integrator_args args) {

    double altitude = get_altitude(current_state);
    // If above 100km, no drag
    if (altitude > 1e5) {
        return zeros();
    }
    // Get atmospheric conditions at current altitude
    cartvec winds_cart = get_cart_wind(current_state, args);

    // Relative velocity (vehicle velocity - wind velocity)
    cartvec v_rel = subtract(current_state.velocity, winds_cart);
    double v_rel_mag = norm(v_rel);

    // Avoid division by zero for near-zero relative velocity
    if (v_rel_mag < 1e-6) {
        return zeros();
    }

    // Drag calculation differs whether we are in boost phase or reentry phase
    // and whether the vehicle is maneuverable or ballistic
    if (t < args.vehicle.booster.total_burn_time) {
        return get_boost_drag(t, current_state, args);
    }
    if (norm(current_state.a_lift) > 0) {
        return get_maneuverable_reentry_drag(t, current_state, args);
    }
    return get_ballistic_reentry_drag(t, current_state, args);
}

#endif
