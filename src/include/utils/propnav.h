#ifndef PROPNAV_H
#define PROPNAV_H

#include "../models/atmosphere.h"
#include "../models/grav.h"
#include "../models/state.h"
#include "../physics/gravity.h"
#include "runparams.h"

/**
 * Use proportional navigation with a linear, altitude-based gain to return
 * commanded accelerations. Accounts for gravity by treating it as a target's
 * acceleration.
 *
 * @param estimated_state Pointer to the estimated vehicle state.
 * @param run_params Pointer to the run configuration parameters.
 * @param grav Pointer to the gravity model used for correction.
 * @return Commanded acceleration vector in ECI coordinates.
 */
cartvec prop_nav(state *estimated_state, runparams *run_params,
                 grav *est_grav) {
  cartvec aimpoint = {run_params->x_aim, run_params->y_aim, run_params->z_aim};
  // Calculate the relative position vector to the target
  cartvec r_target = subtract(aimpoint, estimated_state->position);

  // Calculate the relative velocity vector to the (stationary) target
  cartvec v_rel = smultiply(estimated_state->velocity, -1.0);

  // Get the rotation vector by taking the cross product of the relative
  // position and velocity vectors and dividing by |r|^2
  double r_dot_r = dot(r_target, r_target);
  cartvec cross_product = cross(r_target, v_rel);
  cartvec rot = sdivide(cross_product, r_dot_r);

  // Calculate the acceleration command by taking the cross product of the
  // relative velocity and the rotation vector, scaled by the navigation gain
  double gain = run_params->nav_gain_0 +
                (run_params->nav_gain_1 - run_params->nav_gain_0) / 120e3 *
                    get_altitude(estimated_state->position);

  cartvec cross_v_rot = cross(v_rel, rot);
  cartvec a_cmd_E = smultiply(cross_v_rot, gain);

  double mag_a_cmd = norm(a_cmd_E);

  // Guard against division by zero
  if (mag_a_cmd < 1e-9) {
    return zeros();
  }

  // Subtract gravitational acceleration in the direction of the acceleration
  // command from the acceleration command by treating it as a target's
  // acceleration with augmented proportional navigation. See Zarchan Ch. 16
  // (1994).
  cartvec a_cmd_E_hat = sdivide(a_cmd_E, mag_a_cmd);
  cartvec a_grav_E = get_gravity_acc(est_grav, estimated_state);
  cartvec a_grav_E_perp = smultiply(a_cmd_E_hat, dot(a_grav_E, a_cmd_E_hat));
  a_grav_E_perp = smultiply(a_grav_E_perp, gain / 2.0);
  cartvec a_cmd_E_no_grav = subtract(a_cmd_E, a_grav_E_perp);

  return a_cmd_E_no_grav;
}

#endif