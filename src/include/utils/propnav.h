#ifndef PROPNAV_H
#define PROPNAV_H

#include "../forces/gravity.h"
#include "../models/grav.h"
#include "../models/state.h"
#include "runparams.h"

/**
 * Lift acceleration commands are issued from a proportional navigation
 * guidance law to steer the vehicle to a stationary aimpoint by maintaining a
 * line of sight between the vehicle and aimpoint. Given displacement
 * $\vec r$ and estimated velocity $\vec v$, the line-of-sight rotation vector
 * is
 * $$
 * \begin{align}
 * \vec \Omega = \frac{\vec r \times \vec v}{\vec r \cdot \vec r}.
 * \end{align}
 * $$
 * With navigation gain $N$, the commanded acceleration is
 * $$
 * \begin{align}
 * \vec a_c = -N \vec v \times \vec \Omega.
 * \end{align}
 * $$
 *
 * @param estimated_state Pointer to the estimated vehicle state.
 * @param run_params Pointer to the run configuration parameters.
 * @param grav Pointer to the gravity model used for correction.
 * @return Commanded acceleration vector in ECI coordinates.
 */
cartvec prop_nav(state *estimated_state, runparams *run_params, grav *grav) {
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
  cartvec cross_v_rot = cross(v_rel, rot);
  cartvec a_command = smultiply(cross_v_rot, run_params->nav_gain);

  // Subtract component of estimated gravity in the lift plane because this
  // acceleration exists and does not need to be commanded
  cartvec a_grav = get_gravity_acc(grav, estimated_state);
  cartvec vhat =
      sdivide(estimated_state->velocity, norm(estimated_state->velocity));
  cartvec a_grav_perp = subtract(a_grav, smultiply(vhat, dot(a_grav, vhat)));
  a_command = subtract(a_command, a_grav_perp);

  return a_command;
}

#endif