#ifndef PROPNAV_H
#define PROPNAV_H

#include "../models/atmosphere.h"
#include "../models/grav.h"
#include "../models/state.h"
#include "../physics/gravity.h"
#include "runparams.h"

/**
ECI acceleration commands are issued from an outer-loop proportional navigation
guidance law to steer the vehicle to the stationary aim point by maintaining a
consistent line of sight between the vehicle and the aim point. Given $\mathbf
r_\text{v,a} = \mathbf r_\text{aim} - \mathbf r_\text{est}$ is the displacement
between the vehicle and the aim point, and $\mathbf v_\text{est}$ is the
estimated vehicle velocity, the line-of-sight rotation vector is
\begin{align}
  \boldsymbol \Omega = \frac{\mathbf r_\text{v,a} \times (-\mathbf
v_\text{est})}{\mathbf r_\text{v,a} \cdot \mathbf r_\text{v,a}}.
\end{align}

The commanded acceleration in the ECI frame is orthogonal to the vehicle's
velocity and is issued to bring the line-of-sight rotation rate to zero. The
gravitational acceleration in the direction of commanded acceleration is treated
as the target's acceleration, as described by Zarchan (2012). Subtracting the
gravitational acceleration ensures the acceleration command represents only the
lift acceleration the vehicle needs to generate:
\begin{align}
\mathbf a_{c,E} = -N \mathbf v_\text{est} \times \boldsymbol \Omega -
\frac{N}{2}\mathbf a_{\text{grav,E,est}}.
\end{align}
The navigation gain is a linear function of the estimated altitude and
empirically tuned to take on the value $N_1$ at the reentry altitude,
($r_\text{reentry} - r_\text{Earth}$), and the value $N_0$ at impact:
\begin{equation}
  N = N_0 + \frac{N_1 - N_0}{r_\text{reentry} - r_\text{Earth}} (|\mathbf
r_{v,\text{est}}| - r_\text{Earth}).
\end{equation}
 * @param estimated_state Pointer to the estimated vehicle state.
 * @param run_params Pointer to the run configuration parameters.
 * @param est_grav Pointer to the guidance gravity model.
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