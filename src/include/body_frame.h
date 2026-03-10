#ifndef BODY_FRAME_H
#define BODY_FRAME_H

#include "math/linalg.h"
#include "models/atmosphere.h"
#include "models/state.h"

/**
 * Define a local coordinate system where
 *
 * - $\vec e_1$ points in the direction of relative velocity (if atm_cond
 * provided, otherwise points in the direction of velocity)
 *
 * - $\vec e_2$ points in the direction of (orthonormalized) lift acceleration
 *
 * - $\vec e_3$ is orthogonal to both ($\vec e_3 = \vec e_1 \times \vec e_2$)
 *
 *
 * The basis is only successfully defined if
 *
 * 1. the velocity is not zero
 * @return 1 if successfully defined basis, 0 if unsuccessful
 */
int get_body_frame(state *current_state, atm_cond *atm_cond, cartvec *e_1,
                   cartvec *e_2, cartvec *e_3) {

  cartvec velocity = current_state->velocity;
  cartvec a_lift = current_state->a_lift;

  cartvec v_rel;

  if (atm_cond) {
    cartvec wind_vec = get_cart_wind(current_state, atm_cond);
    v_rel = subtract(velocity, wind_vec);
  } else {
    v_rel = velocity;
  }
  double v_rel_mag = norm(v_rel);
  double initial_lift_mag = norm(a_lift);

  // If the relative velocity is zero, we cannot define a local coordinate
  // system

  if (v_rel_mag < 1e-6) {
    // The velocity at start is zero, but we know the orientation
    if (get_altitude(current_state->position) < 1e-6) {
      v_rel.x = 1;
      v_rel.y = 0;
      v_rel.z = 0;
    } else {
      return 0;
    }
  }

  // e_1 is the unit vector in direction of relative velocity
  // e_2 is the (orthonormalized) lift vector.
  // e_3 = e_1 x e_2

  cartvec tmp_lift;
  // If the initial lift magnitude is zero, define e_2 based on a cross
  // product between e_1 and global z-axis
  if (initial_lift_mag < 1e-6) {
    tmp_lift.x = 0;
    tmp_lift.y = 0;
    tmp_lift.z = 1;
  } else {
    tmp_lift = a_lift;
  }

  // Create e_2 vector by moving the lift vector to be orthogonal to the
  // relative velocity
  gram_schmidt_orthonorm(v_rel, tmp_lift, e_1, e_2);
  *e_3 = cross(*e_1, *e_2);

  return 1;
}

#endif