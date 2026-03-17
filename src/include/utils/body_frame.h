#ifndef BODY_FRAME_H
#define BODY_FRAME_H

#include "../math/linalg.h"
#include "../models/atmosphere.h"
#include "../models/state.h"

/**
 * The body-centric coordinate system is defined as follows: $\hat e_1$ points
 * in the direction of relative velocity (roll axis), $\hat e_2$ points in the
 * direction of lift acceleration (pitch),  $\hat e_3$ is orthogonal to both:
 * $\hat e_3 = \hat e_1 \times \hat e_2$ (yaw). To ensure this process produces
 * a set of orthonormal vectors that span $\mathbb R^3$, the lift acceleration
 * is modified using the Gram-Schmidt process to be orthogonal to the relative
 * velocity.
 *
 * The basis is successfully defined as long as the relative velocity is
 * non-zero.
 *
 * We model the gyroscope error along the pitch and yaw axes as small rotations
 * of the true body frame. A rotation around the pitch axis by a small angle
 * $\theta$ is
 * $$
 * \mathbf{R_y} = \begin{bmatrix} 1 & 0 & \theta  \\ 0 & 1 & 0 \\ -\theta & 0 &
 * 1\end{bmatrix}.
 * $$
 * A rotation around the yaw axis by a small angle $\phi$ is
 * $$
 * \mathbf{R_z} = \begin{bmatrix} 1 & -\phi & 0  \\ \phi & 1 & 0 \\ 0 & 0 &
 * 1\end{bmatrix}.
 * $$
 * So the final cross coupling matrix for small angles is
 * $$
 * \mathbf{R_y R_z} = \begin{bmatrix} 1 & -\phi & \theta  \\ \phi & 1 & 0
 * \\ -\theta & 0 & 1\end{bmatrix}.
 * $$
 *
 * @param current_state Current vehicle state containing position, velocity,
 *                      lift acceleration, and gyro error data.
 * @param atm_cond Atmospheric conditions used to compute wind-relative
 * velocity. May be NULL to use inertial velocity directly.
 * @param xhat Output unit vector aligned with the relative velocity direction.
 * @param yhat Output unit vector aligned with the orthogonalized lift
 * direction.
 * @param zhat Output unit vector completing the right-handed body frame.
 * @param apply_gyro_error 1 to apply gyro cross-coupling error to the
 *                         basis vectors.
 * @return 1 if the basis is successfully defined, 0 otherwise.
 */
int get_body_frame(state *true_state, atm_cond *atm_cond, cartvec *xhat,
                   cartvec *yhat, cartvec *zhat, int apply_gyro_error) {

  cartvec velocity = true_state->velocity;
  cartvec a_lift = true_state->a_lift;

  cartvec v_rel;

  if (atm_cond) {
    cartvec wind_vec = get_cart_wind(true_state, atm_cond);
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
    if (get_altitude(true_state->position) < 1e-6) {
      v_rel.x = 1;
      v_rel.y = 0;
      v_rel.z = 0;
    } else {
      return 0;
    }
  }

  cartvec tmp_lift;
  // If the initial lift magnitude is zero, define yhat based on a cross
  // product between xhat and global z-axis
  if (initial_lift_mag < 1e-6) {
    tmp_lift.x = 0;
    tmp_lift.y = 0;
    tmp_lift.z = 1;
  } else {
    tmp_lift = a_lift;
  }

  // Create yhat vector by moving the lift vector to be orthogonal to the
  // relative velocity
  gram_schmidt_orthonorm(v_rel, tmp_lift, xhat, yhat);
  *zhat = cross(*xhat, *yhat);

  if (apply_gyro_error) {
    // Small angle rotation matrix around y (pitch) and z (yaw)
    double cross_coupling[3][3] = {
        {1.0, -true_state->gyro_error.yaw, true_state->gyro_error.pitch},
        {true_state->gyro_error.yaw, 1.0, 0.0},
        {-true_state->gyro_error.pitch, 0.0, 1.0},
    };
    *xhat = matvec_multiply(cross_coupling, *xhat);
    *yhat = matvec_multiply(cross_coupling, *yhat);
    *zhat = matvec_multiply(cross_coupling, *zhat);
  }

  return 1;
}

#endif