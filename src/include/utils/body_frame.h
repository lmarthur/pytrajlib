#ifndef BODY_FRAME_H
#define BODY_FRAME_H

#include "../math/linalg.h"
#include "../models/atmosphere.h"
#include "../models/grav.h"
#include "../models/state.h"
#include "propnav.h"

/**
 * Determine whether the vehicle is in reentry conditions.
 *
 * Reentry is detected when altitude is below 100 km and velocity is directed
 * generally toward Earth center.
 *
 * @param state Pointer to current state
 * @param t Current simulation time in seconds
 * @return 1 if in reentry, else 0
 */
int is_reentry(state *state, double t) {
  // Check for small t to account for initial velocity error that might make the
  // vehicle appear to be below altitude 0 after a single step
  if (t < 10)
    return 0;
  double v_mag = norm(state->velocity);
  if (v_mag < 1e-6)
    return 0;
  double altitude = get_altitude(state->position);
  if (altitude >= 1e5)
    return 0;
  double cos_angle = dot(state->velocity, smultiply(state->position, -1)) /
                     (v_mag * norm(state->position));
  cos_angle =
      clip(cos_angle, -1.0, 1.0); // guard against floating-point overshoot
  double angle_v_grav = acos(cos_angle);
  return (angle_v_grav > 0) && (angle_v_grav < M_PI_2);
}

/**
 * The body-centric coordinate system is defined as follows: $\hat x$ (roll)
 * points along relative velocity, $\hat z$ (yaw) points along commanded lift
 * acceleration (or global z if there is no lift), and
 * $\hat y = \hat z \times \hat x$. Because commanded lift may not be exactly
 * perpendicular to wind-relative velocity, the lift direction is
 * orthogonalized with Gram-Schmidt to produce an orthonormal basis.
 *
 * Gyroscope error in pitch and yaw is modeled as small-angle rotations of the
 * true body frame. A pitch-axis rotation by $\theta$ is
 * <div>
 * $$
 * \begin{align}
 * \mathbf{R_y} = \begin{bmatrix} 1 & 0 & \theta  \\ 0 & 1 & 0 \\ -\theta & 0 &
 * 1\end{bmatrix}.
 * \end{align}
 * $$
 * </div>
 * A yaw-axis rotation by $\phi$ is
 * <div>
 * $$
 * \begin{align}
 * \mathbf{R_z} = \begin{bmatrix} 1 & -\phi & 0  \\ \phi & 1 & 0 \\ 0 & 0 &
 * 1\end{bmatrix}.
 * \end{align}
 * $$
 * </div>
 * Under small-angle approximation, rotations commute and the cross-coupling
 * matrix is
 * <div>
 * $$
 * \begin{align}
 * \mathbf{C} = \mathbf{R_y R_z} = \mathbf{R_z R_y} =
 * \begin{bmatrix} 1 & -\phi & \theta  \\ \phi & 1 & 0 \\ -\theta & 0 &
 * 1\end{bmatrix}.
 * \end{align}
 * $$
 * </div>
 * With true basis
 * <div>
 * $$
 * \begin{align}
 * \mathbf{B} &= \begin{bmatrix} \hat x & \hat y & \hat z \end{bmatrix} \\
 * \mathbf{B}_\text{est} &= \mathbf{C} \mathbf{B}.
 * \end{align}
 * $$
 * </div>
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
int get_body_frame(state *true_state, state *est_state, runparams *run_params,
                   double t, atm_cond *atm_cond, cartvec *xhat, cartvec *yhat,
                   cartvec *zhat, int apply_gyro_error, grav *grav) {

  cartvec velocity = true_state->velocity;

  cartvec v_rel;

  if (atm_cond) {
    cartvec wind_vec = get_cart_wind(true_state, atm_cond);
    v_rel = subtract(velocity, wind_vec);
  } else {
    v_rel = velocity;
  }
  double v_rel_mag = norm(v_rel);

  // If the relative velocity is zero, we cannot define a local coordinate
  // system
  if (v_rel_mag < 1e-6) {
    // The velocity at start is zero, but we know the orientation
    if (get_altitude(true_state->position) < 1e-3) {
      v_rel.x = 1;
      v_rel.y = 0;
      v_rel.z = 0;
    } else {
      printf("Warning: Relative velocity is zero, cannot define body frame\n");
      return 0;
    }
  }

  cartvec tmp_lift;

  // If the initial lift magnitude is zero, define yhat based on a cross
  // product between xhat and global z-axis
  if (run_params->rv_maneuv != 1 || !is_reentry(true_state, t)) {
    tmp_lift.x = 0;
    tmp_lift.y = 0;
    tmp_lift.z = 1;
  } else {
    tmp_lift = prop_nav(est_state, run_params, grav);
  }

  *xhat = sdivide(v_rel, norm(v_rel));
  cartvec lift_perp =
      subtract(tmp_lift, smultiply(*xhat, dot(tmp_lift, *xhat)));
  // If the lift is parallel to the velocity, print warning
  if (norm(lift_perp) < 1e-6) {
    printf("Warning: lift is parallel to velocity. Can't find body frame\n");
    return 0;

  }
  // Create zhat vector by subtracting the component of the commanded lift along
  // the relative velocity vector
  else {
    *zhat = sdivide(lift_perp, norm(lift_perp));
  }

  // yhat is orthogonal to both lift and relative velocity
  *yhat = cross(*zhat, *xhat);

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