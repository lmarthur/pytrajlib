#ifndef AERO_FORCES_H
#define AERO_FORCES_H

#define _USE_MATH_DEFINES

#include "../math/linalg.h"
#include "../models/vehicle.h"
#include "drag.h"

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
static inline int is_reentry(state *state, double t) {
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

static inline double
interpolate_swerve_coeff(double alpha_deg,
                         const double coeff_table[SWERVE_AERO_TABLE_SIZE]) {
  double alpha_min_deg = SWERVE_ALPHA_DEG_TABLE[0];
  double alpha_max_deg = SWERVE_ALPHA_DEG_TABLE[SWERVE_AERO_TABLE_SIZE - 1];
  double dx_all = alpha_max_deg - alpha_min_deg;

  if (alpha_deg < alpha_min_deg || alpha_deg > alpha_max_deg) {
    double slope =
        (coeff_table[SWERVE_AERO_TABLE_SIZE - 1] - coeff_table[0]) / dx_all;
    if (alpha_deg < alpha_min_deg) {
      return coeff_table[0] + slope * (alpha_deg - alpha_min_deg);
    }
    return coeff_table[SWERVE_AERO_TABLE_SIZE - 1] +
           slope * (alpha_deg - alpha_max_deg);
  }

  return linterp(alpha_deg, (double *)SWERVE_ALPHA_DEG_TABLE,
                 (double *)coeff_table, SWERVE_AERO_TABLE_SIZE);
}

/**
 * Compute angle of attack from body-frame relative-wind direction.
 *
 * $$\alpha = \cos^{-1}(u_3),\quad u_3 = \hat{\mathbf u}_B.z$$
 *
 * @param u_hat_B Unit relative-wind vector in body coordinates.
 * @return Angle of attack in radians.
 */
static inline double get_aoa(cartvec u_hat_B) {
  return acos(clip(u_hat_B.z, -1.0, 1.0));
}

/**
 * Compute body-lift direction by projecting body axis e3 onto the plane normal
 * to freestream direction u_hat_B.
 *
 * $$\hat{\ell}_B = \frac{(I-\hat{u}_B\hat{u}_B^T)\hat{e}_{3,B}}
 * {\|(I-\hat{u}_B\hat{u}_B^T)\hat{e}_{3,B}\|}$$
 *
 * @param u_hat_B Unit relative-wind vector in body coordinates.
 * @return Unit body-lift direction; zero vector if near singular.
 */
static inline cartvec get_body_lift_direction(cartvec u_hat_B) {
  cartvec e3_B = {0.0, 0.0, 1.0};
  cartvec lift_raw = subtract(e3_B, smultiply(u_hat_B, dot(u_hat_B, e3_B)));
  double lift_raw_norm = norm(lift_raw);
  if (lift_raw_norm < 1e-10) {
    printf(
        "Warning: lift direction is near singular for u_hat_B = (%g, %g, %g)\n",
        u_hat_B.x, u_hat_B.y, u_hat_B.z);
    return zeros();
  }
  return sdivide(lift_raw, lift_raw_norm);
}

static inline cartvec get_body_force(state *current_state, atm_cond *atm_cond,
                                     vehicle *vehicle) {
  // Calculate dynamic pressure q_inf
  cartvec v_rel_E = get_relative_wind_eci(current_state, atm_cond);
  double v_rel = norm(v_rel_E);
  cartvec u_hat_E = sdivide(v_rel_E, norm(v_rel_E));
  double q_inf = 0.5 * atm_cond->density * v_rel * v_rel;

  // Drag direction unit vector
  cartvec u_hat_B = eci_to_body(u_hat_E, current_state->q_EB);

  // Lift direction unit vector
  cartvec l_hat_B = get_body_lift_direction(u_hat_B);

  // Get angle of attack based on relative wind in the body frame
  double alpha = get_aoa(u_hat_B);

  // Get drag and lift coefficients based on the angle of attack
  double C_D, C_L;
  if (strcmp(vehicle->rv.name, "SWERVE") == 0) {
    double alpha_deg = alpha * 180.0 / M_PI;
    C_D = interpolate_swerve_coeff(alpha_deg, SWERVE_CD_TABLE);
    C_L = interpolate_swerve_coeff(alpha_deg, SWERVE_CL_TABLE);
  } else {
    C_D = vehicle->rv.c_d_0 + vehicle->rv.c_d_alpha * alpha;
    C_L = vehicle->rv.c_l_alpha * alpha;
  }

  // Reference area for drag and lift coefficients
  double S_ref = vehicle->rv.rv_area;

  // Get drag and lift force vectors in the body frame
  cartvec drag_force_B = smultiply(u_hat_B, q_inf * S_ref * C_D);
  cartvec lift_force_B = smultiply(l_hat_B, q_inf * S_ref * C_L);

  cartvec body_force = add(drag_force_B, lift_force_B);

  return body_force;
}

/**
 * Compute post-shock stagnation-pressure ratio p0,2/p_inf from normal-shock
 * relations for a calorically perfect gas.
 *
 * $$\frac{p_{0,2}}{p_\infty} =
 * \left(1+\frac{\gamma-1}{2}M_\infty^2\right)^{\frac{\gamma}{\gamma-1}}
 * \left[\frac{\gamma+1}{2\gamma
 * M_\infty^2-(\gamma-1)}\right]^{\frac{1}{\gamma-1}}
 * \left[\frac{(\gamma+1)M_\infty^2}{(\gamma-1)M_\infty^2+2}\right]^{\frac{\gamma}{\gamma-1}}$$
 *
 * This implementation uses gamma = 1.4.
 *
 * @param mach Freestream Mach number M_inf.
 * @return p0,2 / p_inf ratio.
 */
static inline double get_post_shock_stagnation_pressure_ratio(double mach) {
  const double gamma = 1.4;
  const double m2 = mach * mach;
  const double g_over_gm1 = gamma / (gamma - 1.0);
  const double inv_gm1 = 1.0 / (gamma - 1.0);

  double term1 = pow(1.0 + 0.5 * (gamma - 1.0) * m2, g_over_gm1);
  double term2 =
      pow((gamma + 1.0) / (2.0 * gamma * m2 - (gamma - 1.0)), inv_gm1);
  double term3 =
      pow(((gamma + 1.0) * m2) / (((gamma - 1.0) * m2) + 2.0), g_over_gm1);

  return term1 * term2 * term3;
}

/**
 * Compute modified-Newtonian stagnation-point pressure coefficient C_p,max.
 *
 * $$C_{p,\max}(M_\infty)=\frac{2}{\gamma
 * M_\infty^2}\left(\frac{p_{0,2}}{p_\infty}-1\right)$$
 *
 * This implementation uses gamma = 1.4.
 * For this model configuration, Mach is fixed at M_inf = 12.
 *
 * @return Stagnation-point pressure coefficient C_p,max evaluated at Mach 12.
 */
static inline double get_cp_max() {
  const double gamma = 1.4;
  const double mach_fixed = 12.0;

  double p0_2_over_p_inf = get_post_shock_stagnation_pressure_ratio(mach_fixed);
  return (2.0 / (gamma * mach_fixed * mach_fixed)) * (p0_2_over_p_inf - 1.0);
}

/**
 * Compute the four inward-pointing undeflected flap normals in body coordinates
 * for a conical reentry vehicle geometry (Equation 55).
 *
 * For a cone with half-angle theta_c, the normals are:
 * $$\hat{\mathbf{n}}_{10,B} = -\cos\theta_c \hat{\mathbf{e}}_{1,B} +
 * \sin\theta_c \hat{\mathbf{e}}_{3,B}$$
 * $$\hat{\mathbf{n}}_{20,B} = -\cos\theta_c \hat{\mathbf{e}}_{2,B} +
 * \sin\theta_c \hat{\mathbf{e}}_{3,B}$$
 * $$\hat{\mathbf{n}}_{30,B} = +\cos\theta_c \hat{\mathbf{e}}_{1,B} +
 * \sin\theta_c \hat{\mathbf{e}}_{3,B}$$
 * $$\hat{\mathbf{n}}_{40,B} = +\cos\theta_c \hat{\mathbf{e}}_{2,B} +
 * \sin\theta_c \hat{\mathbf{e}}_{3,B}$$
 *
 * @param vehicle Vehicle model containing cone half-angle in rv.half_angle.
 * @param n_B Output array of 4 flap normals in body coordinates.
 */
static inline void get_undeflected_flap_normals(vehicle *vehicle,
                                                cartvec n_B[4]) {
  double theta_c = vehicle->rv.half_angle;
  double cos_theta = cos(theta_c);
  double sin_theta = sin(theta_c);

  n_B[0].x = -cos_theta;
  n_B[0].y = 0.0;
  n_B[0].z = sin_theta;

  n_B[1].x = 0.0;
  n_B[1].y = -cos_theta;
  n_B[1].z = sin_theta;

  n_B[2].x = cos_theta;
  n_B[2].y = 0.0;
  n_B[2].z = sin_theta;

  n_B[3].x = 0.0;
  n_B[3].y = cos_theta;
  n_B[3].z = sin_theta;
}

/**
 * Compute the four deflected flap normals in body coordinates.
 *
 * The deflected normal of flap i is
 * $$\hat{\mathbf n}_{i,B}(\delta_{f,i}) = R(\hat{\mathbf
 * h}_{i,B},\delta_{f,i})\hat{\mathbf n}_{i0,B}$$ with hinge axes and pair
 * mapping
 * $$\hat{\mathbf h}_{1,B}=\hat{\mathbf h}_{3,B}=\hat{\mathbf
 * e}_{2,B},\;\hat{\mathbf h}_{2,B}=\hat{\mathbf h}_{4,B}=-\hat{\mathbf
 * e}_{1,B}$$
 * $$\delta_{f,1}=\delta_{f,3}=\delta_1,\;\delta_{f,2}=\delta_{f,4}=\delta_2.$$
 *
 * @param vehicle Vehicle model used to construct undeflected flap normals.
 * @param delta1 Deflection command for flap pair {1, 3} in radians.
 * @param delta2 Deflection command for flap pair {2, 4} in radians.
 * @param n_deflected_B Output array of 4 deflected flap normals in body
 * coordinates.
 */
static inline void get_deflected_flap_normals(vehicle *vehicle, double delta1,
                                              double delta2,
                                              cartvec n_deflected_B[4]) {
  cartvec n_undeflected_B[4];
  get_undeflected_flap_normals(vehicle, n_undeflected_B);

  cartvec e1_B = {1.0, 0.0, 0.0};
  cartvec e2_B = {0.0, 1.0, 0.0};
  cartvec h_pair1 = e2_B;
  cartvec h_pair2 = smultiply(e1_B, -1.0);

  n_deflected_B[0] = rotate(n_undeflected_B[0], h_pair1, delta1);
  n_deflected_B[1] = rotate(n_undeflected_B[1], h_pair2, delta2);
  n_deflected_B[2] = rotate(n_undeflected_B[2], h_pair1, delta1);
  n_deflected_B[3] = rotate(n_undeflected_B[3], h_pair2, delta2);
}

/**
 * Compute per-flap incremental force and apply flap-force magnitude limiting.
 *
 * @param deflected_force_B Deflected flap force vector in body coordinates.
 * @param undeflected_force_B Undeflected flap force vector in body coordinates.
 * @param max_flap_force Maximum allowed flap force magnitude.
 * @return Limited incremental flap force vector in body coordinates.
 */
static inline cartvec
get_limited_incremental_flap_force(cartvec deflected_force_B,
                                   cartvec undeflected_force_B,
                                   runparams *run_params, vehicle *vehicle) {
  double max_flap_force = get_max_flap_force(run_params, vehicle);

  cartvec incremental_force = subtract(deflected_force_B, undeflected_force_B);
  double incremental_force_mag = norm(incremental_force);
  if (incremental_force_mag < 1e-12) {
    return zeros();
  }
  // Each flap is capable of only producing up to the maximum force allowed by
  // the actuators. Set lower bound to 1e-12 to avoid numerical issues if force
  // magnitude is zero.

  double incremental_force_mag_limited =
      clip(incremental_force_mag, 0, max_flap_force);
  cartvec flap_force = smultiply(
      incremental_force, incremental_force_mag_limited / incremental_force_mag);

  return flap_force;
}

/**
 * Compute loaded-side incidence factors for all flaps (Equation 61).
 *
 * $$\lambda_i(\delta_{f,i}) = \max\left(0,\hat{\mathbf u}_B \cdot \hat{\mathbf
 * n}_{i,B}(\delta_{f,i})\right),\; i\in\{1,2,3,4\}$$
 *
 * Flap indexing in output array is zero-based:
 * 0 -> flap 1, 1 -> flap 2, 2 -> flap 3, 3 -> flap 4.
 *
 * @param vehicle Vehicle model used for flap geometry.
 * @param delta1 Deflection command for flap pair {1, 3} in radians.
 * @param delta2 Deflection command for flap pair {2, 4} in radians.
 * @param u_hat_B Relative-wind unit direction in body frame.
 * @param incidence_factors Output array of 4 loaded-side incidence factors.
 */
static inline void get_incidence_factors(vehicle *vehicle, double delta1,
                                         double delta2, cartvec u_hat_B,
                                         double incidence_factors[4]) {

  cartvec n_deflected_B[4];
  get_deflected_flap_normals(vehicle, delta1, delta2, n_deflected_B);

  for (int i = 0; i < 4; i++) {
    // Only the windward side of the flap carries pressure. Accounts for the
    // angle of attack.
    incidence_factors[i] = fmax(0.0, dot(u_hat_B, n_deflected_B[i]));
  }
}

/**
 * Compute absolute force magnitudes for all four flaps.
 *
 * @param current_state Current vehicle state.
 * @param atm_cond Atmospheric conditions.
 * @param vehicle Vehicle model containing flap area.
 * @param delta1 Deflection command for flap pair {1, 3} in radians.
 * @param delta2 Deflection command for flap pair {2, 4} in radians.
 * @param q_EB Quaternion rotating body-frame vectors into ECI.
 * @param flap_force_magnitudes Output array of 4 flap force magnitudes.
 */
static inline void get_absolute_flap_force_magnitudes(
    state *current_state, atm_cond *atm_cond, vehicle *vehicle, double delta1,
    double delta2, quaternion q_EB, double flap_force_magnitudes[4]) {
  // Get relative wind in the body frame
  cartvec v_rel_E = get_relative_wind_eci(current_state, atm_cond);
  double v_rel_mag = norm(v_rel_E);
  cartvec u_hat_E = zeros();
  if (v_rel_mag > 1e-12) {
    u_hat_E = sdivide(v_rel_E, v_rel_mag);
  }
  cartvec u_hat_B = eci_to_body(u_hat_E, q_EB);

  // Calculate dynamic pressure
  double q_inf = 0.5 * atm_cond->density * v_rel_mag * v_rel_mag;

  // Calculate the pressure scale factor
  double K_f = get_cp_max();

  // Use the relative wind direction to get the incidence factor---how aligned
  // each flap is with the wind
  double incidence_factors[4];
  get_incidence_factors(vehicle, delta1, delta2, u_hat_B, incidence_factors);

  for (int i = 0; i < 4; i++) {
    double lambda_i = incidence_factors[i];
    // Calculate the magnitude of the force on the flap
    flap_force_magnitudes[i] =
        q_inf * vehicle->rv.flap_area * K_f * lambda_i * lambda_i;
  }
}

/**
 * Compute absolute flap force vectors for all four flaps in body coordinates.
 *
 * Quantize the flaps according to the actuator resolution.
 *
 * Each force is oriented along its deflected inward-pointing flap normal:
 * $$\mathbf F_{f,i,B} = F_i\hat{\mathbf n}_{i,B}(\delta_{f,i})$$
 *
 * @param current_state Current vehicle state.
 * @param atm_cond Atmospheric conditions.
 * @param vehicle Vehicle model containing flap geometry and area.
 * @param delta1 Deflection command for flap pair {1, 3} in radians.
 * @param delta2 Deflection command for flap pair {2, 4} in radians.
 * @param q_EB Quaternion rotating body-frame vectors into ECI.
 * @param flap_forces_B Output array of 4 flap force vectors in body
 * coordinates.
 */
static inline void get_absolute_flap_forces_body(state *current_state,
                                                 atm_cond *atm_cond,
                                                 vehicle *vehicle,
                                                 runparams *run_params,
                                                 double delta1, double delta2,
                                                 cartvec flap_forces_B[4]) {
  // Convert resolution from degrees to radians
  double resolution = run_params->actuator_resolution * M_PI / 180;

  // Limit resolution of flap deflection angles
  delta1 = round(delta1 / resolution) * resolution;
  delta2 = round(delta2 / resolution) * resolution;

  cartvec n_deflected_B[4];
  get_deflected_flap_normals(vehicle, delta1, delta2, n_deflected_B);

  double flap_force_magnitudes[4];
  get_absolute_flap_force_magnitudes(current_state, atm_cond, vehicle, delta1,
                                     delta2, current_state->q_EB,
                                     flap_force_magnitudes);

  for (int i = 0; i < 4; i++) {
    double F_i = flap_force_magnitudes[i];
    flap_forces_B[i] = smultiply(n_deflected_B[i], F_i);
  }
}

static inline cartvec sum_incremental_forces(state *current_state,
                                             atm_cond *atm_cond,
                                             vehicle *vehicle,
                                             runparams *run_params) {
  cartvec sum_forces = zeros();

  // Compute F(deflection angle 0) and F(deflection angle delta_i) for each flap
  cartvec undeflected_flap_forces_B[4];
  cartvec deflected_flap_forces_B[4];
  get_absolute_flap_forces_body(current_state, atm_cond, vehicle, run_params,
                                0.0, 0.0, undeflected_flap_forces_B);
  get_absolute_flap_forces_body(current_state, atm_cond, vehicle, run_params,
                                current_state->delta_1, current_state->delta_2,
                                deflected_flap_forces_B);

  // Sum the "incremental" forces, F(deflection angle delta_i) - F(deflection
  // angle 0), for each flap i in [1, 4]
  for (int i = 0; i < 4; i++) {
    cartvec incremental_force_limited = get_limited_incremental_flap_force(
        deflected_flap_forces_B[i], undeflected_flap_forces_B[i], run_params,
        vehicle);

    sum_forces = add(sum_forces, incremental_force_limited);
  }
  return sum_forces;
}

static inline cartvec get_maneuverable_lift_drag(double t, state *current_state,
                                                 atm_cond *atm_cond,
                                                 vehicle *vehicle,
                                                 runparams *run_params) {
  // Get body force + sum of incremental forces due to flaps
  cartvec body_force = get_body_force(current_state, atm_cond, vehicle);
  cartvec incremental_force =
      sum_incremental_forces(current_state, atm_cond, vehicle, run_params);
  cartvec total_force_body = add(body_force, incremental_force);

  // Transform from body force to ECI force
  cartvec total_force_eci = body_to_eci(total_force_body, current_state->q_EB);

  // Transform force to acceleration
  cartvec a_aero = sdivide(total_force_eci, vehicle->rv.rv_mass);

  return a_aero;
}

static inline cartvec get_aerodynamic_acc(double t, state *current_state,
                                          atm_cond *atm_cond, vehicle *vehicle,
                                          runparams *run_params) {
  // Boost phase drag
  if (t < vehicle->booster.total_burn_time) {
    cartvec a_drag = boost_drag(t, current_state, atm_cond, vehicle);
    return a_drag;
  }

  // After boost phase, the vehicle should be outside the atmosphere where there
  // are no aerodynamic forces
  if (!is_reentry(current_state, t)) {
    return zeros();
  }

  // Ballistic reentry drag
  if (run_params->rv_maneuv != 1 && run_params->ballistic_drag) {
    cartvec a_drag =
        ballistic_reentry_drag(t, current_state, atm_cond, vehicle);
    return a_drag;
  }

  // Maneuverable reentry lift & drag
  cartvec a_aero = get_maneuverable_lift_drag(t, current_state, atm_cond,
                                              vehicle, run_params);
  return a_aero;
}

#endif