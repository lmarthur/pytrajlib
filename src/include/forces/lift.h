/*
We model lift acceleration during the reentry period below 100km altitude.

There are two components of the state that model the lift acceleration: the
available lift and the true lift. The available lift is an intermediate step to
calculate the true lift, which is used to update the vehicle's velocity.

*/
#ifndef LIFT_H
#define LIFT_H

#include "../math/linalg.h"
#include "../models/atmosphere.h"
#include "../models/state.h"
#include "../models/vehicle.h"
#include "../utils/body_frame.h"
#include "../utils/propnav.h"
#include "../utils/utils.h"
#include "gravity.h"

/**
 * Project a vector into the lift plane, clip, then reconstruct in Cartesian.
 *
 * Given orthonormal lift-plane basis vectors $(\mathbf e_2,\mathbf e_3)$,
 * the vector components are clipped independently to
 * $[-\text{max\_val},\text{max\_val}]$ and mapped back.
 *
 * @param yhat Lift-plane basis vector
 * @param zhat Lift-plane basis vector
 * @param arr Vector to project and clip
 * @param max_val Absolute clip limit in projected coordinates
 * @return Projected and clipped vector in Cartesian coordinates
 */
cartvec project_and_clip(cartvec yhat, cartvec zhat, cartvec arr,
                         double max_val) {
  // Project onto yhat and zhat
  double arr_yhat = dot(arr, yhat);
  double arr_zhat = dot(arr, zhat);

  // Clip to max_val
  arr_yhat = clip(arr_yhat, -max_val, max_val);
  arr_zhat = clip(arr_zhat, -max_val, max_val);

  // Project back to Cartesian basis
  cartvec result = add(smultiply(yhat, arr_yhat), smultiply(zhat, arr_zhat));
  return result;
}

double get_a_lift_mag(state *state, vehicle *vehicle, atm_cond *atm_cond) {
  double v = norm(state->velocity);
  double dynamic_pressure = 0.5 * atm_cond->density * v * v;
  double a_lift = vehicle->rv.c_l_alpha * fabs(state->alpha) *
                  dynamic_pressure * vehicle->rv.rv_area / vehicle->rv.rv_mass;
  return a_lift;
}

cartvec get_lift_acc(state *true_state, state *est_state, runparams *run_params,
                     vehicle *vehicle, atm_cond *atm_cond, double t,
                     grav *grav) {
  if (run_params->rv_maneuv != 1 || !is_reentry(est_state, t)) {
    return zeros();
  }
  cartvec xhat, yhat, zhat;
  get_body_frame(true_state, est_state, run_params, t, atm_cond, &xhat, &yhat,
                 &zhat, 0, grav);
  double a_lift_mag = get_a_lift_mag(true_state, vehicle, atm_cond);
  return smultiply(zhat, a_lift_mag);
}

double get_deflection_angular_speed(state *est_state, vehicle *vehicle,
                                    atm_cond *atm_cond, runparams *run_params,
                                    double t, grav *grav) {
  if (!run_params->rv_maneuv || !is_reentry(est_state, t)) {
    return 0;
  }
  double max_deflection_angle = run_params->max_deflection_angle * M_PI / 180;
  double max_deflection_speed =
      max_deflection_angle /
      (run_params->deflection_time * run_params->gearing_ratio);
  cartvec a_cmd = prop_nav(est_state, run_params, grav);
  double est_lift_mag = get_a_lift_mag(est_state, vehicle, atm_cond);
  double acceleration_to_gain = norm(a_cmd) - est_lift_mag;

  double P = run_params->Kp * acceleration_to_gain;
  double dot_delta = clip(P, -max_deflection_speed, max_deflection_speed);
  return dot_delta;
}

double get_aoa_angular_acceleration(state *true_state, runparams *run_params,
                                    vehicle *vehicle, atm_cond *atm_cond,
                                    double t) {
  if (run_params->rv_maneuv != 1 || !is_reentry(true_state, t)) {
    return 0;
  }
  double v = norm(true_state->velocity);
  double dynamic_pressure = 0.5 * atm_cond->density * v * v;
  double p = vehicle->rv.c_m_q * dynamic_pressure * vehicle->rv.rv_area *
             vehicle->rv.rv_length * vehicle->rv.rv_length /
             (2 * vehicle->rv.Iyy * v);
  double k = vehicle->rv.c_m_alpha * dynamic_pressure * vehicle->rv.rv_area *
             vehicle->rv.rv_length / vehicle->rv.Iyy;
  double n = vehicle->rv.c_m_delta * dynamic_pressure * vehicle->rv.rv_area *
             vehicle->rv.rv_length / vehicle->rv.Iyy;

  // The force produced is a function of the orientation of the flap (deflection
  // angle) and the direction of the freestream air velocity (accounted for by
  // the angle of attack, alpha)
  double delta_effective = true_state->deflection_angle + true_state->alpha;

  // The moment is limited by the maximum flap force
  double max_flap_force = get_max_flap_force(run_params, vehicle);
  double max_forcing_angle = max_flap_force *
                             fabs(vehicle->rv.x_flap - vehicle->rv.x_com) /
                             vehicle->rv.Iyy;

  double ddot_alpha =
      p * true_state->d_alpha_dt + k * true_state->alpha +
      clip(n * delta_effective, -max_forcing_angle, max_forcing_angle);
  return ddot_alpha;
}

#endif