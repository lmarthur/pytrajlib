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
 * The lift acceleration magnitude is modeled from the angle of attack at each
 * time step:
 * $$
 * \begin{align}
 * a_L = C_{L\alpha} \alpha \bar q A.
 * \end{align}
 * $$
 *
 * @param state Pointer to state providing velocity and angle of attack.
 * @param vehicle Pointer to vehicle model constants.
 * @param atm_cond Pointer to atmospheric conditions.
 * @return Lift acceleration magnitude in m/s^2.
 */
double get_a_lift_mag(state *state, vehicle *vehicle, atm_cond *atm_cond) {
  double v = norm(state->velocity);
  double dynamic_pressure = 0.5 * atm_cond->density * v * v;
  double a_lift = vehicle->rv.c_l_alpha * fabs(state->alpha) *
                  dynamic_pressure * vehicle->rv.rv_area / vehicle->rv.rv_mass;
  return a_lift;
}

/**
 * The lift acceleration is directed perpendicular to the vehicle's relative
 * velocity toward the commanded lift direction, and is constructed from the
 * body-frame lift axis.
 * $$
 * \begin{align}
 * \vec a_L = a_L\frac{\vec a_c - \vec a_{g\perp}}{|\vec a_c - \vec a_{g\perp}|}
 * \end{align}
 * $$
 *
 * @param true_state Pointer to true state used for lift magnitude and frame.
 * @param est_state Pointer to estimated state used for reentry checks.
 * @param run_params Pointer to run configuration parameters.
 * @param vehicle Pointer to vehicle model constants.
 * @param atm_cond Pointer to atmospheric conditions.
 * @param t Current simulation time in seconds.
 * @param grav Pointer to gravity model used by body-frame construction.
 * @return Lift acceleration vector in inertial Cartesian coordinates.
 */
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

/**
 * For a given acceleration command $\vec a_c$, current lift acceleration
 * $\vec a_L$, and gravity component perpendicular to velocity
 * $\vec a_{g\perp}$, the commanded change in acceleration is
 * $$
 * \begin{align}
 * \Delta \vec a_c = \vec a_c - \vec a_L - \vec a_{g\perp}.
 * \end{align}
 * $$
 * For acceleration magnitude to be gained $\Delta a_c$ and proportional gain
 * $K_p$, the commanded flap deflection angular speed is clipped by
 * $\dot\delta_\text{max}$:
 * $$
 * \begin{align}
 * \dot \delta = \text{clip}(\Delta a_c K_p, -\dot\delta_\text{max},
 * \dot\delta_\text{max}).
 * \end{align}
 * $$
 *
 * @param est_state Pointer to estimated state.
 * @param vehicle Pointer to vehicle model constants.
 * @param atm_cond Pointer to atmospheric conditions.
 * @param run_params Pointer to run configuration parameters.
 * @param t Current simulation time in seconds.
 * @param grav Pointer to gravity model used by guidance.
 * @return Commanded flap deflection angular speed in rad/s.
 */
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

/**
 * The rotational equation of motion for angle of attack uses a Newtonian
 * pitching-moment approximation with effective flap angle
 * $\delta_\text{eff} = \delta + \alpha$.
 * $$
 * \begin{align}
 * I \ddot \alpha \approx (C_{M_\delta} \delta_\text{eff} + C_{M_\alpha} \alpha
 * + C_{M_q} \frac{c}{2V} \dot\alpha)\bar q A c.
 * \end{align}
 * $$
 * With
 * $$
 * \begin{align}
 * p=\frac{C_{Mq} \bar qAc^2}{2IV},\; k=\frac{C_{M\alpha}\bar q A c}{I},\;
 * n = \frac{C_{M\delta} \bar q A c}{I},
 * \end{align}
 * $$
 * the forced second-order model is
 * $$
 * \begin{align}
 * \ddot \alpha - p\dot\alpha - k \alpha =
 * \text{clip}(n\delta_\text{eff}, \frac{-F_\text{flap,max}r_\text{flap}}{I},
 * \frac{F_\text{flap,max}r_\text{flap}}{I}).
 * \end{align}
 * $$
 *
 * @param true_state Pointer to true state containing angle states.
 * @param run_params Pointer to run configuration parameters.
 * @param vehicle Pointer to vehicle model constants.
 * @param atm_cond Pointer to atmospheric conditions.
 * @param t Current simulation time in seconds.
 * @return Angular acceleration of angle of attack in rad/s^2.
 */
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