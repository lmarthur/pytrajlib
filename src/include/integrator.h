#ifndef INTEGRATOR_H
#define INTEGRATOR_H

#include <math.h>

#include "forces/drag.h"
#include "forces/gravity.h"
#include "forces/lift.h"
#include "forces/thrust.h"
#include "integrator.h"
#include "models/atmosphere.h"
#include "models/grav.h"
#include "models/sensors.h"
#include "models/state.h"
#include "models/vehicle.h"
#include "rng/rng.h"
#include "utils.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

// Define a series of functions to calculate acceleration components

/**
 * Advance the state by one Euler-Maruyama step. Note, it is not strictly EM
 * because the position is updated using the velocity and the acceleration.
 *
 * @param state Pointer to state to be updated in place
 * @param time_step Integration time step in seconds
 */
void euler_maruyama_step(runparams *run_params, imu *imu, vehicle *vehicle,
                         gnss *gnss, grav *true_grav, grav *est_grav,
                         atm_cond *true_atm_cond, atm_cond *est_atm_cond,
                         state *true_state, state *est_state,
                         double time_step) {
  cartvec a_thrust_true = {0};
  cartvec a_thrust_est = {0};
  // Update the thrust of the vehicle
  if (run_params->perfect_boost) {
    a_thrust_true =
        get_thrust_acceleration(true_state, vehicle, run_params, true_grav);
    a_thrust_est = a_thrust_true;
  } else {
    a_thrust_est =
        get_thrust_acceleration(est_state, vehicle, run_params, est_grav);
    a_thrust_true = a_thrust_est;
  }
  // If Lambert Guidance fails, quickly exit
  if (isnan(a_thrust_true.x)) {
    // TODO do a better exit that does not fully crash
    exit(0);
    //   return new_true_state;
  }
  // Get the gravity acceleration
  cartvec a_grav_true = update_gravity(true_grav, true_state);
  cartvec a_grav_est = update_gravity(est_grav, est_state);

  // Get the drag acceleration
  cartvec a_drag_true =
      get_drag_acceleration(run_params, vehicle, true_atm_cond, true_state);
  cartvec a_drag_est =
      get_drag_acceleration(run_params, vehicle, est_atm_cond, est_state);

  cartvec true_d_a_lift_dt = {0};
  cartvec est_d_a_lift_dt = {0};
  cartvec true_d_a_lift_avail_dt = {0};
  cartvec est_d_a_lift_avail_dt = {0};

  double angle_v_grav =
      acos(dot(true_state->velocity, smultiply(true_state->position, -1)) /
           (norm(true_state->velocity) * norm(true_state->position)));

  // If maneuverable RV, use proportional navigation during reentry
  if (run_params->rv_maneuv == 1 && (angle_v_grav > 0) &&
      (angle_v_grav < M_PI_2) && (get_altitude(true_state->position) < 1e5)) {

    // Get lift jerk
    true_d_a_lift_dt =
        get_a_lift_jerk(true_state, run_params, vehicle, true_atm_cond);
    est_d_a_lift_dt =
        get_a_lift_jerk(est_state, run_params, vehicle, est_atm_cond);

    // Get available lift jerk
    int valid = get_a_lift_avail_jerk(
        true_state, est_state, run_params, vehicle, est_atm_cond,
        &true_d_a_lift_avail_dt, &est_d_a_lift_avail_dt);
    if (!valid) {
      true_d_a_lift_avail_dt = zeros();
      est_d_a_lift_avail_dt = zeros();
    }
  }

  // Calculate the total acceleration components
  cartvec a_total_true = add(add(a_grav_true, a_drag_true),
                             add(true_state->a_lift, a_thrust_true));
  cartvec a_total_est =
      add(add(a_grav_est, a_drag_est), add(est_state->a_lift, a_thrust_est));

  if ((run_params->gnss_nav == 1) &&
      (get_altitude(true_state->position) > 100e3)) {
    // GNSS Measurement
    gnss_measurement(gnss, true_state, est_state);
  }

  if (run_params->ins_nav == 1) {
    // INS Measurement
    a_total_est = imu_measurement(imu, true_state, est_state, a_total_true,
                                  a_grav_true, a_grav_est);

    anglevec gyro_drift = get_gyro_drift(imu);
    double gyro_diffusion = get_gyro_diffusion(imu);

    anglevec drift_update = smultiply_angle(gyro_drift, time_step);
    anglevec dW = smultiply_angle(gaussian_anglevec(), sqrt(time_step));
    anglevec diffusion_update = smultiply_angle(dW, gyro_diffusion);
    est_state->gyro_error = add_anglevec(
        add_anglevec(est_state->gyro_error, drift_update), diffusion_update);
  }
  state *states[2] = {true_state, est_state};
  cartvec a_totals[2] = {a_total_true, a_total_est};
  cartvec d_a_lift_dts[2] = {true_d_a_lift_dt, est_d_a_lift_dt};
  cartvec d_a_lift_avail_dts[2] = {true_d_a_lift_avail_dt,
                                   est_d_a_lift_avail_dt};
  for (int i = 0; i < 2; i++) {
    states[i]->t = states[i]->t + time_step;
    states[i]->position =
        add(states[i]->position,
            add(smultiply(states[i]->velocity, time_step),
                smultiply(a_totals[i], 0.5 * time_step * time_step)));
    states[i]->velocity =
        add(states[i]->velocity, smultiply(a_totals[i], time_step));
    states[i]->a_lift =
        add(states[i]->a_lift, smultiply(d_a_lift_dts[i], time_step));
    states[i]->a_lift_avail = add(states[i]->a_lift_avail,
                                  smultiply(d_a_lift_avail_dts[i], time_step));
  }
}

#endif