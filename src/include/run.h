#ifndef RUN_H
#define RUN_H

#include "integrator/integrate.h"
#include "math/interpolate.h"
#include "models/state.h"
#include "rng/rng.h"
#include "utils/derivatives.h"
#include "utils/utils.h"
#include <math.h>
#include <stdio.h>

/**
 * Event function that returns 1 (continue integration) if during boost phase
 * and 0 (stop integration) if after boost phase.
 */
int end_boost_event(double t, multistate *state, dualargs *args) {
  if (t >= args->vehicle.booster.total_burn_time) {
    return 0; // Stop integration
  }
  return 1;
}

/**
 * Event function that returns 1 (continue integration) if during midcourse
 * phase and 0 (stop integration) if after midcourse phase. Note: The event ends
 * at 120km, not 100km for buffer when using a smaller time step below 100km.
 */
int end_midcourse_event(double t, multistate *state, dualargs *args) {
  double altitude = get_altitude(state->true_state);
  if (altitude < 120e3 && (t >= args->vehicle.booster.total_burn_time)) {
    return 0; // Stop integration
  }
  return 1;
}

/**
 * Impact event function that returns 1 (continue integration) if the altitude
 * is above zero and 0 (stop integration) if the altitude is zero or below.
 */
int impact_event(double t, multistate *state, dualargs *args) {
  double altitude = get_altitude(state->true_state);
  if (altitude <= 0 && t > 5) {
    return 0; // Stop integration
  }
  return 1;
}

void run() {
  printf("starting integration run.h\n");

  runparams run_params = init_default_run_params();
  printf("run params init done\n");
  // Initialize state
  multistate current_state;
  current_state.true_state = init_state(run_params);
  current_state.est_state = init_state(run_params);
  current_state.des_state = init_state(run_params);
  printf("state init done\n");

  // Initialize integrator fn_args
  dualatm atm;
  atm.true_atm = init_exp_atm(run_params);
  atm.est_atm = init_exp_atm(run_params);

  dualargs args;
  args.dual_atm = atm;
  args.run_params = run_params;
  args.vehicle = init_mmiii_ballistic(run_params);
  args.gravity = init_grav(run_params);
  args.update_desired_state = 1;

  double dt = 1;
  int max_steps = 4000;

  // Allocate state history array on stack
  multistate boost_history[max_steps];
  multistate midcourse_history[max_steps];
  multistate reentry_history[max_steps];

  printf("about to integrate...\n");

  double t0 = 0;
  int end_boost_idx =
      euler_maruyama(current_state, ballistic_derivs, dummy_deriv, args,
                     max_steps, t0, dt, end_boost_event, boost_history);
  printf("boost final position: %f %f %f\n",
         boost_history[end_boost_idx].true_state.position.x,
         boost_history[end_boost_idx].true_state.position.y,
         boost_history[end_boost_idx].true_state.position.z);
  printf("boost final time : %f\n", (double)end_boost_idx * dt);
  printf("number of boost steps: %d\n", end_boost_idx);
  t0 += end_boost_idx * dt;
  int end_midcourse_idx = euler_maruyama(
      boost_history[end_boost_idx], ballistic_derivs, dummy_deriv, args,
      max_steps, t0, dt, end_midcourse_event, midcourse_history);
  printf("midcourse final position: %f %f %f\n",
         midcourse_history[end_midcourse_idx].true_state.position.x,
         midcourse_history[end_midcourse_idx].true_state.position.y,
         midcourse_history[end_midcourse_idx].true_state.position.z);
  // midcourse altitude
  double midcourse_altitude =
      get_altitude(midcourse_history[end_midcourse_idx].true_state);
  printf("midcourse altitude : %f\n", midcourse_altitude);
  printf("midcourse final time : %f\n", (double)(end_midcourse_idx * dt) + t0);
  printf("number of midcourse steps: %d\n", end_midcourse_idx);

  t0 += end_midcourse_idx * dt;
  dt = 0.01;
  max_steps = 10000;
  int end_reentry_idx = euler_maruyama(
      midcourse_history[end_midcourse_idx], ballistic_derivs, dummy_deriv, args,
      max_steps, t0, dt, impact_event, reentry_history);
  printf("final position: %f %f %f\n",
         reentry_history[end_reentry_idx].true_state.position.x,
         reentry_history[end_reentry_idx].true_state.position.y,
         reentry_history[end_reentry_idx].true_state.position.z);
  // final altitude
  double final_altitude =
      get_altitude(reentry_history[end_reentry_idx].true_state);
  printf("final altitude : %f\n", final_altitude);
  printf("final time : %f\n", (double)(end_reentry_idx * dt) + t0);
  printf("number of reentry steps: %d\n", end_reentry_idx);
  // Use impact_linterp to get precise impact state at altitude 0
  if (end_reentry_idx > 0) {
    double impact_time = 0;
    multistate impact_state =
        impact_linterp(&reentry_history[end_reentry_idx - 1],
                       &reentry_history[end_reentry_idx], 0,
                       (double)((end_reentry_idx - 1) * dt) + t0,
                       (double)(end_reentry_idx * dt) + t0, &impact_time);
    printf("\ninterpolated impact position: %f %f %f\n",
           impact_state.true_state.position.x,
           impact_state.true_state.position.y,
           impact_state.true_state.position.z);
    printf("interpolated impact altitude: %f\n",
           get_altitude(impact_state.true_state));
    printf("interpolated impact time: %f\n", impact_time);
  }

  max_steps = 5000;
  multistate state_history[max_steps];
  dt = 1;
  int final_step =
      euler_maruyama(current_state, ballistic_derivs, dummy_deriv, args,
                     max_steps, 0, dt, impact_event, state_history);
  printf("\n\ncomplete final position: %f %f %f\n",
         state_history[final_step].true_state.position.x,
         state_history[final_step].true_state.position.y,
         state_history[final_step].true_state.position.z);
  printf("complete final time : %f\n", (double)final_step * dt);
}

#endif