#ifndef RUN_H
#define RUN_H

#include "integrator/integrate.h"
#include "models/state.h"
#include "rng/rng.h"
#include "utils/derivatives.h"
#include <math.h>
#include <stdio.h>

/**
 * Impact event function that returns 1 (continue integration) if the altitude
 * is above zero, and 0 (stop integration) if the altitude is zero or below.
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
  int max_steps = 5000;

  // Allocate state history array on stack
  multistate state_history[max_steps + 1];

  printf("about to integrate...\n");
  int final_step =
      euler_maruyama(current_state, ballistic_derivs, dummy_deriv, args,
                     max_steps, dt, impact_event, state_history);
  printf("final position: %f %f %f\n",
         state_history[final_step].true_state.position.x,
         state_history[final_step].true_state.position.y,
         state_history[final_step].true_state.position.z);
  printf("final time : %f\n", (double)final_step * dt);
}

#endif