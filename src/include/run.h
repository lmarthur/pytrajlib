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

#define MAX_RUNS 1000

typedef struct {
    state impact_event;
    double t;
} integration_result;

typedef struct {
    integration_result results[MAX_RUNS];
} integration_results;

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

integration_result fly_single(runparams run_params) {
    // Initialize state
    multistate current_state;
    current_state.true_state = init_true_state(run_params);
    current_state.est_state = init_est_state(run_params);
    current_state.des_state = init_est_state(run_params);

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

    // Allocate state history array on stack
    // Arrays sized to hold maximum expected steps for each phase
    int max_boost_steps =
        (int)(args.vehicle.booster.total_burn_time / run_params.boost_dt) + 1;
    int max_midcourse_steps = (int)(3600 / run_params.midcourse_dt);
    int max_reentry_steps = (int)(60 / run_params.reentry_dt);
    multistate boost_history[max_boost_steps];
    multistate midcourse_history[max_midcourse_steps];
    multistate reentry_history[max_reentry_steps];

    // Boost phase
    double t = 0;
    int end_boost_idx = euler_maruyama(
        current_state, ballistic_derivs, dummy_deriv, args, max_boost_steps, &t,
        run_params.boost_dt, end_boost_event, boost_history);

    args.update_desired_state = 0;

    // Midcourse phase
    int end_midcourse_idx = euler_maruyama(
        boost_history[end_boost_idx], ballistic_derivs, dummy_deriv, args,
        max_midcourse_steps, &t, run_params.midcourse_dt, end_midcourse_event,
        midcourse_history);

    // Reentry phase
    int end_reentry_idx =
        euler_maruyama(midcourse_history[end_midcourse_idx], ballistic_derivs,
                       dummy_deriv, args, max_reentry_steps, &t,
                       run_params.reentry_dt, impact_event, reentry_history);

    // print estimated impact altitude
    double est_impact_altitude =
        get_altitude(reentry_history[end_reentry_idx].est_state);

    // Use impact_linterp to get precise impact state at altitude 0
    double true_impact_time;
    state true_impact_state =
        impact_linterp(&reentry_history[end_reentry_idx - 1].true_state,
                       &reentry_history[end_reentry_idx].true_state, 0,
                       t - run_params.reentry_dt, t, &true_impact_time);

    double est_impact_time;
    state est_impact_state =
        impact_linterp(&reentry_history[end_reentry_idx - 1].est_state,
                       &reentry_history[end_reentry_idx].est_state, 0,
                       t - run_params.reentry_dt, t, &est_impact_time);

    printf("est impact time, true impact time: %f, %f\n", est_impact_time,
           true_impact_time);
    printf("est impact altitude before interpolation: %f meters\n",
           est_impact_altitude);
    printf("true impact position: x=%f, y=%f, z=%f meters\n",
           true_impact_state.position.x, true_impact_state.position.y,
           true_impact_state.position.z);
    printf("est  impact position: x=%f, y=%f, z=%f meters\n",
           est_impact_state.position.x, est_impact_state.position.y,
           est_impact_state.position.z);

    integration_result res;
    res.impact_event = true_impact_state;
    res.t = true_impact_time;
    return res;
}

integration_results fly(int N) {
    integration_results results;

    for (int i = 0; i < N; i++) {
        runparams run_params = init_default_run_params();

        integration_result res = fly_single(run_params);
        results.results[i] = res;

        // Update tqdm loading bar in Python
#ifdef FROM_PYTHON
        int five_percent = (int)(N / 20);
        five_percent = (int)clip(five_percent, 1, 100);
        if ((i + 1) % five_percent == 0) {
            update_loading_bar(i + 1, N);
        }
        if (i == N - 1) {
            update_loading_bar(N, N);
        }
#endif
    }
    return results;
}

#endif