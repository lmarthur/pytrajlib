#ifndef INTEGRATE_H
#define INTEGRATE_H

#include "models/state.h"
#include "rng/rng.h"
#include "utils/derivatives.h"
#include <math.h>
#include <stdio.h>

/**
 @returns 1 if integration should continue, 0 if integration should stop
 */
typedef int (*EventFunction)(double t, multistate *state, dualargs *args);

/**
 * Dummy event function that always returns 1 (continue integration)
 */
int dummy_event(double t, multistate *state, dualargs *args) { return 1; }

/**
 * Quaternion update: q_new = q_old otimes q_omega
 * where q_omega is the quaternion representing rotation by angular velocity *
 * dt
 */
quaternion quaternion_update(state current_state, state state_deriv_drift,
                             double dt) {
    cartvec angular_velocity;
    angular_velocity.x = state_deriv_drift.quaternion.x;
    angular_velocity.y = state_deriv_drift.quaternion.y;
    angular_velocity.z = state_deriv_drift.quaternion.z;

    double norm_angular_velocity = norm(angular_velocity);
    double half_angle = norm_angular_velocity * dt / 2;

    double real_component = cos(half_angle);
    cartvec vector_component = smultiply(
        divide(angular_velocity, norm_angular_velocity), sin(half_angle));

    quaternion q_omega;
    q_omega.w = real_component;
    q_omega.x = vector_component.x;
    q_omega.y = vector_component.y;
    q_omega.z = vector_component.z;

    quaternion new_quaternion = qmultiply(current_state.quaternion, q_omega);
    // TODO normalize?
    return new_quaternion;
}

/**
 * Use the Milstein (reduces to Euler-Maruyama for diffusion term
 * because the noise is additive; gives Ito solution) method for integrating the
 * position, velocity, lift acceleration, and gyro error. Use a second-order
 * position update and an exact quaternion updator method based on the angular
 * velocity.
 *
 * @param current_state: initial state object
 * @param drift: function for deterministic terms
 * @param diffusion: function for stochastic terms
 * @param args: additional arguments for the drift and diffusion functions
 * @param max_steps: maximum number of steps the integrator will take
 * @param dt: time step
 * @param event: function that returns 1 on each step if integration should
 * continue and 0 if it should stop
 * @param state_history: array to store state at each integration step. Expected
 * size is max_steps.
 * @return: number of steps taken
 */
int euler_maruyama(multistate current_state, DerivFunction drift,
                   DerivFunction diffusion, dualargs args, int max_steps,
                   double *t, double dt, EventFunction event,
                   multistate *state_history) {
    // printf("Inside euler_maruyama...\n");

    int step_counter = 0;

    // Store initial state
    state_history[0] = current_state;

    while (event(*t, &current_state, &args) && (step_counter < max_steps)) {
        // printf("Integration step %f\n", step_counter);
        multistate drift_deriv = drift(*t, &current_state, &args);
        multistate diffusion_deriv = diffusion(*t, &current_state, &args);

        // Setup state pointers for true, estimated, and desired states
        state *states[] = {&current_state.true_state, &current_state.est_state,
                           &current_state.des_state};
        state *deriv_states[] = {&drift_deriv.true_state,
                                 &drift_deriv.est_state,
                                 &drift_deriv.des_state};
        state *diffusion_states[] = {&diffusion_deriv.true_state,
                                     &diffusion_deriv.est_state,
                                     &diffusion_deriv.des_state};

        // Loop over true, estimated, and desired states (if applicable)
        int num_states = args.update_desired_state ? 3 : 2;
        for (int i = 0; i < num_states; i++) {
            cartvec velocity = deriv_states[i]->position;
            cartvec acceleration = deriv_states[i]->velocity;
            cartvec d_a_lift_dt = deriv_states[i]->a_lift;
            cartvec d_a_lift_avail_dt = deriv_states[i]->a_lift_avail;

            // Second-order position update: dx = velocity dt + 1/2 acceleration
            // dt^2
            states[i]->position =
                add(add(states[i]->position, smultiply(velocity, dt)),
                    smultiply(acceleration, 0.5 * dt * dt));

            // Velocity update: dv = acceleration dt
            states[i]->velocity =
                add(states[i]->velocity, smultiply(acceleration, dt));

            // Lift acceleration update
            states[i]->a_lift =
                add(states[i]->a_lift, smultiply(d_a_lift_dt, dt));
            states[i]->a_lift_avail =
                add(states[i]->a_lift_avail, smultiply(d_a_lift_avail_dt, dt));

            // Quaternion update
            states[i]->quaternion =
                quaternion_update(*states[i], *deriv_states[i], dt);
        }

        // Gyro error deterministic and stochastic updates
        // The estimated state is the only state with gyro error
        // The stochastic update uses the Milstein method for additive noise
        // (reduces to Euler-Maruyama)
        double dW[2] = {ran_gaussian(sqrt(dt)), ran_gaussian(sqrt(dt))};
        states[1]->gyro_error.lat +=
            deriv_states[1]->gyro_error.lat * dt +
            diffusion_states[1]->gyro_error.lat * dW[0];
        states[1]->gyro_error.lon +=
            deriv_states[1]->gyro_error.lon * dt +
            diffusion_states[1]->gyro_error.lon * dW[1];

        *t += dt;
        step_counter++;

        // Store state in history
        state_history[step_counter] = current_state;
    }

    return step_counter;
}

#endif
