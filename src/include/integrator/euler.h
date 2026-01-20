#ifndef EULER_H
#define EULER_H

#include <math.h>
#include "models/state.h"
#include "utils/utils.h"
#include "utils/derivatives.h"


/**
 @returns 1 if integration should continue, 0 if integration should stop
 */
typedef int (*EventFunction)(double t, state *state, integrator_args *args);

/**
 * Dummy event function that always returns 1 (continue integration)
 */
int dummy_event(double t, state *state, integrator_args *args) {
    return 1;
}


/**
 * Quaternion update: q_new = q_old otimes q_omega
 * where q_omega is the quaternion representing rotation by angular velocity * dt 
 */
quaternion quaternion_update(state current_state, state state_deriv_drift, double dt) {
    cartvec angular_velocity;
    angular_velocity.x = state_deriv_drift.quaternion.x;
    angular_velocity.y = state_deriv_drift.quaternion.y;
    angular_velocity.z = state_deriv_drift.quaternion.z;

    double norm_angular_velocity = norm(angular_velocity);
    double half_angle = norm_angular_velocity * dt / 2;

    double real_component = cos(half_angle);
    cartvec vector_component = smultiply(divide(angular_velocity, norm_angular_velocity), sin(half_angle));

    quaternion q_omega;
    q_omega.w = real_component;
    q_omega.x = vector_component.x;
    q_omega.y = vector_component.y;
    q_omega.z = vector_component.z;

    quaternion new_quaternion = qmultiply(current_state.quaternion, q_omega);

    return new_quaternion;
}

/**
 * Euler-Maruyama method for numerical integration of stochastic differential 
 * equations. This is the semi-implicit variant — assuming the derivative 
 * functions use the v_{n+1} instead of v_n for calculating the position.
 * 
 * @param current_state: initial state object
 * @param drift: function for deterministic term
 * @param args: additional arguments for the drift and diffusion functions
 * @param max_steps: maximum number of steps the integrator will take
 * @param dt: time step
 * @param event: function that returns 1 on each step if integration should continue and 0 if it should stop
 * @return: final state after integration
 */
state euler(state current_state, DerivFunction get_deriv, integrator_args args, int max_steps, double dt, EventFunction event) {
    double t = 0;
    int step_counter = 0;
    while (dummy_event(t, &current_state, &args) & (step_counter < max_steps)) {
        state state_deriv = get_deriv(t, &current_state, &args);
        
        // Update velocity 
        current_state.velocity.x += state_deriv.velocity.x * dt;
        current_state.velocity.y += state_deriv.velocity.y * dt;
        current_state.velocity.z += state_deriv.velocity.z * dt;
        
        // Update position (semi-implicit: use new velocity for position update)
        current_state.position.x += current_state.velocity.x * dt;
        current_state.position.y += current_state.velocity.y * dt;
        current_state.position.z += current_state.velocity.z * dt;
        
        // Update quaternion
        current_state.quaternion = quaternion_update(current_state, state_deriv, dt);
        t += dt;
        step_counter++;
    }
    
    return current_state;
}

#endif
