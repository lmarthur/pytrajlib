#ifndef INTEGRATE_H
#define INTEGRATE_H

#include "models/state.h"
#include "rng/rng.h"
#include "utils/derivatives.h"
#include <math.h>

/**
 @returns 1 if integration should continue, 0 if integration should stop
 */
typedef int (*EventFunction)(double t, state *state, integrator_args *args);

/**
 * Dummy event function that always returns 1 (continue integration)
 */
int dummy_event(double t, state *state, integrator_args *args) { return 1; }

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
 * Use the Velocity Verlet method for integrating the deterministic drift
 * component, the Milstein (reduces to Euler-Maruyama for diffusion term
 * because the noise is additive) method for integrating the stochastic
 * gyro error, and an exact quaternion updator method based on the angular
 * velocity for updating the quaternion.
 *
 * @param current_state: initial state object
 * @param drift: function for deterministic terms
 * @param diffusion: function for stochastic terms
 * @param args: additional arguments for the drift and diffusion functions
 * @param max_steps: maximum number of steps the integrator will take
 * @param dt: time step
 * @param event: function that returns 1 on each step if integration should
 * continue and 0 if it should stop
 * @return: final state after integration
 */
state integrate(state current_state, DerivFunction drift,
                DerivFunction diffusion, integrator_args args, int max_steps,
                double dt, EventFunction event) {
  double t = 0;
  int step_counter = 0;
  state drift_deriv = drift(t, &current_state, &args);
  t += dt;

  while (dummy_event(t, &current_state, &args) && (step_counter < max_steps)) {
    // Velocity Verlet for integrating the position and velocity updates
    cartvec d_a_lift_dt = drift_deriv.a_lift;
    cartvec d_a_lift_avail_dt = drift_deriv.a_lift_avail;

    // Update position
    cartvec acc = drift_deriv.velocity;
    current_state.position.x +=
        current_state.velocity.x * dt + 0.5 * dt * dt * acc.x;
    current_state.position.y +=
        current_state.velocity.y * dt + 0.5 * dt * dt * acc.y;
    current_state.position.z +=
        current_state.velocity.z * dt + 0.5 * dt * dt * acc.z;

    // Velocity half-step
    current_state.velocity.x += 0.5 * dt * acc.x;
    current_state.velocity.y += 0.5 * dt * acc.y;
    current_state.velocity.z += 0.5 * dt * acc.z;

    // Get acceleration & other derivatives
    drift_deriv = drift(t, &current_state, &args);

    // Update velocity
    current_state.velocity.x += drift_deriv.velocity.x * 0.5 * dt;
    current_state.velocity.y += drift_deriv.velocity.y * 0.5 * dt;
    current_state.velocity.z += drift_deriv.velocity.z * 0.5 * dt;

    // Update lift acceleration
    current_state.a_lift.x += (d_a_lift_dt.x + drift_deriv.a_lift.x) * 0.5 * dt;
    current_state.a_lift.y += (d_a_lift_dt.y + drift_deriv.a_lift.y) * 0.5 * dt;
    current_state.a_lift.z += (d_a_lift_dt.z + drift_deriv.a_lift.z) * 0.5 * dt;

    // Update available lift acceleration
    current_state.a_lift_avail.x +=
        (d_a_lift_avail_dt.x + drift_deriv.a_lift_avail.x) * 0.5 * dt;
    current_state.a_lift_avail.y +=
        (d_a_lift_avail_dt.y + drift_deriv.a_lift_avail.y) * 0.5 * dt;
    current_state.a_lift_avail.z +=
        (d_a_lift_avail_dt.z + drift_deriv.a_lift_avail.z) * 0.5 * dt;

    // Milstein (equivalent to Euler-Maruyama with additive noise) for
    // integrating the gyro error
    state diffusion_deriv = diffusion(t, &current_state, &args);

    // Update gyro noise drift (deterministic) component
    current_state.gyro_error.lat += drift_deriv.gyro_error.lat * dt;
    current_state.gyro_error.lon += drift_deriv.gyro_error.lon * dt;
    // Update gyro noise diffusion (stochastic) component
    double dW[2] = {ran_gaussian(sqrt(dt)), ran_gaussian(sqrt(dt))};
    current_state.gyro_error.lat += diffusion_deriv.gyro_error.lat * dW[0];
    current_state.gyro_error.lon += diffusion_deriv.gyro_error.lon * dW[1];

    // Update quaternion based on angular velocity
    current_state.quaternion =
        quaternion_update(current_state, drift_deriv, dt);

    t += dt;
    step_counter++;
  }

  return current_state;
}

#endif
