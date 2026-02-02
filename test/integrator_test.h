#include <math.h>
#include <tau/tau.h>

#include "../src/include/integrator/integrate.h"

TEST(euler_maruyama, quaternion_update_small_rotation) {
  // Identity quaternion rotated about z-axis with small timestep
  state current_state = {0};
  current_state.quaternion = (quaternion){1.0, 0.0, 0.0, 0.0};

  state drift = {0};
  drift.quaternion =
      (quaternion){0.0, 0.0, 0.0, 1.0}; // angular velocity 1 rad/s about z

  double dt = 0.1;
  quaternion result = quaternion_update(current_state, drift, dt);

  double expected_w = cos(0.5 * dt);
  double expected_z = sin(0.5 * dt);

  REQUIRE_LT(fabs(result.w - expected_w), 1e-6);
  REQUIRE_EQ(result.x, 0.0);
  REQUIRE_EQ(result.y, 0.0);
  REQUIRE_LT(fabs(result.z - expected_z), 1e-6);
}

/**
 * Drift function for quaternion integration test: constant z-axis rotation
 */
multistate constant_z_rotation_drift(double t, multistate *current_state,
                                     dualargs *args) {
  multistate deriv = {0};
  deriv.true_state.quaternion =
      (quaternion){0.0, 0.0, 0.0, 1.0}; // 1 rad/s about z-axis
  return deriv;
}

/**
 * Diffusion function (zero for deterministic test)
 */
multistate constant_z_rotation_diffusion(double t, multistate *current_state,
                                         dualargs *args) {
  multistate deriv = {0};
  return deriv;
}

TEST(euler_maruyama, full_integration_quaternion_rotation) {
  // Test full integration with constant z-axis angular velocity
  multistate initial_state = {0};
  initial_state.true_state.quaternion =
      (quaternion){1.0, 0.0, 0.0, 0.0}; // identity

  dualargs args = {0};
  double dt = 0.01;    // 10ms time step
  int num_steps = 100; // total 1 second of integration

  // After 1 second rotating at 1 rad/s about z-axis, total rotation is 1 radian
  // Expected: q = [cos(0.5), 0, 0, sin(0.5)]
  double expected_w = cos(0.5);
  double expected_z = sin(0.5);

  multistate *state_history =
      (multistate *)malloc((num_steps + 1) * sizeof(multistate));
  int steps_taken = euler_maruyama(initial_state, constant_z_rotation_drift,
                                   constant_z_rotation_diffusion, args,
                                   num_steps, dt, dummy_event, state_history);

  multistate final_state = state_history[steps_taken];

  // Verify the quaternion was updated
  REQUIRE_LT(fabs(final_state.true_state.quaternion.w - expected_w), 1e-10);
  REQUIRE_LT(fabs(final_state.true_state.quaternion.x), 1e-10);
  REQUIRE_LT(fabs(final_state.true_state.quaternion.y), 1e-10);
  REQUIRE_LT(fabs(final_state.true_state.quaternion.z - expected_z), 1e-10);

  free(state_history);
}

/**
 * Drift function for exponential decay: dv/dt = -k*v (decay constant k=1.0)
 */
multistate exponential_decay(double t, multistate *current_state,
                             dualargs *args) {
  multistate deriv = {0};
  deriv.true_state.velocity.x = -1.0 * current_state->true_state.velocity.x;
  deriv.true_state.velocity.y = -1.0 * current_state->true_state.velocity.y;
  deriv.true_state.velocity.z = -1.0 * current_state->true_state.velocity.z;
  return deriv;
}

/**
 * Diffusion function for exponential decay (zero for deterministic test)
 */
multistate exponential_decay_diffusion(double t, multistate *current_state,
                                       dualargs *args) {
  multistate deriv = {0};
  return deriv;
}

TEST(euler_maruyama, exponential_decay) {
  // Test integration with exponential decay: dv/dt = -v
  // Solution: v(t) = v0 * exp(-t)
  multistate initial_state = {0};
  initial_state.true_state.velocity.x = 1.0;
  initial_state.true_state.velocity.y = 1.0;
  initial_state.true_state.velocity.z = 1.0;

  dualargs args = {0};
  double dt = 1e-3;
  int num_steps = 1000; // 1 second total

  multistate *state_history =
      (multistate *)malloc((num_steps + 1) * sizeof(multistate));
  int steps_taken = euler_maruyama(initial_state, exponential_decay,
                                   exponential_decay_diffusion, args, num_steps,
                                   dt, dummy_event, state_history);

  multistate final_state = state_history[steps_taken];

  // After 1 second with decay constant 1, v(1) = v0 * exp(-1)
  double expected_velocity = 1.0 * exp(-1.0);

  // Integration method has error < time step
  REQUIRE_LT(fabs(final_state.true_state.velocity.x - expected_velocity), dt);
  REQUIRE_LT(fabs(final_state.true_state.velocity.y - expected_velocity), dt);
  REQUIRE_LT(fabs(final_state.true_state.velocity.z - expected_velocity), dt);

  free(state_history);
}