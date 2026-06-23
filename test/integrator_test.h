#define _USE_MATH_DEFINES

#include "../src/include/integrator.h"
#include <stdint.h>
#include <tau/tau.h>

// Constant acceleration injected into the mock drift model below so the test
// can exercise the integrator with simple closed-form motion.
static cartvec physics_test_acceleration = {0};
static runparams rp = {0};

// Minimal deterministic drift used to test `euler_maruyama_step()` in
// isolation. Position derivative is velocity, and velocity derivative is the
// configurable constant acceleration above.
static int physics_test_drift(runparams *run_params, imu *imu, vehicle *vehicle,
                              grav *true_grav, grav *est_grav,
                              atm_cond *true_atm_cond, atm_cond *est_atm_cond,
                              state *true_state, state *est_state,
                              double true_t, double est_t,
                              state *true_state_drift, state *est_state_drift) {
  (void)run_params;
  (void)imu;
  (void)vehicle;
  (void)true_grav;
  (void)est_grav;
  (void)true_atm_cond;
  (void)est_atm_cond;
  (void)true_t;
  (void)est_t;

  *true_state_drift = (state){0};
  *est_state_drift = (state){0};

  true_state_drift->position = true_state->velocity;
  est_state_drift->position = est_state->velocity;
  true_state_drift->velocity = physics_test_acceleration;
  est_state_drift->velocity = physics_test_acceleration;

  return 1;
}

// No stochastic diffusion term for this test: it is intended to validate only
// the deterministic Euler-Maruyama update path.
static void physics_test_diffusion(imu *imu, state *est_state_diffusion,
                                   state *true_state_diffusion,
                                   runparams *run_params) {
  (void)imu;
  (void)run_params;
  *est_state_diffusion = (state){0};
}

// Zero drift helper for testing the stochastic term in isolation.
static int physics_test_zero_drift(runparams *run_params, imu *imu,
                                   vehicle *vehicle, grav *true_grav,
                                   grav *est_grav, atm_cond *true_atm_cond,
                                   atm_cond *est_atm_cond, state *true_state,
                                   state *est_state, double true_t,
                                   double est_t, state *true_state_drift,
                                   state *est_state_drift) {
  (void)run_params;
  (void)imu;
  (void)vehicle;
  (void)true_grav;
  (void)est_grav;
  (void)true_atm_cond;
  (void)est_atm_cond;
  (void)true_state;
  (void)est_state;
  (void)true_t;
  (void)est_t;

  *true_state_drift = (state){0};
  *est_state_drift = (state){0};
  return 1;
}

// Constant diffusion helper that injects known coefficients into the
// orientation-angle-change SDE so the Wiener increment scaling can be checked
// directly.
static void physics_test_constant_diffusion(imu *imu,
                                            state *est_state_diffusion,
                                            state *true_state_diffusion,
                                            runparams *run_params) {
  (void)imu;
  (void)run_params;
  *est_state_diffusion = (state){0};
  est_state_diffusion->orientation_angle_change.x = 2.0;
  est_state_diffusion->orientation_angle_change.y = -3.0;
}

// Parameters/helpers for deterministic scalar exponential decay,
// x' = -lambda x, represented in state.position.x.
static double exp_decay_lambda = 1.0;

static int exp_decay_drift(runparams *run_params, imu *imu, vehicle *vehicle,
                           grav *true_grav, grav *est_grav,
                           atm_cond *true_atm_cond, atm_cond *est_atm_cond,
                           state *true_state, state *est_state, double true_t,
                           double est_t, state *true_state_drift,
                           state *est_state_drift) {
  (void)run_params;
  (void)imu;
  (void)vehicle;
  (void)true_grav;
  (void)est_grav;
  (void)true_atm_cond;
  (void)est_atm_cond;
  (void)true_t;
  (void)est_t;

  *true_state_drift = (state){0};
  *est_state_drift = (state){0};

  true_state_drift->position.x = -exp_decay_lambda * true_state->position.x;
  est_state_drift->position.x = -exp_decay_lambda * est_state->position.x;
  return 1;
}

static void exp_decay_zero_diffusion(imu *imu, state *est_state_diffusion,
                                     state *true_state_diffusion,
                                     runparams *run_params) {
  (void)imu;
  (void)run_params;
  *est_state_diffusion = (state){0};
}

TEST(integrator, integrate_quaternion_step) {
  // Case 1: identity attitude with a delta rotation about the body X axis
  // represented via `orientation_angle_change` as a 3-vector `delta_angle_B`.
  state s = {0};
  s.q_EB = identity_quaternion();
  s.orientation_angle_change.x = M_PI; // delta vector = (pi, 0, 0)
  s.orientation_angle_change.y = 0.0;
  s.orientation_angle_change.z = 0.0;

  quaternion q1 = integrate_quaternion_step(s);
  REQUIRE_LT(fabs(q1.w), 1e-12);
  REQUIRE_LT(fabs(q1.y), 1e-12);
  REQUIRE_LT(fabs(q1.z), 1e-12);
  REQUIRE_LT(fabs(q1.x - 1.0), 1e-12);

  // Case 2: excessive-length quaternion should be normalized to unit length
  state s_bad = {0};
  s_bad.q_EB = (quaternion){2.0, -1.0, 0.5, -0.25};
  s_bad.orientation_angle_change = (cartvec){0};
  quaternion q2 = integrate_quaternion_step(s_bad);
  REQUIRE_LT(fabs(qnorm(q2) - 1.0), 1e-12);
}

TEST(integrator, euler_maruyama_step) {
  // Scenario 1: zero velocity and zero acceleration should leave the state
  // unchanged after one step.
  state true_state = {0};
  state est_state = {0};
  double true_t = 0;
  double est_t = 0;
  double time_step = 1;

  physics_test_acceleration = zeros();
  rp.actuator_resolution = 1.0;    // degrees
  rp.max_deflection_angle = 180.0; // degrees, large enough to avoid clipping
  int success = euler_maruyama_step(
      &rp, NULL, NULL, NULL, NULL, NULL, NULL, &true_state, &est_state, &true_t,
      &est_t, time_step, physics_test_drift, physics_test_diffusion);

  REQUIRE_EQ(success, 1);
  REQUIRE_EQ(true_state.position.x, 0);
  REQUIRE_EQ(true_state.position.y, 0);
  REQUIRE_EQ(true_state.position.z, 0);
  REQUIRE_EQ(true_state.velocity.x, 0);
  REQUIRE_EQ(true_state.velocity.y, 0);
  REQUIRE_EQ(true_state.velocity.z, 0);

  // Scenario 2: constant velocity with zero acceleration should advance
  // position linearly while leaving velocity unchanged.
  true_state = (state){0};
  est_state = (state){0};
  true_t = 0;
  est_t = 0;
  true_state.velocity.x = 1;
  true_state.velocity.y = 1;
  true_state.velocity.z = 1;
  est_state.velocity = true_state.velocity;
  physics_test_acceleration = zeros();

  success = euler_maruyama_step(
      &rp, NULL, NULL, NULL, NULL, NULL, NULL, &true_state, &est_state, &true_t,
      &est_t, time_step, physics_test_drift, physics_test_diffusion);

  REQUIRE_EQ(success, 1);
  REQUIRE_EQ(true_state.position.x, time_step);
  REQUIRE_EQ(true_state.position.y, time_step);
  REQUIRE_EQ(true_state.position.z, time_step);
  REQUIRE_EQ(true_state.velocity.x, 1);
  REQUIRE_EQ(true_state.velocity.y, 1);
  REQUIRE_EQ(true_state.velocity.z, 1);

  // Scenario 3: zero initial velocity with constant acceleration should match
  // the built-in half-step position update: $x = \tfrac{1}{2}at^2$ and
  // $v = at$ for $t = 1$.
  true_state = (state){0};
  est_state = (state){0};
  true_t = 0;
  est_t = 0;
  physics_test_acceleration.x = 1;
  physics_test_acceleration.y = 1;
  physics_test_acceleration.z = 1;

  success = euler_maruyama_step(
      &rp, NULL, NULL, NULL, NULL, NULL, NULL, &true_state, &est_state, &true_t,
      &est_t, time_step, physics_test_drift, physics_test_diffusion);

  REQUIRE_EQ(success, 1);
  REQUIRE_EQ(true_state.position.x, 0.5);
  REQUIRE_EQ(true_state.position.y, 0.5);
  REQUIRE_EQ(true_state.position.z, 0.5);
  REQUIRE_EQ(true_state.velocity.x, time_step);
  REQUIRE_EQ(true_state.velocity.y, time_step);
  REQUIRE_EQ(true_state.velocity.z, time_step);

  // Scenario 4: nonzero initial velocity and constant acceleration should
  // combine both effects: $x = v_0 t + \tfrac{1}{2}at^2$ and $v = v_0 + at$.
  true_state = (state){0};
  est_state = (state){0};
  true_t = 0;
  est_t = 0;
  true_state.velocity.x = 1;
  true_state.velocity.y = 1;
  true_state.velocity.z = 1;
  est_state.velocity = true_state.velocity;
  physics_test_acceleration.x = 1;
  physics_test_acceleration.y = 1;
  physics_test_acceleration.z = 1;

  success = euler_maruyama_step(
      &rp, NULL, NULL, NULL, NULL, NULL, NULL, &true_state, &est_state, &true_t,
      &est_t, time_step, physics_test_drift, physics_test_diffusion);

  REQUIRE_EQ(success, 1);
  REQUIRE_EQ(true_state.position.x, 1.5);
  REQUIRE_EQ(true_state.position.y, 1.5);
  REQUIRE_EQ(true_state.position.z, 1.5);
  REQUIRE_EQ(true_state.velocity.x, 2);
  REQUIRE_EQ(true_state.velocity.y, 2);
  REQUIRE_EQ(true_state.velocity.z, 2);
}

TEST(integrator, euler_maruyama_step_diffusion) {
  // Seed the RNG so the sampled Wiener increment is reproducible.
  init_genrand64(UINT64_C(12345));
  reset_ran_gaussian();
  cartvec expected_dW = smultiply(gaussian_cartvec(), sqrt(0.25));

  // Reset the RNG so the integrator sees the exact same random draw.
  init_genrand64(UINT64_C(12345));
  reset_ran_gaussian();

  state true_state = {0};
  state est_state = {0};
  double true_t = 0;
  double est_t = 0;
  double time_step = 0.25;

  int success = euler_maruyama_step(&rp, NULL, NULL, NULL, NULL, NULL, NULL,
                                    &true_state, &est_state, &true_t, &est_t,
                                    time_step, physics_test_zero_drift,
                                    physics_test_constant_diffusion);

  REQUIRE_EQ(success, 1);
  REQUIRE_EQ(est_state.orientation_angle_change.x, 2.0 * expected_dW.x);
  REQUIRE_EQ(est_state.orientation_angle_change.y, -3.0 * expected_dW.y);
}

TEST(integrator, em_sra3_exponential_decay_agree) {
  state true_state_em = (state){0};
  state est_state_em = (state){0};
  state true_state_sra3 = (state){0};
  state est_state_sra3 = (state){0};

  true_state_em.position.x = 1.0;
  est_state_em.position.x = 1.0;
  true_state_sra3.position.x = 1.0;
  est_state_sra3.position.x = 1.0;

  double true_t_em = 0.0;
  double est_t_em = 0.0;
  double true_t_sra3 = 0.0;
  double est_t_sra3 = 0.0;

  const double time_step = 1e-3;
  const int num_steps = 1000;
  const double lambda = 0.7;
  const double total_time = time_step * num_steps;

  exp_decay_lambda = lambda;

  for (int i = 0; i < num_steps; i++) {
    int success = euler_maruyama_step(&rp, NULL, NULL, NULL, NULL, NULL, NULL,
                                      &true_state_em, &est_state_em, &true_t_em,
                                      &est_t_em, time_step, exp_decay_drift,
                                      exp_decay_zero_diffusion);
    REQUIRE_EQ(success, 1);
  }

  for (int i = 0; i < num_steps; i++) {
    int success =
        sra3_step(&rp, NULL, NULL, NULL, NULL, NULL, NULL, &true_state_sra3,
                  &est_state_sra3, &true_t_sra3, &est_t_sra3, time_step,
                  exp_decay_drift, exp_decay_zero_diffusion);
    REQUIRE_EQ(success, 1);
  }

  double exact = exp(-lambda * total_time);

  REQUIRE_LT(fabs(true_state_em.position.x - true_state_sra3.position.x), 1e-3);
  REQUIRE_LT(fabs(true_state_em.position.x - exact), 1e-3);
  REQUIRE_LT(fabs(true_state_sra3.position.x - exact), 1e-5);
}