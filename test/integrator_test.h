#include "../src/include/integrator.h"
#include <stdint.h>
#include <tau/tau.h>

// Constant acceleration injected into the mock drift model below so the test
// can exercise the integrator with simple closed-form motion.
static cartvec physics_test_acceleration = {0};

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
static void physics_test_diffusion(imu *imu, state *est_state_diffusion) {
  (void)imu;
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

// Constant diffusion helper that injects known coefficients into the gyro
// error SDE so the Wiener increment scaling can be checked directly.
static void physics_test_constant_diffusion(imu *imu,
                                            state *est_state_diffusion) {
  (void)imu;
  *est_state_diffusion = (state){0};
  est_state_diffusion->gyro_error.lat = 2.0;
  est_state_diffusion->gyro_error.lon = -3.0;
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
  int success = euler_maruyama_step(
      NULL, NULL, NULL, NULL, NULL, NULL, NULL, &true_state, &est_state,
      &true_t, &est_t, time_step, physics_test_drift, physics_test_diffusion);

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
      NULL, NULL, NULL, NULL, NULL, NULL, NULL, &true_state, &est_state,
      &true_t, &est_t, time_step, physics_test_drift, physics_test_diffusion);

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
      NULL, NULL, NULL, NULL, NULL, NULL, NULL, &true_state, &est_state,
      &true_t, &est_t, time_step, physics_test_drift, physics_test_diffusion);

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
      NULL, NULL, NULL, NULL, NULL, NULL, NULL, &true_state, &est_state,
      &true_t, &est_t, time_step, physics_test_drift, physics_test_diffusion);

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
  anglevec expected_dW = smultiply_angle(gaussian_anglevec(), sqrt(0.25));

  // Reset the RNG so the integrator sees the exact same random draw.
  init_genrand64(UINT64_C(12345));
  reset_ran_gaussian();

  state true_state = {0};
  state est_state = {0};
  double true_t = 0;
  double est_t = 0;
  double time_step = 0.25;

  int success = euler_maruyama_step(NULL, NULL, NULL, NULL, NULL, NULL, NULL,
                                    &true_state, &est_state, &true_t, &est_t,
                                    time_step, physics_test_zero_drift,
                                    physics_test_constant_diffusion);

  REQUIRE_EQ(success, 1);
  REQUIRE_EQ(true_state.gyro_error.lat, 0);
  REQUIRE_EQ(true_state.gyro_error.lon, 0);
  REQUIRE_EQ(est_state.gyro_error.lat, 2.0 * expected_dW.lat);
  REQUIRE_EQ(est_state.gyro_error.lon, -3.0 * expected_dW.lon);
}

static double sra3_test_diffusion_input = 1.0;

static int sra3_test_drift(runparams *run_params, imu *imu, vehicle *vehicle,
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

  true_state_drift->gyro_error.lat = -true_state->gyro_error.lat;
  est_state_drift->gyro_error.lat = -est_state->gyro_error.lat;

  sra3_test_diffusion_input = est_state->gyro_error.lat;
  return 1;
}

static void sra3_test_diffusion(imu *imu, state *est_state_diffusion) {
  (void)imu;

  *est_state_diffusion = (state){0};
  est_state_diffusion->gyro_error.lat = 0.5 * sra3_test_diffusion_input;
}

static void sra3_test_zero_diffusion(imu *imu, state *est_state_diffusion) {
  (void)imu;
  *est_state_diffusion = (state){0};
}

TEST(integrator, sra3_sde_stays_positive) {
  init_genrand64(UINT64_C(12345));
  reset_ran_gaussian();
  sra3_test_diffusion_input = 1.0;

  state true_state = {0};
  state est_state = {0};
  double true_t = 0.0;
  double est_t = 0.0;
  const double time_step = 1e-3;
  const int num_steps = 1000;

  true_state.gyro_error.lat = 1.0;
  est_state.gyro_error.lat = 1.0;

  for (int i = 0; i < num_steps; i++) {
    int success = euler_maruyama_step(
        NULL, NULL, NULL, NULL, NULL, NULL, NULL, &true_state, &est_state,
        &true_t, &est_t, time_step, sra3_test_drift, sra3_test_diffusion);
    REQUIRE_EQ(success, 1);
  }

  double final_value = est_state.gyro_error.lat;
  printf("Final sra3 test SDE value: %.12f\n", final_value);
  REQUIRE_GT(final_value, 0.0);
}

TEST(integrator, sra3_sde_differs_from_zero_diffusion) {
  init_genrand64(UINT64_C(12345));
  reset_ran_gaussian();
  sra3_test_diffusion_input = 1.0;

  state true_state_with_diffusion = {0};
  state est_state_with_diffusion = {0};
  double true_t_with_diffusion = 0.0;
  double est_t_with_diffusion = 0.0;
  const double time_step = 1e-3;
  const int num_steps = 1000;

  true_state_with_diffusion.gyro_error.lat = 1.0;
  est_state_with_diffusion.gyro_error.lat = 1.0;

  for (int i = 0; i < num_steps; i++) {
    int success = euler_maruyama_step(
        NULL, NULL, NULL, NULL, NULL, NULL, NULL, &true_state_with_diffusion,
        &est_state_with_diffusion, &true_t_with_diffusion,
        &est_t_with_diffusion, time_step, sra3_test_drift, sra3_test_diffusion);
    REQUIRE_EQ(success, 1);
  }

  state true_state_zero_diffusion = {0};
  state est_state_zero_diffusion = {0};
  double true_t_zero_diffusion = 0.0;
  double est_t_zero_diffusion = 0.0;
  sra3_test_diffusion_input = 1.0;

  true_state_zero_diffusion.gyro_error.lat = 1.0;
  est_state_zero_diffusion.gyro_error.lat = 1.0;

  for (int i = 0; i < num_steps; i++) {
    int success = euler_maruyama_step(
        NULL, NULL, NULL, NULL, NULL, NULL, NULL, &true_state_zero_diffusion,
        &est_state_zero_diffusion, &true_t_zero_diffusion,
        &est_t_zero_diffusion, time_step, sra3_test_drift,
        sra3_test_zero_diffusion);
    REQUIRE_EQ(success, 1);
  }

  REQUIRE_GT(fabs(est_state_with_diffusion.gyro_error.lat -
                  est_state_zero_diffusion.gyro_error.lat),
             1e-12);
}
