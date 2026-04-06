# Integrator

## `euler_maruyama_step`

Advance the state by one Euler-Maruyama step. Note, it is not strictly EM
because the position is updated using the velocity and the acceleration.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `run_params` | `runparams *` |  |
| `imu` | `imu *` |  |
| `vehicle` | `vehicle *` |  |
| `true_grav` | `grav *` |  |
| `est_grav` | `grav *` |  |
| `true_atm_cond` | `atm_cond *` |  |
| `est_atm_cond` | `atm_cond *` |  |
| `true_state` | `state *` |  |
| `est_state` | `state *` |  |
| `true_t` | `double *` |  |
| `est_t` | `double *` |  |
| `time_step` | `double` | Integration time step in seconds |
| `drift_fn` | `drift_func` |  |
| `diffusion_fn` | `diffusion_func` |  |

### Returns

| Type | Description |
| --- | --- |
| `int` |  |

## `sra3_step`

Advance the state using SRA3 (Stochastic Runge-Kutta for Additive Noise).

References:

>Rößler, A. (2010). Runge–Kutta Methods for the Strong Approximation of
Solutions of Stochastic Differential Equations. SIAM Journal on Numerical
Analysis, 48(3), 922–952. https://doi.org/10.1137/09076636X

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `run_params` | `runparams *` |  |
| `imu` | `imu *` |  |
| `vehicle` | `vehicle *` |  |
| `true_grav` | `grav *` |  |
| `est_grav` | `grav *` |  |
| `true_atm_cond` | `atm_cond *` |  |
| `est_atm_cond` | `atm_cond *` |  |
| `true_state` | `state *` |  |
| `est_state` | `state *` |  |
| `true_t` | `double *` |  |
| `est_t` | `double *` |  |
| `time_step` | `double` |  |
| `drift_fn` | `drift_func` |  |
| `diffusion_fn` | `diffusion_func` |  |

### Returns

| Type | Description |
| --- | --- |
| `int` |  |
