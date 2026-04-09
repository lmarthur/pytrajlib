# Integrator

## `euler_maruyama_step`

Advance the state by one Euler-Maruyama step. Note, it is not strictly EM
because the position is updated using the velocity and the acceleration.

This integrator is currently not used in the trajectory simulation path.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `run_params` | `runparams *` | Pointer to run configuration parameters. |
| `imu` | `imu *` | Pointer to IMU model. |
| `vehicle` | `vehicle *` | Pointer to vehicle model. |
| `true_grav` | `grav *` | Pointer to true gravity model. |
| `est_grav` | `grav *` | Pointer to estimated gravity model. |
| `true_atm_cond` | `atm_cond *` | Pointer to true atmospheric conditions. |
| `est_atm_cond` | `atm_cond *` | Pointer to estimated atmospheric conditions. |
| `true_state` | `state *` | Pointer to true state, updated in place. |
| `est_state` | `state *` | Pointer to estimated state, updated in place. |
| `true_t` | `double *` | Pointer to true simulation time, incremented by `time_step`. |
| `est_t` | `double *` | Pointer to estimated simulation time, incremented by<br>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`time_step`. |
| `time_step` | `double` | Integration time step in seconds. |
| `drift_fn` | `drift_func` | Drift callback used to compute deterministic derivatives. |
| `diffusion_fn` | `diffusion_func` | Diffusion callback used to compute stochastic<br>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;derivatives. |

### Returns

| Type | Description |
| --- | --- |
| `int` | 1 on success, 0 if the drift callback reports failure. |

## `sra3_step`

Advance the state using SRA3 (Stochastic Runge-Kutta for Additive Noise).

References:

>Rößler, A. (2010). Runge–Kutta Methods for the Strong Approximation of
Solutions of Stochastic Differential Equations. SIAM Journal on Numerical
Analysis, 48(3), 922–952. https://doi.org/10.1137/09076636X

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `run_params` | `runparams *` | Pointer to run configuration parameters. |
| `imu` | `imu *` | Pointer to IMU model. |
| `vehicle` | `vehicle *` | Pointer to vehicle model. |
| `true_grav` | `grav *` | Pointer to true gravity model. |
| `est_grav` | `grav *` | Pointer to estimated gravity model. |
| `true_atm_cond` | `atm_cond *` | Pointer to true atmospheric conditions. |
| `est_atm_cond` | `atm_cond *` | Pointer to estimated atmospheric conditions. |
| `true_state` | `state *` | Pointer to true state, updated in place. |
| `est_state` | `state *` | Pointer to estimated state, updated in place. |
| `true_t` | `double *` | Pointer to true simulation time, incremented by `time_step`. |
| `est_t` | `double *` | Pointer to estimated simulation time, incremented by<br>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`time_step`. |
| `time_step` | `double` | Integration time step in seconds. |
| `drift_fn` | `drift_func` | Drift callback used for deterministic stage evaluations. |
| `diffusion_fn` | `diffusion_func` | Diffusion callback used for additive-noise stage<br>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;evaluations. |

### Returns

| Type | Description |
| --- | --- |
| `int` | 1 on success, 0 if any drift stage reports failure. |
