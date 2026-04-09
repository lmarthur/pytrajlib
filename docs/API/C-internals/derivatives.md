# Derivatives

## `drift`

Calculate deterministic drift component of the state update.

This function writes derivative outputs into `true_state_drift` and
`est_state_drift`. These output state structs should be initialized to
`{0}` before being passed in.

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
| `true_state` | `state *` | Pointer to true state. |
| `est_state` | `state *` | Pointer to estimated state. |
| `true_t` | `double` | Current true simulation time in seconds. |
| `est_t` | `double` | Current estimated simulation time in seconds. |
| `true_state_drift` | `state *` | Output deterministic drift for true state; initialize<br>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;to `{0}` before passing. |
| `est_state_drift` | `state *` | Output deterministic drift for estimated state;<br>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;initialize to `{0}` before passing. |

### Returns

| Type | Description |
| --- | --- |
| `int` | 1 on success, 0 if thrust guidance fails |

## `diffusion`

Calculate stochastic diffusion component of the state update.

This function writes diffusion outputs into `true_state_diffusion`.
The output state struct should be initialized to `{0}` before being passed
in.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `imu` | `imu *` | Pointer to IMU model. |
| `true_state_diffusion` | `state *` | Output stochastic diffusion for true state;<br>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;initialize to `{0}` before passing. |

### Returns

| Type | Description |
| --- | --- |
| `void` | None. |
