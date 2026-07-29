# Autopilot

## `NDI`

Get the desired flap deflection angle using nonlinear dynamic inversion (NDI)
with a PD controller that reduces the error between the current estimated
angle of attack and the desired angle of attack.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `a_cmd_B` | `cartvec` |  |
| `est_state` | `state *` |  |
| `aoa_est` | `cartvec` |  |
| `vehicle` | `vehicle *` |  |
| `q_inf` | `double` |  |
| `run_params` | `runparams *` |  |

### Returns

| Type | Description |
| --- | --- |
| `cartvec` |  |

## `get_flap_angular_acceleration`

Steer the vehicle by changing the flap angular velocity.

This function also logs to the guidance log file.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `t` | `double` |  |
| `est_state` | `state *` |  |
| `run_params` | `runparams *` |  |
| `vehicle` | `vehicle *` |  |
| `est_grav` | `grav *` |  |
| `est_atm` | `atm_cond *` |  |
| `a_imu` | `cartvec` |  |
| `ddot_deflection` | `double *` |  |

### Returns

| Type | Description |
| --- | --- |
| `void` | None. |