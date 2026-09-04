# Autopilot

## `clamp_aoa_channels`

Bound the two flap-channel angle-of-attack components to the range the
aerodynamic model covers.

Only the x and y components drive the flap pairs, so the pair is rescaled
together. Clamping each channel on its own would rotate the commanded
direction rather than shorten it.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `aoa` | `cartvec` | Angle-of-attack components in radians, indexed by flap pair. |
| `max_aoa` | `double` | Maximum modeled angle of attack in radians. |

### Returns

| Type | Description |
| --- | --- |
| `static inline cartvec` | The input, rescaled if its transverse magnitude exceeded max_aoa. |

## `get_max_trimmable_aoa`

Largest angle of attack the flaps can hold in trim.

Commanding more than this asks the inversion for a deflection the actuator
cannot reach, which leaves the tracking error dominated by the unreachable
setpoint rather than by the vehicle's actual state, so the feedback gains
have nothing left to regulate.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `vehicle` | `vehicle *` | Pointer to vehicle model. |
| `run_params` | `runparams *` | Pointer to run configuration parameters. |

### Returns

| Type | Description |
| --- | --- |
| `static inline double` | Maximum trimmable angle of attack in radians. |

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