# Thrust

## `remaining_delta_v`

Calculate remaining delta-v by summing the delta-v of each stage

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `state` | `state *` |  |
| `vehicle` | `vehicle *` |  |
| `t` | `double` |  |

### Returns

| Type | Description |
| --- | --- |
| `double` |  |

## `get_central_angle`

Get the central angle in radians between current position and aimpoint

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `position` | `cartvec` |  |
| `aimpoint` | `cartvec` |  |

### Returns

| Type | Description |
| --- | --- |
| `double` |  |

## `get_flight_angle`

Find the flight angle (gamma) that results in the estimated flight time being
within tol (1e-8) of the desired flight time. Uses the secant method to find
the root.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `r0` | `double` |  |
| `rf` | `double` |  |
| `phi` | `double` |  |
| `t_f_des` | `double` |  |
| `grav_model` | `grav *` |  |

### Returns

| Type | Description |
| --- | --- |
| `double` |  |

## `get_lambert_velocity_vector`

See Zarchan (2016) Listing 28.2

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `position` | `cartvec` |  |
| `aimpoint` | `cartvec` |  |
| `tf_des` | `double` |  |
| `grav_model` | `grav *` |  |

### Returns

| Type | Description |
| --- | --- |
| `cartvec` |  |

## `thrust_offset`

Generalized Energy Management

See Zarchan (2016) Ch 13

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `state` | `state *` |  |
| `vehicle` | `vehicle *` |  |
| `v_to_gain` | `cartvec` |  |
| `t` | `double` |  |

### Returns

| Type | Description |
| --- | --- |
| `double` |  |

## `get_thrust_acc`

Get thrust acceleration using Lambert Guidance outside the atmosphere

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `state` | `state *` |  |
| `vehicle` | `vehicle *` |  |
| `run_params` | `runparams *` |  |
| `grav_model` | `grav *` |  |
| `t` | `double` |  |

### Returns

| Type | Description |
| --- | --- |
| `cartvec` |  |
