# Aero Moments

## `get_body_moment_axis`

Compute body pitching-moment axis in body coordinates.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `u_hat_B` | `cartvec` | Unit relative-wind vector in body coordinates. |

### Returns

| Type | Description |
| --- | --- |
| `static inline cartvec` | Unit body pitching-moment axis, or zero vector if near singular. |

## `get_omega_perp_body`

Get transverse angular velocity in body coordinates.

Eq. 33 form:
$$\boldsymbol\omega_{\perp,B} = [\omega_{1,B},\omega_{2,B},0]^T.$$

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `current_state` | `state *` | Current vehicle state containing body angular velocity. |

### Returns

| Type | Description |
| --- | --- |
| `static inline cartvec` | Transverse body angular velocity vector. |

## `get_body_moment`

Compute the body aerodynamic moment from angle-of-attack and damping terms.

The body moment is the sum of the static moment contribution and the
pitching-damping contribution, both expressed in body coordinates.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `current_state` | `state *` | Current vehicle state. |
| `atm_cond` | `atm_cond *` | Atmospheric conditions. |
| `vehicle` | `vehicle *` | Vehicle model containing aerodynamic moment tables. |
| `run_params` | `runparams *` | Simulation run parameters. |

### Returns

| Type | Description |
| --- | --- |
| `static inline cartvec` | Body aerodynamic moment in body coordinates. |

## `get_flap_displacements_body`

Compute incremental flap centroid displacement vectors in body coordinates.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `vehicle` | `vehicle *` | Vehicle model containing RV geometry. |
| `flap_r_B` | `cartvec` | Output array of 4 flap displacement vectors in body<br>coordinates. |

### Returns

| Type | Description |
| --- | --- |
| `static inline void` |  |

## `sum_incremental_moments`

Sum the incremental flap moments about the body axes.

The incremental moment from each flap is the cross product of flap
displacement and clipped flap force.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `current_state` | `state *` | Current vehicle state. |
| `atm_cond` | `atm_cond *` | Atmospheric conditions. |
| `vehicle` | `vehicle *` | Vehicle model. |
| `run_params` | `runparams *` | Simulation run parameters. |

### Returns

| Type | Description |
| --- | --- |
| `static inline cartvec` | Total incremental flap moment in body coordinates. |

## `get_angular_acceleration`

Compute the angular acceleration induced by the aerodynamic moments.

Roll-axis angular acceleration is suppressed and the remaining moment is
divided by the pitch/yaw moment of inertia.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `t` | `double` | Current simulation time in seconds. |
| `true_state` | `state *` | True vehicle state. |
| `atm_cond` | `atm_cond *` | Atmospheric conditions. |
| `vehicle` | `vehicle *` | Vehicle model. |
| `run_params` | `runparams *` | Simulation run parameters. |

### Returns

| Type | Description |
| --- | --- |
| `static inline cartvec` | Angular acceleration in body coordinates. |