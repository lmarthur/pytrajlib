# Lift

## `rv_time_constant`

Calculate the time constant of the reentry vehicle based on the current
state.

The time constant $\tau$ is calculated from $I$, the moment of inertia around
the vehicle's y-axis, $C_{m_{\alpha}}$, the pitching moment coefficient
derivative (per radian), $A$, the reentry vehicle reference area, $\rho$, the
atmospheric density, and $v$, the speed of the vehicle:
$$
\tau = \sqrt{-\frac{2I}{C_{m_{\alpha}} A r_e \rho v^2}}
$$

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `current_state` | `state *` | pointer to current vehicle state |
| `atm_cond` | `atm_cond *` | pointer to atmospheric conditions |
| `vehicle` | `vehicle *` | pointer to vehicle model |

### Returns

| Type | Description |
| --- | --- |
| `double` | time constant in seconds |

## `prop_nav`

Get commanded acceleration $\vec a_n$.
The proportional navigation guidance law that flies the vehicle towards the
 target
is given by
$$
\vec a_n = -N \vec v_r \times \Omega,
$$
where $N$ is the navigation gain,
$\vec v_r$ is the relative velocity between the vehicle and the aimpoint,
 known
as the closing velocity, and $\Omega$ is the line-of-sight rotation vector:
$$
\Omega = \frac{\vec r \times \vec v_r}{\vec r \cdot \vec{r}}
$$
with $\vec r$ as the displacement between the vehicle and the aimpoint.

The target is assumed to be stationary, so $\vec v_r$ is the estimated
 vehicle velocity.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `estimated_state` | `state *` | the vehicle's internal estimated state |
| `run_params` | `runparams *` | the run parameters struct |

### Returns

| Type | Description |
| --- | --- |
| `cartvec` | commanded acceleration in the inertial-frame Cartesian basis (m/s^2) |

## `get_acc_resolution`

Compute lift-acceleration quantization from actuator angular resolution.

Based on ISO 3408-3 grade 5, we assume the actuator has a ±10 degree range
with a 0.01 degree resolution.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `run_params` | `runparams *` | run parameters struct |
| `vehicle` | `vehicle *` | vehicle struct |

### Returns

| Type | Description |
| --- | --- |
| `double` | Lift acceleration resolution in m/s^2 |

## `get_jerk_max`

Calculate the maximum jerk

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `run_params` | `runparams *` | run parameters struct |
| `vehicle` | `vehicle *` | vehicle struct |

### Returns

| Type | Description |
| --- | --- |
| `double` | Maximum jerk in m/s^3 |

## `project_and_clip`

Project a vector into the lift plane, clip, then reconstruct in Cartesian.

Given orthonormal lift-plane basis vectors $(\mathbf e_2,\mathbf e_3)$,
the vector components are clipped independently to
$[-\text{max\_val},\text{max\_val}]$ and mapped back.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `yhat` | `cartvec` | Lift-plane basis vector |
| `zhat` | `cartvec` | Lift-plane basis vector |
| `arr` | `cartvec` | Vector to project and clip |
| `max_val` | `double` | Absolute clip limit in projected coordinates |

### Returns

| Type | Description |
| --- | --- |
| `cartvec` | Projected and clipped vector in Cartesian coordinates |

## `is_reentry`

Determine whether the vehicle is in reentry conditions.

Reentry is detected when altitude is below 100 km and velocity is directed
generally toward Earth center.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `state` | `state *` | Pointer to current state |
| `t` | `double` | Current simulation time in seconds |

### Returns

| Type | Description |
| --- | --- |
| `int` | 1 if in reentry, else 0 |

## `get_a_lift_avail_jerk`

Get the time derivative of the available lift acceleration.

The available lift acceleration encodes the position of the control surfaces.
The control surfaces are assumed to instantaneously accelerate to
a fixed maximum angular velocity. The maximum angular velocity of the control
surfaces is equivalent to a maximum available jerk.

To avoid oscillations, as the available acceleration approaches the commanded
acceleration, the jerk reduces from the maximum jerk to a jerk proportional
to the difference. When the difference is less than the actuator resolution,
the derivative is zero.

Only valid during reentry phase.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `est_state` | `state *` | pointer to the estimated state |
| `run_params` | `runparams *` | pointer to the run parameters struct |
| `vehicle` | `vehicle *` | pointer to the vehicle struct |
| `est_atm_cond` | `atm_cond *` | pointer to the estimated atmospheric conditions |
| `est_t` | `double` | current estimated flight time (seconds) |

### Returns

| Type | Description |
| --- | --- |
| `cartvec` | Time derivative of available lift acceleration (m/s^3) |

## `get_a_lift_jerk`

Compute lift jerk from first-order lag toward available lift.

The lift acceleration approaches the available lift acceleration
exponentially based on the time constant:
$$
a(t) = a_\text{avail} (1 - e^{-t/\tau})
$$

The lift jerk is zero outside reentry or when a valid lift basis is
unavailable.

The proportional navigation commands may produce a commanded lift
acceleration with a component in the direction of the velocity, but
this function will only attempt to produce lift acceleration in the plane
perpendicular to the relative velocity (plane of yhat, zhat).

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `true_state` | `state *` |  |
| `run_params` | `runparams *` | Pointer to run configuration parameters |
| `vehicle` | `vehicle *` | Pointer to vehicle model |
| `atm_cond` | `atm_cond *` | Pointer to atmospheric conditions |
| `t` | `double` | Current simulation time in seconds |

### Returns

| Type | Description |
| --- | --- |
| `cartvec` | Time derivative of lift acceleration (jerk) in m/s^3 |
