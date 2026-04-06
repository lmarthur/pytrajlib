# Thrust

## `get_current_stage`

Determine the active booster stage at simulation time t.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `t` | `double` | Current simulation time in seconds. |
| `vehicle` | `vehicle *` | Pointer to vehicle model/state. |

### Returns

| Type | Description |
| --- | --- |
| `int` | Zero-based stage index. |

## `remaining_delta_v`

Calculate remaining delta-v by summing the delta-v of each stage

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `state` | `state *` | Pointer to current vehicle state. |
| `vehicle` | `vehicle *` | Pointer to vehicle model/state. |
| `t` | `double` | Current simulation time in seconds. |

### Returns

| Type | Description |
| --- | --- |
| `double` | Remaining ideal delta-v in m/s. |

## `get_central_angle`

Get the central angle in radians between current position and aimpoint

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `position` | `cartvec` | Current position vector. |
| `aimpoint` | `cartvec` | Target aimpoint vector. |

### Returns

| Type | Description |
| --- | --- |
| `double` | Central angle between the vectors in radians. |

## `get_lambert_velocity`

Compute Lambert transfer speed magnitude for a given flight-path angle.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `r0` | `double` | Initial radius magnitude. |
| `rf` | `double` | Final radius magnitude. |
| `phi` | `double` | Central angle between start and end points in radians. |
| `gamma` | `double` | Flight-path angle in radians. |
| `grav_model` | `grav *` | Pointer to gravity model. |

### Returns

| Type | Description |
| --- | --- |
| `double` | Lambert transfer speed magnitude. |

## `time_to_fly`

Estimate time of flight for a Lambert transfer.

The time of flight for each flight path angle $\gamma$ and associated speed
$v_\text{Lambert}$ is calculated assuming an elliptical flight path:

<div class="math-scroll">
$$
t=\frac{|\vec x|}{v_\text{Lambert}\cos\gamma} \left[  \frac{\tan\gamma(1 -
\cos\phi) + (1-\lambda)\sin\phi}
{(2-\lambda)\left(\frac{1-\cos\phi}{\lambda\cos^2\gamma}+\frac{\cos(\gamma+\phi)}{\cos\gamma}\right)}
+ \frac{2\cos\gamma}{\lambda\left(\frac{2}{\lambda}-1\right)^{3/2}}
\arctan\left(  \frac{\sqrt{\frac{2}{\lambda}-1}}
{\cos\gamma\cot(\phi/2)-\sin\gamma}  \right)  \right]
$$
</div>

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `r0` | `double` | Initial radius magnitude. |
| `phi` | `double` | Central angle between start and end points in radians. |
| `gamma` | `double` | Flight-path angle in radians. |
| `lambert_velocity` | `double` | Lambert transfer speed magnitude. |
| `grav_model` | `grav *` | Pointer to gravity model. |

### Returns

| Type | Description |
| --- | --- |
| `double` | Estimated time of flight in seconds, or NAN when invalid. |

## `get_min_flight_angle`

Compute minimum feasible flight-path angle for the transfer geometry.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `r0` | `double` | Initial radius magnitude. |
| `rf` | `double` | Final radius magnitude. |
| `phi` | `double` | Central angle between start and end points in radians. |

### Returns

| Type | Description |
| --- | --- |
| `double` | Minimum feasible flight-path angle in radians. |

## `get_max_flight_angle`

Compute maximum feasible flight-path angle for the transfer geometry.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `r0` | `double` | Initial radius magnitude. |
| `rf` | `double` | Final radius magnitude. |
| `phi` | `double` | Central angle between start and end points in radians. |

### Returns

| Type | Description |
| --- | --- |
| `double` | Maximum feasible flight-path angle in radians. |

## `get_flight_angle`

The flight path angle $\gamma$ is determined numerically using the
secant-method to achieve the desired flight time (within a relative tolerance
of $10^{-8}$).

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `r0` | `double` | Initial radius magnitude. |
| `rf` | `double` | Final radius magnitude. |
| `phi` | `double` | Central angle between start and end points in radians. |
| `t_f_des` | `double` | Desired time of flight in seconds. |
| `grav_model` | `grav *` | Pointer to gravity model. |

### Returns

| Type | Description |
| --- | --- |
| `double` | Flight-path angle gamma in radians, or NAN if no convergence. |

## `get_lambert_velocity_vector`

The direction of the Lambert velocity vector is calculated as

$$
\hat v_\text{Lambert} = \hat x \cos\left(\frac{\pi}{2} - \gamma \right) -
\frac{\vec x \cdot \vec x_\text{aim}|\vec x| \sin\left(\frac{\pi}{2} - \gamma
\right)}{|\vec x|^2 |\vec x \times \vec x_\text{aim}|} + \frac{\vec
x_\text{aim} |\vec x| \sin\left(\frac{\pi}{2} - \gamma \right)}{|\vec x
\times \vec x_\text{aim}|}.
$$

>See Zarchan (2012) Listing 28.2

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `position` | `cartvec` | Current position vector. |
| `aimpoint` | `cartvec` | Desired impact aimpoint vector. |
| `tf_des` | `double` | Desired remaining flight time in seconds. |
| `grav_model` | `grav *` | Pointer to gravity model. |

### Returns

| Type | Description |
| --- | --- |
| `cartvec` | Lambert guidance velocity vector. |

## `thrust_offset`

General energy management steering offsets the thrust by an angle $\theta$
calculated from the remaining delta-v (Note for clarity: delta-v or $\Delta
v$ refers to the total velocity changes the vehicle is capable of, not the
difference between the desired velocity and the current velocity which is
denoted $\vec v_\text{gain}$). This allows us to achieve the same final
desired velocity without early thrust termination.

$$
\theta = \sqrt{6\left(1 - \frac{\vec v_\text{gain}}{\Delta v}\right)}
$$
>See Zarchan (2012) Chapter 13 and Listing 13.4

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `state` | `state *` | Pointer to current state. |
| `vehicle` | `vehicle *` | Pointer to vehicle model/state. |
| `v_to_gain` | `cartvec` | Velocity increment vector still required. |
| `t` | `double` | Current simulation time in seconds. |

### Returns

| Type | Description |
| --- | --- |
| `double` | Thrust steering offset angle in radians. |

## `get_a_thrust_magnitude`

The magnitude of thrust acceleration for booster stage $i$ is a function of
the specific impulse, $I_{sp,i}$, the gravitational acceleration at sea level
$g_0$, positive fuel burn rate $\dot m_i$, and mass $m(t)$:

$$
a_\text{thrust} = I_{sp,i} g_0 \dot m_i / m(t).
$$

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `state` | `state *` | Pointer to current vehicle state. |
| `vehicle` | `vehicle *` | Pointer to vehicle model/state. |
| `t` | `double` | Current simulation time in seconds. |

### Returns

| Type | Description |
| --- | --- |
| `double` | Thrust acceleration magnitude in m/s^2. |

## `get_thrust_vector`

Lambert Guidance finds the velocity needed at the end of boost phase for the
vehicle to reach its target at the desired time. Writing the final desired
velocity as $\vec v_\text{Lambert}$ and the current velocity as $\vec v$, the
velocity to be gained is

$$
\vec v_\text{gain} = \vec v_\text{Lambert} - \vec v.
$$

We use Rodrigues' rotation formula to rotate the velocity to-be-gained vector
in the plane of motion around the vector orthogonal to both the position and
the aimpoint (given by their cross-product):

$$
\hat a_\text{thrust} = \text{rotate}(\hat v_\text{gain}, \theta, \vec x
\times \vec x_\text{aim}).
$$

The thrust acceleration vector is the thrust direction vector scaled by the
magnitude of thrust acceleration:
$$
\vec a_\text{thrust} = a_\text{thrust} \hat a_\text{thrust}.
$$

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `state` | `state *` | Pointer to state used for guidance calculations. |
| `vehicle` | `vehicle *` | Pointer to vehicle model/state. |
| `run_params` | `runparams *` | Pointer to run configuration parameters. |
| `grav_model` | `grav *` | Pointer to gravity model for Lambert calculations. |
| `t` | `double` | Current simulation time in seconds. |

### Returns

| Type | Description |
| --- | --- |
| `cartvec` | Commanded thrust acceleration vector. |

## `get_thrust_acc`

After the first ten seconds of vertical flight, the thrust is directed along
a constant vector in an ECEF frame until the vehicle's altitude reaches
100km. Above 100km, low air density allows for efficient, low-drag,
maneuvering. Maneuvers are determined using Lambert Guidance with general
energy management steering as described in Zarchan (2012).

When the `perfect_boost` option in `run_params` is disabled (default), the
thrust angles are rotated by the gyroscope error and the Lambert Guidance
routine relies on estimated state measurements. When `perfect_boost` is
enabled, the gyroscope errors have no effect and the Lambert Guidance uses
the true vehicle state.

>Zarchan, P. (2012). Tactical and Strategic Missile Guidance, Sixth Edition.
American Institute of Aeronautics and Astronautics, Inc.
https://doi.org/10.2514/4.868948

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `true_state` | `state *` | Pointer to true vehicle state. |
| `est_state` | `state *` | Pointer to estimated vehicle state. |
| `vehicle` | `vehicle *` | Pointer to vehicle model/state. |
| `run_params` | `runparams *` | Pointer to run configuration parameters. |
| `true_grav` | `grav *` | Pointer to true gravity model. |
| `est_grav` | `grav *` | Pointer to estimated gravity model. |
| `t` | `double` | Current simulation time in seconds. |

### Returns

| Type | Description |
| --- | --- |
| `cartvec` | Thrust acceleration vector for the active guidance mode. |
