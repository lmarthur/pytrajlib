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

The central angle between the current position and the aim point is
$$
\begin{equation}
\phi = \arccos\left( \frac{\mathbf r \cdot \mathbf r\_\text{aim}}{|\mathbf
r||\mathbf r\_\text{aim}|} \right).
\end{equation}
$$

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

Given the standard gravitational parameter $GM$, the angle between the position
and aim point $\phi$, and the flight path angle $\gamma$, the final desired
speed is
$$
\begin{equation}
v_\text{Lambert} = \sqrt{ \frac{ GM (1 - \cos\phi) }{ |\mathbf r| \cos(\gamma)
(|\mathbf r| \cos(\gamma) / |\mathbf r_\text{aim}| - \cos(\phi + \gamma) } }.
\end{equation}
$$

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

The time of flight for each flight path angle $\gamma$ and associated speed
$v_\text{Lambert}$ is calculated assuming an elliptical flight path ($\lambda =
\frac{|\mathbf r| v_\text{Lambert}^2}{GM} < 2$):
<div class="math-scroll">

$$
\begin{align}
t=\frac{|\mathbf r|}{v_\text{Lambert}\cos\gamma}
    \left[
\frac{\tan\gamma(1 - \cos\phi) + (1-\lambda)\sin\phi}
{(2-\lambda)\left(\frac{1-\cos\phi}{\lambda\cos^2\gamma}+\frac{\cos(\gamma+\phi)}{\cos\gamma}\right)}
+ \frac{2\cos\gamma}{\lambda\left(\frac{2}{\lambda}-1\right)^{3/2}}
\arctan\left(
\frac{\sqrt{\frac{2}{\lambda}-1}}
{\cos\gamma\cot\phi/2)-\sin\gamma}
\right)
\right]
\end{align}
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
| `r0` | `double` | Vehicle position magnitude wrt center of Earth. |
| `rf` | `double` | Aimpoint position magnitude wrt center of Earth. |
| `phi` | `double` | Central angle between vehicle and aimpoint in radians. |

### Returns

| Type | Description |
| --- | --- |
| `double` | Maximum feasible flight-path angle in radians. |

## `get_flight_angle`

The flight path angle $\gamma$ is determined numerically using the secant method
to achieve the desired flight time (within a relative tolerance of $10^{-8}$).

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `r0` | `double` | Vehicle position magnitude wrt center of Earth. |
| `rf` | `double` | Aimpoint position magnitude wrt center of Earth. |
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
\begin{align}
\hat{\mathbf v}\_\text{Lambert} = \frac{\mathbf r}{|\mathbf r|}
\cos\left(\frac{\pi}{2} - \gamma \right)
-  \mathbf r \frac{\mathbf r \cdot \mathbf r\_\text{aim}|\mathbf r|
\sin\left(\frac{\pi}{2} - \gamma \right)}{|\mathbf r|^2
|\mathbf r \times \mathbf r\_\text{aim}|}
+ \frac{\mathbf r\_\text{aim} |\mathbf r|
\sin\left(\frac{\pi}{2} - \gamma \right)}{|\mathbf r \times \mathbf
r_\text{aim}|}.
\end{align}
$$

So the desired final velocity vector is
$$
\begin{align}
\mathbf v_\text{Lambert} = v_\text{Lambert} \hat{\mathbf v}_\text{Lambert}.
\end{align}
$$

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `position` | `cartvec` | Current position vector. |
| `aimpoint` | `cartvec` | Desired impact aimpoint vector. |
| `tf_des` | `double` | Desired remaining flight time in seconds. |
| `grav_model` | `grav *` | Pointer to gravity model. |
| `run_params` | `runparams *` | Pointer to run parameters. |

### Returns

| Type | Description |
| --- | --- |
| `cartvec` | Lambert guidance velocity vector. |

## `thrust_offset`

Instead of directing the thrust along the velocity-to-be-gained vector, general
energy management steering offsets the thrust by an angle $\theta$ calculated
from the remaining delta-v. Energy management steering allows the
vehicle to achieve the same final desired velocity without early thrust
termination. The offset angle is calculated as
$$
\begin{align}
\theta = \sqrt{6\left(1 - \frac{|\mathbf v_\text{gain}|}{\Delta v}\right)}.
\end{align}
$$

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
\begin{align}
a_\text{thrust} = I_{sp,i} g_0 \dot m_i / m(t).
\end{align}
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

Lambert Guidance determines the velocity required at the end of the boost phase
for the vehicle to reach its target on a ballistic trajectory at the desired
time, ignoring drag. Denoting the final desired velocity as $\mathbf
v_\text{Lambert}$ and the current velocity as $\mathbf v$, the velocity to be
gained is
$$
\begin{align}
\mathbf v_\text{gain} = \mathbf v_\text{Lambert} - \mathbf v.
\end{align}
$$

Specifically, we use Rodrigues' rotation formula to rotate the
velocity-to-be-gained vector in the plane of motion around the vector orthogonal
to both the position and the aimpoint (given by their cross-product):

$$
\begin{align}
\hat{\mathbf a}\_\text{thrust} = \text{rotate}(\hat{\mathbf v}\_\text{gain},
\theta, \mathbf r \times \mathbf r\_\text{aim}).
\end{align}
$$

The thrust acceleration vector is the thrust direction vector scaled by the
magnitude of thrust acceleration:
$$
\begin{align}
\mathbf a_\text{thrust} = a\_\text{thrust} \hat{\mathbf a}\_\text{thrust}.
\end{align}
$$

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `state` | `state *` | Pointer to state used for guidance calculations. |
| `vehicle` | `vehicle *` | Pointer to vehicle model/state. |
| `run_params` | `runparams *` | Pointer to run configuration parameters. |
| `grav_model` | `grav *` | Pointer to gravity model for Lambert calculations. |
| `t` | `double` | Current simulation time in seconds. |
| `a_thrust_mag` | `double` |  |

### Returns

| Type | Description |
| --- | --- |
| `cartvec` | Commanded thrust acceleration vector. |

## `get_thrust_acc`

After the first ten seconds of vertical flight, the thrust is directed along a
constant vector in an ECI frame until the vehicle's altitude reaches 100km.
Above 100km, low air density allows for efficient, low-drag maneuvering.
Maneuvers are determined using Lambert Guidance with general energy management
steering as described by Zarchan (2012).

When `perfect_boost` in `run_params` is disabled (default), thrust angles
are rotated by gyroscope error and Lambert Guidance relies on estimated
state measurements. When `perfect_boost` is enabled, gyroscope errors have
no effect and Lambert Guidance uses the true vehicle state.

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