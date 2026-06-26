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

The central angle between current position and aimpoint is
$$
\begin{align}
\phi = \arccos\left( \frac{\vec x \cdot \vec x_\text{aim}}
{|\vec x||\vec x_\text{aim}|} \right).
\end{align}
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

Given the standard gravitational parameter $GM$, the angle between current
position and aimpoint $\phi$, and the flight path angle $\gamma$, the final
desired speed is
$$
\begin{align}
v_\text{Lambert} = \sqrt{ \frac{ GM (1 - \cos\phi) }
{ |\vec x| \cos(\gamma) (|\vec x| \cos(\gamma) / |\vec x_\text{aim}|
- \cos(\phi + \gamma)) } }.
\end{align}
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
$v_\text{Lambert}$ is calculated assuming an elliptical flight path
$(\lambda = \frac{|\vec x| v_\text{Lambert}^2}{GM} < 2)$:
<div class="math-scroll">
$$
\begin{align}
t=\frac{|\vec x|}{v_\text{Lambert}\cos\gamma}  \left[
\frac{\tan\gamma(1 - \cos\phi) + (1-\lambda)\sin\phi}
{(2-\lambda)\left(\frac{1-\cos\phi}{\lambda\cos^2\gamma}+
\frac{\cos(\gamma+\phi)}{\cos\gamma}\right)}
+ \frac{2\cos\gamma}{\lambda\left(\frac{2}{\lambda}-1\right)^{3/2}}
\arctan\left(
\frac{\sqrt{\frac{2}{\lambda}-1}}
{\cos\gamma\cot(\phi/2)-\sin\gamma}
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

The flight path angle $\gamma$ is determined numerically using the
secant method to achieve the desired flight time (within a relative
tolerance of $10^{-8}$).

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

The direction of the Lambert velocity vector is
$$
\begin{align}
\hat v_\text{Lambert} = \frac{\vec x}{|\vec x|}
\cos\left(\frac{\pi}{2} - \gamma \right)
-  \frac{\vec x \cdot \vec x_\text{aim}|\vec x|
\sin\left(\frac{\pi}{2} - \gamma \right)}{|\vec x|^2
|\vec x \times \vec x_\text{aim}|}
+ \frac{\vec x_\text{aim} |\vec x|
\sin\left(\frac{\pi}{2} - \gamma \right)}{|\vec x \times \vec x_\text{aim}|}.
\end{align}
$$
so
$$
\begin{align}
\vec v_\text{Lambert} = v_\text{Lambert} \hat v_\text{Lambert}.
\end{align}
$$

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

Instead of directing thrust along the velocity-to-be-gained vector, general
energy management steering offsets the thrust by an angle $\theta$ computed
from the remaining delta-v. Here $\Delta v$ is the total thrust impulse per
unit mass the vehicle can still produce, not the difference between desired
and current velocity.
$$
\begin{align}
\theta = \sqrt{6\left(1 - \frac{|\vec v_\text{gain}|}{\Delta v}\right)}.
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

Lambert Guidance determines the velocity required at the end of boost for
the vehicle to reach the target at the desired time, ignoring drag. Denoting
final desired velocity as $\vec v_\text{Lambert}$ and current velocity as
$\vec v$, the velocity to be gained is
$$
\begin{align}
\vec v_\text{gain} = \vec v_\text{Lambert} - \vec v.
\end{align}
$$
Using Rodrigues' rotation formula in the plane of motion around
$\vec x \times \vec x_\text{aim}$:
$$
\begin{align}
\hat a_\text{thrust} = \text{rotate}(\hat v_\text{gain}, \theta,
\vec x \times \vec x_\text{aim}).
\end{align}
$$
The thrust acceleration vector is
$$
\begin{align}
\vec a_\text{thrust} = a_\text{thrust} \hat a_\text{thrust}.
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

After the first ten seconds of vertical flight, thrust is directed along a
constant vector in an ECI frame until altitude reaches 100 km. Above 100 km,
maneuvers are determined using Lambert Guidance with general energy
management steering.

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
