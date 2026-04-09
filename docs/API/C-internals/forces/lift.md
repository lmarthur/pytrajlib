# Lift

We model lift acceleration during the reentry period below 100km altitude.

There are two components of the state that model the lift acceleration: the
available lift and the true lift. The available lift is an intermediate step to
calculate the true lift, which is used to update the vehicle's velocity.

## `get_a_lift_mag`

The lift acceleration magnitude is modeled from the angle of attack at each
time step:
$$
\begin{align}
a_L = C_{L\alpha} \alpha \bar q A.
\end{align}
$$

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `state` | `state *` | Pointer to state providing velocity and angle of attack. |
| `vehicle` | `vehicle *` | Pointer to vehicle model constants. |
| `atm_cond` | `atm_cond *` | Pointer to atmospheric conditions. |

### Returns

| Type | Description |
| --- | --- |
| `double` | Lift acceleration magnitude in m/s^2. |

## `get_lift_acc`

The lift acceleration is directed perpendicular to the vehicle's relative
velocity toward the commanded lift direction, and is constructed from the
body-frame lift axis.
$$
\begin{align}
\vec a_L = a_L\frac{\vec a_c - \vec a_{g\perp}}{|\vec a_c - \vec a_{g\perp}|}
\end{align}
$$

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `true_state` | `state *` | Pointer to true state used for lift magnitude and frame. |
| `est_state` | `state *` | Pointer to estimated state used for reentry checks. |
| `run_params` | `runparams *` | Pointer to run configuration parameters. |
| `vehicle` | `vehicle *` | Pointer to vehicle model constants. |
| `atm_cond` | `atm_cond *` | Pointer to atmospheric conditions. |
| `t` | `double` | Current simulation time in seconds. |
| `grav` | `grav *` | Pointer to gravity model used by body-frame construction. |

### Returns

| Type | Description |
| --- | --- |
| `cartvec` | Lift acceleration vector in inertial Cartesian coordinates. |

## `get_deflection_angular_speed`

For a given acceleration command $\vec a_c$, current lift acceleration
$\vec a_L$, and gravity component perpendicular to velocity
$\vec a_{g\perp}$, the commanded change in acceleration is
$$
\begin{align}
\Delta \vec a_c = \vec a_c - \vec a_L - \vec a_{g\perp}.
\end{align}
$$
For acceleration magnitude to be gained $\Delta a_c$ and proportional gain
$K_p$, the commanded flap deflection angular speed is clipped by
$\dot\delta_\text{max}$:
$$
\begin{align}
\dot \delta = \text{clip}(\Delta a_c K_p, -\dot\delta_\text{max},
\dot\delta_\text{max}).
\end{align}
$$

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `est_state` | `state *` | Pointer to estimated state. |
| `vehicle` | `vehicle *` | Pointer to vehicle model constants. |
| `atm_cond` | `atm_cond *` | Pointer to atmospheric conditions. |
| `run_params` | `runparams *` | Pointer to run configuration parameters. |
| `t` | `double` | Current simulation time in seconds. |
| `grav` | `grav *` | Pointer to gravity model used by guidance. |

### Returns

| Type | Description |
| --- | --- |
| `double` | Commanded flap deflection angular speed in rad/s. |

## `get_aoa_angular_acceleration`

The rotational equation of motion for angle of attack uses a Newtonian
pitching-moment approximation with effective flap angle
$\delta_\text{eff} = \delta + \alpha$.
$$
\begin{align}
I \ddot \alpha \approx (C_{M_\delta} \delta_\text{eff} + C_{M_\alpha} \alpha
+ C_{M_q} \frac{c}{2V} \dot\alpha)\bar q A c.
\end{align}
$$
With
$$
\begin{align}
p=\frac{C_{Mq} \bar qAc^2}{2IV},\; k=\frac{C_{M\alpha}\bar q A c}{I},\;
n = \frac{C_{M\delta} \bar q A c}{I},
\end{align}
$$
the forced second-order model is
$$
\begin{align}
\ddot \alpha - p\dot\alpha - k \alpha =
\text{clip}(n\delta_\text{eff}, \frac{-F_\text{flap,max}r_\text{flap}}{I},
\frac{F_\text{flap,max}r_\text{flap}}{I}).
\end{align}
$$

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `true_state` | `state *` | Pointer to true state containing angle states. |
| `run_params` | `runparams *` | Pointer to run configuration parameters. |
| `vehicle` | `vehicle *` | Pointer to vehicle model constants. |
| `atm_cond` | `atm_cond *` | Pointer to atmospheric conditions. |
| `t` | `double` | Current simulation time in seconds. |

### Returns

| Type | Description |
| --- | --- |
| `double` | Angular acceleration of angle of attack in rad/s^2. |
