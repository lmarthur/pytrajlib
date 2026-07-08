# Drag

## `get_drag_acceleration_generic`

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `t` | `double` | Current simulation time in seconds |
| `current_state` | `state *` | Current vehicle state |
| `atm_cond` | `atm_cond *` | Pointer to atmospheric conditions |
| `vehicle` | `vehicle *` | Pointer to vehicle model |
| `c_d` | `double` | Drag coefficient |
| `area` | `double` | Reference area in square meters |

### Returns

| Type | Description |
| --- | --- |
| `static inline cartvec` | Drag acceleration in inertial Cartesian coordinates |

## `boost_drag`

During the boost phase, the change in the drag coefficient as a function of the
angle of attack is proportional to $\cos^2(\alpha)$ for an axisymmetric vehicle,
as described by Jorgensen and Center (1973). Using a small-angle approximation,
the boost-phase drag coefficient is modeled as independent of the angle of
attack. The boost phase drag acceleration is oriented along the direction of
relative wind $\hat{\mathbf{u}}\_E$
$$
\begin{equation}
    \mathbf{a}\_\text{drag} = q_\infty C\_D S/m \hat{\mathbf{u}}\_E
\end{equation}
$$
where $C\_D, S$ refer to the drag coefficient and cross-sectional base area,
respectively, of the missile with the booster.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `t` | `double` |  |
| `current_state` | `state *` |  |
| `atm_cond` | `atm_cond *` |  |
| `vehicle` | `vehicle *` |  |

### Returns

| Type | Description |
| --- | --- |
| `static inline cartvec` |  |

## `ballistic_reentry_drag`

Ballistic reentry vehicles have the option to use the same physics described
in the aero forces functions or a simplified ballistic drag where the
vehicle's roll axis is aligned with the current velocity and the vehicle is
assumed to be at a trim angle of attack:
$$
\begin{equation}
    \alpha = \arctan(v_\text{wind} / v).
\end{equation}
$$
The drag coefficient is a function of the angle of attack, so the total drag
acceleration is
$$
\begin{equation}
    \mathbf{a}\_\text{drag} = q\_\infty (C\_{D_0} + C_{D\_\alpha} \alpha) S/m
\hat{\mathbf{u}}\_E.
\end{equation}
$$

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `t` | `double` |  |
| `current_state` | `state *` |  |
| `atm_cond` | `atm_cond *` |  |
| `vehicle` | `vehicle *` |  |

### Returns

| Type | Description |
| --- | --- |
| `static inline cartvec` |  |