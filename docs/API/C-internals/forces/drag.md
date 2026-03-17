# Drag

## `get_drag_acceleration_generic`

Calculate the drag acceleration for any drag coefficient and characteristic
area.

Given the atmospheric density $\rho$ ($kg/m^3$), the magnitude of the
relative velocity with respect to the current wind $v_\text{rel}$ ($m/s$),
the cross-sectional area of the booster $A$ ($m^2$), the booster's drag
coefficient $C_D$, and the current mass $m$, the magnitude of the drag
acceleration ($m/s^2$) is
$$
|\vec a_\text{drag}| = \frac{1}{2} \rho v^2 A C_D / m.
$$
The drag vector opposes the direction of relative velocity.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `t` | `double` | Current simulation time in seconds |
| `current_state` | `state` | Current vehicle state |
| `atm_cond` | `atm_cond *` | Pointer to atmospheric conditions |
| `vehicle` | `vehicle *` | Pointer to vehicle model |
| `c_d` | `double` | Drag coefficient |
| `area` | `double` | Reference area in square meters |

### Returns

| Type | Description |
| --- | --- |
| `static inline cartvec` | Drag acceleration in inertial Cartesian coordinates |

## `get_drag_acc`

Compute drag acceleration for each regime (boost, reentry) and mode
(ballistic, maneuverable).

During boost phase, the change of drag coefficient as a function of angle of
attack is proportional to $\cos^2(\alpha)$ for an axially symmetric vehicle.
The boost-phase angle of attack remains small ($<1\degree$), so the boost
drag coefficient is modeled as independent of angle of attack.

During reentry, drag coefficient is modeled as a linear function of angle of
attack, where $C_{D,0}$ is the drag coefficient at zero angle of attack and
$C_{D,\alpha}$ is its derivative with respect to angle of attack:
$$
C_D = C_{D,0} + C_{D,\alpha} \alpha.
$$
For ballistic reentry vehicles, angle of attack is estimated assuming the
vehicle is oriented along the velocity vector:
$$
\alpha = \arctan(|\vec v_\text{wind}| / |\vec v|).
$$
For maneuverable reentry vehicles, angle of attack is estimated from current
lift, maximum achievable lift, and maximum achievable angle of attack:
$$
\alpha = |\vec a_\text{lift}| \alpha_\text{max} / a_\text{lift,max}.
$$

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `run_params` | `runparams *` | Pointer to run configuration parameters |
| `vehicle` | `vehicle *` | Pointer to vehicle model/state |
| `atm_cond` | `atm_cond *` | Pointer to atmospheric conditions |
| `state` | `state *` | Pointer to state updated with drag acceleration |
| `t` | `double` | Current simulation time in seconds |

### Returns

| Type | Description |
| --- | --- |
| `cartvec` | Drag acceleration in inertial Cartesian coordinates |
