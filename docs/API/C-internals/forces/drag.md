# Drag

## `get_drag_acceleration_generic`

For all modes, the magnitude of the drag acceleration is a function of the
atmospheric density $\rho$ ($kg/m^3$), the magnitude of the relative
velocity with respect to the current wind $v_\text{rel}$ ($m/s$), the
cross-sectional area $A$ ($m^2$), drag coefficient $C_D$, and current mass
$m$ ($kg$):
$$
\begin{align}
a_\text{drag} = \frac{1}{2} \rho v_\text{rel}^2 A C_D / m.
\end{align}
$$
The drag acceleration vector is directed opposite the relative velocity.

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

The drag acceleration opposes the direction of relative velocity and depends
on the angle of attack, $\alpha$. During boost phase, the change in drag
coefficient as a function of angle of attack is proportional to
$\cos^2(\alpha)$ for an axisymmetric vehicle, and the boost-phase angle of
attack remains small, so the boost drag coefficient is modeled as
independent of angle of attack.

During reentry, the drag coefficient is modeled as a linear function of
angle of attack, where $C_{D,0}$ is the drag coefficient at zero angle of
attack and $C_{D,\alpha}$ is the derivative with respect to angle of attack:
$$
\begin{align}
C_D = C_{D,0} + C_{D,\alpha} \alpha.
\end{align}
$$
For ballistic reentry vehicles, the angle of attack is estimated assuming
the vehicle is oriented along the velocity vector:
$$
\begin{align}
\alpha = \arctan(v_\text{wind} / v).
\end{align}
$$
For maneuverable reentry vehicles, the angle of attack is a state variable.

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
