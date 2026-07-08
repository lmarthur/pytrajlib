# Propnav

## `prop_nav`

ECI acceleration commands are issued from an outer-loop proportional navigation
guidance law to steer the vehicle to the stationary aim point by maintaining a
consistent line of sight between the vehicle and the aim point. Given $\mathbf
r_\text{v,a} = \mathbf r_\text{aim} - \mathbf r_\text{est}$ is the displacement
between the vehicle and the aim point, and $\mathbf v_\text{est}$ is the
estimated vehicle velocity, the line-of-sight rotation vector is
$$
\begin{align}
  \boldsymbol \Omega = \frac{\mathbf r_\text{v,a} \times (-\mathbf
v_\text{est})}{\mathbf r_\text{v,a} \cdot \mathbf r_\text{v,a}}.
\end{align}
$$

The commanded acceleration in the ECI frame is orthogonal to the vehicle's
velocity and is issued to bring the line-of-sight rotation rate to zero. The
gravitational acceleration in the direction of commanded acceleration is treated
as the target's acceleration, as described by Zarchan (2012). Subtracting the
gravitational acceleration ensures the acceleration command represents only the
lift acceleration the vehicle needs to generate:
$$
\begin{align}
\mathbf a_{c,E} = -N \mathbf v_\text{est} \times \boldsymbol \Omega -
\frac{N}{2}\mathbf a_{\text{grav,E,est}}.
\end{align}
$$
The navigation gain is a linear function of the estimated altitude and
empirically tuned to take on the value $N_1$ at the reentry altitude,
($r_\text{reentry} - r_\text{Earth}$), and the value $N_0$ at impact:
$$
\begin{equation}
  N = N_0 + \frac{N_1 - N_0}{r_\text{reentry} - r_\text{Earth}} (|\mathbf
r_{v,\text{est}}| - r_\text{Earth}).
\end{equation}
$$

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `estimated_state` | `state *` | Pointer to the estimated vehicle state. |
| `run_params` | `runparams *` | Pointer to the run configuration parameters. |
| `est_grav` | `grav *` | Pointer to the guidance gravity model. |

### Returns

| Type | Description |
| --- | --- |
| `cartvec` | Commanded acceleration vector in ECI coordinates. |