# Propnav

## `prop_nav`

Lift acceleration commands are issued from a proportional navigation
guidance law to steer the vehicle to a stationary aimpoint by maintaining a
line of sight between the vehicle and aimpoint. Given displacement
$\vec r$ and estimated velocity $\vec v$, the line-of-sight rotation vector
is
$$
\begin{align}
\vec \Omega = \frac{\vec r \times \vec v}{\vec r \cdot \vec r}.
\end{align}
$$
With navigation gain $N$, the commanded acceleration is
$$
\begin{align}
\vec a_c = -N \vec v \times \vec \Omega.
\end{align}
$$

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `estimated_state` | `state *` | Pointer to the estimated vehicle state. |
| `run_params` | `runparams *` | Pointer to the run configuration parameters. |
| `grav` | `grav *` | Pointer to the gravity model used for correction. |

### Returns

| Type | Description |
| --- | --- |
| `cartvec` | Commanded acceleration vector in ECI coordinates. |
