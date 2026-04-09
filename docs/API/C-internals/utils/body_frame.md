# Body Frame

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

## `get_body_frame`

The body-centric coordinate system is defined as follows: $\hat x$ (roll)
points along relative velocity, $\hat z$ (yaw) points along commanded lift
acceleration (or global z if there is no lift), and
$\hat y = \hat z \times \hat x$. Because commanded lift may not be exactly
perpendicular to wind-relative velocity, the lift direction is
orthogonalized with Gram-Schmidt to produce an orthonormal basis.

Gyroscope error in pitch and yaw is modeled as small-angle rotations of the
true body frame. A pitch-axis rotation by $\theta$ is
<div>
$$
\begin{align}
\mathbf{R_y} = \begin{bmatrix} 1 & 0 & \theta  \\ 0 & 1 & 0 \\ -\theta & 0 &
1\end{bmatrix}.
\end{align}
$$
</div>
A yaw-axis rotation by $\phi$ is
<div>
$$
\begin{align}
\mathbf{R_z} = \begin{bmatrix} 1 & -\phi & 0  \\ \phi & 1 & 0 \\ 0 & 0 &
1\end{bmatrix}.
\end{align}
$$
</div>
Under small-angle approximation, rotations commute and the cross-coupling
matrix is
<div>
$$
\begin{align}
\mathbf{C} = \mathbf{R_y R_z} = \mathbf{R_z R_y} =
\begin{bmatrix} 1 & -\phi & \theta  \\ \phi & 1 & 0 \\ -\theta & 0 &
1\end{bmatrix}.
\end{align}
$$
</div>
With true basis
<div>
$$
\begin{align}
\mathbf{B} &= \begin{bmatrix} \hat x & \hat y & \hat z \end{bmatrix} \\
\mathbf{B}_\text{est} &= \mathbf{C} \mathbf{B}.
\end{align}
$$
</div>

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `true_state` | `state *` |  |
| `est_state` | `state *` |  |
| `run_params` | `runparams *` |  |
| `t` | `double` |  |
| `atm_cond` | `atm_cond *` | Atmospheric conditions used to compute wind-relative<br>velocity. May be NULL to use inertial velocity directly. |
| `xhat` | `cartvec *` | Output unit vector aligned with the relative velocity direction. |
| `yhat` | `cartvec *` | Output unit vector aligned with the orthogonalized lift<br>direction. |
| `zhat` | `cartvec *` | Output unit vector completing the right-handed body frame. |
| `apply_gyro_error` | `int` | 1 to apply gyro cross-coupling error to the<br>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;basis vectors. |
| `grav` | `grav *` |  |

### Returns

| Type | Description |
| --- | --- |
| `int` | 1 if the basis is successfully defined, 0 otherwise. |
