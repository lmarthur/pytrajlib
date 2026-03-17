# Body Frame

## `get_body_frame`

The body-centric coordinate system is defined as follows: $\hat e_1$ points
in the direction of relative velocity (roll axis), $\hat e_2$ points in the
direction of lift acceleration (pitch),  $\hat e_3$ is orthogonal to both:
$\hat e_3 = \hat e_1 \times \hat e_2$ (yaw). To ensure this process produces
a set of orthonormal vectors that span $\mathbb R^3$, the lift acceleration
is modified using the Gram-Schmidt process to be orthogonal to the relative
velocity.

The basis is successfully defined as long as the relative velocity is
non-zero.

We model the gyroscope error along the pitch and yaw axes as small rotations
of the true body frame. A rotation around the pitch axis by a small angle
$\theta$ is

$$
\mathbf{R_y} = \begin{bmatrix} 1 & 0 & \theta  \\\ 0 & 1 & 0 \\\ -\theta & 0
& 1\end{bmatrix}.
$$

A rotation around the yaw axis by a small angle $\phi$ is
$$
\mathbf{R_z} = \begin{bmatrix} 1 & -\phi & 0  \\\ \phi & 1 & 0 \\\ 0 & 0 &
1\end{bmatrix}.
$$
So the final cross coupling matrix for small angles is
$$
\mathbf{R_y R_z} = \begin{bmatrix} 1 & -\phi & \theta  \\\ \phi & 1 & 0
\\\ -\theta & 0 & 1\end{bmatrix}.
$$

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `true_state` | `state *` |  |
| `atm_cond` | `atm_cond *` | Atmospheric conditions used to compute wind-relative<br>velocity. May be NULL to use inertial velocity directly. |
| `xhat` | `cartvec *` | Output unit vector aligned with the relative velocity direction. |
| `yhat` | `cartvec *` | Output unit vector aligned with the orthogonalized lift<br>direction. |
| `zhat` | `cartvec *` | Output unit vector completing the right-handed body frame. |
| `apply_gyro_error` | `int` | 1 to apply gyro cross-coupling error to the<br>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;basis vectors. |

### Returns

| Type | Description |
| --- | --- |
| `int` | 1 if the basis is successfully defined, 0 otherwise. |
