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
