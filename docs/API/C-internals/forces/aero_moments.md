# Aero Moments

## `get_body_moment_axis`

Compute body pitching-moment axis in body coordinates.

For nontrivial AoA (s = sqrt(u1^2 + u2^2) > 0), Eq. 29 form is used:
$$\hat{\mathbf m}_B = \frac{1}{s}[u_2, -u_1, 0]^T.$$

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `u_hat_B` | `cartvec` | Unit relative-wind vector in body coordinates. |

### Returns

| Type | Description |
| --- | --- |
| `static inline cartvec` | Unit body pitching-moment axis, or zero vector if near singular. |

## `get_omega_perp_body`

Get transverse angular velocity in body coordinates.

Eq. 33 form:
$$\boldsymbol\omega_{\perp,B} = [\omega_{1,B},\omega_{2,B},0]^T.$$

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `current_state` | `state *` | Current vehicle state containing body angular velocity. |

### Returns

| Type | Description |
| --- | --- |
| `static inline cartvec` | Transverse body angular velocity vector. |

## `get_flap_displacements_body`

Compute incremental flap centroid displacement vectors in body coordinates.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `vehicle` | `vehicle *` | Vehicle model containing RV geometry. |
| `flap_r_B` | `cartvec` | Output array of 4 flap displacement vectors in body<br>coordinates. |

### Returns

| Type | Description |
| --- | --- |
| `static inline void` |  |
