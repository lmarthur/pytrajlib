# Aero Forces

## `is_reentry`

Determine whether the vehicle is in reentry conditions.

Reentry is detected when altitude is below 100 km and velocity is directed
generally toward Earth center.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `state` | `state *` | Pointer to current state |
| `t` | `double` | Current simulation time in seconds |
| `run_params` | `runparams *` | Pointer to run configuration parameters |

### Returns

| Type | Description |
| --- | --- |
| `static inline int` | 1 if in reentry, else 0 |

## `get_aoa`

Compute angle of attack from body-frame relative-wind direction.

$$\alpha = \cos^{-1}(u_3),\quad u_3 = \hat{\mathbf u}_B.z$$

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `u_hat_B` | `cartvec` | Unit relative-wind vector in body coordinates. |

### Returns

| Type | Description |
| --- | --- |
| `static inline double` | Angle of attack in radians. |

## `get_max_modeled_aoa`

Largest angle of attack the vehicle's aerodynamic model is defined for based
on the aerodynamic table.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `vehicle` | `vehicle *` | Vehicle model holding the aerodynamic tables. |

### Returns

| Type | Description |
| --- | --- |
| `static inline double` | Maximum modeled angle of attack in radians. |

## `get_body_lift_direction`

Compute body-lift direction by projecting the roll axis onto the plane normal
to freestream direction u_hat_B.

Lift acts perpendicular to the freestream, in the plane spanned by the
freestream and the vehicle's roll axis, on the side the nose is pitched
toward. The roll axis is body $-\hat{e}\_{3,B}$.

$$\hat{\ell}\_B = \frac{(I-\hat{u}_B\hat{u}_B^T)(-\hat{e}\_{3,B})}
{\|(I-\hat{u}\_B\hat{u}\_B^T)(-\hat{e}\_{3,B})\|}$$

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `u_hat_B` | `cartvec` | Unit relative-wind vector in body coordinates. |

### Returns

| Type | Description |
| --- | --- |
| `static inline cartvec` | Unit body-lift direction; zero vector if near singular. |

## `get_post_shock_stagnation_pressure_ratio`

Compute post-shock stagnation-pressure ratio p0,2/p_inf from normal-shock
relations for a calorically perfect gas.

$$\frac{p_{0,2}}{p_\infty} =
\left(1+\frac{\gamma-1}{2}M_\infty^2\right)^{\frac{\gamma}{\gamma-1}}
\left[\frac{\gamma+1}{2\gamma
M_\infty^2-(\gamma-1)}\right]^{\frac{1}{\gamma-1}}
\left[\frac{(\gamma+1)M_\infty^2}{(\gamma-1)M_\infty^2+2}\right]^{\frac{\gamma}{\gamma-1}}$$

This implementation uses gamma = 1.4.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `mach` | `double` | Freestream Mach number M_inf. |

### Returns

| Type | Description |
| --- | --- |
| `static inline double` | p0,2 / p_inf ratio. |

## `get_cp_max`

Compute modified-Newtonian stagnation-point pressure coefficient C_p,max.

$$C_{p,\max}(M_\infty)=\frac{2}{\gamma
M_\infty^2}\left(\frac{p_{0,2}}{p_\infty}-1\right)$$

This implementation uses gamma = 1.4.
For this model configuration, Mach is fixed at M_inf = 12.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| (none) | - | - |

### Returns

| Type | Description |
| --- | --- |
| `static inline double` | Stagnation-point pressure coefficient C_p,max evaluated at Mach 12. |

## `get_undeflected_flap_normals`

Compute the four inward-pointing undeflected flap normals in body coordinates
for a conical reentry vehicle geometry.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `vehicle` | `vehicle *` | Vehicle model containing cone half-angle in rv.half_angle. |
| `n_B` | `cartvec` | Output array of 4 flap normals in body coordinates. |

### Returns

| Type | Description |
| --- | --- |
| `static inline void` |  |

## `get_deflected_flap_normals`

Compute the four deflected flap normals in body coordinates.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `vehicle` | `vehicle *` | Vehicle model used to construct undeflected flap normals. |
| `delta1` | `double` | Deflection command for flap pair {1, 3} in radians. |
| `delta2` | `double` | Deflection command for flap pair {2, 4} in radians. |
| `n_deflected_B` | `cartvec` | Output array of 4 deflected flap normals in body<br>coordinates. |

### Returns

| Type | Description |
| --- | --- |
| `static inline void` |  |

## `get_limited_incremental_flap_force`

Compute per-flap incremental force and apply flap-force magnitude limiting.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `deflected_force_B` | `cartvec` | Deflected flap force vector in body coordinates. |
| `undeflected_force_B` | `cartvec` | Undeflected flap force vector in body coordinates. |
| `run_params` | `runparams *` |  |
| `vehicle` | `vehicle *` |  |

### Returns

| Type | Description |
| --- | --- |
| `static inline cartvec` | Limited incremental flap force vector in body coordinates. |

## `get_incidence_factors`

Compute loaded-side incidence factors for all flaps.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `vehicle` | `vehicle *` | Vehicle model used for flap geometry. |
| `delta1` | `double` | Deflection command for flap pair {1, 3} in radians. |
| `delta2` | `double` | Deflection command for flap pair {2, 4} in radians. |
| `u_hat_B` | `cartvec` | Relative-wind unit direction in body frame. |
| `incidence_factors` | `double` | Output array of 4 loaded-side incidence factors. |

### Returns

| Type | Description |
| --- | --- |
| `static inline void` |  |

## `get_absolute_flap_force_magnitudes`

Compute absolute force magnitudes for all four flaps.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `current_state` | `state *` | Current vehicle state. |
| `atm_cond` | `atm_cond *` | Atmospheric conditions. |
| `vehicle` | `vehicle *` | Vehicle model containing flap area. |
| `delta1` | `double` | Deflection command for flap pair {1, 3} in radians. |
| `delta2` | `double` | Deflection command for flap pair {2, 4} in radians. |
| `q_EB` | `quaternion` | Quaternion rotating body-frame vectors into ECI. |
| `flap_force_magnitudes` | `double` | Output array of 4 flap force magnitudes. |

### Returns

| Type | Description |
| --- | --- |
| `static inline void` |  |

## `get_absolute_flap_forces_body`

Compute absolute flap force vectors for all four flaps in body coordinates.

Quantize the flaps according to the actuator resolution.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `current_state` | `state *` | Current vehicle state. |
| `atm_cond` | `atm_cond *` | Atmospheric conditions. |
| `vehicle` | `vehicle *` | Vehicle model containing flap geometry and area. |
| `run_params` | `runparams *` |  |
| `delta1` | `double` | Deflection command for flap pair {1, 3} in radians. |
| `delta2` | `double` | Deflection command for flap pair {2, 4} in radians. |
| `flap_forces_B` | `cartvec` | Output array of 4 flap force vectors in body<br>coordinates. |

### Returns

| Type | Description |
| --- | --- |
| `static inline void` |  |