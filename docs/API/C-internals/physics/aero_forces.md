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

## `get_body_lift_direction`

Compute body-lift direction by projecting body axis e3 onto the plane normal
to freestream direction u_hat_B.

$$\hat{\ell}_B = \frac{(I-\hat{u}_B\hat{u}_B^T)\hat{e}_{3,B}}
{\|(I-\hat{u}_B\hat{u}_B^T)\hat{e}_{3,B}\|}$$

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
for a conical reentry vehicle geometry (Equation 55).

For a cone with half-angle theta_c, the normals are:
$$\hat{\mathbf{n}}_{10,B} = -\cos\theta_c \hat{\mathbf{e}}_{1,B} +
\sin\theta_c \hat{\mathbf{e}}_{3,B}$$
$$\hat{\mathbf{n}}_{20,B} = -\cos\theta_c \hat{\mathbf{e}}_{2,B} +
\sin\theta_c \hat{\mathbf{e}}_{3,B}$$
$$\hat{\mathbf{n}}_{30,B} = +\cos\theta_c \hat{\mathbf{e}}_{1,B} +
\sin\theta_c \hat{\mathbf{e}}_{3,B}$$
$$\hat{\mathbf{n}}_{40,B} = +\cos\theta_c \hat{\mathbf{e}}_{2,B} +
\sin\theta_c \hat{\mathbf{e}}_{3,B}$$

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

The deflected normal of flap i is
$$\hat{\mathbf n}_{i,B}(\delta_{f,i}) = R(\hat{\mathbf
h}_{i,B},\delta_{f,i})\hat{\mathbf n}_{i0,B}$$ with hinge axes and pair
mapping
$$\hat{\mathbf h}_{1,B}=\hat{\mathbf h}_{3,B}=\hat{\mathbf
e}_{2,B},\;\hat{\mathbf h}_{2,B}=\hat{\mathbf h}_{4,B}=-\hat{\mathbf
e}_{1,B}$$
$$\delta_{f,1}=\delta_{f,3}=\delta_1,\;\delta_{f,2}=\delta_{f,4}=\delta_2.$$

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

Compute loaded-side incidence factors for all flaps (Equation 61).

$$\lambda_i(\delta_{f,i}) = \max\left(0,\hat{\mathbf u}_B \cdot \hat{\mathbf
n}_{i,B}(\delta_{f,i})\right),\; i\in\{1,2,3,4\}$$

Flap indexing in output array is zero-based:
0 -> flap 1, 1 -> flap 2, 2 -> flap 3, 3 -> flap 4.

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

Each force is oriented along its deflected inward-pointing flap normal:
$$\mathbf F_{f,i,B} = F_i\hat{\mathbf n}_{i,B}(\delta_{f,i})$$

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
