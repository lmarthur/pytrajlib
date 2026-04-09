# Utils

## `get_altitude`

Calculates the altitude of a point above Earth's surface.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `position` | `cartvec` | Cartesian position vector. |

### Returns

| Type | Description |
| --- | --- |
| `double` | Altitude above Earth's mean radius in meters. |

## `cartcoords_to_sphercoords`

Converts Cartesian coordinates to spherical coordinates.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `cart_coords` | `double *` | Pointer to Cartesian coordinates `[x, y, z]`. |
| `spher_coords` | `double *` | Output spherical coordinates `[r, long, lat]`. |

### Returns

| Type | Description |
| --- | --- |
| `void` | None. |

## `sphercoords_to_cartcoords`

Converts spherical coordinates to Cartesian coordinates.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `spher_coords` | `double *` | Pointer to spherical coordinates `[r, long, lat]`. |
| `cart_coords` | `double *` | Output Cartesian coordinates `[x, y, z]`. |

### Returns

| Type | Description |
| --- | --- |
| `void` | None. |

## `sphervec_to_cartvec`

Converts a spherical vector to Cartesian components at given spherical
coordinates.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `sphervec` | `double *` | Pointer to spherical vector components. |
| `cartvec` | `double *` | Output Cartesian vector components. |
| `spher_coords` | `double *` | Pointer to spherical coordinates `[r, long, lat]`. |

### Returns

| Type | Description |
| --- | --- |
| `void` | None. |

## `print_config`

Prints run parameters to the console.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `run_params` | `runparams *` | Pointer to run parameters struct. |

### Returns

| Type | Description |
| --- | --- |
| `void` | None. |

## `linterp`

Performs linear interpolation on tabulated data.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `x` | `double` | Query value. |
| `xs` | `double` | Monotonic x-value array. |
| `ys` | `double` | Corresponding y-value array. |
| `n` | `int` | Number of data points. |

### Returns

| Type | Description |
| --- | --- |
| `double` | Interpolated y-value. |

## `min`

Returns the minimum of two values.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `a` | `double` | First value. |
| `b` | `double` | Second value. |

### Returns

| Type | Description |
| --- | --- |
| `double` | Smaller of `a` and `b`. |

## `sign`

Returns the sign of a value.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `x` | `double` | Input value. |

### Returns

| Type | Description |
| --- | --- |
| `double` | `1` if positive, `-1` if negative, otherwise `0`. |

## `get_max_flap_force`

The maximum achievable flap force is limited by actuator force $F_a$ and
gearing ratio $G$:
$$
\begin{align}
F_\text{flap,max} = G F_a.
\end{align}
$$
The actuator rate limit used by lift control is modeled as
$$
\begin{align}
\dot\delta_\text{max} = \frac{\delta_\text{max}}{t_\text{deflect}}.
\end{align}
$$

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `run_params` | `runparams *` |  |
| `vehicle` | `vehicle *` |  |

### Returns

| Type | Description |
| --- | --- |
| `double` |  |

## `clip`

Clip a value to a specified range

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `value` | `double` | value to be clipped |
| `min` | `double` | minimum value |
| `max` | `double` | maximum value |

### Returns

| Type | Description |
| --- | --- |
| `double` | clipped value |
