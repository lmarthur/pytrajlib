# Sensors

## `imu_init`

Initialize IMU parameters and initial gyro error states.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `run_params` | `runparams *` | Pointer to run configuration parameters |
| `initial_state` | `state *` | Pointer to initial vehicle state |

### Returns

| Type | Description |
| --- | --- |
| `imu` | Initialized IMU model |

## `imu_measurement`

The missile's guidance system uses an inertial measurement unit to obtain a
noisy reading of the current acceleration. An accelerometer in free fall
measures no acceleration, so the measurable acceleration is the acceleration of
the vehicle without gravity
$$
\begin{equation}
\mathbf a_{\text{measurable},E} = \mathbf a_E - \mathbf a_{\text{grav},E}.
\end{equation}
$$

The accelerometer measures along the body frame
$$
\begin{equation}
\mathbf a_{\text{measurable},B} = \mathbf C_{BE} \mathbf
a_{\text{measurable},E}.
\end{equation}
$$

Constant Gaussian-distributed scale factor errors, $\varepsilon_i$, introduce
discrepancies between the measured acceleration and the actual measurable
acceleration of the vehicle:
$$
\mathbf a_\text{measured,B} =
\begin{bmatrix}
1 + \varepsilon_1 & 0 & 0 \\\
0 & 1 + \varepsilon_2 & 0 \\\
0 & 0 & 1 + \varepsilon_3
\end{bmatrix}
\mathbf a_{\text{measurable},B}.
$$

Transform the measured acceleration to the ECI basis:
$$
\begin{equation}
\mathbf a_{\text{measured},E} = \mathbf C_{EB} \mathbf a_{\text{measured},B}.
\end{equation}
$$

And add the estimated acceleration due to gravity based on the current estimated
position
$$
\begin{equation}
  \mathbf a_{\text{est},E} = \mathbf a_{\text{measured},E} + \mathbf
a_{\text{grav,est},E}.
\end{equation}
$$


The gyroscope model outputs a noisy measurement of the current angular velocity.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `imu` | `imu *` | Pointer to IMU model/state |
| `run_params` | `runparams *` | Pointer to run configuration parameters |
| `true_state` | `state *` | Pointer to true vehicle state |
| `est_state` | `state *` | Pointer to estimated vehicle state to update |
| `a_total_true` | `cartvec` | Total acceleration of true state in m/s^2. |
| `a_grav_true` | `cartvec` | Gravitational acceleration at true position in m/s^2. |
| `a_grav_est` | `cartvec` | Gravitational acceleration at estimated position in m/s^2. |

### Returns

| Type | Description |
| --- | --- |
| `cartvec` | Measured total estimated acceleration in ECI coordinates in m/s^2. |

## `get_gyro_drift`

Get drift component of gyro error (scales with dt)

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `imu` | `imu *` |  |

### Returns

| Type | Description |
| --- | --- |
| `cartvec` |  |

## `get_gyro_diffusion`

Get stochastic diffusion component of gyro error (scales with sqrt(dt))

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `imu` | `imu *` |  |

### Returns

| Type | Description |
| --- | --- |
| `double` |  |

## `gnss_init`

Initialize GNSS measurement model.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `run_params` | `runparams *` | Pointer to run configuration parameters |

### Returns

| Type | Description |
| --- | --- |
| `gnss` | Initialized GNSS model |

## `gnss_measurement`

Apply GNSS position measurement model to update estimated state position.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `gnss` | `gnss *` | Pointer to GNSS model |
| `true_state` | `state *` | Pointer to true vehicle state |
| `est_state` | `state *` | Pointer to estimated state to update |

### Returns

| Type | Description |
| --- | --- |
| `void` | None. |

## `stellar_measurement`

Apply a star-tracker attitude fix, overwriting the estimated attitude with a
noisy measurement of the true attitude.

Above the atmosphere the vehicle can sight known stars and solve for its
inertial attitude directly, which resets the attitude error the gyros have
accumulated since launch. The tracker does not recover the attitude exactly:
centroiding error, catalogue error and boresight misalignment leave a small
residual, modelled here as a random body-frame rotation with a per-axis
standard deviation of `noise` radians,
$$
\begin{equation}
  \mathbf q_{EB,\text{est}} = \mathbf q_{EB,\text{true}} \otimes
  \mathbf q(\boldsymbol\varepsilon_B), \qquad
  \varepsilon_{B,i} \sim \mathcal N(0, \sigma^2).
\end{equation}
$$
The error vector is given in body components, so it composes on the right,
the same way the gyro's increment does in the integrator. It is taken
isotropic for simplicity; a real tracker is several times more accurate
about the two cross-boresight axes than about the boresight itself.

The gyro bias is left untouched, so the attitude error starts growing again
from this residual immediately after the fix.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `true_state` | `state *` | Pointer to true vehicle state |
| `est_state` | `state *` | Pointer to estimated state to update |
| `noise` | `double` | Per-axis attitude measurement standard deviation in radians |

### Returns

| Type | Description |
| --- | --- |
| `void` | None. |