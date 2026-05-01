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

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `imu` | `imu *` | Pointer to IMU model/state |
| `run_params` | `runparams *` | Pointer to run configuration parameters |
| `true_atm_cond` | `atm_cond *` | Pointer to true atmospheric conditions |
| `est_atm_cond` | `atm_cond *` | Pointer to estimated atmospheric conditions |
| `true_t` | `double` | Current true simulation time in seconds |
| `est_t` | `double` | Current estimated simulation time in seconds |
| `true_state` | `state *` | Pointer to true vehicle state |
| `est_state` | `state *` | Pointer to estimated vehicle state to update |
| `a_total_true` | `cartvec` | Total acceleration of true state in m/s^2. |
| `a_grav_true` | `cartvec` | Gravitational acceleration at true position in m/s^2. |
| `a_grav_est` | `cartvec` | Gravitational acceleration at estimated position in m/s^2. |
| `est_grav` | `grav *` | Pointer to estimated gravity model. |

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
| `anglevec` |  |

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

## `perfect_measurement`

Set estimated state equal to true state (ideal measurement).

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `true_state` | `state *` | Pointer to true vehicle state |
| `est_state` | `state *` | Pointer to estimated vehicle state |

### Returns

| Type | Description |
| --- | --- |
| `void` | None. |
