# Derivatives

## `drift`

Calculate deterministic drift component of the state update.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `run_params` | `runparams *` |  |
| `imu` | `imu *` |  |
| `vehicle` | `vehicle *` |  |
| `true_grav` | `grav *` |  |
| `est_grav` | `grav *` |  |
| `true_atm_cond` | `atm_cond *` |  |
| `est_atm_cond` | `atm_cond *` |  |
| `true_state` | `state *` |  |
| `est_state` | `state *` |  |
| `true_t` | `double` |  |
| `est_t` | `double` |  |
| `true_state_drift` | `state *` |  |
| `est_state_drift` | `state *` |  |

### Returns

| Type | Description |
| --- | --- |
| `int` |  |
