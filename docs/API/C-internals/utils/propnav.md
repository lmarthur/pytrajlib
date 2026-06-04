# Propnav

## `prop_nav`

Use proportional navigation with a linear, altitude-based gain to return
commanded accelerations. Accounts for gravity by treating it as a target's
acceleration.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `estimated_state` | `state *` | Pointer to the estimated vehicle state. |
| `run_params` | `runparams *` | Pointer to the run configuration parameters. |
| `est_grav` | `grav *` |  |

### Returns

| Type | Description |
| --- | --- |
| `cartvec` | Commanded acceleration vector in ECI coordinates. |
