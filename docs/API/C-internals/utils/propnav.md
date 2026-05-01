# Propnav

## `prop_nav`

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `estimated_state` | `state *` | Pointer to the estimated vehicle state. |
| `run_params` | `runparams *` | Pointer to the run configuration parameters. |

### Returns

| Type | Description |
| --- | --- |
| `cartvec` | Commanded acceleration vector in ECI coordinates. |
