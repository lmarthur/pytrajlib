# Vehicle

## `get_vehicle_mass`

Updates vehicle mass based on stage burn timing.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `vehicle` | `vehicle *` | Pointer to vehicle struct. |
| `t` | `double` | Current simulation time in seconds. |

### Returns

| Type | Description |
| --- | --- |
| `double` |  |

## `apply_burn_time_error`

Apply independent burn-time error to each booster stage.

Recomputes stage burn rates and total burn time from perturbed stage times.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `run_params` | `runparams *` | Pointer to run configuration parameters. |
| `vehicle` | `vehicle *` | Pointer to vehicle struct. |

### Returns

| Type | Description |
| --- | --- |
| `static inline void` |  |
