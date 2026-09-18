# Vehicle

## `get_spent_booster_mass`

Booster hardware still attached once every stage has burned out.

Earlier stages are jettisoned as they burn out, so only the bus and the dry
mass of the final stage remain.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `vehicle` | `vehicle *` | Pointer to vehicle struct. |

### Returns

| Type | Description |
| --- | --- |
| `static inline double` |  |

## `get_vehicle_mass`

Updates vehicle mass based on stage burn timing.

After burnout the mass is that of the reentry vehicle alone, unless the
reentry vehicle never separates, in which case the spent booster mass is
carried along with it.

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