# Drag

## `get_drag_acceleration_generic`

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `t` | `double` | Current simulation time in seconds |
| `current_state` | `state *` | Current vehicle state |
| `atm_cond` | `atm_cond *` | Pointer to atmospheric conditions |
| `vehicle` | `vehicle *` | Pointer to vehicle model |
| `c_d` | `double` | Drag coefficient |
| `area` | `double` | Reference area in square meters |

### Returns

| Type | Description |
| --- | --- |
| `static inline cartvec` | Drag acceleration in inertial Cartesian coordinates |
