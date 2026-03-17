# Vehicle

## `init_ballistic_rv`

Initializes a ballistic reentry vehicle.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| (none) | - | - |

### Returns

| Type | Description |
| --- | --- |
| `rv` | Ballistic reentry vehicle parameters. |

## `init_swerve_rv`

Initializes a maneuverable reentry vehicle.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| (none) | - | - |

### Returns

| Type | Description |
| --- | --- |
| `rv` | Maneuverable reentry vehicle parameters. |

## `init_mmiii_booster`

Initializes a MMIII booster.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| (none) | - | - |

### Returns

| Type | Description |
| --- | --- |
| `booster` | MMIII booster parameters. |

## `apply_vehicle_overrides`

Apply optional vehicle parameter overrides.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `vehicle` | `vehicle *` | Pointer to vehicle to update |
| `run_params` | `runparams *` | Run parameters containing override values. |

### Returns

| Type | Description |
| --- | --- |
| `void` | None. |

## `init_mmiii_ballistic`

Initializes a MMIII vehicle carrying a ballistic reentry vehicle.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `run_params` | `runparams *` |  |

### Returns

| Type | Description |
| --- | --- |
| `vehicle` | Vehicle parameters. |

## `init_mmiii_swerve`

Initializes a MMIII vehicle carrying a maneuverable reentry vehicle.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `run_params` | `runparams *` | Run parameters containing optional override values. |

### Returns

| Type | Description |
| --- | --- |
| `vehicle` | Vehicle parameters. |

## `init_mock_booster`

Initializes a mock booster.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| (none) | - | - |

### Returns

| Type | Description |
| --- | --- |
| `booster` | Mock booster parameters. |

## `init_mock_rv`

Initializes a mock reentry vehicle.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| (none) | - | - |

### Returns

| Type | Description |
| --- | --- |
| `rv` | Mock reentry vehicle parameters. |

## `init_mock_vehicle`

Initializes a mock vehicle.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| (none) | - | - |

### Returns

| Type | Description |
| --- | --- |
| `vehicle` | Mock vehicle parameters. |

## `init_reentry_only`

Initializes a reentry-only vehicle configuration.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| (none) | - | - |

### Returns

| Type | Description |
| --- | --- |
| `vehicle` | Reentry-only vehicle parameters. |

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
