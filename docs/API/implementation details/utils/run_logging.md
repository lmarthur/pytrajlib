# Run Logging

## `record_reentry_guidance_sample`

Record the guidance sample for the current integration step.

Called from the drift evaluation, which runs several times per step. Only
the first call after each flush is kept, so the retained sample is the one
taken at the step's own start time.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `t` | `double` |  |
| `a_cmd_E` | `cartvec` |  |
| `a_total_est` | `cartvec` |  |
| `desired_aoa_deg` | `double` |  |
| `desired_flap_deflection` | `cartvec` |  |

### Returns

| Type | Description |
| --- | --- |
| `static inline void` |  |

## `flush_reentry_guidance_log_row`

Write the recorded guidance sample, if there is one, and clear it.

Called once per integration step, so the log holds one row per step with
strictly increasing timestamps.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| (none) | - | - |

### Returns

| Type | Description |
| --- | --- |
| `static inline void` |  |