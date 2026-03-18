# State

## `init_true_state`

Initialize the true vehicle state at launch/reentry with stochastic
position, velocity, and attitude perturbations.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `run_params` | `runparams *` | Pointer to run configuration parameters |

### Returns

| Type | Description |
| --- | --- |
| `state` | Initialized true state |

## `init_est_state`

Initialize the estimated vehicle state without stochastic perturbations.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `run_params` | `runparams *` | Pointer to run configuration parameters |

### Returns

| Type | Description |
| --- | --- |
| `state` | Initialized estimated state |

## `add_state`

Add two states together component-wise

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `a` | `state` |  |
| `b` | `state` |  |

### Returns

| Type | Description |
| --- | --- |
| `state` |  |

## `smultiply_state`

Multiply each element of the state by a scalar double

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `a` | `state` |  |
| `s` | `double` |  |

### Returns

| Type | Description |
| --- | --- |
| `state` |  |
