# Atmosphere

## `init_atm_data`

Initializes atmospheric profile data so the file is read only once.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `atmprofilepath` | `char *` | Path to the atmospheric profile file. |

### Returns

| Type | Description |
| --- | --- |
| `void` | None. |

## `init_mean_atm_data`

Initializes mean atmospheric profile data so the file is read only once.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `atmprofilepath` | `char *` | Path to the mean atmospheric profile file. |

### Returns

| Type | Description |
| --- | --- |
| `void` | None. |

## `init_exp_atm`

Initializes the exponential atmosphere model and optional perturbations.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `run_params` | `runparams *` | Pointer to simulation run parameters. |

### Returns

| Type | Description |
| --- | --- |
| `atm_model` | Initialized atmosphere model. |

## `get_exp_atm_cond`

Calculates atmospheric conditions at a given altitude using an exponential
model.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `altitude` | `double` | Altitude in meters. |
| `atm_model` | `atm_model *` | Pointer to atmospheric model. |

### Returns

| Type | Description |
| --- | --- |
| `atm_cond` | Local atmospheric conditions. |

## `get_pert_atm_cond`

Calculates atmospheric conditions using the perturbed exponential model.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `altitude` | `double` | Altitude in meters. |
| `atm_model` | `atm_model *` | Pointer to atmospheric model. |

### Returns

| Type | Description |
| --- | --- |
| `atm_cond` | Local atmospheric conditions. |

## `get_eg_atm_cond`

Calculates atmospheric conditions at a given altitude using an EarthGRAM
2016 profile.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `altitude` | `double` | Altitude in meters. |
| `atm_profile` | `eg16_profile *` | Pointer to EarthGRAM 2016 profile. |

### Returns

| Type | Description |
| --- | --- |
| `atm_cond` | Local atmospheric conditions. |

## `get_atm_cond`

Calculates atmospheric conditions at a given altitude for the configured
atmosphere model.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `altitude` | `double` | Altitude in meters. |
| `exp_atm_model` | `atm_model *` | Pointer to exponential atmosphere model. |
| `run_params` | `runparams *` | Pointer to run parameters. |
| `atm_profile` | `eg16_profile *` | Pointer to EarthGRAM profile. |

### Returns

| Type | Description |
| --- | --- |
| `atm_cond` | Local atmospheric conditions. |

## `parse_atm`

Parses atmospheric profile data and selects the requested profile.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `atmprofilepath` | `char *` | Path to atmospheric profile data file. |
| `profilenum` | `int` | Profile index to use; `-1` selects the mean profile. |

### Returns

| Type | Description |
| --- | --- |
| `eg16_profile` | Atmospheric profile data. |

## `get_cart_wind`

Gets wind at the current location in standard Cartesian coordinates.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `state` | `state *` | Pointer to current vehicle state. |
| `atm_cond` | `atm_cond *` | Pointer to local atmospheric conditions. |

### Returns

| Type | Description |
| --- | --- |
| `cartvec` | Wind vector in Cartesian coordinates. |

## `get_relative_wind_eci`

Gets relative wind in ECI coordinates.

The relative wind is defined as wind minus vehicle velocity:
$$\mathbf V_{\mathrm{rel},E} = \mathbf V_{\mathrm{wind},E} - \mathbf v_E$$

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `current_state` | `state *` | Pointer to current vehicle state. |
| `atm_cond` | `atm_cond *` | Pointer to local atmospheric conditions. |

### Returns

| Type | Description |
| --- | --- |
| `static inline cartvec` | Relative wind vector in ECI coordinates. |
