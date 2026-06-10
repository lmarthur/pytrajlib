# Trajectory

## `flight_path_angle`

Calculate flight path angle relative to local horizon.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `position` | `cartvec` | Position vector |
| `velocity` | `cartvec` | Velocity vector |

### Returns

| Type | Description |
| --- | --- |
| `static inline double` | Flight path angle in radians |

## `set_entry_angle`

See Regan 6.7 "Deployment Attitudes"

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `true_state` | `state *` |  |
| `est_state` | `state *` |  |
| `run_params` | `runparams *` |  |
| `vehicle` | `vehicle *` |  |
| `grav` | `grav *` |  |

### Returns

| Type | Description |
| --- | --- |
| `static inline void` |  |

## `impact_linterp`

Interpolate between two states to estimate impact crossing at altitude 0.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `state_0` | `state *` | Pointer to pre-impact state |
| `state_1` | `state *` | Pointer to post-impact state |
| `t0` | `double` |  |
| `t1` | `double` |  |
| `impact_t` | `double *` |  |

### Returns

| Type | Description |
| --- | --- |
| `state` | Interpolated impact state |

## `impact_with_coriolis`

In an ECI frame, assuming boost and reentry guidance account for Earth
rotation, the discrepancy between true and estimated trajectories due to
Coriolis appears through the difference between true and estimated impact
times. For random aimpoints with fixed great-circle range, rotation speed is
adjusted from equatorial surface speed to local latitude:
$$
\begin{align}
v_\text{rot} = 464 \cos(\text{lat}).
\end{align}
$$
With interpolated impact-time error
$$
\begin{align}
\Delta t = t_\text{est} - t_\text{true},
\end{align}
$$
the Coriolis offset is
$$
\begin{align}
c = v_\text{rot} \Delta t.
\end{align}
$$
In the local east direction, Cartesian offsets are
$$
\begin{align}
\Delta x &= -c \sin(\text{lon}) \\
\Delta y &= c \cos(\text{lon}).
\end{align}
$$
The corrected impact position is $(x + \Delta x, y + \Delta y, z)$.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `old_true_state` | `state *` | Pointer to pre-impact true state |
| `true_state` | `state *` | Pointer to post-impact true state |
| `old_true_t` | `double` | Pre-impact true time |
| `true_t` | `double` | Post-impact true time |
| `old_est_state` | `state *` | Pointer to pre-impact estimated state |
| `est_state` | `state *` | Pointer to post-impact estimated state |
| `old_est_t` | `double` | Pre-impact estimated time |
| `est_t` | `double` | Post-impact estimated time |
| `run_params` | `runparams *` | Pointer to run configuration parameters |
| `true_final_t` | `double *` | Output interpolated true impact time |
| `est_final_state` | `state *` | Output interpolated estimated impact state |

### Returns

| Type | Description |
| --- | --- |
| `state` | Corrected true impact state |

## `output_impact`

Write impact state data for all Monte Carlo runs to a file stream.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `impact_file` | `FILE *` | Output file stream |
| `impact_data` | `impact_data *` | Pointer to impact data container |
| `num_runs` | `int` | Number of Monte Carlo runs to write |

### Returns

| Type | Description |
| --- | --- |
| `void` | None. |

## `fly`

Simulate a single trajectory until impact and return final impact state.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `run_params` | `runparams *` | Pointer to run configuration parameters |
| `initial_state` | `state *` | Pointer to initial true state |
| `vehicle` | `vehicle *` | Pointer to vehicle model/state |
| `impact_time` | `double *` |  |
| `burnout_vel_mag` | `double *` | Output burnout speed in m/s |
| `burnout_alt` | `double *` | Output burnout altitude in m |
| `burnout_ang` | `double *` | Output burnout flight-path angle in radians |
| `apogee_alt` | `double *` | Output maximum altitude (apogee) in m |
| `reentry_vel` | `double *` | Output speed at 120 km reentry crossing in m/s |
| `reentry_ang` | `double *` | Output flight-path angle at reentry crossing in radians |

### Returns

| Type | Description |
| --- | --- |
| `state` | Final impact state |

## `mc_run`

Run a Monte Carlo trajectory simulation.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `run_params` | `runparams` | Run configuration parameters |

### Returns

| Type | Description |
| --- | --- |
| `impact_data` | Impact states for a single run |
