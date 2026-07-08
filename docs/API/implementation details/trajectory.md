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

The Coriolis effect is accounted for at the end of the simulation. It accounts
for miss distance at randomly selected aim points due to the Earth's rotation
and differences between the vehicle's true and guidance-system-estimated states.

The Coriolis correction algorithm is as follows:

First, select a random aimpoint latitude ($\phi \in [-\frac{\pi}{2},
\frac{\pi}{2}]$) and longitude ($\theta \in [0, 2\pi]$) angle by transforming
independent samples from a uniform distribution, as described in Weisstein's
Sphere Point Picking https://mathworld.wolfram.com/SpherePointPicking.html:
$$
\begin{align}
  u,v &\sim \text{Uniform}(0, 1)\\
  \theta &= 2 \pi u\\
  \phi &= \arccos(2 v - 1) - \frac{\pi}{2}.
\end{align}
$$

Second, determine the Coriolis miss distance, denoted $c$ below, based on the
Earth's rotation speed at the selected latitude and the time error between true
and estimated impact times. The Earth's rotation speed at any latitude is the
Earth's rotation at the equator, 464 m/s, multiplied by the cosine of the
latitude. This correction ensures if the missile impacts later than expected,
the impact point will be further west (and vice-versa):
$$
\begin{equation}
c = 464 \cos(\phi) (t_\text{est} - t_\text{true}).
\end{equation}
$$

Third, update the true impact position by projecting the additional miss
distance onto the true impact location. $c$ is always in the local east
direction, so there are only errors in the $x$ and $y$ components of the impact
location, which also means the error depends only on a single $\sin$ or $\cos$
of the longitude. The Coriolis-corrected impact position vector is
$$
\begin{align}
\mathbf{r} = \begin{bmatrix}
  x - c \sin\theta \\\
  y + c \cos\theta \\\
  z\end{bmatrix}.
\end{align}
$$

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

Run a Monte Carlo trajectory simulation. This will be called by the Python
wrapper.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `run_params` | `runparams` | Run configuration parameters |
| `vehicle` | `vehicle` |  |

### Returns

| Type | Description |
| --- | --- |
| `impact_data` | Impact states for a single run |