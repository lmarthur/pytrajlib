# Integrator

## `integrate_quaternion_step`

The quaternion update step is implemented as a function of the incremental
angular change across a single step size. The incremental angular change is
calculated with the SRA3 algorithm. The quaternion rotation increment with
$\theta = |\boldsymbol  \theta_B|$ is
$$
\begin{equation}
  \Delta \mathbf q = \begin{bmatrix}
    \cos(\theta/2) \\
    \sin(\theta/2) \hat{\boldsymbol{\mathbf\theta}}\_B
  \end{bmatrix},
\end{equation}
$$
so the update is
$$
\begin{equation}
  \mathbf q_{EB,t+1} = \mathbf q_{EB,t} \otimes \Delta \mathbf q
\end{equation}
$$
where the $\otimes$ operator is the Hamilton product.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `current_state` | `state` |  |

### Returns

| Type | Description |
| --- | --- |
| `static inline quaternion` |  |

## `euler_maruyama_step`

Advance the state by one Euler-Maruyama step. Note, it is not strictly EM
because the position is updated using the velocity and the acceleration.

This integrator is currently not used in the trajectory simulation path.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `run_params` | `runparams *` | Pointer to run configuration parameters. |
| `imu` | `imu *` | Pointer to IMU model. |
| `vehicle` | `vehicle *` | Pointer to vehicle model. |
| `true_grav` | `grav *` | Pointer to true gravity model. |
| `est_grav` | `grav *` | Pointer to estimated gravity model. |
| `true_atm_cond` | `atm_cond *` | Pointer to true atmospheric conditions. |
| `est_atm_cond` | `atm_cond *` | Pointer to estimated atmospheric conditions. |
| `true_state` | `state *` | Pointer to true state, updated in place. |
| `est_state` | `state *` | Pointer to estimated state, updated in place. |
| `true_t` | `double *` | Pointer to true simulation time, incremented by `time_step`. |
| `est_t` | `double *` | Pointer to estimated simulation time, incremented by<br>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`time_step`. |
| `time_step` | `double` | Integration time step in seconds. |
| `drift_fn` | `drift_func` | Drift callback used to compute deterministic derivatives. |
| `diffusion_fn` | `diffusion_func` | Diffusion callback used to compute stochastic<br>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;derivatives. |

### Returns

| Type | Description |
| --- | --- |
| `int` | 1 on success, 0 if the drift callback reports failure. |

## `sra3_step`

Stochastic differential equations admit two standard interpretations, Itô and
Stratonovich, which correspond to left-endpoint and midpoint evaluation rules,
respectively. These generally yield different solutions but are equivalent for
stochastic differential equations with additive noise, as is the case in our
simulation, where the gyroscope noise is assumed to be constant.

We integrate the state using the Stochastic Runge-Kutta for Additive Noise
(SRA3) method, as described by Rössler (2010). SRA3 is of strong order 3 for
deterministic differential equations and strong order 1.5 for stochastic
differential equations. This is a significant improvement over the standard
Euler-Maruyama method, which is of strong order 1 for drift and diffusion with
additive noise, as discussed by Higham and Kloeden (2021), especially because
the deterministic components drive most of the dynamics. Higher-order numerical
integrators allow using larger time steps without sacrificing integration
accuracy.

Solutions to stochastic differential equations are probability distributions
over trajectories. We use a Monte Carlo approach to sample from both the initial
error terms and the solution to the stochastic differential equation.

### Parameters

| Name | Type | Description |
| --- | --- | --- |
| `run_params` | `runparams *` | Pointer to run configuration parameters. |
| `imu` | `imu *` | Pointer to IMU model. |
| `vehicle` | `vehicle *` | Pointer to vehicle model. |
| `true_grav` | `grav *` | Pointer to true gravity model. |
| `est_grav` | `grav *` | Pointer to estimated gravity model. |
| `true_atm_cond` | `atm_cond *` | Pointer to true atmospheric conditions. |
| `est_atm_cond` | `atm_cond *` | Pointer to estimated atmospheric conditions. |
| `true_state` | `state *` | Pointer to true state, updated in place. |
| `est_state` | `state *` | Pointer to estimated state, updated in place. |
| `true_t` | `double *` | Pointer to true simulation time, incremented by `time_step`. |
| `est_t` | `double *` | Pointer to estimated simulation time, incremented by<br>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`time_step`. |
| `time_step` | `double` | Integration time step in seconds. |
| `drift_fn` | `drift_func` | Drift callback used for deterministic stage evaluations. |
| `diffusion_fn` | `diffusion_func` | Diffusion callback used for additive-noise stage<br>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;evaluations. |

### Returns

| Type | Description |
| --- | --- |
| `int` | 1 on success, 0 if any drift stage reports failure. |