from copy import deepcopy

import numpy as np
import optuna
from scipy.optimize import minimize

from pytrajlib import runtime
from pytrajlib.runtime import _keep_alive
from pytrajlib.utils import get_miss_distance

# The loft stage's two decision variables are rescaled to order 1 before being
# handed to Nelder-Mead: t_des_final is ~2400 s while theta_long is ~1 rad, and
# xatol is a single absolute number applied as a max over all coordinates, so
# without rescaling no one value suits both. Nelder-Mead builds its simplex by
# perturbing each coordinate 5% and all its moves are affine, so rescaling
# visits exactly the same physical points -- it only makes xatol interpretable.
LOFT_SCALES = np.array([1000.0, 1.0])  # (t_des_final [s], theta_long [rad])
LOFT_XATOL = 1e-3
LOFT_FATOL = 1e-1
T_DES_FINAL_BOUNDS = (1000.0, 6000.0)  # s
THETA_LONG_BOUNDS = (0.0, np.pi / 2)  # rad
LAMBERT_V_OFFSET_BOUNDS = (-0.1, 0.1)
LAMBERT_V_OFFSET_X0 = 0.0
LAMBERT_V_OFFSET_STEP = 0.01
LAMBERT_V_OFFSET_XATOL = 1e-6
LAMBERT_V_OFFSET_FATOL = 1e-3

# Moves the optimizer onto its own seed stream so that the optimizer and the runs
# are not using the same random seeds
OPTIMIZER_SEED_OFFSET = 1_000_000

# Starting point for the reentry search
REENTRY_START_POINT = {
    "max_deflection_angle": 5.0,
    "gearing_ratio": 1.0,
    "nav_gain_0": 10.0,
    "nav_gain_1": 1.0,
    "K_q": 10.0,
    "K_pp": 10.0,
    "K_delta_p": 1.0,
    "K_delta_d": 1.0,
}

# Zeroed for both boost and reentry
_DISABLED_ERROR_KEYS = (
    "initial_pos_error",
    "initial_vel_error",
    "initial_angle_error",
    "grav_error",
    "burn_time_error",
)

# Zeroed for the boost stage only. The reentry optimizer should be exposed to errors
# so that it actually attempts to maneuver --- without these errors the reentry
# maneuvering would not help much because the ballistic maneuver is already optimized.
_BOOST_ONLY_DISABLED_ERROR_KEYS = (
    "acc_scale_stability",
    "gyro_bias_stability",
    "gyro_noise",
    "gnss_noise",
)


def _prepare_optimizer_config(config_dict, extra_updates=None, disabled_keys=()):
    params = deepcopy(config_dict)
    params["traj_output"] = 0
    params["num_runs"] = params["num_runs_optimizer"]
    for key in (*_DISABLED_ERROR_KEYS, *disabled_keys):
        params[key] = 0
    if int(params["random_seed"]) >= 0:
        params["random_seed"] = int(params["random_seed"]) + OPTIMIZER_SEED_OFFSET
    if extra_updates:
        params.update(extra_updates)
    return params


def _evaluate_candidate(config_dict, parameter_names, parameter_values):
    """Loss function is mean miss distance."""
    config = deepcopy(config_dict)
    # num_processes is not part of runparams, so remove it from the config
    num_processes = config.pop("num_processes")
    for name, value in zip(parameter_names, parameter_values):
        config[name] = float(value)
    impact_df = runtime.run(
        config, min(config["num_runs"], num_processes), show_progress_bar=False
    )

    dist = get_miss_distance(
        impact_df=impact_df,
        aimpoint=(config["x_aim"], config["y_aim"], config["z_aim"]),
    )
    return np.mean(dist)


def _optimize_lambert_offset(objective_config, loft_params):
    """Second boost stage: trim the Lambert drag-loss budget.

    Runs with the loft solution from the first stage held fixed, so this is a
    one-dimensional Nelder-Mead search. `lambert_v_offset` is
    added straight to the Lambert velocity magnitude, so it shifts the impact
    downrange without reshaping the trajectory.

    Args:
        objective_config: optimizer config from `_prepare_optimizer_config`.
        loft_params: `t_des_final` and `theta_long` found by the first stage.

    Returns:
        float: the offset in m/s.
    """
    config = {**objective_config, **loft_params}

    def objective(xs):
        (lambert_v_offset,) = xs
        miss_dist = _evaluate_candidate(
            config,
            ("lambert_v_offset",),
            (lambert_v_offset,),
        )
        print(f"{lambert_v_offset=:.6f}, {miss_dist=:.6f}")
        return miss_dist

    result = minimize(
        fun=objective,
        x0=[LAMBERT_V_OFFSET_X0],
        method="Nelder-Mead",
        bounds=[LAMBERT_V_OFFSET_BOUNDS],
        options=dict(
            initial_simplex=[
                [LAMBERT_V_OFFSET_X0],
                [LAMBERT_V_OFFSET_X0 + LAMBERT_V_OFFSET_STEP],
            ],
            xatol=LAMBERT_V_OFFSET_XATOL,
            fatol=LAMBERT_V_OFFSET_FATOL,
            maxfev=objective_config["num_trials_optimizer"],
        ),
    )
    print(result)
    return float(result.x[0])


def optimize_boost(config_dict, num_processes):
    """
    Lambert Guidance assumes there is no drag upon reentry to the atmosphere. To overcome this limitation, we tune the initial thrust angle, the desired flight time, and the Lambert velocity offset (drag loss budget) for an optimally lofted flight, as described in the boost optimization section.

    The two are tuned in sequence rather than jointly: the loft search fixes how
    the trajectory is shaped, then the Lambert offset trims the leftover energy
    shortfall on that shape.
    """
    tf_des = 2000.0
    theta_long = np.pi / 4

    extra_updates = {
        "gnss_nav": 0,
        "perfect_boost": 0,
        "rv_maneuv": 0,
        # Important to note that the optimizer uses the exponential atmosphere wtih gaussian perturbations,
        # not the EarthGram atmospheres which are reserved for the actual simulation.
        # This prevents overfitting.
        "atm_model": 1 if config_dict["atm_model"] > 0 else 0,
        "num_processes": num_processes,
    }

    objective_config = _prepare_optimizer_config(
        config_dict, extra_updates, disabled_keys=_BOOST_ONLY_DISABLED_ERROR_KEYS
    )

    def objective(xs):
        tf, theta = np.asarray(xs) * LOFT_SCALES
        miss_dist = _evaluate_candidate(
            objective_config,
            ("t_des_final", "theta_long"),
            (tf, theta),
        )
        print(f"{tf=:.6f}, {theta=:.6f}, {miss_dist=:.6f}")
        return miss_dist

    print("Boost stage 1/2: loft (t_des_final, theta_long)")
    result = minimize(
        fun=objective,
        x0=np.array([tf_des, theta_long]) / LOFT_SCALES,
        method="Nelder-Mead",
        # Bounds are applied in the rescaled coordinates Nelder-Mead sees.
        bounds=[
            tuple(np.array(T_DES_FINAL_BOUNDS) / LOFT_SCALES[0]),
            tuple(np.array(THETA_LONG_BOUNDS) / LOFT_SCALES[1]),
        ],
        options=dict(
            xatol=LOFT_XATOL,
            fatol=LOFT_FATOL,
            maxfev=objective_config["num_trials_optimizer"],
        ),
    )
    print(result)
    tf_opt, theta_opt = result.x * LOFT_SCALES
    loft_params = {
        "t_des_final": tf_opt,
        "theta_long": theta_opt,
    }

    print("Boost stage 2/2: Lambert drag-loss budget (lambert_v_offset)")
    lambert_v_offset = _optimize_lambert_offset(objective_config, loft_params)

    return {**loft_params, "lambert_v_offset": lambert_v_offset}


def optimize_reentry(config_dict, num_processes):
    """
    Tune nav_gain and control gains for realistic RV maneuverability using Optuna.
    Returns the best parameter dictionary found by Optuna.
    """
    extra_updates = {
        "gnss_nav": 1,
        "perfect_boost": 0,
        "rv_maneuv": 1,
        "ballistic_drag": 0,
        # Important to note that the optimizer uses the exponential atmosphere wtih gaussian perturbations,
        # not the EarthGram atmospheres which are reserved for the actual simulation.
        # This prevents overfitting.
        "atm_model": 1 if config_dict["atm_model"] > 0 else 0,
        "num_processes": num_processes,
    }

    objective_config = _prepare_optimizer_config(config_dict, extra_updates)

    def optuna_objective(trial):
        # Suggest parameters to tune.
        max_deflection_angle = trial.suggest_float("max_deflection_angle", 0.0, 10.0)
        gearing_ratio = trial.suggest_float("gearing_ratio", 1, 100)
        nav_gain_0 = trial.suggest_float("nav_gain_0", 0.0, 100.0)
        nav_gain_1 = trial.suggest_float("nav_gain_1", 0.0, 100.0)
        # Include control gains in optimization
        K_q = trial.suggest_float("K_q", 0.0, 100.0)
        K_pp = trial.suggest_float("K_pp", 0.0, 100.0)
        K_delta_p = trial.suggest_float("K_delta_p", 0.0, 100.0)
        K_delta_d = trial.suggest_float("K_delta_d", 0.0, 100.0)

        parameter_names = (
            "max_deflection_angle",
            "gearing_ratio",
            "nav_gain_0",
            "nav_gain_1",
            "K_q",
            "K_pp",
            "K_delta_p",
            "K_delta_d",
        )

        parameter_values = (
            max_deflection_angle,
            gearing_ratio,
            nav_gain_0,
            nav_gain_1,
            K_q,
            K_pp,
            K_delta_p,
            K_delta_d,
        )

        sq_miss_dist = _evaluate_candidate(
            objective_config, parameter_names, parameter_values
        )

        return sq_miss_dist

    # Best to have the run config random seed > 0 for the optimizer to be deterministic
    sampler = optuna.samplers.CmaEsSampler(seed=0)
    study = optuna.create_study(sampler=sampler, direction="minimize")

    # Seed the first trial with reasonably good values
    study.enqueue_trial(dict(REENTRY_START_POINT))

    study.optimize(optuna_objective, n_trials=config_dict["num_trials_optimizer"])
    return study.best_params
