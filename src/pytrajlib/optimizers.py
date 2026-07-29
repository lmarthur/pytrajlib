from copy import deepcopy

import numpy as np
import optuna
from scipy.optimize import minimize

from pytrajlib import runtime
from pytrajlib.runtime import _keep_alive
from pytrajlib.utils import get_miss_distance

_DISABLED_ERROR_KEYS = (
    "initial_pos_error",
    "initial_vel_error",
    "initial_angle_error",
    "acc_scale_stability",
    "gyro_bias_stability",
    "gyro_noise",
    "gnss_noise",
    "grav_error",
    "burn_time_error",
)


def _prepare_optimizer_config(config_dict, extra_updates=None):
    params = deepcopy(config_dict)
    params["traj_output"] = 0
    params["num_runs"] = params["num_runs_optimizer"]
    for key in _DISABLED_ERROR_KEYS:
        params[key] = 0
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


def optimize_boost(config_dict, num_processes):
    """
    Lambert Guidance assumes there is no drag upon reentry to the atmosphere. To overcome this limitation, we tune the initial thrust angle, the desired flight time, and the Lambert velocity offset (drag loss budget) for an optimally lofted flight, as described in the boost optimization section.
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

    objective_config = _prepare_optimizer_config(config_dict, extra_updates)

    def objective(xs):
        tf, theta = xs
        miss_dist = _evaluate_candidate(
            objective_config,
            ("t_des_final", "theta_long"),
            (tf, theta),
        )
        print(f"{tf=:.6f}, {theta=:.6f}, {miss_dist=:.6f}")
        return miss_dist

    result = minimize(
        fun=objective,
        x0=[tf_des, theta_long],
        method="Nelder-Mead",
        options=dict(maxfev=objective_config["num_trials_optimizer"]),
    )
    print(result)
    return {
        "t_des_final": result.x[0],
        "theta_long": result.x[1],
    }


def optimize_reentry(config_dict, num_processes):
    """
    Tune nav_gain and control gains for realistic RV maneuverability using Optuna.
    Returns the best parameter dictionary found by Optuna.
    """
    # Optuna should run with a prepared objective config that disables
    # non-deterministic error sources and limits output.
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
        gearing_ratio = trial.suggest_float("gearing_ratio", 1, 50)
        nav_gain_0 = trial.suggest_float("nav_gain_0", 0.0, 100.0)
        nav_gain_1 = trial.suggest_float("nav_gain_1", 0.0, 100.0)
        # Include control gains in optimization
        K_q = trial.suggest_float("K_q", 0.0, 50.0)
        K_pp = trial.suggest_float("K_pp", 0.0, 50.0)
        K_delta_p = trial.suggest_float("K_delta_p", 0.0, 50.0)
        K_delta_d = trial.suggest_float("K_delta_d", 0.0, 50.0)

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

    sampler = optuna.samplers.TPESampler(seed=0)
    study = optuna.create_study(sampler=sampler, direction="minimize")

    # Seed the first trial with reasonably good values
    study.enqueue_trial(
        {
            "max_deflection_angle": 5.0,
            "gearing_ratio": 1.0,
            "nav_gain_0": 10,
            "nav_gain_1": 1,
            "K_q": 10,
            "K_pp": 10,
            "K_delta_p": 1.0,
            "K_delta_d": 1.0,
        }
    )

    study.optimize(optuna_objective, n_trials=config_dict["num_trials_optimizer"])
    return study.best_params
