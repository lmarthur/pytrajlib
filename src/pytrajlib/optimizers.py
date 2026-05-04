from copy import deepcopy

import numpy as np
from scipy.optimize import minimize

from pytrajlib._traj import lib as traj
from pytrajlib.runtime import (
    _LOADING_BAR_DISABLED,
    _keep_alive,
    create_runparams_struct,
    impact_data_to_df,
)
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
    for name, value in zip(parameter_names, parameter_values):
        config_dict[name] = float(value)

    rp = create_runparams_struct(config_dict)
    impact_df = impact_data_to_df(traj.mc_run(rp[0]), config_dict)
    return np.mean(
        get_miss_distance(
            impact_df=impact_df,
            aimpoint=(config_dict["x_aim"], config_dict["y_aim"], config_dict["z_aim"]),
        )
    )


def _run_nelder_mead(config_dict, parameter_names, x0, bounds, extra_updates, log_fn):
    _keep_alive["loading_bar"] = _LOADING_BAR_DISABLED
    objective_config = _prepare_optimizer_config(config_dict, extra_updates)

    def objective(parameter_values):
        miss_dist = _evaluate_candidate(
            objective_config, parameter_names, parameter_values
        )

        _keep_alive.clear()
        _keep_alive["loading_bar"] = _LOADING_BAR_DISABLED
        print(log_fn(miss_dist, parameter_values))

        return miss_dist

    result = minimize(
        objective,
        x0=tuple(float(value) for value in x0),
        method="Nelder-Mead",
        bounds=bounds,
        options=dict(maxfev=200),
    )

    print(result)
    return tuple(float(value) for value in result.x)


def optimize_boost(config_dict):
    """
    Tune initial thrust angle and desired flight time for an optimally lofted
    flight.
    """
    range_km = config_dict["range"] / 1000.0
    tf_des = 188 + 0.223 * range_km - 4e-6 * range_km**2
    theta_long = 0.9

    return _run_nelder_mead(
        config_dict,
        parameter_names=("t_des_final", "theta_long"),
        x0=(tf_des, theta_long),
        bounds=[(300, 5000), (0, np.pi)],
        extra_updates={
            "gnss_nav": 0,
            "perfect_boost": 1,
            "rv_maneuv": 0,
        },
        log_fn=lambda miss_dist, values: (
            f"{miss_dist=:.9f} (avg), {float(values[0]):.9f}, {float(values[1]):.9f}"
        ),
    )


def optimize_maneuv(config_dict):
    """
    Tune tau_deflect and nav_gain for realistic RV maneuverability.
    """
    return _run_nelder_mead(
        config_dict,
        parameter_names=("tau_deflect", "nav_gain"),
        x0=(
            config_dict["tau_deflect"],
            config_dict["nav_gain"],
        ),
        bounds=[(1e-3, 100), (1, 5)],
        extra_updates={
            "gnss_nav": 1,
            "perfect_boost": 1,
            "rv_maneuv": 1,
        },
        log_fn=lambda miss_dist, values: (
            f"{miss_dist=:.9f} (avg), {float(values[0]):.9f}, {float(values[1]):.9f}"
        ),
    )
