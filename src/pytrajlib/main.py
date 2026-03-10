import argparse
import importlib.resources
import os
import tempfile
import time
import tomllib
from copy import deepcopy
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.optimize import minimize
from tqdm.auto import tqdm

from pytrajlib._traj import ffi
from pytrajlib._traj import lib as traj

# Import plotting functions
from pytrajlib.plotting import plot_impact, plot_trajectory
from pytrajlib.utils import get_miss_distance

# Create a package-specific temp directory
_TEMP_DIR = os.path.join(tempfile.gettempdir(), "pytrajlib")
os.makedirs(_TEMP_DIR, exist_ok=True)

_keep_alive = {}
_LOADING_BAR_DISABLED = object()


@ffi.def_extern()
def update_loading_bar(n, total):
    """
    Create or update the loading bar with the current progress and total.
    This is called from the C code.
    """
    loading_bar = _keep_alive.get("loading_bar")
    if loading_bar is _LOADING_BAR_DISABLED:
        return
    if loading_bar is None:
        _keep_alive["loading_bar"] = tqdm(total=total, desc="Progress")
        _keep_alive["loading_bar"].update(n=n)
    else:
        current_progress = loading_bar.n
        update_size = n - current_progress
        loading_bar.update(n=update_size)
        loading_bar.refresh()


def _get_default_config():
    config = importlib.resources.files("pytrajlib.config").joinpath(
        "maneuv_realistic.toml"
    )
    with open(config) as f:
        config_dict = tomllib.loads(f.read())
    config_dict = {
        **config_dict["RUN"],
        **config_dict["FLIGHT"],
        **config_dict["VEHICLE"],
        **config_dict["ERRORPARAMS"],
    }
    return config_dict


# Sentinel to detect non-provided arguments
_UNSET = object()


def to_c_type(value):
    """
    Convert a Python value to its corresponding C type.

    Args
        value: any
            The value to convert.

    Returns
        c_value: ctype
            The converted value.

    """
    if isinstance(value, str):
        s = ffi.new("char[]", value.encode("utf-8"))
        return s
    return value


def create_runparams_struct(config_dict):
    """
    Set the the run_params struct from the config.

    Args
        config_dict: The configuration dictionary.
    Returns
        run_params: runparams struct
    """
    run_params_struct = ffi.new("runparams *")
    for key, value in config_dict.items():
        p = to_c_type(value)
        run_params_struct.__setattr__(key, p)
        _keep_alive[key] = p
    return run_params_struct


def impact_data_to_df(impact_data, num_runs):
    """
    Convert the impact data to a Pandas DataFrame.

    Args
        impact_data: impact_data
            The impact data from the Monte Carlo run.
        num_runs: int
            The number of runs in the Monte Carlo simulation.

    Returns
        impact_df: pd.DataFrame
            The impact data as a Pandas DataFrame.
    """
    impact_df = pd.DataFrame()
    rows = []
    for i in range(num_runs):
        row_data = dict(
            t=impact_data.impact_times[i],
            x=impact_data.impact_states[i].position.x,
            y=impact_data.impact_states[i].position.y,
            z=impact_data.impact_states[i].position.z,
        )
        rows.append(row_data)
    impact_df = pd.DataFrame(rows)
    return impact_df


def optimize_trajectory(config_dict):
    """
    Find optimal desired time of flight, thrust angle inside the atmosphere
    """
    _keep_alive["loading_bar"] = _LOADING_BAR_DISABLED
    without_error_params = deepcopy(config_dict)
    without_error_params["num_runs"] = 1
    without_error_params["initial_pos_error"] = 0
    without_error_params["initial_vel_error"] = 0
    without_error_params["initial_angle_error"] = 0
    without_error_params["acc_scale_stability"] = 0
    without_error_params["gyro_bias_stability"] = 0
    without_error_params["gyro_noise"] = 0
    without_error_params["gnss_noise"] = 0

    without_error_params["rv_maneuv"] = 0
    without_error_params["atm_model"] = 0

    def obj(params):
        t_des_final, thrust_lon = params
        without_error_params["t_des_final"] = t_des_final
        without_error_params["theta_long"] = thrust_lon
        rp = create_runparams_struct(without_error_params)

        impact_df = impact_data_to_df(traj.mc_run(rp[0]), config_dict["num_runs"])
        miss_dist = np.sum(
            get_miss_distance(
                impact_df=impact_df,
                aimpoint=(
                    without_error_params["x_aim"],
                    without_error_params["y_aim"],
                    without_error_params["z_aim"],
                ),
            )
        )

        _keep_alive.clear()
        _keep_alive["loading_bar"] = _LOADING_BAR_DISABLED
        print(f"{miss_dist=:.9f}, {t_des_final=:.9f}, {thrust_lon=:.9f}")

        return miss_dist

    # Set desired time
    # Calculate the range to the aimpoint over the surface of the Earth
    # This is the great circle distance between the aimpoint and the origin
    aimpoint_lon = np.arctan2(config_dict["y_aim"], config_dict["x_aim"])
    aimpoint_lat = np.arctan2(
        config_dict["z_aim"],
        np.sqrt(config_dict["x_aim"] ** 2 + config_dict["y_aim"] ** 2),
    )

    range_to_aimpoint = np.arccos(
        np.sin(aimpoint_lat) * np.sin(0)
        + np.cos(aimpoint_lat) * np.cos(0) * np.cos(aimpoint_lon)
    )
    range_km = range_to_aimpoint * 6371

    tf_des = 252.0 + 0.223 * range_km - (5.44e-6) * range_km**2

    result = minimize(
        obj,
        x0=(tf_des, without_error_params["theta_long"]),
        method="Nelder-Mead",
        bounds=[(300, 5000), (0, np.pi)],
    )
    print(result)
    return result.x


def run(config=None, plot=False, plot_path=None, **kwargs):
    """Load config, override with kwargs, and run the C Monte Carlo code."""
    config_dict = {}

    # Load config file if provided
    if config is not None:
        print(f"Loading config from: {config}")
        with open(config) as f:
            config_dict = tomllib.loads(f.read())
            config_dict = {
                **config_dict.get("RUN", {}),
                **config_dict.get("FLIGHT", {}),
                **config_dict.get("VEHICLE", {}),
                **config_dict.get("ERRORPARAMS", {}),
            }
        print(f"Loaded from TOML: {config_dict}")
    else:
        config_dict = _get_default_config().copy()
        print("No config file, using built-in defaults")

    explicit_kwargs = {k: v for k, v in kwargs.items() if v is not _UNSET}

    if explicit_kwargs:
        print(f"Overriding with CLI args: {explicit_kwargs}")
        config_dict.update(explicit_kwargs)

    atm_path = str(
        importlib.resources.files("pytrajlib.config").joinpath("atmprofiles.txt")
    )
    mean_atm_path = str(
        importlib.resources.files("pytrajlib.config").joinpath("mean_atm.txt")
    )
    config_dict["atm_path"] = atm_path
    config_dict["mean_atm_path"] = mean_atm_path
    config_dict["trajectory_path"] = _TEMP_DIR + "/trajectory.txt"
    config_dict.setdefault("include_drag", 1)

    # config_dict['t_des_final'] = tf_des

    print("Final config:", config_dict)
    print("Running...")

    # t_des_final, thrust_lon = optimize_trajectory(config_dict)
    t_des_final, thrust_lon = 1851.429509858, 1.147028114

    config_dict["t_des_final"] = t_des_final
    config_dict["theta_long"] = thrust_lon
    rp = create_runparams_struct(config_dict)
    impact_df = impact_data_to_df(traj.mc_run(rp[0]), config_dict["num_runs"])
    aimpoint = (config_dict["x_aim"], config_dict["y_aim"], config_dict["z_aim"])
    miss_distance = get_miss_distance(impact_df=impact_df, aimpoint=aimpoint)
    # Load trajectory data if plotting
    if plot:
        trajectory_df = pd.read_csv(
            config_dict["trajectory_path"], skipinitialspace=True
        )
        save_path = Path(plot_path) if plot_path else None

        print("Generating impact plot...")
        plot_impact(impact_df, save_path=save_path, aimpoint=aimpoint)

        print("Generating trajectory plots...")
        plot_trajectory(trajectory_df, save_path=save_path)

    print(impact_df)
    print(f"CEP={np.quantile(miss_distance, 0.5)}")
    print("Done!")
    _keep_alive.clear()

    return impact_df


def cli():
    parser = argparse.ArgumentParser(
        description="Trajectory Simulation",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument(
        "--config", type=str, default=None, help="Path to TOML config file"
    )
    parser.add_argument(
        "--plot", action="store_true", help="Generate and display plots"
    )
    parser.add_argument(
        "--plot-path",
        type=str,
        default=None,
        help="Path to save plots (current directory if not specified with --plot)",
    )

    for param_name, default_value in _get_default_config().items():
        parser.add_argument(
            f"--{param_name.replace('_', '-')}",
            type=type(default_value),
            default=_UNSET,
            help=f"Default: {default_value}",
        )

    args = parser.parse_args()
    kwargs = vars(args)
    config = kwargs.pop("config")
    plot = kwargs.pop("plot")
    plot_path = kwargs.pop("plot_path")

    # Set default plot path to current directory if --plot is set but --plot-path is not
    if plot and plot_path is None:
        plot_path = os.getcwd()

    run(config=config, plot=plot, plot_path=plot_path, **kwargs)


if __name__ == "__main__":
    cli()
