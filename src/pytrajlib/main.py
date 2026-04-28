import argparse
import importlib.resources
import os
import tomllib
from pathlib import Path

import numpy as np
import pandas as pd

from pytrajlib._traj import lib as traj

# Import plotting functions
from pytrajlib.optimizers import optimize_boost, optimize_maneuv
from pytrajlib.runtime import (
    _LOADING_BAR_DISABLED,
    _TEMP_DIR,
    _UNSET,
    _get_default_config,
    _keep_alive,
    _set_aimpoint_from_range,
    create_runparams_struct,
    impact_data_to_df,
)
from pytrajlib.plotting import plot_impact, plot_reentry_guidance, plot_trajectory
from pytrajlib.utils import get_miss_distance


def run(
    config: str = None,
    plot: bool = False,
    plot_path: str = None,
    return_config=False,
    **kwargs,
):
    """Load config, override with kwargs, and run the C Monte Carlo code.

    Args
        config: path to config file
        plot: whether to save plots
        plot_path: path to save plots
        **kwargs: all other kwargs (see default TOML)
    """
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
    else:
        config_dict = _get_default_config().copy()

    explicit_kwargs = {k: v for k, v in kwargs.items() if v is not _UNSET}

    if explicit_kwargs:
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
    config_dict.setdefault("optimize_boost", 1)
    config_dict.setdefault("optimize_maneuv", 0)
    config_dict.setdefault("Glp", 1)
    config_dict.setdefault("random_seed", -1)
    _set_aimpoint_from_range(config_dict)

    print("Running...")

    if config_dict["optimize_boost"]:
        t_des_final, thrust_lon = optimize_boost(config_dict)
        print(f"{t_des_final=}, {thrust_lon=}")
        config_dict["t_des_final"] = t_des_final
        config_dict["theta_long"] = thrust_lon
    else:
        print("Skipping boost optimization; using configured t_des_final/theta_long")

    if config_dict["optimize_maneuv"]:
        if int(config_dict.get("rv_maneuv", 0)) == 1:
            tau_deflect, Glp, nav_gain = optimize_maneuv(config_dict)
            print(f"{tau_deflect=}, {Glp=}, {nav_gain=}")
            config_dict["tau_deflect"] = tau_deflect
            config_dict["Glp"] = Glp
            config_dict["nav_gain"] = nav_gain
        else:
            print("Skipping maneuverability optimization; requires rv_maneuv = 1")

    _keep_alive["loading_bar"] = None

    rp = create_runparams_struct(config_dict)
    impact_df = impact_data_to_df(traj.mc_run(rp[0]), config_dict)
    aimpoint = (config_dict["x_aim"], config_dict["y_aim"], config_dict["z_aim"])
    miss_distance = get_miss_distance(impact_df=impact_df, aimpoint=aimpoint)
    impact_df["miss_distance"] = miss_distance
    # Load trajectory data if plotting
    if plot:
        trajectory_df = pd.read_csv(
            config_dict["trajectory_path"], skipinitialspace=True
        )
        save_path = Path(plot_path) if plot_path else None
        reentry_guidance_path = Path(config_dict["trajectory_path"]).with_name(
            "reentry_guidance.csv"
        )

        print("Generating impact plot...")
        plot_impact(impact_df, save_path=save_path, aimpoint=aimpoint)

        print("Generating trajectory plots...")
        plot_trajectory(trajectory_df, save_path=save_path, aimpoint=aimpoint)

        if reentry_guidance_path.exists():
            guidance_df = pd.read_csv(reentry_guidance_path, skipinitialspace=True)
            print("Generating reentry guidance plot...")
            plot_reentry_guidance(guidance_df, save_path=save_path)
        else:
            print("Skipping reentry guidance plot (reentry_guidance.csv not found).")

    print(f"CEP={np.quantile(miss_distance, 0.5)}")
    print("Done!")
    _keep_alive.clear()

    if return_config:
        return impact_df, config_dict
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

    impact_df = run(config=config, plot=plot, plot_path=plot_path, **kwargs)
    print(impact_df)


if __name__ == "__main__":
    cli()
