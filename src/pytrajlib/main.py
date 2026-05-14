import argparse
import importlib.resources
import os
import tomllib
from copy import deepcopy
from multiprocessing import Pool
from pathlib import Path

import numpy as np
import pandas as pd

# from tqdm.auto import tqdm
from tqdm.auto import tqdm

# Import plotting functions
from pytrajlib.optimizers import optimize_boost, optimize_maneuv
from pytrajlib.plotting import (
    create_impact_plot,
    create_traj_plots,
    plot_reentry_guidance,
)
from pytrajlib.runtime import (
    _LOADING_BAR_DISABLED,
    _TEMP_DIR,
    _UNSET,
    _get_default_config,
    _keep_alive,
    _set_aimpoint_from_range,
    impact_data_to_df,
)
from pytrajlib.utils import get_miss_distance

np.random.seed(0)


def _mc_run_wrapper(config_dict):
    """Wrapper function for multiprocessing.

    This function is pickled and executed in worker processes.
    It imports the CFFI library, runs mc_run, and converts the result to a
    picklable format to avoid CFFI serialization issues.
    """
    from pytrajlib._traj import lib as traj_lib
    from pytrajlib.runtime import create_runparams_struct

    rp = create_runparams_struct(config_dict)
    impact_data = traj_lib.mc_run(rp[0])
    df = impact_data_to_df(impact_data, config_dict)
    return df


def run(
    config: str = None,
    plot_trajectory: bool = False,
    plot_impact: bool = False,
    plot_path: str = None,
    num_processes: int = 10,
    return_config=False,
    **kwargs,
):
    """Load config, override with kwargs, and run the C Monte Carlo code.

    Args
        config: path to config file
        plot_trajectory: whether to save trajectory plots
        plot_impact: whether to save impact plot
        plot_path: path to save plots
        num_processes: number of concurrent processes on which to run simulation
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
    config_dict.setdefault("ballistic_drag", 0)
    config_dict.setdefault("optimize_boost", 1)
    config_dict.setdefault("optimize_maneuv", 0)
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
            tau_deflect, nav_gain = optimize_maneuv(config_dict)
            print(f"{tau_deflect=}, {nav_gain=}")
            config_dict["tau_deflect"] = tau_deflect
            config_dict["nav_gain"] = nav_gain
        else:
            print("Skipping maneuverability optimization; requires rv_maneuv = 1")

    # Prepare config for multiprocessing
    random_seed = config_dict["random_seed"]
    N_processes = min(num_processes, config_dict["num_runs"])
    traj_output = config_dict["traj_output"]
    config_dict["traj_output"] = 0
    configs = []
    for i in range(config_dict["num_runs"]):
        configs.append(deepcopy(config_dict))
        # Give each processes a different random seed
        if random_seed >= 0:
            configs[-1]["random_seed"] = random_seed + np.random.randint(0, 10000)
    configs[0]["traj_output"] = traj_output

    # Run simulation across multiple processes
    with Pool(processes=N_processes) as p:
        res = list(
            tqdm(
                p.imap_unordered(_mc_run_wrapper, configs),
                total=config_dict["num_runs"],
                desc="Progress",
            )
        )

    # Restore original params
    config_dict["traj_output"] = traj_output

    # Concatenate results and reset index to ascending
    impact_df = pd.concat(res)
    impact_df = impact_df.reset_index().drop(columns="index")

    # Add miss distance column to DataFrame
    aimpoint = (config_dict["x_aim"], config_dict["y_aim"], config_dict["z_aim"])
    miss_distance = get_miss_distance(impact_df=impact_df, aimpoint=aimpoint)
    impact_df["miss_distance"] = miss_distance

    save_path = Path(plot_path) if plot_path else None
    if plot_impact:
        print("Generating impact plot...")
        create_impact_plot(impact_df, save_path=save_path, aimpoint=aimpoint)

    # Load trajectory data if plotting
    if plot_trajectory:
        trajectory_df = pd.read_csv(
            config_dict["trajectory_path"], skipinitialspace=True
        )
        reentry_guidance_path = Path(config_dict["trajectory_path"]).with_name(
            "reentry_guidance.csv"
        )

        print("Generating trajectory plots...")
        guidance_df = None
        if reentry_guidance_path.exists():
            guidance_df = pd.read_csv(reentry_guidance_path, skipinitialspace=True)

        # Pass guidance_df into create_traj_plots so reentry guidance is plotted
        # using the same phase masks as the other trajectory plots.
        create_traj_plots(
            trajectory_df,
            save_path=save_path,
            aimpoint=aimpoint,
            guidance_df=guidance_df,
        )

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
        "--plot-impact",
        default=True,
        action="store_true",
        help="Create and save impact plot",
    )
    parser.add_argument(
        "--plot-trajectory",
        default=False,
        action="store_true",
        help="Generate and save trajectory plots",
    )
    parser.add_argument(
        "--plot-path",
        type=str,
        default=None,
        help="Path to save plots",
    )
    parser.add_argument(
        "--num-processes",
        type=int,
        default=10,
        help="Number of processes to run concurrently. Default 10",
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
    plot_trajectory = kwargs.pop("plot_trajectory")
    plot_impact = kwargs.pop("plot_impact")

    plot_path = kwargs.pop("plot_path")

    # Set default plot path to current directory if --plot is set but --plot-path is not
    if (plot_trajectory or plot_impact) and plot_path is None:
        plot_path = os.getcwd()

    impact_df, config = run(
        config=config,
        plot_trajectory=plot_trajectory,
        plot_impact=plot_impact,
        plot_path=plot_path,
        return_config=True,
        **kwargs,
    )
    if plot_impact:
        impact_df.to_csv(Path(plot_path) / "impact.csv", index=False)
    print(impact_df)
    from pytrajlib.utils import get_local_impact

    impact_x_local, impact_y_local = get_local_impact(
        impact_df, (config["x_aim"], config["y_aim"], config["z_aim"])
    )
    r = np.corrcoef(impact_x_local, impact_y_local)[0][1]
    print(f"{r=}")


if __name__ == "__main__":
    cli()
