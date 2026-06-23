import argparse
import importlib.resources
import json
import os
import tomllib
from pathlib import Path

import numpy as np
import pandas as pd

# from tqdm.auto import tqdm
from pytrajlib import runtime

# Import plotting functions
from pytrajlib.optimizers import optimize_boost, optimize_reentry
from pytrajlib.plotting import (
    create_impact_plot,
    create_traj_plots,
)
from pytrajlib.runtime import (
    _TEMP_DIR,
    _UNSET,
    _get_default_config,
    _keep_alive,
    _set_aimpoint_from_range,
)
from pytrajlib.scripts.atm_plot import save_atm_plots
from pytrajlib.scripts.sensitivity import run_sensitivity
from pytrajlib.utils import get_miss_distance

np.random.seed(0)


def _load_config_dict(config: str = None):
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

    return config_dict


def run(
    config: str = None,
    plot_trajectory: bool = False,
    plot_impact: bool = False,
    plot_path: str = None,
    num_processes: int = (os.cpu_count() * 5) // 8,
    sensitivity: int = None,
    return_config=False,
    **kwargs,
):
    """Load config, override with kwargs, and run the C Monte Carlo code.

    Args
        config: path to config file
        plot_trajectory: whether to save trajectory plots
        plot_impact: whether to save impact plot
        plot_path: path to save plots
        num_processes: number of concurrent processes on which to run simulation. Default is 5/8 of the number of cores available so if you have 16 cores, the number of concurrent processes will be 10.
        sensitivity: run error-parameter sensitivity sweep instead of a single simulation
        **kwargs: all other kwargs (see default TOML)
    """
    config_dict = _load_config_dict(config)

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
    config_dict.setdefault("optimize_reentry", 0)
    config_dict.setdefault("random_seed", -1)
    _set_aimpoint_from_range(config_dict)

    if sensitivity is not None:
        config_dict["num_processes"] = int(num_processes)

        run_name = str(config_dict.get("run_name", "sensitivity"))
        output_dir = (
            Path(plot_path) if plot_path is not None else Path("output") / run_name
        )

        sensitivity_results = run_sensitivity(
            base_config=config_dict,
            output_dir=output_dir,
            use_zero_baseline=sensitivity == 0,
        )
        print(sensitivity_results)

        try:
            save_atm_plots(output_dir)
        except Exception as exc:
            print(f"Warning: failed to generate atm plots: {exc}")

        if return_config:
            return sensitivity_results, config_dict
        return sensitivity_results

    print("Running...")

    if config_dict["optimize_boost"]:
        optimized_params = optimize_boost(config_dict)
        print(optimized_params)
        config_dict = {**config_dict, **optimized_params}
    else:
        print("Skipping boost optimization; using configured t_des_final/theta_long")

    if config_dict["optimize_reentry"]:
        if int(config_dict.get("rv_maneuv", 0)) == 1:
            optimized_params = optimize_reentry(config_dict)
            print(optimized_params)
            config_dict = {**config_dict, **optimized_params}
        else:
            print("Skipping maneuverability optimization; requires rv_maneuv = 1")

    impact_df = runtime.run(config_dict, num_processes)

    # Add miss distance column to DataFrame
    aimpoint = (config_dict["x_aim"], config_dict["y_aim"], config_dict["z_aim"])
    miss_distance = get_miss_distance(impact_df=impact_df, aimpoint=aimpoint)
    impact_df["miss_distance"] = miss_distance

    save_path = Path(plot_path) if plot_path else None
    if plot_path:
        save_path.mkdir(parents=True, exist_ok=True)
        with open(save_path / "config.json", "w") as f:
            f.write(json.dumps(config_dict))

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
        "--sensitivity",
        type=int,
        default=None,
        help="Run the error-parameter sensitivity sweep instead of a single simulation. 0 indicates using zero error as baseline. 1 indicates using standard parameter values as baseline.",
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
    sensitivity = kwargs.pop("sensitivity")
    num_processes = int(kwargs.pop("num_processes", 10))

    plot_path = kwargs.pop("plot_path")

    if sensitivity is not None:
        run(
            config=config,
            plot_path=plot_path,
            num_processes=num_processes,
            sensitivity=sensitivity,
            **kwargs,
        )
        return

    # Set default plot path to current directory if --plot is set but --plot-path is not
    if (plot_trajectory or plot_impact) and plot_path is None:
        plot_path = os.getcwd()

    impact_df, config = run(
        config=config,
        plot_trajectory=plot_trajectory,
        plot_impact=plot_impact,
        plot_path=plot_path,
        num_processes=num_processes,
        return_config=True,
        **kwargs,
    )
    if plot_impact:
        csv_path = Path(plot_path) / "impact.csv"
        impact_df.to_csv(csv_path, index=False)
        print(f"Saved: {csv_path}")
    print(impact_df)
    from pytrajlib.utils import get_local_impact

    impact_x_local, impact_y_local = get_local_impact(
        impact_df, (config["x_aim"], config["y_aim"], config["z_aim"])
    )
    r = np.corrcoef(impact_x_local, impact_y_local)[0][1]
    print(f"{r=}")


if __name__ == "__main__":
    cli()
