import argparse
import importlib.metadata
import importlib.resources
import json
import os
from collections.abc import Mapping
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
    _UNSET,
    _flatten_config_sections,
    _keep_alive,
    _set_aimpoint_from_range,
    get_default_config,
)
from pytrajlib.scripts.atm_plot import save_atm_plots
from pytrajlib.scripts.sensitivity import run_sensitivity
from pytrajlib.utils import get_miss_distance

np.random.seed(0)


CLI_PARAM_HELP = {
    "run_name": "Run identifier used for output folders and artifacts.",
    "num_runs": "Number of simulation runs to execute.",
    "num_runs_optimizer": "Number of Monte Carlo runs used by the boost and reentry optimizers.",
    "num_trials_optimizer": "Number of optimization trials per optimizer run.",
    "time_step_boost": "Time step used during the boost phase, in seconds.",
    "time_step_lambert": "Time step used during Lambert maneuver, in seconds.",
    "time_step_midcourse": "Time step used during the midcourse phase, in seconds.",
    "time_step_reentry": "Time step used during the reentry phase, in seconds.",
    "traj_output": "Write trajectory output logs for the first run with 1, and disable with 0.",
    "range": "Downrange distance in meters; supersedes the aimpoint.",
    "x_aim": "Target aimpoint x-coordinate in meters.",
    "y_aim": "Target aimpoint y-coordinate in meters.",
    "z_aim": "Target aimpoint z-coordinate in meters.",
    "theta_long": "Thrust angle from x axis in x-y plane.",
    "theta_lat": "Thrust angle above x-y plane.",
    "integrator": "Integrator selection; 0 is modified Euler-Maruyama, 1 is SRA3.",
    "random_seed": "Random seed used to initialize stochastic simulation inputs.",
    "grav_error": "Enable the gravitational error model.",
    "ballistic_drag": "Use simplified drag; 1 enables it and 0 disables it.",
    "atm_model": "Atmospheric model selection; 0 is exponential, 1 adds perturbations, 2 is EarthGram, 3 is mean EarthGram.",
    "gnss_nav": "Enable GNSS position updates during exoatmospheric flight.",
    "rv_maneuv": "Reentry vehicle maneuverability mode; 1 uses realistic maneuverability, 2 uses idealized maneuverability.",
    "reentry_vel": "Reentry velocity in meters per second.",
    "perfect_boost": "Set to 1 for a perfect boost phase and 0 for a realistic boost phase.",
    "optimize_boost": "Optimize t_des_final and theta_long when set to 1.",
    "optimize_reentry": "Optimize reentry maneuver parameters (max_deflection_angle, gearing_ratio, nav_gain_0, nav_gain_1, K_q, K_pp, K_delta_p, K_delta_d) when set to 1.",
    "t_des_final": "Desired final time for the boost phase, in seconds.",
    "t_vert_boost": "Vertical boost time, in seconds.",
    "deflection_time": "Actuator deflection time, in seconds.",
    "actuator_force": "Maximum actuator force in kN.",
    "gearing_ratio": "Actuator gearing ratio. Higher gearing ratios correspond to increased max force and decreased max speed.",
    "actuator_resolution": "Actuator resolution in degrees.",
    "max_deflection_angle": "Maximum deflection angle allowed for the reentry vehicle in degrees.",
    "nav_gain_0": "Navigation gain at surface used by the reentry guidance law.",
    "nav_gain_1": "Navigation gain at reentry used by the reentry guidance law.",
    "K_q": "Pitch-rate feedback gain.",
    "K_pp": "Proportional restoring angle of attack gain.",
    "K_delta_p": "Proportional deflection gain.",
    "K_delta_d": "Derivative deflection gain.",
    "initial_x_error": "Initial x-position error.",
    "initial_pos_error": "Initial position error magnitude.",
    "initial_vel_error": "Initial velocity error magnitude.",
    "initial_angle_error": "Initial angle error magnitude.",
    "acc_scale_stability": "Accelerometer scale-factor stability.",
    "gyro_bias_stability": "Gyroscope bias stability.",
    "gyro_noise": "Gyroscope noise level.",
    "gnss_noise": "GNSS measurement noise level.",
    "gnss_freq": "GNSS update frequency in Hz.",
    "roll_gyro_error_factor": "Roll gyroscope error scaling factor.",
    "burn_time_error": "Burn time error magnitude in seconds.",
}


def _get_version() -> str:
    """Get the version from package metadata."""
    try:
        return importlib.metadata.version("pytrajlib")
    except importlib.metadata.PackageNotFoundError:
        return "unknown"


def _load_config_dict(config: str | Mapping = None):
    if config is None:
        config_dict = get_default_config().copy()
    elif isinstance(config, Mapping):
        config_dict = _flatten_config_sections(dict(config))
    else:
        print(f"Loading config from: {config}")
        config_path = Path(config)
        if config_path.suffix.lower() != ".json":
            raise ValueError("Only JSON config files are supported")
        with open(config_path) as f:
            raw_config = json.load(f)

        config_dict = _flatten_config_sections(raw_config)

    return config_dict


def run(
    config: str | Mapping = None,
    plot_trajectory: bool = False,
    plot_impact: bool = False,
    output_dir: str = None,
    num_processes: int = max((os.cpu_count() * 5) // 8, 1),
    sensitivity: int = None,
    return_config=False,
    **kwargs,
):
    """Load config, override with kwargs, and run the C Monte Carlo code.

    Args
        config: path to a JSON config file, a config dictionary, or `None`
            to use the default config
        plot_trajectory: whether to save trajectory plots
        plot_impact: whether to save impact plot
        output_dir: path to save plots and run artifacts
        num_processes: number of concurrent processes on which to run simulation. Default is 5/8 of the number of cores available so if you have 16 cores, the number of concurrent processes will be 10.
        sensitivity: run error-parameter sensitivity sweep instead of a single simulation
        return_config: whether to return the config dict along with the impact DataFrame
        **kwargs: overrides applied on top of the loaded config
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
    config_dict.setdefault("ballistic_drag", 0)
    config_dict.setdefault("optimize_boost", 1)
    config_dict.setdefault("optimize_reentry", 0)
    config_dict.setdefault("random_seed", -1)
    _set_aimpoint_from_range(config_dict)

    if sensitivity is not None:
        config_dict["num_processes"] = int(num_processes)

        run_name = str(config_dict.get("run_name", "sensitivity"))
        output_dir_path = (
            Path(output_dir) if output_dir is not None else Path("output") / run_name
        )
        output_dir_path.mkdir(parents=True, exist_ok=True)
        config_dict["trajectory_path"] = str(output_dir_path / "trajectory.csv")

        sensitivity_results = run_sensitivity(
            base_config=config_dict,
            output_dir=output_dir_path,
            use_zero_baseline=sensitivity == 0,
        )
        print(sensitivity_results)

        try:
            save_atm_plots(output_dir_path)
        except Exception as exc:
            print(f"Warning: failed to generate atm plots: {exc}")

        if return_config:
            return sensitivity_results, config_dict
        return sensitivity_results

    print("Running...")

    run_name = str(config_dict.get("run_name", "run"))
    output_dir_path = (
        Path(output_dir) if output_dir is not None else Path("output") / run_name
    )
    output_dir_path.mkdir(parents=True, exist_ok=True)
    config_dict["trajectory_path"] = str(output_dir_path / "trajectory.csv")

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

    save_path = output_dir_path
    if output_dir_path:
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
    version = _get_version()
    parser = argparse.ArgumentParser(
        description=f"Trajectory Simulation (version {version})",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument(
        "--version",
        action="version",
        version=f"%(prog)s {version}",
    )
    parser.add_argument(
        "--config",
        type=str,
        default=None,
        help="Path to JSON config file for simulation & vehicle specification ",
    )
    parser.add_argument(
        "--plot-impact",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Create and save impact plot (default: enabled)",
    )
    parser.add_argument(
        "--plot-trajectory",
        default=False,
        action="store_true",
        help="Generate and save trajectory plots",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default=None,
        help="Directory to save plots and run artifacts",
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
        help="Number of processes to run concurrently. Default is 5/8 number of cores available, rounded down to a minimum of 1.",
    )

    for param_name, default_value in get_default_config().items():
        if param_name == "vehicle":
            continue
        parser.add_argument(
            f"--{param_name.replace('_', '-')}",
            type=type(default_value),
            default=_UNSET,
            help=f"{CLI_PARAM_HELP.get(param_name, '')} (default: {default_value})",
        )

    args = parser.parse_args()
    kwargs = vars(args)
    config = kwargs.pop("config")
    plot_trajectory = kwargs.pop("plot_trajectory")
    plot_impact = kwargs.pop("plot_impact")
    sensitivity = kwargs.pop("sensitivity")
    num_processes = int(kwargs.pop("num_processes", max((os.cpu_count() * 5) // 8, 1)))

    output_dir = kwargs.pop("output_dir")

    if sensitivity is not None:
        run(
            config=config,
            output_dir=output_dir,
            num_processes=num_processes,
            sensitivity=sensitivity,
            **kwargs,
        )
        return

    if output_dir is None:
        run_name = kwargs.get("run_name")
        if run_name is _UNSET:
            run_name = get_default_config().get("run_name", "run_4")
        output_dir = str(Path("output") / str(run_name))

    impact_df, config = run(
        config=config,
        plot_trajectory=plot_trajectory,
        plot_impact=plot_impact,
        output_dir=output_dir,
        num_processes=num_processes,
        return_config=True,
        **kwargs,
    )
    if plot_impact:
        csv_path = Path(output_dir) / "impact.csv"
        impact_df.to_csv(csv_path, index=False)
        print(f"Saved: {csv_path}")
    print(impact_df)
    from pytrajlib.utils import get_local_impact

    impact_x_local, impact_y_local = get_local_impact(
        impact_df, (config["x_aim"], config["y_aim"], config["z_aim"])
    )
    if config["num_runs"] > 1:
        r = np.corrcoef(impact_x_local, impact_y_local)[0][1]
        print(f"Pearson's {r=}")


if __name__ == "__main__":
    cli()
