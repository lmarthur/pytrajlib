import argparse
import configparser
import importlib.resources
import inspect
import os
from datetime import datetime

import numpy as np
from tqdm.auto import tqdm

import pytrajlib.plot as plot
from pytrajlib.utils import (
    EARTH_RADIUS,
    check_config_exists,
    get_run_params,
    impact_data_to_df,
    sphere2cart,
    to_c_type,
)

from ._traj import ffi
from ._traj import lib as traj

_keep_alive = {}


@ffi.def_extern()
def update_loading_bar(update_size, total):
    """
    Create or update the loading bar with the given update size and total.
    This is called from the C code.
    """
    if _keep_alive.get("loading_bar") is None:
        _keep_alive["loading_bar"] = tqdm(total=total, desc="Progress")
        _keep_alive["loading_bar"].update(n=update_size)
    else:
        _keep_alive["loading_bar"].update(n=update_size)


def get_run_params_struct(config):
    """
    Set the the run_params struct from the config.

    INPUTS:
    ----------
        config: dict
            The configuration dictionary.
    OUTPUTS:
    ----------
        run_params: runparams
            The run parameters.
    """
    run_params_struct = ffi.new("struct runparams *")
    for key, value in config.items():
        # Ignore config entries that do not exist in runparams
        try:
            p = to_c_type(value)
            run_params_struct.__setattr__(key, p)
            _keep_alive[key] = p
        except AttributeError:
            pass
    return run_params_struct


def create_output_dirs(run_params):
    """
    Create the output directories specified by the run params so
    the C code can write files to them.

    Params:
        run_params (dict): Dictionary containing the run parameters.

    Returns:
        None
    """
    path_params = ["output_dir", "impact_data_path", "trajectory_path"]
    for path_param in path_params:
        dir_path = os.path.dirname(run_params[path_param])
        if not dir_path:
            dir_path = os.getcwd()
        os.makedirs(dir_path, exist_ok=True)


def write_config_toml(run_params, file_path):
    """
    Write the configuration dictionary to a toml file.

    INPUTS:
    -------
        run_params (dict): Dictionary containing the configuration parameters.
        file_path (str): Path to the output toml file.
    """
    # Copy the structure of the default config, but write the values from the
    # user-provided config_dict
    default_config = str(
        importlib.resources.files("pytrajlib.config").joinpath("default.toml")
    )
    default_config_parser = configparser.ConfigParser()
    default_config_parser.read(default_config)
    new_config_dict = {}
    for section in default_config_parser.sections():
        new_config_dict[section] = {}
        for key, _ in default_config_parser.items(section):
            new_config_dict[section][key] = run_params.get(key)

    new_config_parser = configparser.ConfigParser()
    new_config_parser.read_dict(new_config_dict)
    new_config_parser.write(open(file_path, "w"))


def booster_type_parser(val):
    booster_name_map = {
        "MMIII": 0,
        "SCUD": 1,
        "SCUD-ER": 2,
        "GBSD": 3,
        "D5": 4,
        "MOCK": 5,
    }
    if isinstance(val, int):
        return val
    try:
        return int(val)
    except ValueError:
        name = str(val).strip().upper()
        print(f"Parsing booster type: {name}")
        # Accept both "scud-er" and "scuder"
        if name.lower().replace("-", "") == "SCUDER":
            name = "SCUD-ER"
        if name in booster_name_map:
            return booster_name_map[name]
        raise argparse.ArgumentTypeError(
            f"Invalid booster_type '{val}'. Must be one of: {', '.join(booster_name_map.keys())} or 0-5."
        )


def plot_parser(plot_name):
    """
    Return the plotting function for the given plot name.
    See plot.py for the available plots.

    Plot names are case-insensitive and can use underscores or dashes.

    INPUTS:
    -------
        plot_name: str
            The plot type to parse.

    OUTPUTS:
    -------
        function: The plotting function corresponding to the plot name.
    """
    plot_name = plot_name.lower().replace("-", "_")
    return getattr(plot, plot_name, None)


def run(config=None, return_config=True, **kwargs):
    """
    Run the Monte Carlo code with the given parameters. If neither are provided,
    the default configuration is used. If both are provided, config_dict will be used.

    INPUTS:
    -------
        config: optional, Dictionary containing the run parameters from the
            config file or command line.
        return_config: optional, If True, the function will return the
            configuration dictionary used for the run. E.g. if kwargs are passed
            in, the returned config will be updated with those values.
        kwargs: optional, Override the values in the config file, pass in arguments
            like launch_lat_lon=[0,0], aim_lat_lon=[10, 10].,

    OUTPUTS:
    -------
        tuple of (impact_df, run_params) if return_config is True, otherwise just impact_df.
            impact_df (pd.DataFrame): Pandas DataFrame containing the impact data
            from the Monte Carlo run.  Each row is a run, and each column is a field
            from the State Structure.
    """
    # Convert lat lon to cartesian and set cart coordinates in kwargs to override the config
    if "launch_lat_lon" in kwargs:
        cart_launch = sphere2cart(
            EARTH_RADIUS, *np.deg2rad(kwargs["launch_lat_lon"][::-1])
        )
        kwargs["x_launch"], kwargs["y_launch"], kwargs["z_launch"] = cart_launch
    if "aim_lat_lon" in kwargs:
        cart_aim = sphere2cart(EARTH_RADIUS, *np.deg2rad(kwargs["aim_lat_lon"][::-1]))
        kwargs["x_aim"], kwargs["y_aim"], kwargs["z_aim"] = cart_aim

    if isinstance(config, str | None):
        run_params = get_run_params(config)
    else:
        run_params = config
    handle_overrides(run_params, kwargs)

    if "booster_type" in run_params:
        # Convert booster_type to int if it is a string
        run_params["booster_type"] = booster_type_parser(run_params["booster_type"])

    create_output_dirs(run_params)
    run_params_struct = get_run_params_struct(run_params)

    impact_data = traj.mc_run(run_params_struct[0])
    _keep_alive["loading_bar"].close()
    _keep_alive.clear()
    impact_df = impact_data_to_df(impact_data, int(run_params["num_runs"]))

    # Copy the config toml to the output directory if output dir is specified
    if run_params["output_dir"]:
        print(f"output directory: {run_params['output_dir']}")
        toml_path = os.path.join(
            run_params["output_dir"], f"{run_params['run_name']}.toml"
        )
        write_config_toml(run_params, toml_path)

    # Save plots to the output directory
    if run_params.get("plot"):
        if not isinstance(run_params["plot"], list):
            run_params["plot"] = [run_params["plot"]]
        for plot_func in run_params["plot"]:
            plot_func(
                run_params=run_params,
                data=impact_df,
                output_dir=run_params["output_dir"],
            )

    return impact_df, run_params if return_config else impact_df


def get_all_plot_function_names():
    """
    Get all public plot function names from the pytrajlib.plot module.

    OUTPUTS:
    -------
        list: A list of plot function names.
    """
    plot_functions = inspect.getmembers(plot, inspect.isfunction)
    public_plot_functions = [
        name.replace("_", "-")
        for name, func in plot_functions
        if not name.startswith("_") and func.__module__ == plot.__name__
    ]

    return public_plot_functions


def add_arguments_to_parser(parser):
    """
    Add command line arguments to the parser based on the run parameters.

    INPUTS:
    -------
        parser: argparse.ArgumentParser
            The argument parser to add the arguments to.
    """
    default_config_path = str(
        importlib.resources.files("pytrajlib.config").joinpath("default.toml")
    )
    parser.add_argument(
        "-c",
        "--config",
        type=str,
        required=False,
        default=default_config_path,
        help=f"Path to the configuration file (default: {default_config_path})",
    )

    parser.add_argument(
        "-l",
        "--launch",
        type=float,
        default=[0.0, 0.0],
        nargs=2,
        metavar=("LATITUDE", "LONGITUDE"),
        dest="launch_lat_lon",
        help="Launch latitude and longitude in decimal degrees (default: 0.0 0.0)",
    )
    parser.add_argument(
        "-a",
        "--aim",
        type=float,
        default=[0.0, 120.64],
        nargs=2,
        metavar=("LATITUDE", "LONGITUDE"),
        dest="aim_lat_lon",
        help="Aimpoint latitude and longitude in decimal degrees (default: 0.0 0.0)",
    )

    parser.add_argument(
        "-p",
        "--plot",
        type=plot_parser,
        default=None,
        nargs="*",
        help=f"Plot type to use. One (or more) of the following: {get_all_plot_function_names()} (default: None, no plot will be generated). Plots are saved to the output directory.",
    )

    run_params = get_run_params()
    help_text = {
        "run_name": f"Name of the run (default: '{run_params['run_name']}')",
        "run_type": f"0 for simulating a full trajectory, 1 for reentry only. (default: {run_params['run_type']})",
        "num_runs": f"Number of Monte Carlo runs to perform (default: {run_params['num_runs']})",
        "output_dir": "Directory in which to save the configuration details, plots, and data (default: currrent date and time e.g. './20250623_102744/').",
        "impact_data_path": "Path to save the impact data (default: './{date}/impact.txt')",
        "trajectory_path": "Path to save the trajectory data (default: './{date}/trajectory.txt')",
        "atm_profile_path": f"Path to the atmospheric profile file (default: '{run_params['atm_profile_path']}')",
        "time_step_main": f"Time step for the main simulation in seconds (default: {run_params['time_step_main']})",
        "time_step_reentry": f"Time step for the reentry simulation in seconds (default: {run_params['time_step_reentry']})",
        "traj_output": f"Whether to output trajectory data (0 for no output, 1 for all runs, 2 for the first run) (default: {run_params['traj_output']})",
        "impact_output": f"Whether to output impact data (0 for no, 1 for yes) (default: {run_params['impact_output']})",
        "grav_error": f"Whether to include Gaussian-distributed uncertainty in the geoid height (default {run_params['grav_error']})",
        "atm_model": f"Atmospheric model to use. If 0 and atm_error is 0, an exponential atmospheric model is used. If 0 and atm_error is 1, an exponential model with Gaussian wind injection from the EarthGram model is used. If both atm_model and amt_error are 1, then the EarthGram 2016 model for is used below altitude of 100km. (default: {run_params['atm_model']})",
        "atm_error": f"See atm_model. Default: {run_params['atm_error']})",
        "gnss_nav": f"Whether to use GNSS navigation (default: {run_params['gnss_nav']})",
        "ins_nav": f"If off, indicates perfect inertial navigation system state measurements (default: {run_params['ins_nav']})",
        "rv_maneuv": f"If set to 0, there is no additional maneuverability. If set to 1, enables RV proportional navigation w/ realistic maneuverability, if set to 2, idealized maneuverability (default: {run_params['rv_maneuv']})",
        "reentry_vel": f"Reentry velocity (m/s) for reentry only simulation (run_type = 1) (default: {run_params['reentry_vel']})",
        "deflection_time": f"Deflection time (s) for control surfaces (default: {run_params['deflection_time']})",
        "booster_type": f"0 for MMIII, 1 for SCUD, 2 for SCUD-ER, 3 for GBSD, 4 for D5, 5 for Mock (default: {run_params['booster_type']}). You can also specify by name: MMIII, SCUD, SCUD-ER, GBSD, D5, MOCK.",
        "rv_type": f"0 for ballistic reentry vehicle, 1 for maneuverable reentry vehicle (default: {run_params['rv_type']})",
        "initial_vel_error": f"Initial veleocity error in m/s (default: {run_params['initial_vel_error']})",
        "acc_scale_stability": f"Accelerometer scale stability in ppm (default : {run_params['acc_scale_stability']})",
        "gyro_bias_stability": f"Gyroscope bias stability in radians/s (default: {run_params['gyro_bias_stability']})",
        "gyro_noise": f"Gyroscope noise in radians/s/sqrt(s) (default: {run_params['gyro_noise']})",
        "gnss_noise": f"GNSS noise in m (default: {run_params['gnss_noise']})",
        "cl_pert": f"Coefficient of lift perturbation {run_params['cl_pert']})",
        "step_acc_mag": f"Step acceleration perturbation magnitude for reentry simulation run_type = 1 (default: {run_params['step_acc_mag']})",
        "step_acc_hgt": f"Step acceleration perturbation height (altitude) in meters for reentry simulation run_type = 1 (default: {run_params['step_acc_hgt']})",
        "step_acc_dur": f"Step acceleration perturbation duration in seconds for reentry simulation run_type = 1 (default: {run_params['step_acc_dur']})",
    }

    short_names = {
        "run_name": "r",
        "run_type": "t",
        "num_runs": "n",
        "output_dir": "o",
        "impact_output": "i",
        "traj_output": "j",
        "booster_type": "b",
        "rv_type": "v",
    }

    # Patch the type for booster_type to allow name or number
    for key, value in run_params.items():
        flags = [f"--{key.replace('_', '-')}"]
        if key in short_names:
            flags.insert(0, f"-{short_names[key]}")
        arg_type = type(value)
        if key == "booster_type":
            arg_type = booster_type_parser
        parser.add_argument(
            *flags,
            default=value,
            type=arg_type,
            required=False,
            help=help_text.get(key),
        )

    return parser


def handle_overrides(config_dict, override_dict):
    """
    Handle overrides for the configuration dictionary.

    INPUTS:
    -------
        config_dict: dict
            The configuration dictionary.
        override_dict: dict
            The dictionary containing overrides.

    OUTPUTS:
    -------
        config_dict: dict
            The updated configuration dictionary with overrides applied.
    """
    # Update the config_dict if the user manually overrides a value
    for key, value in override_dict.items():
        if key not in config_dict or value != config_dict[key]:
            config_dict[key] = value
    atm_profile_path = str(
        importlib.resources.files("pytrajlib.config").joinpath("atmprofiles.txt")
    )
    config_dict["atm_profile_path"] = atm_profile_path

    # Set output directory to current date/time if not provided by user and the user
    # wants to output impact data, trajectory data, or plots
    if (
        config_dict.get("impact_output") == 1
        or config_dict.get("traj_output") > 0
        or config_dict.get("plot")
    ):
        if not config_dict.get("output_dir"):
            config_dict["output_dir"] = os.path.abspath(
                f"./{datetime.now().strftime('%Y%m%d_%H%M%S')}"
            )
        if not config_dict.get("impact_data_path"):
            config_dict["impact_data_path"] = os.path.join(
                config_dict["output_dir"], "impact.txt"
            )
        if not config_dict.get("trajectory_path"):
            config_dict["trajectory_path"] = os.path.join(
                config_dict["output_dir"], "trajectory.txt"
            )
    return config_dict


def cli():
    """
    Command line interface for running the Monte Carlo code. Users can provide
    a toml configuration file or command line arguments to override the default
    configuration.
    """
    arg_parser = argparse.ArgumentParser()

    add_arguments_to_parser(arg_parser)

    defaults_dict = vars(arg_parser.parse_args([]))
    args_dict = vars(arg_parser.parse_args())

    # Remove `config` param because it is not present in run_params
    config_path = os.path.abspath(args_dict.pop("config"))

    # Ensure the configuration file provided exists
    if not check_config_exists(config_path):
        arg_parser.error(f"The input file {config_path} does not exist.")

    # Read config file if it is not the default
    if os.path.abspath(defaults_dict["config"]) != config_path:
        config_parser = configparser.ConfigParser()
        config_parser.read(config_path)
        config_dict = {
            key: value
            for section in config_parser.sections()
            for key, value in config_parser.items(section)
        }
    else:
        config_dict = defaults_dict
        config_dict.pop("config")

    return run(config=config_dict, return_config=False, **args_dict)
