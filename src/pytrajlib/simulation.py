import argparse
from ctypes import CDLL
import os
import configparser

import importlib.resources

with importlib.resources.path("pytrajlib", "libPyTraj.so") as so_path:
    pytraj = CDLL(str(so_path))


from pytrajlib.pylib import run_param_type, update_aimpoint, mc_run, set_runparams


def check_config_exists(config_path):
    """
    Check if the configuration file exists.

    Params:
        config_path (str): Path to the configuration file.

    Returns:
        bool: True if the file exists, False otherwise.
    """
    return os.path.isfile(config_path)


def run(config_path=None, config_dict=None):
    """
    Run the Monte Carlo code with the given parameters. One of config_path or
    config_dict must be provided. If both are provided, config_dict will be used.

    INPUTS:
    -------
        config_dict (dict): optional, Dictionary containing the run parameters from the
            config file or command line.

    OUTPUTS:
    -------
        impact_df (pd.DataFrame): Pandas DataFrame containing the impact data
        from the Monte Carlo run.  Each row is a run, and each column is a field
        from the State Structure.
    """
    if config_dict is None:
        if not check_config_exists(config_path):
            raise FileNotFoundError(f"The input file {config_path} does not exist.")
        config_parser = configparser.ConfigParser()
        config_parser.read(config_path)
        config_dict = {
            key: value
            for section in config_parser.sections()
            for key, value in config_parser.items(section)
        }
    run_params = set_runparams(config_dict)
    update_aimpoint(run_params)
    impact_df = mc_run(run_params)
    return impact_df


def cli():
    """
    Command line interface for running the Monte Carlo code. Users can provide
    a toml configuration file or command line arguments to override the default
    configuration.
    """
    arg_parser = argparse.ArgumentParser()
    default_config = str(importlib.resources.path("pytrajlib.config", "default.toml"))
    arg_parser.add_argument(
        "-c",
        "--config",
        type=str,
        required=False,
        default=default_config,
        help="Path to the configuration file. If provided, it will override the default configuration.",
    )
    default_config_parser = configparser.ConfigParser()
    default_config_parser.read(default_config)

    # Set up the command line arguments and defaults from the config file
    for section in default_config_parser.sections():
        for key, value in default_config_parser.items(section):
            arg_parser.add_argument(
                f"--{key.replace('_', '-')}",
                default=value,
                # Ensure the type is correct based on the C run_param type
                type=run_param_type(key),
                required=False,
            )
    defaults_dict = vars(arg_parser.parse_args([]))
    args_dict = vars(arg_parser.parse_args())

    # Remove the name of the config file as a config param because it is not in run_params
    config_path = os.path.abspath(args_dict.pop("config"))
    
    # Ensure the configuration file provided exists
    if not check_config_exists(config_path):
        arg_parser.error(f"The input file {config_path} does not exist.")

    # If the config file is not the default, read from it
    if os.path.abspath(defaults_dict["config"]) != config_path:
        config_parser = configparser.ConfigParser()
        config_parser.read(config_path)
        config_dict = {
            # Convert the value, which is a string, to the class it should be
            # based on the C run_param type. 
            key: run_param_type(key)(value)
            for section in config_parser.sections()
            for key, value in config_parser.items(section)
        }
    else:
        config_dict = defaults_dict
        config_dict.pop("config")

    # If the user manually overrides a value, update the config_dict
    for key, value in args_dict.items():
        if key not in config_dict or value != config_dict[key]:
            config_dict[key] = value

    atm_profile_path = str(
        importlib.resources.path("pytrajlib.config", "atmprofiles.txt")
    )
    config_dict["atm_profile_path"] = atm_profile_path
    return run(config_dict=config_dict)

    # TODO save the config dict to the output directory as a toml file. Consider how to get the hierarchy back
    # # Copy the input file to the output directory
    # os.system(f"cp {config_path} ./output/{config_name}")
