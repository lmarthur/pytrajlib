import argparse
from ctypes import CDLL
import os
import configparser
from datetime import datetime

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


def create_output_dirs(config_dict):
    """
    Create the output directories specified by the configuration dictionary so
    the C code can write files to them.

    Params:
        config_dict (dict): Dictionary containing the run parameters.

    Returns:
        None
    """
    path_params = ["output_path", "impact_data_path", "trajectory_path"]
    for path_param in path_params:
        dir_path = os.path.dirname(config_dict[path_param])
        os.makedirs(dir_path, exist_ok=True)


def get_default_config_dict():
    """
    Get the default configuration dictionary from the default.toml file.

    OUTPUTS:
    --------
        default_config_dict (dict): Dictionary containing the default configuration
            parameters.
    """
    default_config = str(importlib.resources.path("pytrajlib.config", "default.toml"))
    default_config_parser = configparser.ConfigParser()
    default_config_parser.read(default_config)
    default_config_dict = {
        key: value
        for section in default_config_parser.sections()
        for key, value in default_config_parser.items(section)
    }
    # Override the default atm_profile_path because atmprofiles.txt is
    # bundled with the package and would not have a stable fixed path when
    # the package is installed on a variety of systems.
    default_config_dict["atm_profile_path"] = str(
        importlib.resources.path("pytrajlib.config", "atmprofiles.txt")
    )
    return default_config_dict

def write_config_toml(config_dict, file_path):
    """
    Write the configuration dictionary to a toml file.

    INPUTS:
    -------
        config_dict (dict): Dictionary containing the configuration parameters.
        file_path (str): Path to the output toml file.
    """
    # Copy the structure of the default config, but write the values from the 
    # user-provided config_dict
    default_config = str(importlib.resources.path("pytrajlib.config", "default.toml"))
    default_config_parser = configparser.ConfigParser()
    default_config_parser.read(default_config)
    new_config_dict = {}
    for section in default_config_parser.sections():
        new_config_dict[section] = {}
        for key, value in default_config_parser.items(section):
            new_config_dict[section][key] = config_dict.get(key)
    
    new_config_parser = configparser.ConfigParser()
    new_config_parser.read_dict(new_config_dict)
    new_config_parser.write(open(file_path, "w"))


def run(config_path=None, config_dict=None):
    """
    Run the Monte Carlo code with the given parameters. If neither are provided,
    the default configuration is used. If both are provided, config_dict will be used.

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
        if config_path is None:
            config_dict = get_default_config_dict()
        else:
            if not check_config_exists(config_path):
                raise FileNotFoundError(f"The input file {config_path} does not exist.")
            config_parser = configparser.ConfigParser()
            config_parser.read(config_path)
            config_dict = {
                key: value
                for section in config_parser.sections()
                for key, value in config_parser.items(section)
            }
    create_output_dirs(config_dict)
    run_params = set_runparams(config_dict)
    update_aimpoint(run_params)
    impact_df = mc_run(run_params)

    # Copy the config toml to the output directory
    toml = os.path.join(config_dict["output_path"], f"{config_dict['run_name']}.toml")
    write_config_toml(config_dict, toml)
    return impact_df


def cli():
    """
    Command line interface for running the Monte Carlo code. Users can provide
    a toml configuration file or command line arguments to override the default
    configuration.
    """
    arg_parser = argparse.ArgumentParser()
    default_config_path = str(
        importlib.resources.path("pytrajlib.config", "default.toml")
    )
    arg_parser.add_argument(
        "-c",
        "--config",
        type=str,
        required=False,
        default=default_config_path,
        help=f"Path to the configuration file (default: {default_config_path})",
    )

    # Set up the command line arguments and defaults from the config file
    for key, value in get_default_config_dict().items():
        arg_parser.add_argument(
            f"--{key.replace('_', '-')}",
            default=value,
            # Ensure the type is correct based on the C run_param type
            type=run_param_type(key),
            required=False,
            help=f"{key.replace('_', ' ').capitalize()} (default: {value})",
        )
    defaults_dict = vars(arg_parser.parse_args([]))
    args_dict = vars(arg_parser.parse_args())

    # Remove `config` param because it is not present in run_params
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
    some_overrides = False
    for key, value in args_dict.items():
        if key not in config_dict or value != config_dict[key]:
            config_dict[key] = value
            if key != "run_name":
                some_overrides = True
    # If there are manual overrides and the user did not update the run_name, 
    # change the run name to include the datetime to avoid overwriting previous runs
    # with the same name.  
    if some_overrides:
        config_dict["run_name"] = f"default-{datetime.now().strftime('%Y%m%d_%H%M%S')}"

    atm_profile_path = str(
        importlib.resources.path("pytrajlib.config", "atmprofiles.txt")
    )
    config_dict["atm_profile_path"] = atm_profile_path
    return run(config_dict=config_dict)
