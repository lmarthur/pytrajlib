import argparse
from ctypes import CDLL
import os
import re

import importlib.resources

with importlib.resources.path("pytrajlib", "libPyTraj.so") as so_path:
    pytraj = CDLL(str(so_path))


from pytrajlib.pylib import read_config, update_aimpoint


def check_config_exists(config_path):
    """
    Check if the configuration file exists.

    Params:
        config_path (str): Path to the configuration file.

    Returns:
        bool: True if the file exists, False otherwise.
    """
    return os.path.isfile(config_path)


def check_output_exists(config_name):
    """
    Check if the output directory exists. If not, create it.

    Params:
        config_name (str): Name of the configuration file (without extension).
    """
    # Check for the existence of the output directory
    if not os.path.isdir(f"./output/{config_name}"):
        # Create the output directory if it does not exist
        os.makedirs(f"./output/{config_name}")


def run_simulation(config_name, config_path):
    print("Reading configuration file " + config_name + "...")
    print("running simulation")
    atm_profile_path = str(importlib.resources.path("pytrajlib", "atmprofiles.txt"))
    print(f"Atmospheric profile path: {atm_profile_path}")
    run_params = read_config(config_path, config_name, atm_profile_path)

    aimpoint = update_aimpoint(run_params, config_path)
    print(f"Aimpoint: ({aimpoint.x}, {aimpoint.y}, {aimpoint.z})")
    impact_data_pointer = pytraj.mc_run(run_params)


def cli():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-c",
        "--config",
        type=str,
        required=True,
        help="Path to the configuration file.",
    )
    args = parser.parse_args()

    config_path = args.config
    # Extract the name of the file from the path
    config_name = re.sub(r"\.toml$", "", os.path.basename(config_path))
    check_config_exists(config_path)
    check_output_exists(config_name)
    if not check_config_exists(config_path):
        parser.error(f"The input file {config_name}.toml does not exist.")

    # Run the simulation
    run_simulation(config_name, config_path)

    # Copy the input file to the output directory
    os.system(f"cp {config_path} ./output/{config_name}")
