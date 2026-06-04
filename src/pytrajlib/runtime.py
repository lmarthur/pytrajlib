import importlib.resources
import os
import tempfile
import tomllib
from copy import deepcopy
from multiprocessing import Pool

import numpy as np
import pandas as pd
from tqdm.auto import tqdm

from pytrajlib._traj import ffi
from pytrajlib._traj import lib as traj_lib

_TEMP_DIR = os.path.join(tempfile.gettempdir(), "pytrajlib")
os.makedirs(_TEMP_DIR, exist_ok=True)

_keep_alive = {}
EARTH_RADIUS_M = 6371e3


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


def _set_aimpoint_from_range(config_dict):
    """Set equatorial aimpoint from downrange arc length in meters."""
    if "range" not in config_dict:
        return
    range_m = float(config_dict["range"])
    aimpoint_lon = (range_m / EARTH_RADIUS_M) % (2 * np.pi)
    config_dict["x_aim"] = EARTH_RADIUS_M * np.cos(aimpoint_lon)
    config_dict["y_aim"] = EARTH_RADIUS_M * np.sin(aimpoint_lon)
    config_dict["z_aim"] = 0.0


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


def impact_data_to_df(impact_data, config):
    """
    Convert the impact data to a Pandas DataFrame.

    Args
        impact_data: impact_data
            The impact data from the Monte Carlo run.
        config: dict
            Config dict

    Returns
        impact_df: pd.DataFrame
            The impact data as a Pandas DataFrame.
    """
    impact_df = pd.DataFrame(
        [
            dict(
                t=impact_data.impact_times[0],
                x=impact_data.impact_states[0].position.x,
                y=impact_data.impact_states[0].position.y,
                z=impact_data.impact_states[0].position.z,
                burnout_speed=impact_data.burnout_speed[0],
                burnout_altitude=impact_data.burnout_altitude[0],
                burnout_angle=impact_data.burnout_angle[0],
                apogee=impact_data.apogee[0],
                reentry_speed=impact_data.reentry_speed[0],
                reentry_angle=impact_data.reentry_angle[0],
            )
        ]
    )
    impact_df["x_aim"] = config["x_aim"]
    impact_df["y_aim"] = config["y_aim"]
    impact_df["z_aim"] = config["z_aim"]
    impact_df["range"] = config["range"]
    return impact_df


def _mc_run_wrapper(config_dict):
    """Wrapper function for multiprocessing.

    This function is pickled and executed in worker processes.
    It imports the CFFI library, runs mc_run, and converts the result to a
    picklable format to avoid CFFI serialization issues.
    """
    rp = create_runparams_struct(config_dict)
    impact_data = traj_lib.mc_run(rp[0])
    df = impact_data_to_df(impact_data, config_dict)
    return df


def run(config_dict, num_processes, show_progress_bar=True):
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

    # Run simulation across multiple processes unless the number of processes is 1.
    # Nested multiprocessing is not permitted, so if a higher level runner wants
    # to do their own multiprocessing, they can set the number of proccesses to 1.
    if N_processes == 1:
        res = []
        for i in tqdm(range(config_dict["num_runs"]), disable=not show_progress_bar):
            res.append(_mc_run_wrapper(configs[i]))
    else:
        with Pool(processes=N_processes) as p:
            res = list(
                tqdm(
                    p.imap_unordered(_mc_run_wrapper, configs),
                    total=config_dict["num_runs"],
                    desc="Progress",
                    disable=not show_progress_bar,
                )
            )

    # Restore original params
    config_dict["traj_output"] = traj_output

    # Concatenate results and reset index to ascending
    impact_df = pd.concat(res)
    impact_df = impact_df.reset_index().drop(columns="index")
    return impact_df
