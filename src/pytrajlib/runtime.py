import importlib.resources
import os
import tempfile
import tomllib

import numpy as np
import pandas as pd
from tqdm.auto import tqdm

from pytrajlib._traj import ffi

_TEMP_DIR = os.path.join(tempfile.gettempdir(), "pytrajlib")
os.makedirs(_TEMP_DIR, exist_ok=True)

_keep_alive = {}
_LOADING_BAR_DISABLED = object()
EARTH_RADIUS_M = 6371e3


@ffi.def_extern()
def _update_loading_bar(n, total) -> None:
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
    rows = []
    for i in range(config["num_runs"]):
        row_data = dict(
            t=impact_data.impact_times[i],
            x=impact_data.impact_states[i].position.x,
            y=impact_data.impact_states[i].position.y,
            z=impact_data.impact_states[i].position.z,
            burnout_speed=impact_data.burnout_speed[i],
            burnout_altitude=impact_data.burnout_altitude[i],
            burnout_angle=impact_data.burnout_angle[i],
            apogee=impact_data.apogee[i],
            reentry_speed=impact_data.reentry_speed[i],
            reentry_angle=impact_data.reentry_angle[i],
        )
        rows.append(row_data)
    impact_df = pd.DataFrame(rows)
    impact_df["x_aim"] = config["x_aim"]
    impact_df["y_aim"] = config["y_aim"]
    impact_df["z_aim"] = config["z_aim"]
    impact_df["range"] = config["range"]
    return impact_df