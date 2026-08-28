import importlib.resources
import json
import math
from copy import deepcopy
from multiprocessing import Pool

import numpy as np
import pandas as pd
from tqdm.auto import tqdm

from pytrajlib._traj import ffi
from pytrajlib._traj import lib as traj_lib

_keep_alive = {}
EARTH_RADIUS_M = 6371e3
MAX_BOOSTER_STAGES = 10


def _flatten_config_sections(raw_config):
    """Flatten known sectioned config formats into a single run-parameter dict."""
    section_names = ("run", "flight", "optimized", "error")
    if any(section in raw_config for section in section_names):
        config_dict = {
            **raw_config.get("run", {}),
            **raw_config.get("flight", {}),
            **raw_config.get("optimized", {}),
            **raw_config.get("error", {}),
        }
        vehicle = raw_config.get("vehicle")
        if isinstance(vehicle, dict):
            config_dict["vehicle"] = vehicle
        return config_dict

    config_dict = raw_config.copy()
    vehicle = config_dict.get("vehicle")
    if isinstance(vehicle, dict):
        config_dict["vehicle"] = vehicle
    return config_dict


def get_default_config():
    """Load and flatten the library's default maneuver/config JSON.

    Returns
        dict: configuration dictionary with any sectioned fields flattened
              into a single run-parameter mapping.
    """
    config_resource_root = importlib.resources.files("pytrajlib.config")
    json_config = config_resource_root.joinpath("maneuv.json")

    with open(json_config) as f:
        raw_config = json.load(f)
    config_dict = _flatten_config_sections(raw_config)

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
        if key == "vehicle":
            continue
        p = to_c_type(value)
        run_params_struct.__setattr__(key, p)
        _keep_alive[key] = p
    return run_params_struct


def _set_fixed_string(field, value: str):
    """Write a Python string into a fixed-size C char buffer (UTF-8)."""
    encoded = value.encode("utf-8")
    ffi.memmove(field, encoded, len(encoded))


def create_vehicle_struct(config_dict):
    """Create and populate a C `vehicle` struct from a config dictionary."""
    vehicle_dict = config_dict.get("vehicle")
    if not isinstance(vehicle_dict, dict):
        raise ValueError(
            "vehicle specification missing; provide a nested vehicle object in the config"
        )

    booster_dict = vehicle_dict.get("booster", {})
    rv_dict = vehicle_dict.get("rv", {})
    aero = rv_dict.get("aerodynamics", {})
    stages = booster_dict.get("stages", [])

    if len(stages) > MAX_BOOSTER_STAGES:
        raise ValueError(
            f"vehicle defines {len(stages)} stages, but only {MAX_BOOSTER_STAGES} are supported"
        )

    aero_table_keys = (
        "alpha_deg_table",
        "c_d_table",
        "c_l_table",
        "c_m_table",
        "c_m_q_table",
    )
    aero_tables = {k: [float(v) for v in aero.get(k, [])] for k in aero_table_keys}
    table_lengths = {len(t) for t in aero_tables.values()} - {0}
    if len(table_lengths) > 1:
        raise ValueError("aerodynamic coefficient tables must all have the same length")
    aero_table_size = next(iter(table_lengths), 0)

    vehicle_struct = ffi.new("vehicle *")
    b = vehicle_struct.booster
    rv = vehicle_struct.rv

    _set_fixed_string(b.name, booster_dict.get("name", ""))
    b.num_stages = len(stages)
    b.area = float(booster_dict.get("area", 0.0))
    b.bus_mass = float(booster_dict.get("bus_mass", 0.0))
    b.c_d_0 = float(booster_dict.get("c_d_0", 0.0))
    b.total_burn_time = sum(float(s.get("burn_time", 0.0)) for s in stages)
    b.total_mass = b.bus_mass + sum(float(s.get("wet_mass", 0.0)) for s in stages)

    for i, stage in enumerate(stages):
        wet = float(stage.get("wet_mass", 0.0))
        fuel = float(stage.get("fuel_mass", 0.0))
        burn = float(stage.get("burn_time", 0.0))
        b.wet_mass[i] = wet
        b.fuel_mass[i] = fuel
        b.dry_mass[i] = (
            float(stage["dry_mass"])
            if stage.get("dry_mass") is not None
            else wet - fuel
        )
        b.isp0[i] = float(stage.get("isp0", 0.0))
        b.burn_time[i] = burn
        b.fuel_burn_rate[i] = (
            float(stage["fuel_burn_rate"])
            if stage.get("fuel_burn_rate") is not None
            else (0.0 if burn == 0 else fuel / burn)
        )

    rv_radius = float(rv_dict.get("rv_radius", 0.0))
    _set_fixed_string(rv.name, str(rv_dict.get("name", "")))
    rv.maneuverability_flag = int(rv_dict.get("maneuverability_flag", 0))
    rv.rv_mass = float(rv_dict.get("rv_mass", 0.0))
    rv.rv_length = float(rv_dict.get("rv_length", 0.0))
    rv.rv_radius = rv_radius
    rv.half_angle = float(rv_dict.get("half_angle", 0.0))
    rv.rv_area = float(rv_dict.get("rv_area", math.pi * rv_radius**2))

    for key in ("c_d_0", "c_d_alpha", "c_m_alpha", "c_m_q", "c_m_delta", "c_l_alpha"):
        setattr(rv, key, float(aero.get(key, rv_dict.get(key, 0.0))))

    rv.flap_area = float(rv_dict.get("flap_area", 0.0))
    rv.x_flap = float(rv_dict.get("x_flap", 0.0))
    rv.x_com = float(rv_dict.get("x_com", 0.0))
    rv.Iyy = float(rv_dict.get("Iyy", 0.0))
    rv.aero_table_size = aero_table_size

    for i in range(aero_table_size):
        rv.aero_alpha_deg_table[i] = aero_tables["alpha_deg_table"][i]
        rv.c_d_table[i] = aero_tables["c_d_table"][i]
        rv.c_l_table[i] = aero_tables["c_l_table"][i]
        rv.c_m_table[i] = aero_tables["c_m_table"][i]
        rv.c_m_q_table[i] = aero_tables["c_m_q_table"][i]

    vehicle_struct.total_mass = b.total_mass + rv.rv_mass
    _keep_alive["vehicle_struct"] = vehicle_struct
    return vehicle_struct


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


def _mc_run_wrapper(indexed_config):
    """Wrapper function for multiprocessing.

    This function is pickled and executed in worker processes.
    It imports the CFFI library, runs mc_run, and converts the result to a
    picklable format to avoid CFFI serialization issues.

    Args:
        indexed_config: ``(run_index, config_dict)``. The index is carried
            through untouched so the caller can restore submission order after
            collecting results out of order.

    Returns:
        tuple[int, pd.DataFrame]: the same index, and that run's impact row.
    """
    run_index, config_dict = indexed_config
    rp = create_runparams_struct(config_dict)
    vehicle = create_vehicle_struct(config_dict)
    impact_data = traj_lib.mc_run(rp[0], vehicle[0])
    df = impact_data_to_df(impact_data, config_dict)
    return run_index, df


def per_run_seeds(random_seed, num_runs):
    """Derive one independent RNG seed per Monte Carlo run from a single root seed.

    ``random_seed`` is the seed for the *batch*, not for an individual run. Each
    run needs its own seed, or every run would draw the same atmosphere and the
    batch would collapse to ``num_runs`` copies of one flight. ``SeedSequence``
    does the mixing, so the children are well separated and collision-free.

    Args:
        random_seed: batch seed. Negative means "not reproducible": fresh OS
            entropy is drawn, so each call gives a different batch. Any value
            >= 0 reproduces the identical batch every time, in any process.
        num_runs: number of per-run seeds to derive.

    Returns:
        list[int]: ``num_runs`` distinct non-negative seeds. They are uint32, so
        they fit the C ``long random_seed`` field and stay clear of the negative
        sentinel that means auto-seed.
    """
    entropy = None if int(random_seed) < 0 else int(random_seed)
    state = np.random.SeedSequence(entropy).generate_state(num_runs, dtype=np.uint32)
    return [int(seed) for seed in state]


def run(config_dict, num_processes, show_progress_bar=True):
    """Execute a Monte Carlo impact simulation using the compiled C library.

    This function prepares per-process configurations (including random
    seeds), runs `traj_lib.mc_run` either in a process pool or serially,
    and returns the concatenated Pandas DataFrame of impact results.

    Args:
        config_dict: mutable configuration mapping used to parameterize runs.
        num_processes: number of worker processes to use (1 = serial).
        show_progress_bar: if True, display a tqdm progress bar.

    Returns:
        pd.DataFrame: concatenated impact results for all runs.
    """
    # Prepare config for multiprocessing
    N_processes = min(num_processes, config_dict["num_runs"])
    traj_output = config_dict["traj_output"]
    config_dict["traj_output"] = 0
    # Give every run its own seed, derived from the batch seed so that a given
    # random_seed >= 0 reproduces the whole batch exactly while the runs within
    # it stay independent.
    seeds = per_run_seeds(config_dict["random_seed"], config_dict["num_runs"])
    configs = []
    for seed in seeds:
        configs.append(deepcopy(config_dict))
        configs[-1]["random_seed"] = seed
    configs[0]["traj_output"] = traj_output

    # Run simulation across multiple processes unless the number of processes is 1.
    # Nested multiprocessing is not permitted, so if a higher level runner wants
    # to do their own multiprocessing, they can set the number of processes to 1.
    if N_processes == 1:
        res = []
        for i in tqdm(range(config_dict["num_runs"]), disable=not show_progress_bar):
            res.append(_mc_run_wrapper((i, configs[i])))
    else:
        with Pool(processes=N_processes) as p:
            # imap_unordered keeps the progress bar advancing as runs finish
            # rather than stalling on a slow early one. Each result carries its
            # submission index so the order is restored below.
            res = list(
                tqdm(
                    p.imap_unordered(_mc_run_wrapper, enumerate(configs)),
                    total=config_dict["num_runs"],
                    desc="Progress",
                    disable=not show_progress_bar,
                )
            )

    # Restore original params
    config_dict["traj_output"] = traj_output

    # Restore submission order, so a given random_seed returns the same rows in
    # the same order and row 0 is the run whose trajectory was written to
    # trajectory_path.
    res.sort(key=lambda indexed_df: indexed_df[0])

    # Concatenate results and reset index to ascending
    impact_df = pd.concat([df for _, df in res])
    impact_df = impact_df.reset_index().drop(columns="index")
    return impact_df
