import argparse
import json
import re

import numpy as np
from scipy.optimize import minimize_scalar

from pytrajlib.main import run
from pytrajlib.runtime import (
    DEFAULT_VEHICLE,
    EARTH_RADIUS_M,
    VEHICLE_CONFIGS,
    _flatten_config_sections,
    get_vehicle_config_path,
)

# Error-section keys that are settings rather than error magnitudes
NON_ERROR_KEYS = ("gnss_freq",)

# Search bracket for the thrust angle
THETA_LONG_BOUNDS = (0.05, 1.05)  # rad
THETA_LONG_XATOL = 1e-3  # rad
MAX_RANGE_MAXITER = 40

# An impact just short of the launch point wraps to nearly a full circumference.
# Anything past half a circumference is read as landing behind the launch point.
HALF_CIRCUMFERENCE_M = np.pi * EARTH_RADIUS_M


def _error_free_config(raw_config):
    """Flatten a sectioned config and zero every error source in it."""
    config = _flatten_config_sections(raw_config)
    config.update(
        num_runs=1,
        atm_model=3,
        grav_error=0,
        optimize_boost=0,
        optimize_reentry=0,
    )
    for key in raw_config["error"]:
        if key not in NON_ERROR_KEYS:
            config[key] = 0.0
    return config


def _impact_geometry(impact):
    """Downrange, crossrange (both in meters) of an impact row."""
    lon = np.arctan2(impact["y"], impact["x"]) % (2 * np.pi)
    r_mag = np.sqrt(impact["x"] ** 2 + impact["y"] ** 2 + impact["z"] ** 2)
    return float(EARTH_RADIUS_M * lon), float(
        EARTH_RADIUS_M * np.arcsin(impact["z"] / r_mag)
    )


def maximize_range(config, output_dir):
    """
    Find the thrust angle that flies furthest, error-free.

    Below 100 km the thrust direction is held at theta_long (see get_a_thrust in
    physics/thrust.h); above it, Lambert guidance steers to the aimpoint instead.
    So this is only meaningful for a vehicle whose burnout is below 100 km -- an
    ICBM handed off to Lambert flies to whatever aimpoint it was given, near
    enough regardless of theta_long.

    Params:
        config (dict): flat, error-free run parameters.
        output_dir (str): directory for the search's trajectory artifacts.

    Returns:
        (theta_long, range in meters) of the furthest-flying trajectory.
    """

    def objective(theta_long):
        candidate = {**config, "theta_long": float(theta_long), "traj_output": 0}
        impact = run(
            candidate, output_dir=output_dir, num_processes=1, plot_impact=False
        ).iloc[0]
        range_m, _ = _impact_geometry(impact)
        # Report a backwards impact as negative downrange so the search walks
        # away from it rather than treating the wrap as a near-orbital flight.
        if range_m > HALF_CIRCUMFERENCE_M:
            range_m -= 2 * HALF_CIRCUMFERENCE_M
        print(f"theta_long={float(theta_long):.6f}, range={range_m / 1e3:.3f} km")
        return -range_m

    result = minimize_scalar(
        fun=objective,
        bounds=THETA_LONG_BOUNDS,
        method="bounded",
        options=dict(xatol=THETA_LONG_XATOL, maxiter=MAX_RANGE_MAXITER),
    )
    print(result)

    theta_long, range_m = float(result.x), -float(result.fun)
    print(f"Max range {range_m / 1e3:.3f} km at theta_long={theta_long:.6f} rad")
    return theta_long, range_m


def _substitute(text, key, value):
    """Replace one scalar's value in config JSON text, leaving formatting alone."""
    text, n = re.subn(rf'("{key}":\s*)[^,\n}}]+', rf"\g<1>{float(value)!r}", text)
    if n != 1:
        raise ValueError(f"expected one {key!r} entry in the config, found {n}")
    return text


def set_error_free_aimpoint(config_path, output_dir=None, max_range=False):
    """
    Fly error-free trajectories through the mean EarthGram atmosphere and write
    the impact point back to the config as the new range.

    By default this flies a single trajectory at the config's current theta_long.
    With max_range, it first searches for the thrust angle that flies furthest and
    writes that theta_long back alongside the range it achieves.

    The aimpoint is stored as a range because "range" supersedes x_aim/y_aim/z_aim
    when a config is loaded. Range places the aimpoint on the equator, so any
    crossrange at the error-free impact is reported but not carried over.

    Params:
        config_path (str): Path to the sectioned JSON config to update in place.
        output_dir (str): Directory for the error-free run's artifacts; defaults to
            output/<run_name>/error_free.
        max_range (bool): Maximize range over theta_long instead of flying the
            config's current thrust angle.

    Returns:
        range in meters written to the config.
    """
    with open(config_path) as f:
        raw_config = json.load(f)

    config = _error_free_config(raw_config)

    if output_dir is None:
        output_dir = f"output/{config['run_name']}/error_free"

    updates = {}
    if max_range:
        theta_long, _ = maximize_range(config, f"{output_dir}/max_range")
        config["theta_long"] = theta_long
        updates["theta_long"] = theta_long

    impact = run(config, output_dir=output_dir, num_processes=1).iloc[0]
    range_m, crossrange_m = _impact_geometry(impact)
    print(
        f"Error-free impact: range={range_m / 1e3:.3f} km, "
        f"crossrange={crossrange_m:.1f} m, t={impact['t']:.1f} s"
    )
    updates["range"] = range_m

    # Substitute only the changed values so the rest of the file keeps its formatting
    with open(config_path) as f:
        text = f.read()
    for key, value in updates.items():
        text = _substitute(text, key, value)
    with open(config_path, "w") as f:
        f.write(text)
    written = ", ".join(f"{k}={v:.6g}" for k, v in updates.items())
    print(f"Wrote {written} to {config_path}")

    return range_m


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Set a config's aimpoint (range) from an error-free run."
    )
    source = parser.add_mutually_exclusive_group()
    source.add_argument(
        "--vehicle",
        choices=sorted(VEHICLE_CONFIGS),
        default=None,
        help=f"Bundled vehicle config to update (default: {DEFAULT_VEHICLE})",
    )
    source.add_argument(
        "--config", default=None, help="Path to a JSON config to update"
    )
    parser.add_argument("--output-dir", default=None)
    parser.add_argument(
        "--max-range",
        action="store_true",
        help="Maximize range over the thrust angle and write back the best "
        "theta_long as well as the range. Only meaningful when burnout is "
        "below the 100 km Lambert guidance handover.",
    )
    args = parser.parse_args()

    config_path = args.config or str(
        get_vehicle_config_path(args.vehicle or DEFAULT_VEHICLE)
    )
    set_error_free_aimpoint(config_path, args.output_dir, max_range=args.max_range)
