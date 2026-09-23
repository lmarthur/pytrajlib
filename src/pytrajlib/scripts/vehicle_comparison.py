"""Tabulate configuration and flight metrics for each bundled vehicle config as a booktabs LaTeX table.

One column per vehicle: its configuration (stage count, RV separation, navigation
aiding, reentry guidance and aerodynamics) read from the config, its design range, and the
mean apogee and burnout speed and angle, and the CEP, from a Monte Carlo batch
"""

import argparse
from pathlib import Path

import numpy as np
import pandas as pd

from pytrajlib.main import run
from pytrajlib.runtime import VEHICLE_CONFIGS, get_config

# Ordered from simplest to most capable configuration
VEHICLES = ("scud", "scud-er", "d5", "swerve")
# Only the default is modeled at full fidelity; the others are loosely based on
# the SCUD-B, SCUD-ER, and Trident D5 and labeled so they aren't read as those systems
CONFIG_LABELS = {
    "scud": "Illustrative SRBM A",
    "scud-er": "Illustrative SRBM B",
    "d5": "Illustrative SLBM",
    "swerve": "Default MMIII + SWERVE",
}
DEFAULT_NUM_RUNS = 1000


def get_vehicle_metrics(vehicle, num_runs, output_dir, num_processes=None):
    """
    Fly a Monte Carlo batch for one bundled vehicle config.

    Params:
        vehicle (str): name of a bundled vehicle config (see VEHICLE_CONFIGS).
        num_runs (int): Monte Carlo samples used to estimate the CEP.
        output_dir (Path): directory for this vehicle's run artifacts.
        num_processes (int): concurrent simulation processes; None leaves run's default.

    Returns:
        dict with the design range (m); the means over the batch of the apogee
        altitude (m), burnout speed (m/s), and burnout flight-path angle (rad);
        and the median miss distance, i.e. the CEP (m).
    """
    config = get_config(vehicle)
    extra = {} if num_processes is None else {"num_processes": num_processes}
    impact_df = run(
        config,
        num_runs=num_runs,
        # The trajectory log is per-run overhead with nothing downstream reading it
        traj_output=0,
        output_dir=str(output_dir),
        **extra,
    )
    means = impact_df[["apogee", "burnout_speed", "burnout_angle"]].mean()
    metrics = {
        "range": float(config["range"]),
        "apogee": float(means["apogee"]),
        "burnout_speed": float(means["burnout_speed"]),
        "burnout_angle": float(means["burnout_angle"]),
        # CEP is the median miss distance; see get_miss_distance in pytrajlib.utils
        "cep": float(np.median(impact_df["miss_distance"])),
    }
    print(
        f"{vehicle}: range={metrics['range'] / 1e3:.1f} km, "
        f"apogee={metrics['apogee'] / 1e3:.1f} km, "
        f"burnout={metrics['burnout_speed'] / 1e3:.2f} km/s "
        f"at {np.degrees(metrics['burnout_angle']):.1f} deg, "
        f"CEP={metrics['cep']:.1f} m"
    )
    return metrics


def _describe_config(vehicle):
    """Summarize the capability flags of a bundled vehicle config for the table."""
    config = get_config(vehicle)
    rv = config["vehicle"]["rv"]
    if config["gnss_nav"]:
        nav = "INS + GNSS"
    elif config["stellar_nav"]:
        nav = "INS + Stellar"
    else:
        nav = "INS"
    # Mirrors the branch in get_angular_acceleration (physics/aero_moments.h):
    # simplified drag skips pitch/yaw dynamics; main.py defaults ballistic_drag to 0
    simplified = config["rv_maneuv"] != 1 and config.get("ballistic_drag", 0)
    return {
        "Stages": len(config["vehicle"]["booster"]["stages"]),
        # runtime.py treats a missing is_detached as detached
        "RV separates": "Yes" if rv.get("is_detached", True) else "No",
        "Navigation": nav,
        "Reentry guidance": "PN" if config["rv_maneuv"] else "Ballistic",
        "Reentry aero": "Simplified" if simplified else "Pitch--yaw",
    }


def _round_sig(x, sig=2):
    """Format x to `sig` significant figures without scientific notation,
    keeping trailing zeros (1 -> "1.0", 0.8 -> "0.80")."""
    rounded = float(f"{x:.{sig}g}")
    if rounded == 0:
        return f"{0:.{sig - 1}f}"
    decimals = max(0, sig - 1 - int(np.floor(np.log10(abs(rounded)))))
    return f"{rounded:,.{decimals}f}"


def write_range_cep_table(results, output_dir):
    """
    Write a booktabs LaTeX table of each vehicle's configuration and flight metrics.

    Params:
        results (dict): vehicle name -> metrics dict from get_vehicle_metrics.
        output_dir (Path): directory to write the table into.

    Returns:
        Path of the written .tex file.
    """
    rows = [
        {
            "Configuration": CONFIG_LABELS[vehicle],
            **_describe_config(vehicle),
            "Range (km)": f"{m['range'] / 1e3:,.0f}",
            "Apogee (km)": f"{m['apogee'] / 1e3:,.0f}",
            "Burnout speed (km/s)": f"{m['burnout_speed'] / 1e3:.2f}",
            "Burnout angle (deg)": f"{np.degrees(m['burnout_angle']):.1f}",
            "CEP (m)": _round_sig(m["cep"]),
        }
        for vehicle, m in results.items()
    ]
    # One column per vehicle so the table stays narrow
    df = pd.DataFrame(rows).set_index("Configuration").T
    # Break vehicle labels onto two bottom-aligned lines to keep the columns narrow
    df.columns = [
        rf"\shortstack{{{name.replace(' ', r'\\', 1)}}}" if " " in name else name
        for name in df.columns
    ]
    latex = df.to_latex(column_format="l" * (len(df.columns) + 1))

    tex_path = output_dir / "range_vs_cep.tex"
    tex_path.write_text(latex)
    print(latex)
    print(f"Wrote {tex_path}")
    return tex_path


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Tabulate configuration and flight metrics, one column per vehicle, as a LaTeX table."
    )
    parser.add_argument(
        "--vehicles",
        nargs="+",
        choices=sorted(VEHICLE_CONFIGS),
        default=list(VEHICLES),
        help=f"Bundled vehicle configs to fly (default: {' '.join(VEHICLES)})",
    )
    parser.add_argument(
        "--num-runs",
        type=int,
        default=DEFAULT_NUM_RUNS,
        help=f"Monte Carlo runs per vehicle (default: {DEFAULT_NUM_RUNS})",
    )
    parser.add_argument("--output-dir", default="output/range_vs_cep")
    parser.add_argument("--num-processes", type=int, default=None)
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    results = {}
    for vehicle in args.vehicles:
        results[vehicle] = get_vehicle_metrics(
            vehicle,
            num_runs=args.num_runs,
            output_dir=output_dir / vehicle,
            num_processes=args.num_processes,
        )

    write_range_cep_table(results, output_dir)
