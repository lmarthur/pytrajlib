"""
Runs diagnostic plots for both reentry simulations and full simulations by
doing parameter sweeps.

Diagnostics :

All
`uv run src/custom_scripts/diagnostics.py`

Deflection time sweep
`uv run src/custom_scripts/diagnostics.py -dt`

Navigation gain sweep
`uv run src/custom_scripts/diagnostics.py -ng`

Gearing ratio sweep
`uv run src/custom_scripts/diagnostics.py -gr`

Navigation gain vs gearing ratio heatmap
`uv run src/custom_scripts/diagnostics.py -hm`
"""

import argparse
import os

import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
from matplotlib.colors import LogNorm

from pytrajlib import simulation, utils

base_output_dir = "output/diagnostics/"
os.makedirs(base_output_dir, exist_ok=True)

# Create subdirectories for each diagnostic type
deflection_time_dir = os.path.join(base_output_dir, "deflection_time")
navigation_gain_dir = os.path.join(base_output_dir, "navigation_gain")
gearing_ratio_dir = os.path.join(base_output_dir, "gearing_ratio")
nav_gear_heatmap_dir = os.path.join(base_output_dir, "nav_gear_heatmap")

os.makedirs(deflection_time_dir, exist_ok=True)
os.makedirs(navigation_gain_dir, exist_ok=True)
os.makedirs(gearing_ratio_dir, exist_ok=True)
os.makedirs(nav_gear_heatmap_dir, exist_ok=True)


def get_base_run_params():
    """Get base run parameters from the diagnostics config file."""
    run_params = utils.get_run_params("input/diagnostics.toml")
    return run_params


def run_type_sweep(func):
    """
    Decorator that runs the diagnostic function for both full and reentry simulations.

    The decorated function should accept run_params as parameter.
    """

    def wrapper():
        run_types = [0, 1]

        for run_type in run_types:
            run_params = get_base_run_params()
            run_params["run_type"] = run_type

            # For reentry simulations, set aim point to launch point
            if run_type == 1:
                run_params["x_aim"] = 6371e3
                run_params["y_aim"] = 0
                run_params["z_aim"] = 0

            # Call the original function with the modified run_params
            func(run_params)

    return wrapper


@run_type_sweep
def deflection_time(run_params):
    """
    Plot the CEP for multiple deflection times.

    Produces plots for deflection times from 1e-4 to 1e2 s for both
    reentry and full simulations.
    """
    deflection_times = np.logspace(-4, 4, 30)
    ceps = []

    run_type = run_params["run_type"]
    print(
        f"Running deflection time sweep for {'full' if run_type == 0 else 'reentry'} trajectory..."
    )

    for dt in deflection_times:
        run_params["deflection_time"] = dt
        impact_data = simulation.run(run_params)
        cep = utils.get_cep(run_params=run_params, impact_data=impact_data)
        ceps.append(cep)

    run_type_label = "full" if run_type == 0 else "reentry"
    ceps = np.array(ceps)

    plt.figure(figsize=(10, 6))
    sns.regplot(x=deflection_times, y=ceps, scatter_kws={"alpha": 0.6})
    plt.semilogx()
    plt.semilogy()

    plt.xlabel("Deflection times (s)")
    plt.ylabel("CEP (m)")
    plt.title(f"Deflection Time Diagnostic | run_type {run_type_label}")
    plt.grid(True, alpha=0.3)
    plt.semilogx()

    path = f"{deflection_time_dir}/deflection-rt_{run_type_label}.png"
    plt.savefig(path, dpi=150, bbox_inches="tight")
    print(f"Saved to: {path}")
    plt.close()


@run_type_sweep
def navigation_gain_sweep(run_params):
    """
    Plot the CEP for multiple navigation gain values.

    Produces plots for navigation gains from 1.5 to 20 for both
    reentry and full simulations.
    """
    nav_gains = np.linspace(3, 5, 20)
    ceps = []

    run_type = run_params["run_type"]
    print(
        f"Running navigation gain sweep for {'full' if run_type == 0 else 'reentry'} trajectory..."
    )

    for nav_gain in nav_gains:
        run_params["nav_gain"] = nav_gain
        impact_data = simulation.run(run_params)
        cep = utils.get_cep(run_params=run_params, impact_data=impact_data)
        ceps.append(cep)

    run_type_label = "full" if run_type == 0 else "reentry"

    plt.figure(figsize=(10, 6))
    sns.regplot(x=nav_gains, y=ceps, scatter_kws={"alpha": 0.6})
    plt.xlabel("Navigation Gain")
    plt.ylabel("CEP (m)")
    plt.title(f"Navigation Gain Diagnostic | run_type {run_type_label}")
    plt.grid(True, alpha=0.3)

    path = f"{navigation_gain_dir}/navigation_gain-rt_{run_type_label}.png"
    plt.savefig(path, dpi=150, bbox_inches="tight")
    print(f"Saved to: {path}")
    plt.close()


@run_type_sweep
def gearing_ratio_sweep(run_params):
    """
    Plot the CEP for multiple gearing ratio values.

    Produces plots for gearing ratios from 1 to 10 for both
    reentry and full simulations.
    """
    gearing_ratios = np.logspace(-3, 3, 20)
    ceps = []

    run_type = run_params["run_type"]
    print(
        f"Running gearing ratio sweep for {'full' if run_type == 0 else 'reentry'} trajectory..."
    )

    for gearing_ratio in gearing_ratios:
        run_params["gearing_ratio"] = gearing_ratio
        impact_data = simulation.run(run_params)
        cep = utils.get_cep(run_params=run_params, impact_data=impact_data)
        ceps.append(cep)

    run_type_label = "full" if run_type == 0 else "reentry"

    plt.figure(figsize=(10, 6))
    sns.regplot(x=gearing_ratios, y=ceps, scatter_kws={"alpha": 0.6})
    plt.xlabel("Gearing Ratio")
    plt.ylabel("CEP (m)")
    plt.title(f"Gearing Ratio Diagnostic | run_type {run_type_label}")
    plt.grid(True, alpha=0.3)
    plt.semilogx()

    path = f"{gearing_ratio_dir}/gearing_ratio-rt_{run_type_label}.png"
    plt.savefig(path, dpi=150, bbox_inches="tight")
    print(f"Saved to: {path}")
    plt.close()


@run_type_sweep
def nav_gear_heatmap(run_params):
    """
    Create a 2D heatmap with navigation gain vs gearing ratio and CEP as color.

    Produces heatmap plots showing the interaction between navigation gain
    and gearing ratio for both reentry and full simulations.
    """
    # Define parameter ranges
    nav_gains = np.linspace(3, 5, 10)
    gearing_ratios = np.logspace(-3, 3, 10)

    run_type = run_params["run_type"]
    print(
        f"Running navigation gain vs gearing ratio heatmap for {'full' if run_type == 0 else 'reentry'} trajectory..."
    )

    # Create meshgrid for parameter combinations
    nav_grid, gear_grid = np.meshgrid(nav_gains, gearing_ratios)
    cep_grid = np.zeros_like(nav_grid)

    # Calculate CEP for each parameter combination
    total_combinations = len(nav_gains) * len(gearing_ratios)
    combination_count = 0

    for i, nav_gain in enumerate(nav_gains):
        for j, gearing_ratio in enumerate(gearing_ratios):
            combination_count += 1
            print(
                f"  Progress: {combination_count}/{total_combinations} ({100 * combination_count / total_combinations:.1f}%)"
            )

            # Set parameters for this combination
            run_params["nav_gain"] = nav_gain
            run_params["gearing_ratio"] = gearing_ratio

            # Run simulation and calculate CEP
            impact_data = simulation.run(run_params)
            cep = utils.get_cep(run_params=run_params, impact_data=impact_data)
            cep_grid[j, i] = cep
            print(
                f"    nav_gain: {nav_gain}, gearing_ratio: {gearing_ratio}, CEP: {cep} m"
            )

    run_type_label = "full" if run_type == 0 else "reentry"

    plt.figure(figsize=(12, 8))
    pcm = plt.pcolormesh(
        nav_grid, gear_grid, cep_grid, cmap="viridis", shading="auto", norm=LogNorm()
    )
    cbar = plt.colorbar(pcm)
    cbar.set_label("CEP (m)", rotation=270, labelpad=20)
    plt.xlim(nav_gains.min(), nav_gains.max())
    plt.ylim(gearing_ratios.min(), gearing_ratios.max())
    plt.yscale("log")

    plt.xlabel("Navigation Gain")
    plt.ylabel("Gearing Ratio")
    plt.title(
        f"CEP Heatmap: Navigation Gain vs Gearing Ratio | run_type {run_type_label}"
    )

    path = f"{nav_gear_heatmap_dir}/nav_gear_heatmap-rt_{run_type_label}.png"
    plt.savefig(path, dpi=150, bbox_inches="tight")
    print(f"Saved to: {path}")
    plt.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run diagnostics.")
    parser.add_argument(
        "--deflection-time",
        "-dt",
        action="store_true",
        help="Run deflection time diagnostic",
    )
    parser.add_argument(
        "--navigation-gain",
        "-ng",
        action="store_true",
        help="Run navigation gain diagnostic",
    )
    parser.add_argument(
        "--gearing-ratio",
        "-gr",
        action="store_true",
        help="Run gearing ratio diagnostic",
    )
    parser.add_argument(
        "--heatmap",
        "-hm",
        action="store_true",
        help="Run navigation gain vs gearing ratio heatmap diagnostic",
    )
    args = parser.parse_args()

    if args.deflection_time:
        deflection_time()
    elif args.navigation_gain:
        navigation_gain_sweep()
    elif args.gearing_ratio:
        gearing_ratio_sweep()
    elif args.heatmap:
        nav_gear_heatmap()
    else:
        deflection_time()
        navigation_gain_sweep()
        gearing_ratio_sweep()
        nav_gear_heatmap()
