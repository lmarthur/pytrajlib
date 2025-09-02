"""
Sensitivity Analysis Tool for Trajectory Simulations

Usage examples:
    uv run src/custom_scripts/sensitivity_analysis.py                        # Default: run all configurations
    uv run src/custom_scripts/sensitivity_analysis.py -c all                  # Run all configurations (icbm, ins, ins-gnss)
    uv run src/custom_scripts/sensitivity_analysis.py -c icbm                 # ICBM configuration (run_0)
    uv run src/custom_scripts/sensitivity_analysis.py -c ins                  # INS configuration (run_2)
    uv run src/custom_scripts/sensitivity_analysis.py -c ins-gnss             # INS+GNSS configuration (run_3)
    uv run src/custom_scripts/sensitivity_analysis.py -c /path/to/config.toml # Full path to config file
"""

import argparse

import numpy as np
import pandas as pd
import sens_plot

from pytrajlib import simulation, utils


def scale_params(run_params, sensitivity_params, params_to_scale, scale_factor):
    """
    Scale the sensitivity parameters in the run parameters dictionary by the given scale factor.

    Parameters:
    run_params (dict): The original run parameters.
    sensitivity_params (list): List of parameter names to be scaled.
    params_to_scale (list): List of parameter names to scale.
    scale_factor (float): The factor by which to scale the parameters.

    Returns:
    dict: A new dictionary with the scaled parameters.
    """
    scaled_params = run_params.copy()
    for param in sensitivity_params:
        if param in params_to_scale:
            scaled_params[param] *= scale_factor
        else:
            scaled_params[param] = 0.0
    return scaled_params


def get_config_file(args):
    """
    Map command line argument to config file name and load run parameters.

    Parameters:
    args: Parsed command line arguments

    Returns:
    list: List of run_params dictionaries for each configuration to run
    """
    config_map = {"icbm": "run_0", "ins": "run_2", "ins-gnss": "run_3"}

    if args.config == "all":
        # Return all configurations
        configs = []
        for config_name, config_file in config_map.items():
            config_path = f"input/{config_file}.toml"
            print(f"Reading configuration file {config_path}...")
            run_params = utils.get_run_params(config_path)
            print("Configuration file read.")
            configs.append(run_params)
        return configs
    elif args.config in config_map:
        config_file = config_map[args.config]
        config_path = f"input/{config_file}.toml"
    else:
        # User provided a full file path
        config_path = args.config

    print(f"Reading configuration file {config_path}...")
    run_params = utils.get_run_params(config_path)
    print("Configuration file read.")

    return [run_params]


def run_sensitivity_analysis(run_params):
    """
    Run sensitivity analysis for the given configuration.

    Parameters:
    run_params (dict): Run parameters dictionary
    """
    grid_points = np.logspace(-1, 1, num=7)
    print("Grid points: ", grid_points)

    sensitivity_params = [
        "initial_pos_error",
        "initial_vel_error",
        "initial_angle_error",
        "acc_scale_stability",
        "gyro_bias_stability",
        "gyro_noise",
    ]
    if run_params["gnss_nav"]:
        sensitivity_params.append("gnss_noise")
        print("Including GNSS noise in sensitivity analysis.")

    # initialize the sensitivity data structure with pandas
    sensitivity_data = pd.DataFrame(columns=sensitivity_params + ["cep"])

    # Run sensitivity analysis for each parameter individually and all of them together
    for param_name in sensitivity_params + [sensitivity_params]:
        print(f"Testing sensitivity for {param_name}...")
        param_name = param_name if isinstance(param_name, list) else [param_name]
        for scale_factor in grid_points:
            # Reset all parameters to baseline (0.0) except the one being tested
            current_params = scale_params(
                run_params, sensitivity_params, param_name, scale_factor
            )

            impact_data = simulation.run(current_params)

            cep = utils.get_cep(current_params, impact_data)

            sensitivity_data.loc[len(sensitivity_data)] = [
                current_params[p] for p in sensitivity_params
            ] + [cep]

    # save the sensitivity data to a csv file using run_name from params
    config_name = run_params["run_name"]
    sensitivity_data.to_csv(f"./output/{config_name}/sensitivity_data.csv", index=False)

    # print the sensitivity data
    print(sensitivity_data)

    # Plot the stability data
    sens_plot.sens_plot(grid_points, sensitivity_data, run_params)


# Code block to run the Monte Carlo simulation
if __name__ == "__main__":
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description="Run sensitivity analysis on trajectory simulation parameters."
    )
    parser.add_argument(
        "--config",
        "-c",
        default="all",
        help='Configuration to use: "all" (run all configs), "icbm" (run_0), "ins" (run_2), "ins-gnss" (run_3), or full file path (default: all)',
    )

    args = parser.parse_args()
    configs = get_config_file(args)

    for i, run_params in enumerate(configs):
        print(f"\n{'=' * 60}")
        print(
            f"Running sensitivity analysis for {run_params['run_name']} configuration"
        )
        print(f"{'=' * 60}")

        run_sensitivity_analysis(run_params)
