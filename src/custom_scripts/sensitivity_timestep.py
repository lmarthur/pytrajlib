import os
import sys
from ctypes import *
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd
import scienceplots

sys.path.append('.')
from src.pylib import *


MAIN_TIME_STEPS = np.logspace(-1, 1.5, num=25)
REENTRY_TIME_STEPS = np.logspace(-3, 1.5, num=25)


def run_timestep_sweep(
    config_file,
    pytraj,
    run_params,
    time_steps,
    sweep_name,
    *,
    vary_main_time_step,
    fixed_main_time_step,
    fixed_reentry_time_step,
):
    rows = []
    impact_data_path = Path(f"./output/{config_file}/impact_data.txt")
    config_path = Path(f"./input/{config_file}.toml")

    for time_step in time_steps:
        if vary_main_time_step:
            run_params.time_step_main = c_double(time_step)
            run_params.time_step_reentry = c_double(fixed_reentry_time_step)
        else:
            run_params.time_step_main = c_double(fixed_main_time_step)
            run_params.time_step_reentry = c_double(time_step)

        aimpoint = update_aimpoint(run_params, str(config_path))
        print(f"Aimpoint: ({aimpoint.x}, {aimpoint.y}, {aimpoint.z})")

        print(f"Running {sweep_name} sweep at time step: {time_step}")
        pytraj.mc_run(run_params)

        impact_data = np.loadtxt(impact_data_path, delimiter=",", skiprows=1)
        rows.append({
            "sweep": sweep_name,
            "time_step": time_step,
            "cep": get_cep(impact_data, run_params),
        })

    return pd.DataFrame(rows)


def get_sensitivity_data(config_file):
    config_path = Path(f"./input/{config_file}.toml")
    if not config_path.is_file():
        print(f"Error: The input file {config_file}.toml does not exist.")
        sys.exit()

    output_dir = Path(f"./output/{config_file}")
    output_dir.mkdir(parents=True, exist_ok=True)

    pytraj = CDLL("./build/libPyTraj.so")

    print("Reading configuration file " + config_file + ".toml...")
    run_params = read_config(config_file)
    print("Configuration file read.")

    fixed_main_time_step = float(run_params.time_step_main)
    fixed_reentry_time_step = float(run_params.time_step_reentry)

    aimpoint = update_aimpoint(run_params, str(config_path))
    print(f"Aimpoint: ({aimpoint.x}, {aimpoint.y}, {aimpoint.z})")
    print("Main time steps: ", MAIN_TIME_STEPS)
    print("Reentry time steps: ", REENTRY_TIME_STEPS)

    main_df = run_timestep_sweep(
        config_file,
        pytraj,
        run_params,
        MAIN_TIME_STEPS,
        "Main Time Step",
        vary_main_time_step=True,
        fixed_main_time_step=fixed_main_time_step,
        fixed_reentry_time_step=fixed_reentry_time_step,
    )
    reentry_df = run_timestep_sweep(
        config_file,
        pytraj,
        run_params,
        REENTRY_TIME_STEPS,
        "Reentry Time Step",
        vary_main_time_step=False,
        fixed_main_time_step=fixed_main_time_step,
        fixed_reentry_time_step=fixed_reentry_time_step,
    )

    df = pd.concat([main_df, reentry_df], ignore_index=True)
    print(df)
    df.to_csv(output_dir / "timestep_sensitivity_data.csv", index=False)
    plot_sensitivity_data(config_file)


def plot_sensitivity_data(config_file):
    data = pd.read_csv(f"./output/{config_file}/timestep_sensitivity_data.csv")

    figure, axes = plt.subplots(2, 1, figsize=(6, 8))
    sweep_specs = [
        ("Main Time Step", axes[0], "Main Time Step (s)"),
        ("Reentry Time Step", axes[1], "Reentry Time Step (s)"),
    ]

    for sweep_name, axis, x_label in sweep_specs:
        sweep_data = data[data["sweep"] == sweep_name].sort_values("time_step")
        axis.plot(sweep_data["time_step"].values, sweep_data["cep"].values, marker='o')
        axis.set_xscale('log')
        axis.set_yscale('log')
        axis.set_xlabel(x_label)
        axis.set_ylabel("CEP (m)")
        axis.set_title(sweep_name)

    figure.tight_layout()
    figure.savefig(f"./output/{config_file}/timestep_sensitivity_plot.pdf")
    plt.close(figure)


if __name__ == "__main__":
    for config_file in ["run_0", "run_2", "run_3"]:
        get_sensitivity_data(config_file=config_file)
        plot_sensitivity_data(config_file=config_file)
