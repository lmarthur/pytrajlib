import os
import sys
from ctypes import *

import pandas as pd

sys.path.append(".")
import matplotlib.pyplot as plt
import scienceplots
from src.pylib import *


def get_sensitivity_data(config_file):
    # Check for the existence of the input file
    config_path = f"./input/{config_file}.toml"
    if not os.path.isfile(config_path):
        print(f"Error: The input file {config_file}.toml does not exist.")
        sys.exit()

    # Check for the existence of the output directory
    if not os.path.isdir(f"./output/{config_file}"):
        # Create the output directory if it does not exist
        os.makedirs(f"./output/{config_file}")

    # Import the necessary functions from the Python library
    so_file = "./build/libPyTraj.so"
    pytraj = CDLL(so_file)

    reentry_time_steps = np.logspace(-4, 1, num=6)
    main_time_steps = np.logspace(-3, 1, num=5)
    print("Main time steps: ", main_time_steps)
    print("Reentry time steps: ", reentry_time_steps)

    # Read the configuration file
    print("Reading configuration file " + config_file + ".toml...")
    run_params = read_config(config_file)
    print("Configuration file read.")

    # Iterate over reentry time steps
    df = pd.DataFrame()
    for main_time_step in main_time_steps:
        print("Running simulations for main time step: ", main_time_step)
        ceps = []
        run_params.time_step_main = c_double(main_time_step)
        for reentry_time_step in reentry_time_steps:
            run_params.time_step_reentry = c_double(reentry_time_step)

            aimpoint = update_aimpoint(run_params, config_path)
            print(f"Aimpoint: ({aimpoint.x}, {aimpoint.y}, {aimpoint.z})")

            run_params.initial_pos_error = c_double(0.0)
            run_params.initial_vel_error = c_double(0.0)
            run_params.initial_angle_error = c_double(0.0)
            run_params.acc_scale_stability = c_double(0.0)
            run_params.gyro_bias_stability = c_double(0.0)
            run_params.gyro_noise = c_double(0.0)
            run_params.gnss_noise = c_double(0.0)

            impact_data_pointer = pytraj.mc_run(run_params)

            # read the impact data
            impact_data = np.loadtxt(
                "./output/" + config_file + "/impact_data.txt",
                delimiter=",",
                skiprows=1,
            )

            # get the cep
            cep = get_cep(impact_data, run_params)

            ceps.append(cep)
        df = pd.concat(
            [
                df,
                pd.DataFrame(
                    {
                        "reentry_time_step": reentry_time_steps,
                        "cep": ceps,
                        "main_time_step": main_time_step,
                    }
                ),
            ]
        )
        print(df)

    df.to_csv(f"./output/{config_file}/timestep_sensitivity_data.csv", index=False)


def plot_sensitivity_data(config_file):
    data = pd.read_csv(f"./output/{config_file}/timestep_sensitivity_data.csv")
    main_time_steps = data["main_time_step"].unique()
    for i, time_step in enumerate(main_time_steps):
        reentry_time_steps = data[data["main_time_step"] == time_step][
            "reentry_time_step"
        ].values
        ceps = data[data["main_time_step"] == time_step]["cep"].values
        # use a color gradient to plot each line
        plt.plot(
            reentry_time_steps,
            ceps,
            label=f"Main Time Step: {time_step} s",
            color=plt.cm.viridis(i / len(main_time_steps)),
            marker="o",
        )
    plt.semilogx()
    plt.semilogy()
    plt.xlabel("Reentry Time Step (s)")
    plt.ylabel("CEP (m)")
    plt.title("Time Step Sensitivity")
    plt.legend()
    plt.savefig(f"./output/{config_file}/timestep_sensitivity_plot.pdf")
    plt.close()


if __name__ == "__main__":
    for config_file in ["run_0", "run_2", "run_3"]:
        get_sensitivity_data(config_file=config_file)
        plot_sensitivity_data(config_file=config_file)
