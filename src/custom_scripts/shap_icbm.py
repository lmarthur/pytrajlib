import sys
import os

from ctypes import *
import pandas as pd

sys.path.append('.')
from src.pylib import *

import scienceplots
import matplotlib.pyplot as plt

import shap



def shap_helper(features_df, run_params, config_file):
    results = []
    
    for idx, row in features_df.iterrows():
        run_params.initial_pos_error = c_double(row['initial_pos_error'])
        run_params.initial_vel_error = c_double(row['initial_vel_error'])
        run_params.initial_angle_error = c_double(row['initial_angle_error'])
        run_params.acc_scale_stability = c_double(row['acc_scale_stability'])
        run_params.gyro_bias_stability = c_double(row['gyro_bias_stability'])
        run_params.gyro_noise = c_double(row['gyro_noise'])
        run_params.gnss_noise = c_double(row['gnss_noise'])
        
        impact_data_pointer = pytraj.mc_run(run_params)
        impact_data = np.loadtxt("./output/" + config_file + "/impact_data.txt", delimiter=",", skiprows=1)
        cep = get_cep(impact_data, run_params)
        results.append(cep)
    
    print(f"{features_df=}")
    print(f"SHAP helper results: {results}")
    return np.array(results)


def make_plot(config_file):
    # Read the configuration file
    print("Reading configuration file " + config_file + ".toml...")
    run_params = read_config(config_file)
    print("Configuration file read.")

    aimpoint = update_aimpoint(run_params, config_path)
    print(f"Aimpoint: ({aimpoint.x}, {aimpoint.y}, {aimpoint.z})")


    X = pd.DataFrame({
        "initial_pos_error": [run_params.initial_pos_error],
        "initial_vel_error": [run_params.initial_vel_error],
        "initial_angle_error": [run_params.initial_angle_error],
        "acc_scale_stability": [run_params.acc_scale_stability],
        "gyro_bias_stability": [run_params.gyro_bias_stability],
        "gyro_noise": [run_params.gyro_noise],
        "gnss_noise": [run_params.gnss_noise],
    })
    print(f"{X=}")
    # Set shap mask to X, but to zeros
    masker = X * 0


    explainer = shap.ExactExplainer(
        model=lambda features_df: shap_helper(features_df=features_df, run_params=run_params, config_file=config_file),
        masker=masker,
        )
    explanation = explainer(X)
    shap_values = explanation.values

    # put shap values into a dataframe
    shap_df = pd.DataFrame(shap_values, columns=X.columns)
    print(f"{shap_df=}")
    shap_df.to_csv(f"./output/{config_file}/shap_values.csv", index=False)

    shap.plots.beeswarm(explanation, show=False)
    plt.tight_layout()
    plt.savefig(f"./output/{config_file}/shap_beeswarm.png", dpi=300)
    plt.close()

    shap.plots.bar(explanation, show=False)
    plt.tight_layout()
    plt.savefig(f"./output/{config_file}/shap_bar.png", dpi=300)
    plt.close()

if __name__ == "__main__":
    config_file = "run_3"
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

    make_plot(config_file=config_file)