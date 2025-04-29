import sys
import os
from ctypes import *
from traj_plot import *
from impact_plot import *
from sens_plot import *
import pandas as pd

# Specify the input file name (without the extension)
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
sys.path.append('.')
from pylib import *
so_file = "./build/libPyTraj.so"
pytraj = CDLL(so_file)

# generate the grid points, evenly spaced on a log scale from 0.1 to 10
grid_points = np.logspace(-1, 1, num=7)
print('Grid points: ', grid_points)

# Code block to run the Monte Carlo simulation
if __name__ == "__main__":
    # Read the configuration file
    print("Reading configuration file " + config_file + ".toml...")
    run_params = read_config(config_file)
    print("Configuration file read.")

    aimpoint = update_aimpoint(run_params, config_path)
    print(f"Aimpoint: ({aimpoint.x}, {aimpoint.y}, {aimpoint.z})")

    # initialize the sensitivity data structure with pandas
    sensitivity_data = pd.DataFrame(columns=["initial_pos_error", "initial_vel_error", "initial_angle_error", "acc_scale_stability", "gyro_bias_stability", "gyro_noise", "gnss_noise", "cep"])

    config = configparser.ConfigParser()
    config.read(config_path)

    expected_pos_error = run_params.initial_pos_error
    expected_vel_error = run_params.initial_vel_error
    expected_angle_error = run_params.initial_angle_error
    expected_acc_scale_stability = run_params.acc_scale_stability
    expected_gyro_bias_stability = run_params.gyro_bias_stability
    expected_gyro_noise = run_params.gyro_noise
    expected_gnss_noise = run_params.gnss_noise

    # Manually iterate through the stability parameters

    for i in grid_points:
        # initial_pos_error
        run_params.initial_pos_error = c_double(expected_pos_error * i)
        run_params.initial_vel_error = c_double(0.0)
        run_params.initial_angle_error = c_double(0.0)
        run_params.acc_scale_stability = c_double(0.0)
        run_params.gyro_bias_stability = c_double(0.0)
        run_params.gyro_noise = c_double(0.0)
        run_params.gnss_noise = c_double(0.0)

        impact_data_pointer = pytraj.mc_run(run_params)

        # read the impact data
        impact_data = np.loadtxt("./output/" + config_file + "/impact_data.txt", delimiter = ",", skiprows=1)

        # get the cep
        cep = get_cep(impact_data, run_params)

        # add the cep to the sensitivity data
        sensitivity_data.loc[len(sensitivity_data)] = [run_params.initial_pos_error, run_params.initial_vel_error, run_params.initial_angle_error, run_params.acc_scale_stability, run_params.gyro_bias_stability, run_params.gyro_noise, run_params.gnss_noise, cep]

    for i in grid_points:
        # initial_vel_error
        run_params.initial_pos_error = c_double(0.0)
        run_params.initial_vel_error = c_double(expected_vel_error * i)
        run_params.initial_angle_error = c_double(0.0)
        run_params.acc_scale_stability = c_double(0.0)
        run_params.gyro_bias_stability = c_double(0.0)
        run_params.gyro_noise = c_double(0.0)
        run_params.gnss_noise = c_double(0.0)

        impact_data_pointer = pytraj.mc_run(run_params)

        # read the impact data
        impact_data = np.loadtxt("./output/" + config_file + "/impact_data.txt", delimiter = ",", skiprows=1)

        # get the cep
        cep = get_cep(impact_data, run_params)

        # add the cep to the sensitivity data
        sensitivity_data.loc[len(sensitivity_data)] = [run_params.initial_pos_error, run_params.initial_vel_error, run_params.initial_angle_error, run_params.acc_scale_stability, run_params.gyro_bias_stability, run_params.gyro_noise, run_params.gnss_noise, cep]
    
    for i in grid_points:
        # initial_angle_error
        run_params.initial_pos_error = c_double(0.0)
        run_params.initial_vel_error = c_double(0.0)
        run_params.initial_angle_error = c_double(expected_angle_error * i)
        run_params.acc_scale_stability = c_double(0.0)
        run_params.gyro_bias_stability = c_double(0.0)
        run_params.gyro_noise = c_double(0.0)
        run_params.gnss_noise = c_double(0.0)


        impact_data_pointer = pytraj.mc_run(run_params)

        # read the impact data
        impact_data = np.loadtxt("./output/" + config_file + "/impact_data.txt", delimiter = ",", skiprows=1)

        # get the cep
        cep = get_cep(impact_data, run_params)

        # add the cep to the sensitivity data
        sensitivity_data.loc[len(sensitivity_data)] = [run_params.initial_pos_error, run_params.initial_vel_error, run_params.initial_angle_error, run_params.acc_scale_stability, run_params.gyro_bias_stability, run_params.gyro_noise, run_params.gnss_noise, cep]    
    
    for i in grid_points:
        # acc_scale_stability
        run_params.initial_pos_error = c_double(0.0)
        run_params.initial_vel_error = c_double(0.0)
        run_params.initial_angle_error = c_double(0.0)
        run_params.acc_scale_stability = c_double(expected_acc_scale_stability * i)
        run_params.gyro_bias_stability = c_double(0.0)
        run_params.gyro_noise = c_double(0.0)
        run_params.gnss_noise = c_double(0.0)

        impact_data_pointer = pytraj.mc_run(run_params)

        # read the impact data
        impact_data = np.loadtxt("./output/" + config_file + "/impact_data.txt", delimiter = ",", skiprows=1)

        # get the cep
        cep = get_cep(impact_data, run_params)

        # add the cep to the sensitivity data
        sensitivity_data.loc[len(sensitivity_data)] = [run_params.initial_pos_error, run_params.initial_vel_error, run_params.initial_angle_error, run_params.acc_scale_stability, run_params.gyro_bias_stability, run_params.gyro_noise, run_params.gnss_noise, cep]
    
    for i in grid_points:
        # gyro_bias_stability
        run_params.initial_pos_error = c_double(0.0)
        run_params.initial_vel_error = c_double(0.0)
        run_params.initial_angle_error = c_double(0.0)
        run_params.acc_scale_stability = c_double(0.0)
        run_params.gyro_bias_stability = c_double(expected_gyro_bias_stability * i)
        run_params.gyro_noise = c_double(0.0)
        run_params.gnss_noise = c_double(0.0)

        impact_data_pointer = pytraj.mc_run(run_params)

        # read the impact data
        impact_data = np.loadtxt("./output/" + config_file + "/impact_data.txt", delimiter = ",", skiprows=1)

        # get the cep
        cep = get_cep(impact_data, run_params)

        # add the cep to the sensitivity data
        sensitivity_data.loc[len(sensitivity_data)] = [run_params.initial_pos_error, run_params.initial_vel_error, run_params.initial_angle_error, run_params.acc_scale_stability, run_params.gyro_bias_stability, run_params.gyro_noise, run_params.gnss_noise, cep]
    
    for i in grid_points:
        # gyro_noise
        run_params.initial_pos_error = c_double(0.0)
        run_params.initial_vel_error = c_double(0.0)
        run_params.initial_angle_error = c_double(0.0)
        run_params.acc_scale_stability = c_double(0.0)
        run_params.gyro_bias_stability = c_double(0.0)
        run_params.gyro_noise = c_double(expected_gyro_noise * i)
        run_params.gnss_noise = c_double(0.0)

        impact_data_pointer = pytraj.mc_run(run_params)

        # read the impact data
        impact_data = np.loadtxt("./output/" + config_file + "/impact_data.txt", delimiter = ",", skiprows=1)

        # get the cep
        cep = get_cep(impact_data, run_params)

        # add the cep to the sensitivity data
        sensitivity_data.loc[len(sensitivity_data)] = [run_params.initial_pos_error, run_params.initial_vel_error, run_params.initial_angle_error, run_params.acc_scale_stability, run_params.gyro_bias_stability, run_params.gyro_noise, run_params.gnss_noise, cep]

    if run_params.gnss_nav:
        for i in grid_points:
            # gnss_noise
            run_params.initial_pos_error = c_double(0.0)
            run_params.initial_vel_error = c_double(0.0)
            run_params.initial_angle_error = c_double(0.0)
            run_params.acc_scale_stability = c_double(0.0)
            run_params.gyro_bias_stability = c_double(0.0)
            run_params.gyro_noise = c_double(0.0)
            run_params.gnss_noise = c_double(expected_gnss_noise * i)

            impact_data_pointer = pytraj.mc_run(run_params)

            # read the impact data
            impact_data = np.loadtxt("./output/" + config_file + "/impact_data.txt", delimiter = ",", skiprows=1)

            # get the cep
            cep = get_cep(impact_data, run_params)

            # add the cep to the sensitivity data
            sensitivity_data.loc[len(sensitivity_data)] = [run_params.initial_pos_error, run_params.initial_vel_error, run_params.initial_angle_error, run_params.acc_scale_stability, run_params.gyro_bias_stability, run_params.gyro_noise, run_params.gnss_noise, cep]
    
    # Combined
    for i in grid_points:
        # gnss_noise
        run_params.initial_pos_error = c_double(expected_pos_error * i)
        run_params.initial_vel_error = c_double(expected_vel_error * i)
        run_params.initial_angle_error = c_double(expected_angle_error * i)
        run_params.acc_scale_stability = c_double(expected_acc_scale_stability * i)
        run_params.gyro_bias_stability = c_double(expected_gyro_bias_stability * i)
        run_params.gyro_noise = c_double(expected_gyro_noise * i)
        run_params.gnss_noise = c_double(expected_gnss_noise * i)

        impact_data_pointer = pytraj.mc_run(run_params)

        # read the impact data
        impact_data = np.loadtxt("./output/" + config_file + "/impact_data.txt", delimiter = ",", skiprows=1)

        # get the cep
        cep = get_cep(impact_data, run_params)

        # add the cep to the sensitivity data
        sensitivity_data.loc[len(sensitivity_data)] = [run_params.initial_pos_error, run_params.initial_vel_error, run_params.initial_angle_error, run_params.acc_scale_stability, run_params.gyro_bias_stability, run_params.gyro_noise, run_params.gnss_noise, cep]
    
    # save the sensitivity data to a csv file
    sensitivity_data.to_csv(f"./output/{config_file}/sensitivity_data.csv", index=False)

    # print the sensitivity data
    print(sensitivity_data)

    # Plot the stability data
    sens_plot(grid_points, sensitivity_data, run_params)