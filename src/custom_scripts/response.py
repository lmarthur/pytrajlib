# This script runs the reentry-only simulations specified in the reentry.toml configuration file. 

import sys
import os
from ctypes import *
import matplotlib.pyplot as plt
import numpy as np
import scienceplots

plt.style.use(['science'])
plt.style.use(['no-latex'])

sys.path.append('./src')
from traj_plot import *
from impact_plot import *

# Specify the input file name (without the extension)
config_file = "reentry"

# Import the necessary functions from the Python library
sys.path.append('.')
from src.pylib import *
so_file = "./build/libPyTraj.so"
pytraj = CDLL(so_file)

def get_miss(config_file, run_params):
    # Check for the existence of the input file
    config_path = f"./input/{config_file}.toml"
    if not os.path.isfile(config_path):
        print(f"Error: The input file {config_file}.toml does not exist.")
        sys.exit()

    # Check for the existence of the output directory
    if not os.path.isdir(f"./output/{config_file}"):
        # Create the output directory if it does not exist
        os.makedirs(f"./output/{config_file}")

    # Read the configuration file
    print("Reading configuration file " + config_file + ".toml...")

    # print("Configuration file read.")

    aimpoint = update_aimpoint(run_params, config_path)
    # print(f"Aimpoint: ({aimpoint.x}, {aimpoint.y}, {aimpoint.z})")

    impact_data_pointer = pytraj.mc_run(run_params)
    # print("Monte Carlo simulation complete.")

    # Copy the input file to the output directory
    os.system(f"cp {config_path} ./output/{config_file}")
    
    # Plot the trajectory
    if run_params.traj_output:
        print("Plotting trajectory...")
        traj_plot("./output/" + config_file + "/")
        print("Trajectory plotted.")

    # Plot the impact data
    # print("Plotting impact data...")
    # impact_plot("./output/" + config_file + "/", run_params)
    # print("Impact data plotted.")

    # Read the impact data from the file
    impact_data = np.loadtxt("./output/" + config_file + "/impact_data.txt", delimiter = ",", skiprows=1)

    impact_t = impact_data[0]
    impact_x = impact_data[1]
    impact_y = impact_data[2]
    impact_z = impact_data[3]

    # get vector relative to aimpoint
    impact_x = impact_x - run_params.x_aim
    impact_y = impact_y - run_params.y_aim
    impact_z = impact_z - run_params.z_aim

    # convert impact data to local tangent plane coordinates
    aimpoint_lon = np.arctan2(run_params.y_aim, run_params.x_aim)
    aimpoint_lat = np.arctan2(run_params.z_aim, np.sqrt(run_params.x_aim**2 + run_params.y_aim**2))
    impact_x_local = -np.sin(aimpoint_lon)*impact_x + np.cos(aimpoint_lon)*impact_y
    impact_y_local = -np.sin(aimpoint_lat)*np.cos(aimpoint_lon)*impact_x - np.sin(aimpoint_lat)*np.sin(aimpoint_lon)*impact_y + np.cos(aimpoint_lat)*impact_z

    # get the miss distances
    miss_distance = np.sqrt(impact_x_local**2 + impact_y_local**2)

    print('Miss distance: ', miss_distance, "\n")

    return miss_distance
    
    
if __name__ == "__main__":
    # iterate through the parameters of interest by manipulating the input file

    # First, standardized time delay with variable anomaly height
    # anomaly_heights = np.linspace(0, 50000, 50)
    # miss_distances_0 = np.zeros(len(anomaly_heights))
    # for i in range(len(anomaly_heights)):
    #     run_params = read_config(config_file)
    #     run_params.step_acc_mag = c_double(-10*9.81) 
    #     run_params.step_acc_hgt = c_double(anomaly_heights[i])
    #     run_params.step_acc_dur = c_double(-0.1)
    #     run_params.reentry_vel = c_double(7500)  # Set a constant reentry velocity for comparison
    #     print("Anomaly height: " + str(run_params.step_acc_hgt))
    #     miss_distances_0[i] = get_miss(config_file, run_params)
# 
    # miss_distances_1 = np.zeros(len(anomaly_heights))
    # for i in range(len(anomaly_heights)):
    #     run_params = read_config(config_file)
    #     run_params.reentry_vel = c_double(5000)
    #     run_params.step_acc_mag = c_double(-100*9.81)
    #     run_params.step_acc_hgt = c_double(anomaly_heights[i])
    #     run_params.step_acc_dur = c_double(-0.1)
    #     print("Anomaly height: " + str(run_params.step_acc_hgt))
    #     miss_distances_1[i] = get_miss(config_file, run_params)
# 
    # miss_distances_2 = np.zeros(len(anomaly_heights))
    # for i in range(len(anomaly_heights)):
    #     run_params = read_config(config_file)
    #     run_params.reentry_vel = c_double(2500)
    #     run_params.step_acc_mag = c_double(-100*9.81)
    #     run_params.step_acc_hgt = c_double(anomaly_heights[i])
    #     run_params.step_acc_dur = c_double(-0.1)
    #     print("Anomaly height: " + str(run_params.step_acc_hgt))
    #     miss_distances_2[i] = get_miss(config_file, run_params)

    # Plot the miss distances
    
    params = {
    'axes.labelsize': 18,
    'font.size': 18,
    'font.family': 'serif',
    'legend.fontsize': 18,
    'xtick.labelsize': 18,
    'ytick.labelsize': 18,
    }

    plt.figure(figsize=(10,10))
    ax = plt.gca()

    plt.rcParams.update(params)
    # set color palette
    colors = plt.cm.viridis(np.linspace(0, 1, 7))
    # plt.plot(anomaly_heights, miss_distances_0, label="7500 m/s")
    # plt.plot(anomaly_heights, miss_distances_1, label="5000 m/s")
    # plt.plot(anomaly_heights, miss_distances_2, label="2500 m/s")
    # plt.xlabel("Anomaly height (m)")
    # plt.ylabel("Miss distance (m)")
    # # plt.yscale('symlog')
    # plt.legend()
    # plt.savefig("./output/" + config_file + "/miss_distance_anomaly_height.pdf")
    # plt.close()

    """
    # Second, for 10km anomaly height, probe the sensitivity of the miss distance to the time delay
    deflection_times = np.logspace(-3, 0, 100)

    miss_distances_0 = np.zeros(len(deflection_times))
    for i in range(len(deflection_times)):
        run_params = read_config(config_file)
        run_params.step_acc_mag = c_double(10*9.81)
        run_params.step_acc_hgt = c_double(10000)
        run_params.deflection_time = c_double(deflection_times[i])
        print("Deflection time: " + str(run_params.deflection_time))
        miss_distances_0[i] = get_miss(config_file, run_params)

    miss_dstances_1 = np.zeros(len(deflection_times))
    for i in range(len(deflection_times)):
        run_params = read_config(config_file)
        run_params.step_acc_mag = c_double(10*9.81)
        run_params.step_acc_hgt = c_double(1000)
        run_params.deflection_time = c_double(deflection_times[i])
        print("Deflection time: " + str(run_params.deflection_time))
        miss_dstances_1[i] = get_miss(config_file, run_params)

    miss_dstances_2 = np.zeros(len(deflection_times))
    for i in range(len(deflection_times)):
        run_params = read_config(config_file)
        run_params.step_acc_mag = c_double(10*9.81)
        run_params.step_acc_hgt = c_double(50000)
        run_params.deflection_time = c_double(deflection_times[i])
        print("Deflection time: " + str(run_params.deflection_time))
        miss_dstances_2[i] = get_miss(config_file, run_params)

    miss_dstances_3 = np.zeros(len(deflection_times))
    for i in range(len(deflection_times)):
        run_params = read_config(config_file)
        run_params.step_acc_mag = c_double(9.81)
        run_params.step_acc_hgt = c_double(10000)
        run_params.deflection_time = c_double(deflection_times[i])
        print("Deflection time: " + str(run_params.deflection_time))
        miss_dstances_3[i] = get_miss(config_file, run_params)

    miss_dstances_4 = np.zeros(len(deflection_times))
    for i in range(len(deflection_times)):
        run_params = read_config(config_file)
        run_params.step_acc_mag = c_double(9.81)
        run_params.step_acc_hgt = c_double(1000)
        run_params.deflection_time = c_double(deflection_times[i])
        print("Deflection time: " + str(run_params.deflection_time))
        miss_dstances_4[i] = get_miss(config_file, run_params)

    miss_dstances_5 = np.zeros(len(deflection_times))
    for i in range(len(deflection_times)):
        run_params = read_config(config_file)
        run_params.step_acc_mag = c_double(9.81)
        run_params.step_acc_hgt = c_double(50000)
        run_params.deflection_time = c_double(deflection_times[i])
        print("Deflection time: " + str(run_params.deflection_time))
        miss_dstances_5[i] = get_miss(config_file, run_params)

    plt.figure(figsize=(10,10))
    ax = plt.gca()
    plt.plot(deflection_times, miss_dstances_2, label="50km, 10g")
    plt.plot(deflection_times, miss_distances_0, label="10km, 10g")
    plt.plot(deflection_times, miss_dstances_1, label="1km, 10g")
    plt.plot(deflection_times, miss_dstances_5, label="50km, 1g")
    plt.plot(deflection_times, miss_dstances_3, label="10km, 1g")
    plt.plot(deflection_times, miss_dstances_4, label="1km, 1g")
    plt.title("Reentry velocity: " + str(run_params.reentry_vel) + " m/s")
    plt.xlabel("Deflection time (s)")
    plt.ylabel("Miss distance (m)")
    plt.yscale('symlog')
    plt.xscale('log')
    plt.legend()
    plt.savefig("./output/" + config_file + "/miss_distance_deflection_time.pdf")
    plt.close()
    """

    # Plot the miss distance with respect to the gearing ratio 
    gearing_ratios = np.logspace(-3,  0.5, 50)

    miss_distances_0 = np.zeros(len(gearing_ratios))
    for i in range(len(gearing_ratios)):
        run_params = read_config(config_file)
        run_params.step_acc_hgt = c_double(10000)
        run_params.reentry_vel = c_double(7500)
        run_params.gearing_ratio = c_double(gearing_ratios[i])
        print("Gearing ratio: " + str(run_params.gearing_ratio))
        miss_distances_0[i] = get_miss(config_file, run_params)

    miss_distances_1 = np.zeros(len(gearing_ratios))
    for i in range(len(gearing_ratios)):
        run_params = read_config(config_file)
        run_params.step_acc_hgt = c_double(10000)
        run_params.reentry_vel = c_double(5000)
        run_params.gearing_ratio = c_double(gearing_ratios[i])
        print("Gearing ratio: " + str(run_params.gearing_ratio))
        miss_distances_1[i] = get_miss(config_file, run_params)

    miss_distances_2 = np.zeros(len(gearing_ratios))
    for i in range(len(gearing_ratios)):
        run_params = read_config(config_file)
        run_params.step_acc_hgt = c_double(10000)
        run_params.reentry_vel = c_double(2500)
        run_params.gearing_ratio = c_double(gearing_ratios[i])
        print("Gearing ratio: " + str(run_params.gearing_ratio))
        miss_distances_2[i] = get_miss(config_file, run_params)
    
    plt.figure(figsize=(10,10))
    ax = plt.gca()
    plt.plot(gearing_ratios, miss_distances_2, label="2500 m/s")
    plt.plot(gearing_ratios, miss_distances_1, label="5000 m/s")
    plt.plot(gearing_ratios, miss_distances_0, label="7500 m/s")
    plt.xlabel("Gearing ratio")
    plt.ylabel("Miss distance (m)")
    plt.title("Anomaly height: " + str(run_params.step_acc_hgt) + " m")
    plt.xscale('log')
    plt.yscale('symlog')
    # set y range 
    plt.ylim(-1e0, 1e3)
    plt.legend()
    plt.savefig("./output/" + config_file + "/miss_distance_gearing_ratio.pdf")
    plt.close()



