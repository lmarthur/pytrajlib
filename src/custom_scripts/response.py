# This script runs the reentry-only simulations specified in the reentry.toml configuration file. 

import sys
import os
from ctypes import *
import matplotlib.pyplot as plt
import numpy as np
import scipy as sp
import scienceplots

from maneuverability import *

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

    # # First, standardized time delay with variable anomaly height
    # anomaly_heights = np.linspace(0, 50000, 500)
    # miss_distances_0 = np.zeros(len(anomaly_heights))
    # for i in range(len(anomaly_heights)):
    #     run_params = read_config(config_file)
    #     run_params.step_acc_mag = c_double(100.0)
    #     run_params.step_acc_hgt = c_double(anomaly_heights[i])
    #     print("Anomaly height: " + str(run_params.step_acc_hgt))
    #     miss_distances_0[i] = get_miss(config_file, run_params)
# 
    # # Repeat with 10x larger anomaly
    # miss_distances_1 = np.zeros(len(anomaly_heights))
    # for i in range(len(anomaly_heights)):
    #     run_params = read_config(config_file)
    #     run_params.step_acc_mag = c_double(1000.0)
    #     run_params.step_acc_hgt = c_double(anomaly_heights[i])
    #     print("Anomaly height: " + str(run_params.step_acc_hgt))
    #     miss_distances_1[i] = get_miss(config_file, run_params)

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
    # plt.plot(anomaly_heights, miss_distances_1, label="100g")
    # plt.plot(anomaly_heights, miss_distances_0, label="10g")
    # plt.annotate("Deflection time: " + str(run_params.deflection_time) + " s", xy=(0.2, 0.85), xycoords='axes fraction', fontsize=18, ha='center', va='center')
    # plt.xlabel("Anomaly height (m)")
    # plt.ylabel("Miss distance (m)")
    # # plt.yscale('symlog')
    # plt.legend()
    # plt.savefig("./output/" + config_file + "/miss_distance_anomaly_height.pdf")
    # plt.close()

    # Second, for 10km anomaly height, probe the sensitivity of the miss distance to the time delay
    # deflection_times = np.logspace(-3, 0, 100)
# 
    # miss_distances_0 = np.zeros(len(deflection_times))
    # for i in range(len(deflection_times)):
    #     run_params = read_config(config_file)
    #     run_params.step_acc_mag = c_double(100.0)
    #     run_params.step_acc_hgt = c_double(10000)
    #     run_params.deflection_time = c_double(deflection_times[i])
    #     print("Deflection time: " + str(run_params.deflection_time))
    #     miss_distances_0[i] = get_miss(config_file, run_params)
# 
    # miss_dstances_1 = np.zeros(len(deflection_times))
    # for i in range(len(deflection_times)):
    #     run_params = read_config(config_file)
    #     run_params.step_acc_mag = c_double(100.0)
    #     run_params.step_acc_hgt = c_double(1000)
    #     run_params.deflection_time = c_double(deflection_times[i])
    #     print("Deflection time: " + str(run_params.deflection_time))
    #     miss_dstances_1[i] = get_miss(config_file, run_params)
# 
    # miss_dstances_2 = np.zeros(len(deflection_times))
    # for i in range(len(deflection_times)):
    #     run_params = read_config(config_file)
    #     run_params.step_acc_mag = c_double(100.0)
    #     run_params.step_acc_hgt = c_double(50000)
    #     run_params.deflection_time = c_double(deflection_times[i])
    #     print("Deflection time: " + str(run_params.deflection_time))
    #     miss_dstances_2[i] = get_miss(config_file, run_params)
# 
    # miss_dstances_3 = np.zeros(len(deflection_times))
    # for i in range(len(deflection_times)):
    #     run_params = read_config(config_file)
    #     run_params.step_acc_mag = c_double(10.0)
    #     run_params.step_acc_hgt = c_double(10000)
    #     run_params.deflection_time = c_double(deflection_times[i])
    #     print("Deflection time: " + str(run_params.deflection_time))
    #     miss_dstances_3[i] = get_miss(config_file, run_params)
# 
    # miss_dstances_4 = np.zeros(len(deflection_times))
    # for i in range(len(deflection_times)):
    #     run_params = read_config(config_file)
    #     run_params.step_acc_mag = c_double(10.0)
    #     run_params.step_acc_hgt = c_double(1000)
    #     run_params.deflection_time = c_double(deflection_times[i])
    #     print("Deflection time: " + str(run_params.deflection_time))
    #     miss_dstances_4[i] = get_miss(config_file, run_params)
# 
    # miss_dstances_5 = np.zeros(len(deflection_times))
    # for i in range(len(deflection_times)):
    #     run_params = read_config(config_file)
    #     run_params.step_acc_mag = c_double(10.0)
    #     run_params.step_acc_hgt = c_double(50000)
    #     run_params.deflection_time = c_double(deflection_times[i])
    #     print("Deflection time: " + str(run_params.deflection_time))
    #     miss_dstances_5[i] = get_miss(config_file, run_params)
# 
    # plt.figure(figsize=(10,10))
    # ax = plt.gca()
    # plt.plot(deflection_times, miss_dstances_2, label="50km, 10g, 0.1s")
    # plt.plot(deflection_times, miss_distances_0, label="10km, 10g, 0.1s")
    # plt.plot(deflection_times, miss_dstances_1, label="1km, 10g, 0.1s")
    # plt.plot(deflection_times, miss_dstances_5, label="50km, 1g, 0.1s")
    # plt.plot(deflection_times, miss_dstances_3, label="10km, 1g, 0.1s")
    # plt.plot(deflection_times, miss_dstances_4, label="1km, 1g, 0.1s")
    # plt.title("Reentry velocity: " + str(run_params.reentry_vel) + " m/s")
    # plt.xlabel("Deflection time (s)")
    # plt.ylabel("Miss distance (m)")
    # plt.yscale('symlog')
    # plt.xscale('log')
    # plt.legend()
    # plt.savefig("./output/" + config_file + "/miss_distance_deflection_time.pdf")
    # plt.close()

    # Simple trajectory run
    run_params = read_config(config_file)
    miss_distance = get_miss(config_file, run_params)

    # Extract the drag acceleration data from the trajectory data
    traj_data = np.loadtxt("./output/reentry/trajectory.txt", delimiter = ",", skiprows=1)
    # remove all but the 2nd, 3rd, 4th, 11th, 12th, and 13th columns
    traj_data = traj_data[:,[0,2,3,4,11,12,13]]

    # extract the velocity data 
    vel_data = np.loadtxt("./output/reentry/trajectory.txt", delimiter = ",", skiprows=1)
    # remove all but the 5th, 6th, and 7th columns
    vel_data = vel_data[:,[0,5,6,7]]


    # iterate through the rows, and if the altitude is greater than 100km, remove the row
    for i in range(len(traj_data)):
        # calculate the altitude
        altitude = np.sqrt(np.square(traj_data[i,1]) + np.square(traj_data[i,2]) + np.square(traj_data[i,3])) - 6371e3

        if altitude < 100000:
            traj_data = traj_data[i:,]
            break
    
    # plot the drag acceleration data
    lat_acc_mag = np.sqrt(np.square(traj_data[:,5]) + np.square(traj_data[:,6]))
    # multiply by a hanning window
    n = len(traj_data[:,0]) # length of the signal
    print(n)
    window = np.hanning(n)
    plt.figure(figsize=(10,10))
    ax = plt.gca()
    plt.plot(traj_data[:,0], lat_acc_mag, label="Drag acceleration", color=colors[0])
    plt.plot(traj_data[:,0], lat_acc_mag * window, label="Drag acceleration (windowed)", color=colors[1])
    plt.title("Lateral drag acceleration")
    plt.xlabel("Time (s)")
    plt.ylabel("Drag acceleration (m/s^2)")
    plt.legend()
    plt.savefig("./output/" + config_file + "/drag_acceleration_red.pdf")
    plt.close()

    # plot the power spectrum of the drag acceleration data
    # calculate the power spectrum with scipy, for a 10kHz sampling rate
    # flattop, hann, blackmanharris windows seem to work the best
    f, Pxx = sp.signal.welch(lat_acc_mag, fs=10000, nperseg=256, detrend='linear', average='mean', return_onesided=True, window='hann', scaling='density', nfft=256)
    # plot the power spectrum
    plt.figure(figsize=(10,10))
    ax = plt.gca()
    plt.plot(f, Pxx, label="Hann", color=colors[0])
    f, Pxx = sp.signal.welch(lat_acc_mag, fs=10000, nperseg=256, detrend='linear', average='mean', return_onesided=True, window='blackmanharris', scaling='density', nfft=256)
    plt.plot(f, Pxx, label="Blackman-Harris", color=colors[2])
    f, Pxx = sp.signal.welch(lat_acc_mag, fs=10000, nperseg=256, detrend='linear', average='mean', return_onesided=True, window='flattop', scaling='density', nfft=256)
    plt.plot(f, Pxx, label="Flattop", color=colors[4])
    # plt.xscale('log')
    plt.yscale('log')
    plt.title("Lateral drag acceleration power spectrum")
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Power spectral density (m^2/s^4/Hz)")

    plt.legend()
    plt.savefig("./output/" + config_file + "/drag_acceleration_spectrum_red.pdf")
    plt.close()

# 
    # # find the frequency of the peak in the power spectrum
    peak_freq = f[np.argmax(Pxx)]
    print("Peak frequency: ", peak_freq, "Hz")

    # At each time step, calculate the pitching time constant
    time_constants = np.zeros(len(traj_data[:,0]))
    pitching_freqs = np.zeros(len(traj_data[:,0]))
    drag_accel = np.zeros(len(traj_data[:,0]))
    # Calculate the moment of inertia
    Iy = moment_of_inertia(radius=0.25, length_cylinder=1.63, length_cone=1.12, density=rv_density)
    
    for i in range(len(traj_data[:,0])):
        # Calculate the atmospheric density
        rho = atm_density(altitude = np.sqrt(np.square(traj_data[i,1]) + np.square(traj_data[i,2]) + np.square(traj_data[i,3])) - 6371e3)
        
        # Calculate the time constant
        tc = time_constant(Iy, rho, c_m_alpha=-0.15, radius=0.5, vel=np.sqrt(np.square(vel_data[i,1]) + np.square(vel_data[i,2]) + np.square(vel_data[i,3])))
        
        # Calculate the total drag acceleration
        drag_accel[i] = np.sqrt((np.square(traj_data[i,4]) + np.square(traj_data[i,5]) + np.square(traj_data[i,6])))
        # Store the time constant
        time_constants[i] = tc
        pitching_freqs[i] = 1 / tc
        
    print("Time constant: ", time_constants)
    print("Pitching frequency: ", pitching_freqs)

    # histogram of the pitching frequencies
    # np.histogram(pitching_freqs, bins=150, range=(0, 150), density=True)
    plt.figure(figsize=(10,10))
    ax = plt.gca()
    plt.hist(pitching_freqs, bins=120, range=(0, 120), weights=np.square(drag_accel), density=True, color=colors[0], label="Pitching frequency")
    plt.title("Pitching frequency distribution")
    plt.xlabel("Pitching frequency (Hz)")
    plt.ylabel("Power spectral density (m$^2$ s$^{-4}$ Hz$^{-1}$)")
    plt.xlim(0, 120)
    plt.legend()
    plt.savefig("./output/" + config_file + "/pitching_frequency_distribution.pdf")