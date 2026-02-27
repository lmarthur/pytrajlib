import os
import sys
from ctypes import *

from scipy.optimize import differential_evolution, minimize

from impact_plot import *
from traj_plot import *

# Specify the input file name (without the extension)
config_file = "run_0"

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
sys.path.append(".")
from src.pylib import *

so_file = "./build/libPyTraj.so"
pytraj = CDLL(so_file)


def get_miss_distance(params):
    # thrust_lon, t_vert_boost = params
    t_des_final, thrust_lon, t_vert_boost = params
    # t_des_final = params
    run_params.t_des_final = c_double(t_des_final)
    run_params.theta_long = c_double(thrust_lon)
    run_params.t_vert_boost = c_double(t_vert_boost)
    impact_data_pointer = pytraj.mc_run(run_params)

    # Copy the input file to the output directory
    os.system(f"cp {config_path} ./output/{config_file}")

    impact_data = np.loadtxt(
        "./output/" + config_file + "/" + "impact_data.txt", delimiter=",", skiprows=1
    )

    # get longitude and latitude of aimpoint
    aimpoint_lon = np.arctan2(run_params.y_aim, run_params.x_aim)
    aimpoint_lat = np.arctan2(
        run_params.z_aim, np.sqrt(run_params.x_aim**2 + run_params.y_aim**2)
    )

    # Calculate the range to the aimpoint over the surface of the Earth
    # This is the great circle distance between the aimpoint and the origin
    range_to_aimpoint = np.arccos(
        np.sin(aimpoint_lat) * np.sin(0)
        + np.cos(aimpoint_lat) * np.cos(0) * np.cos(aimpoint_lon)
    )
    range_to_aimpoint = range_to_aimpoint * 6371e3
    # print('Range to aimpoint: ', range_to_aimpoint)

    impact_t = impact_data[0]
    impact_x = impact_data[1]
    impact_y = impact_data[2]
    impact_z = impact_data[3]

    # get vector relative to aimpoint
    impact_x = impact_x - run_params.x_aim
    impact_y = impact_y - run_params.y_aim
    impact_z = impact_z - run_params.z_aim

    # get the miss distances
    dist = np.sqrt(impact_x**2 + impact_y**2 + impact_z**2)
    print(
        "miss dist: ",
        dist,
        "t",
        run_params.t_des_final,
        "thrust lon",
        thrust_lon,
        "t_vert_boost",
        t_vert_boost,
    )
    return dist


# Code block to run the Monte Carlo simulation
if __name__ == "__main__":
    # Read the configuration file
    print("Reading configuration file " + config_file + ".toml...")
    run_params = read_config(config_file)
    print("Configuration file read.")

    aimpoint_lon = np.arctan2(run_params.y_aim, run_params.x_aim)
    aimpoint_lat = np.arctan2(
        run_params.z_aim, np.sqrt(run_params.x_aim**2 + run_params.y_aim**2)
    )
    print(f"{aimpoint_lat=}, {aimpoint_lon=}")
    print()
    # Calculate the range to the aimpoint over the surface of the Earth
    # This is the great circle distance between the aimpoint and the origin
    range_to_aimpoint = np.arccos(
        np.sin(aimpoint_lat) * np.sin(0)
        + np.cos(aimpoint_lat) * np.cos(0) * np.cos(aimpoint_lon)
    )
    range_to_aimpoint = range_to_aimpoint * 6371e3
    print("Range to aimpoint: ", range_to_aimpoint)

    RDESKM = range_to_aimpoint / 1000
    tf_des = 252.0 + 0.223 * RDESKM - (5.44e-6) * RDESKM * RDESKM
    print("Initial guess for t_des_final: ", tf_des)
    # tf_des = 4000
    thrust_lon = 1.04719755
    t_vert_boost = 10
    run_params.t_des_final = c_double(tf_des)
    run_params.theta_long = c_double(thrust_lon)
    run_params.t_vert_boost = c_double(t_vert_boost)

    # Save original parameters
    orig_num_runs = run_params.num_runs
    tmp_traj_output = run_params.traj_output
    tmp_initial_pos_error = run_params.initial_pos_error
    tmp_initial_vel_error = run_params.initial_vel_error
    tmp_initial_angle_error = run_params.initial_angle_error
    tmp_acc_scale_stability = run_params.acc_scale_stability
    tmp_gyro_bias_stability = run_params.gyro_bias_stability
    tmp_gyro_noise = run_params.gyro_noise
    tmp_grav_error = run_params.grav_error
    tmp_atm_model = run_params.atm_model
    tmp_rv_maneuv = run_params.rv_maneuv

    # # Set error parameters to zero for optimization
    run_params.initial_pos_error = 0
    run_params.initial_vel_error = 0
    run_params.initial_angle_error = 0
    run_params.acc_scale_stability = 0
    run_params.gyro_bias_stability = 0
    run_params.gyro_noise = 0
    run_params.grav_error = 0
    run_params.atm_model = 0 if tmp_atm_model <= 1 else 3
    run_params.rv_maneuv = 0
    run_params.traj_output = 0
    run_params.num_runs = 1

    result = minimize(
        get_miss_distance,
        x0=(tf_des, thrust_lon, t_vert_boost),
        method="Nelder-Mead",
        bounds=[(300, 5000), (0, np.pi), (0.1, 100)],
    )
    print(result)

    impact_data_pointer = pytraj.mc_run(run_params)

    impact_data = np.loadtxt(
        "./output/" + config_file + "/" + "impact_data.txt", delimiter=",", skiprows=1
    )
    impact_data = np.atleast_2d(np.array(impact_data))
    impact_t = impact_data[:, 0]
    impact_x = impact_data[:, 1]
    impact_y = impact_data[:, 2]
    impact_z = impact_data[:, 3]
    print(
        f"Aimpoint location: {impact_x[0]}, {impact_y[0]}, {impact_z[0]} t={impact_t[0]}"
    )

    # # Restore original parameters after optimization
    run_params.num_runs = orig_num_runs
    run_params.initial_pos_error = tmp_initial_pos_error
    run_params.initial_vel_error = tmp_initial_vel_error
    run_params.initial_angle_error = tmp_initial_angle_error
    run_params.acc_scale_stability = tmp_acc_scale_stability
    run_params.gyro_bias_stability = tmp_gyro_bias_stability
    run_params.gyro_noise = tmp_gyro_noise
    run_params.grav_error = tmp_grav_error
    run_params.atm_model = tmp_atm_model
    run_params.rv_maneuv = tmp_rv_maneuv
    run_params.traj_output = tmp_traj_output

    impact_data_pointer = pytraj.mc_run(run_params)
    print("Monte Carlo simulation complete.")

    # Copy the input file to the output directory
    os.system(f"cp {config_path} ./output/{config_file}")

    # Plot the trajectory
    if run_params.traj_output:
        print("Plotting trajectory...")
        traj_plot("./output/" + config_file + "/")
        print("Trajectory plotted.")

    # Plot the impact data
    print("Plotting impact data...")
    impact_plot("./output/" + config_file + "/", run_params)
    print("Impact data plotted.")
