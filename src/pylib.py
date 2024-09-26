import numpy as np
from ctypes import *
import configparser

so_file = "./build/libPyTraj.so"
pytraj = CDLL(so_file)

# define the runparam struct
class runparams(Structure):
    _fields_ = [
        ("num_runs", c_int),
        ("time_step", c_double),
        ("traj_output", c_int),
        ("x_aim", c_double),
        ("y_aim", c_double),
        ("z_aim", c_double),
        
        ("grav_error", c_int),
        ("atm_error", c_int),
        ("gnss_nav", c_int),
        ("ins_nav", c_int),
        ("filter_type", c_int), # filter type (0: None, 1: KF, 2: EKF)

        ("rv_type", c_int), # 0 for ballistic, 1 for maneuverable

        ("initial_x_error", c_double),
        ("initial_pos_error", c_double),
        ("initial_vel_error", c_double),
        ("initial_angle_error", c_double),
    ]

# define the state struct
class state(Structure):
    _fields_ = [
        ("t", c_double),
        ("theta_long",c_double),
        ("theta_lat", c_double),
        ("x", c_double),
        ("y", c_double),
        ("z", c_double),
        ("vx", c_double),
        ("vy", c_double),
        ("vz", c_double),
        ("ax_grav", c_double),
        ("ay_grav", c_double),
        ("az_grav", c_double),
        ("ax_drag", c_double),
        ("ay_drag", c_double),
        ("az_drag", c_double),
        ("ax_lift", c_double),
        ("ay_lift", c_double),
        ("az_lift", c_double),
        ("ax_thrust", c_double),
        ("ay_thrust", c_double),
        ("az_thrust", c_double),
        ("ax_total", c_double),
        ("ay_total", c_double),
        ("az_total", c_double),
    ]

# define the booster struct
class booster(Structure):
    _fields_ = [
        ("name", c_char_p),
        ("num_stages", c_int),
        ("maxdiam", c_double),
        ("area", c_double),
        ("total_burn_time", c_double),
        ("bus_mass", c_double),
        ("total_mass", c_double),
        ("c_d_0", c_double),
        ("wet_mass", c_double * 3),
        ("fuel_mass", c_double * 3),
        ("dry_mass", c_double * 3),
        ("isp0", c_double * 3),
        ("burn_time", c_double * 3),
        ("fuel_burn_rate", c_double * 3),
    ]

# define the rv struct
class rv(Structure):
    _fields_ = [
        ("name", c_char_p),
        ("maneuverability_flag", c_int),
        ("rv_mass", c_double),
        ("rv_length", c_double),
        ("rv_radius", c_double),
        ("rv_area", c_double),
        ("c_d_0", c_double),
        ("c_d_alpha", c_double),
        ("c_m_alpha", c_double),
        ("c_m_q", c_double),
        ("c_l_alpha", c_double),
        ("flap_area", c_double),
        ("x_flap", c_double),
        ("x_com", c_double),
        ("Iyy", c_double),
    ]

# define the vehicle struct
class vehicle(Structure):
    _fields_ = [
        ("booster", POINTER(booster)),
        ("rv", POINTER(rv)),
        ("total_mass", c_double),
        ("current_mass", c_double),
    ]

def fly(run_params, initial_state, booster_type, rv_type):
    """
    Python wrapper for the fly function in the C library. This function flies the vehicle.

    INPUTS:
    ----------
        run_params: runparams
            The run parameters.
        initial_state: state * (pointer to a state struct)
            The initial state of the vehicle.
        vehicle: vehicle * (pointer to a vehicle struct)
            The vehicle to fly. Default is MMIII.
    OUTPUTS:
    ----------
        final_state: state
            The final state of the vehicle.
    """
    # set the return types
    pytraj.fly.restype = state

    # set the argument types
    pytraj.fly.argtypes = [POINTER(runparams), POINTER(state), POINTER(booster), POINTER(rv)]

    final_state = pytraj.fly(run_params, initial_state, booster_type, rv_type)

    return final_state

def read_config(config_file):
    """
    Function to read the .toml configuration file and return the run parameters.

    INPUTS:
    ----------
        config_file: str
            The path to the configuration file.
    OUTPUTS:
    ----------
        run_params: runparams
            The run parameters.
    """
    # read the configuration file
    config = configparser.ConfigParser()
    config.read(config_file)

    # create the run parameters struct
    run_params = runparams()

    # set the run parameters
    run_params.num_runs = c_int(int(config['RUN']['num_runs']))
    run_params.time_step = c_double(float(config['RUN']['time_step']))
    run_params.traj_output = c_int(int(config['RUN']['traj_output']))
    run_params.x_aim = c_double(float(config['RUN']['x_aim']))
    run_params.y_aim = c_double(float(config['RUN']['y_aim']))
    run_params.z_aim = c_double(float(config['RUN']['z_aim']))

    # set the flight parameters
    run_params.grav_error = c_int(int(config['FLIGHT']['grav_error']))
    run_params.atm_error = c_int(int(config['FLIGHT']['atm_error']))
    run_params.gnss_nav = c_int(int(config['FLIGHT']['gnss_nav']))
    run_params.ins_nav = c_int(int(config['FLIGHT']['ins_nav']))
    run_params.filter_type = c_int(int(config['FLIGHT']['filter_type']))

    # set the vehicle parameters
    run_params.rv_type = c_int(int(config['VEHICLE']['rv_type']))
    
    # set the error parameters
    run_params.initial_x_error = c_double(float(config['ERRORPARAMS']['initial_x_error']))
    run_params.initial_pos_error = c_double(float(config['ERRORPARAMS']['initial_pos_error']))
    run_params.initial_vel_error = c_double(float(config['ERRORPARAMS']['initial_vel_error']))
    run_params.initial_angle_error = c_double(float(config['ERRORPARAMS']['initial_angle_error']))

    return run_params

def init_state(run_params):
    """
    Function to initialize the state of the vehicle.

    INPUTS:
    ----------
        run_params: runparams
            The run parameters.
    OUTPUTS:
    ----------
        initial_state: state
            The initial state of the vehicle.
    """
    # Generic initialization of the state
    initial_state = pytraj.init_state()

    # tune the initial state based on the run parameters
    initial_state.x += np.random.normal(0, run_params.initial_x_error)
    initial_state.y += np.random.normal(0, run_params.initial_pos_error)
    initial_state.z += np.random.normal(0, run_params.initial_pos_error)

    initial_state.vy += np.random.normal(0, run_params.initial_vel_error)
    initial_state.vz += np.random.normal(0, run_params.initial_vel_error)

    # set the initial launch angle
    # TODO: add random error to the launch angle
    # initial_state.theta_long = c_double(np.pi/4)

    return initial_state

def mc_run(run_params):
    """
    Function to run a Monte Carlo simulation of the vehicle flight by calling the fly function with random error injections.
    
    INPUTS:
    ----------
        config_file: str
            The path to the configuration file.

    """

    print("Running Monte Carlo simulation...")

    # set the return types
    pytraj.init_state.restype = state
    pytraj.init_mmiii_booster.restype = booster
    pytraj.init_ballistic_rv.restype = rv

    # initialize the run parameters and utilities
    
    num_runs = run_params.num_runs

    # initialize the booster and rv types
    booster_type = pytraj.init_mmiii_booster()
    rv_type = pytraj.init_ballistic_rv()

    # instantiate the runresults numpy array
    run_results = np.zeros((num_runs, 7))

    # iterate over the number of Monte Carlo runs
    for i in range(num_runs):

        # initialize the state
        initial_state = init_state(run_params)
        # print("Initial state: ", initial_state.t, initial_state.x, initial_state.y, initial_state.z, initial_state.vx, initial_state.vy, initial_state.vz, initial_state.ax_total, initial_state.ay_total, initial_state.az_total)
        # fly the vehicle
        final_state = fly(run_params, initial_state, booster_type, rv_type)

        # store the final state in the run results array
        run_results[i] = [final_state.t, final_state.x, final_state.y, final_state.z, final_state.vx, final_state.vy, final_state.vz]

    # output the results to a text file
    np.savetxt("./output/impact_data.txt", run_results)

    return run_results