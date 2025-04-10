import pytest
import sys
from ctypes import *
import numpy as np


sys.path.append('.')
from src.pylib import *
so_file = "./build/libPyTraj.so"
pytraj = CDLL(so_file)

# Specify the input file name (without the extension)
config_file = "test"

# Check for the existence of the input file
config_path = f"./input/{config_file}.toml"

def test_read_config():
    """
    Test suite for the read_config function
    """
    run_params = read_config("test")

    assert run_params.run_type == 0
    assert run_params.num_runs == 2
    assert run_params.time_step_main == 1.0
    assert run_params.time_step_reentry == 0.01
    assert run_params.traj_output == 0
    assert run_params.x_aim == 6371e3
    assert run_params.y_aim == 0
    assert run_params.z_aim == 0
    assert run_params.theta_long == 0.785398163397
    assert run_params.theta_lat == 0.0
    
    assert run_params.grav_error == 0
    assert run_params.atm_error == 0
    assert run_params.gnss_nav == 0
    assert run_params.ins_nav == 1
    assert run_params.rv_maneuv == 1
    assert run_params.reentry_vel == 7500
    assert run_params.deflection_time == 0.0

    assert run_params.rv_type == 1

    assert run_params.initial_x_error == 0.0
    assert run_params.initial_pos_error == 0.0
    assert run_params.initial_vel_error == 0.0
    assert run_params.initial_angle_error == 0.0
    assert run_params.acc_scale_stability == 0.0
    assert run_params.gyro_bias_stability == 0.0
    assert run_params.gyro_noise == 0.0
    assert run_params.gnss_noise == 0.0
    assert run_params.cl_pert == 0.0
    assert run_params.step_acc_mag == 0.0
    assert run_params.step_acc_hgt == 0.0
    assert run_params.step_acc_dur == 0.0
    


def test_integration_0():
    """
    Verify that first two runs are identical when no random errors are turned on
    """
    
    run_params = read_config("test")
    impact_data_pointer = pytraj.mc_run(run_params)

    # Get the impact data from the pointer
    assert impact_data_pointer != None
    
    # Read the impact data
    run_path = "./output/test/"
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    assert np.allclose(impact_data[0,:], impact_data[1,:], atol=1e-6)


def test_integration_1():
    """
    Verify that first two runs are different when random errors are turned on
    """

    # Turn on random errors and verify that the first two runs are different
    run_params = read_config("test")
    run_params.initial_pos_error = c_double(1.0)

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    run_path = "./output/test/"
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    assert not np.allclose(impact_data[0,:], impact_data[1,:], atol=1e-6)


def test_integration_2():
    """
    Verify that miss distance is near 0 when random errors are turned off
    """

    # Turn off random errors and verify that the miss distance is near 0
    run_params = read_config("test")
    run_params.num_runs = 10

    aimpoint = update_aimpoint(run_params, config_path)

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    run_path = "./output/test/"
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep = get_cep(impact_data, run_params)

    assert cep < 1e-3


def test_integration_3():
    """
    Verify that turning on atmospheric error increases miss distance
    """

    # Turn on atmospheric error and verify that the miss distance increases
    run_params = read_config("test")
    run_params.atm_error = 1
    run_params.num_runs = 10
    run_params.rv_maneuv = 0

    aimpoint = update_aimpoint(run_params, config_path)

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    run_path = "./output/test/"
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep = get_cep(impact_data, run_params)

    assert cep > 1e-3 and cep < 1e3


def test_integration_4():
    """
    Verify that turning on atmospheric error with idealized maneuverability makes miss distance near 0
    """

    # Turn on gravitational error and verify that the miss distance increases
    run_params = read_config("test")
    run_params.atm_error = 1
    run_params.num_runs = 10
    run_params.rv_maneuv = 2

    aimpoint = update_aimpoint(run_params, config_path)

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    run_path = "./output/test/"
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep = get_cep(impact_data, run_params)

    assert cep < 1e-3


def test_integration_5():
    """
    Verify that turning on gravitational error increases miss distance
    """

    # Turn on gravitational error and verify that the miss distance increases
    run_params = read_config("test")
    run_params.grav_error = 1
    run_params.num_runs = 10
    run_params.rv_maneuv = 0

    aimpoint = update_aimpoint(run_params, config_path)

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    run_path = "./output/test/"
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep = get_cep(impact_data, run_params)

    assert cep > 1e-3 and cep < 1e2


def test_integration_6():
    """
    Verify that turning on/increasing initial position error increases miss distance (all guidance combinations)
    """

    # First, no guidance
    # Turn on initial position error and verify that the miss distance increases
    run_params = read_config("test")
    aimpoint = update_aimpoint(run_params, config_path)
    # Boost guidance only
    # Turn on initial position error and verify that the miss distance increases
    run_params.initial_pos_error = c_double(1.0)
    run_params.num_runs = 10
    run_params.rv_maneuv = 0
    

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    run_path = "./output/test/"
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep1 = get_cep(impact_data, run_params)

    run_params.initial_pos_error = c_double(10.0)

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    run_path = "./output/test/"
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep2 = get_cep(impact_data, run_params)

    assert cep1 > 1e-3 and cep1 < 1e2

    # Boost guidance and realistic rv maneuver
    # Turn on initial position error and verify that the miss distance increases
    run_params.initial_pos_error = c_double(1.0)
    run_params.num_runs = 10
    run_params.rv_maneuv = 1

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    run_path = "./output/test/"
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep1 = get_cep(impact_data, run_params)

    run_params.initial_pos_error = c_double(10.0)

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    run_path = "./output/test/"
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep2 = get_cep(impact_data, run_params)

    assert cep1 > 1e-3 and cep1 < 1e2

    # Boost guidance and idealized rv maneuver
    # Turn on initial position error and verify that the miss distance increases
    run_params.initial_pos_error = c_double(1.0)
    run_params.num_runs = 10
    run_params.rv_maneuv = 2

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    run_path = "./output/test/"
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep1 = get_cep(impact_data, run_params)

    run_params.initial_pos_error = c_double(10.0)

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    run_path = "./output/test/"
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep2 = get_cep(impact_data, run_params)

    assert cep1 > 1e-3 and cep1 < 1e2


def test_integration_7():
    """
    Verify that turning on/increasing initial velocity error increases miss distance
    """

    # Turn on initial velocity error and verify that the miss distance increases
    run_params = read_config("test")
    aimpoint = update_aimpoint(run_params, config_path)

    # Boost guidance only
    # Turn on initial velocity error and verify that the miss distance increases
    run_params.initial_vel_error = c_double(0.1)
    run_params.num_runs = 10
    run_params.rv_maneuv = 0

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    run_path = "./output/test/"
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep1 = get_cep(impact_data, run_params)

    run_params.initial_vel_error = c_double(1.0)

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    run_path = "./output/test/"
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep2 = get_cep(impact_data, run_params)

    assert cep1 > 1e-3 and cep1 < cep2

    # Boost guidance and realistic rv maneuver
    # Turn on initial velocity error and verify that the miss distance increases
    run_params.initial_vel_error = c_double(0.1)
    run_params.num_runs = 10
    run_params.rv_maneuv = 1

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    run_path = "./output/test/"
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep1 = get_cep(impact_data, run_params)

    run_params.initial_vel_error = c_double(1.0)

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    run_path = "./output/test/"
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep2 = get_cep(impact_data, run_params)

    assert cep1 > 1e-3 and cep1 < cep2

    # Boost guidance and idealized rv maneuver
    # Turn on initial velocity error and verify that the miss distance increases
    run_params.initial_vel_error = c_double(0.1)
    run_params.num_runs = 10
    run_params.rv_maneuv = 2

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    run_path = "./output/test/"
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep1 = get_cep(impact_data, run_params)

    run_params.initial_vel_error = c_double(1.0)

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    run_path = "./output/test/"
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep2 = get_cep(impact_data, run_params)

    assert cep1 > 1e-3 and cep1 < cep2


def test_integration_8():
    """
    Verify that turning on/increasing initial angle error increases miss distance
    """

    # Turn on initial velocity error and verify that the miss distance increases
    run_params = read_config("test")
    aimpoint = update_aimpoint(run_params, config_path)

    # Read the impact data
    run_path = "./output/test/"

    # Boost guidance only
    # Turn on initial velocity error and verify that the miss distance increases
    run_params.initial_angle_error = c_double(1e-6)
    run_params.num_runs = 10
    run_params.rv_maneuv = 0

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep1 = get_cep(impact_data, run_params)

    run_params.initial_angle_error = c_double(1e-4)

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep2 = get_cep(impact_data, run_params)

    assert cep1 > 1e-3 and cep1 < cep2

    # Boost guidance and realistic rv maneuver
    # Turn on initial velocity error and verify that the miss distance increases
    run_params.initial_angle_error = c_double(1e-6)
    run_params.num_runs = 10
    run_params.rv_maneuv = 1

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep1 = get_cep(impact_data, run_params)

    run_params.initial_angle_error = c_double(1e-4)

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep2 = get_cep(impact_data, run_params)

    assert cep1 > 1e-3 and cep1 < cep2

    # Boost guidance and idealized rv maneuver
    # Turn on initial velocity error and verify that the miss distance increases
    run_params.initial_angle_error = c_double(1e-6)
    run_params.num_runs = 10
    run_params.rv_maneuv = 2

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep1 = get_cep(impact_data, run_params)

    run_params.initial_angle_error = c_double(1e-4)

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep2 = get_cep(impact_data, run_params)

    assert cep1 > 1e-3 and cep1 < cep2


def test_integration_9():
    """
    Verify that turning on/increasing acc scale stability increases miss distance
    """

    # Boost guidance only
    run_params = read_config("test")
    run_params.acc_scale_stability = c_double(1e-8)
    run_params.num_runs = 10
    run_params.rv_maneuv = 0

    aimpoint = update_aimpoint(run_params, config_path)

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    run_path = "./output/test/"
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep1 = get_cep(impact_data, run_params)

    run_params.acc_scale_stability = c_double(1e-6)

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep2 = get_cep(impact_data, run_params)

    assert cep1 > 1e-3 and cep1 < cep2

    # Second, boost guidance and realistic rv maneuver
    run_params.acc_scale_stability = c_double(1e-8)
    run_params.num_runs = 10
    run_params.rv_maneuv = 1

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep1 = get_cep(impact_data, run_params)

    run_params.acc_scale_stability = c_double(1e-6)

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep2 = get_cep(impact_data, run_params)

    assert cep1 > 1e-3 and cep1 < cep2

    # Third, boost guidance and idealized rv maneuver
    # Turn on initial velocity error and verify that the miss distance increases
    run_params.acc_scale_stability = c_double(1e-8)
    run_params.num_runs = 10
    run_params.rv_maneuv = 2

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep1 = get_cep(impact_data, run_params)

    run_params.acc_scale_stability = c_double(1e-6)

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep2 = get_cep(impact_data, run_params)

    assert cep1 > 1e-3 and cep1 < cep2


def test_integration_10():
    """
    Verify that turning on/increasing gyro bias stability increases miss distance
    """

    # Boost guidance only
    run_params = read_config("test")
    run_params.gyro_bias_stability = c_double(1e-8)
    run_params.num_runs = 10
    run_params.rv_maneuv = 0

    aimpoint = update_aimpoint(run_params, config_path)

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    run_path = "./output/test/"
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep1 = get_cep(impact_data, run_params)

    run_params.gyro_bias_stability = c_double(1e-6)

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep2 = get_cep(impact_data, run_params)

    assert cep1 > 1e-3 and cep1 < cep2

    # Second, boost guidance and realistic rv maneuver
    run_params = read_config("test")
    run_params.gyro_bias_stability = c_double(1e-8)
    run_params.num_runs = 10
    run_params.rv_maneuv = 0

    aimpoint = update_aimpoint(run_params, config_path)

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep1 = get_cep(impact_data, run_params)

    run_params.gyro_bias_stability = c_double(1e-6)

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep2 = get_cep(impact_data, run_params)

    assert cep1 > 1e-3 and cep1 < cep2

    # Third, boost guidance and idealized rv maneuver
    run_params.gyro_bias_stability = c_double(1e-8)
    run_params.num_runs = 10
    run_params.rv_maneuv = 2

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep1 = get_cep(impact_data, run_params)

    run_params.gyro_bias_stability = c_double(1e-6)

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep2 = get_cep(impact_data, run_params)

    assert cep1 > 1e-3 and cep1 < cep2


def test_integration_11():
    """
    Verify that turning on/increasing gyro noiseincreases miss distance
    """

    # Boost guidance only
    run_params = read_config("test")
    run_params.gyro_noise = c_double(1e-8)
    run_params.num_runs = 10
    run_params.rv_maneuv = 0

    aimpoint = update_aimpoint(run_params, config_path)

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    run_path = "./output/test/"
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep1 = get_cep(impact_data, run_params)

    run_params.gyro_noise = c_double(1e-6)

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    run_path = "./output/test/"
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep2 = get_cep(impact_data, run_params)

    assert cep1 > 1e-3 and cep1 < cep2

    # Second, boost guidance and realistic rv maneuver

    run_params = read_config("test")
    run_params.gyro_noise = c_double(1e-8)
    run_params.num_runs = 10
    run_params.rv_maneuv = 0

    aimpoint = update_aimpoint(run_params, config_path)

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    run_path = "./output/test/"
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep1 = get_cep(impact_data, run_params)

    run_params.gyro_noise = c_double(1e-6)

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    run_path = "./output/test/"
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep2 = get_cep(impact_data, run_params)

    assert cep1 > 1e-3 and cep1 < cep2

    # Third, boost guidance and idealized rv maneuver
    run_params.gyro_noise = c_double(1e-8)
    run_params.num_runs = 10
    run_params.rv_maneuv = 2

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    run_path = "./output/test/"
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep1 = get_cep(impact_data, run_params)

    run_params.gyro_noise = c_double(1e-6)

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    run_path = "./output/test/"
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep2 = get_cep(impact_data, run_params)

    assert cep1 > 1e-3 and cep1 < cep2


def test_integration_12():
    """
    Verify that turning on rv maneuver decreases miss distance
    """

    run_params = read_config("test")
    run_params.num_runs = 10
    run_params.rv_maneuv = 0
    run_params.atm_error = 1
    run_params.grav_error = 1

    aimpoint = update_aimpoint(run_params, config_path)

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    run_path = "./output/test/"
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep1 = get_cep(impact_data, run_params)

    run_params.rv_maneuv = 1

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    run_path = "./output/test/"
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep2 = get_cep(impact_data, run_params)

    assert cep1 > cep2


def test_integration_13():
    """
    Verify that turning on perfect rv maneuv decreases miss distance compared to realistic rv maneuv
    """

    run_params = read_config("test")
    run_params.run_type = 0
    run_params.num_runs = 10
    run_params.rv_maneuv = 1
    run_params.atm_error = 1

    aimpoint = update_aimpoint(run_params, config_path)

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    run_path = "./output/test/"
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep1 = get_cep(impact_data, run_params)

    run_params.rv_maneuv = 2

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    run_path = "./output/test/"
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep2 = get_cep(impact_data, run_params)

    assert cep1 > cep2


def test_integration_14():
    """
    Verify that turning on GNSS navigation decreases miss distance
    """

    run_params = read_config("test")
    run_params.run_type = 0
    run_params.num_runs = 10
    run_params.rv_maneuv = 1
    run_params.atm_error = 1
    run_params.acc_scale_stability = c_double(1e-6)
    run_params.gyro_bias_stability = c_double(1e-8)
    run_params.gyro_noise = c_double(1e-8)
    run_params.gnss_nav = 0
    run_params.gnss_noise = c_double(1e-2)

    aimpoint = update_aimpoint(run_params, config_path)

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    run_path = "./output/test/"
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep1 = get_cep(impact_data, run_params)

    run_params.gnss_nav = 1

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    run_path = "./output/test/"
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep2 = get_cep(impact_data, run_params)

    assert cep1 > cep2


def test_integration_15():
    """
    Verify that turning on/increasing GNSS noise increases miss distance
    """

    run_params = read_config("test")
    run_params.run_type = 0
    run_params.num_runs = 10
    run_params.rv_maneuv = 1
    run_params.atm_error = 1
    run_params.acc_scale_stability = c_double(1e-6)
    run_params.gyro_bias_stability = c_double(1e-8)
    run_params.gyro_noise = c_double(1e-8)
    run_params.gnss_nav = 1
    run_params.gnss_noise = c_double(0)

    aimpoint = update_aimpoint(run_params, config_path)

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    run_path = "./output/test/"
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep1 = get_cep(impact_data, run_params)

    run_params.gnss_noise = c_double(1e-2)

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    run_path = "./output/test/"
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep2 = get_cep(impact_data, run_params)

    run_params.gnss_noise = c_double(1e1)

    impact_data_pointer = pytraj.mc_run(run_params)

    # Read the impact data
    run_path = "./output/test/"
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    cep3 = get_cep(impact_data, run_params)

    assert cep1 < cep2 < cep3