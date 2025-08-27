import sys
from ctypes import *

import numpy as np
import pytest

from src.pytrajlib.simulation import get_run_params, run
from src.pytrajlib.utils import get_cep


@pytest.fixture()
def run_params():
    """
    Fixture to get test run parameters from the configuration file
    """
    return get_run_params("input/test.toml")


def test_read_config(run_params):
    """
    Test suite for the read_config function
    """
    assert run_params["run_type"] == 0
    assert run_params["num_runs"] == 2
    assert run_params["time_step_main"] == 1.0
    assert run_params["time_step_reentry"] == 0.01
    assert run_params["traj_output"] == 0
    assert run_params["impact_output"] == 1
    # x, y, and z_aim are the locations that would be achieved by the thrust_angles
    assert run_params["x_aim"] == -557819.616403
    assert run_params["y_aim"] == 6.346534e06
    assert run_params["z_aim"] == -31.37706
    assert run_params["x_launch"] == 6371e3
    assert run_params["y_launch"] == 0
    assert run_params["z_launch"] == 0
    assert run_params["theta_long"] == 0.785398163397
    assert run_params["theta_lat"] == 0.0

    assert run_params["grav_error"] == 0
    assert run_params["atm_model"] == 0
    assert run_params["gnss_nav"] == 0
    assert run_params["ins_nav"] == 1
    assert run_params["rv_maneuv"] == 1
    assert run_params["reentry_vel"] == 7500
    assert run_params["deflection_time"] == 0.0

    assert run_params["booster_type"] == 0
    assert run_params["deflection_time"] == 0.0
    assert run_params["actuator_force"] == 12.0
    assert run_params["gearing_ratio"] == 1.0
    assert run_params["nav_gain"] == 5.0

    assert run_params["initial_x_error"] == 0.0
    assert run_params["initial_pos_error"] == 0.0
    assert run_params["initial_vel_error"] == 0.0
    assert run_params["initial_angle_error"] == 0.0
    assert run_params["acc_scale_stability"] == 0.0
    assert run_params["gyro_bias_stability"] == 0.0
    assert run_params["gyro_noise"] == 0.0
    assert run_params["gnss_noise"] == 0.0
    assert run_params["cl_pert"] == 0.0
    assert run_params["step_acc_mag"] == 0.0
    assert run_params["step_acc_hgt"] == 0.0
    assert run_params["step_acc_dur"] == 0.0


def test_identical_runs_without_random_errors(run_params):
    """
    Verify that first two runs are identical when no random errors are turned on
    """
    impact_data = run(run_params, return_config=False)
    assert not impact_data.empty
    assert np.allclose(impact_data.iloc[0, :], impact_data.iloc[1, :], atol=1e-6)


def test_runs_differ_with_initial_position_error(run_params):
    """
    Verify that first two runs are different when random errors are turned on
    """
    run_params["initial_pos_error"] = 1.0
    impact_data = run(run_params, return_config=False)

    assert not np.allclose(impact_data.iloc[0, :], impact_data.iloc[1, :], atol=1e-6)


def test_cep_near_zero_without_random_errors(run_params):
    """
    Verify that miss distance is near 0 when random errors are turned off
    """
    run_params["num_runs"] = 50
    impact_data = run(run_params, return_config=False)
    cep = get_cep(run_params, impact_data)

    assert cep < 1e-3


def test_atmospheric_error_increases_cep(run_params):
    """
    Verify that turning on atmospheric error increases miss distance
    """
    run_params["atm_model"] = 1  # Use exponential model with wind perturbations
    run_params["num_runs"] = 50
    run_params["rv_maneuv"] = 0

    impact_data = run(run_params, return_config=False)
    cep = get_cep(run_params, impact_data)

    assert cep > 1e-3 and cep < 1e3


def test_gravity_error_increases_cep(run_params):
    """
    Verify that turning on gravitational error increases miss distance
    """
    run_params["grav_error"] = 1
    run_params["num_runs"] = 50
    run_params["rv_maneuv"] = 0

    impact_data = run(run_params, return_config=False)

    cep = get_cep(run_params, impact_data)

    assert cep > 1e-3 and cep < 1e2


@pytest.mark.parametrize(
    "error_type, error_value_low, error_value_high",
    [
        ("initial_pos_error", 1.0, 10.0),
        ("initial_vel_error", 0.1, 1.0),
        ("initial_angle_error", 1e-6, 1e-4),
        ("acc_scale_stability", 1e-6, 1e-4),
        ("gyro_bias_stability", 1e-6, 1e-4),
        ("gyro_noise", 1e-6, 1e-4),
    ],
)
@pytest.mark.parametrize(
    "rv_maneuv",
    [
        0,
        1,
    ],
)
def test_increase_error_increase_cep_all_guidance(
    run_params, error_type, error_value_low, error_value_high, rv_maneuv
):
    """
    Verify that turning on/increasing initial position error increases miss distance (all guidance combinations)

    rv_maneuv: 0 for boost guidance only, 1 for realistic rv maneuver
    """
    ceps = []
    errors = [0, error_value_low, error_value_high]
    for error in errors:
        run_params[error_type] = error
        run_params["num_runs"] = 100
        run_params["rv_maneuv"] = rv_maneuv
        impact_data = run(run_params, return_config=False)
        cep = get_cep(run_params, impact_data)
        ceps.append(cep)

    assert ceps[0] < ceps[1] < ceps[2], (
        f"Expected increasing CEP with increasing {error_type}, got {ceps}"
    )


def test_rv_maneuver(run_params):
    """
    Verify that turning on rv maneuver decreases miss distance when deflection
    time is zero.
    """
    assert run_params["num_runs"] == 2
    run_params["num_runs"] = 1000
    run_params["rv_maneuv"] = 0
    # run_params["atm_model"] = 1  # Use exponential model with wind perturbations

    # Grav error seems to cause the test to fail...
    run_params["grav_error"] = 1

    impact_data = run(run_params, return_config=False)
    cep1 = get_cep(run_params, impact_data)

    run_params["rv_maneuv"] = 1
    impact_data = run(run_params, return_config=False)
    cep2 = get_cep(run_params, impact_data)

    assert cep1 > cep2, (
        f"Expected CEP with rv_maneuv 0 to be greater than with rv_maneuv 1, "
        f"got {cep1} > {cep2}"
    )


def test_gnss_navigation_decreases_cep(run_params):
    """
    Verify that turning on GNSS navigation decreases miss distance
    """

    run_params["run_type"] = 0
    run_params["num_runs"] = 50
    run_params["rv_maneuv"] = 1
    run_params["atm_model"] = 1  # Use exponential model with wind perturbations
    run_params["acc_scale_stability"] = 1e-6
    run_params["gyro_bias_stability"] = 1e-8
    run_params["gyro_noise"] = 1e-8
    run_params["gnss_nav"] = 0
    run_params["gnss_noise"] = 1e-2

    impact_data = run(run_params, return_config=False)
    cep1 = get_cep(run_params, impact_data)

    run_params["gnss_nav"] = 1
    impact_data = run(run_params, return_config=False)
    cep2 = get_cep(run_params, impact_data)

    assert cep1 > cep2, (
        f"Expected CEP with GNSS navigation off to be greater than with GNSS navigation on, "
        f"got {cep1} > {cep2}"
    )


@pytest.mark.parametrize(
    "gnss_noise",
    [
        ([0, 1e-2, 1e1]),
    ],
)
def test_gnss_noise_increases_cep(run_params, gnss_noise):
    """
    Verify that turning on/increasing GNSS noise increases miss distance
    """
    run_params["run_type"] = 0
    run_params["num_runs"] = 50
    run_params["rv_maneuv"] = 1
    run_params["atm_model"] = 1  # Use exponential model with wind perturbations
    run_params["acc_scale_stability"] = 1e-6
    run_params["gyro_bias_stability"] = 1e-8
    run_params["gyro_noise"] = 1e-8
    run_params["gnss_nav"] = 1

    ceps = []
    for noise in gnss_noise:
        run_params["gnss_noise"] = noise
        impact_data = run(run_params, return_config=False)
        cep = get_cep(run_params, impact_data)
        ceps.append(cep)

    assert ceps[0] < ceps[1] < ceps[2], (
        f"Expected increasing CEP with increasing GNSS noise, got {ceps}"
    )
