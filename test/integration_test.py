import sys
import tomllib

import numpy as np
import pytest

from pytrajlib.utils import get_local_impact

sys.path.append("./src")
from pytrajlib.main import run

CONFIG_PATH = "./test/test.toml"

ERROR_FIELDS = (
    "initial_x_error",
    "initial_pos_error",
    "initial_vel_error",
    "initial_angle_error",
    "acc_scale_stability",
    "gyro_bias_stability",
    "gyro_noise",
    "gnss_noise",
    "cl_pert",
    "step_acc_mag",
    "step_acc_hgt",
    "step_acc_dur",
)


def _get_cep_from_df(impact_df):
    return float(np.quantile(impact_df["miss_distance"], 0.5))


@pytest.fixture(scope="session")
def base_case():
    """Canonical executed no-error baseline used for comparisons."""
    impact_df = run(config=CONFIG_PATH)
    return {
        "impact_df": impact_df,
        "cep": _get_cep_from_df(impact_df),
    }


def test_read_config():
    """Baseline test TOML has zero defaults for all error parameters."""
    with open(CONFIG_PATH, "rb") as f:
        cfg = tomllib.load(f)

    err_cfg = cfg["ERRORPARAMS"]
    for field in ERROR_FIELDS:
        assert float(err_cfg[field]) == 0.0


def test_zero_error_baseline_has_near_zero_cep(base_case):
    """CEP should stay near zero when all error parameters are off."""
    cep = base_case["cep"]
    assert cep < 1e-3


def test_optimizer_to_zero():
    """Test the optimizer of desired flight time can take the CEP to zero"""
    impact_df = run(
        config=CONFIG_PATH, num_runs=1, num_runs_optimizer=1, optimize_boost=1
    )
    assert _get_cep_from_df(impact_df) < 0.01


def test_zero_error_runs_are_identical():
    """First two runs should be identical when all error parameters are zero."""
    impact_df = run(config=CONFIG_PATH, num_runs=2)
    p0 = impact_df.iloc[0][["x", "y", "z"]].to_numpy()
    p1 = impact_df.iloc[1][["x", "y", "z"]].to_numpy()
    assert np.allclose(p0, p1, atol=1e-6)


@pytest.mark.parametrize("rv_maneuv", [pytest.param(1, marks=pytest.mark.xfail), 2])
def test_maneuv_to_zero(rv_maneuv):
    """Test maneuvering will take cep to zero"""
    # use ballistic drag for perfect maneuvering simulation
    impact_df = run(
        num_runs=1,
        rv_maneuv=rv_maneuv,
        ballistic_drag=rv_maneuv - 1,
        **{e: 0 for e in ERROR_FIELDS},
    )
    assert _get_cep_from_df(impact_df) < 0.1


def test_perfect_boost():
    """Test perfect boost has lower CEP than realistic boost"""
    perfect_df = run(
        config=CONFIG_PATH,
        num_runs=1,
        perfect_boost=1,
        initial_angle_error=1e-6,
        gyro_noise=1e-8,
        gyro_bias_stability=1e-8,
    )
    realistic_df = run(
        config=CONFIG_PATH,
        num_runs=1,
        perfect_boost=0,
        initial_angle_error=1e-6,
        gyro_noise=1e-8,
        gyro_bias_stability=1e-8,
    )

    perfect_cep = _get_cep_from_df(perfect_df)
    realistic_cep = _get_cep_from_df(realistic_df)
    assert perfect_cep < realistic_cep


@pytest.mark.parametrize(
    "error_field,small_value,large_value,maneuv_modes",
    (
        ("atm_model", 0, 2, (0,)),
        ("grav_error", 0, 1, (0,)),
        ("initial_pos_error", 0, 0.1, (0, 1)),
        ("initial_vel_error", 1e-5, 1e-3, (0, 1, 2)),
        ("initial_angle_error", 1e-6, 1e-1, (0,)),
        ("acc_scale_stability", 1e-8, 1e-4, (0, 1, 2)),
        ("gyro_bias_stability", 1e-9, 1e-5, (0, 1, 2)),
        ("gyro_noise", 1e-9, 1e-3, (0, 1, 2)),
    ),
)
def test_error_sensitivity_increases_cep(
    error_field, small_value, large_value, maneuv_modes
):
    """Larger disturbances should increase CEP for each supported maneuv mode."""

    for rv_maneuv in maneuv_modes:
        if rv_maneuv == 0:
            config = CONFIG_PATH
        else:
            config = None
        maneuv_configs = dict(
            ballistic_drag=rv_maneuv - 1,
            num_runs=10,
            **{e: 0 for e in ERROR_FIELDS if e != error_field},
        )
        small = dict()
        small["rv_maneuv"] = rv_maneuv
        small[error_field] = small_value
        cep_small = _get_cep_from_df(run(config=config, **{**small, **maneuv_configs}))

        large = dict()
        large["rv_maneuv"] = rv_maneuv
        large[error_field] = large_value
        cep_large = _get_cep_from_df(run(config=config, **{**large, **maneuv_configs}))

        assert cep_small < cep_large


@pytest.mark.parametrize("rv_maneuv", [1, 2])
def test_maneuverability_supresses_atm_errors(rv_maneuv):
    """Enabling RV maneuverability should supress errors from a realistic atmosphere."""
    params = {
        "atm_model": 2,
        "grav_error": 1,
    }
    cep_no_maneuv = _get_cep_from_df(run(config=CONFIG_PATH, rv_maneuv=0, **params))
    cep_maneuv = _get_cep_from_df(
        run(
            num_runs=1,
            rv_maneuv=rv_maneuv,
            ballistic_drag=rv_maneuv - 1,
            **{e: 0 for e in ERROR_FIELDS if e != "grav_error"},
        )
    )

    assert cep_no_maneuv > cep_maneuv


def test_gnss_navigation_reduces_cep_with_ins_errors():
    """Turning on GNSS nav should reduce CEP when INS error terms are present."""
    params = {
        "atm_model": 2,
        "acc_scale_stability": 1e-6,
        "gyro_bias_stability": 1e-8,
        "gyro_noise": 1e-8,
        "gnss_nav": 0,
        "rv_maneuv": 2,
    }
    cep_no_gnss = _get_cep_from_df(run(config=CONFIG_PATH, **params))

    params["gnss_nav"] = 1
    cep_with_gnss = _get_cep_from_df(run(config=CONFIG_PATH, **params))

    assert cep_no_gnss > cep_with_gnss


@pytest.mark.xfail(reason="Reluctantly xfailing this for now. TODO investigate")
def test_no_impact_correlation():
    """With standard error params, there should be no correlation between downrange and crossrange errors."""
    impact_df, config = run(return_config=True, num_runs=200, random_seed=0)
    impact_x_local, impact_y_local = get_local_impact(
        impact_df, (config["x_aim"], config["y_aim"], config["z_aim"])
    )
    r = np.corrcoef(impact_x_local, impact_y_local)[0][1]
    assert np.abs(r) < 1e-1
