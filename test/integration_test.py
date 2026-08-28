import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import pytest

from pytrajlib.utils import get_local_impact

sys.path.append("./src")
from pytrajlib.main import run
from pytrajlib.runtime import per_run_seeds

CONFIG_PATH = "./test/test.json"

ERROR_FIELDS = (
    "initial_x_error",
    "initial_pos_error",
    "initial_vel_error",
    "initial_angle_error",
    "acc_scale_stability",
    "gyro_bias_stability",
    "gyro_noise",
    "gnss_noise",
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
    """Baseline test JSON has zero defaults for all error parameters."""
    with open(CONFIG_PATH) as f:
        cfg = json.load(f)

    err_cfg = cfg["error"]
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
        ("initial_pos_error", 0, 0.1, (0,)),
        ("initial_vel_error", 1e-5, 1e-3, (0, 2)),
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
            gnss_nav=0,
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


def test_sensitivity_runs(tmp_path):
    """Sensitivity mode should run all configured sweeps with one sample per case."""

    results = run(
        config=CONFIG_PATH,
        sensitivity=True,
        num_runs=1,
        num_processes=1,
        random_seed=0,
        output_dir=str(tmp_path),
    )

    assert results is not None
    assert isinstance(results, pd.DataFrame)
    assert not results.empty


def test_no_impact_correlation():
    """With standard error params, there should be no correlation between downrange and crossrange errors."""
    impact_df, config = run(return_config=True, num_runs=300, random_seed=0)
    impact_x_local, impact_y_local = get_local_impact(
        impact_df, (config["x_aim"], config["y_aim"], config["z_aim"])
    )
    r = np.corrcoef(impact_x_local, impact_y_local)[0][1]

    output_dir = Path("/tmp/pytrajlib/")
    output_dir.mkdir(parents=True, exist_ok=True)
    impact_df.to_csv("/tmp/pytrajlib/correlation-test-impact.csv")
    assert np.abs(r) < 1e-1


def test_per_run_seeds_are_distinct_and_reproducible():
    """`random_seed` seeds the whole batch; each run gets its own derived seed.

    per_run_seeds(1, 100) returns 100 seeds: `1` is the batch seed a
    user sets, and each run is handed a different child seed so it flies a
    different atmosphere. The four assertions below each rule out a specific
    way that derivation can break, all of which fail silently:

      distinct        `return [random_seed] * num_runs` would seed every run
                      identically, so the batch becomes N copies of one flight
                      and CEP collapses to a single value with no spread.
      non-negative    The C side reads a negative seed as "seed from the clock"
                      (trajectory.h), so a negative child would quietly make
                      that one run non-reproducible.
      repeatable      Same batch should give same seeds.
      seed-dependent  A derivation ignoring its argument would give every batch
                      the same seeds no matter what the user set.
    """
    seeds = per_run_seeds(1, 100)

    assert len(set(seeds)) == 100
    assert all(seed >= 0 for seed in seeds)
    assert per_run_seeds(1, 100) == seeds
    assert per_run_seeds(2, 100) != seeds


def test_negative_seed_is_not_reproducible():
    """A negative batch seed means "fresh entropy", so batches differ."""
    assert per_run_seeds(-1, 16) != per_run_seeds(-1, 16)
    assert all(seed >= 0 for seed in per_run_seeds(-1, 16))


def test_same_seed_reproduces_batch_with_independent_runs():
    """random_seed reproduces the batch while its runs stay independent."""
    kwargs = dict(config=CONFIG_PATH, num_runs=8, random_seed=1, gyro_noise=1e-6)
    first = run(**kwargs)
    second = run(**kwargs)
    other = run(**{**kwargs, "random_seed": 2})

    columns = ["x", "y", "z"]
    assert np.allclose(first[columns].to_numpy(), second[columns].to_numpy(), atol=0)
    assert not np.allclose(first[columns].to_numpy(), other[columns].to_numpy())
    # Runs within one batch must not be copies of each other.
    assert len(np.unique(first["x"].to_numpy())) == len(first)


@pytest.mark.parametrize("t_des_final", [2000.0, 3000.0, 4000.0, 6000.0])
def test_requested_flight_time_never_aborts_the_run(t_des_final):
    """Ensure Lambert can handle long flight times."""
    impact_df = run(
        config=CONFIG_PATH,
        num_runs=6,
        t_des_final=t_des_final,
        optimize_boost=0,
        optimize_reentry=0,
    )
    aborted = int((impact_df["burnout_speed"] == 0).sum())
    assert aborted == 0, (
        f"{aborted}/{len(impact_df)} runs never flew at t_des_final={t_des_final}"
    )
