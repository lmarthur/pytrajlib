import tomllib
from copy import deepcopy
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import scienceplots

import pytrajlib as ptl
from pytrajlib.runtime import _get_default_config

# Avoid unused import warning by asserting scienceplots
assert scienceplots


plt.style.use(["science"])
plt.style.use(["no-latex"])
DEFAULT_SCALE_FACTORS = np.logspace(-1, 1, 7)
TIME_STEP_SCALE_FACTORS = np.logspace(-1, 3, 9)
GNSS_FREQ_SCALE_FACTORS = np.logspace(-3, 1, 9)
RANGE_SCALE_FACTORS = np.linspace(0.5, 1.0, 11)

SENSITIVITY_SPECS = (
    {
        "name": "initial_pos_error",
        "label": "Initial position error",
        "sweep_factors": DEFAULT_SCALE_FACTORS,
    },
    {
        "name": "gnss_freq",
        "label": "GNSS update frequency",
        "sweep_factors": GNSS_FREQ_SCALE_FACTORS,
    },
    {
        "name": "range",
        "label": "Downrange distance",
        "sweep_factors": RANGE_SCALE_FACTORS,
        "optimize_boost": True,
    },
    {
        "name": "burn_time_error",
        "label": "Burn time error",
        "sweep_factors": DEFAULT_SCALE_FACTORS,
    },
    {
        "name": "initial_vel_error",
        "label": "Initial velocity error",
        "sweep_factors": DEFAULT_SCALE_FACTORS,
    },
    {
        "name": "initial_angle_error",
        "label": "Initial angle error",
        "sweep_factors": DEFAULT_SCALE_FACTORS,
    },
    {
        "name": "acc_scale_stability",
        "label": "Accelerometer scale stability",
        "sweep_factors": DEFAULT_SCALE_FACTORS,
    },
    {
        "name": "gyro_bias_stability",
        "label": "Gyro bias stability",
        "sweep_factors": DEFAULT_SCALE_FACTORS,
    },
    {
        "name": "gyro_noise",
        "label": "Gyroscope noise",
        "sweep_factors": DEFAULT_SCALE_FACTORS,
    },
    {
        "name": "gnss_noise",
        "label": "GNSS error",
        "sweep_factors": DEFAULT_SCALE_FACTORS,
    },
    {
        "name": "grav_error",
        "label": "Gravity perturbation",
        "sweep_factors": np.array([0.0, 1.0]),
    },
    {
        "name": "actuator_resolution",
        "label": "Actuator resolution",
        "sweep_factors": DEFAULT_SCALE_FACTORS,
    },
    {
        "name": "time_step_boost",
        "label": "Boost-phase time step",
        "sweep_factors": TIME_STEP_SCALE_FACTORS,
    },
    {
        "name": "time_step_lambert",
        "label": "Lambert maneuver time step",
        "sweep_factors": TIME_STEP_SCALE_FACTORS,
    },
    {
        "name": "time_step_midcourse",
        "label": "Midcourse time step",
        "sweep_factors": TIME_STEP_SCALE_FACTORS,
    },
    {
        "name": "time_step_reentry",
        "label": "Reentry time step",
        "sweep_factors": TIME_STEP_SCALE_FACTORS,
    },
)


def _is_binary_spec(spec: dict) -> bool:
    sweep_values = (
        spec["sweep_factors"] if "sweep_factors" in spec else spec["sweep_values"]
    )
    return np.array_equal(np.asarray(sweep_values), np.array([0.0, 1.0]))


def pretty_parameter_name(parameter_name: str) -> str:
    return parameter_name.replace("_", " ").strip().capitalize()


def load_config(config_path: Path | None) -> dict:
    base_config = _get_default_config().copy()
    if config_path is None:
        return base_config

    with config_path.open("rb") as handle:
        loaded = tomllib.load(handle)

    merged = base_config.copy()
    for section_name in ("RUN", "FLIGHT", "VEHICLE", "ERRORPARAMS"):
        merged.update(loaded.get(section_name, {}))
    return merged


def make_case_config(
    base_config: dict,
    parameter_name: str,
    parameter_value: float,
    is_binary: bool,
    use_zero_baseline: bool,
) -> dict:
    case_config = deepcopy(base_config)
    if use_zero_baseline:
        case_config.update(
            {
                "traj_output": 0,
                "optimize_boost": 0,
                "optimize_reentry": 0,
                "random_seed": 0,
                "initial_pos_error": 0.0,
                "initial_vel_error": 0.0,
                "initial_angle_error": 0.0,
                "acc_scale_stability": 0.0,
                "gyro_bias_stability": 0.0,
                "gyro_noise": 0.0,
                "gnss_noise": 0.0,
                "grav_error": 0,
                "roll_gyro_error_factor": 0,
            }
        )

    if is_binary:
        case_config[parameter_name] = int(parameter_value)
    else:
        case_config[parameter_name] = float(parameter_value)

    if parameter_name == "roll_gyro_error_factor":
        case_config["gyro_bias_stability"] = base_config["gyro_bias_stability"]
        case_config["gyro_noise"] = base_config["gyro_noise"]

    if next((s for s in SENSITIVITY_SPECS if s["name"] == parameter_name), {}).get(
        "optimize_boost"
    ):
        case_config["optimize_boost"] = 1

    return case_config


def build_sweep_values(spec: dict, base_config: dict) -> tuple[np.ndarray, np.ndarray]:
    sweep_factors = (
        spec["sweep_factors"] if "sweep_factors" in spec else spec["sweep_values"]
    )
    if _is_binary_spec(spec):
        return sweep_factors, sweep_factors

    baseline_value = float(base_config.get(spec["name"], 0.0))
    if baseline_value == 0.0:
        raise SystemExit(
            f"{spec['name']} has a zero baseline in the selected config, so a log sweep is not meaningful."
        )

    return sweep_factors, baseline_value * sweep_factors


def run_case(
    base_config: dict,
    parameter_name: str,
    parameter_value: float,
    num_runs: int,
    num_processes: int,
    is_binary: bool,
    use_zero_baseline: bool,
) -> dict:
    case_config = make_case_config(
        base_config, parameter_name, parameter_value, is_binary, use_zero_baseline
    )
    case_config["num_runs"] = num_runs
    case_config.pop("num_processes", None)

    impact_df = ptl.run(
        config=None,
        plot_trajectory=False,
        plot_impact=False,
        num_processes=num_processes,
        **case_config,
    )

    miss_distance = impact_df["miss_distance"].to_numpy()
    return {
        "mean_miss": float(np.mean(miss_distance)),
        "mean_miss_distance": float(np.mean(miss_distance)),
        "std_miss_distance": float(np.std(miss_distance)),
    }


def sweep_parameter(
    base_config: dict,
    spec: dict,
    num_runs: int,
    num_processes: int,
    use_zero_baseline: bool,
) -> pd.DataFrame:
    is_binary = _is_binary_spec(spec)
    factor_values, actual_values = build_sweep_values(spec, base_config)
    rows = []

    for factor, actual_value in zip(factor_values, actual_values):
        run_result = run_case(
            base_config,
            spec["name"],
            actual_value,
            num_runs,
            num_processes,
            is_binary,
            use_zero_baseline,
        )
        rows.append(
            {
                "parameter": spec["name"],
                "label": spec["label"],
                "factor": float(factor),
                "value": float(actual_value),
                "baseline": float(base_config.get(spec["name"], 0.0)),
                **run_result,
            }
        )
        print(
            f"{spec['name']} factor={factor:.6g} value={actual_value:.6g} mean_miss={run_result['mean_miss']:.6f}"
        )

    return pd.DataFrame(rows)


def _get_spec_by_name(parameter_name: str) -> dict:
    """Look up a sensitivity spec by parameter name."""
    return next(spec for spec in SENSITIVITY_SPECS if spec["name"] == parameter_name)


def _save_figure(fig: plt.Figure, stem: str, output_dir: Path) -> tuple[Path, Path]:
    """Save a figure as both PDF and PNG, returning both paths."""
    pdf_path = output_dir / f"{stem}.pdf"
    png_path = output_dir / f"{stem}.png"
    fig.savefig(pdf_path, bbox_inches="tight")
    fig.savefig(png_path, dpi=250, bbox_inches="tight")
    return pdf_path, png_path


def _setup_sensitivity_axis(
    ax: plt.Axes, spec: dict, ylabel: str = "Mean miss distance (m)"
) -> None:
    """Configure an axis for sensitivity plotting based on parameter type."""
    ax.set_yscale("log")
    ax.set_ylabel(ylabel)
    ax.set_title(spec["label"])
    ax.grid(True, linestyle=":", linewidth=0.7, alpha=0.6)

    if _is_binary_spec(spec):
        ax.set_xticks([0, 1], ["off", "on"])
        ax.set_xlabel(f"{spec['label']} flag")
    else:
        ax.set_xscale("log")
        ax.set_xlabel("Scale factor")


def _plot_sensitivity_panel(
    ax: plt.Axes,
    subset: pd.DataFrame,
    spec: dict,
    color: str = "#1f77b4",
    label: str | None = None,
) -> None:
    """Plot sensitivity data (error bars) on the given axis."""
    x_values = subset["value"] if _is_binary_spec(spec) else subset["factor"]
    ax.errorbar(
        x_values,
        subset["mean_miss"],
        yerr=subset["std_miss_distance"],
        marker="o",
        linewidth=1.4,
        markersize=4,
        capsize=2.5,
        color=color,
        label=label,
    )


def plot_combined(results: pd.DataFrame, output_dir: Path) -> tuple[Path, Path]:
    scale_parameters = [
        spec["name"] for spec in SENSITIVITY_SPECS if not _is_binary_spec(spec)
    ]
    scale_results = results[results["parameter"].isin(scale_parameters)].copy()
    fig, ax = plt.subplots(figsize=(7.2, 5.0))

    colors = plt.cm.viridis(np.linspace(0, 1, len(scale_results["parameter"].unique())))
    for color, parameter_name in zip(colors, scale_results["parameter"].unique()):
        subset = scale_results[
            scale_results["parameter"] == parameter_name
        ].sort_values("factor")
        spec = _get_spec_by_name(parameter_name)
        _plot_sensitivity_panel(
            ax, subset, spec, color=color, label=pretty_parameter_name(parameter_name)
        )

    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel("Estimated parameter scale factor (E)")
    ax.set_ylabel("Mean miss distance (m)")
    ax.set_title("Miss distance sensitivity to error parameters")
    ax.grid(True, linestyle=":", linewidth=0.7, alpha=0.6)
    ax.legend(frameon=False, fontsize=8, ncols=2)

    fig.tight_layout()
    pdf_path, png_path = _save_figure(fig, "sensitivity_combined", output_dir)
    plt.close(fig)
    return pdf_path, png_path


def save_panel_plot(
    subset: pd.DataFrame, spec: dict, output_dir: Path
) -> tuple[Path, Path]:
    fig, ax = plt.subplots(figsize=(5.6, 4.2))
    _plot_sensitivity_panel(ax, subset, spec)
    _setup_sensitivity_axis(ax, spec)
    fig.tight_layout()

    panel_stem = f"sensitivity_panel_{spec['name']}"
    pdf_path, png_path = _save_figure(fig, panel_stem, output_dir)
    plt.close(fig)
    return pdf_path, png_path


def plot_panels(results: pd.DataFrame, output_dir: Path) -> tuple[Path, Path]:
    ordered_parameters = [
        spec["name"]
        for spec in SENSITIVITY_SPECS
        if spec["name"] in results["parameter"].unique()
    ]
    n_panels = len(ordered_parameters)
    n_cols = 3
    n_rows = int(np.ceil(n_panels / n_cols))

    fig, axes = plt.subplots(
        n_rows, n_cols, figsize=(12.0, 3.5 * n_rows), squeeze=False
    )
    axes_flat = axes.ravel()

    saved_panel_paths: list[tuple[Path, Path]] = []
    for axis, parameter_name in zip(axes_flat, ordered_parameters):
        subset = results[results["parameter"] == parameter_name].sort_values("factor")
        spec = _get_spec_by_name(parameter_name)

        _plot_sensitivity_panel(axis, subset, spec)
        _setup_sensitivity_axis(axis, spec)

        saved_panel_paths.append(save_panel_plot(subset, spec, output_dir))

    for axis in axes_flat[n_panels:]:
        axis.axis("off")

    fig.suptitle("Sensitivity of miss distance to error sources", y=1.01, fontsize=12)
    fig.tight_layout()

    pdf_path, png_path = _save_figure(fig, "sensitivity_panels", output_dir)
    plt.close(fig)
    return pdf_path, png_path


def run_sensitivity(
    base_config: dict | None = None,
    output_dir: Path | None = None,
    use_zero_baseline: bool = True,
) -> pd.DataFrame:
    if base_config is None:
        base_config = load_config(None)

    output_dir = output_dir or Path()
    output_dir = output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    num_runs = int(base_config.get("num_runs", 30))
    num_processes = int(base_config.get("num_processes", 10))
    config_for_runs = {k: v for k, v in base_config.items() if k != "num_processes"}

    frames = []
    for spec in SENSITIVITY_SPECS:
        if (
            not _is_binary_spec(spec)
            and float(base_config.get(spec["name"], 0.0)) == 0.0
        ):
            print(f"Skipping {spec['name']}: baseline is zero in the selected config.")
            continue
        frames.append(
            sweep_parameter(
                config_for_runs, spec, num_runs, num_processes, use_zero_baseline
            )
        )

    results = pd.concat(frames, ignore_index=True)
    csv_path = output_dir / "sensitivity_results.csv"
    results.to_csv(csv_path, index=False)

    combined_pdf, combined_png = plot_combined(results, output_dir)
    panels_pdf, panels_png = plot_panels(results, output_dir)

    print(f"Saved data to {csv_path}")
    print(
        f"Saved plots to {combined_pdf}, {combined_png}, {panels_pdf}, and {panels_png}"
    )

    return results
