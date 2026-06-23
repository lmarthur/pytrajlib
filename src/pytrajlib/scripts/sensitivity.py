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


SCALE_FACTORS = np.logspace(-1, 1, 7)
SENSITIVITY_SPECS = (
    {
        "name": "initial_pos_error",
        "label": "Initial position error",
        "sweep_kind": "scale",
    },
    {
        "name": "initial_vel_error",
        "label": "Initial velocity error",
        "sweep_kind": "scale",
    },
    {
        "name": "initial_angle_error",
        "label": "Initial angle error",
        "sweep_kind": "scale",
    },
    {
        "name": "acc_scale_stability",
        "label": "Accelerometer scale stability",
        "sweep_kind": "scale",
    },
    {
        "name": "gyro_bias_stability",
        "label": "Gyro bias stability",
        "sweep_kind": "scale",
    },
    {"name": "gyro_noise", "label": "Gyroscope noise", "sweep_kind": "scale"},
    {"name": "gnss_noise", "label": "GNSS noise", "sweep_kind": "scale"},
    {"name": "grav_error", "label": "Gravity perturbation", "sweep_kind": "binary"},
    {
        "name": "actuator_resolution",
        "label": "Actuator resolution",
        "sweep_kind": "scale",
    },
    {"name": "gearing_ratio", "label": "Gearing ratio", "sweep_kind": "scale"},
    {
        "name": "time_step_boost",
        "label": "Boost-phase time step",
        "sweep_kind": "scale",
    },
    {
        "name": "time_step_lambert",
        "label": "Lambert maneuver time step",
        "sweep_kind": "scale",
    },
    {
        "name": "time_step_midcourse",
        "label": "Midcourse time step",
        "sweep_kind": "scale",
    },
    {
        "name": "time_step_reentry",
        "label": "Reentry time step",
        "sweep_kind": "scale",
    },
    {
        "name": "cd_error_factor",
        "label": "Drag coefficient error factor",
        "sweep_kind": "scale",
    },
    {
        "name": "cl_error_factor",
        "label": "Lift coefficient error factor",
        "sweep_kind": "scale",
    },
    {
        "name": "cmq_error_factor",
        "label": "Pitch damping moment coefficient error factor",
        "sweep_kind": "scale",
    },
    {
        "name": "cm_error_factor",
        "label": "Pitching moment coefficient error factor",
        "sweep_kind": "scale",
    },
    {
        "name": "cm_delta_error_factor",
        "label": "Flap deflection moment coefficient error factor",
        "sweep_kind": "scale",
    },
    {
        "name": "roll_gyro_error_factor",
        "label": "Roll gyro error factor",
        "sweep_kind": "scale",
    },
)

BINARY_SENSITIVITY_PARAMS = {
    spec["name"] for spec in SENSITIVITY_SPECS if spec["sweep_kind"] == "binary"
}


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
    base_config: dict, parameter_name: str, parameter_value: float
) -> dict:
    case_config = deepcopy(base_config)
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
        }
    )

    if parameter_name in BINARY_SENSITIVITY_PARAMS:
        case_config[parameter_name] = int(parameter_value)
    else:
        case_config[parameter_name] = float(parameter_value)

    if parameter_name == "roll_gyro_error_factor":
        case_config["gyro_bias_stability"] = base_config["gyro_bias_stability"]
        case_config["gyro_noise"] = base_config["gyro_noise"]

    return case_config


def build_sweep_values(spec: dict, base_config: dict) -> tuple[np.ndarray, np.ndarray]:
    if spec["sweep_kind"] == "binary":
        return np.array([0.0, 1.0]), np.array([0.0, 1.0])

    baseline_value = float(base_config.get(spec["name"], 0.0))
    if baseline_value == 0.0:
        raise SystemExit(
            f"{spec['name']} has a zero baseline in the selected config, so a log sweep is not meaningful."
        )

    return SCALE_FACTORS, baseline_value * SCALE_FACTORS


def run_case(
    base_config: dict,
    parameter_name: str,
    parameter_value: float,
    num_runs: int,
    num_processes: int,
) -> dict:
    case_config = make_case_config(base_config, parameter_name, parameter_value)
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
        "cep": float(np.quantile(miss_distance, 0.5)),
        "mean_miss_distance": float(np.mean(miss_distance)),
        "std_miss_distance": float(np.std(miss_distance)),
    }


def sweep_parameter(
    base_config: dict, spec: dict, num_runs: int, num_processes: int
) -> pd.DataFrame:
    factor_values, actual_values = build_sweep_values(spec, base_config)
    rows = []

    for factor, actual_value in zip(factor_values, actual_values):
        run_result = run_case(
            base_config, spec["name"], actual_value, num_runs, num_processes
        )
        rows.append(
            {
                "parameter": spec["name"],
                "label": spec["label"],
                "sweep_kind": spec["sweep_kind"],
                "factor": float(factor),
                "value": float(actual_value),
                "baseline": float(base_config.get(spec["name"], 0.0)),
                **run_result,
            }
        )
        print(
            f"{spec['name']} factor={factor:.6g} value={actual_value:.6g} cep={run_result['cep']:.6f}"
        )

    return pd.DataFrame(rows)


def plot_combined(results: pd.DataFrame, output_dir: Path) -> tuple[Path, Path]:
    scale_results = results[results["sweep_kind"] == "scale"].copy()
    fig, ax = plt.subplots(figsize=(7.2, 5.0))

    colors = plt.cm.viridis(np.linspace(0, 1, len(scale_results["parameter"].unique())))
    for color, parameter_name in zip(colors, scale_results["parameter"].unique()):
        subset = scale_results[
            scale_results["parameter"] == parameter_name
        ].sort_values("factor")
        ax.errorbar(
            subset["factor"],
            subset["cep"],
            yerr=subset["std_miss_distance"],
            marker="o",
            linewidth=1.4,
            markersize=4,
            capsize=2.5,
            color=color,
            label=pretty_parameter_name(parameter_name),
        )

    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel("Estimated parameter scale factor (E)")
    ax.set_ylabel("CEP (m)")
    ax.set_title("CEP sensitivity to error parameters")
    ax.grid(True, linestyle=":", linewidth=0.7, alpha=0.6)
    ax.legend(frameon=False, fontsize=8, ncols=2)
    fig.tight_layout()

    pdf_path = output_dir / "sensitivity_combined.pdf"
    png_path = output_dir / "sensitivity_combined.png"
    fig.savefig(pdf_path, bbox_inches="tight")
    fig.savefig(png_path, dpi=250, bbox_inches="tight")
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

    for axis, parameter_name in zip(axes_flat, ordered_parameters):
        subset = results[results["parameter"] == parameter_name].sort_values("factor")
        spec = next(
            spec for spec in SENSITIVITY_SPECS if spec["name"] == parameter_name
        )

        x_values = (
            subset["factor"] if spec["sweep_kind"] == "scale" else subset["value"]
        )
        axis.errorbar(
            x_values,
            subset["cep"],
            yerr=subset["std_miss_distance"],
            marker="o",
            linewidth=1.4,
            markersize=4,
            capsize=2.5,
            color="#1f77b4",
        )

        if spec["sweep_kind"] == "scale":
            axis.set_xscale("log")
            axis.set_xlabel("Scale factor")
        else:
            axis.set_xticks([0, 1], ["off", "on"])
            axis.set_xlabel(f"{spec['label']} flag")

        axis.set_yscale("log")
        axis.set_ylabel("CEP (m)")
        axis.set_title(spec["label"])
        axis.grid(True, linestyle=":", linewidth=0.7, alpha=0.6)

    for axis in axes_flat[n_panels:]:
        axis.axis("off")

    fig.suptitle("Sensitivity of CEP to error sources", y=1.01, fontsize=12)
    fig.tight_layout()

    pdf_path = output_dir / "sensitivity_panels.pdf"
    png_path = output_dir / "sensitivity_panels.png"
    fig.savefig(pdf_path, bbox_inches="tight")
    fig.savefig(png_path, dpi=250, bbox_inches="tight")
    plt.close(fig)
    return pdf_path, png_path


def run_sensitivity(
    base_config: dict | None = None,
    output_dir: Path | None = None,
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
            spec["sweep_kind"] == "scale"
            and float(base_config.get(spec["name"], 0.0)) == 0.0
        ):
            print(f"Skipping {spec['name']}: baseline is zero in the selected config.")
            continue
        frames.append(sweep_parameter(config_for_runs, spec, num_runs, num_processes))

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
