
import pytrajlib as ptl
import pandas as pd
import matplotlib.pyplot as plt
from itertools import product
from multiprocessing import Pool

MAX_PROCESSES = 14

DEFAULT = {        
    "num_runs": 50,
        "optimize_boost": False,
        "traj_output": False,
}

TIME_STEP_PARAMS = (
    "time_step_lambert",
    "time_step_midcourse",
    "time_step_atm",
)

TIME_STEP_VALUES = [1e-4, 1e-3, 1e-2]

NO_ERROR = {**DEFAULT,
            "num_runs": 1,
        "initial_x_error":0.0,
        "initial_pos_error":0.0,
        "initial_vel_error":0.0,
        "initial_angle_error":0.0,
        "acc_scale_stability":0.0,
        "gyro_bias_stability":0.0,
        "gyro_noise":0.0,
        "gnss_noise":0.0,
}

def run_single_param(params):
    impact_df = ptl.run(**params)
    return impact_df


def _run_single_param_for_values(values):
    params = {**DEFAULT}
    params.update(dict(zip(TIME_STEP_PARAMS, values)))
    df = run_single_param(params=params)
    for param_name, value in zip(TIME_STEP_PARAMS, values):
        df[param_name] = value
    return df


def run_matrix(param_values):
    combinations = list(
        product(*(param_values[param_name] for param_name in TIME_STEP_PARAMS))
    )
    print(combinations)
    with Pool(MAX_PROCESSES) as p:
        results = p.map(_run_single_param_for_values, combinations)

    return pd.concat(results, ignore_index=True)


def summarize_matrix(impact_df):
    grouped = (
        impact_df.groupby(list(TIME_STEP_PARAMS))["miss_distance"]
        .median()
        .sort_index()
    )
    return grouped


def plot_scatter_series(grouped, output_path="sensitivity-lines.png"):
    midcourse_values = grouped.index.get_level_values("time_step_midcourse").unique()
    lambert_values = grouped.index.get_level_values("time_step_lambert").unique()
    n_panels = len(midcourse_values)
    fig, axes = plt.subplots(
        1,
        n_panels,
        figsize=(5 * n_panels, 4),
        sharex=True,
        sharey=True,
    )
    if n_panels == 1:
        axes = [axes]

    vmin = grouped.min()
    vmax = grouped.max()
    cmap = plt.get_cmap("tab10")

    for ax, midcourse_value in zip(axes, midcourse_values):
        midcourse_slice = grouped.xs(midcourse_value, level="time_step_midcourse")
        for color_index, lambert_value in enumerate(lambert_values):
            series = midcourse_slice.xs(lambert_value, level="time_step_lambert")
            series = series.sort_index()
            ax.plot(
                series.index,
                series.values,
                color=cmap(color_index % cmap.N),
                linewidth=1.5,
                label=f"Lambert = {lambert_value:g}",
            )
            ax.scatter(
                series.index,
                series.values,
                color=cmap(color_index % cmap.N),
                s=30,
            )

        ax.set_title(f"time_step_midcourse = {midcourse_value:g}")
        ax.set_xlabel("time_step_atm")
        ax.set_ylabel("CEP (m)")
        ax.set_xscale("log")
        ax.grid(True, which="both", linestyle=":", linewidth=0.6, alpha=0.6)

    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(
        handles,
        labels,
        loc="upper center",
        bbox_to_anchor=(0.5, 1.02),
        ncol=min(len(labels), 4),
        frameon=False,
    )
    fig.suptitle("Sensitivity of CEP to time steps", y=1.12)
    fig.tight_layout(rect=[0, 0, 1, 0.9])
    fig.savefig(output_path, dpi=200, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    param_values = {param_name: TIME_STEP_VALUES for param_name in TIME_STEP_PARAMS}
    impact_df = run_matrix(param_values)
    impact_df.to_csv("sensitivity-timestep.csv", index=False)
    # impact_df = pd.read_csv("sensitivity-timestep.csv")
    print(impact_df)
    grouped = summarize_matrix(impact_df)
    print(grouped)
    plot_scatter_series(grouped)