"""
Plotting utilities for trajectory and impact analysis.
"""

from pathlib import Path
from typing import Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import scipy.stats as stats

from pytrajlib.utils import get_local_impact

# Matplotlib configuration
PLOT_PARAMS = {
    "axes.labelsize": 10,
    "font.size": 10,
    "font.family": "serif",
    "legend.fontsize": 10,
    "xtick.labelsize": 10,
    "ytick.labelsize": 10,
}


def _setup_pyplot():
    """Configure matplotlib parameters."""
    plt.rcParams.update(PLOT_PARAMS)


def _save_figure(fig_path: Optional[Path], fig_name: str) -> None:
    """Save figure if path is provided."""
    if fig_path is not None:
        output_path = fig_path / f"{fig_name}"
        fig = plt.gcf()
        fig.savefig(output_path, dpi=150, bbox_inches="tight")
        print(f"Saved: {output_path}")


def create_impact_plot(
    impact_df: pd.DataFrame,
    save_path: Optional[Path] = None,
    aimpoint: Optional[Tuple[float, float, float]] = None,
) -> dict:
    """
    Generate impact scatter plot and miss distance histogram.

    Args:
        impact_df: DataFrame with columns [t, x, y, z] in ECEF coordinates
        save_path: Optional path to save figures
        aimpoint: Tuple of (x_aim, y_aim, z_aim) in ECEF coordinates

    Returns:
        Dictionary with computed statistics (cep, miss_distance, etc.)
    """
    _setup_pyplot()

    impact_x_local, impact_y_local = get_local_impact(impact_df, aimpoint)

    # Calculate statistics
    miss_distance = np.sqrt(impact_x_local**2 + impact_y_local**2)
    cep = np.percentile(miss_distance, 50)
    plotrange = 4 * cep

    # Fit Nakagami distribution
    x_fit = np.linspace(0, 5 * cep, 100)
    shape, loc, scale = stats.nakagami.fit(miss_distance, floc=0)
    nakagami_pdf = stats.nakagami.pdf(x_fit, shape, loc, scale)

    # Create figure with gridspec
    fig = plt.figure(figsize=(6, 7))
    gs = fig.add_gridspec(2, 1, height_ratios=(6, 1), hspace=0.18)
    ax_scatter = fig.add_subplot(gs[0])
    ax_hist = fig.add_subplot(gs[1])

    # Scatter plot
    ax_scatter.scatter(
        impact_x_local,
        impact_y_local,
        c="grey",
        marker="x",
        s=20,
        alpha=0.5,
        linewidths=1,
    )

    # CEP circle
    t = np.linspace(0, 2 * np.pi, 400)
    ax_scatter.plot(cep * np.cos(t), cep * np.sin(t), "k--", linewidth=1.5, label="CEP")

    ax_scatter.set_xlim(-plotrange, plotrange)
    ax_scatter.set_ylim(-plotrange, plotrange)
    ax_scatter.set_aspect("equal")
    ax_scatter.set_xlabel("Downrange (m)")
    ax_scatter.set_ylabel("Crossrange (m)")
    ax_scatter.legend(frameon=False)
    ax_scatter.text(
        -0.6 * plotrange,
        0.8 * plotrange,
        f"N = {len(miss_distance)}\nCEP = {cep:.1f} m",
        fontsize=10,
        verticalalignment="top",
        horizontalalignment="center",
    )

    # Histogram
    bins = 50
    ax_hist.hist(
        miss_distance,
        bins=bins,
        range=(0, 5 * cep),
        color="grey",
        edgecolor="black",
        alpha=0.7,
        histtype="stepfilled",
    )

    # Renormalize PDF
    nakagami_pdf_norm = nakagami_pdf * len(miss_distance) * 5 * cep / bins

    ax_hist.plot(
        x_fit,
        nakagami_pdf_norm,
        "k-",
        linewidth=1.5,
        label="Nakagami(" + str(round(shape, 2)) + ", " + str(round(scale, 2)) + ")",
    )
    ax_hist.axvline(x=cep, color="k", linestyle="--", linewidth=1.5)

    # Style histogram
    for spine in ["top", "right", "left"]:
        ax_hist.spines[spine].set_visible(False)
    ax_hist.yaxis.set_visible(False)
    ax_hist.set_xlabel("Miss Distance (m)")
    ax_hist.legend(frameon=False, loc="upper right")

    if save_path:
        _save_figure(Path(save_path), "impact_plot.png")
    else:
        plt.show()

    plt.close()

    return {
        "cep": cep,
        "miss_distance": miss_distance,
        "shape": shape,
        "scale": scale,
    }


def create_traj_plots(
    trajectory_df: pd.DataFrame,
    save_path: Optional[Path] = None,
    aimpoint: Optional[Tuple[float, float, float]] = None,
    guidance_df: Optional[pd.DataFrame] = None,
) -> None:
    """
    Generate trajectory analysis plots.

    Args:
        trajectory_df: DataFrame with trajectory data
        save_path: Optional path to save figures
        aimpoint: Optional aim point tuple of (x_aim, y_aim, z_aim) in ECEF
    """
    _setup_pyplot()

    # Normalize CSV headers to avoid misses from trailing spaces.
    trajectory_df = trajectory_df.copy()
    trajectory_df.columns = trajectory_df.columns.str.strip()

    # Speed up plot generation by subsampling
    trajectory_df = trajectory_df[::10]
    # Extract columns
    t = trajectory_df["t"].values
    x, y, z = (
        trajectory_df["x"].values,
        trajectory_df["y"].values,
        trajectory_df["z"].values,
    )
    vx, vy, vz = (
        trajectory_df["vx"].values,
        trajectory_df["vy"].values,
        trajectory_df["vz"].values,
    )
    est_x, est_y, est_z = (
        trajectory_df["est_x"].values,
        trajectory_df["est_y"].values,
        trajectory_df["est_z"].values,
    )
    est_vx, est_vy, est_vz = (
        trajectory_df["est_vx"].values,
        trajectory_df["est_vy"].values,
        trajectory_df["est_vz"].values,
    )
    current_mass = trajectory_df["current_mass"].values

    # Extract flap-pair deflections
    delta_1 = trajectory_df["true_delta_1"].values
    delta_2 = trajectory_df["true_delta_2"].values
    desired_delta_1 = None
    desired_delta_2 = None
    desired_aoa_deg = None

    has_quaternion = {
        "true_q_w",
        "true_q_x",
        "true_q_y",
        "true_q_z",
        "est_q_w",
        "est_q_x",
        "est_q_y",
        "est_q_z",
    }.issubset(trajectory_df.columns)
    if has_quaternion:
        true_q_w = trajectory_df["true_q_w"].values
        true_q_x = trajectory_df["true_q_x"].values
        true_q_y = trajectory_df["true_q_y"].values
        true_q_z = trajectory_df["true_q_z"].values
        est_q_w = trajectory_df["est_q_w"].values
        est_q_x = trajectory_df["est_q_x"].values
        est_q_y = trajectory_df["est_q_y"].values
        est_q_z = trajectory_df["est_q_z"].values

    has_omega_body = {
        "true_omega_B_1",
        "true_omega_B_2",
        "true_omega_B_3",
        "est_omega_B_1",
        "est_omega_B_2",
        "est_omega_B_3",
    }.issubset(trajectory_df.columns)
    if has_omega_body:
        true_omega_B_1 = trajectory_df["true_omega_B_1"].values
        true_omega_B_2 = trajectory_df["true_omega_B_2"].values
        true_omega_B_3 = trajectory_df["true_omega_B_3"].values
        est_omega_B_1 = trajectory_df["est_omega_B_1"].values
        est_omega_B_2 = trajectory_df["est_omega_B_2"].values
        est_omega_B_3 = trajectory_df["est_omega_B_3"].values

    has_wind_components = {"u1", "u2", "u3"}.issubset(trajectory_df.columns)
    if has_wind_components:
        u1 = trajectory_df["u1"].values
        u2 = trajectory_df["u2"].values
        u3 = trajectory_df["u3"].values
        aoa_alpha_deg, aoa_azimuth_deg = _compute_aoa_angles(u1, u2, u3)

    # Calculate altitude
    altitude = np.sqrt(x**2 + y**2 + z**2) - 6.371e6
    est_altitude = np.sqrt(est_x**2 + est_y**2 + est_z**2) - 6.371e6

    # Define phases
    boost_mask = t <= 188
    midcourse_mask = (t > 188) & (altitude >= 1e5)
    reentry_mask = (t > 188) & (altitude < 1e5)

    if guidance_df is not None:
        guidance_df = guidance_df.copy()
        guidance_df.columns = guidance_df.columns.str.strip()
        required_columns = {"t", "desired_delta_1", "desired_delta_2"}
        if required_columns.issubset(guidance_df.columns):
            t_reentry = t[reentry_mask]
            if t_reentry.size > 0:
                t0 = float(t_reentry.min())
                t1 = float(t_reentry.max())
                guidance_df = guidance_df[
                    (guidance_df["t"] >= t0) & (guidance_df["t"] <= t1)
                ]
                guidance_df = guidance_df[::10]
                desired_aoa_raw = (
                    guidance_df["desired_aoa_deg"].values
                    if "desired_aoa_deg" in guidance_df.columns
                    else None
                )
                desired_delta_1_raw = guidance_df["desired_delta_1"].values
                desired_delta_2_raw = guidance_df["desired_delta_2"].values
                if desired_delta_1_raw.size > 0:
                    if desired_delta_1_raw.size == t_reentry.size:
                        desired_aoa_deg = desired_aoa_raw
                        desired_delta_1 = desired_delta_1_raw
                        desired_delta_2 = desired_delta_2_raw
                        pass
                    else:
                        sample_idx = np.linspace(
                            0,
                            desired_delta_1_raw.size - 1,
                            t_reentry.size,
                        ).astype(int)
                        if desired_aoa_raw is not None:
                            desired_aoa_deg = desired_aoa_raw[sample_idx]
                        desired_delta_1 = desired_delta_1_raw[sample_idx]
                        desired_delta_2 = desired_delta_2_raw[sample_idx]
                        pass

    plots = {
        "position": lambda: _plot_position(t, x, y, z, est_x, est_y, est_z, save_path),
        "position_error": lambda: _plot_position_error(
            t, x, y, z, est_x, est_y, est_z, save_path
        ),
        "position_phases": lambda: _plot_position_phases(
            t,
            x,
            y,
            z,
            est_x,
            est_y,
            est_z,
            boost_mask,
            midcourse_mask,
            reentry_mask,
            save_path,
        ),
        "altitude": lambda: _plot_altitude(t, altitude, est_altitude, save_path),
        "altitude_error": lambda: _plot_altitude_error(
            t, altitude, est_altitude, save_path
        ),
        "mass": lambda: _plot_mass(t, current_mass, save_path),
        "velocity": lambda: _plot_velocity_phases(
            t,
            vx,
            vy,
            vz,
            est_vx,
            est_vy,
            est_vz,
            boost_mask,
            midcourse_mask,
            reentry_mask,
            save_path,
        ),
        "velocity_error": lambda: _plot_velocity_error_phases(
            t,
            vx,
            vy,
            vz,
            est_vx,
            est_vy,
            est_vz,
            boost_mask,
            midcourse_mask,
            reentry_mask,
            save_path,
        ),
        "flap_deltas": lambda: _plot_flap_deltas(
            t,
            delta_1,
            delta_2,
            desired_delta_1,
            desired_delta_2,
            reentry_mask,
            save_path,
        ),
        "flap_vs_altitude": lambda: _plot_flap_vs_altitude(
            altitude,
            delta_1,
            delta_2,
            desired_delta_1,
            desired_delta_2,
            reentry_mask,
            save_path,
        ),
        "orbit": lambda: _plot_orbit(x, y, est_x, est_y, save_path, aimpoint),
    }

    if has_quaternion:
        plots["quaternion"] = lambda: _plot_quaternion(
            t,
            boost_mask,
            midcourse_mask,
            reentry_mask,
            true_q_w,
            true_q_x,
            true_q_y,
            true_q_z,
            est_q_w,
            est_q_x,
            est_q_y,
            est_q_z,
            save_path,
        )
    else:
        print("Skipping quaternion plot (true_q_* / est_q_* not found).")

    if has_omega_body:
        plots["omega_body"] = lambda: _plot_omega_body_components(
            t,
            reentry_mask,
            true_omega_B_1,
            true_omega_B_2,
            true_omega_B_3,
            est_omega_B_1,
            est_omega_B_2,
            est_omega_B_3,
            save_path,
        )
    else:
        print("Skipping omega_body plot (true_omega_B_* / est_omega_B_* not found).")

    if has_wind_components:
        plots["aoa_components"] = lambda: _plot_aoa_components(
            t,
            aoa_alpha_deg,
            aoa_azimuth_deg,
            desired_aoa_deg,
            reentry_mask,
            save_path,
        )
        plots["aoa_vs_altitude"] = lambda: _plot_aoa_vs_altitude(
            altitude, aoa_alpha_deg, desired_aoa_deg, reentry_mask, save_path
        )
        plots["wind_components"] = lambda: _plot_rel_wind_components(
            t, reentry_mask, u1, u2, u3, save_path
        )
    else:
        print("Skipping aoa_components plot (u1/u2/u3 not found).")

    for plot_name, plot_func in plots.items():
        print(f"Generating {plot_name}...")
        plot_func()

    # If reentry guidance data was provided, plot it using the reentry time window
    if guidance_df is not None:
        try:
            # Compute reentry time window from trajectory reentry mask (aligned to trajectory_df)
            t_reentry = t[reentry_mask]
            if t_reentry.size > 0:
                reentry_window = (float(t_reentry.min()), float(t_reentry.max()))
                guidance_reentry_df = guidance_df.copy()
                guidance_reentry_df.columns = guidance_reentry_df.columns.str.strip()
                if "t" in guidance_reentry_df.columns:
                    guidance_reentry_df = guidance_reentry_df[
                        (guidance_reentry_df["t"] >= reentry_window[0])
                        & (guidance_reentry_df["t"] <= reentry_window[1])
                    ]
                plot_reentry_guidance(
                    guidance_df, save_path=save_path, reentry_window=reentry_window
                )
                guidance_df = guidance_reentry_df
            else:
                print(
                    "No reentry interval found in trajectory; skipping reentry guidance plot."
                )
        except Exception as e:
            print(f"Failed to plot reentry guidance: {e}")


def _plot_flap_vs_altitude(
    altitude,
    delta_1,
    delta_2,
    desired_delta_1,
    desired_delta_2,
    reentry_mask,
    save_path,
):
    """Plot flap pair deflections vs altitude during reentry."""
    alt_reentry = altitude[reentry_mask] / 1000.0
    delta_1_reentry = np.degrees(delta_1[reentry_mask])
    delta_2_reentry = np.degrees(delta_2[reentry_mask])
    desired_delta_1_reentry = (
        np.degrees(desired_delta_1) if desired_delta_1 is not None else None
    )
    desired_delta_2_reentry = (
        np.degrees(desired_delta_2) if desired_delta_2 is not None else None
    )

    if alt_reentry.size == 0:
        print("No reentry data for flap deflection vs altitude; skipping.")
        return

    sort_idx = np.argsort(alt_reentry)

    fig, axes = plt.subplots(2, 1, figsize=(8, 6))
    axes[0].plot(
        alt_reentry[sort_idx],
        delta_1_reentry[sort_idx],
        linewidth=2,
        color="C0",
        label="Delta 1",
    )
    axes[0].plot(
        alt_reentry[sort_idx],
        delta_2_reentry[sort_idx],
        linewidth=2,
        color="C1",
        label="Delta 2",
    )
    if desired_delta_1_reentry is not None:
        axes[0].plot(
            alt_reentry[sort_idx],
            desired_delta_1_reentry[sort_idx],
            linewidth=2,
            color="C0",
            linestyle="--",
            label="Desired Delta 1",
        )
    if desired_delta_2_reentry is not None:
        axes[0].plot(
            alt_reentry[sort_idx],
            desired_delta_2_reentry[sort_idx],
            linewidth=2,
            color="C1",
            linestyle="--",
            label="Desired Delta 2",
        )

    axes[0].set_ylabel("Deflection (deg)")
    axes[0].set_title("Flap Deflection vs Altitude (Reentry)")
    axes[0].grid(alpha=0.3)
    axes[0].legend()
    axes[0].set_ylim(-11, 11)

    alt_sorted = alt_reentry[sort_idx]
    low_mask_sorted = alt_sorted < 1.0

    axes[1].plot(
        alt_sorted[low_mask_sorted],
        delta_1_reentry[sort_idx][low_mask_sorted],
        linewidth=2,
        color="C0",
        label="Delta 1",
    )
    axes[1].plot(
        alt_sorted[low_mask_sorted],
        delta_2_reentry[sort_idx][low_mask_sorted],
        linewidth=2,
        color="C1",
        label="Delta 2",
    )
    if desired_delta_1_reentry is not None:
        desired_delta_1_sorted = desired_delta_1_reentry[sort_idx]
        axes[1].plot(
            alt_sorted[low_mask_sorted],
            desired_delta_1_sorted[low_mask_sorted],
            linewidth=2,
            color="C0",
            linestyle="--",
            label="Desired Delta 1",
        )
    if desired_delta_2_reentry is not None:
        desired_delta_2_sorted = desired_delta_2_reentry[sort_idx]
        axes[1].plot(
            alt_sorted[low_mask_sorted],
            desired_delta_2_sorted[low_mask_sorted],
            linewidth=2,
            color="C1",
            linestyle="--",
            label="Desired Delta 2",
        )

    axes[1].set_xlabel("Altitude (km)")
    axes[1].set_ylabel("Deflection (deg)")
    axes[1].set_title("Flap Deflection vs Altitude (Reentry < 1km)")
    axes[1].grid(alpha=0.3)
    axes[1].legend()
    axes[1].set_ylim(-11, 11)

    if save_path:
        _save_figure(Path(save_path), "flap_vs_altitude.png")
    else:
        plt.show()
    plt.tight_layout()
    plt.close()


def plot_reentry_guidance(
    guidance_df: pd.DataFrame,
    save_path: Optional[Path] = None,
    reentry_window: Optional[Tuple[float, float]] = None,
) -> None:
    """Plot commanded and estimated acceleration vectors during reentry.

    `reentry_window` is a simple (t_start, t_end) tuple that specifies the
    time interval to plot. This keeps alignment simple: the caller computes
    the reentry interval from the trajectory data and `plot_reentry_guidance`
    filters `guidance_df` by `t`.
    """
    _setup_pyplot()

    guidance_df = guidance_df.copy()
    guidance_df.columns = guidance_df.columns.str.strip()

    if "t" not in guidance_df.columns:
        print("plot_reentry_guidance: 't' column missing; skipping plot.")
        return

    t0, t1 = reentry_window
    guidance_df = guidance_df[(guidance_df["t"] >= t0) & (guidance_df["t"] <= t1)]
    if guidance_df.empty:
        print("No reentry guidance data in the specified window; skipping plot.")
        return

    # Subsample for plotting
    guidance_df = guidance_df[::10]

    t = guidance_df["t"].values
    a_cmd_x = guidance_df["a_cmd_x"].values
    a_cmd_y = guidance_df["a_cmd_y"].values
    a_cmd_z = guidance_df["a_cmd_z"].values
    a_est_x = guidance_df["a_total_est_x"].values
    a_est_y = guidance_df["a_total_est_y"].values
    a_est_z = guidance_df["a_total_est_z"].values

    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)

    component_data = [
        (axes[0], "x", a_est_x, a_cmd_x, "C0"),
        (axes[1], "y", a_est_y, a_cmd_y, "C1"),
        (axes[2], "z", a_est_z, a_cmd_z, "C2"),
    ]
    for ax, component, a_est, a_cmd, color in component_data:
        ax.plot(
            t, a_est, label=f"a_est {component}", linewidth=2, alpha=0.5, color=color
        )
        ax.plot(
            t,
            a_cmd,
            label=f"a_cmd {component}",
            linewidth=2,
            alpha=0.5,
            linestyle="--",
            color=color,
        )
        ax.set_ylabel("Acceleration (m/s²)")
        ax.set_title(f"Reentry Guidance {component.upper()} Component")
        ax.legend(frameon=False)
        ax.grid(alpha=0.3)
        ax.set_ylim((-100, 100))

    axes[-1].set_xlabel("Time (s)")

    plt.tight_layout()

    if save_path:
        _save_figure(Path(save_path), "reentry_guidance.png")
    else:
        plt.show()
    plt.close()


def _plot_position(t, x, y, z, est_x, est_y, est_z, save_path):
    """Position vs time."""
    plt.figure(figsize=(10, 6))
    (line_x,) = plt.plot(t, x, label="x", linewidth=2)
    (line_y,) = plt.plot(t, y, label="y", linewidth=2)
    (line_z,) = plt.plot(t, z, label="z", linewidth=2)
    plt.plot(
        t,
        est_x,
        label="est x",
        linewidth=2,
        linestyle="--",
        color=line_x.get_color(),
    )
    plt.plot(
        t,
        est_y,
        label="est y",
        linewidth=2,
        linestyle="--",
        color=line_y.get_color(),
    )
    plt.plot(
        t,
        est_z,
        label="est z",
        linewidth=2,
        linestyle="--",
        color=line_z.get_color(),
    )
    plt.xlabel("Time (s)")
    plt.ylabel("Position (m)")
    plt.title("Position vs Time")
    plt.legend()
    plt.grid(alpha=0.3)
    if save_path:
        _save_figure(Path(save_path), "position.png")
    else:
        plt.show()
    plt.close()


def _plot_position_error(t, x, y, z, est_x, est_y, est_z, save_path):
    """Position error vs time."""
    plt.figure(figsize=(10, 6))
    plt.plot(t, x - est_x, label="x error", linewidth=2)
    plt.plot(t, y - est_y, label="y error", linewidth=2)
    plt.plot(t, z - est_z, label="z error", linewidth=2)
    plt.xlabel("Time (s)")
    plt.ylabel("Position Error (m)")
    plt.title("Position Error vs Time")
    plt.legend()
    plt.grid(alpha=0.3)
    if save_path:
        _save_figure(Path(save_path), "position_error.png")
    else:
        plt.show()
    plt.close()


def _plot_position_phases(
    t,
    x,
    y,
    z,
    est_x,
    est_y,
    est_z,
    boost_mask,
    midcourse_mask,
    reentry_mask,
    save_path,
):
    """Position during different flight phases with components on separate axes."""
    fig, axes = plt.subplots(3, 3, figsize=(12, 10))

    phases = [
        ("Boost", boost_mask),
        ("Midcourse", midcourse_mask),
        ("Reentry", reentry_mask),
    ]

    components = [
        (x, est_x, "x"),
        (y, est_y, "y"),
        (z, est_z, "z"),
    ]

    for row, (phase_name, mask) in enumerate(phases):
        for col, (true_comp, est_comp, comp_name) in enumerate(components):
            ax = axes[row, col]
            (line_comp,) = ax.plot(
                t[mask], true_comp[mask], label=comp_name, linewidth=2
            )
            ax.plot(
                t[mask],
                est_comp[mask],
                label=f"est {comp_name}",
                linewidth=2,
                linestyle="--",
                color=line_comp.get_color(),
            )

            # Labels and titles
            if col == 0:
                ax.set_ylabel("Position (m)")
            if row == 0:
                ax.set_title(f"Position {comp_name.upper()} Component")
            if row == 2:
                ax.set_xlabel(f"Time (s) - {phase_name} Phase")
            else:
                ax.set_xlabel("")

            ax.legend()
            ax.grid(alpha=0.3)

    plt.tight_layout()
    if save_path:
        _save_figure(Path(save_path), "position_phases.png")
    else:
        plt.show()
    plt.close()


def _plot_position_est(t, est_x, est_y, est_z, save_path):
    """Estimated position vs time."""
    plt.figure(figsize=(10, 6))
    plt.plot(t, est_x, label="est x", linewidth=2)
    plt.plot(t, est_y, label="est y", linewidth=2)
    plt.plot(t, est_z, label="est z", linewidth=2)
    plt.xlabel("Time (s)")
    plt.ylabel("Estimated Position (m)")
    plt.title("Estimated Position vs Time")
    plt.legend()
    plt.grid(alpha=0.3)
    if save_path:
        _save_figure(Path(save_path), "position_est.png")
    else:
        plt.show()
    plt.close()


def _plot_altitude(t, altitude, est_altitude, save_path):
    """Altitude vs time."""
    plt.figure(figsize=(10, 6))
    (line_alt,) = plt.plot(t, altitude / 1000, linewidth=2, label="true")
    plt.plot(
        t,
        est_altitude / 1000,
        linewidth=2,
        linestyle="--",
        label="est",
        color=line_alt.get_color(),
    )
    plt.xlabel("Time (s)")
    plt.ylabel("Altitude (km)")
    plt.title("Altitude vs Time")
    plt.fill_between(t, altitude / 1000, 0, where=(altitude >= 0), alpha=0.2)
    plt.legend()
    plt.grid(alpha=0.3)
    plt.gca().spines["top"].set_visible(False)
    plt.gca().spines["right"].set_visible(False)
    if save_path:
        _save_figure(Path(save_path), "altitude.png")
    else:
        plt.show()
    plt.close()


def _plot_altitude_error(t, altitude, est_altitude, save_path):
    """Altitude error vs time."""
    plt.figure(figsize=(10, 6))
    plt.plot(t, altitude - est_altitude, linewidth=2)
    plt.xlabel("Time (s)")
    plt.ylabel("Altitude Error (m)")
    plt.title("Altitude Error vs Time")
    plt.grid(alpha=0.3)
    if save_path:
        _save_figure(Path(save_path), "altitude_error.png")
    else:
        plt.show()
    plt.close()


def _plot_mass(t, current_mass, save_path):
    """Mass vs time."""
    plt.figure(figsize=(10, 6))
    plt.plot(t, current_mass, linewidth=2)
    plt.xlabel("Time (s)")
    plt.ylabel("Mass (kg)")
    plt.title("Mass vs Time")
    plt.grid(alpha=0.3)
    if save_path:
        _save_figure(Path(save_path), "mass.png")
    else:
        plt.show()
    plt.close()


def _plot_velocity_phases(
    t,
    vx,
    vy,
    vz,
    est_vx,
    est_vy,
    est_vz,
    boost_mask,
    midcourse_mask,
    reentry_mask,
    save_path,
):
    """Velocity during different flight phases."""
    fig, axes = plt.subplots(3, 1, figsize=(10, 10))

    phases = [
        ("Boost", boost_mask),
        ("Midcourse", midcourse_mask),
        ("Reentry", reentry_mask),
    ]

    for ax, (phase_name, mask) in zip(axes, phases):
        (line_vx,) = ax.plot(t[mask], vx[mask], label="vx", linewidth=2)
        (line_vy,) = ax.plot(t[mask], vy[mask], label="vy", linewidth=2)
        (line_vz,) = ax.plot(t[mask], vz[mask], label="vz", linewidth=2)
        ax.plot(
            t[mask],
            est_vx[mask],
            label="est vx",
            linewidth=2,
            linestyle="--",
            color=line_vx.get_color(),
        )
        ax.plot(
            t[mask],
            est_vy[mask],
            label="est vy",
            linewidth=2,
            linestyle="--",
            color=line_vy.get_color(),
        )
        ax.plot(
            t[mask],
            est_vz[mask],
            label="est vz",
            linewidth=2,
            linestyle="--",
            color=line_vz.get_color(),
        )
        ax.set_ylabel("Velocity (m/s)")
        ax.set_title(f"Velocity ({phase_name} Phase)")
        ax.legend()
        ax.grid(alpha=0.3)

    axes[-1].set_xlabel("Time (s)")
    if save_path:
        _save_figure(Path(save_path), "velocity_phases.png")
    else:
        plt.show()
    plt.close()


def _plot_velocity_error_phases(
    t,
    vx,
    vy,
    vz,
    est_vx,
    est_vy,
    est_vz,
    boost_mask,
    midcourse_mask,
    reentry_mask,
    save_path,
):
    """Velocity error during different flight phases."""
    fig, axes = plt.subplots(3, 1, figsize=(10, 10))

    phases = [
        ("Boost", boost_mask),
        ("Midcourse", midcourse_mask),
        ("Reentry", reentry_mask),
    ]

    for ax, (phase_name, mask) in zip(axes, phases):
        ax.plot(t[mask], (vx - est_vx)[mask], label="vx error", linewidth=2)
        ax.plot(t[mask], (vy - est_vy)[mask], label="vy error", linewidth=2)
        ax.plot(t[mask], (vz - est_vz)[mask], label="vz error", linewidth=2)
        ax.set_ylabel("Velocity Error (m/s)")
        ax.set_title(f"Velocity Error ({phase_name} Phase)")
        ax.legend()
        ax.grid(alpha=0.3)

    axes[-1].set_xlabel("Time (s)")
    if save_path:
        _save_figure(Path(save_path), "velocity_error_phases.png")
    else:
        plt.show()
    plt.close()


def _plot_orbit(x, y, est_x, est_y, save_path, aimpoint=None):
    """Orbital trajectory in x-y plane."""
    plt.figure(figsize=(10, 10))

    earth_radius = 6.371e6
    earth = plt.Circle((0, 0), earth_radius, color="blue", alpha=0.5, label="Earth")
    atmosphere = plt.Circle((0, 0), earth_radius + 200e3, color="lightblue", alpha=0.3)

    plt.gca().add_artist(atmosphere)
    plt.gca().add_artist(earth)

    (line_true,) = plt.plot(x, y, "r-", linewidth=2, label="True Trajectory")
    plt.plot(
        est_x,
        est_y,
        color=line_true.get_color(),
        linewidth=2,
        linestyle="--",
        label="Estimated Trajectory",
    )

    if aimpoint is not None:
        x_aim, y_aim, _ = aimpoint
        plt.scatter(
            x_aim,
            y_aim,
            s=50,
            c="black",
            marker="x",
            label="Aim Point",
            zorder=5,
        )

    plt.xlim(-1.2 * earth_radius, 1.5 * earth_radius)
    plt.ylim(-1.2 * earth_radius, 1.5 * earth_radius)
    plt.axis("off")
    plt.legend(loc="upper right")

    if save_path:
        _save_figure(Path(save_path), "orbit.png")
    else:
        plt.show()
    plt.close()


def _plot_flap_deltas(
    t,
    delta_1,
    delta_2,
    desired_delta_1,
    desired_delta_2,
    reentry_mask,
    save_path,
):
    """True and desired flap-pair deflections over time during reentry."""
    t_reentry = t[reentry_mask]
    delta_1_reentry = delta_1[reentry_mask]
    delta_2_reentry = delta_2[reentry_mask]

    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    axes[0].plot(
        t_reentry,
        np.degrees(delta_1_reentry),
        label="Delta 1",
        linewidth=2,
        alpha=0.75,
    )
    if desired_delta_1 is not None:
        axes[0].plot(
            t_reentry,
            np.degrees(desired_delta_1),
            label="Desired Delta 1",
            linewidth=2,
            linestyle="--",
            alpha=0.75,
        )
    axes[0].set_ylabel("Delta 1 (deg)")
    axes[0].set_title("Flap Pair 1 Deflection vs Time (Reentry)")
    axes[0].legend()
    axes[0].grid(alpha=0.3)
    axes[0].set_ylim(-11, 11)

    axes[1].plot(
        t_reentry,
        np.degrees(delta_2_reentry),
        label="Delta 2",
        linewidth=2,
        alpha=0.75,
    )
    if desired_delta_2 is not None:
        axes[1].plot(
            t_reentry,
            np.degrees(desired_delta_2),
            label="Desired Delta 2",
            linewidth=2,
            linestyle="--",
            alpha=0.75,
        )
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Delta 2 (deg)")
    axes[1].set_title("Flap Pair 2 Deflection vs Time (Reentry)")
    axes[1].legend()
    axes[1].grid(alpha=0.3)
    axes[1].set_ylim(-11, 11)

    if save_path:
        _save_figure(Path(save_path), "flap_deltas.png")
    else:
        plt.show()
    plt.close()


def _compute_aoa_angles(u1, u2, u3):
    """Compute AoA alpha=acos(u3) and azimuth chi=atan2(u2, u1) in degrees."""
    # Clip u3 to avoid invalid acos values from small floating-point drift.
    clipped_u3 = np.clip(u3, -1.0, 1.0)
    aoa_alpha_deg = np.degrees(np.arccos(clipped_u3))
    aoa_azimuth_deg = np.degrees(np.arctan2(u2, u1))
    # aoa_azimuth_deg[np.sqrt(u1**2 + u2**2) < 1e-2] = 0
    return aoa_alpha_deg, aoa_azimuth_deg


def _plot_aoa_components(
    t, aoa_alpha_deg, aoa_azimuth_deg, desired_aoa_deg, reentry_mask, save_path
):
    """Plot AoA alpha and azimuth during reentry phase from alpha=acos(u3), chi=atan2(u2, u1)."""
    # Filter for reentry phase only
    t_reentry = t[reentry_mask]
    aoa_alpha_reentry = aoa_alpha_deg[reentry_mask]
    aoa_azimuth_reentry = aoa_azimuth_deg[reentry_mask]
    desired_aoa_reentry = None
    if desired_aoa_deg is not None:
        desired_aoa_reentry = desired_aoa_deg

    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    axes[0].plot(
        t_reentry, aoa_alpha_reentry, linewidth=2, label=r"$\alpha=\arccos(u_3)$"
    )
    if desired_aoa_reentry is not None:
        axes[0].plot(
            t_reentry,
            desired_aoa_reentry,
            linewidth=2,
            linestyle="--",
            label="Desired AoA",
        )
    axes[0].set_ylabel(r"$\alpha$ (deg)")
    axes[0].set_title("Angle of Attack During Reentry")
    axes[0].grid(alpha=0.3)
    axes[0].legend()
    axes[0].set_ylim(0, 11)

    axes[1].plot(
        t_reentry,
        aoa_azimuth_reentry,
        linewidth=2,
        label=r"$\chi_{\alpha}=\operatorname{atan2}(u_2,u_1)$",
    )
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel(r"$\chi_{\alpha}$ (deg)")
    axes[1].set_title("AoA Azimuth During Reentry")
    axes[1].grid(alpha=0.3)
    axes[1].legend()
    # axes[1].set_ylim(0, 11)

    if save_path:
        _save_figure(Path(save_path), "aoa_components.png")
    else:
        plt.show()
    plt.close()


def _plot_aoa_vs_altitude(
    altitude, aoa_alpha_deg, desired_aoa_deg, reentry_mask, save_path
):
    """Plot AoA alpha vs altitude during reentry phase.

    Altitude is plotted in kilometers on the x-axis; AoA in degrees on the y-axis.
    """
    # Filter for reentry phase only
    alt_reentry = altitude[reentry_mask] / 1000.0
    aoa_reentry = aoa_alpha_deg[reentry_mask]
    desired_aoa_reentry = None
    if desired_aoa_deg is not None:
        desired_aoa_reentry = desired_aoa_deg

    if alt_reentry.size == 0:
        print("No reentry data for AoA vs altitude; skipping.")
        return

    # Sort by altitude so the line plot is monotonic in x
    sort_idx = np.argsort(alt_reentry)

    fig, axes = plt.subplots(2, 1, figsize=(8, 6))
    axes[0].plot(
        alt_reentry[sort_idx],
        aoa_reentry[sort_idx],
        linewidth=2,
        label=r"$\alpha=\arccos(u_3)$",
    )
    if desired_aoa_reentry is not None:
        axes[0].plot(
            alt_reentry[sort_idx],
            desired_aoa_reentry[sort_idx],
            linewidth=2,
            linestyle="--",
            label="Desired AoA",
        )

    # axes[0].set_xticks([])
    axes[0].set_ylabel(r"$\alpha$ (deg)")
    axes[0].set_title("Angle of Attack vs Altitude (Reentry)")
    axes[0].grid(alpha=0.3)
    axes[0].legend()
    axes[0].set_ylim(0, 11)

    # Create a low-altitude view (< 1 km) by sorting and masking the sorted arrays.
    alt_sorted = alt_reentry[sort_idx]
    aoa_sorted = aoa_reentry[sort_idx]
    low_mask_sorted = alt_sorted < 1.0

    axes[1].plot(
        alt_sorted[low_mask_sorted],
        aoa_sorted[low_mask_sorted],
        linewidth=2,
        label=r"$\alpha=\arccos(u_3)$",
    )
    if desired_aoa_reentry is not None:
        desired_sorted = desired_aoa_reentry[sort_idx]
        axes[1].plot(
            alt_sorted[low_mask_sorted],
            desired_sorted[low_mask_sorted],
            linewidth=2,
            linestyle="--",
            label="Desired AoA",
        )

    axes[1].set_xlabel("Altitude (km)")
    axes[1].set_ylabel(r"$\alpha$ (deg)")
    axes[1].set_title("Angle of Attack vs Altitude (Reentry < 1km)")
    axes[1].grid(alpha=0.3)
    axes[1].legend()
    axes[1].set_ylim(0, 20)

    if save_path:
        _save_figure(Path(save_path), "aoa_vs_altitude.png")
    else:
        plt.show()
    plt.tight_layout()
    plt.close()


def _plot_omega_body_components(
    t,
    reentry_mask,
    true_omega_B_1,
    true_omega_B_2,
    true_omega_B_3,
    est_omega_B_1,
    est_omega_B_2,
    est_omega_B_3,
    save_path,
):
    """Plot body angular velocity components during reentry."""
    t_reentry = t[reentry_mask]
    true_omega_reentry = [
        np.degrees(true_omega_B_1[reentry_mask]),
        np.degrees(true_omega_B_2[reentry_mask]),
        np.degrees(true_omega_B_3[reentry_mask]),
    ]
    est_omega_reentry = [
        np.degrees(est_omega_B_1[reentry_mask]),
        np.degrees(est_omega_B_2[reentry_mask]),
        np.degrees(est_omega_B_3[reentry_mask]),
    ]

    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    component_labels = [r"$\omega_{B,1}$", r"$\omega_{B,2}$", r"$\omega_{B,3}$"]

    for idx, (ax, label, true_values, est_values) in enumerate(
        zip(axes, component_labels, true_omega_reentry, est_omega_reentry)
    ):
        ax.plot(t_reentry, true_values, linewidth=2, label="true")
        ax.plot(t_reentry, est_values, linewidth=2, linestyle="--", label="est")
        ax.set_ylabel("deg/s")
        ax.set_title(f"{label} During Reentry")
        ax.grid(alpha=0.3)
        ax.legend(frameon=False)
        # ax.set_ylim(-360, 360)

    axes[-1].set_xlabel("Time (s)")

    fig.suptitle("Body Angular Velocity Components During Reentry")
    plt.tight_layout()

    if save_path:
        _save_figure(Path(save_path), "omega_body.png")
    else:
        plt.show()
    plt.close()


def _plot_rel_wind_components(t, reentry_mask, u1, u2, u3, save_path):
    """Plot u1, u2, u3 components during reentry."""
    t_reentry = t[reentry_mask]
    u1_re = u1[reentry_mask]
    u2_re = u2[reentry_mask]
    u3_re = u3[reentry_mask]

    if t_reentry.size == 0:
        print("No reentry data for wind components; skipping.")
        return

    plt.figure(figsize=(10, 6))
    plt.plot(t_reentry, u1_re, label="u1", alpha=0.5)
    plt.plot(t_reentry, u2_re, label="u2", alpha=0.5)
    plt.plot(t_reentry, u3_re, label="u3", alpha=0.5)
    plt.xlabel("Time (s)")
    plt.ylabel("Component Value")
    plt.title("Relative Wind Components (u1,u2,u3) During Reentry")
    plt.legend()
    plt.grid(alpha=0.3)

    if save_path:
        _save_figure(Path(save_path), "rel_wind_components.png")
    else:
        plt.show()
    plt.close()


def _plot_quaternion(
    t,
    boost_mask,
    midcourse_mask,
    reentry_mask,
    true_q_w,
    true_q_x,
    true_q_y,
    true_q_z,
    est_q_w,
    est_q_x,
    est_q_y,
    est_q_z,
    save_path,
):
    """True and estimated quaternion components vs time, split by phase."""
    fig, axes = plt.subplots(3, 1, figsize=(11, 10), sharey=True)
    components = [
        ("w", true_q_w, est_q_w),
        ("x", true_q_x, est_q_x),
        ("y", true_q_y, est_q_y),
        ("z", true_q_z, est_q_z),
    ]
    phases = [
        ("Boost", boost_mask),
        ("Midcourse", midcourse_mask),
        ("Reentry", reentry_mask),
    ]

    component_colors = ["C0", "C1", "C2", "C3"]

    for ax, (phase_name, mask) in zip(axes, phases):
        for color, (name, true_values, est_values) in zip(component_colors, components):
            ax.plot(
                t[mask],
                true_values[mask],
                color=color,
                linewidth=2,
                label=f"{name} true",
            )
            ax.plot(
                t[mask],
                est_values[mask],
                color=color,
                linewidth=2,
                linestyle="--",
                label=f"{name} est",
            )

        ax.set_title(f"Quaternion Components ({phase_name} Phase)")
        ax.set_ylabel("Value")
        ax.grid(alpha=0.3)
        ax.set_ylim((-1.1, 1.1))

    axes[0].legend(frameon=False, ncol=4, fontsize=9)

    axes[-1].set_xlabel("Time (s)")

    fig.suptitle("True vs Estimated Quaternion Components by Phase")
    plt.tight_layout()

    if save_path:
        _save_figure(Path(save_path), "quaternion.png")
    else:
        plt.show()
    plt.close()
