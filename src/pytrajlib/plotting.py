"""
Plotting utilities for trajectory and impact analysis.
"""

import os
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


def setup_pyplot():
    """Configure matplotlib parameters."""
    plt.rcParams.update(PLOT_PARAMS)


def _get_series_or_default(
    trajectory_df: pd.DataFrame, column_name: str, default: float = 0.0
) -> np.ndarray:
    """Return a trajectory column if present, otherwise a constant array."""
    if column_name in trajectory_df.columns:
        return trajectory_df[column_name].values
    return np.full(len(trajectory_df), default)


def _save_figure(fig_path: Optional[Path], fig_name: str) -> None:
    """Save figure if path is provided."""
    if fig_path is not None:
        output_path = fig_path / f"{fig_name}"
        fig = plt.gcf()
        fig.savefig(output_path, dpi=150, bbox_inches="tight")
        print(f"Saved: {output_path}")


def plot_impact(
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
    setup_pyplot()

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


def plot_trajectory(
    trajectory_df: pd.DataFrame,
    save_path: Optional[Path] = None,
) -> None:
    """
    Generate trajectory analysis plots.

    Args:
        trajectory_df: DataFrame with trajectory data
        save_path: Optional path to save figures
    """
    setup_pyplot()

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
    ax_total, ay_total, az_total = (
        trajectory_df["ax_total"].values,
        trajectory_df["ay_total"].values,
        trajectory_df["az_total"].values,
    )
    current_mass = trajectory_df["current_mass"].values
    est_ax_total, est_ay_total, est_az_total = (
        trajectory_df["est_ax_total"].values,
        trajectory_df["est_ay_total"].values,
        trajectory_df["est_az_total"].values,
    )
    ax_lift, ay_lift, az_lift = (
        trajectory_df["true_a_lift_x"].values,
        trajectory_df["true_a_lift_y"].values,
        trajectory_df["true_a_lift_z"].values,
    )
    ax_thrust, ay_thrust, az_thrust = (
        trajectory_df["ax_thrust"].values,
        trajectory_df["ay_thrust"].values,
        trajectory_df["az_thrust"].values,
    )

    est_ax_lift = _get_series_or_default(trajectory_df, "est_a_lift_x")
    est_ay_lift = _get_series_or_default(trajectory_df, "est_a_lift_y")
    est_az_lift = _get_series_or_default(trajectory_df, "est_a_lift_z")
    est_ax_thrust = _get_series_or_default(trajectory_df, "est_ax_thrust")
    est_ay_thrust = _get_series_or_default(trajectory_df, "est_ay_thrust")
    est_az_thrust = _get_series_or_default(trajectory_df, "est_az_thrust")

    # Calculate altitude
    altitude = np.sqrt(x**2 + y**2 + z**2) - 6.371e6
    est_altitude = np.sqrt(est_x**2 + est_y**2 + est_z**2) - 6.371e6

    # Define phases
    boost_mask = t <= 188
    midcourse_mask = (t > 188) & (altitude >= 1e5)
    reentry_mask = (t > 188) & (altitude < 1e5)

    plots = {
        "position": lambda: _plot_position(t, x, y, z, est_x, est_y, est_z, save_path),
        "position_error": lambda: _plot_position_error(
            t, x, y, z, est_x, est_y, est_z, save_path
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
        "acceleration_true_est": lambda: _plot_acceleration_true_est(
            t,
            ax_total,
            ay_total,
            az_total,
            est_ax_total,
            est_ay_total,
            est_az_total,
            boost_mask,
            midcourse_mask,
            reentry_mask,
            save_path,
        ),
        "lift_true_est": lambda: _plot_force_true_est(
            t,
            ax_lift,
            ay_lift,
            az_lift,
            est_ax_lift,
            est_ay_lift,
            est_az_lift,
            boost_mask,
            midcourse_mask,
            reentry_mask,
            "Lift",
            "lift_true_est.png",
            save_path,
        ),
        "thrust_true_est": lambda: _plot_force_true_est(
            t,
            ax_thrust,
            ay_thrust,
            az_thrust,
            est_ax_thrust,
            est_ay_thrust,
            est_az_thrust,
            boost_mask,
            midcourse_mask,
            reentry_mask,
            "Thrust",
            "thrust_true_est.png",
            save_path,
        ),
        "orbit": lambda: _plot_orbit(x, y, est_x, est_y, save_path),
    }

    for plot_name, plot_func in plots.items():
        print(f"Generating {plot_name}...")
        plot_func()


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


def _plot_velocity_est_phases(
    t, est_vx, est_vy, est_vz, boost_mask, midcourse_mask, reentry_mask, save_path
):
    """Estimated velocity during different flight phases."""
    fig, axes = plt.subplots(3, 1, figsize=(10, 10))

    phases = [
        ("Boost", boost_mask),
        ("Midcourse", midcourse_mask),
        ("Reentry", reentry_mask),
    ]

    for ax, (phase_name, mask) in zip(axes, phases):
        ax.plot(t[mask], est_vx[mask], label="est vx", linewidth=2)
        ax.plot(t[mask], est_vy[mask], label="est vy", linewidth=2)
        ax.plot(t[mask], est_vz[mask], label="est vz", linewidth=2)
        ax.set_ylabel("Estimated Velocity (m/s)")
        ax.set_title(f"Estimated Velocity ({phase_name} Phase)")
        ax.legend()
        ax.grid(alpha=0.3)

    axes[-1].set_xlabel("Time (s)")
    if save_path:
        _save_figure(Path(save_path), "velocity_est_phases.png")
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


def _plot_orbit(x, y, est_x, est_y, save_path):
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
    plt.xlim(-1.2 * earth_radius, 1.5 * earth_radius)
    plt.ylim(-1.2 * earth_radius, 1.5 * earth_radius)
    plt.axis("off")
    plt.legend(loc="upper right")

    if save_path:
        _save_figure(Path(save_path), "orbit.png")
    else:
        plt.show()
    plt.close()


def _plot_acceleration_true_est(
    t,
    ax_total,
    ay_total,
    az_total,
    est_ax_total,
    est_ay_total,
    est_az_total,
    boost_mask,
    midcourse_mask,
    reentry_mask,
    save_path,
):
    """True and estimated acceleration during different flight phases."""
    fig, axes = plt.subplots(3, 1, figsize=(10, 10))

    phases = [
        ("Boost", boost_mask),
        ("Midcourse", midcourse_mask),
        ("Reentry", reentry_mask),
    ]

    for ax, (phase_name, mask) in zip(axes, phases):
        (line_ax,) = ax.plot(t[mask], ax_total[mask], label="ax", linewidth=2)
        (line_ay,) = ax.plot(t[mask], ay_total[mask], label="ay", linewidth=2)
        (line_az,) = ax.plot(t[mask], az_total[mask], label="az", linewidth=2)

        ax.plot(
            t[mask],
            est_ax_total[mask],
            label="est ax",
            linewidth=2,
            linestyle="--",
            color=line_ax.get_color(),
        )
        ax.plot(
            t[mask],
            est_ay_total[mask],
            label="est ay",
            linewidth=2,
            linestyle="--",
            color=line_ay.get_color(),
        )
        ax.plot(
            t[mask],
            est_az_total[mask],
            label="est az",
            linewidth=2,
            linestyle="--",
            color=line_az.get_color(),
        )

        ax.set_ylabel("Acceleration (m/s²)")
        ax.set_title(f"Acceleration ({phase_name} Phase)")
        ax.legend()
        ax.grid(alpha=0.3)

    axes[-1].set_xlabel("Time (s)")

    if save_path:
        _save_figure(Path(save_path), "acceleration_true_est.png")
    else:
        plt.show()
    plt.close()


def _plot_force_true_est(
    t,
    ax_true,
    ay_true,
    az_true,
    ax_est,
    ay_est,
    az_est,
    boost_mask,
    midcourse_mask,
    reentry_mask,
    force_name,
    fig_name,
    save_path,
):
    """True and estimated force components during different flight phases."""
    fig, axes = plt.subplots(3, 1, figsize=(10, 10))

    phases = [
        ("Boost", boost_mask),
        ("Midcourse", midcourse_mask),
        ("Reentry", reentry_mask),
    ]

    for ax, (phase_name, mask) in zip(axes, phases):
        (line_x,) = ax.plot(t[mask], ax_true[mask], label="x", linewidth=2)
        (line_y,) = ax.plot(t[mask], ay_true[mask], label="y", linewidth=2)
        (line_z,) = ax.plot(t[mask], az_true[mask], label="z", linewidth=2)

        ax.plot(
            t[mask],
            ax_est[mask],
            label="est x",
            linewidth=2,
            linestyle="--",
            color=line_x.get_color(),
        )
        ax.plot(
            t[mask],
            ay_est[mask],
            label="est y",
            linewidth=2,
            linestyle="--",
            color=line_y.get_color(),
        )
        ax.plot(
            t[mask],
            az_est[mask],
            label="est z",
            linewidth=2,
            linestyle="--",
            color=line_z.get_color(),
        )

        ax.set_ylabel(f"{force_name} Accel (m/s²)")
        ax.set_title(f"{force_name} Acceleration ({phase_name} Phase)")
        ax.legend()
        ax.grid(alpha=0.3)

    axes[-1].set_xlabel("Time (s)")

    if save_path:
        _save_figure(Path(save_path), fig_name)
    else:
        plt.show()
    plt.close()
