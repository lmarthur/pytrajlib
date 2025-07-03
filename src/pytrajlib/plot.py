# This script contains code to generate scatter plots and histograms of the impact data.
import os

import folium
import matplotlib.pyplot as plt
import numpy as np
import scipy.stats as stats
from folium.features import DivIcon

from pytrajlib.utils import (
    EARTH_RADIUS,
    cart2sphere,
    get_cep,
    get_cep_miss_distance_from_local_impact,
    get_impact_data,
    get_local_impact,
    get_run_params,
    haversine_distance,
    transform_to_earth_coords,
)


def impact(run_params=None, data=None, output_dir=None):
    """
    Plot the impact data from the simulation.

    INPUTS
    --------
        run_params (dict): Run parameters for the simulation. If None, use the
            default parameters.
        data (np.ndarray | pd.DataFrame): Impact data from the simulation.
            If None, read from the file specified by run_params.
        output_dir (str): Directory to save the plots, e.g. "output/". If None, use do not save
            the plots.
    """
    if run_params is None:
        run_params = get_run_params()

    # get longitude and latitude of aimpoint and launchpoint
    aimpoint_lat, aimpoint_lon = cart2sphere(
        run_params["x_aim"], run_params["y_aim"], run_params["z_aim"]
    )
    launch_lat, launch_lon = cart2sphere(
        run_params["x_launch"], run_params["y_launch"], run_params["z_launch"]
    )
    # Calculate the range to the aimpoint over the surface of the Earth
    # This is the great circle distance between the aimpoint and the origin
    range_to_aimpoint = haversine_distance(
        (launch_lat, launch_lon), (aimpoint_lat, aimpoint_lon)
    )
    print("Range to aimpoint: ", range_to_aimpoint)

    impact_x_local, impact_y_local = get_local_impact(run_params, data)
    miss_distance, cep = get_cep_miss_distance_from_local_impact(
        impact_x_local, impact_y_local
    )
    plotrange = 4 * cep

    # Plot the data
    params = {
        "axes.labelsize": 8,
        "font.size": 8,
        "font.family": "serif",
        "legend.fontsize": 10,
        "xtick.labelsize": 10,
        "ytick.labelsize": 10,
    }
    plt.rcParams.update(params)

    fig = plt.figure(figsize=(5, 5))
    # plot a circle of radius CEP m centered on (0,0)
    N = 400
    t = np.linspace(0, 2 * np.pi, N)
    x, y = cep * np.cos(t), cep * np.sin(t)

    # gridspec
    gs = fig.add_gridspec(
        2,
        1,
        height_ratios=(6, 1),
        hspace=0.18,
        bottom=0.1,
        top=0.95,
        left=0.025,
        right=0.975,
    )

    a0 = fig.add_subplot(gs[0, 0])
    a1 = fig.add_subplot(gs[1, 0])

    a0.scatter(
        impact_x_local,
        impact_y_local,
        c="grey",
        marker="x",
        label="Impact Points",
        s=20,
        alpha=0.5,
        linewidths=1,
    )
    a0.plot(x, y, c="k", label="CEP", linestyle="--", linewidth=1.5)
    a0.legend(["Impact Points", "CEP"], frameon=False, framealpha=0)

    # center the plot on (0,0)
    a0.set_xlim(-plotrange, plotrange)
    a0.set_ylim(-plotrange, plotrange)
    a0.set_aspect("equal")

    # add N=len(guided_r) to the top left of the plot
    a0.text(
        -0.6 * plotrange,
        0.8 * plotrange,
        "N = " + str(len(miss_distance)) + "\nCEP = " + str(cep) + "m",
        fontsize=10,
        verticalalignment="top",
        horizontalalignment="center",
    )

    # add label to a0
    a0.set_xlabel("Downrange (m)", labelpad=-1)
    a0.set_ylabel("Crossrange (m)", labelpad=-1)
    # For a0 axis (if you have operations on it that require adjustments)
    a0.tick_params(axis="x", which="major", pad=1)  # Adjust pad for x-axis ticks
    a0.tick_params(axis="y", which="major", pad=1)  # Adjust pad for y-axis ticks

    if run_params["run_name"] == "run_0":
        a0.set_title("Minuteman III: Ballistic RV, INS-Only")
    elif run_params["run_name"] == "run_2":
        a0.set_title("Minuteman III: Perfectly Maneuverable RV, INS-Only")
    elif run_params["run_name"] == "run_3":
        a0.set_title("Minuteman III: Perfectly Maneuverable RV, INS+GNSS")
    elif run_params["run_name"] == "run_4":
        a0.set_title("Minuteman III: Maneuverable RV, INS+GNSS")
    elif run_params["run_name"] != "test":
        a0.set_title("Trajectory Impact Points")
    # plot the histogram of the miss distances

    cep = round(np.percentile(miss_distance, 50), 2)

    # Fit a Nakagami distribution to the data
    x = np.linspace(0, 5 * cep, 100)
    shape, loc, scale = stats.nakagami.fit(miss_distance, floc=0)
    nakagamipdf = stats.nakagami.pdf(x, shape, loc, scale)
    print("Nakagami fit: shape =", shape, "loc =", loc, "scale =", scale)

    # Compute number of bins for the histogram
    bins = 50
    # plot histogram up to 5 times the CEP, with no y axis
    a1.hist(
        miss_distance,
        bins=bins,
        range=(0, 5 * cep),
        color="grey",
        edgecolor="black",
        alpha=0.7,
        histtype="stepfilled",
    )
    # renormalize the pdfs to the histogram
    nakagamipdf = nakagamipdf * len(miss_distance) * 5 * cep / bins
    # evaluate the pdf at the CEP
    pdf_cep = nakagamipdf[np.argmin(np.abs(x - cep))]
    # Add a vertical line at the CEP, to the top of the histogram at that point
    plotmax = a1.get_ylim()[1]
    a1.axvline(
        x=cep,
        ymax=pdf_cep / plotmax,
        color="k",
        linestyle="--",
        linewidth=1.5,
        label="CEP",
    )

    a1.plot(
        x,
        nakagamipdf,
        "k",
        linewidth=1.5,
        label="Nakagami(" + str(round(shape, 2)) + ", " + str(round(scale, 2)) + ")",
    )

    # omit the frame
    a1.spines["top"].set_visible(False)
    a1.spines["right"].set_visible(False)
    a1.spines["left"].set_visible(False)
    a1.spines["bottom"].set_visible(False)

    a1.yaxis.set_visible(False)
    # Legend with no border or box around it, and with the nakagami fit parameters
    a1.legend(frameon=False, framealpha=0)
    a1.tick_params(axis="x", which="major", pad=1)
    a1.tick_params(axis="y", which="major", pad=1)
    a1.set_xlabel("Miss Distance Histogram (m)", labelpad=1)

    if output_dir is not None:
        plt.savefig(output_dir + "/impact_plot.jpg", dpi=1000)
        plt.savefig(output_dir + "/impact_plot.pdf")
        plt.close()


def _set_trajectory_plot_params():
    params = {
        "axes.labelsize": 18,
        "font.size": 18,
        "font.family": "serif",
        "legend.fontsize": 18,
        "xtick.labelsize": 18,
        "ytick.labelsize": 18,
    }
    plt.rcParams.update(params)


def _get_trajectory_data(run_params=None, data=None):
    if run_params is None:
        run_params = get_run_params()
    if data is None:
        # print error if the paths are not found
        if not os.path.exists(run_params["trajectory_path"]):
            print(f"Trajectory data file {run_params['trajectory_path']} not found")
            return
        data = np.loadtxt(run_params["trajectory_path"], delimiter=",", skiprows=1)
    return data


def _get_altitude(x, y, z):
    """
    Get the altitude from the trajectory data.

    INPUTS
    --------
        x (np.ndarray): x position from the trajectory data.
        y (np.ndarray): y position from the trajectory data.
        z (np.ndarray): z position from the trajectory data.
    OUTPUTS
    --------
        altitude (np.ndarray): Altitude from the trajectory data.
    """
    true_altitude = np.sqrt(np.square(x) + np.square(y) + np.square(z)) - EARTH_RADIUS
    return true_altitude


def position(run_params=None, data=None, output_dir=None):
    """
    Plot position vs. time

    INPUTS
    --------
        run_params (dict): Run parameters for the simulation. If None, and data
            is None, use the default parameters.
        data (np.ndarray | pd.DataFrame): Trajectory data from the simulation.
            If None, read from the file specified by run_params.
        output_dir (str): Directory to save the plots, e.g. "output/". If None, use do not save
            the plots.
    """
    _set_trajectory_plot_params()
    data = _get_trajectory_data(run_params, data)
    true_t = data[:, 0]
    true_x = data[:, 2]
    true_y = data[:, 3]
    true_z = data[:, 4]

    plt.figure(figsize=(10, 10))
    plt.plot(true_t, true_x, label="x")
    plt.plot(true_t, true_y, label="y")
    plt.plot(true_t, true_z, label="z")
    plt.xlabel("Time (s)")
    plt.ylabel("Position (m)")
    plt.title("Position")
    plt.legend()
    plt.grid()
    if output_dir is not None:
        plt.savefig(output_dir + "/position.pdf")
        plt.close()


def position_error(run_params=None, data=None, output_dir=None):
    """
    Plot position error

    INPUTS
    --------
        run_params (dict): Run parameters for the simulation. If None, and data
            is None, use the default parameters.
        data (np.ndarray | pd.DataFrame): Trajectory data from the simulation.
            If None, read from the file specified by run_params.
        output_dir (str): Directory to save the plots, e.g. "output/". If None, use do not save
            the plots.
    """
    _set_trajectory_plot_params()
    data = _get_trajectory_data(run_params, data)
    true_t = data[:, 0]
    true_x = data[:, 2]
    true_y = data[:, 3]
    true_z = data[:, 4]
    est_x = data[:, 22]
    est_y = data[:, 23]
    est_z = data[:, 24]

    plt.figure(figsize=(10, 10))
    plt.plot(true_t, true_x - est_x, label="x")
    plt.plot(true_t, true_y - est_y, label="y")
    plt.plot(true_t, true_z - est_z, label="z")
    plt.xlabel("Time (s)")
    plt.ylabel("Position Error (m)")
    plt.title("Position Error")
    plt.legend()
    plt.grid()
    if output_dir is not None:
        plt.savefig(output_dir + "/position_error.pdf")
    plt.close()


def orbit(run_params=None, data=None, output_dir=None):
    """
    Orbit plot

    INPUTS
    --------
        run_params (dict): Run parameters for the simulation. If None, and data
            is None, use the default parameters.
        data (np.ndarray | pd.DataFrame): Trajectory data from the simulation.
            If None, read from the file specified by run_params.
        output_dir (str): Directory to save the plots, e.g. "output/". If None, use do not save
            the plots.
    """
    _set_trajectory_plot_params()
    data = _get_trajectory_data(run_params, data)
    true_x = data[:, 2]
    true_y = data[:, 3]

    plt.figure(figsize=(10, 10))

    # add shaded region for Earth's atmosphere
    earth_atmosphere = plt.Circle(
        (0, 0), EARTH_RADIUS + 200e3, color="lightblue", label="Atmosphere"
    )
    plt.gca().add_artist(earth_atmosphere)

    # plot the Earth
    earth = plt.Circle((0, 0), EARTH_RADIUS, color="blue", label="Earth")
    plt.gca().add_artist(earth)
    # set range for x and y axes to 2*earth_radius
    plt.xlim(-1.2 * EARTH_RADIUS, 1.5 * EARTH_RADIUS)
    plt.ylim(-1.2 * EARTH_RADIUS, 1.5 * EARTH_RADIUS)

    # plot the vehicle's trajectory in the x-y plane
    plt.plot(true_x, true_y, "r", label="True Trajectory")
    # turn off the axis labels
    plt.axis("off")

    if output_dir is not None:
        plt.savefig(output_dir + "/orbit.pdf")
    plt.close()


def altitude(run_params=None, data=None, output_dir=None):
    """
    Plot altitude vs. time

    INPUTS
    --------
        run_params (dict): Run parameters for the simulation. If None, and data
            is None, use the default parameters.
        data (np.ndarray | pd.DataFrame): Trajectory data from the simulation.
            If None, read from the file specified by run_params.
        output_dir (str): Directory to save the plots, e.g. "output/". If None, use do not save
            the plots.
    """
    _set_trajectory_plot_params()
    data = _get_trajectory_data(run_params, data)
    true_t = data[:, 0]
    true_x = data[:, 2]
    true_y = data[:, 3]
    true_z = data[:, 4]
    true_altitude = _get_altitude(true_x, true_y, true_z)

    plt.figure(figsize=(10, 10))
    plt.plot(true_t, true_altitude / 1000)
    plt.xlabel("Time (s)")
    plt.ylabel("Altitude (km)")
    # remove top and right spines
    plt.gca().spines["top"].set_visible(False)
    plt.gca().spines["right"].set_visible(False)
    # shade under the curve from 0 to 160 seconds
    plt.fill_between(
        true_t,
        true_altitude / 1000,
        0,
        where=(true_t < 188),
        color="lightblue",
        alpha=0.5,
    )
    # add "guided" label to shaded region with arrow
    # plt.annotate('Boost (INS)', xy=(188, 40), xytext=(500, 50), arrowprops=dict(facecolor='black', arrowstyle='->'))
    # add "ballistic phase"
    # plt.annotate('Ballistic Phase\n (No Control, GNSS)', xy=(1500, 1500), ha='center')
    # shade under the curve for altitude < 100 and t < 1000
    # plt.fill_between(true_t, true_altitude/1000, 0, where=(true_t > 2915), color='red', alpha=0.5)
    # plt.annotate('Reentry\n (INS)', xy=(2910, 40), xytext=(2200, 250), arrowprops=dict(facecolor='black', arrowstyle='->'), ha='center')
    if output_dir is not None:
        plt.savefig(output_dir + "/altitude.pdf")
    plt.close()


def altitude_error(run_params=None, data=None, output_dir=None):
    """
    Plot altitude error

    INPUTS
    --------
        run_params (dict): Run parameters for the simulation. If None, and data
            is None, use the default parameters.
        data (np.ndarray | pd.DataFrame): Trajectory data from the simulation.
            If None, read from the file specified by run_params.
        output_dir (str): Directory to save the plots, e.g. "output/". If None, use do not save
            the plots.
    """
    _set_trajectory_plot_params()
    data = _get_trajectory_data(run_params, data)
    true_t = data[:, 0]
    true_x = data[:, 2]
    true_y = data[:, 3]
    true_z = data[:, 4]
    est_x = data[:, 22]
    est_y = data[:, 23]
    est_z = data[:, 24]
    true_altitude = _get_altitude(true_x, true_y, true_z)
    est_altitude = _get_altitude(est_x, est_y, est_z)

    plt.figure(figsize=(10, 10))
    plt.plot(true_t, true_altitude - est_altitude)
    plt.xlabel("Time (s)")
    plt.ylabel("Altitude Error (m)")
    plt.title("Altitude Error")
    plt.grid()
    if output_dir is not None:
        plt.savefig(output_dir + "/altitude_error.pdf")
        plt.close()


def velocity(run_params=None, data=None, output_dir=None):
    """
    Plot velocity vs. time

    INPUTS
    --------
        run_params (dict): Run parameters for the simulation. If None, and data
            is None, use the default parameters.
        data (np.ndarray | pd.DataFrame): Trajectory data from the simulation.
            If None, read from the file specified by run_params.
        output_dir (str): Directory to save the plots, e.g. "output/". If None, use do not save
            the plots.
    """
    _set_trajectory_plot_params()
    data = _get_trajectory_data(run_params, data)
    true_t = data[:, 0]
    true_vx = data[:, 5]
    true_vy = data[:, 6]
    true_vz = data[:, 7]

    plt.figure(figsize=(10, 10))
    plt.plot(true_t, true_vx, label="vx")
    plt.plot(true_t, true_vy, label="vy")
    plt.plot(true_t, true_vz, label="vz")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")
    plt.title("Velocity")
    plt.legend()
    plt.grid()
    if output_dir is not None:
        plt.savefig(output_dir + "/velocity.pdf")
        plt.close()


def velocity_error(run_params=None, data=None, output_dir=None):
    """
    Plot velocity error

    INPUTS
    --------
        run_params (dict): Run parameters for the simulation. If None, and data
            is None, use the default parameters.
        data (np.ndarray | pd.DataFrame): Trajectory data from the simulation.
            If None, read from the file specified by run_params.
        output_dir (str): Directory to save the plots, e.g. "output/". If None, use do not save
            the plots.
    """
    _set_trajectory_plot_params()
    data = _get_trajectory_data(run_params, data)
    true_t = data[:, 0]
    true_vx = data[:, 5]
    true_vy = data[:, 6]
    true_vz = data[:, 7]
    est_vx = data[:, 25]
    est_vy = data[:, 26]
    est_vz = data[:, 27]

    plt.figure(figsize=(10, 10))
    plt.plot(true_t, true_vx - est_vx, label="vx")
    plt.plot(true_t, true_vy - est_vy, label="vy")
    plt.plot(true_t, true_vz - est_vz, label="vz")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity Error (m/s)")
    plt.title("Velocity Error")
    plt.legend()
    plt.grid()
    if output_dir is not None:
        plt.savefig(output_dir + "/velocity_error.pdf")
        plt.close()


def thrust(run_params=None, data=None, output_dir=None):
    """
    Plot thrust vs. time

    INPUTS
    --------
        run_params (dict): Run parameters for the simulation. If None, and data
            is None, use the default parameters.
        data (np.ndarray | pd.DataFrame): Trajectory data from the simulation.
            If None, read from the file specified by run_params.
        output_dir (str): Directory to save the plots, e.g. "output/". If None, use do not save
            the plots.
    """
    _set_trajectory_plot_params()
    data = _get_trajectory_data(run_params, data)
    true_t = data[:, 0]
    true_ax_thrust = data[:, 16]
    true_ay_thrust = data[:, 17]
    true_az_thrust = data[:, 18]
    true_thrust_mag = np.sqrt(
        np.square(true_ax_thrust)
        + np.square(true_ay_thrust)
        + np.square(true_az_thrust)
    )

    plt.figure(figsize=(10, 10))
    plt.plot(true_t, true_thrust_mag)
    plt.xlabel("Time (s)")
    plt.ylabel("Thrust Acceleration (m/s^2)")
    plt.title("Thrust")
    plt.grid()
    if output_dir is not None:
        plt.savefig(output_dir + "/thrust.pdf")
        plt.close()


def mass(run_params=None, data=None, output_dir=None):
    """
    Plot mass vs. time

    INPUTS
    --------
        run_params (dict): Run parameters for the simulation. If None, and data
            is None, use the default parameters.
        data (np.ndarray | pd.DataFrame): Trajectory data from the simulation.
            If None, read from the file specified by run_params.
        output_dir (str): Directory to save the plots, e.g. "output/". If None, use do not save
            the plots.
    """
    _set_trajectory_plot_params()
    data = _get_trajectory_data(run_params, data)
    true_t = data[:, 0]
    true_mass = data[:, 1]

    plt.figure(figsize=(10, 10))
    plt.plot(true_t, true_mass)
    plt.xlabel("Time (s)")
    plt.ylabel("Mass (kg)")
    plt.title("Mass")
    plt.grid()
    if output_dir is not None:
        plt.savefig(output_dir + "/mass.pdf")
        plt.close()


def acceleration(run_params=None, data=None, output_dir=None):
    """
    Plot acceleration vs. time

    INPUTS
    --------
        run_params (dict): Run parameters for the simulation. If None, and data
            is None, use the default parameters.
        data (np.ndarray | pd.DataFrame): Trajectory data from the simulation.
            If None, read from the file specified by run_params.
        output_dir (str): Directory to save the plots, e.g. "output/". If None, use do not save
            the plots.
    """
    _set_trajectory_plot_params()
    data = _get_trajectory_data(run_params, data)
    true_t = data[:, 0]
    true_ax_total = data[:, 19]
    true_ay_total = data[:, 20]
    true_az_total = data[:, 21]

    plt.figure(figsize=(10, 10))
    plt.plot(true_t, true_ax_total, label="ax")
    plt.plot(true_t, true_ay_total, label="ay")
    plt.plot(true_t, true_az_total, label="az")
    plt.xlabel("Time (s)")
    plt.ylabel("Acceleration (m/s^2)")
    plt.title("Acceleration")
    plt.legend()
    plt.grid()
    if output_dir is not None:
        plt.savefig(output_dir + "/acceleration.pdf")
        plt.close()


def acceleration_error(run_params=None, data=None, output_dir=None):
    """
    Plot acceleration error

    INPUTS
    --------
        run_params (dict): Run parameters for the simulation. If None, and data
            is None, use the default parameters.
        data (np.ndarray | pd.DataFrame): Trajectory data from the simulation.
            If None, read from the file specified by run_params.
        output_dir (str): Directory to save the plots, e.g. "output/". If None, use do not save
            the plots.
    """
    _set_trajectory_plot_params()
    data = _get_trajectory_data(run_params, data)
    true_t = data[:, 0]
    true_ax_total = data[:, 19]
    true_ay_total = data[:, 20]
    true_az_total = data[:, 21]
    est_ax_total = data[:, 28]
    est_ay_total = data[:, 29]
    est_az_total = data[:, 30]

    plt.figure(figsize=(10, 10))
    plt.plot(true_t, true_ax_total - est_ax_total, label="ax")
    plt.plot(true_t, true_ay_total - est_ay_total, label="ay")
    plt.plot(true_t, true_az_total - est_az_total, label="az")
    plt.xlabel("Time (s)")
    plt.ylabel("Acceleration Error (m/s^2)")
    plt.title("Acceleration Error")
    plt.legend()
    plt.grid()
    if output_dir is not None:
        plt.savefig(output_dir + "/acceleration_error.pdf")
        plt.close()


def drag_acceleration(run_params=None, data=None, output_dir=None):
    """
    Plot drag acceleration

    INPUTS
    --------
        run_params (dict): Run parameters for the simulation. If None, and data
            is None, use the default parameters.
        data (np.ndarray | pd.DataFrame): Trajectory data from the simulation.
            If None, read from the file specified by run_params.
        output_dir (str): Directory to save the plots, e.g. "output/". If None, use do not save
            the plots.
    """
    _set_trajectory_plot_params()
    data = _get_trajectory_data(run_params, data)
    true_t = data[:, 0]
    true_ax_drag = data[:, 11]
    true_ay_drag = data[:, 12]
    true_az_drag = data[:, 13]

    plt.figure(figsize=(10, 10))
    plt.plot(true_t, true_ax_drag, label="ax")
    plt.plot(true_t, true_ay_drag, label="ay")
    plt.plot(true_t, true_az_drag, label="az")
    plt.xlabel("Time (s)")
    plt.ylabel("Drag Acceleration (m/s^2)")
    plt.title("Drag Acceleration")
    plt.legend()
    plt.grid()
    if output_dir is not None:
        plt.savefig(output_dir + "/drag_acceleration.pdf")
        plt.close()


def lift_acceleration(run_params=None, data=None, output_dir=None):
    """
    Plot lift acceleration

    INPUTS
    --------
        run_params (dict): Run parameters for the simulation. If None, and data
            is None, use the default parameters.
        data (np.ndarray | pd.DataFrame): Trajectory data from the simulation.
            If None, read from the file specified by run_params.
        output_dir (str): Directory to save the plots, e.g. "output/". If None, use do not save
            the plots.
    """
    _set_trajectory_plot_params()
    data = _get_trajectory_data(run_params, data)
    true_t = data[:, 0]
    a_command = data[:, 14]
    a_exec = data[:, 15]

    plt.figure(figsize=(10, 10))
    plt.plot(true_t[0:-10], a_command[0:-10], label="a_command")
    plt.plot(true_t[0:-10], a_exec[0:-10], label="a_exec")
    # plt.ylim(0, 25) # limit y-axis to 0-50 for better visibility of the lift acceleration

    plt.yscale("symlog")
    plt.xlabel("Time (s)")
    plt.ylabel("Lift Acceleration (m/s^2)")
    plt.title("Lift Acceleration")
    plt.legend()
    plt.grid()
    if output_dir is not None:
        plt.savefig(output_dir + "/lift_acceleration.pdf")
        plt.close()


def position_vs_altitude(run_params=None, data=None, output_dir=None):
    """
    Plot y and z position vs. altitude

    INPUTS
    --------
        run_params (dict): Run parameters for the simulation. If None, and data
            is None, use the default parameters.
        data (np.ndarray | pd.DataFrame): Trajectory data from the simulation.
            If None, read from the file specified by run_params.
        output_dir (str): Directory to save the plots, e.g. "output/". If None, use do not save
            the plots.
    """
    _set_trajectory_plot_params()
    data = _get_trajectory_data(run_params, data)
    true_x = data[:, 2]
    true_y = data[:, 3]
    true_z = data[:, 4]
    true_altitude = _get_altitude(true_x, true_y, true_z)

    plt.figure(figsize=(10, 10))
    plt.plot(500000 - true_altitude, true_y, label="y")
    plt.plot(500000 - true_altitude, true_z, label="z")
    plt.xlabel("Altitude")
    plt.ylabel("Position (m)")
    # no x axis ticks
    plt.xticks([])
    plt.title("Lateral Position vs. Altitude")
    plt.legend()
    plt.grid()
    if output_dir is not None:
        plt.savefig(output_dir + "/position_vs_altitude.pdf")
        plt.close()


def all_trajectory_plots(run_params=None, data=None, output_dir=None):
    """
    Plot the trajectory of the vehicle.

    INPUTS
    --------
        run_params (dict): Run parameters for the simulation. If None, and data
            is None, use the default parameters.
        data (np.ndarray | pd.DataFrame): Trajectory data from the simulation.
            If None, read from the file specified by run_params.
        output_dir (str): Directory to save the plots, e.g. "output/". If None, use do not save
            the plots.
    """
    position(run_params, data, output_dir)
    position_error(run_params, data, output_dir)
    orbit(run_params, data, output_dir)
    altitude(run_params, data, output_dir)
    altitude_error(run_params, data, output_dir)
    velocity(run_params, data, output_dir)
    velocity_error(run_params, data, output_dir)
    thrust(run_params, data, output_dir)
    mass(run_params, data, output_dir)
    acceleration(run_params, data, output_dir)
    acceleration_error(run_params, data, output_dir)
    drag_acceleration(run_params, data, output_dir)
    lift_acceleration(run_params, data, output_dir)
    position_vs_altitude(run_params, data, output_dir)


def map(run_params=None, data=None, output_dir=None, show_attribution=True):
    """
    Plot the trajectory on a map.

    INPUTS
    --------
        run_params (dict): Run parameters for the simulation. If None, and data
            is None, use the default parameters.
        data (np.ndarray | pd.DataFrame): Trajectory data from the simulation.
            If None, read from the file specified by run_params.
        output_dir (str): Directory to save the plots, e.g. "output/". If None, use do not save
            the plots.
        show_attribution (bool): Whether to show the tiles attribution on the map.
    """
    if run_params is None:
        run_params = get_run_params()

    attrctrl = 1 if show_attribution else 0
    m = folium.Map(
        location=[0, 0],
        zoom_start=2,
        attributionControl=attrctrl,
        control_scale=True,
        world_copy_jump=True,
    )

    launch_lat, launch_lon = cart2sphere(
        run_params["x_launch"], run_params["y_launch"], run_params["z_launch"]
    )
    launch = (launch_lat, launch_lon)
    print("xaim in run params: ", "x_aim" in run_params)
    print(run_params["x_aim"])
    aim_lat, aim_lon = cart2sphere(
        run_params["x_aim"], run_params["y_aim"], run_params["z_aim"]
    )

    # Add CEP circle
    cep = get_cep(run_params, data)
    if cep is not None:
        folium.Circle(
            location=[np.rad2deg(aim_lat), np.rad2deg(aim_lon)],
            radius=cep,
            color="black",
            dash_array="5, 5",
            fill=True,
            fill_color="black",
            fill_opacity=0.1,
            weight=1,
            tooltip=f"CEP: {cep:.2f} m",
        ).add_to(m)

    # Add the launch and aim markers
    folium.Marker(
        location=[np.degrees(launch_lat), np.degrees(launch_lon)],
        tooltip=f"Launch Point {np.degrees(launch_lat):.2f}°N, {np.degrees(launch_lon):.2f}°E",
    ).add_to(m)
    folium.Marker(
        location=[np.degrees(aim_lat), np.degrees(aim_lon)],
        tooltip=f"Aim Point {np.degrees(aim_lat):.2f}°N, {np.degrees(aim_lon):.2f}°E",
    ).add_to(m)

    impact_x, impact_y, impact_z = get_impact_data(run_params, data)
    transformed_lat, transformed_lon = transform_to_earth_coords(
        impact_x, impact_y, impact_z, launch
    )
    x_svg = """
    <svg width="24" height="24" viewBox="0 0 32 32" xmlns="http://www.w3.org/2000/svg">
        <line x1="8" y1="8" x2="24" y2="24" stroke="black" stroke-width="3" stroke-linecap="round"/>
        <line x1="24" y1="8" x2="8" y2="24" stroke="black" stroke-width="3" stroke-linecap="round"/>
    </svg>
    """

    # Add the X icon marker
    for lat, lon in zip(transformed_lat, transformed_lon):
        folium.Marker(
            location=[np.rad2deg(lat), np.rad2deg(lon)],
            icon=DivIcon(
                icon_size=(32, 32),
                icon_anchor=(16, 16),
                html=x_svg,
            ),
        ).add_to(m)

    # Add trajectory line
    trajectory_data = _get_trajectory_data(run_params)
    if trajectory_data is None:
        return m
    traj_x = trajectory_data[:, 2]
    traj_y = trajectory_data[:, 3]
    traj_z = trajectory_data[:, 4]
    # Transform the trajectory coordinates to Earth coordinates
    traj_lat, traj_lon = transform_to_earth_coords(traj_x, traj_y, traj_z, launch)
    # Create a PolyLine for the trajectory
    folium.PolyLine(
        locations=[
            (np.rad2deg(lat), np.rad2deg(lon)) for lat, lon in zip(traj_lat, traj_lon)
        ],
        color="black",
        weight=2,
        opacity=0.6,
    ).add_to(m)

    if output_dir is not None:
        m.save(output_dir + "/trajectory_map.html")
    return m
