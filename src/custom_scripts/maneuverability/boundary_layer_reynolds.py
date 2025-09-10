import os

import folium
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import scienceplots
import scipy.stats as stats
from folium.features import DivIcon

from pytrajlib import simulation, utils
from pytrajlib.plot import _get_altitude, _get_trajectory_data
from pytrajlib.utils import (
    EARTH_RADIUS,
    cart2sphere,
    get_cep,
    get_cep_miss_distance_from_local_impact,
    get_impact_data,
    get_local_impact,
    get_run_params,
    haversine_distance,
    sphervec2cartvec,
    transform_to_earth_coords,
)

# Avoid an unused import warning for scienceplots
assert scienceplots
plt.style.use(["science"])

params = {
    "axes.labelsize": 8,
    "font.size": 8,
    "font.family": "serif",
    "legend.fontsize": 8,
    "xtick.labelsize": 10,
    "ytick.labelsize": 10,
    "text.usetex": True,
}

plt.rcParams.update(params)


def get_reynolds_number(data):
    """
    Approximations following Regan p. 315.
    """
    wind_vec = data[:, 32:35]
    x, y, z = data[:, 2], data[:, 3], data[:, 4]
    lat, lon = cart2sphere(x, y, z)
    spher_coords = np.zeros_like(wind_vec)
    spher_coords[:, 0] = np.nan  # radius not needed
    spher_coords[:, 1] = lon
    spher_coords[:, 2] = lat
    cart_wind_vec = sphervec2cartvec(wind_vec, spher_coords).reshape(-1, 3)

    density = data[:, 31]

    rel_v = data[:, 5:8] - cart_wind_vec

    rho = density
    molar_mass = 0.0289652
    Na = 6.022e23  # avogadro's number
    n = rho * Na / molar_mass
    sigma = 3.5e-10  # between nitrogen and oxygen
    gamma = 7 / 5  # Regan 11.6 p. 313
    R = 8.314
    T = 288  # approx sea level temp in K
    # T = 180 # temperature at ~86km (Regan p. 40)

    # This is speed of sound in perfect gas, but it is not giving ~300 m/s..
    a = np.sqrt(gamma * R * T)
    print(f"{a=}")
    lmbda = 1 / (np.sqrt(2) * np.pi * sigma**2 * n)
    mu = 1 / 2 * rho * lmbda * a * np.sqrt(8 / (np.pi * gamma))
    L = 2.75  # Length of RV. Should it be length of flap? something else?

    V = np.sqrt(np.sum(np.square(rel_v)))
    Re = rho * V * L / mu
    return Re


def reynolds_number(run_params=None, data=None, output_dir=None):
    data = _get_trajectory_data(run_params, data)

    true_t = data[:, 0]
    x, y, z = data[:, 2], data[:, 3], data[:, 4]
    altitude = _get_altitude(x, y, z)

    Re = get_reynolds_number(data)
    plt.figure(figsize=(10, 10))
    plt.plot(altitude, Re)
    plt.xlabel("Altitude (m)")
    plt.ylabel("Reynolds Number")
    plt.title("Reynolds Number")
    plt.grid()
    if output_dir is not None:
        plt.savefig(output_dir + "/reynolds_number.pdf")
        plt.close()
    return Re


def boundary_layer_thickness(run_params=None, data=None, output_dir=None):
    data = _get_trajectory_data(run_params, data)

    true_t = data[:, 0]
    x, y, z = data[:, 2], data[:, 3], data[:, 4]
    altitude = _get_altitude(x, y, z)
    Re = get_reynolds_number(data)

    L = 2.75  # Length of RV. Should it be length of flap? something else?
    # L = 0.3

    # Laminar approximation
    delta_laminar = L / np.sqrt(Re)

    # Turbulent approximation
    delta_turbulent = 0.37 * L / Re ** (1 / 5)

    plt.figure(figsize=(10, 10))
    plt.plot(altitude, delta_laminar, label="Laminar approximation")
    plt.plot(altitude, delta_turbulent, label="Turbulent approximation")
    plt.xlabel("Altitude (m)")
    plt.ylabel("Boundary Layer Thickness (m)")
    plt.title("Boundary Layer Thickness")
    plt.grid()
    plt.legend()
    plt.semilogy()
    plt.semilogx()
    if output_dir is not None:
        plt.savefig(output_dir + "/boundary_layer.pdf")
        plt.close()


run_params = simulation.get_run_params("input/reentry.toml")
run_params["num_runs"] = 1
run_params["time_step_reentry"] = 0.01
run_params["traj_output"] = 2

impact_data = simulation.run(run_params)

reynolds_number(run_params, output_dir="output")
boundary_layer_thickness(run_params, output_dir="output")
