import configparser
import importlib.resources
import os

import numpy as np
import pandas as pd

from ._traj import ffi

EARTH_RADIUS = 6371e3


def to_python_type(value):
    """
    Convert string values to their corresponding Python types.
    INPUTS:
    ----------
        value: str
            The value to convert.
    OUTPUTS:
    ----------
        python_value: any
            The converted value.
    """
    if value.isdecimal():
        return int(value)
    try:
        return float(value)
    except ValueError:
        return value


def to_c_type(value):
    """
    Convert a Python value to its corresponding C type.

    INPUTS:
    ----------
        value: any
            The value to convert.

    OUTPUTS:
    ----------
        c_value: ctype
            The converted value.

    """
    if isinstance(value, str):
        s = ffi.new("char[]", value.encode("utf-8"))
        return s
    return value


def impact_data_to_df(impact_data, num_runs):
    """
    Convert the impact data to a Pandas DataFrame.

    INPUTS:
    -------
        impact_data: impact_data
            The impact data from the Monte Carlo run.
        num_runs: int
            The number of runs in the Monte Carlo simulation.

    OUTPUTS:
    -------
        impact_df: pd.DataFrame
            The impact data as a Pandas DataFrame.
    """
    impact_df = pd.DataFrame()
    rows = []
    for i in range(num_runs):
        row_data = dict(
            t=impact_data.impact_states[i].t,
            x=impact_data.impact_states[i].x,
            y=impact_data.impact_states[i].y,
            z=impact_data.impact_states[i].z,
            vx=impact_data.impact_states[i].vx,
            vy=impact_data.impact_states[i].vy,
            vz=impact_data.impact_states[i].vz,
        )
        rows.append(row_data)
    impact_df = pd.DataFrame(rows)
    return impact_df


def cart2sphere(x, y, z):
    """Convert Cartesian coordinates to spherical coordinates.

    INPUTS
    --------
        x (np.ndarray): x coordinate.
        y (np.ndarray): y coordinate.
        z (np.ndarray): z coordinate.
    OUTPUTS
    --------
        lat (np.ndarray): Latitude in radians.
        lon (np.ndarray): Longitude in radians.
    """
    lat = np.atan(z / np.sqrt(x**2 + y**2))
    lon = np.arctan2(y, x)
    return lat, lon


def sphere2cart(r, lon, lat):
    """
    Converts spherical coordinates to Cartesian coordinates.

    INPUTS:
    ----------
        spher_coords: [r, lon, lat] in radians

    OUTPUTS:
    ----------
        cart_coords: cartesian coords in meters
            [x, y, z]
    """
    x = r * np.cos(lon) * np.cos(lat)
    y = r * np.sin(lon) * np.cos(lat)
    z = r * np.sin(lat)
    return x, y, z


def calc_bearing(start, end):
    """
    Calculate the bearing (in radians) from start to end (lat, lon in radians).

    INPUTS:
    ----------
        start: tuple of (lat, lon) in radians
        end: tuple of (lat, lon) in radians
    OUTPUTS:
    ----------
        bearing: bearing in radians
    """
    launch_lat, launch_lon = start
    aim_lat, aim_lon = end
    lon_diff = aim_lon - launch_lon

    east = np.sin(lon_diff) * np.cos(aim_lat)
    north = np.cos(launch_lat) * np.sin(aim_lat) - np.sin(launch_lat) * np.cos(
        aim_lat
    ) * np.cos(lon_diff)
    return np.arctan2(north, east)


def haversine_distance(start, end):
    """
    Calculate the haversine distance between two points (lat, lon in radians).

    INPUTS:
    ----------
        start: tuple of (lat, lon) in radians
        end: tuple of (lat, lon) in radians
    OUTPUTS:
    ----------
        distance: distance in meters
    """
    launch_lat, launch_lon = start
    aim_lat, aim_lon = end
    a = (
        np.sin((aim_lat - launch_lat) / 2) ** 2
        + np.cos(launch_lat) * np.cos(aim_lat) * np.sin((aim_lon - launch_lon) / 2) ** 2
    )
    angular_distance = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
    distance = EARTH_RADIUS * angular_distance
    return distance


def get_location(bearing, distance, start):
    """
    Calculate the end location (lat, lon in radians) given a start location (lat, lon in radians),
    a bearing (in radians), and a distance (in meters).

    INPUTS:
    ----------
        bearing: bearing in radians
        distance: distance in meters
        start: tuple of (lat, lon) in radians
    OUTPUTS:
    ----------
        tuple of (aim_lat, aim_lon) in radians
    """
    bearing = -(bearing - np.pi / 2)
    launch_lat = start[0]
    launch_lon = start[1]
    angular_distance = distance / EARTH_RADIUS
    aim_lat = np.arcsin(
        np.sin(launch_lat) * np.cos(angular_distance)
        + np.cos(launch_lat) * np.sin(angular_distance) * np.cos(bearing)
    )
    aim_lon = launch_lon + np.arctan2(
        np.sin(bearing) * np.sin(angular_distance) * np.cos(launch_lat),
        np.cos(angular_distance) - np.sin(launch_lat) * np.sin(aim_lat),
    )
    return aim_lat, aim_lon


def transform_to_earth_coords(x, y, z, launchpoint):
    """
    Transform the cartesian x, y, z impact points to the lat lon points they would
    have had if following a great circle projected to the surface of the Earth and
    launched from the launchpoint instead of 0, 0.

    INPUTS:
    ----------
        x (np.ndarray): x coordinate in meters.
        y (np.ndarray): y coordinate in meters.
        z (np.ndarray): z coordinate in meters.
        launchpoint: tuple of (lat, lon) in radians
    OUTPUTS:
    ----------
        tuple of (lat, lon) in radians
    """
    launch_lat, launch_lon = launchpoint
    lat_from_origin, long_from_origin = cart2sphere(x, y, z)
    bearing = calc_bearing((0, 0), (lat_from_origin, long_from_origin))
    distance = haversine_distance((0, 0), (lat_from_origin, long_from_origin))
    lat, lon = get_location(bearing, distance, (launch_lat, launch_lon))
    return lat, lon


def get_impact_data(run_params=None, impact_data=None):
    """
    Get the x y z locations of the impact points from the run parameters or data.

    """
    if impact_data is None:
        # print error if the paths are not found
        if not os.path.exists(run_params["impact_data_path"]):
            print(f"Impact data file {run_params['impact_data_path']} not found")
            return

        impact_data = np.loadtxt(
            run_params.get("impact_data_path"), delimiter=",", skiprows=1
        )
        impact_x = impact_data[:, 1]
        impact_y = impact_data[:, 2]
        impact_z = impact_data[:, 3]
    elif isinstance(impact_data, pd.DataFrame):
        # Convert the DataFrame to a numpy array
        impact_x = impact_data["x"].values
        impact_y = impact_data["y"].values
        impact_z = impact_data["z"].values
    return impact_x, impact_y, impact_z


def get_local_impact(run_params, impact_data):
    """
    Get the local impact coordinates

    INPUTS:
    ----------
        impact_data: numpy.ndarray
            The impact data.
        run_params_struct: runparams
            The run parameters.
    OUTPUTS:
    ----------
        impact_x_local: numpy.ndarray
            The x coordinates of the impact points in local tangent plane coordinates.
        impact_y_local: numpy.ndarray
            The y coordinates of the impact points in local tangent plane coordinates.
    """
    # Get longitude and latitude of aimpoint and launchpoint
    aimpoint_lat, aimpoint_lon = cart2sphere(
        run_params["x_aim"], run_params["y_aim"], run_params["z_aim"]
    )
    launch_lat, launch_lon = cart2sphere(
        run_params["x_launch"], run_params["y_launch"], run_params["z_launch"]
    )

    impact_x, impact_y, impact_z = get_impact_data(run_params, impact_data)

    lat, lon = transform_to_earth_coords(
        impact_x, impact_y, impact_z, (launch_lat, launch_lon)
    )

    impact_x, impact_y, impact_z = sphere2cart(EARTH_RADIUS, lon, lat)
    # get vector relative to aimpoint
    impact_x = impact_x - run_params["x_aim"]
    impact_y = impact_y - run_params["y_aim"]
    impact_z = impact_z - run_params["z_aim"]

    # convert impact data to local tangent plane coordinates
    print(f"aimpont_lat: {aimpoint_lat}, aimpoint_lon: {aimpoint_lon}")
    print(f"launch_lat: {launch_lat}, launch_lon: {launch_lon}")
    impact_x_local = -np.sin(aimpoint_lon) * impact_x + np.cos(aimpoint_lon) * impact_y
    impact_y_local = (
        -np.sin(aimpoint_lat) * np.cos(aimpoint_lon) * impact_x
        - np.sin(aimpoint_lat) * np.sin(aimpoint_lon) * impact_y
        + np.cos(aimpoint_lat) * impact_z
    )

    return impact_x_local, impact_y_local


def get_cep_miss_distance_from_local_impact(impact_x_local, impact_y_local):
    """
    Calculate the circular error probable (CEP) from the local impact coordinates.

    INPUTS:
    ----------
        impact_x_local: numpy.ndarray
            The x coordinates of the impact points in local tangent plane coordinates.
        impact_y_local: numpy.ndarray
            The y coordinates of the impact points in local tangent plane coordinates.
    OUTPUTS:
    ----------
        miss_distance: numpy.ndarray
            The miss distances of the impact points.
        cep: double
            The circular error probable.
    """
    miss_distance = np.sqrt(impact_x_local**2 + impact_y_local**2)
    cep = np.percentile(miss_distance, 50)
    return miss_distance, round(cep, 2)


def get_cep(run_params, impact_data):
    """
    Calculate the circular error probable (CEP) from the impact data.

    INPUTS:
    ----------
        impact_data: numpy.ndarray
            The impact data.
        run_params_struct: runparams
            The run parameters.
    OUTPUTS:
    ----------
        cep: double
            The circular error probable.
    """
    impact_x_local, impact_y_local = get_local_impact(run_params, impact_data)
    _, cep = get_cep_miss_distance_from_local_impact(impact_x_local, impact_y_local)
    return cep


def check_config_exists(config_path):
    """
    Check if the configuration file exists.

    Params:
        config_path (str): Path to the configuration file.

    Returns:
        bool: True if the file exists, False otherwise.
    """
    return os.path.isfile(config_path)


def get_run_params(config_path=None):
    """
    Get run params dict from the config file. If no config file is provided,
    the default config file is used.

    INPUTS
    -------
        config_path (str): Path to the configuration file. If None, the default
            configuration file is used.
    OUTPUTS
    -------
        run_params (dict): Dictionary containing the run parameters.
    """
    retrieving_default = config_path is None
    if retrieving_default:
        config_path = str(
            importlib.resources.files("pytrajlib.config").joinpath("default.toml")
        )
    if not check_config_exists(config_path):
        raise FileNotFoundError(f"The input file {config_path} does not exist.")
    config_parser = configparser.ConfigParser()
    config_parser.read(config_path)
    run_params = {
        key: to_python_type(value)
        for section in config_parser.sections()
        for key, value in config_parser.items(section)
    }
    if retrieving_default:
        # Override the default atm_profile_path because atmprofiles.txt does not
        # have a stable fixed path when the script is run as part of a package.
        run_params["atm_profile_path"] = str(
            importlib.resources.files("pytrajlib.config").joinpath("atmprofiles.txt")
        )
    return run_params
