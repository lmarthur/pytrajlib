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
            t = impact_data.impact_states[i].t,
            x = impact_data.impact_states[i].x,
            y = impact_data.impact_states[i].y,
            z = impact_data.impact_states[i].z,
            vx = impact_data.impact_states[i].vx,
            vy = impact_data.impact_states[i].vy,
            vz = impact_data.impact_states[i].vz,
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
    north = (
        np.cos(launch_lat) * np.sin(aim_lat)
        - np.sin(launch_lat) * np.cos(aim_lat) * np.cos(lon_diff)
    )
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
        + np.cos(launch_lat) * np.cos(aim_lat)
        * np.sin((aim_lon - launch_lon) / 2) ** 2
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