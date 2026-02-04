from dataclasses import dataclass

import numpy as np
import pandas as pd

# Earth's mean radius in meters
EARTH_RADIUS = 6371e3


@dataclass
class Result:
    impact_df: pd.DataFrame
    aimpoint: np.ndarray
    name: str


def parse_impact_result(results, N: int, name: str) -> Result:
    columns = ["t", "x", "y", "z"]
    df = pd.DataFrame(columns=columns)

    times = [results.results[i].t for i in range(N)]
    df.t = times

    positions = [results.results[i].impact_event.position for i in range(N)]
    df.x = [positions[i].x for i in range(N)]
    df.y = [positions[i].y for i in range(N)]
    df.z = [positions[i].z for i in range(N)]

    aimpoint = np.array([results.aimpoint.x, results.aimpoint.y, results.aimpoint.z])
    result = Result(impact_df=df, aimpoint=aimpoint, name=name)
    return result


def cartcoords_to_sphercoords(cart_coords: np.ndarray) -> np.ndarray:
    """
    Convert Cartesian coordinates to spherical coordinates.

    Args:
        cart_coords: Cartesian coordinates with shape (..., 3) where the last
            dimension contains [x, y, z].

    Returns:
        Spherical coordinates with shape (..., 3) where the last dimension
        contains [r, lat, lon].
    """
    # Extract components
    x = cart_coords[..., 0]
    y = cart_coords[..., 1]
    z = cart_coords[..., 2]

    # Calculate radial coordinate
    r = np.sqrt(x**2 + y**2 + z**2)

    # Calculate longitudinal coordinate
    lon = np.arctan2(y, x)

    # Calculate latitudinal coordinate
    lat = np.arctan(z / np.sqrt(x**2 + y**2))

    # Stack results
    return np.stack([r, lat, lon], axis=-1)


def haversine_distance(start: tuple, end: tuple) -> float:
    """Calculate the haversine distance between two points on Earth.

    Uses the haversine formula to compute the great-circle distance between
    two points specified by latitude and longitude in radians.

    Args:
        start: Tuple of (latitude, longitude) in radians for the starting point.
        end: Tuple of (latitude, longitude) in radians for the ending point.

    Returns:
        Distance between the two points in meters.

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


def get_local_impact(result: Result) -> np.ndarray:
    """Get x, y coordinates of impact points in local tangent plane coordinates.

    Args:
        result: Result object containing impact data and aimpoint.

    Returns:
        Array of shape (2, N) containing [impact_x_local, impact_y_local] coordinates.
    """
    # Get longitude and latitude of aimpoint
    sphere_coords = cartcoords_to_sphercoords(result.aimpoint)
    _, aimpoint_lat, aimpoint_lon = sphere_coords

    # Get impact positions
    positions = result.impact_df[["x", "y", "z"]].values

    # Get vector relative to aimpoint
    impact = positions - result.aimpoint
    impact_x, impact_y, impact_z = impact.T

    # Convert impact data to local tangent plane coordinates
    impact_x_local = -np.sin(aimpoint_lon) * impact_x + np.cos(aimpoint_lon) * impact_y
    impact_y_local = (
        -np.sin(aimpoint_lat) * np.cos(aimpoint_lon) * impact_x
        - np.sin(aimpoint_lat) * np.sin(aimpoint_lon) * impact_y
        + np.cos(aimpoint_lat) * impact_z
    )
    return np.array([impact_x_local.flatten(), impact_y_local.flatten()])


def cep_from_local_impact(impact_local, quantile=0.5):
    """
    Calculate the circular error probable (CEP) from the local impact coordinates.
    """
    miss_distance = np.linalg.norm(impact_local, axis=0)
    is_finite = np.isfinite(miss_distance)
    cep = np.quantile(miss_distance[is_finite], quantile)
    return miss_distance[is_finite], cep
