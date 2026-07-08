from typing import Tuple

import numpy as np
import pandas as pd


def get_local_impact(
    impact_df: pd.DataFrame,
    aimpoint: Tuple[float, float, float],
) -> Tuple[np.ndarray, np.ndarray]:
    r"""
    The distance between an impact point $(x, y, z)$ and the aimpoint $(x_\text{aim}, y_\text{aim}, z_\text{aim})$ in a local tangent plane is found by converting the miss distances to local East-North-Up coordinates where $\theta$ is the aimpoint longitude, $\phi$ is the aimpoint latitude, $x_\text{local}$ is the east component, $y_\text{local}$ is the north component, and the up component is discarded:
    
    $$
    \begin{aligned}
    x_\text{local} &= -(x - x_\text{aim})\sin\theta + (y - y_\text{aim})\cos\theta \\\\
    y_\text{local} &= -(x - x_\text{aim})\sin\phi\cos\theta - (y - y_\text{aim})\sin\phi\sin\theta + (z - z_\text{aim})\cos\phi.
    \end{aligned}
    $$

    Args:
        impact_df: DataFrame containing impact coordinates with columns ``x``, ``y``, and ``z``.
        aimpoint: Aimpoint coordinates ``(x_aim, y_aim, z_aim)`` in the same global frame as
            ``impact_df``.

    Returns:
        ``(impact_x_local, impact_y_local)`` arrays containing east and north local offsets for each impact sample.
    """

    x_aim, y_aim, z_aim = aimpoint

    # Extract coordinates
    impact_x = impact_df["x"].values
    impact_y = impact_df["y"].values
    impact_z = impact_df["z"].values

    # Calculate aimpoint angles
    aimpoint_lon = np.arctan2(y_aim, x_aim)
    aimpoint_lat = np.arctan2(z_aim, np.sqrt(x_aim**2 + y_aim**2))

    # Offset from aimpoint
    impact_x_rel = impact_x - x_aim
    impact_y_rel = impact_y - y_aim
    impact_z_rel = impact_z - z_aim

    # Convert to local tangent plane coordinates
    impact_x_local = (
        -np.sin(aimpoint_lon) * impact_x_rel + np.cos(aimpoint_lon) * impact_y_rel
    )
    impact_y_local = (
        -np.sin(aimpoint_lat) * np.cos(aimpoint_lon) * impact_x_rel
        - np.sin(aimpoint_lat) * np.sin(aimpoint_lon) * impact_y_rel
        + np.cos(aimpoint_lat) * impact_z_rel
    )
    return impact_x_local, impact_y_local


def get_miss_distance(
    impact_df: pd.DataFrame,
    aimpoint: Tuple[float, float, float],
) -> np.ndarray:
    r"""
    The miss distance as a function of the downrange and crossrange error is

    $$
    d = \sqrt{x_\text{local}^2 + y_\text{local}^2}
    $$

    The circular error probable (CEP) is the median miss distance.

    Args:
        impact_df: DataFrame containing impact coordinates with columns ``x``, ``y``, and ``z``.
        aimpoint: Aimpoint coordinates ``(x_aim, y_aim, z_aim)`` in the same global frame as
            ``impact_df``.

    Returns:
        Per-sample miss distance computed as ``sqrt(impact_x_local**2 + impact_y_local**2)``.
    """

    impact_x_local, impact_y_local = get_local_impact(impact_df, aimpoint)

    # Calculate statistics
    miss_distance = np.sqrt(impact_x_local**2 + impact_y_local**2)
    return miss_distance
