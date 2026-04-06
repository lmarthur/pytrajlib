import numpy as np


def get_local_impact(
    impact_df,
    aimpoint,
):
    r"""
    Get distance from aim point to local ENU where $\phi$ is the aimpoint longitude, $\theta$ is the aimpoint latitude, $x_\text{local}$ is the east component, $y_\text{local}$ is the north component, and the up component is discarded:
    
    $$
    \begin{aligned}
    x_\text{local} &= -\sin\phi(x - x_\text{aim}) + \cos\phi(y - y_\text{aim}) \\\\
    y_\text{local} &= -\sin\theta\cos\phi(x - x_\text{aim}) - \sin\theta\sin\phi(y - y_\text{aim}) + \cos\theta(z - z_\text{aim}).
    \end{aligned}
    $$

    Args:
        impact_df : pandas.DataFrame
            DataFrame containing impact coordinates with columns ``x``, ``y``, and ``z``.
        aimpoint : tuple[float, float, float]
            Aimpoint coordinates ``(x_aim, y_aim, z_aim)`` in the same global frame as
            ``impact_df``.

    Returns:
        tuple[numpy.ndarray, numpy.ndarray]
            ``(impact_x_local, impact_y_local)`` arrays containing east and north local
            offsets for each impact sample.
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
    impact_df,
    aimpoint,
):
    r"""
    Compute scalar miss distance from local east/north impact offsets.

    $$
    d = \sqrt{x_\text{local}^2 + y_\text{local}^2}
    $$

    Args:
        impact_df : pandas.DataFrame
            DataFrame containing impact coordinates with columns ``x``, ``y``, and ``z``.
        aimpoint : tuple[float, float, float]
            Aimpoint coordinates ``(x_aim, y_aim, z_aim)`` in the same global frame as
            ``impact_df``.

    Returns:
        numpy.ndarray
            Per-sample miss distance computed as
            ``sqrt(impact_x_local**2 + impact_y_local**2)``.
    """

    impact_x_local, impact_y_local = get_local_impact(impact_df, aimpoint)

    # Calculate statistics
    miss_distance = np.sqrt(impact_x_local**2 + impact_y_local**2)
    return miss_distance
