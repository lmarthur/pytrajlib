import numpy as np


def get_local_impact(
    impact_df,
    aimpoint,
):

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

    impact_x_local, impact_y_local = get_local_impact(impact_df, aimpoint)

    # Calculate statistics
    miss_distance = np.sqrt(impact_x_local**2 + impact_y_local**2)
    return miss_distance
