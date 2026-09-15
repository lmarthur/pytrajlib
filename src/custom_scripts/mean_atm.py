import importlib.resources
import os

import pandas as pd

DATA_COLUMNS = [
    "density_kg_m3",
    "v_meridional_m_s",
    "v_zonal_m_s",
    "v_vertical_m_s",
]


def save_mean_atm_profile(atm_profile_path, overwrite=True):
    """
    Calculate the mean atmospheric profile from the given atmospheric profile file
    and save it to a file named 'mean_atm.txt' in the same directory.

    Params:
        atm_profile_path (str): Path to the atmospheric profile CSV, with a header row
            and columns profile_num, altitude_km, and the entries of DATA_COLUMNS.
        overwrite (bool): Whether to replace an existing mean_atm.txt.

    Returns:
        path to the saved mean atmospheric profile file.
    """
    folder_path = os.path.dirname(atm_profile_path)
    mean_atm_path = os.path.join(folder_path, "mean_atm.txt")

    if os.path.isfile(mean_atm_path) and not overwrite:
        print("Mean EarthGram file exists")
        return mean_atm_path

    # usecols drops the empty column created by the header's trailing comma
    atm = pd.read_csv(
        atm_profile_path, usecols=["profile_num", "altitude_km", *DATA_COLUMNS]
    )
    num_profiles = atm["profile_num"].nunique()
    print(f"Creating mean EarthGram file from {num_profiles} profiles")

    mean_atm = atm.groupby("altitude_km")[DATA_COLUMNS].mean()

    # Space-delimited with no header: altitude followed by the four data columns,
    # as read by init_mean_atm_data in atmosphere.h
    mean_atm.to_csv(mean_atm_path, header=False, sep=" ")
    return mean_atm_path


if __name__ == "__main__":
    save_mean_atm_profile(
        str(importlib.resources.files("pytrajlib.config").joinpath("atmprofiles.csv"))
    )
