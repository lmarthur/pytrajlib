import os

import pandas as pd


def save_mean_atm_profile(atm_profile_path):
    """
    Calculate the mean atmospheric profile from the given atmospheric profile file
    and save it to a file named 'mean_atm.txt' in the same directory.

    Params:
        atm_profile_path (str): Path to the atmospheric profile file.

    Returns:
        path to the saved mean atmospheric profile file.
    """
    atm = pd.read_csv(atm_profile_path, header=None, sep=r"\s+")
    folder_path = os.path.dirname(atm_profile_path)
    mean_atm_path = folder_path + "/mean_atm.txt"

    # check if the file exists
    if os.path.isfile(mean_atm_path):
        print("Mean EarthGram file exists")
        return

    print("Creating mean EarthGram file")

    mean_atm = atm.groupby(1)[[2, 3, 4, 5]].apply(
        lambda altitude_df: altitude_df.mean(axis=0)
    )

    mean_atm.to_csv(mean_atm_path, header=None, sep=" ")
    return mean_atm_path


if __name__ == "__main__":
    save_mean_atm_profile("input/atmprofiles.txt")
