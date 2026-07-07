import importlib.resources
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import scienceplots

# Avoid unused import warning by asserting scienceplots
assert scienceplots

plt.style.use(["science"])
plt.style.use(["no-latex"])


def save_atm_plots(
    output_dir: Path | str | None = None, dpi: int = 1000
) -> tuple[Path, Path]:
    """Generate and save mean atmosphere density and wind plots.

    Args:
        output_dir: directory to save the plots (created if missing). Defaults to
            current working directory.
        dpi: image DPI to use when saving.

    Returns:
        (density_path, wind_path): Paths to the saved files.
    """
    output_dir = Path(output_dir) if output_dir is not None else Path.cwd()
    output_dir.mkdir(parents=True, exist_ok=True)

    params = {
        "axes.labelsize": 8,
        "font.size": 8,
        "font.family": "serif",
        "legend.fontsize": 8,
        "xtick.labelsize": 10,
        "ytick.labelsize": 10,
        # 'text.usetex': True,
    }
    plt.rcParams.update(params)

    mean_atm_path = str(
        importlib.resources.files("pytrajlib.config").joinpath("mean_atm.txt")
    )
    atm_profiles_path = str(
        importlib.resources.files("pytrajlib.config").joinpath("atmprofiles.csv")
    )

    # Import mean atmosphere
    mean_atm = np.loadtxt(mean_atm_path)

    # Import batch of atmospheric profiles
    atm_profiles = np.loadtxt(atm_profiles_path)

    altitudes = np.unique(atm_profiles[:, 1])
    num_altitudes = len(altitudes)

    density_values = []
    wind_values = []
    for i in range(num_altitudes):
        # values at altitude i for all profiles
        density = atm_profiles[atm_profiles[:, 1] == i, 2]
        wind = np.sqrt(
            np.square(atm_profiles[atm_profiles[:, 1] == i, 3])
            + np.square(atm_profiles[atm_profiles[:, 1] == i, 4])
            + np.square(atm_profiles[atm_profiles[:, 1] == i, 5])
        )

        density_values.append(density)
        wind_values.append(wind)

    density_values = np.array(density_values)
    wind_values = np.array(wind_values)

    mean_density = np.mean(density_values, axis=1)
    std_density = np.std(density_values, axis=1)
    mean_wind = np.mean(wind_values, axis=1)
    std_wind = np.std(wind_values, axis=1)

    altitude = mean_atm[:, 0]

    # Plot mean density
    fig, ax = plt.subplots(figsize=(5, 5))
    ax.plot(mean_density, altitude, label="Mean Density")
    ax.fill_betweenx(
        altitude,
        mean_density - std_density,
        mean_density + std_density,
        alpha=0.2,
        label="Std Dev",
    )
    ax.set_xscale("log")
    ax.set_xlabel("Density (kg / m^3)")
    ax.set_ylabel("Altitude (km)")
    ax.set_title("Mean Atmospheric Density")
    ax.legend()
    fig.tight_layout()
    density_path = output_dir / "mean_density.jpg"
    fig.savefig(density_path, dpi=dpi)
    plt.close(fig)

    # Plot mean atmosphere
    fig, ax = plt.subplots(figsize=(5, 5))
    ax.plot(mean_wind, altitude, label="Mean Windspeed")
    ax.fill_betweenx(
        altitude,
        mean_wind - std_wind,
        mean_wind + std_wind,
        alpha=0.2,
        label="1 Standard Deviation",
    )
    ax.set_xlabel("Windspeed (m/s)")
    ax.set_ylabel("Altitude (km)")
    ax.set_title("Mean Atmospheric Windspeed")
    ax.legend()
    fig.tight_layout()
    wind_path = output_dir / "mean_winds.jpg"
    fig.savefig(wind_path, dpi=dpi)
    plt.close(fig)

    print(f"Saved to {density_path} and {wind_path}")
    return density_path, wind_path


if __name__ == "__main__":
    save_atm_plots(None)
