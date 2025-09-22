import matplotlib.pyplot as plt
import numpy as np
import scienceplots

plt.style.use(["science"])
plt.style.use(["no-latex"])

plt.figure(figsize=(5, 5))
ax = plt.gca()

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
colors = plt.cm.viridis(np.linspace(0, 1, 7))


# Import mean atmosphere
mean_atm = np.loadtxt("src/pytrajlib/config/mean_atm.txt")

# Import batch of atmospheric profiles
atm_profiles = np.loadtxt("src/pytrajlib/config/atmprofiles.txt")

profile_numbers = np.unique(atm_profiles[:, 0])
num_profiles = len(profile_numbers)
altitudes = np.unique(atm_profiles[:, 1])
num_altitudes = len(altitudes)

std_values = np.zeros((num_altitudes))  # Altitude, mean density, std density
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
plt.plot(mean_density, altitude, label="Mean Density")
plt.fill_betweenx(
    altitude,
    mean_density - std_density,
    mean_density + std_density,
    alpha=0.2,
    label="Std Dev",
)
plt.xscale("log")
plt.xlabel("Density (kg / m^3)")
plt.ylabel("Altitude (km)")
plt.title("Mean Atmospheric Density")
plt.legend()
plt.tight_layout()
density_path = "output/mean_density.jpg"
plt.savefig(density_path, dpi=1000)
plt.close()

# Plot mean atmosphere
plt.plot(mean_wind, altitude, label="Mean Windspeed")
plt.fill_betweenx(
    altitude, mean_wind - std_wind, mean_wind + std_wind, alpha=0.2, label="Std Dev"
)
plt.xlabel("Windspeed (m/s)")
plt.ylabel("Altitude (km)")
plt.title("Mean Atmospheric Windspeed")
plt.legend()
plt.tight_layout()
wind_path = "output/mean_winds.jpg"
plt.savefig(wind_path, dpi=1000)
plt.close()

print(f"Saved to {density_path} and {wind_path}")
