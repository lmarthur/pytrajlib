import numpy as np
import matplotlib.pyplot as plt
import scienceplots

plt.style.use(['science'])
plt.style.use(['no-latex'])

# Import mean atmosphere
mean_atm = np.loadtxt("src/pytrajlib/config/mean_atm.txt")

# Import batch of atmospheric profiles
atm_profiles = np.loadtxt("src/pytrajlib/config/atmprofiles.txt")

# Calculate the standard deviation of the atmospheric profiles
# First, reformat the data by iterating over the profiles
# Get the number of distinct values of atm_profiles[:, 0]
profile_numbers = np.unique(atm_profiles[:, 0])
num_profiles = len(profile_numbers)
altitudes = np.unique(atm_profiles[:, 1])
num_altitudes = len(altitudes)

std_values = np.zeros((num_altitudes))  # Altitude, mean density, std density
for i in range(num_profiles):
    density_values = atm_profiles[atm_profiles[:, 1] == i, 2]
    # Calculate the mean and standard deviation of the density values
    mean_density = np.mean(density_values)
    std_density = np.std(density_values)
    std_values[i] = std_density

plt.figure(figsize=(5,5))
ax = plt.gca()

params = {
    'axes.labelsize': 8,
    'font.size': 8,
    'font.family': 'serif',
    'legend.fontsize': 8,
    'xtick.labelsize': 10,
    'ytick.labelsize': 10,
    # 'text.usetex': True,
}

plt.rcParams.update(params)
colors = plt.cm.viridis(np.linspace(0, 1, 7))

# Plot mean density
plt.plot(mean_atm[:, 1], mean_atm[:, 0], label="Mean Density")
plt.fill_betweenx(mean_atm[:, 0], mean_atm[:, 1]-std_values, mean_atm[:, 1]+std_values, alpha=0.2, label="Std Dev")
plt.xscale("log")
plt.xlabel("Pressure (Pa)")
plt.ylabel("Altitude (km)")
plt.title("Mean Atmospheric Pressure")
# Add legend
plt.legend()
plt.tight_layout()
plt.savefig("output/mean_density.jpg", dpi=1000)
plt.close()

# Now, repeat this for the velocity profile
std_values = np.zeros((num_altitudes))  # Altitude, mean density, std density
for i in range(num_profiles):
    wind_values = np.sqrt(np.square(atm_profiles[atm_profiles[:, 1] == i, 3]) + np.square(atm_profiles[atm_profiles[:, 1] == i, 4]) + np.square(atm_profiles[atm_profiles[:, 1] == i, 5]))

    mean_wind = np.mean(wind_values)
    std_wind = np.std(wind_values)
    std_values[i] = std_wind

# Plot mean atmosphere
plt.plot(mean_atm[:, 1], mean_atm[:, 0], label="Mean Windspeed")
plt.fill_betweenx(mean_atm[:, 0], mean_atm[:, 1]-std_values, mean_atm[:, 1]+std_values, alpha=0.2, label="Std Dev")
plt.xlabel("Windspeed (m/s)")
plt.ylabel("Altitude (km)")
plt.title("Mean Atmospheric Windspeed")
# Add legend
plt.legend()
plt.tight_layout()
plt.savefig("output/mean_winds.jpg", dpi=1000)
plt.close()