# Script to analyze relationship between CEP and pitch excitations
import matplotlib.pyplot as plt
import numpy as np
import scienceplots

from pytrajlib import simulation, utils

# Avoid an unused import warning for scienceplots
assert scienceplots
plt.style.use(["science"])

params = {
    "axes.labelsize": 8,
    "font.size": 8,
    "font.family": "serif",
    "legend.fontsize": 8,
    "xtick.labelsize": 10,
    "ytick.labelsize": 10,
    "text.usetex": True,
}

plt.rcParams.update(params)


heights = np.logspace(0, np.log10(5e5), 100)
reentry_velocities = [1e2, 1e3, 5e3, 7.5e3, 1e4]

run_params = simulation.get_run_params("input/reentry.toml")
# run_params["num_runs"] = 100
# run_params["time_step_reentry"] = 0.01
run_params["step_acc_mag"] = -1
run_params["step_acc_dur"] = -1
run_params["nav_gain"] = 5

aoa_perts = np.linspace(0, np.radians(5), 5)
for aoa_pert in aoa_perts:
    ceps = []
    for height in heights:
        print(
            f"Running simulation for height {height:.2e} m and aoa perturbation {aoa_pert:.2e} rad"
        )
        run_params["step_acc_hgt"] = height
        run_params["aoa_pert"] = aoa_pert

        impact_data = simulation.run(run_params)

        cep = utils.get_cep(run_params, impact_data)
        ceps.append(cep)

    plt.plot(heights, ceps, label=f"aoa_pert={np.degrees(aoa_pert):.2f}°")


plt.xscale("log")
plt.yscale("log")
plt.xlabel("Altitude of excitation (m)")
plt.ylabel("CEP (m)")
plt.title("Pitching Mode Excitation")
plt.legend()
plt.tight_layout()
filepath = "./output/" + f"pitch-gain{run_params['nav_gain']}.pdf"
plt.savefig(filepath)
print("Saved to", filepath)
