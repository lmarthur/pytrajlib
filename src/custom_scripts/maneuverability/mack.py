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


heights = np.logspace(0, np.log10(5e5), 10)
durations = np.logspace(-3, 2, 5)

run_params = simulation.get_run_params("input/reentry.toml")
# run_params["num_runs"] = 10
# run_params["time_step_reentry"] = 1e-7
run_params["step_acc_mag"] = -1


for dur in durations:
    ceps = []
    for height in heights:
        print(f"Running simulation for height {height:.2e} and duration {dur:.2e} s")
        run_params["step_acc_dur"] = dur
        run_params["step_acc_hgt"] = height

        impact_data = simulation.run(run_params)

        cep = utils.get_cep(run_params, impact_data)
        ceps.append(cep)

    plt.plot(heights, ceps, label=f"t={dur:.4f} s")

plt.xscale("log")
plt.yscale("log")
plt.xlabel("Altitude of excitation (m)")
plt.ylabel("CEP (m)")
plt.title("Second Mack Mode Excitation")
plt.legend()
plt.tight_layout()
filepath = "./output/" + "mack.pdf"
plt.savefig(filepath)
print("Saved to", filepath)
