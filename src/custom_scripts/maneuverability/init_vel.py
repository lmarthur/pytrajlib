# Script to analyze relationship between CEP and perturbations to lift coefficient
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


init_vel_errs = np.logspace(-4, 3, 100)

run_params = simulation.get_run_params("input/reentry.toml")
# run_params["num_runs"] = 100
# run_params["time_step_reentry"] = 0.01

ceps = []
for err in init_vel_errs:
    print(f"Running simulation for initial_vel_error {err:.2e}")
    run_params["initial_vel_error"] = err

    impact_data = simulation.run(run_params)

    cep = utils.get_cep(run_params, impact_data)
    ceps.append(cep)

plt.plot(init_vel_errs, ceps)

plt.xscale("log")
plt.yscale("log")
plt.xlabel("Initial Velocity Error (m/s)")
plt.ylabel("CEP (m)")
plt.title("Sensitivity to Initial Velocity Error")
plt.tight_layout()
filepath = "./output/" + "init-vel-err.pdf"
plt.savefig(filepath)
print("Saved to", filepath)
