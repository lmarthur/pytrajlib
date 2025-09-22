# Script to analyze relationship between CEP and reentry angle and velocity
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


reentry_angles = np.linspace(np.pi / 2, np.pi / 2 - 0.1, 100)
reentry_velocities = [1e2, 1e3, 5e3, 7.5e3, 1e4]

run_params = simulation.get_run_params("input/reentry.toml")
# run_params["num_runs"] = 100
# run_params["time_step_reentry"] = 0.01

for reentry_vel in reentry_velocities:
    ceps = []
    for reentry_angle in reentry_angles:
        print(
            f"Running simulation for reentry angle {reentry_angle:.2e} rad and velocity {reentry_vel:.2e} m/s"
        )
        run_params["reentry_vel"] = reentry_vel
        run_params["reentry_angle"] = reentry_angle

        impact_data = simulation.run(run_params)

        cep = utils.get_cep(run_params, impact_data)
        ceps.append(cep)

    plt.plot(reentry_angles, ceps, label=f"v={reentry_vel / 1e3:.1f} km/s")

# plt.xscale("log")
plt.yscale("log")
plt.xlabel("Reentry Angle (rad)")
plt.ylabel("CEP (m)")
plt.title("Sensitivity to Reentry Angle and Velocity")
plt.legend()
plt.tight_layout()
path = "./output/" + "angle_vel.pdf"
plt.savefig(path)
print("Saved to", path)
