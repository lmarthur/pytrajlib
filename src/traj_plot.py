# This script contains code to generate plots of the vehicle's trajectory and control surfaces.

import matplotlib.pyplot as plt
import numpy as np

EARTH_RADIUS_M = 6371e3

params = {
    "axes.labelsize": 18,
    "font.size": 18,
    "font.family": "serif",
    "legend.fontsize": 18,
    "xtick.labelsize": 18,
    "ytick.labelsize": 18,
}
plt.rcParams.update(params)


def traj_plot(run_path):
    """
    Function to plot the trajectory of the vehicle.
    """
    # load the trajectory data from the .txt file, skipping the first row
    traj_data = np.loadtxt(run_path + "trajectory.txt", delimiter=",", skiprows=1)

    true_t = traj_data[:, 0]
    true_mass = traj_data[:, 1]
    true_x = traj_data[:, 2]
    true_y = traj_data[:, 3]
    true_z = traj_data[:, 4]
    true_vx = traj_data[:, 5]
    true_vy = traj_data[:, 6]
    true_vz = traj_data[:, 7]
    true_ax_total = traj_data[:, 8]
    true_ay_total = traj_data[:, 9]
    true_az_total = traj_data[:, 10]
    est_x = traj_data[:, 11]
    est_y = traj_data[:, 12]
    est_z = traj_data[:, 13]
    est_vx = traj_data[:, 14]
    est_vy = traj_data[:, 15]
    est_vz = traj_data[:, 16]
    est_ax_total = traj_data[:, 17]
    est_ay_total = traj_data[:, 18]
    est_az_total = traj_data[:, 19]
    true_deflection_angle = traj_data[:, 20]
    true_alpha = traj_data[:, 21]
    est_deflection_angle = traj_data[:, 22]
    est_alpha = traj_data[:, 23]
    est_az_lift = traj_data[:, 25]

    true_altitude = (
        np.sqrt(np.square(true_x) + np.square(true_y) + np.square(true_z))
        - EARTH_RADIUS_M
    )
    est_altitude = (
        np.sqrt(np.square(est_x) + np.square(est_y) + np.square(est_z)) - EARTH_RADIUS_M
    )

    # position vs. time
    plt.figure(figsize=(10, 10))
    plt.plot(true_t, true_x, label="x")
    plt.plot(true_t, true_y, label="y")
    plt.plot(true_t, true_z, label="z")
    # plt.plot(true_t, est_x, label="x_est")
    # plt.plot(true_t, est_y, label="y_est")
    # plt.plot(true_t, est_z, label="z_est")
    plt.xlabel("Time (s)")
    plt.ylabel("Position (m)")
    plt.title("Position")
    plt.legend()
    plt.grid()
    plt.savefig(run_path + "position.pdf")
    plt.close()

    # position error
    plt.figure(figsize=(10, 10))
    plt.plot(true_t, true_x - est_x, label="x")
    plt.plot(true_t, true_y - est_y, label="y")
    plt.plot(true_t, true_z - est_z, label="z")
    plt.xlabel("Time (s)")
    plt.ylabel("Position Error (m)")
    plt.title("Position Error")
    plt.legend()
    plt.grid()
    plt.savefig(run_path + "position_error.pdf")
    plt.close()

    # orbit plot
    earth_radius = EARTH_RADIUS_M
    plt.figure(figsize=(10, 10))

    # add shaded region for Earth's atmosphere
    earth_atmosphere = plt.Circle(
        (0, 0), earth_radius + 200e3, color="lightblue", label="Atmosphere"
    )
    plt.gca().add_artist(earth_atmosphere)

    # plot the Earth
    earth = plt.Circle((0, 0), earth_radius, color="blue", label="Earth")
    plt.gca().add_artist(earth)
    # set range for x and y axes to 2*earth_radius
    plt.xlim(-1.2 * earth_radius, 1.5 * earth_radius)
    plt.ylim(-1.2 * earth_radius, 1.5 * earth_radius)

    # plot the vehicle's trajectory in the x-y plane
    plt.plot(true_x, true_y, "r", label="True Trajectory")
    # turn off the axis labels
    plt.axis("off")

    # plt.xlabel("x (m)")
    # plt.ylabel("y (m)")
    # plt.title("Position (x-y plane)")
    plt.savefig(run_path + "orbit.pdf")
    plt.close()

    # altitude vs. time
    plt.figure(figsize=(10, 10))
    plt.plot(true_t, true_altitude / 1000)
    plt.xlabel("Time (s)")
    plt.ylabel("Altitude (km)")
    # remove top and right spines
    plt.gca().spines["top"].set_visible(False)
    plt.gca().spines["right"].set_visible(False)
    # shade under the curve from 0 to 160 seconds
    plt.fill_between(
        true_t,
        true_altitude / 1000,
        0,
        where=(true_t < 188),
        color="lightblue",
        alpha=0.5,
    )
    # add "guided" label to shaded region with arrow
    # plt.annotate('Boost (INS)', xy=(188, 40), xytext=(500, 50), arrowprops=dict(facecolor='black', arrowstyle='->'))
    # add "ballistic phase"
    # plt.annotate('Ballistic Phase\n (No Control, GNSS)', xy=(1500, 1500), ha='center')
    # shade under the curve for altitude < 100 and t < 1000
    # plt.fill_between(true_t, true_altitude/1000, 0, where=(true_t > 2915), color='red', alpha=0.5)
    # plt.annotate('Reentry\n (INS)', xy=(2910, 40), xytext=(2200, 250), arrowprops=dict(facecolor='black', arrowstyle='->'), ha='center')
    plt.savefig(run_path + "altitude.pdf")
    plt.close()

    # altitude error
    plt.figure(figsize=(10, 10))
    plt.plot(true_t, true_altitude - est_altitude)
    plt.xlabel("Time (s)")
    plt.ylabel("Altitude Error (m)")
    plt.title("Altitude Error")
    plt.grid()
    plt.savefig(run_path + "altitude_error.pdf")
    plt.close()

    boost_mask = true_t <= 188
    midcourse_mask = (true_t > 188) & (true_altitude >= 1e5)
    reentry_mask = (true_t > 188) & (true_altitude < 1e5)

    # velocity vs. time (boost)
    plt.figure(figsize=(10, 10))
    plt.plot(true_t[boost_mask], true_vx[boost_mask], label="vx")
    plt.plot(true_t[boost_mask], true_vy[boost_mask], label="vy")
    plt.plot(true_t[boost_mask], true_vz[boost_mask], label="vz")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")
    plt.title("Velocity (Boost)")
    plt.legend()
    plt.grid()
    plt.savefig(run_path + "velocity_boost.pdf")
    plt.close()

    # velocity vs. time (midcourse)
    plt.figure(figsize=(10, 10))
    plt.plot(true_t[midcourse_mask], true_vx[midcourse_mask], label="vx")
    plt.plot(true_t[midcourse_mask], true_vy[midcourse_mask], label="vy")
    plt.plot(true_t[midcourse_mask], true_vz[midcourse_mask], label="vz")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")
    plt.title("Velocity (Midcourse)")
    plt.legend()
    plt.grid()
    plt.savefig(run_path + "velocity_midcourse.pdf")
    plt.close()

    # velocity vs. time (reentry)
    plt.figure(figsize=(10, 10))
    plt.plot(true_t[reentry_mask], true_vx[reentry_mask], label="vx")
    plt.plot(true_t[reentry_mask], true_vy[reentry_mask], label="vy")
    plt.plot(true_t[reentry_mask], true_vz[reentry_mask], label="vz")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")
    plt.title("Velocity (Reentry)")
    plt.legend()
    plt.grid()
    plt.savefig(run_path + "velocity_reentry.pdf")
    plt.close()

    # velocity error (boost)
    plt.figure(figsize=(10, 10))
    plt.plot(true_t[boost_mask], (true_vx - est_vx)[boost_mask], label="vx")
    plt.plot(true_t[boost_mask], (true_vy - est_vy)[boost_mask], label="vy")
    plt.plot(true_t[boost_mask], (true_vz - est_vz)[boost_mask], label="vz")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity Error (m/s)")
    plt.title("Velocity Error (Boost)")
    plt.legend()
    plt.grid()
    plt.savefig(run_path + "velocity_error_boost.pdf")
    plt.close()

    # velocity error (midcourse)
    plt.figure(figsize=(10, 10))
    plt.plot(true_t[midcourse_mask], (true_vx - est_vx)[midcourse_mask], label="vx")
    plt.plot(true_t[midcourse_mask], (true_vy - est_vy)[midcourse_mask], label="vy")
    plt.plot(true_t[midcourse_mask], (true_vz - est_vz)[midcourse_mask], label="vz")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity Error (m/s)")
    plt.title("Velocity Error (Midcourse)")
    plt.legend()
    plt.grid()
    plt.savefig(run_path + "velocity_error_midcourse.pdf")
    plt.close()

    # velocity error (reentry)
    plt.figure(figsize=(10, 10))
    plt.plot(true_t[reentry_mask], (true_vx - est_vx)[reentry_mask], label="vx")
    plt.plot(true_t[reentry_mask], (true_vy - est_vy)[reentry_mask], label="vy")
    plt.plot(true_t[reentry_mask], (true_vz - est_vz)[reentry_mask], label="vz")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity Error (m/s)")
    plt.title("Velocity Error (Reentry)")
    plt.legend()
    plt.grid()
    plt.savefig(run_path + "velocity_error_reentry.pdf")
    plt.close()

    # mass vs. time
    thrust_mask = true_t <= 200
    plt.figure(figsize=(10, 10))
    plt.plot(true_t[thrust_mask], true_mass[thrust_mask])
    plt.xlabel("Time (s)")
    plt.ylabel("Mass (kg)")
    plt.title("Mass")
    plt.grid()
    plt.savefig(run_path + "mass.png")
    plt.close()

    # acceleration vs. time
    plt.figure(figsize=(10, 10))
    plt.plot(true_t, true_ax_total, label="ax")
    plt.plot(true_t, true_ay_total, label="ay")
    plt.plot(true_t, true_az_total, label="az")
    plt.xlabel("Time (s)")
    plt.ylabel("Acceleration (m/s^2)")
    plt.title("Acceleration")
    plt.legend()
    plt.grid()
    plt.savefig(run_path + "acceleration.pdf")
    plt.close()

    # acceleration error
    plt.figure(figsize=(10, 10))
    plt.plot(true_t, true_ax_total - est_ax_total, label="ax")
    plt.plot(true_t, true_ay_total - est_ay_total, label="ay")
    plt.plot(true_t, true_az_total - est_az_total, label="az")
    plt.xlabel("Time (s)")
    plt.ylabel("Acceleration Error (m/s^2)")
    plt.title("Acceleration Error")
    plt.legend()
    plt.grid()
    plt.savefig(run_path + "acceleration_error.pdf")
    plt.close()

    # lift acceleration
    # plot deflection angle and angle of attack vs. time during reentry
    plt.figure(figsize=(10, 10))
    plt.plot(
        true_t[(true_altitude < 1e5) & (true_t > 200)],
        true_deflection_angle[(true_altitude < 1e5) & (true_t > 200)] * 180 / np.pi,
        label="Deflection Angle",
    )
    plt.plot(
        true_t[(true_altitude < 1e5) & (true_t > 200)],
        true_alpha[(true_altitude < 1e5) & (true_t > 200)] * 180 / np.pi,
        label="Angle of Attack",
    )
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (degrees)")
    plt.title("Control Surface Deflection and Angle of Attack During Reentry")
    plt.legend()
    plt.grid()
    plt.savefig(run_path + "deflection_and_aoa.pdf")
    plt.close()

    # plot y and z position vs. altitude
    plt.figure(figsize=(10, 10))
    plt.plot(500000 - true_altitude, true_y, label="y")
    plt.plot(500000 - true_altitude, true_z, label="z")
    plt.xlabel("Altitude")
    plt.ylabel("Position (m)")
    # no x axis ticks
    plt.xticks([])
    plt.title("Lateral Position vs. Altitude")
    plt.legend()
    plt.grid()
    plt.savefig(run_path + "position_vs_altitude.pdf")
    plt.close()
