# This script contains code to generate plots of the vehicle's trajectory and control surfaces. 

import matplotlib.pyplot as plt
import numpy as np

params = {
    'axes.labelsize': 18,
    'font.size': 18,
    'font.family': 'serif',
    'legend.fontsize': 18,
    'xtick.labelsize': 18,
    'ytick.labelsize': 18,
}
plt.rcParams.update(params)

def traj_plot(run_path):
    """
    Function to plot the trajectory of the vehicle.
    """
    # load the trajectory data from the .txt file, skipping the first row
    traj_data = np.loadtxt(run_path + "trajectory.txt", delimiter = ",", skiprows=1)

    true_t = traj_data[:,0]
    true_mass = traj_data[:,1]
    true_x = traj_data[:,2]
    true_y = traj_data[:,3]
    true_z = traj_data[:,4]
    true_vx = traj_data[:,5]
    true_vy = traj_data[:,6]
    true_vz = traj_data[:,7]
    true_ax_grav = traj_data[:,8]
    true_ay_grav = traj_data[:,9]
    true_az_grav = traj_data[:,10]
    true_ax_drag = traj_data[:,11]
    true_ay_drag = traj_data[:,12]
    true_az_drag = traj_data[:,13]
    true_ax_lift = traj_data[:,14]
    true_ay_lift = traj_data[:,15]
    true_az_lift = traj_data[:,16]
    true_ax_thrust = traj_data[:,17]
    true_ay_thrust = traj_data[:,18]
    true_az_thrust = traj_data[:,19]
    true_ax_total = traj_data[:,20]
    true_ay_total = traj_data[:,21]
    true_az_total = traj_data[:,22]
    est_x = traj_data[:,23]
    est_y = traj_data[:,24]
    est_z = traj_data[:,25]
    est_vx = traj_data[:,26]
    est_vy = traj_data[:,27]
    est_vz = traj_data[:,28]
    est_ax_grav = traj_data[:,29]
    est_ay_grav = traj_data[:,30]
    est_az_grav = traj_data[:,31]
    est_ax_drag = traj_data[:,32]
    est_ay_drag = traj_data[:,33]
    est_az_drag = traj_data[:,34]
    est_ax_lift = traj_data[:,35]
    est_ay_lift = traj_data[:,36]
    est_az_lift = traj_data[:,37]
    est_ax_thrust = traj_data[:,38]
    est_ay_thrust = traj_data[:,39]
    est_az_thrust = traj_data[:,40]
    est_ax_total = traj_data[:,41]
    est_ay_total = traj_data[:,42]
    est_az_total = traj_data[:,43]


    true_altitude = np.sqrt(np.square(true_x) + np.square(true_y) + np.square(true_z)) - 6371e3
    est_altitude = np.sqrt(np.square(est_x) + np.square(est_y) + np.square(est_z)) - 6371e3
    true_thrust_mag = true_ax_thrust + true_ay_thrust + true_az_thrust

    # position vs. time
    plt.figure(figsize=(10,10))
    plt.plot(true_t, true_x, label="x")
    plt.plot(true_t, true_y, label="y")
    plt.plot(true_t, true_z, label="z")
    plt.plot(true_t, est_x, label="x_est")
    plt.plot(true_t, est_y, label="y_est")
    plt.plot(true_t, est_z, label="z_est")
    plt.xlabel("Time (s)")
    plt.ylabel("Position (m)")
    plt.title("Position")
    plt.legend()
    plt.grid()
    plt.savefig(run_path + "position.pdf")
    plt.close()

    # position error
    plt.figure(figsize=(10,10))
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
    earth_radius = 6371e3
    plt.figure(figsize=(10,10))

    # add shaded region for Earth's atmosphere
    earth_atmosphere = plt.Circle((0, 0), earth_radius + 200e3, color='lightblue', label="Atmosphere")
    plt.gca().add_artist(earth_atmosphere)

    # plot the Earth
    earth = plt.Circle((0, 0), earth_radius, color='blue', label="Earth")
    plt.gca().add_artist(earth)
    # set range for x and y axes to 2*earth_radius
    plt.xlim(-1.2*earth_radius, 1.5*earth_radius)
    plt.ylim(-1.2*earth_radius, 1.5*earth_radius)

    # plot the vehicle's trajectory in the x-y plane
    plt.plot(true_x, true_y, 'r', label="True Trajectory")
    # turn off the axis labels
    plt.axis('off')

    # plt.xlabel("x (m)")
    # plt.ylabel("y (m)")
    # plt.title("Position (x-y plane)")
    plt.savefig(run_path + "orbit.pdf")
    plt.close()

    # altitude vs. time
    plt.figure(figsize=(10,10))
    plt.plot(true_t, true_altitude/1000)
    plt.xlabel("Time (s)")
    plt.ylabel("Altitude (km)")
    # remove top and right spines
    plt.gca().spines['top'].set_visible(False)
    plt.gca().spines['right'].set_visible(False)
    # shade under the curve from 0 to 160 seconds
    plt.fill_between(true_t, true_altitude/1000, 0, where=(true_t < 188), color='lightblue', alpha=0.5)
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
    plt.figure(figsize=(10,10))
    plt.plot(true_t, true_altitude - est_altitude)
    plt.xlabel("Time (s)")
    plt.ylabel("Altitude Error (m)")
    plt.title("Altitude Error")
    plt.grid()
    plt.savefig(run_path + "altitude_error.pdf")
    plt.close()

    # velocity vs. time
    plt.figure(figsize=(10,10))
    plt.plot(true_t, true_vx, label="vx")
    plt.plot(true_t, true_vy, label="vy")
    plt.plot(true_t, true_vz, label="vz")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")
    plt.title("Velocity")
    plt.legend()
    plt.grid()
    plt.savefig(run_path + "velocity.pdf")
    plt.close()

    # velocity error
    plt.figure(figsize=(10,10))
    plt.plot(true_t, true_vx - est_vx, label="vx")
    plt.plot(true_t, true_vy - est_vy, label="vy")
    plt.plot(true_t, true_vz - est_vz, label="vz")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity Error (m/s)")
    plt.title("Velocity Error")
    plt.legend()
    plt.grid()
    plt.savefig(run_path + "velocity_error.pdf")
    plt.close()

    # thrust vs. time
    plt.figure(figsize=(10,10))
    plt.plot(true_t, true_thrust_mag)
    plt.xlabel("Time (s)")
    plt.ylabel("Thrust Acceleration (m/s^2)")
    plt.title("Thrust")
    plt.grid()
    plt.savefig(run_path + "thrust.pdf")
    plt.close()

    # mass vs. time
    plt.figure(figsize=(10,10))
    plt.plot(true_t, true_mass)
    plt.xlabel("Time (s)")
    plt.ylabel("Mass (kg)")
    plt.title("Mass")
    plt.grid()
    plt.savefig(run_path + "mass.pdf")
    plt.close()

    # acceleration vs. time
    plt.figure(figsize=(10,10))
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
    plt.figure(figsize=(10,10))
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

    # grav acceleration error
    plt.figure(figsize=(10,10))
    plt.plot(true_t, true_ax_grav - est_ax_grav, label="ax")
    plt.plot(true_t, true_ay_grav - est_ay_grav, label="ay")
    plt.plot(true_t, true_az_grav - est_az_grav, label="az")
    plt.xlabel("Time (s)")
    plt.ylabel("Gravitational Acceleration Error (m/s^2)")
    plt.title("Gravitational Acceleration Error")
    plt.legend()
    plt.grid()
    plt.savefig(run_path + "grav_acceleration_error.pdf")
    plt.close()

    # drag acceleration error
    plt.figure(figsize=(10,10))
    plt.plot(true_t, true_ax_drag - est_ax_drag, label="ax")
    plt.plot(true_t, true_ay_drag - est_ay_drag, label="ay")
    plt.plot(true_t, true_az_drag - est_az_drag, label="az")
    plt.xlabel("Time (s)")
    plt.ylabel("Drag Acceleration Error (m/s^2)")
    plt.title("Drag Acceleration Error")
    plt.legend()
    plt.grid()
    plt.savefig(run_path + "drag_acceleration_error.pdf")
    plt.close()

    # lift acceleration
    plt.figure(figsize=(10,10))
    plt.plot(true_t, true_ax_lift, label="ax")
    plt.plot(true_t, true_ay_lift, label="ay")
    plt.plot(true_t, true_az_lift, label="az")
    plt.xlabel("Time (s)")
    plt.ylabel("Lift Acceleration (m/s^2)")
    plt.title("Lift Acceleration")
    plt.legend()
    plt.grid()
    plt.savefig(run_path + "lift_acceleration.pdf")
    plt.close()
    
    # lift acceleration error
    plt.figure(figsize=(10,10))
    plt.plot(true_t, true_ax_lift - est_ax_lift, label="ax")
    plt.plot(true_t, true_ay_lift - est_ay_lift, label="ay")
    plt.plot(true_t, true_az_lift - est_az_lift, label="az")
    plt.xlabel("Time (s)")
    plt.ylabel("Lift Acceleration Error (m/s^2)")
    plt.title("Lift Acceleration Error")
    plt.legend()
    plt.grid()
    plt.savefig(run_path + "lift_acceleration_error.pdf")
    plt.close()

    # thrust acceleration error
    plt.figure(figsize=(10,10))
    plt.plot(true_t, true_ax_thrust - est_ax_thrust, label="ax")
    plt.plot(true_t, true_ay_thrust - est_ay_thrust, label="ay")
    plt.plot(true_t, true_az_thrust - est_az_thrust, label="az")
    plt.xlabel("Time (s)")
    plt.ylabel("Thrust Acceleration Error (m/s^2)")
    plt.title("Thrust Acceleration Error")
    plt.legend()
    plt.grid()
    plt.savefig(run_path + "thrust_acceleration_error.pdf")
    plt.close()

