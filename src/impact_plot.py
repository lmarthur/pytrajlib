# This script contains code to generate scatter plots and histograms of the impact data.
import scipy.stats as stats
import os
import matplotlib.pyplot as plt
import sys
sys.path.append('.')
import numpy as np
from src.pylib import *

# TODO: Add a calculation of the range to the aimpoint

def impact_plot(run_path, run_params):

    # print error if the paths are not found
    if not os.path.exists(run_path + "impact_data.txt"):
        print("Error: impact data not found.")
        sys.exit()

    print("Reading impact data...")
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    # get longitude and latitude of aimpoint
    aimpoint_lon = np.arctan2(run_params.y_aim, run_params.x_aim)
    aimpoint_lat = np.arctan2(run_params.z_aim, np.sqrt(run_params.x_aim**2 + run_params.y_aim**2))

    # Calculate the range to the aimpoint over the surface of the Earth
    # This is the great circle distance between the aimpoint and the origin
    range_to_aimpoint = np.arccos(np.sin(aimpoint_lat)*np.sin(0) + np.cos(aimpoint_lat)*np.cos(0)*np.cos(aimpoint_lon))
    range_to_aimpoint = range_to_aimpoint * 6371e3
    print('Range to aimpoint: ', range_to_aimpoint)

    impact_t = impact_data[:,0]
    impact_x = impact_data[:,1]
    impact_y = impact_data[:,2]
    impact_z = impact_data[:,3]

    # get vector relative to aimpoint
    impact_x = impact_x - run_params.x_aim
    impact_y = impact_y - run_params.y_aim
    impact_z = impact_z - run_params.z_aim

    # convert impact data to local tangent plane coordinates
    impact_x_local = -np.sin(aimpoint_lon)*impact_x + np.cos(aimpoint_lon)*impact_y
    impact_y_local = -np.sin(aimpoint_lat)*np.cos(aimpoint_lon)*impact_x - np.sin(aimpoint_lat)*np.sin(aimpoint_lon)*impact_y + np.cos(aimpoint_lat)*impact_z

    # get the miss distances
    miss_distance = np.sqrt(impact_x_local**2 + impact_y_local**2)
    cep = np.percentile(miss_distance, 50)
    print('CEP: ', cep)
    cep = round(np.percentile(miss_distance, 50), 2)
    plotrange = 4*cep

    # Plot the data
    params = {
        'axes.labelsize': 8,
        'font.size': 8,
        'font.family': 'serif',
        'legend.fontsize': 10,
        'xtick.labelsize': 10,
        'ytick.labelsize': 10,
    }
    plt.rcParams.update(params)

    fig = plt.figure(figsize=(5,5))
    # plot a circle of radius CEP m centered on (0,0)
    N = 400
    t = np.linspace(0, 2 * np.pi, N)
    x, y = cep * np.cos(t), cep * np.sin(t)

    # gridspec
    gs = fig.add_gridspec(2, 1, height_ratios=(6, 1), hspace=0.18, bottom=0.1, top=0.95, left=0.025, right=0.975)
    
    a0 = fig.add_subplot(gs[0, 0])
    a1 = fig.add_subplot(gs[1, 0])

    
    a0.scatter(impact_x_local, impact_y_local, c='grey', marker='x', label='Impact Points', s=20, alpha=0.5, linewidths=1)
    a0.plot(x, y, c='k', label='CEP', linestyle='--', linewidth=1.5)
    a0.legend(['Impact Points', 'CEP'], frameon=False, framealpha=0)

    # center the plot on (0,0)
    a0.set_xlim(-plotrange, plotrange)
    a0.set_ylim(-plotrange, plotrange)
    a0.set_aspect('equal')

    # add N=len(guided_r) to the top left of the plot
    a0.text(-0.6*plotrange, 0.8*plotrange, 'N = ' + str(len(miss_distance)) + '\nCEP = ' + str(cep) + 'm', fontsize=10, verticalalignment='top', horizontalalignment='center')

    # add label to a0
    a0.set_xlabel('Downrange (m)', labelpad=-1)
    a0.set_ylabel('Crossrange (m)', labelpad=-1)
    # For a0 axis (if you have operations on it that require adjustments)
    a0.tick_params(axis='x', which='major', pad=1)  # Adjust pad for x-axis ticks
    a0.tick_params(axis='y', which='major', pad=1)  # Adjust pad for y-axis ticks

    if str(run_params.run_name, 'utf-8') == 'run_0':
        a0.set_title('Minuteman III: Ballistic RV, INS-Only')
    elif str(run_params.run_name, 'utf-8') == 'run_2':
        a0.set_title('Minuteman III: Perfectly Maneuverable RV, INS-Only')
    elif str(run_params.run_name, 'utf-8') == 'run_3':
        a0.set_title('Minuteman III: Perfectly Maneuverable RV, INS+GNSS')
    elif str(run_params.run_name, 'utf-8') == 'run_4':
        a0.set_title('Minuteman III: Maneuverable RV, INS+GNSS')
    elif str(run_params.run_name, 'utf-8') != 'test':
        print('Warning: run_name ' + str(run_params.run_name, 'utf-8') + ' not recognized')

        a0.set_title('Trajectory Impact Points')
    # plot the histogram of the miss distances

    cep = round(np.percentile(miss_distance, 50), 2)

    # Fit a Nakagami distribution to the data
    x = np.linspace(0, 5*cep, 100)
    shape, loc, scale = stats.nakagami.fit(miss_distance, floc=0)
    nakagamipdf = stats.nakagami.pdf(x, shape, loc, scale)
    print('Nakagami fit: shape =', shape, 'loc =', loc, 'scale =', scale)
    
    # Compute number of bins for the histogram
    bins = 50
    # plot histogram up to 5 times the CEP, with no y axis
    a1.hist(miss_distance, bins=bins, range=(0, 5*cep), color='grey', edgecolor='black', alpha=0.7, histtype='stepfilled')
    # renormalize the pdfs to the histogram
    nakagamipdf = nakagamipdf * len(miss_distance) * 5*cep / bins
    # evaluate the pdf at the CEP
    pdf_cep = nakagamipdf[np.argmin(np.abs(x - cep))]
    # Add a vertical line at the CEP, to the top of the histogram at that point
    plotmax = a1.get_ylim()[1]
    a1.axvline(x=cep, ymax=pdf_cep/plotmax, color='k', linestyle='--', linewidth=1.5, label='CEP')

    a1.plot(x, nakagamipdf, 'k', linewidth=1.5, label = 'Nakagami(' + str(round(shape, 2)) + ', ' + str(round(scale, 2)) + ')')
    
    # omit the frame
    a1.spines['top'].set_visible(False)
    a1.spines['right'].set_visible(False)
    a1.spines['left'].set_visible(False)
    a1.spines['bottom'].set_visible(False)

    a1.yaxis.set_visible(False)
    # Legend with no border or box around it, and with the nakagami fit parameters
    a1.legend(frameon=False, framealpha=0)
    a1.tick_params(axis='x', which='major', pad=1)  # Adjust pad for x-axis ticks
    a1.tick_params(axis='y', which='major', pad=1)  # Adjust pad for y-axis ticks (if y-axis is used)
    a1.set_xlabel('Miss Distance Histogram (m)', labelpad=1)  # Adjust labelpad for x-axis label
    plt.savefig(run_path + "impact_plot.jpg", dpi=1000)
    plt.savefig(run_path + "impact_plot.pdf")
    plt.close()