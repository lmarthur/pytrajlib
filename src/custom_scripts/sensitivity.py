# This script contains code to generate sensitivity plots from the output sensitivity_data.csv files.

import matplotlib.pyplot as plt
import sys
import os
import pandas as pd
import scienceplots

plt.style.use(['science'])
plt.style.use(['no-latex'])
# plt.style.use(['grayscale'])
sys.path.append('.')
import numpy as np
from pylib import *

# import the run_0 sensitivity data with pandas
sens_data = pd.read_csv('./output/run_0/sensitivity_data.csv')

# import the run_0 configuration file
run_0_params = read_config('run_0')

# generate the x-axis values
x = np.logspace(-1, 1, 7)

name = str(run_0_params.run_name, 'utf-8')
print('Plotting sensitivity data for ' + name + '...')
num_runs = run_0_params.num_runs

# get the values into numpy arrays
length = len(x)

cep_pos = sens_data['cep'][0:length].values
cep_vel = sens_data['cep'][length:2*length].values
cep_ang = sens_data['cep'][2*length:3*length].values
cep_acc = sens_data['cep'][3*length:4*length].values
cep_gyrob = sens_data['cep'][4*length:5*length].values
cep_gyron = sens_data['cep'][5*length:6*length].values
if run_0_params.gnss_nav:
    cep_gnssn = sens_data['cep'][6*length:7*length].values
    cep_total = sens_data['cep'][7*length:8*length].values
else:
    cep_total = sens_data['cep'][6*length:7*length].values

plt.figure(figsize=(5,5))
ax = plt.gca()

params = {
    'axes.labelsize': 8,
    'font.size': 8,
    'font.family': 'serif',
    'legend.fontsize': 8,
    'xtick.labelsize': 10,
    'ytick.labelsize': 10,
    #'text.usetex': True,
}

plt.rcParams.update(params)
# set color palette
colors = plt.cm.viridis(np.linspace(0, 1, 7))


plt.errorbar(x, cep_pos, cep_pos/np.sqrt(num_runs), label='Initial Position')
plt.errorbar(x, cep_vel, cep_vel/np.sqrt(num_runs), label='Initial Velocity')
plt.errorbar(x, cep_ang, cep_ang/np.sqrt(num_runs), label='Initial Angle')
plt.errorbar(x, cep_acc, cep_acc/np.sqrt(num_runs), label='Acc Scale')
plt.errorbar(x, cep_gyrob, cep_gyrob/np.sqrt(num_runs), label='Gyro Bias')
plt.errorbar(x, cep_gyron, cep_gyron/np.sqrt(num_runs), label='Gyro Noise')
if run_0_params.gnss_nav:
    plt.errorbar(x, cep_gnssn, cep_gnssn/np.sqrt(num_runs), label='GNSS Noise')
plt.errorbar(x, cep_total, cep_total/np.sqrt(num_runs), label='Total')

plt.yscale('log')
# Add categorical xticks
plt.xscale('log')
# plt.xticks(x, ['0.1', '1', '10'])
# Add y ticks
plt.yticks([5e1, 1e2, 5e2], ['$5 \\times 10^1$', '$10^2$', '$5 \\times 10^2$'])

plt.xlabel('Estimated Parameter (E)')
plt.ylabel('CEP (m)')
plt.title('Ballistic RV Error Sensitivity')

# add plot annotations with arrows
plt.annotate('Initial Position', (x[-3], cep_pos[-3]), xytext=(20,-12), textcoords = "offset points", arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=-.2', color='black'))
plt.annotate('Initial Vel.', (x[-1], cep_vel[-1]), xytext=(-60,-25), textcoords = "offset points", arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=-.2', color='black'))
plt.annotate('Initial Angle', (x[-1], cep_ang[-1]), xytext=(-60, -10), textcoords = "offset points", arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=-.2', color='black'))
plt.annotate('Acc. Scale', (x[-2], cep_acc[-2]), xytext=(-75, 0), textcoords = "offset points", arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=-.2', color='black'))
plt.annotate('Gyro Bias', (x[-3], cep_gyrob[-3]), xytext=(15, 10), textcoords = "offset points", arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=-.2', color='black'))
plt.annotate('Gyro Noise', (x[-1], cep_gyron[-1]), xytext=(-35,20), textcoords = "offset points", arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=-.2', color='black'))
plt.annotate('Total', (x[-2], cep_total[-2]), textcoords="offset points", xytext=(-50,20), arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=-.2', color='black'))

plt.tight_layout()
plt.savefig('./output/' + name + '/sensitivity_plot.pdf')
plt.close()

# Repeat for run_2
sens_data = pd.read_csv('./output/run_2/sensitivity_data.csv')

run_2_params = read_config('run_2')

# generate the x-axis values
x = np.logspace(-1, 1, 7)

name = str(run_2_params.run_name, 'utf-8')
print('Plotting sensitivity data for ' + name + '...')
num_runs = run_2_params.num_runs

# get the values into numpy arrays
length = len(x)

cep_pos = sens_data['cep'][0:length].values
cep_vel = sens_data['cep'][length:2*length].values
cep_ang = sens_data['cep'][2*length:3*length].values
cep_acc = sens_data['cep'][3*length:4*length].values
cep_gyrob = sens_data['cep'][4*length:5*length].values
cep_gyron = sens_data['cep'][5*length:6*length].values
if run_2_params.gnss_nav:
    cep_gnssn = sens_data['cep'][6*length:7*length].values
    cep_total = sens_data['cep'][7*length:8*length].values
else:
    cep_total = sens_data['cep'][6*length:7*length].values

plt.figure(figsize=(5,5))
ax = plt.gca()

params = {
    'axes.labelsize': 8,
    'font.size': 8,
    'font.family': 'serif',
    'legend.fontsize': 8,
    'xtick.labelsize': 10,
    'ytick.labelsize': 10,
    #'text.usetex': True,
}

plt.rcParams.update(params)
# set color palette
colors = plt.cm.viridis(np.linspace(0, 1, 7))

plt.errorbar(x, cep_pos, cep_pos/np.sqrt(num_runs), label='Initial Position')
plt.errorbar(x, cep_vel, cep_vel/np.sqrt(num_runs), label='Initial Velocity')
plt.errorbar(x, cep_ang, cep_ang/np.sqrt(num_runs), label='Initial Angle')
plt.errorbar(x, cep_acc, cep_acc/np.sqrt(num_runs), label='Acc Scale')
plt.errorbar(x, cep_gyrob, cep_gyrob/np.sqrt(num_runs), label='Gyro Bias')
plt.errorbar(x, cep_gyron, cep_gyron/np.sqrt(num_runs), label='Gyro Noise')
if run_2_params.gnss_nav:
    plt.errorbar(x, cep_gnssn, cep_gnssn/np.sqrt(num_runs), label='GNSS Noise')
plt.errorbar(x, cep_total, cep_total/np.sqrt(num_runs), label='Total')

plt.yscale('log')
# Add categorical xticks
plt.xscale('log')
# plt.xticks(x, ['0.1', '1', '10'])

plt.xlabel('Estimated Parameter (E)')
plt.ylabel('CEP (m)')
plt.title('Maneuverable RV: INS-Only Error Sensitivity')

# add plot annotations with arrows
plt.annotate('Initial Position', (x[-3], cep_pos[-3]), xytext=(20,-12), textcoords = "offset points", arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=-.2', color='black'))
plt.annotate('Initial Vel.', (x[-2], cep_vel[-2]), xytext=(18,-20), textcoords = "offset points", arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=-.2', color='black'))
plt.annotate('Initial Angle', (x[-2], cep_ang[-2]), xytext=(12, -10), textcoords = "offset points", arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=-.2', color='black'))
plt.annotate('Acc. Scale', (x[-3], cep_acc[-3]), xytext=(-75, 0), textcoords = "offset points", arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=-.2', color='black'))
plt.annotate('Gyro Bias', (x[-3], cep_gyrob[-3]), xytext=(-20, -25), textcoords = "offset points", arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=-.2', color='black'))
plt.annotate('Gyro Noise', (x[-2], cep_gyron[-2]), xytext=(10,-25), textcoords = "offset points", arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=-.2', color='black'))
if run_2_params.gnss_nav:
    plt.annotate('GNSS Noise', (x[-2], cep_gnssn[-2]), xytext=(-10, -25), textcoords = "offset points", arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=-.2', color='black'))
plt.annotate('Total', (x[2], cep_total[2]), textcoords="offset points", xytext=(-50,20), arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=-.2', color='black'))

plt.tight_layout()
plt.savefig('./output/' + name + '/sensitivity_plot.pdf')
plt.close()

# Repeat for run_3
sens_data = pd.read_csv('./output/run_3/sensitivity_data.csv')

run_3_params = read_config('run_3')

# generate the x-axis values
x = np.logspace(-1, 1, 7)

name = str(run_3_params.run_name, 'utf-8')
print('Plotting sensitivity data for ' + name + '...')
num_runs = run_3_params.num_runs

# get the values into numpy arrays
length = len(x)

cep_pos = sens_data['cep'][0:length].values
cep_vel = sens_data['cep'][length:2*length].values
cep_ang = sens_data['cep'][2*length:3*length].values
cep_acc = sens_data['cep'][3*length:4*length].values
cep_gyrob = sens_data['cep'][4*length:5*length].values
cep_gyron = sens_data['cep'][5*length:6*length].values
if run_3_params.gnss_nav:
    cep_gnssn = sens_data['cep'][6*length:7*length].values
    cep_total = sens_data['cep'][7*length:8*length].values
else:
    cep_total = sens_data['cep'][6*length:7*length].values

plt.figure(figsize=(5,5))
ax = plt.gca()

params = {
    'axes.labelsize': 8,
    'font.size': 8,
    'font.family': 'serif',
    'legend.fontsize': 8,
    'xtick.labelsize': 10,
    'ytick.labelsize': 10,
    #'text.usetex': True,
}

plt.rcParams.update(params)

# set color palette
colors = plt.cm.viridis(np.linspace(0, 1, 7))

plt.errorbar(x, cep_pos, cep_pos/np.sqrt(num_runs), label='Initial Position')
plt.errorbar(x, cep_vel, cep_vel/np.sqrt(num_runs), label='Initial Velocity')
plt.errorbar(x, cep_ang, cep_ang/np.sqrt(num_runs), label='Initial Angle')
plt.errorbar(x, cep_acc, cep_acc/np.sqrt(num_runs), label='Acc Scale')
plt.errorbar(x, cep_gyrob, cep_gyrob/np.sqrt(num_runs), label='Gyro Bias')
plt.errorbar(x, cep_gyron, cep_gyron/np.sqrt(num_runs), label='Gyro Noise')
if run_3_params.gnss_nav:
    plt.errorbar(x, cep_gnssn, cep_gnssn/np.sqrt(num_runs), label='GNSS Noise')
plt.errorbar(x, cep_total, cep_total/np.sqrt(num_runs), label='Total')

plt.yscale('log')
# Add categorical xticks
plt.xscale('log')
# plt.xticks(x, ['0.1', '1', '10'])

plt.xlabel('Estimated Parameter (E)')
plt.ylabel('CEP (m)')
plt.title('Maneuverable RV: INS+GNSS Error Sensitivity')

# add plot annotations with arrows
plt.annotate('Initial Position', (x[-3], cep_pos[-3]), xytext=(-40,20), textcoords = "offset points", arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=-.2', color='black'))
plt.annotate('Initial Vel.', (x[-2], cep_vel[-2]), xytext=(18,-20), textcoords = "offset points", arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=-.2', color='black'))
plt.annotate('Initial Angle', (x[2], cep_ang[2]), xytext=(-20, 20), textcoords = "offset points", arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=-.2', color='black'))
plt.annotate('Acc. Scale', (x[-3], cep_acc[-3]), xytext=(-30, -20), textcoords = "offset points", arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=-.2', color='black'))
plt.annotate('Gyro Bias', (x[-2], cep_gyrob[-2]), xytext=(15, 20), textcoords = "offset points", arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=-.2', color='black'))
plt.annotate('Gyro Noise', (x[3], cep_gyron[3]), xytext=(10,-25), textcoords = "offset points", arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=-.2', color='black'))
if run_3_params.gnss_nav:
    plt.annotate('GNSS Error', (x[2], cep_gnssn[2]), xytext=(-10, -25), textcoords = "offset points", arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=-.2', color='black'))
plt.annotate('Total', (x[3], cep_total[3]), textcoords="offset points", xytext=(-50,20), arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=-.2', color='black'))

plt.tight_layout()
plt.savefig('./output/' + name + '/sensitivity_plot.pdf')
plt.close()
