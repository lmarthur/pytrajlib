import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import scienceplots

plt.style.use(['science'])

def sens_plot(x, sens_data, run_params):
    """
    Function to plot the sensitivity data.

    INPUTS:
    ----------
        x: numpy.ndarray
            The x-axis data.
        sens_data: pandas.DataFrame
            The sensitivity data.
        run_params: runparams
            The run parameters.
    """
    name = str(run_params.run_name, 'utf-8')
    print('Plotting sensitivity data for ' + name + '...')
    num_runs = run_params.num_runs

    # get the values into numpy arrays
    length = len(x)

    cep_pos = sens_data['cep'][0:length].values
    cep_vel = sens_data['cep'][length:2*length].values
    cep_ang = sens_data['cep'][2*length:3*length].values
    cep_acc = sens_data['cep'][3*length:4*length].values
    cep_gyrob = sens_data['cep'][4*length:5*length].values
    cep_gyron = sens_data['cep'][5*length:6*length].values
    if run_params.gnss_nav:
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
       'text.usetex': True,
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
    if run_params.gnss_nav:
        plt.errorbar(x, cep_gnssn, cep_gnssn/np.sqrt(num_runs), label='GNSS Noise')
    plt.errorbar(x, cep_total, cep_total/np.sqrt(num_runs), label='Total')

    plt.yscale('log')
    # Add categorical xticks
    plt.xscale('log')
    # plt.xticks(x, ['0.1', '1', '10'])

    plt.xlabel('Estimated Parameter (E)')
    plt.ylabel('CEP (m)')
    plt.title('CEP Contribution From Error Parameters')

    # legend with shadow and no frame
    plt.legend()
    plt.tight_layout()
    plt.savefig('./output/' + name + '/sensitivity_plot.pdf')
    plt.close()
