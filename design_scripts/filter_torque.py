


""" Plots user interaction with the virtual wall.
The virtual wall is situated at x = -0.05 in the xy plane. Data in the 
produced plots show how force commands and motor currents are activated when 
the end effector position goes through the virtual wall.

Takes data from the latest file in the data/ folder.

Antoine Henri, 2024-02-27
"""

import scipy.signal as sg
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

import pathlib 

if __name__ == "__main__":
    sampling_frequency = 800

    f_c = 100
    
    plant = sg.bessel(N=6, Wn=f_c, fs=sampling_frequency)
    print(plant)

    num = plant[0]
    den = plant[1]

    w, h = sg.freqz(num, den, fs=sampling_frequency)

    data_path = pathlib.Path("data")
    data_files = list(pathlib.Path.glob(data_path, "hybrid_*"))
    
    df_panto = pd.read_csv(data_files[-1], sep=", ")

    t = np.array(df_panto["t"])
    current_m = np.array(df_panto["current_motor"])
    current_b = np.array(df_panto["current_brake"])
    torque = np.array(df_panto["torque"])
    theta = np.array(df_panto["theta"])
    omega = np.array(df_panto["omega"])
    cmd_voltage_m = np.array(df_panto["cmd_voltage_motor"])
    cmd_voltage_b = np.array(df_panto["cmd_voltage_brake"])
    cmd_theta = np.array(df_panto['theta_cmd'])

    sim_filt_torque = sg.lfilter(num, den, torque)

    fig2, axes = plt.subplots(4, 2)

    axes[0][0].plot(t, current_m, 'r')
    axes[1][0].plot(t, torque, 'g')
    axes[1][0].plot(t, sim_filt_torque, 'orange')
    axes[2][0].plot(t, theta, 'b')
    axes[2][0].plot(t, cmd_theta, 'orange')
    axes[3][0].plot(t, cmd_voltage_m, 'y')

    axes[0][0].set_ylabel('Motor Current (A)')
    axes[1][0].set_ylabel('Torque (N)')
    axes[2][0].set_ylabel(r'$\theta$ (rad)')
    axes[3][0].set_ylabel(r'Motor Voltage Cmd(V)')

    axes[0][0].set_xlabel('Time (sec)')
    axes[1][0].set_xlabel('Time (sec)')
    axes[2][0].set_xlabel('Time (sec)')
    axes[3][0].set_xlabel('Time (sec)')

    axes[0][1].plot(t, current_b, 'r')
    axes[1][1].plot(t, torque, 'g')
    axes[2][1].plot(t, omega, 'b')
    
    axes[3][1].plot(t, cmd_voltage_b, 'y')

    axes[0][1].set_ylabel('Brake Current (A)')
    axes[1][1].set_ylabel('Torque (N)')
    axes[2][1].set_ylabel(r'$\omega$ (rad/s)')
    axes[3][1].set_ylabel(r'Brake Voltage Cmd(V)')

    axes[0][1].set_xlabel('Time (sec)')
    axes[1][1].set_xlabel('Time (sec)')
    axes[2][1].set_xlabel('Time (sec)')
    axes[3][1].set_xlabel('Time (sec)')

    for ax in axes:
        for a in ax:
            a.grid()
    fig2.legend()

    plt.show()