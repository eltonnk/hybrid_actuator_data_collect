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

    data_path = pathlib.Path("data")
    data_files = list(pathlib.Path.glob(data_path, "hybrid_*"))
    
    df_panto = pd.read_csv(data_files[-1], sep=", ")

    t = np.array(df_panto["t"])
    current = np.array(df_panto["current_motor"])
    torque = np.array(df_panto["torque"])
    omega = np.array(df_panto["omega"])

    fig2, axes = plt.subplots(3, 1)

    axes[0].plot(t, current, 'r')
    axes[1].plot(t, torque, 'g')
    axes[2].plot(t, omega, 'b')

    axes[0].set_ylabel('Current (A)')
    axes[1].set_ylabel('Torque (N)')
    axes[2].set_ylabel(r'$\omega$ (rad/s)')

    axes[0].set_xlabel('Time (sec)')
    axes[1].set_xlabel('Time (sec)')
    axes[2].set_xlabel('Time (sec)')

    for ax in axes:
        ax.grid()
    fig2.legend()

    plt.show()