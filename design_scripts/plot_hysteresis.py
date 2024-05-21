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

from matplotlib.collections import LineCollection
from matplotlib.colors import BoundaryNorm, ListedColormap

if __name__ == "__main__":

    data_path = pathlib.Path("data")
    data_files = list(pathlib.Path.glob(data_path, "hybrid_*"))
    
    df_panto = pd.read_csv(data_files[-1], sep=", ")

    t = np.array(df_panto["t"])
    # current_m = np.array(df_panto["current_motor"])
    # current_b = np.array(df_panto["current_brake"])
    torque = np.array(df_panto["torque"])
    omega = np.array(df_panto["omega"])
    # cmd_voltage_m = np.array(df_panto["cmd_voltage_motor"])
    # cmd_voltage_b = np.array(df_panto["cmd_voltage_brake"])

    # fig2, axes = plt.subplots(1, 1)

    # axes.plot(omega, torque, 'r')


    # axes.set_ylabel(r'Torque (N)')


    # axes.set_xlabel(r'$\omega$ (rad/s)')





    # axes.grid()
    # fig2.legend()

    # plt.show()

    # Create a set of line segments so that we can color them individually
    # This creates the points as an N x 1 x 2 array so that we can stack points
    # together easily to get the segments. The segments array for line collection
    # needs to be (numlines) x (points per line) x 2 (for x and y)
    points = np.array([omega, torque]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)

    fig, axs = plt.subplots(1, 1)

    # Create a continuous norm to map from data points to colors
    norm = plt.Normalize(t.min(), t.max())
    lc = LineCollection(segments, cmap='viridis', norm=norm)
    # Set the values used for colormapping
    lc.set_array(t)
    lc.set_linewidth(2)
    line = axs.add_collection(lc)
    fig.colorbar(line, ax=axs)


    axs.set_xlim(omega.min()*1.1, omega.max()*1.1)
    axs.set_ylim(torque.min()*1.1, torque.max()*1.1)
    plt.show()



    



    