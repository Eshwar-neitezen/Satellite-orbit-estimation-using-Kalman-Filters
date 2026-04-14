#This file gives the true orbit of the satellite by adding noise to the simulated orbit.

import numpy as np



def measurement(state):
    px, py, pz = state[:3]

    r = np.sqrt(px**2 + py**2 + pz**2)
    az = np.arctan2(py, px)
    el = np.arctan2(pz, np.sqrt(px**2 + py**2))

    noise = np.random.normal(0, [10000, 0.01, 0.01])

    return np.array([r, az, el]) + noise

