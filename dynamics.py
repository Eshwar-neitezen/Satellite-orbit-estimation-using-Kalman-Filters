import numpy as np

mu = 3.986e14

def dynamics(t, state):
    x, y, z, vx, vy, vz = state

    r = np.sqrt(x**2 + y**2 + z**2)

    ax = -mu * x / r**3 + 1e-5
    ay = -mu * y / r**3 + 1e-5
    az = -mu * z / r**3 + 1e-5

    return np.array([vx, vy, vz, ax, ay, az])