import numpy as np
from filterpy.kalman import ExtendedKalmanFilter
from dynamics import dynamics

dim_x = 6  # State vector: [x, y, z, vx, vy, vz]
dim_z = 3  # Measurement vector: [x, y, z]


rk = ExtendedKalmanFilter(dim_x=dim_x, dim_z=dim_z)

rk.x = np.array([7000e3, 0, 0, 0, 7500, 0])  # Initial state
rk.P = np.diag([500**2, 500**2, 500**2, 10**2, 10**2, 10**2])# Initial covariance(large uncertainty in position, smaller in velocity)

#Noise matrices(sigma = 50)

rk.R = np.eye(3)*(50.**2)  # Measurement noise covariance
rk.Q = np.eye(6)*(0.1**2)   # Process noise covariance


def hx(x):
    px, py, pz = x[0], x[1], x[2]

    r = np.sqrt(px**2 + py**2 + pz**2)
    az = np.arctan2(py, px)
    el = np.arctan2(pz, np.sqrt(px**2 + py**2))

    return np.array([r, az, el])

print(hx(rk.x))

#defining the jacobian matrix which only depennds on meaurement state and not on velocity
def H_jacobian(x):
    n = len(x)
    m = 3  # measurement dimension

    H = np.zeros((m, n))
    eps = 1e-5

    for i in range(n):
        dx = np.zeros(n)
        dx[i] = eps

        h1 = hx(x + dx)
        h2 = hx(x - dx)

        H[:, i] = (h1 - h2) / (2 * eps)

    return H
  
def fx(x, dt):
    x = np.array(x)  # Ensure x is a numpy array
    # Propagate the state using the dynamics function
    k1 = dynamics(0, x)
    k2 = dynamics(0, x + 0.5 * dt * k1)
    k3 = dynamics(0, x + 0.5 * dt * k2)
    k4 = dynamics(0, x + dt * k3)
    
    return x + (dt / 6) * (k1 + 2*k2 + 2*k3 + k4)

def F_jacobian(x, dt):
    n = len(x)
    F= np.zeros((n, n))
    eps = 1e-5
    for i in range(n):
      dx = np.zeros(n)
      dx[i] = eps

      xplus = x + dx
      xminus = x - dx

      f1 = fx(xplus, dt)
      f2 = fx(xminus, dt)

      F[:, i] = (f1 - f2) / (2 * eps)
      
    return F
      
