import numpy as np
from filterpy.kalman import UnscentedKalmanFilter
from dynamics import dynamics
from filterpy.kalman import MerweScaledSigmaPoints
from filterpy.common import Q_discrete_white_noise



def fx(x, dt):
    x = np.array(x)  # Ensure x is a numpy array
    # Propagate the state using the dynamics function
    k1 = dynamics(0, x)
    k2 = dynamics(0, x + 0.5 * dt * k1)
    k3 = dynamics(0, x + 0.5 * dt * k2)
    k4 = dynamics(0, x + dt * k3)
    
    return x + (dt / 6) * (k1 + 2*k2 + 2*k3 + k4)

def hx(x):
    px, py, pz = x[0], x[1], x[2]

    r = np.sqrt(px**2 + py**2 + pz**2)
    az = np.arctan2(py, px)
    el = np.arctan2(pz, np.sqrt(px**2 + py**2))

    return np.array([r, az, el])

points = MerweScaledSigmaPoints(n=6, alpha=0.1, beta=2., kappa=0)

uk = UnscentedKalmanFilter(dim_x=6, dim_z=3, fx=fx, hx=hx, dt=60, points=points)


uk.x = np.array([7000e3, 0, 0, 0, 7500, 0])  # Initial state
uk.P = np.diag([500**2, 500**2, 500**2, 10**2, 10**2, 10**2])# Initial covariance(large uncertainty in position, smaller in velocity)


uk.R = np.eye(3)*(50.**2)  # Measurement noise covariance
uk.Q = np.eye(6)*(0.1**2)   # Process noise covariance


