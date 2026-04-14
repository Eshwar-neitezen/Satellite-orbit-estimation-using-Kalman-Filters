import matplotlib.pyplot as plt
from simulator import simulate_orbit
from ekf import rk, hx as hx_ekf, H_jacobian, fx as fx_ekf, F_jacobian
from ukf import uk, hx as hx_ukf, fx as fx_ukf
from sensor import measurement
import numpy as np


t, state = simulate_orbit()

#Generate noise for both EKF and UKF
measurements = []
for i in range(len(t)):
    z = measurement(state[:, i])  # Get the noisy measurement
    measurements.append(z)
measurements = np.array(measurements)  # Convert to numpy array

#EKF results
ekf_estimates = []

dt = t[1] - t[0]  # Time step

for i in range(len(t)):

    rk.F = F_jacobian(rk.x, dt)  # Update the state transition Jacobian
    rk.x = fx_ekf(rk.x, dt = dt)  # Predict the next state
    rk.P = rk.F @ rk.P @ rk.F.T + rk.Q  # Predict the next covariance

    z = measurements[i]  # Get the noisy measurement

    rk.update(z, H_jacobian, hx_ekf)  # EKF update step

    ekf_estimates.append(rk.x.copy())  # Store the EKF estimate

ekf_estimates = np.array(ekf_estimates)  # Convert to numpy array a


#UKF results
ukf_estimates = []

dt = t[1] - t[0]  # Time step

for i in range(len(t)):
    uk.predict(dt = dt)  # UKF prediction step

    z = measurements[i]  # Get the noisy measurement

    uk.update(z)  # UKF update step

    ukf_estimates.append(uk.x.copy())  # Store the UKF estimate
ukf_estimates = np.array(ukf_estimates)  # Convert to numpy array

print("EKF shape:", ekf_estimates.shape)
print("UKF shape:", ukf_estimates.shape)
print("EKF first 5:", ekf_estimates[:5, :2])

plt.figure()

# TRUE orbit (make it bold + dashed + black)
plt.plot(state[0], state[1], 'k--', linewidth=2, label="True")

# EKF
plt.plot(ekf_estimates[:,0], ekf_estimates[:,1], 'b', label="EKF")

# UKF
plt.plot(ukf_estimates[:,0], ukf_estimates[:,1], 'g', label="UKF")

plt.legend()
plt.axis("equal")
plt.title("Orbit Estimation: EKF vs UKF")
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.show()

ekf_error = np.linalg.norm(state[:3].T - ekf_estimates[:, :3], axis=1)
ukf_error = np.linalg.norm(state[:3].T - ukf_estimates[:, :3], axis=1)

plt.plot(ekf_error, label="EKF Error")
plt.plot(ukf_error, label="UKF Error")
plt.legend()
plt.title("Error Comparison")
plt.show()
plt.figure()

plt.plot(state[0], state[1], 'k--', label="True")
plt.plot(ekf_estimates[:,0], ekf_estimates[:,1], 'r', label="EKF")

plt.xlim(6.99e6, 7.01e6)
plt.ylim(-2000, 2000)

plt.legend()
plt.title("Extreme Zoom: EKF vs True")
plt.show()

plt.figure()

plt.plot(state[0], state[1], 'k--', label="True")
plt.plot(state[0] + (ekf_estimates[:,0] - state[0]),
         state[1] + (ekf_estimates[:,1] - state[1]),
         'r', label="EKF deviation")

plt.legend()
plt.axis("equal")
plt.show()