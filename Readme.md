Satellite Orbit Estimation using Kalman Filters
Overview

This project implements satellite orbit estimation using Extended Kalman Filter (EKF) and Unscented Kalman Filter (UKF).
A satellite orbit is simulated using two-body orbital mechanics, and the state is estimated from noisy measurements.

The project demonstrates how nonlinear estimation techniques perform under different conditions and compares EKF and UKF accuracy.

Key Concepts
Two-body orbital dynamics
Numerical integration using Runge-Kutta (RK4)
Nonlinear state estimation
Extended Kalman Filter (EKF)
Unscented Kalman Filter (UKF)
Measurement modeling (linear and nonlinear)
Features
6D state simulation (position and velocity)
RK4-based orbit propagation
Noisy sensor measurements
EKF and UKF implementations using FilterPy
Error analysis and comparison plots
Project Structure
satellite-orbit-estimation/
│
├── main.py
├── dynamics.py
├── simulator.py
├── ekf.py
├── ukf.py
├── sensor.py
├── requirements.txt
└── README.md
Results
EKF performs well under near-linear and low-noise conditions
UKF shows improved robustness under nonlinear measurements and higher noise
Error plots are used to evaluate estimation performance
Example Output
Orbit Estimation

True orbit compared with EKF and UKF estimates

Error Analysis

Position estimation error over time and comparison between EKF and UKF

(Add your plot images here)

Installation

Clone the repository:

git clone https://github.com/Eshwar-neitezen/satellite-orbit-estimation-kalman-filters.git
cd satellite-orbit-estimation-kalman-filters

Install dependencies:

pip install -r requirements.txt
Run the Project
python main.py
Insights
EKF approximates nonlinear systems using linearization
UKF uses sigma points to better capture nonlinear transformations
In ideal conditions, EKF performs comparably to UKF
Under nonlinear and noisy conditions, UKF provides better accuracy
Future Improvements
Integration with real satellite data (TLE and SGP4)
Incorporating perturbations such as J2 and atmospheric drag
Ground station tracking using azimuth and elevation
3D visualization of orbit and estimation
Technologies Used
Python
NumPy
Matplotlib
FilterPy