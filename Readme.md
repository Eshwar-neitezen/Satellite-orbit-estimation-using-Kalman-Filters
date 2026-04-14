# Satellite Orbit Estimation using Kalman Filters

A comprehensive implementation of satellite orbit estimation using **Extended Kalman Filter (EKF)** and **Unscented Kalman Filter (UKF)** for accurate state estimation under nonlinear dynamics and noisy measurements.

## 📋 Table of Contents

- [Overview](#overview)
- [Key Features](#key-features)
- [Installation](#installation)
- [Project Structure](#project-structure)
- [Quick Start](#quick-start)
- [Usage Examples](#usage-examples)
- [Theory](#theory)
- [Results & Performance](#results--performance)
- [Configuration](#configuration)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)

---

## Overview

This project demonstrates advanced nonlinear estimation techniques for tracking satellite orbits in real-time. It simulates a realistic scenario where a satellite's state (position and velocity) must be estimated from noisy sensor measurements using two different Kalman filter approaches.

### What Problem Does This Solve?

In real-world satellite applications, you rarely have perfect measurements. Sensors introduce noise, and orbital dynamics are nonlinear. This project shows:

- How to model satellite dynamics accurately
- When EKF is sufficient and when UKF is necessary
- How different noise levels affect estimation accuracy
- Practical implementation of advanced filtering algorithms

---

## Key Features

✅ **6D State Simulation**
- Full position (x, y, z) and velocity (ẋ, ẏ, ż) tracking

✅ **Accurate Orbit Propagation**
- RK4 (4th-order Runge-Kutta) numerical integration
- Two-body gravitational dynamics
- Customizable orbital elements (semi-major axis, eccentricity, inclination, etc.)

✅ **Dual Filter Implementation**
- Extended Kalman Filter (EKF) - computationally efficient
- Unscented Kalman Filter (UKF) - handles strong nonlinearities better

✅ **Flexible Measurement Models**
- Linear measurements (Cartesian position)
- Nonlinear measurements (range, azimuth, elevation)
- Configurable sensor noise levels

✅ **Comprehensive Analysis**
- Real-time estimation error tracking
- Performance comparison (EKF vs UKF)
- Publication-quality visualization plots

✅ **Built on Established Libraries**
- FilterPy for Kalman filter implementations
- NumPy for numerical computations
- Matplotlib for visualization

---

## Installation

### Prerequisites

- **Python 3.8 or higher**
- pip (Python package manager)
- Git

### Step 1: Clone the Repository

```bash
git clone https://github.com/Eshwar-neitezen/satellite-orbit-estimation-kalman-filters.git
cd satellite-orbit-estimation-kalman-filters
```

### Step 2: Create a Virtual Environment (Recommended)

```bash
# On Windows
python -m venv venv
venv\Scripts\activate

# On macOS/Linux
python3 -m venv venv
source venv/bin/activate
```

### Step 3: Install Dependencies

```bash
pip install -r requirements.txt
```

#### If `requirements.txt` doesn't exist, install manually:

```bash
pip install numpy matplotlib filterpy scipy
```

### Step 4: Verify Installation

```bash
python -c "import numpy, matplotlib, filterpy; print('✓ All dependencies installed successfully!')"
```

---

## Project Structure

```
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
---

## Quick Start

### Run a Simple Example (30 seconds)

```bash
cd examples
python basic_ekf_estimation.py
```

This will:
1. Simulate a satellite orbit
2. Generate noisy measurements
3. Run the Extended Kalman Filter
4. Display estimation error

---

## Theory

### Two-Body Orbital Dynamics

The fundamental equation of motion under gravitational attraction:

$$\ddot{\mathbf{r}} = -\frac{\mu}{|\mathbf{r}|^3}\mathbf{r}$$

Where:
- **μ** = Earth's gravitational parameter (3.986 × 10¹⁴ m³/s²)
- **r** = Position vector from Earth's center

### State Representation

State vector: **x** = [x, y, z, ẋ, ẏ, ż]ᵀ

6D state with:
- Position: (x, y, z) in ECEF coordinates
- Velocity: (ẋ, ẏ, ż) in ECEF coordinates

### Extended Kalman Filter (EKF)

Best for: Weakly nonlinear systems, real-time applications

**Prediction:**
- **x̂⁻** = f(x̂⁺, u)
- **P⁻** = F·P⁺·Fᵀ + Q

**Update:**
- **y** = z - h(x̂⁻)
- **K** = P⁻·Hᵀ·(H·P⁻·Hᵀ + R)⁻¹
- **x̂⁺** = x̂⁻ + K·y
- **P⁺** = (I - K·H)·P⁻

Where:
- **F** = Jacobian of f (state transition)
- **H** = Jacobian of h (measurement model)

### Unscented Kalman Filter (UKF)

Best for: Highly nonlinear systems, improved accuracy at higher computational cost

Uses deterministic sigma points instead of Jacobians:
- Avoids linearization errors
- Handles strong nonlinearities better
- More computationally expensive than EKF

---

## Results & Performance

### Typical Performance Metrics

| Metric | EKF | UKF | Advantage |
|--------|-----|-----|-----------|
| **Position Error (low noise)** | ~5-10 m | ~5-10 m | Comparable |
| **Position Error (high noise)** | ~50-100 m | ~30-50 m | **UKF** |
| **Velocity Error** | ~0.1-0.5 m/s | ~0.05-0.2 m/s | **UKF** |
| **Computational Time** | ~1 ms/step | ~5 ms/step | **EKF** |

### When to Use Each Filter

**Use EKF when:**
- ✓ Computational resources are limited
- ✓ System dynamics are weakly nonlinear
- ✓ Measurement noise is low
- ✓ Real-time performance is critical

**Use UKF when:**
- ✓ Measurement model is highly nonlinear
- ✓ You can afford 3-5x more computation
- ✓ Estimation accuracy is critical
- ✓ Process noise is significant

---

## Configuration

### Key Parameters to Tune

#### Filter Parameters

```python
# Process noise (how much we expect the true state to deviate from the model)
Q = np.diag([1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-8])  # Position and velocity variance

# Measurement noise (sensor accuracy)
R = np.diag([10.0, 10.0, 10.0])  # Position measurement noise (m²)

# Initial state uncertainty
P0 = np.diag([1e6, 1e6, 1e6, 100, 100, 100])  # Large initial uncertainty
```

#### UKF-Specific Parameters

```python
alpha = 1e-3      # Controls sigma point spread (1e-4 to 1)
beta = 2.0        # Optimal for Gaussian distributions
kappa = 0.0       # Usually 0 for state estimation
```

#### Orbital Elements

```python
a = 6800e3        # Semi-major axis (meters)
e = 0.001         # Eccentricity (0 = circular, ~1 = highly elliptical)
i = 45.0          # Inclination (degrees, 0 = equatorial, 90 = polar)
raan = 0.0        # Right ascension of ascending node
aop = 0.0         # Argument of perigee
nu = 0.0          # True anomaly (0 = periapsis, 180 = apoapsis)
```

### Tuning Guide

1. **Start with default values** - Most provided configs are well-tuned
2. **Increase Q if filter lags behind actual changes** - Trusts model less
3. **Increase R if filter oscillates** - Trusts measurements less
4. **Adjust alpha in UKF** - Smaller values (1e-4) stay closer to mean; larger values spread sigma points
5. **Compare EKF and UKF** - If they agree, EKF is sufficient; if they differ, nonlinearity matters

---

## Troubleshooting

### Issue: Filter diverges or estimates become increasingly wrong

**Solution:** 
- Increase measurement noise (R) - current measurement may be too trusted
- Check that noise parameters match your actual sensor specs
- Verify orbital mechanics implementation is correct

### Issue: Filter is sluggish and lags behind actual motion

**Solution:**
- Decrease process noise (Q) - increase trust in the model
- Verify that time step (dt) is appropriate
- Check orbital dynamics is being propagated correctly

### Issue: UKF is much slower than expected

**Solution:**
- Consider reducing simulation steps for testing
- Profile the code to find bottleneck
- For real-time applications, consider EKF as alternative

### Issue: Matplotlib plots not displaying

**Solution:**
```bash
# Add to your Python script
import matplotlib
matplotlib.use('TkAgg')  # Or 'Qt5Agg', 'Agg' for non-interactive
```

### Issue: Import errors when running examples

**Solution:**
```bash
# Make sure you're in the project directory
cd satellite-orbit-estimation-kalman-filters

# Add project to Python path
export PYTHONPATH="${PYTHONPATH}:$(pwd)"  # macOS/Linux
set PYTHONPATH=%PYTHONPATH%;%cd%         # Windows

# Then run examples
python examples/basic_ekf_estimation.py
```

### Development Setup

```bash
git clone https://github.com/Eshwar-neitezen/satellite-orbit-estimation-kalman-filters.git
cd satellite-orbit-estimation-kalman-filters
python -m venv venv
source venv/bin/activate  # or venv\Scripts\activate on Windows
pip install -r requirements.txt
pip install pytest pytest-cov  # For testing
```

---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## References and Further Reading

### Kalman Filtering
- Welch, G., & Bishop, G. (2006). *An Introduction to the Kalman Filter*
- Labbe, R. (2020). *Kalman and Bayesian Filters in Python* :contentReference[oaicite:0]{index=0}
- Bar-Shalom, Y., Li, X. R., & Kirubarajan, T. (2001). *Estimation with Applications to Tracking and Navigation*

### Orbit Determination and Estimation
- Manarvi, A., & Henderson, T. (2013). *Application of Kalman Filters in Orbit Determination: A Literature Survey* :contentReference[oaicite:1]{index=1}  
- Ipek, M. (2017). *Satellite Orbit Estimation Using Kalman Filters (M.S. Thesis)* :contentReference[oaicite:2]{index=2}  

### Practical EKF Applications (Space Systems)
- Tirmal, N., et al. (ISRO). *Extended Kalman Filter Based Onboard Orbit Determination Using GNSS Receiver for LEO and GEO Satellites* :contentReference[oaicite:3]{index=3}  

### Orbital Mechanics
- Curtis, H. D. (2013). *Orbital Mechanics for Engineering Students (3rd ed.)*
- Vallado, D. A., et al. (2006). *Revisiting Spacetrack Report #3*

### Unscented Kalman Filter and Nonlinear Estimation
- Julier, S. J., & Uhlmann, J. K. (2004). *Unscented Filtering and Nonlinear Estimation*
- Särkkä, S. (2013). *Bayesian Filtering and Smoothing*
---

---

## Acknowledgments

- FilterPy library maintainers for excellent Kalman filter implementations
- NumPy and SciPy communities
- Orbital mechanics references and open-source satellite tracking community

---

**Last Updated:** April 2026  
**Version:** 1.0.0
